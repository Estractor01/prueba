#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "soc/clk_tree_defs.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ADC_channel_V  ADC_CHANNEL_0
#define ADC_channel_I  ADC_CHANNEL_3

typedef struct{
    bool A;
    bool B;
    bool C;
    bool D;
}state;
typedef struct {
    bool S;
    bool L;
    bool F;
    bool N;
    bool E;
    bool P;
}rules;
typedef struct{
    int V;          //Velocidad x1
    float I;        //Corriente x2
}reading;           //X

static const char *TAG = "Main";

static adc_oneshot_unit_handle_t adc_handle;

typedef enum {LED_PROT = 4, LED_STATUS = 5, LED1 = 12, LED2 = 13, LED3 = 14}Leds;
typedef enum {BTN1 = 18}BTN;
static Leds led_array[5] = {LED_STATUS, LED1, LED2, LED3, LED_PROT};
static Leds *ptr_leds = led_array;
static volatile uint16_t _state = false;
static volatile uint16_t lastInterruptTime = 0;

static void IRAM_ATTR btn_isr_handler(void* arg)
{
    //Debounce de boton
    uint16_t InterruptTime = xTaskGetTickCountFromISR();
    if (InterruptTime - lastInterruptTime > 45/portTICK_PERIOD_MS){
        _state = !_state;
    }
    lastInterruptTime = InterruptTime;
}

static void init_btn_isr()
{
    gpio_set_intr_type(BTN1, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN1, btn_isr_handler, nullptr);
    gpio_intr_enable(BTN1);
}

static void init_leds()
{
    gpio_reset_pin(BTN1);
    gpio_set_direction(BTN1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN1, GPIO_PULLUP_ONLY);
    for(uint16_t i=0; i < 5; i++){
        gpio_reset_pin(*(ptr_leds + i));
        gpio_set_direction(*(ptr_leds + i), GPIO_MODE_OUTPUT);
        gpio_set_level(*(ptr_leds + i), false);
    }
}

static void init_ADC()
{
    adc_oneshot_unit_init_cfg_t adcconfig_t = {0};
    adcconfig_t.unit_id = ADC_UNIT_1;
    adcconfig_t.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    adcconfig_t.ulp_mode = ADC_ULP_MODE_DISABLE;
    
    adc_oneshot_new_unit(&adcconfig_t, &adc_handle);

    adc_oneshot_chan_cfg_t adc_channelconfig_t = {0};
    adc_channelconfig_t.atten = ADC_ATTEN_DB_12;
    adc_channelconfig_t.bitwidth = ADC_BITWIDTH_12;

    adc_oneshot_config_channel(adc_handle, ADC_channel_I, &adc_channelconfig_t);
    adc_oneshot_config_channel(adc_handle, ADC_channel_V, &adc_channelconfig_t);
}

static reading ADC_Lectures()
{
    int I, V;
    reading trans;
    adc_oneshot_read(adc_handle, ADC_channel_I, &I); 
    adc_oneshot_read(adc_handle, ADC_channel_V, &V);
    //El adc maneja una resolución de 12 bits que es igual a valores de 0-4095
    //Su lectura se tiene que transformar para estar dentro de un rango util para
    //las conduciones de nuestro sistema.
    //Se va a manejar rangos ficticios para las pruebas de:
    // 0 - 3 A para la corriente 
    // 0 - 2000 RPM para la velocidad
    trans.I = ((float)I * 3)/4095;
    trans.V = (V * 2000)/4095;

    printf("Lectura base I: %d, lectura transformada: %f\n",I,trans.I);
    printf("Lectura base V: %d, lectura transformada: %d\n",V,trans.V);
    return trans; 
}

static state base_logic(int V, float I)
{
    state b_logic;
    
    // Estructura de booleanos para manejar los estados

    b_logic.A = (I > 2.0);  //Si la corriente es mayor a 2(alta), es verdadero
    b_logic.B = (I < 0.5);  //Si la corriente es menor a 0.5(baja), es verdadero
    b_logic.C = (V < 500);  //Si la velocidad es menor a 500 rpm(baja), es verdadero
    b_logic.D = (V > 1500);  //Si la velocidad es mayor a 1500 rpm(alta), es verdadero
    return b_logic;
}

static rules set_rules(state ABCD)
{
    rules r;
    
    //Reglas a base de las condiciones lógicas pasadas

    r.S = (ABCD.A && ABCD.C);                      //Sobrecarga
    r.L = (r.S && ABCD.C);                         //Bloqueo
    r.F = (ABCD.B && ABCD.C);                      //Falla eléctrica
    r.N = (ABCD.A || ABCD.B || ABCD.C || ABCD.D);  //Condición normal
    r.E = (r.S || r.L || r.F);                     //Condición anormal
    r.P = (r.L || r.F);                            //Protección
    return r;
}

static void write_l(rules r)
{
    if(r.N == true){
     ESP_LOGI(TAG, "\t\tFuncionamiento correcto del motor");
     gpio_set_level(LED1, false);
     gpio_set_level(LED2, false);
     gpio_set_level(LED3, false);
     gpio_set_level(LED_PROT, false);
    }
    if(r.E == true){
        ESP_LOGI(TAG, "\t\tAlerta! Condicion anormal:");
        gpio_set_level(LED1, true);
        if(r.S == true){
            ESP_LOGI(TAG, "Sobrecarga ");
            gpio_set_level(LED2, true);
            gpio_set_level(LED3, false);
        }
        if(r.L == true){
            ESP_LOGI(TAG, "Bloqueo ");
            gpio_set_level(LED2, false);
            gpio_set_level(LED3, true);
        }
        if(r.F == true){
            ESP_LOGI(TAG, "Falla electrica ");
            gpio_set_level(LED2, true);
            gpio_set_level(LED3, true);
        }
    }

    if(r.P == true){
        ESP_LOGI(TAG, "\t\tActivando proteccion del motor");
        gpio_set_level(LED_PROT, true);
    }

    if(!r.N && !r.E && !r.P == true){
        ESP_LOGI(TAG, "\t\tCondición fuera de rango");
        gpio_set_level(LED1,true);
        gpio_set_level(LED2,true);
        gpio_set_level(LED3,true);
        gpio_set_level(LED_PROT,true);

        vTaskDelay(pdMS_TO_TICKS(200));

        gpio_set_level(LED1,false);
        gpio_set_level(LED2,false);
        gpio_set_level(LED3,false);
        gpio_set_level(LED_PROT,false);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    init_leds();
    init_btn_isr();
    init_ADC();
    while(true){
        if(_state == true){
            printf("\033[H\033[J");
            gpio_set_level(LED_STATUS, true);
            reading l = ADC_Lectures();
            state s = base_logic(l.V,l.I);
            rules r = set_rules(s);
            write_l(r);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if(_state == false){
            gpio_set_level(LED1, false);
            gpio_set_level(LED2, false);
            gpio_set_level(LED3, false);
            gpio_set_level(LED_PROT, false);
            gpio_set_level(LED_STATUS, false);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }   
}

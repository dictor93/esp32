#include <stdio.h>
#include "esp_types.h"
#include "esp_err.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"

#include <math.h>

#include "esp_dsp.h"
#include "fft.h"

#define WIDTH    16
#define HEIGHT   16
#define NUM_LEDS 256

#include "ws2812_control.h"

#define RED   0x00FF00
#define GREEN 0xFF0000
#define BLUE  0x0000FF
#define WHITE 0xFFFFFF
#define BLACK 0x000000
#define ORANGE 0xffff00


#define TIMER_DIVIDER         2  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define ADC_TIMER_INTERVAL    (0.000003125)   // sample test interval for the second timer
#define ADC_TIMER             TIMER_0        // testing will be done with auto reload
#define RENDER_TIMER_INTERVAL (1.0/30)
#define RENDER_TIMER          TIMER_1

#define DEFAULT_VREF    5000        //Use adc2_vref_to_gpio() to obtain a better estimate
#define SAMPLES         1024
#define ADC_ACCURACY    2

#define SMOOTH 0.3

struct led_state new_state;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
float wind[SAMPLES];
int posOffset[17] = {(int)(SAMPLES/64/2), (int)(SAMPLES/39.38/2), (int)(SAMPLES/30.12/2), (int)(SAMPLES/20.48/2), (int)(SAMPLES/15.51/2), (int)(SAMPLES/12.19/2), (int)(SAMPLES/10.24/2), (int)(SAMPLES/8.83/2), (int)(SAMPLES/7.64), (int)(SAMPLES/6.17/2), (int)(SAMPLES/4.92/2), (int)(SAMPLES/4.1/2), (int)(SAMPLES/3.51/2), (int)(SAMPLES/2.1/2), (int)(SAMPLES/1.54/2), (int)(SAMPLES/1.25/2), (int)(SAMPLES/2)};
float posLevel_old[16];

static void check_efuse()
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }

}
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;
xQueueHandle render_queue;


float arrToPrint[16];

void printToLed(float *arrToPrint) {
    // int datA[16] = {16, 8,9,3,7,8,6,1,5,6,3,4,2,7,5,6};
    for (int y = 0; y < HEIGHT; y++) {
      for (int x = 0; x < WIDTH; x++)
      {
        new_state.leds[WIDTH*y+x] = 0;
      }
    }
    for (int y = 0; y < HEIGHT; y++) {
      int max = arrToPrint[y];
      int middle = (int)max/2;
      for (int x = 0; x < max; x++)
      {
        int color = x < middle ? BLUE : ( x == max-1 ? RED : GREEN );
        if(y%2) {
          new_state.leds[WIDTH*y+x] = color;
        } else {
          new_state.leds[WIDTH*y+WIDTH - 1 - x] = color;
        }
      }
    }
    printf("NUM_LEDS: %d", NUM_LEDS);
    ws2812_write_leds(new_state);
}

void convert_fft(float *data_array) {
    
    // printf("\e[1;1H\e[2J");
    
    fft_config_t *real_fft_plan = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, NULL, NULL);

    for (int i=0 ; i< SAMPLES ; i++)
    {
        real_fft_plan->input[i] = data_array[i] * wind[i];
    }
    fft_execute(real_fft_plan);
    float max = real_fft_plan->output[0];
    for (int i = 0 ; i < SAMPLES/2 ; i++) {
        if(real_fft_plan->output[i] < 0) real_fft_plan->output[i] = -real_fft_plan->output[i];
        if(real_fft_plan->output[i] > max) {
            max = real_fft_plan->output[i];
        }
    }
    float divider = max/HEIGHT;
    for (int pos = 0; pos < 16; pos++)
    {
        int posLevel = real_fft_plan->output[posOffset[pos]];
        int linesBetween;
        if (pos > 0 && pos < 16) {
            linesBetween = posOffset[pos] - posOffset[pos - 1];
            for (int i = 0; i < linesBetween; i++) {  // от предыдущей полосы до текущей
                posLevel += (float) ((float)i / linesBetween) * real_fft_plan->output[posOffset[pos] - linesBetween + i];
            }
        }

        // найти максимум из пачки тонов
        //      if (posLevel > maxValue) maxValue = posLevel;

        // фильтрация длины столбиков, для их плавного движения
        arrToPrint[pos] = (posLevel * SMOOTH + posLevel_old[pos] * (1 - SMOOTH))/divider;
        // olprintf("attToPrint: %f, pos: %i\n", arrToPrint[pos], pos);
        posLevel_old[pos] = arrToPrint[pos];
    }
    // printf("attToPrint: %f\n", arrToPrint[8]);
    fft_destroy(real_fft_plan);
    printToLed(arrToPrint);

    
}

int samplesRecorded = 0;
uint16_t adc_reading[SAMPLES];
uint16_t adc_copy[SAMPLES];

void IRAM_ATTR timer_isr(void *para)
{
    int timer_idx = (int) para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;

    if ((intr_status & BIT(timer_idx)) && timer_idx == ADC_TIMER) {
        timer_counter_value += (uint64_t) (ADC_TIMER_INTERVAL * TIMER_SCALE);
        evt.type = ADC_TIMER;
        TIMERG0.int_clr_timers.t0 = 1;
        if (samplesRecorded < SAMPLES) {
            uint32_t currentAdc = 0;
            for (int adcCount = 0; adcCount < ADC_ACCURACY; adcCount++)
            {
                currentAdc += (uint16_t)adc1_get_raw((adc1_channel_t)channel);
            }
            
            adc_reading[samplesRecorded] = (int16_t)(currentAdc / ADC_ACCURACY);
            samplesRecorded++;
        } else {
            timer_pause(evt.timer_group, evt.timer_idx);
            samplesRecorded = 0;
            for (int i = 0; i < SAMPLES; i++)
            {
                adc_copy[i] = adc_reading[i];
            }
            
            xQueueSendFromISR(timer_queue, &evt, NULL);
        }
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == RENDER_TIMER) {
        timer_counter_value += (uint64_t) (RENDER_TIMER_INTERVAL * TIMER_SCALE);
        evt.type = RENDER_TIMER;
        TIMERG0.int_clr_timers.t1 = 1;
        xQueueSendFromISR(render_queue, &evt, NULL);
    } else {
        evt.type = -1; // not supported even type
    }
    TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
    TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

static void example_tg0_timer_init(int timer_idx, double timer_interval_sec)
{
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 0;
    timer_init(TIMER_GROUP_0, timer_idx, &config);
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
    printf("TIMER INDEX: %f. \n", timer_interval_sec);
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

float floatArray[SAMPLES];
static void timer_example_evt_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        
        if (evt.type == ADC_TIMER) {
            // printf("ADC_TIMER RetUrN\n");
            timer_start(evt.timer_group, evt.timer_idx);
        } else {
            printf("\n    UNKNOWN EVENT TYPE\n");
        }
    }
}

static void render_task(void *arg){
     while (1) {
        timer_event_t evt;
        xQueueReceive(render_queue, &evt, portMAX_DELAY);
        
       if (evt.type == RENDER_TIMER) {
            timer_start(evt.timer_group, ADC_TIMER);
            for (int i = 0; i < SAMPLES; i++){
                floatArray[i] = (float)adc_copy[i]/4095;
            }
            // printToLed();
            convert_fft(floatArray);
            // printf("RENDER\n");
        } else {
            printf("\n    UNKNOWN EVENT TYPE\n");
        }
    }
}

void app_main()
{
    check_efuse();

    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // check FFT
    esp_err_t ret;
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret  != ESP_OK)
    {
        ESP_LOGE("main", "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    dsps_wind_hann_f32(wind, SAMPLES);


    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    ws2812_control_init();
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    render_queue = xQueueCreate(10, sizeof(timer_event_t));
    // example_tg0_timer_init(ADC_TIMER,    ADC_TIMER_INTERVAL);
    example_tg0_timer_init(RENDER_TIMER, RENDER_TIMER_INTERVAL);
    // xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreatePinnedToCore(render_task, "render_event_task", 4096, NULL, 1, NULL, 1);
}

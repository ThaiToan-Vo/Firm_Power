#include <stdio.h>
#include <math.h>
#include "esp_task_wdt.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define FRAME_SAMPLES   50
#define FRAME_BYTES     (1 + FRAME_SAMPLES * 2)
#define AVG_FRAMES      10

#define ADC_MAX   4095.0f
#define VREF      3.3f
#define VAC_RMS     220.0f
#define VAC_PEAK    (VAC_RMS * 1.41421356f)   // 311 V
#define ADC_VPEAK   3.25f                     // đo trên oscilloscope
#define VAC_PER_ADC_V   (VAC_PEAK / ADC_VPEAK)   // ≈ 95.69


 #define V_OFFSET  1.6f

#define I_ADC_RMS_NOISE  0.228f   // RMS ADC khi không tải (đã đo)
#define I_SCALE         28.0f


// #define V_GAIN    (820.0f / 5.6f)

// #define I_OFFSET  1.6f
// #define I_GAIN    (1000.0f / 51.0f)


static spi_device_handle_t spi_v;
static spi_device_handle_t spi_i;

static void spi_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .max_transfer_sz = FRAME_BYTES,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 100000,
        .mode = 0,
        .queue_size = 2,
    };

    devcfg.spics_io_num = 15; // STM voltage
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi_v);

    devcfg.spics_io_num = 14; // STM current
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi_i);
}

void Ex_ISR_Init(void){
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << 13,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);

    gpio_set_level(13, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(13, 1);
}

static inline float adc_to_vin(uint16_t adc)
{
    return (adc * VREF) / ADC_MAX;
}

static bool spi_read_frame(spi_device_handle_t dev, uint16_t *out)
{
    static uint8_t tx[FRAME_BYTES] = {0};
    static uint8_t rx[FRAME_BYTES];

    spi_transaction_t t = {
        .length = FRAME_BYTES * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(dev, &t);

    if (rx[0] != 1) return false;

    for (int i = 0; i < FRAME_SAMPLES; i++) {
        out[i] = rx[1 + 2*i] | (rx[2 + 2*i] << 8);
    }
    return true;
}

typedef struct {
    float vrms;
} frame_v_t;

float last_mean;
static frame_v_t process_v_frame(uint16_t *v_buf)
{
    frame_v_t r;
    float mean = 0.0f;
    float sum  = 0.0f;

    /* 1. ADC -> Vin, tính mean */
    for (int i = 0; i < FRAME_SAMPLES; i++) {
        float vin = (v_buf[i] * VREF) / ADC_MAX;
        mean += vin;
    }
    mean /= FRAME_SAMPLES;
    last_mean = mean;

    /* 2. Trừ mean, tính RMS tại ADC */
    for (int i = 0; i < FRAME_SAMPLES; i++) {
        float vin = (v_buf[i] * VREF) / ADC_MAX;
        float v_ac = vin - mean;
        sum += v_ac * v_ac;
    }

    float vin_rms = sqrtf(sum / FRAME_SAMPLES);

    /* 3. Scale RMS */
    r.vrms = vin_rms * 190.0f;  
    // VAC_PER_ADC_V = 220 / vin_rms_đo_chuẩn

    return r;
}

typedef struct {
    float irms;
    float mean;
    float rms_adc;
} frame_i_t;

static frame_i_t process_i_frame(uint16_t *i_buf)
{
    frame_i_t r;
    float mean = 0.0f;
    float sum  = 0.0f;

    for (int i = 0; i < FRAME_SAMPLES; i++) {
        mean += adc_to_vin(i_buf[i]);
    }
    mean /= FRAME_SAMPLES;

    for (int i = 0; i < FRAME_SAMPLES; i++) {
        float i_ac = adc_to_vin(i_buf[i]) - mean;
        sum += i_ac * i_ac;
    }

    float rms_adc = sqrtf(sum / FRAME_SAMPLES);

    /* Trừ noise RMS đúng bản chất */
    float rms_eff = 0.0f;
    if (rms_adc > I_ADC_RMS_NOISE) {
        rms_eff = sqrtf(
            rms_adc * rms_adc -
            I_ADC_RMS_NOISE * I_ADC_RMS_NOISE
        );
    }

    r.irms    = rms_eff * I_SCALE;
    r.mean    = mean;
    r.rms_adc = rms_adc;

    return r;
}

typedef struct {
    float p;      // công suất tác dụng (W)
} frame_p_t;

static frame_p_t process_p_frame(uint16_t *v_buf, uint16_t *i_buf)
{
    frame_p_t r;
    float mean_v = 0.0f;
    float mean_i = 0.0f;
    float sum_p  = 0.0f;

    /* 1. Tính mean */
    for (int n = 0; n < FRAME_SAMPLES; n++) {
        mean_v += adc_to_vin(v_buf[n]);
        mean_i += adc_to_vin(i_buf[n]);
    }
    mean_v /= FRAME_SAMPLES;
    mean_i /= FRAME_SAMPLES;

    /* 2. Nhân mẫu tức thời và trung bình */
    for (int n = 0; n < FRAME_SAMPLES; n++) {
        float v_ac = adc_to_vin(v_buf[n]) - mean_v;
        float i_ac = adc_to_vin(i_buf[n]) - mean_i;
        sum_p += v_ac * i_ac;
    }

    /* 3. Scale sang Watt */
    r.p = (sum_p / FRAME_SAMPLES) * (190.0f) * (I_SCALE);

    return r;
}


void app_main(void)
{
    Ex_ISR_Init();
    //vTaskDelay(pdMS_TO_TICKS(200));
    spi_bus_init();
 
    uint16_t v_buf[FRAME_SAMPLES];
    uint16_t i_buf[FRAME_SAMPLES];

    float v_acc = 0;
    int   v_cnt   = 0;
    float i_acc = 0.0f;
    int   i_cnt = 0;
    bool v_ready = false;
    bool i_ready = false;

    float p_acc = 0.0f;
    int   p_cnt = 0;

    while (1)
{
        /* ===== READ VOLTAGE ===== */
    if (spi_read_frame(spi_v, v_buf)) {
        frame_v_t v = process_v_frame(v_buf);
        
        v_acc += v.vrms;
        v_cnt++;
        v_ready = true;
        if (v_cnt >= AVG_FRAMES) {
            printf("Vrms = %.2f V \n ",
                   v_acc / v_cnt);
            v_acc = 0.0f;
            v_cnt = 0;
        }
    }
    
    /* delay frame-level */
    vTaskDelay(pdMS_TO_TICKS(50));
    /* ===== READ CURRENT ===== */
    if (spi_read_frame(spi_i, i_buf)) {
        frame_i_t i = process_i_frame(i_buf);
        
        i_acc += i.irms;
        i_cnt++;
        i_ready = true;
        if (i_cnt >= AVG_FRAMES) {
            printf("Irms = %.2f A\n", i_acc / i_cnt);
            i_acc = 0.0f;
            i_cnt = 0;
        }
    }
    
    /* delay nhỏ để nhả SPI + STM */
    vTaskDelay(pdMS_TO_TICKS(50));



    if (v_ready && i_ready) {
    frame_p_t p = process_p_frame(v_buf, i_buf);

    p_acc += p.p;
    p_cnt++;

    if (p_cnt >= AVG_FRAMES) {
        printf("P = %.2f W\n", p_acc / p_cnt);
        p_acc = 0.0f;
        p_cnt = 0;
    }

    v_ready = false;
    i_ready = false;
}

}

}


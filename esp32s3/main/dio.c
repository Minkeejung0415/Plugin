/* Digital I/O — rising-edge counter and timestamper on PIN_DIO_IN.
   State packed into a uint8_t bitmask for the UDP channel.             */
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "include/config.h"

static const char *TAG = "dio";

static volatile uint32_t s_edge_count = 0;
static volatile int64_t  s_last_edge_us = 0;

static void IRAM_ATTR dio_isr(void *arg)
{
    s_edge_count++;
    s_last_edge_us = esp_timer_get_time();
}

esp_err_t dio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_DIO_IN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_DIO_IN, dio_isr, NULL));

    /* DIO output for sync pulse (master) */
    gpio_set_direction(PIN_DIO_OUT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DIO_OUT, 0);

    ESP_LOGI(TAG, "DIO init: in=GPIO%d out=GPIO%d", PIN_DIO_IN, PIN_DIO_OUT);
    return ESP_OK;
}

/* Return current DIO state packed in a byte: bit0 = pin level, bits1–7 = edge count mod 128 */
uint8_t dio_read_state(void)
{
    uint8_t level = gpio_get_level(PIN_DIO_IN) & 0x01;
    uint8_t edges = (uint8_t)(s_edge_count & 0x7F);
    return (edges << 1) | level;
}

uint32_t dio_edge_count(void)  { return s_edge_count; }
int64_t  dio_last_edge_us(void){ return s_last_edge_us; }

void dio_pulse_out(uint32_t width_us)
{
    gpio_set_level(PIN_DIO_OUT, 1);
    esp_rom_delay_us(width_us);
    gpio_set_level(PIN_DIO_OUT, 0);
}

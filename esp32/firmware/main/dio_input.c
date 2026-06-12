#include "dio_input.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#define DIO_DEBOUNCE_US (15 * 1000)

static int s_gpio = -1;
static bool s_stable_high = true;
static bool s_pending_raw = true;
static int64_t s_pending_since_us;
static uint16_t s_edge_count;

void dio_input_init(int gpio_num)
{
    s_gpio = gpio_num;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    bool level = gpio_get_level(gpio_num);
    s_stable_high = level;
    s_pending_raw = level;
    s_pending_since_us = esp_timer_get_time();
    s_edge_count = 0;
}

void dio_input_update(void)
{
    if (s_gpio < 0) {
        return;
    }
    bool raw = gpio_get_level(s_gpio);
    int64_t now = esp_timer_get_time();
    if (raw != s_pending_raw) {
        s_pending_raw = raw;
        s_pending_since_us = now;
    }
    if ((now - s_pending_since_us) >= DIO_DEBOUNCE_US && s_pending_raw != s_stable_high) {
        s_stable_high = s_pending_raw;
        if (s_edge_count < 0x7FFF) {
            s_edge_count++;
        }
    }
}

int16_t dio_input_read_channel(void)
{
    if (s_gpio < 0) {
        return 0;
    }
    return (int16_t)((s_stable_high ? 1 : 0) | ((uint32_t)(s_edge_count & 0x7FFF) << 1));
}

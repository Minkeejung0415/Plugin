#include "sd_logger.h"

#include <stdio.h>

#include "esp_log.h"

static const char *TAG = "sd_logger";
static FILE *s_fp;
static bool s_enabled;

void sd_logger_init(void)
{
    /* Mount SD on Sense expansion — wire pins per Seeed wiki in production */
    s_fp = fopen("/sdcard/step_session.bin", "ab");
    s_enabled = (s_fp != NULL);
    if (!s_enabled) {
        ESP_LOGW(TAG, "SD not mounted — logging disabled");
    }
}

void sd_logger_append(const oe_sample_t *sample)
{
    if (!s_enabled || !s_fp) {
        return;
    }
    fwrite(sample, sizeof(*sample), 1, s_fp);
    fflush(s_fp);
}

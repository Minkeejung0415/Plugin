#pragma once
#include <stdint.h>
#include "esp_err.h"

/* TEVM-AR0234 (ON Semi AR0234 global-shutter 2MP sensor).
   The module exposes MIPI CSI-2; for direct DVP connection an
   AR0234 parallel-output configuration must be written over I2C at
   init time (register 0x301A bit 10 = 0 selects parallel interface).

   Technexion note:
   The Technexion TEVI-AR0234CS-S32-IR and related modules use the same
   AR0234 sensor with the S32 lens (fixed-focus, ~90° HFOV, IR-cut or
   no-IR versions).  They present the same I2C register map.  Electrical
   interface is MIPI CSI-2 (2-lane, 800 Mbps/lane).  Connecting these to
   the ESP32-S3 requires a MIPI→DVP bridge IC on the carrier board
   (e.g. TI DS90UB954 or Toshiba TC358746).  Once the bridge is
   configured the driver below is transparent to either module.          */

typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t  jpeg_quality;   /* 0=best, 63=worst */
    bool     continuous;     /* true = free-running; false = on-demand */
} cam_cfg_t;

esp_err_t camera_init(const cam_cfg_t *cfg);
esp_err_t camera_stop(void);

/* Capture one JPEG frame.  *buf is allocated from PSRAM; caller must
   free() it after use.  *len is byte count.                            */
esp_err_t camera_capture_jpeg(uint8_t **buf, size_t *len);

/* Non-blocking: returns true if a new frame is available in the DMA
   ring since the last call.  Used by action_verify to check without
   stalling the acquisition loop.                                        */
bool      camera_frame_ready(void);

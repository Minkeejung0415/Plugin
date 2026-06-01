#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── Board role ─────────────────────────────────────────────────────── */
/* Set via NVS at first boot or via provisioning command.
   MASTER: has WiFi + streams to OpenEphys/OpenSim + bridges ESPNow.
   SLAVE : no WiFi; sends IMU/DIO/cam frames to master via ESPNow.       */
typedef enum { ROLE_MASTER = 0, ROLE_SLAVE = 1 } node_role_t;

/* ── Pin assignments (Seeed XIAO ESP32-S3 Sense pinout) ────────────── */

/* ICM-20948 via SPI */
#define PIN_IMU_SCLK   12
#define PIN_IMU_MOSI   11
#define PIN_IMU_MISO   13
#define PIN_IMU_CS      9

/* SD card via SDMMC 4-bit */
#define PIN_SD_CLK     39
#define PIN_SD_CMD     38
#define PIN_SD_D0      40
#define PIN_SD_D1      41
#define PIN_SD_D2      42
#define PIN_SD_D3       2

/* Digital I/O (trigger / sync pulse in) */
#define PIN_DIO_IN      4   /* rising edge counted / timestamped */
#define PIN_DIO_OUT     5   /* sync pulse out (master only) */

/* Camera — TEVM-AR0234 via DVP (parallel) 8-bit
   AR0234 can output parallel data when configured over I2C at boot.
   NOTE: If the Technexion MIPI module is used instead, a MIPI→DVP
   bridge (e.g. TI DS90UB954) is required on the carrier board; the
   software interface is identical once the bridge is initialised.       */
#define PIN_CAM_XCLK    15
#define PIN_CAM_PCLK    16
#define PIN_CAM_VSYNC   17
#define PIN_CAM_HREF    18
#define PIN_CAM_D0       7
#define PIN_CAM_D1       8
#define PIN_CAM_D2      20
#define PIN_CAM_D3      19
#define PIN_CAM_D4      21
#define PIN_CAM_D5       3
#define PIN_CAM_D6      14
#define PIN_CAM_D7       6
/* AR0234 I2C config bus (shared with system I2C if desired) */
#define PIN_CAM_SDA     47
#define PIN_CAM_SCL     48

/* ── Acquisition parameters ─────────────────────────────────────────── */
#define DEFAULT_SAMPLE_RATE_HZ   100
#define MAX_SAMPLE_RATE_HZ      1000

/* ── Network (master only) ──────────────────────────────────────────── */
#define TCP_CTRL_PORT   5000
#define UDP_DATA_PORT  55001
#define UDP_QUAT_PORT   5005   /* quaternion stream to OpenSim; avoid
                                  colliding with TCP_CTRL_PORT 5000 */
#define MAX_OPEN_EPHYS_CLIENTS  4

/* ── ESPNow ─────────────────────────────────────────────────────────── */
#define ESPNOW_CHANNEL          1
#define ESPNOW_MAX_NODES       16
#define ESPNOW_SYNC_INTERVAL_MS 1000   /* master broadcasts sync every 1 s */

/* ── SD card ────────────────────────────────────────────────────────── */
#define SD_MOUNT_POINT  "/sdcard"
#define SD_MAX_FILES    8

/* ── Camera action verification ─────────────────────────────────────── */
/* A "verify frame" is captured when any ICM-20948 linear acceleration
   exceeds VERIFY_ACCEL_THRESH_G for VERIFY_HOLD_SAMPLES samples,
   i.e. a deliberate motion event is detected.                           */
#define VERIFY_ACCEL_THRESH_G    0.5f
#define VERIFY_HOLD_SAMPLES      5
#define VERIFY_JPEG_QUALITY     12   /* 0=best … 63=worst; 12 ≈ 160 KB */
#define VERIFY_FRAME_WIDTH     640
#define VERIFY_FRAME_HEIGHT    400

/* Quaternion channels per IMU in the UDP data packet */
#define QUAT_CHANNELS   4
#define RAW_CHANNELS_9D 9   /* ax ay az gx gy gz mx my mz */
#define CHANNELS_PER_IMU (RAW_CHANNELS_9D + QUAT_CHANNELS)

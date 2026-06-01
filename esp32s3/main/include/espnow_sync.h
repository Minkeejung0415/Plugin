#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Packet types sent over ESPNow */
typedef enum {
    ESPNOW_PKT_SYNC   = 0x01,  /* master → slaves: timestamp broadcast */
    ESPNOW_PKT_DATA   = 0x02,  /* slave  → master: IMU + DIO + cam event */
    ESPNOW_PKT_CMD    = 0x03,  /* master → slaves: start / stop / config */
    ESPNOW_PKT_ACK    = 0x04,  /* slave  → master: command acknowledgment */
} espnow_pkt_type_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  node_id;
    int64_t  master_us;     /* master µs epoch at send time */
    int64_t  slave_us;      /* slave local µs (slaves echo back for offset calc) */
} espnow_sync_pkt_t;

/* One IMU frame forwarded from a slave to the master */
typedef struct __attribute__((packed)) {
    uint8_t  type;          /* ESPNOW_PKT_DATA */
    uint8_t  node_id;
    int64_t  timestamp_us;  /* slave-local, corrected to master epoch after sync */
    int16_t  raw[13];       /* ax ay az gx gy gz mx my mz qw qx qy qz  Q15 */
    uint8_t  dio_state;     /* bitmask of 8 DIO pins */
    uint8_t  cam_event;     /* 1 = verify frame attached in next packet */
} espnow_data_pkt_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;          /* ESPNOW_PKT_CMD */
    uint8_t  node_id;       /* 0xFF = broadcast */
    uint8_t  cmd;           /* 'S'=start 'T'=stop 'F'=freq 'A'=accel_range 'G'=gyro_range */
    int32_t  param;
} espnow_cmd_pkt_t;

esp_err_t espnow_sync_init(uint8_t node_id, bool is_master);
esp_err_t espnow_sync_add_peer(const uint8_t mac[6]);

/* Master: broadcast sync pulse and receive slave data */
esp_err_t espnow_master_send_sync(void);
esp_err_t espnow_master_send_cmd(uint8_t target_id, uint8_t cmd, int32_t param);

/* Slave: send one data frame to master */
esp_err_t espnow_slave_send_data(const espnow_data_pkt_t *pkt);

/* Slave: get current time offset (master_us - slave_us), 0 until first sync */
int64_t   espnow_slave_time_offset_us(void);

/* Callback installed by main.c — called on master when slave data arrives */
typedef void (*espnow_data_cb_t)(const espnow_data_pkt_t *pkt);
void espnow_set_data_callback(espnow_data_cb_t cb);

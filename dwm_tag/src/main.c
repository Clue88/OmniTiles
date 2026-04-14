// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

/*
 * DWM tag (nRF52) application: BLE-to-SPI bridge + UWB ranging.
 *
 * Runs as a BLE peripheral (Nordic UART Service), advertising as OmniTile_1.
 * Data received from a device is queued and then sent to the STM32 over SPI when
 * the master asserts CS. A DRDY GPIO is driven high when a payload is ready and
 * cleared after the SPI transaction completes.
 *
 * A separate UWB thread performs DS-TWR ranging against 3 fixed anchors and
 * stores the distances. The BLE telemetry path appends these distances to the
 * motor telemetry before sending over NUS.
 */

#include <bluetooth/services/nus.h>
#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define DRDY_GPIO_NODE DT_ALIAS(spis_drdy_gpios)
#define SPI_BUF_SIZE   128

/* Protocol: same as STM32/omnitiles (START_BYTE, msg_id, checksum). No payload => checksum = msg_id */
#define CMD_START_BYTE 0xA5
#define CMD_M1_BRAKE   0x32
#define CMD_M2_BRAKE   0x42

// ---------------------------------------------------------------------------
// Per-tile calibration. Set with: west build -- -DCONFIG_TILE_ID=<n>
// Each tile gets its own BLE name and antenna delay, calibrated against the
// golden anchor (CONFIG_ANCHOR_ID=0, which stays at nominal 16385).
// ---------------------------------------------------------------------------
#if CONFIG_TILE_ID == 1
  #define TILE_NAME      "OmniTile_1"
  #define UWB_TX_ANT_DLY 16385
  #define UWB_RX_ANT_DLY 16385
#elif CONFIG_TILE_ID == 2
  #define TILE_NAME      "OmniTile_2"
  #define UWB_TX_ANT_DLY 16385
  #define UWB_RX_ANT_DLY 16385
#elif CONFIG_TILE_ID == 3
  #define TILE_NAME      "OmniTile_3"
  #define UWB_TX_ANT_DLY 16385
  #define UWB_RX_ANT_DLY 16385
#else
  #error "CONFIG_TILE_ID must be 1, 2, or 3"
#endif

// ---------------------------------------------------------------------------
// UWB ranging constants — based on Qorvo SS-TWR examples (ex_06a/ex_06b)
// ---------------------------------------------------------------------------
#define NUM_ANCHORS 3

#define UWB_FUNC_POLL 0xE0
#define UWB_FUNC_RESP 0xE1

// SDK ex_06a defaults: 240 UUS delay, 400 UUS timeout
// We need much larger values for 8 MHz SPI (anchor needs 1500 UUS to respond)
#define UWB_POLL_TX_TO_RESP_RX_DLY_UUS 1400
#define UWB_RESP_RX_TIMEOUT_UUS        3000

#define SPEED_OF_LIGHT 299702547.0

// Two-stage UWB range filter: per-anchor Hampel outlier rejection followed
// by an EMA smoother. Tuned for ~32 Hz static data (see spec in PR).
#define UWB_HAMPEL_WINDOW 21
#define UWB_HAMPEL_NSIGMA 3.0f
#define UWB_EMA_ALPHA     0.02f
#define UWB_MAX_DIST_MM   20000
// Reset filter state after this many consecutive missed responses so stale
// buffer contents don't pollute outputs when an anchor comes back.
#define UWB_FILTER_MISS_THRESHOLD 10

typedef struct {
  uint16_t ring[UWB_HAMPEL_WINDOW];
  uint8_t ring_idx;
  uint8_t ring_filled;
  float ema_value;
  bool ema_initialized;
  uint8_t miss_count;
} uwb_filter_t;

static uwb_filter_t uwb_filters[NUM_ANCHORS];

// Shared UWB distance data (written by UWB thread, read by main loop for BLE TX)
// 0xFFFF = no valid measurement
static volatile uint16_t uwb_dist_mm[NUM_ANCHORS] = {0xFFFF, 0xFFFF, 0xFFFF};
static volatile uint16_t last_m1_adc;
static volatile uint16_t last_m2_adc;
static volatile uint16_t last_tof_mm = 0xFFFF;
/* Raw IMU telemetry bytes from STM32, forwarded as-is in BLE telemetry.
 * Layout: 6 × f32 little-endian = ax, ay, az (m/s²), gx, gy, gz (rad/s). */
static uint8_t last_imu_bytes[24];
/* Raw motor ADC telemetry bytes from STM32, forwarded as-is.
 * Layout: 6 × u16 little-endian = m1_adc1..4, m2_adc1..2. */
static uint8_t last_motor_adc_bytes[12];

static uint16_t median_u16(uint16_t* buf, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }
  return buf[n / 2];
}

static void uwb_filter_reset(int anchor) {
  uwb_filter_t* f = &uwb_filters[anchor];
  f->ring_idx = 0;
  f->ring_filled = 0;
  f->ema_value = 0.0f;
  f->ema_initialized = false;
  f->miss_count = 0;
}

static void uwb_filter_note_miss(int anchor) {
  uwb_filter_t* f = &uwb_filters[anchor];
  if (f->miss_count < 0xFF) {
    f->miss_count++;
  }
  if (f->miss_count >= UWB_FILTER_MISS_THRESHOLD) {
    uwb_filter_reset(anchor);
  }
}

static void uwb_filter_update(int anchor, uint16_t dist_mm) {
  if (dist_mm > UWB_MAX_DIST_MM) {
    return;
  }

  uwb_filter_t* f = &uwb_filters[anchor];
  f->miss_count = 0;

  f->ring[f->ring_idx] = dist_mm;
  f->ring_idx = (f->ring_idx + 1) % UWB_HAMPEL_WINDOW;
  if (f->ring_filled < UWB_HAMPEL_WINDOW) {
    f->ring_filled++;
  }

  // Stage 1: Hampel outlier rejection. Tests the newest sample (zero lag)
  // against the local median/MAD computed over the full window.
  uint16_t y_hampel = dist_mm;
  if (f->ring_filled >= UWB_HAMPEL_WINDOW) {
    uint16_t scratch[UWB_HAMPEL_WINDOW];
    memcpy(scratch, f->ring, sizeof(scratch));
    uint16_t m = median_u16(scratch, UWB_HAMPEL_WINDOW);

    uint16_t dev[UWB_HAMPEL_WINDOW];
    for (int i = 0; i < UWB_HAMPEL_WINDOW; i++) {
      int32_t diff = (int32_t)f->ring[i] - (int32_t)m;
      dev[i] = (uint16_t)(diff < 0 ? -diff : diff);
    }
    uint16_t mad = median_u16(dev, UWB_HAMPEL_WINDOW);
    float sigma = 1.4826f * (float)mad;

    if (sigma > 0.0f) {
      float d = (float)dist_mm - (float)m;
      if (d < 0.0f) {
        d = -d;
      }
      if (d > UWB_HAMPEL_NSIGMA * sigma) {
        y_hampel = m;
      }
    }
  }

  // Stage 2: EMA smoothing.
  if (!f->ema_initialized) {
    f->ema_value = (float)y_hampel;
    f->ema_initialized = true;
  } else {
    f->ema_value =
        UWB_EMA_ALPHA * (float)y_hampel + (1.0f - UWB_EMA_ALPHA) * f->ema_value;
  }

  float rounded = f->ema_value + 0.5f;
  if (rounded > (float)UWB_MAX_DIST_MM) {
    rounded = (float)UWB_MAX_DIST_MM;
  }
  uwb_dist_mm[anchor] = (uint16_t)rounded;
}

/* Define NUS UUID so scanners can see us in Scan Response */
#define NUS_UUID_Service_Val                                                             \
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00,    \
      0x40, 0x6E

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, TILE_NAME, sizeof(TILE_NAME) - 1),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, NUS_UUID_Service_Val),
};

static struct k_work adv_work;
static struct bt_conn* current_conn;

/* Avoid flooding BLE and stressing stack: rate-limit NUS send and back off on failure */
#define NUS_SEND_INTERVAL_MS 25
#define NUS_SEND_BACKOFF_MS  3000
static uint32_t last_nus_send_ms;
static uint32_t nus_send_backoff_until_ms;

/* Set in disconnected(); main loop sends M1+M2 brake to STM32 on next SPI transaction */
static volatile bool send_brake_on_disconnect;
/* Set in bt_receive_cb when queue is full; main loop sends M1+M2 brake on next SPI transaction */
static volatile bool send_brake_on_queue_full;

/* BLE command watchdog: if connected but no BLE data arrives within this period, brake.
 * Catches frozen/crashed GUI. Reset on every bt_receive_cb. */
#define BLE_CMD_WATCHDOG_MS 2000
static volatile uint32_t last_ble_rx_ms;
static bool ble_watchdog_braked;

static void adv_work_handler(struct k_work* work) {
  int err =
      bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Advertising failed to restart (err %d)", err);
  }
}

static const struct device* const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);
static const struct gpio_dt_spec drdy_pin = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);

K_MSGQ_DEFINE(ble_msgq, SPI_BUF_SIZE, 32, 4);

static uint8_t tx_buffer[SPI_BUF_SIZE];
static uint8_t rx_buffer[SPI_BUF_SIZE];

static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE,
};

// SPI bridge thread: runs blocking spi_transceive so main loop never hangs
static K_SEM_DEFINE(spi_start_sem, 0, 1);
static K_SEM_DEFINE(spi_done_sem, 0, 1);
static volatile int spi_result;

static void spi_bridge_thread_fn(void* p1, void* p2, void* p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    k_sem_take(&spi_start_sem, K_FOREVER);

    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    gpio_pin_set_dt(&drdy_pin, 1);
    spi_result = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    gpio_pin_set_dt(&drdy_pin, 0);

    k_sem_give(&spi_done_sem);
  }
}

K_THREAD_STACK_DEFINE(spi_bridge_stack, 1024);
static struct k_thread spi_bridge_thread;

/* BLE callbacks */

static void bt_receive_cb(struct bt_conn* conn, const uint8_t* const data, uint16_t len) {
  last_ble_rx_ms = k_uptime_get_32();
  uint8_t temp_buf[SPI_BUF_SIZE];

  uint16_t copy_len = (len > SPI_BUF_SIZE) ? SPI_BUF_SIZE : len;
  memcpy(temp_buf, data, copy_len);

  if (copy_len < SPI_BUF_SIZE) {
    memset(temp_buf + copy_len, 0, SPI_BUF_SIZE - copy_len);
  }

  if (k_msgq_put(&ble_msgq, temp_buf, K_NO_WAIT) != 0) {
    LOG_WRN("BLE Queue Full");
    send_brake_on_queue_full = true;
  }
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

static void connected(struct bt_conn* conn, uint8_t err) {
  if (err) {
    LOG_ERR("Connection failed (err 0x%02x)", err);
    return;
  }

  LOG_INF("Connected");

  /* Track the current connection for NUS TX */
  last_ble_rx_ms = k_uptime_get_32();
  ble_watchdog_braked = false;
  current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
  LOG_INF("Disconnected (reason 0x%02x). Restarting advertising...", reason);

  if (current_conn == conn) {
    send_brake_on_disconnect = true;
    bt_conn_unref(current_conn);
    current_conn = NULL;
    nus_send_backoff_until_ms = 0;
  }

  k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err) {
  if (err) {
    LOG_ERR("BLE init failed (err %d)", err);
    return;
  }
  LOG_INF("BLE Initialized");

  err = bt_nus_init(&nus_cb);
  if (err) {
    LOG_ERR("Failed to init NUS (err %d)", err);
    return;
  }

  err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Advertising failed (err %d)", err);
  } else {
    LOG_INF("Advertising started as OmniTile_1");
  }
}

// ---------------------------------------------------------------------------
// UWB Ranging Thread — SS-TWR initiator (based on Qorvo ex_06a)
// ---------------------------------------------------------------------------
static dwt_config_t uwb_config = {
    .chan = 5,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = DWT_SFD_DW_8,
    .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .sfdTO = (128 + 1 + 8 - 8),
    .stsMode = DWT_STS_MODE_OFF,
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0,
};

// Address format: dst[0] = target anchor ID, dst[1] = 'A';
// src[0] = CONFIG_TILE_ID, src[1] = 'G' (magic). The anchor echoes src[0]
// back into the response dst[0] so the tag can reject responses meant for
// other tiles.
#define ADDR_DST_IDX 5
#define ADDR_SRC_IDX 7
static uint8_t tx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 'A', CONFIG_TILE_ID, 'G', UWB_FUNC_POLL, 0, 0};
#define ALL_MSG_SN_IDX          2
#define ALL_MSG_COMMON_LEN      10
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4

static uint8_t uwb_frame_seq_nb = 0;

#define UWB_RX_BUF_LEN 20
static uint8_t uwb_rx_buf[UWB_RX_BUF_LEN];

static void resp_msg_get_ts(const uint8_t* ts_field, uint32_t* ts) {
  *ts = (uint32_t)ts_field[0] | ((uint32_t)ts_field[1] << 8) |
        ((uint32_t)ts_field[2] << 16) | ((uint32_t)ts_field[3] << 24);
}

static void uwb_ranging_thread_fn(void* p1, void* p2, void* p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  dw3000_hw_init();
  dw3000_hw_reset();
  k_sleep(K_MSEC(10));

  if (dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf) != DWT_SUCCESS) {
    LOG_ERR("UWB: dwt_probe failed");
    return;
  }

  if (dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID |
                     DWT_READ_OTP_BAT | DWT_READ_OTP_TMP) != DWT_SUCCESS) {
    LOG_ERR("UWB: dwt_initialise failed");
    return;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
    LOG_ERR("UWB: dwt_configure failed");
    return;
  }

  static dwt_txconfig_t txconfig = {0x34, 0xfdfdfdfd, 0x0};
  dwt_configuretxrf(&txconfig);

  dwt_setrxantennadelay(UWB_RX_ANT_DLY);
  dwt_settxantennadelay(UWB_TX_ANT_DLY);

  dwt_setrxaftertxdelay(UWB_POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(UWB_RESP_RX_TIMEOUT_UUS);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  LOG_INF("UWB: ready (SS-TWR, 3 anchors)");

  // Per-tile slot stagger: offset this tile's ranging cycle so three tiles
  // polling the same anchors are statistically unlikely to collide. Each
  // exchange is ~3 ms and the inter-exchange sleep is 10 ms, so a 5 ms
  // offset between tiles keeps their polls out of each other's slots.
  k_sleep(K_MSEC((CONFIG_TILE_ID - 1) * 5));

  int cur_anchor = 0;

  while (1) {
    uint32_t status_reg;

    // Set target anchor ID in poll frame
    tx_poll_msg[ADDR_DST_IDX] = (uint8_t)cur_anchor;
    tx_poll_msg[ALL_MSG_SN_IDX] = uwb_frame_seq_nb;
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    dwt_starttx(DWT_START_TX_IMMEDIATE);

    // Wait for TX done
    while (!((status_reg = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {
    }
    dwt_writesysstatuslo(0xFFFFFFFF);

    // Manually enable RX with timeout (bypass DWT_RESPONSE_EXPECTED)
    dwt_setrxtimeout(UWB_RESP_RX_TIMEOUT_UUS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // Wait for RX good frame, timeout, or error
    while (!((status_reg = dwt_readsysstatuslo()) &
             (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    }

    uwb_frame_seq_nb++;

    if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
      uint16_t frame_len;

      dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

      frame_len = dwt_getframelength(NULL);
      if (frame_len <= sizeof(uwb_rx_buf)) {
        dwt_readrxdata(uwb_rx_buf, frame_len, 0);

        uwb_rx_buf[ALL_MSG_SN_IDX] = 0;
        // Reject responses not addressed to this tile (dst[0] == CONFIG_TILE_ID)
        // or not from the anchor we just polled (src[0] == cur_anchor).
        // Without these checks, simultaneous ranging from multiple tiles can
        // produce phantom distances.
        if (uwb_rx_buf[9] == UWB_FUNC_RESP && uwb_rx_buf[5] == CONFIG_TILE_ID &&
            uwb_rx_buf[6] == 'G' && uwb_rx_buf[7] == (uint8_t)cur_anchor) {
          uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
          int32_t rtd_init, rtd_resp;
          float clockOffsetRatio;

          poll_tx_ts = dwt_readtxtimestamplo32();
          resp_rx_ts = dwt_readrxtimestamplo32(0);

          clockOffsetRatio = ((float)dwt_readclockoffset()) / (float)(1 << 26);

          resp_msg_get_ts(&uwb_rx_buf[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
          resp_msg_get_ts(&uwb_rx_buf[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

          rtd_init = resp_rx_ts - poll_tx_ts;
          rtd_resp = resp_tx_ts - poll_rx_ts;

          double tof =
              ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
          double distance = tof * SPEED_OF_LIGHT;
          int32_t dist_mm = (int32_t)(distance * 1000.0);

          if (dist_mm < 0) {
            dist_mm = 0;
          }

          uwb_filter_update(cur_anchor, (uint16_t)dist_mm);
        } else if (uwb_rx_buf[9] != UWB_FUNC_RESP) {
          LOG_WRN("UWB: anchor %d unexpected func=0x%02X", cur_anchor, uwb_rx_buf[9]);
        } else {
          // Response addressed to a different tile or anchor — treat as a
          // miss for filter-reset purposes but don't log (noisy when tiles
          // share the UWB channel).
          uwb_filter_note_miss(cur_anchor);
        }
      }
    } else {
      dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      uwb_filter_note_miss(cur_anchor);
    }

    // Cycle to next anchor
    cur_anchor = (cur_anchor + 1) % NUM_ANCHORS;

    // Telemetry is sent exclusively from the main loop after SPI exchange,
    // which includes both motor ADC values and UWB distances.

    // Brief delay between ranging exchanges
    k_sleep(K_MSEC(10));
  }
}

K_THREAD_STACK_DEFINE(uwb_stack, 8192);
static struct k_thread uwb_thread;

int main(void) {
  int ret;

  if (!gpio_is_ready_dt(&drdy_pin) || !device_is_ready(spi_dev)) {
    LOG_ERR("Hardware init failed");
    return -1;
  }
  gpio_pin_configure_dt(&drdy_pin, GPIO_OUTPUT_INACTIVE);

  k_work_init(&adv_work, adv_work_handler);

  ret = bt_enable(bt_ready);
  if (ret) {
    LOG_ERR("Bluetooth enable failed");
    return -1;
  }

  // Start UWB ranging thread (lower priority than BLE)
  k_thread_create(&uwb_thread,
      uwb_stack,
      K_THREAD_STACK_SIZEOF(uwb_stack),
      uwb_ranging_thread_fn,
      NULL,
      NULL,
      NULL,
      K_PRIO_PREEMPT(10),
      0,
      K_NO_WAIT);
  k_thread_name_set(&uwb_thread, "uwb_ranging");

  // Start SPI bridge thread (blocks on spi_transceive so main loop stays free)
  k_thread_create(&spi_bridge_thread,
      spi_bridge_stack,
      K_THREAD_STACK_SIZEOF(spi_bridge_stack),
      spi_bridge_thread_fn,
      NULL,
      NULL,
      NULL,
      K_PRIO_PREEMPT(5),
      0,
      K_NO_WAIT);
  k_thread_name_set(&spi_bridge_thread, "spi_bridge");

  LOG_INF("System Ready: %s (UWB_TX_ANT_DLY=%d, UWB_RX_ANT_DLY=%d)",
      TILE_NAME,
      UWB_TX_ANT_DLY,
      UWB_RX_ANT_DLY);

  bool spi_in_flight = false;
  bool spi_was_timing_out = false;

  while (1) {
    // --- Collect SPI response from previous transaction ---
    if (spi_in_flight) {
      ret = k_sem_take(&spi_done_sem, K_MSEC(10));
      if (ret == 0) {
        spi_in_flight = false;

        if (spi_result >= 0 && rx_buffer[0] == 0xA5 && rx_buffer[1] == 0x60) {
          last_m1_adc = (uint16_t)rx_buffer[2] | ((uint16_t)rx_buffer[3] << 8);
          last_m2_adc = (uint16_t)rx_buffer[4] | ((uint16_t)rx_buffer[5] << 8);
          last_tof_mm = (uint16_t)rx_buffer[6] | ((uint16_t)rx_buffer[7] << 8);
          memcpy(last_imu_bytes, &rx_buffer[8], 24);
          memcpy(last_motor_adc_bytes, &rx_buffer[32], 12);

          if (spi_was_timing_out) {
            LOG_INF("SPI: STM32 reconnected, clearing stale brake flags");
            spi_was_timing_out = false;
            send_brake_on_disconnect = false;
            send_brake_on_queue_full = false;
            ble_watchdog_braked = false;
            last_ble_rx_ms = k_uptime_get_32();
          }
        } else if (spi_result < 0) {
          LOG_WRN("spi_transceive failed: %d", spi_result);
        }

        k_sleep(K_MSEC(5));
      } else {
        LOG_WRN("SPI: STM32 did not respond (50 ms timeout), re-pulsing DRDY");
        spi_was_timing_out = true;
        gpio_pin_set_dt(&drdy_pin, 0);
        k_sleep(K_MSEC(5));
        gpio_pin_set_dt(&drdy_pin, 1);
      }
    }

    // --- BLE command watchdog: brake if GUI stops sending ---
    if (current_conn != NULL && !ble_watchdog_braked) {
      uint32_t elapsed = k_uptime_get_32() - last_ble_rx_ms;
      if (elapsed >= BLE_CMD_WATCHDOG_MS) {
        LOG_WRN("BLE watchdog: no commands in %u ms, sending brake", elapsed);
        ble_watchdog_braked = true;
        send_brake_on_disconnect = true;
      }
    }

    // --- Prepare next SPI payload (only if previous completed) ---
    if (!spi_in_flight) {
      ret = k_msgq_get(&ble_msgq, tx_buffer, K_MSEC(20));
      if (ret != 0) {
        memset(tx_buffer, 0, SPI_BUF_SIZE);
      }

      if (send_brake_on_disconnect) {
        send_brake_on_disconnect = false;
        tx_buffer[0] = CMD_START_BYTE;
        tx_buffer[1] = CMD_M1_BRAKE;
        tx_buffer[2] = CMD_M1_BRAKE;
        tx_buffer[3] = CMD_START_BYTE;
        tx_buffer[4] = CMD_M2_BRAKE;
        tx_buffer[5] = CMD_M2_BRAKE;
        memset(tx_buffer + 6, 0, SPI_BUF_SIZE - 6);
      }
      if (send_brake_on_queue_full) {
        send_brake_on_queue_full = false;
        tx_buffer[0] = CMD_START_BYTE;
        tx_buffer[1] = CMD_M1_BRAKE;
        tx_buffer[2] = CMD_M1_BRAKE;
        tx_buffer[3] = CMD_START_BYTE;
        tx_buffer[4] = CMD_M2_BRAKE;
        tx_buffer[5] = CMD_M2_BRAKE;
        memset(tx_buffer + 6, 0, SPI_BUF_SIZE - 6);
      }

      spi_in_flight = true;
      k_sem_give(&spi_start_sem);

      // Wait for this transaction too (first attempt)
      ret = k_sem_take(&spi_done_sem, K_MSEC(50));
      if (ret == 0) {
        spi_in_flight = false;

        if (spi_result >= 0 && rx_buffer[0] == 0xA5 && rx_buffer[1] == 0x60) {
          last_m1_adc = (uint16_t)rx_buffer[2] | ((uint16_t)rx_buffer[3] << 8);
          last_m2_adc = (uint16_t)rx_buffer[4] | ((uint16_t)rx_buffer[5] << 8);
          last_tof_mm = (uint16_t)rx_buffer[6] | ((uint16_t)rx_buffer[7] << 8);
          memcpy(last_imu_bytes, &rx_buffer[8], 24);
          memcpy(last_motor_adc_bytes, &rx_buffer[32], 12);

          if (spi_was_timing_out) {
            LOG_INF("SPI: STM32 reconnected, clearing stale brake flags");
            spi_was_timing_out = false;
            send_brake_on_disconnect = false;
            send_brake_on_queue_full = false;
            ble_watchdog_braked = false;
            last_ble_rx_ms = k_uptime_get_32();
          }
        } else if (spi_result < 0) {
          LOG_WRN("spi_transceive failed: %d", spi_result);
        }

        k_sleep(K_MSEC(5));
      } else {
        LOG_WRN("SPI: STM32 did not respond (50 ms timeout), re-pulsing DRDY");
        spi_was_timing_out = true;
        gpio_pin_set_dt(&drdy_pin, 0);
        k_sleep(K_MSEC(5));
        gpio_pin_set_dt(&drdy_pin, 1);
      }
    }

    // --- BLE telemetry (always runs, even if SPI is stuck) ---
    uint16_t m1_adc = last_m1_adc;
    uint16_t m2_adc = last_m2_adc;
    uint16_t tof_mm = last_tof_mm;

    if (current_conn != NULL) {
      uint32_t now = k_uptime_get_32();
      bool in_backoff = (now < nus_send_backoff_until_ms);
      bool rate_ok = (now - last_nus_send_ms >= NUS_SEND_INTERVAL_MS);

      if (!in_backoff && rate_ok) {
        uint8_t telem[51];
        telem[0] = 0xA5;
        telem[1] = 0x60;
        telem[2] = (uint8_t)(m1_adc);
        telem[3] = (uint8_t)(m1_adc >> 8);
        telem[4] = (uint8_t)(m2_adc);
        telem[5] = (uint8_t)(m2_adc >> 8);

        uint16_t d0 = uwb_dist_mm[0];
        uint16_t d1 = uwb_dist_mm[1];
        uint16_t d2 = uwb_dist_mm[2];

        telem[6] = (uint8_t)(d0);
        telem[7] = (uint8_t)(d0 >> 8);
        telem[8] = (uint8_t)(d1);
        telem[9] = (uint8_t)(d1 >> 8);
        telem[10] = (uint8_t)(d2);
        telem[11] = (uint8_t)(d2 >> 8);
        telem[12] = (uint8_t)(tof_mm);
        telem[13] = (uint8_t)(tof_mm >> 8);

        memcpy(&telem[14], last_imu_bytes, 24);
        memcpy(&telem[38], last_motor_adc_bytes, 12);

        uint8_t csum = 0;
        for (int i = 1; i < 50; i++) {
          csum += telem[i];
        }
        telem[50] = csum;

        int err = bt_nus_send(current_conn, telem, sizeof(telem));
        if (err == 0) {
          last_nus_send_ms = now;
          nus_send_backoff_until_ms = 0;
        } else {
          nus_send_backoff_until_ms = now + NUS_SEND_BACKOFF_MS;
          LOG_WRN("bt_nus_send failed: %d (backing off %d ms)",
              err,
              (int)NUS_SEND_BACKOFF_MS);
        }
      }
    }
  }
  return 0;
}

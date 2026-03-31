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
// UWB ranging constants — must match anchor firmware exactly
// ---------------------------------------------------------------------------
#define NUM_ANCHORS 3

#define UWB_FC_0        0x41
#define UWB_FC_1        0x88
#define UWB_PAN_ID_0    0xCA  // PAN 0xDECA
#define UWB_PAN_ID_1    0xDE
#define UWB_FUNC_POLL   0x21
#define UWB_FUNC_RESP   0x10
#define UWB_FUNC_FINAL  0x23
#define UWB_FUNC_RESULT 0x24

#define UWB_TAG_ADDR_0 0x01
#define UWB_TAG_ADDR_1 0x00

// Anchor short addresses: 0x0010, 0x0011, 0x0012
#define UWB_ANCHOR_ADDR_BASE 0x10

#define UWB_TX_ANT_DLY 16385
#define UWB_RX_ANT_DLY 16385

// Delay between TX of Poll and expected RX of Response, in UWB microseconds
#define UWB_POLL_TX_TO_RESP_RX_DLY_UUS 300
// RX timeout waiting for Response, in UWB microseconds
#define UWB_RESP_RX_TIMEOUT_UUS 3000
// Delay between RX of Response and TX of Final, in UWB microseconds
#define UWB_RESP_RX_TO_FINAL_TX_DLY_UUS 300
// RX timeout waiting for Distance Result, in UWB microseconds
#define UWB_RESULT_RX_TIMEOUT_UUS 3000

// Convert UWB microseconds to DW3000 device time units
#define UUS_TO_DWT_TIME 65536

#define SPEED_OF_LIGHT 299702547.0

// Shared UWB distance data (written by UWB thread, read by main loop for BLE TX)
// 0xFFFF = no valid measurement
static volatile uint16_t uwb_dist_mm[NUM_ANCHORS] = {0xFFFF, 0xFFFF, 0xFFFF};

/* Define NUS UUID so scanners can see us in Scan Response */
#define NUS_UUID_Service_Val                                                             \
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00,    \
      0x40, 0x6E

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "OmniTile_1", 10),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, NUS_UUID_Service_Val),
};

static struct k_work adv_work;
static struct bt_conn* current_conn;

/* Avoid flooding BLE and stressing stack: rate-limit NUS send and back off on failure */
#define NUS_SEND_INTERVAL_MS 50
#define NUS_SEND_BACKOFF_MS  3000
static uint32_t last_nus_send_ms;
static uint32_t nus_send_backoff_until_ms;

/* Set in disconnected(); main loop sends M1+M2 brake to STM32 on next SPI transaction */
static volatile bool send_brake_on_disconnect;
/* Set in bt_receive_cb when queue is full; main loop sends M1+M2 brake on next SPI transaction */
static volatile bool send_brake_on_queue_full;

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

static void set_drdy(bool active) {
  gpio_pin_set_dt(&drdy_pin, active ? 1 : 0);
}

/* BLE callbacks */

static void bt_receive_cb(struct bt_conn* conn, const uint8_t* const data, uint16_t len) {
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
// UWB Ranging Thread — DS-TWR initiator
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

static uint8_t uwb_seq_num = 0;
static uint8_t uwb_rx_buf[24];

static uint64_t uwb_get_rx_timestamp_u64(void) {
  uint8_t ts[5];
  dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
  return ((uint64_t)ts[0]) | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16) |
         ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

static uint64_t uwb_get_tx_timestamp_u64(void) {
  uint8_t ts[5];
  dwt_readtxtimestamp(ts);
  return ((uint64_t)ts[0]) | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16) |
         ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

// Perform one DS-TWR exchange with a single anchor. Returns distance in mm or 0xFFFF on failure.
static uint16_t uwb_range_one_anchor(uint8_t anchor_id) {
  uint8_t anchor_addr_lo = UWB_ANCHOR_ADDR_BASE + anchor_id;

  // --- Send Poll ---
  uint8_t poll_msg[] = {
      UWB_FC_0,
      UWB_FC_1,
      uwb_seq_num++,
      UWB_PAN_ID_0,
      UWB_PAN_ID_1,
      anchor_addr_lo,
      0x00,  // Dest = anchor
      UWB_TAG_ADDR_0,
      UWB_TAG_ADDR_1,  // Src = tag
      UWB_FUNC_POLL,
  };

  dwt_writetxdata(sizeof(poll_msg) + FCS_LEN, poll_msg, 0);
  dwt_writetxfctrl(sizeof(poll_msg) + FCS_LEN, 0, 1);

  dwt_setrxaftertxdelay((uint32_t)UWB_POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(UWB_RESP_RX_TIMEOUT_UUS);

  if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
    return 0xFFFF;
  }

  // Wait for TX complete
  uint32_t status;
  while (!((status = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {
  }
  dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

  uint64_t poll_tx_ts = uwb_get_tx_timestamp_u64();

  // --- Wait for Response ---
  while (!(
      (status = dwt_readsysstatuslo()) &
      (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK |
          DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK))) {
  }

  if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
    dwt_writesysstatuslo(0xFFFFFFFF);
    return 0xFFFF;
  }
  dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

  uint8_t rng_bit = 0;
  uint16_t rx_len = dwt_getframelength(&rng_bit);
  if (rx_len > sizeof(uwb_rx_buf))
    rx_len = sizeof(uwb_rx_buf);
  dwt_readrxdata(uwb_rx_buf, rx_len, 0);

  if (uwb_rx_buf[9] != UWB_FUNC_RESP) {
    return 0xFFFF;
  }

  uint64_t resp_rx_ts = uwb_get_rx_timestamp_u64();

  // --- Send Final (with embedded timestamps for the anchor to compute distance) ---
  uint32_t final_tx_time =
      (uint32_t)((resp_rx_ts +
                     ((uint64_t)UWB_RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                 8);
  dwt_setdelayedtrxtime(final_tx_time);

  // The actual TX timestamp will be final_tx_time << 8 (top 32 bits)
  uint64_t final_tx_ts = (((uint64_t)final_tx_time) << 8) + UWB_TX_ANT_DLY;

  uint8_t final_msg[22] = {
      UWB_FC_0,
      UWB_FC_1,
      uwb_seq_num++,
      UWB_PAN_ID_0,
      UWB_PAN_ID_1,
      anchor_addr_lo,
      0x00,
      UWB_TAG_ADDR_0,
      UWB_TAG_ADDR_1,
      UWB_FUNC_FINAL,
      // Bytes 10..13: poll_tx_ts (lower 32 bits, LE)
      (uint8_t)(poll_tx_ts),
      (uint8_t)(poll_tx_ts >> 8),
      (uint8_t)(poll_tx_ts >> 16),
      (uint8_t)(poll_tx_ts >> 24),
      // Bytes 14..17: resp_rx_ts (lower 32 bits, LE)
      (uint8_t)(resp_rx_ts),
      (uint8_t)(resp_rx_ts >> 8),
      (uint8_t)(resp_rx_ts >> 16),
      (uint8_t)(resp_rx_ts >> 24),
      // Bytes 18..21: final_tx_ts (lower 32 bits, LE)
      (uint8_t)(final_tx_ts),
      (uint8_t)(final_tx_ts >> 8),
      (uint8_t)(final_tx_ts >> 16),
      (uint8_t)(final_tx_ts >> 24),
  };

  dwt_writetxdata(sizeof(final_msg) + FCS_LEN, final_msg, 0);
  dwt_writetxfctrl(sizeof(final_msg) + FCS_LEN, 0, 1);

  dwt_setrxaftertxdelay(0);
  dwt_setrxtimeout(UWB_RESULT_RX_TIMEOUT_UUS);

  if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
    return 0xFFFF;
  }

  // Wait for TX complete
  while (!((status = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {
  }
  dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

  // --- Wait for Distance Result from anchor ---
  while (!(
      (status = dwt_readsysstatuslo()) &
      (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK |
          DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK))) {
  }

  if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
    dwt_writesysstatuslo(0xFFFFFFFF);
    return 0xFFFF;
  }
  dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

  rx_len = dwt_getframelength(&rng_bit);
  if (rx_len > sizeof(uwb_rx_buf))
    rx_len = sizeof(uwb_rx_buf);
  dwt_readrxdata(uwb_rx_buf, rx_len, 0);

  if (uwb_rx_buf[9] != UWB_FUNC_RESULT) {
    return 0xFFFF;
  }

  // Extract distance (uint32 LE at bytes 10..13), clamp to uint16
  uint32_t dist = (uint32_t)uwb_rx_buf[10] | ((uint32_t)uwb_rx_buf[11] << 8) |
                  ((uint32_t)uwb_rx_buf[12] << 16) | ((uint32_t)uwb_rx_buf[13] << 24);

  return (dist > 0xFFFE) ? 0xFFFE : (uint16_t)dist;
}

static void uwb_ranging_thread_fn(void* p1, void* p2, void* p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  LOG_INF("UWB: Initializing DW3000...");

  dw3000_hw_init();
  dw3000_hw_reset();
  k_sleep(K_MSEC(2));

  if (dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf) != DWT_SUCCESS) {
    LOG_ERR("UWB: dwt_probe failed");
    return;
  }

  LOG_INF("UWB: DW3000 Device ID: 0x%08X", dwt_readdevid());

  if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
    LOG_ERR("UWB: dwt_initialise failed");
    return;
  }

  if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
    LOG_ERR("UWB: dwt_configure failed");
    return;
  }

  dwt_setrxantennadelay(UWB_RX_ANT_DLY);
  dwt_settxantennadelay(UWB_TX_ANT_DLY);
  dwt_setaddress16(0x0001);  // Tag address
  dwt_setpanid(0xDECA);

  LOG_INF("UWB: Ranging started (3 anchors)");

  while (1) {
    for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
      uint16_t dist = uwb_range_one_anchor(i);
      uwb_dist_mm[i] = dist;

      if (dist != 0xFFFF) {
        LOG_DBG("UWB: Anchor %d = %u mm", i, dist);
      }

      k_sleep(K_MSEC(2));  // Brief pause between anchors
    }
  }
}

K_THREAD_STACK_DEFINE(uwb_stack, 4096);
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

  LOG_INF("System Ready. Waiting for BLE data...");

  while (1) {
    ret = k_msgq_get(&ble_msgq, tx_buffer, K_MSEC(1000));

    if (ret != 0) {
      memset(tx_buffer, 0, SPI_BUF_SIZE);
    }

    /* On BLE disconnect, send brake to STM32 so motors stop when link is lost */
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
    /* When queue overflowed, send brake so actuators stop immediately */
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

    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    set_drdy(true);
    ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    set_drdy(false);

    // Check for incoming telemetry packet, append UWB distances, and forward over BLE
    if (rx_buffer[0] == 0xA5 && rx_buffer[1] == 0x60) {
      uint8_t orig_checksum = (rx_buffer[1] + rx_buffer[2] + rx_buffer[3]) & 0xFF;
      if (rx_buffer[4] != orig_checksum) {
        LOG_WRN(
            "Telemetry checksum mismatch (got %02X, expected %02X) — forwarding anyway",
            rx_buffer[4],
            orig_checksum);
      }

      if (current_conn != NULL) {
        uint32_t now = k_uptime_get_32();
        bool in_backoff = (now < nus_send_backoff_until_ms);
        bool rate_ok = (now - last_nus_send_ms >= NUS_SEND_INTERVAL_MS);

        if (!in_backoff && rate_ok) {
          // Build extended telemetry: [0xA5, 0x60, m1, m2, d0_lo, d0_hi, d1_lo, d1_hi, d2_lo, d2_hi, csum]
          uint8_t telem[11];
          telem[0] = 0xA5;
          telem[1] = 0x60;
          telem[2] = rx_buffer[2];  // m1_adc
          telem[3] = rx_buffer[3];  // m2_adc

          // Snapshot UWB distances
          uint16_t d0 = uwb_dist_mm[0];
          uint16_t d1 = uwb_dist_mm[1];
          uint16_t d2 = uwb_dist_mm[2];

          telem[4] = (uint8_t)(d0);
          telem[5] = (uint8_t)(d0 >> 8);
          telem[6] = (uint8_t)(d1);
          telem[7] = (uint8_t)(d1 >> 8);
          telem[8] = (uint8_t)(d2);
          telem[9] = (uint8_t)(d2 >> 8);

          // Recompute checksum over bytes 1..9
          uint8_t csum = 0;
          for (int i = 1; i < 10; i++) {
            csum += telem[i];
          }
          telem[10] = csum;

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

    if (ret < 0) {
      LOG_WRN("spi_transceive failed: %d", ret);
    }
  }
  return 0;
}

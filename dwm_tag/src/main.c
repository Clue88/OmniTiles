// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

/*
 * DWM tag (nRF52) application: BLE-to-SPI bridge.
 *
 * Runs as a BLE peripheral (Nordic UART Service), advertising as OmniTile_1.
 * Data received from a device is queued and then sent to the STM32 over SPI when
 * the master asserts CS. A DRDY GPIO is driven high when a payload is ready and
 * cleared after the SPI transaction completes.
 */

#include <bluetooth/services/nus.h>
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
#define CMD_START_BYTE  0xA5
#define CMD_M1_BRAKE    0x32
#define CMD_M2_BRAKE    0x42

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
#define NUS_SEND_INTERVAL_MS  50
#define NUS_SEND_BACKOFF_MS   3000
static uint32_t last_nus_send_ms;
static uint32_t nus_send_backoff_until_ms;

/* Set in disconnected(); main loop sends M1+M2 brake to STM32 on next SPI transaction */
static volatile bool send_brake_on_disconnect;

static void adv_work_handler(struct k_work* work) {
  int err =
      bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Advertising failed to restart (err %d)", err);
  }
}

static const struct device* const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);
static const struct gpio_dt_spec drdy_pin = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);

K_MSGQ_DEFINE(ble_msgq, SPI_BUF_SIZE, 10, 4);

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

    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    set_drdy(true);
    ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    set_drdy(false);

    // Check for incoming telemetry packet and forward over BLE
    if (rx_buffer[0] == 0xA5 && rx_buffer[1] == 0x60) {
      uint8_t checksum = (rx_buffer[1] + rx_buffer[2] + rx_buffer[3]) & 0xFF;
      if (rx_buffer[4] != checksum) {
        LOG_WRN(
            "Telemetry checksum mismatch (got %02X, expected %02X) — forwarding anyway",
            rx_buffer[4],
            checksum);
      }

      if (current_conn == NULL) {
      } else {
        uint32_t now = k_uptime_get_32();
        bool in_backoff = (now < nus_send_backoff_until_ms);
        bool rate_ok = (now - last_nus_send_ms >= NUS_SEND_INTERVAL_MS);

        if (!in_backoff && rate_ok) {
          int err = bt_nus_send(current_conn, rx_buffer, 5);
          if (err == 0) {
            last_nus_send_ms = now;
            nus_send_backoff_until_ms = 0;
          } else {
            /* -22 EINVAL: notifications not enabled; -128 ENOTCONN: link down.
             * Back off to avoid hammering the stack and contributing to drops. */
            nus_send_backoff_until_ms = now + NUS_SEND_BACKOFF_MS;
            LOG_WRN("bt_nus_send failed: %d (backing off %d ms)", err,
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

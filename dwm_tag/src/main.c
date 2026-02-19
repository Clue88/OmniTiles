// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define DRDY_GPIO_NODE DT_ALIAS(spis_drdy_gpios)
#define SPI_BUF_SIZE   128

#define NUS_UUID_Service_Val                                                             \
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00,    \
      0x40, 0x6E

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

/* --- BLE Callbacks --- */

static void bt_receive_cb(struct bt_conn* conn, const uint8_t* const data, uint16_t len) {
  uint8_t temp_buf[SPI_BUF_SIZE];

  uint16_t copy_len = (len > SPI_BUF_SIZE) ? SPI_BUF_SIZE : len;

  memcpy(temp_buf, data, copy_len);

  if (copy_len < SPI_BUF_SIZE) {
    memset(temp_buf + copy_len, 0, SPI_BUF_SIZE - copy_len);
  }

  if (k_msgq_put(&ble_msgq, temp_buf, K_NO_WAIT) != 0) {
    LOG_WRN("BLE Queue Full");
  } else {
    LOG_INF("BLE RX: %d bytes -> Queue", copy_len);
  }
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
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

  // Primary Advertising Data (Flags + Name)
  const struct bt_data ad[] = {
      BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
      BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
  };

  // Scan Response Data (UUID)
  const struct bt_data sd[] = {
      BT_DATA_BYTES(BT_DATA_UUID128_ALL, NUS_UUID_Service_Val),
  };

  // Pass BOTH ad and sd to the advertising macro
  err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Advertising failed (err %d)", err);
  } else {
    LOG_INF("Advertising started");
  }
}

/* --- Main --- */

int main(void) {
  int ret;

  // Init Hardware.
  if (!gpio_is_ready_dt(&drdy_pin) || !device_is_ready(spi_dev)) {
    LOG_ERR("Hardware init failed");
    return -1;
  }
  gpio_pin_configure_dt(&drdy_pin, GPIO_OUTPUT_INACTIVE);

  // Init Bluetooth.
  ret = bt_enable(bt_ready);
  if (ret) {
    LOG_ERR("Bluetooth enable failed");
    return -1;
  }

  LOG_INF("System Ready. Waiting for BLE data...");

  while (1) {
    // Block until data arrives.
    ret = k_msgq_get(&ble_msgq, tx_buffer, K_FOREVER);

    if (ret == 0) {
      struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
      struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

      struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
      struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

      // Handshake with STM32.
      set_drdy(true);
      ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
      set_drdy(false);

      if (ret == 0) {
        LOG_INF("SPI Transaction Complete");
      }
    }
  }
  return 0;
}

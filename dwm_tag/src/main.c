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
#include <zephyr/net/ieee802154_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(main);

#define UWB_THREAD_STACK_SIZE 2048
#define UWB_THREAD_PRIORITY   10
#define UWB_POLL_INTERVAL_MS  100
#define UWB_ANCHOR_PORT       4242
#define UWB_POLL_PAYLOAD      "Poll"
#define UWB_POLL_LEN          4

#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define DRDY_GPIO_NODE DT_ALIAS(spis_drdy_gpios)
#define SPI_BUF_SIZE   128

/* Protocol: same as STM32/omnitiles (START_BYTE, msg_id, checksum). No payload => checksum = msg_id */
#define CMD_START_BYTE 0xA5
#define CMD_M1_BRAKE   0x32
#define CMD_M2_BRAKE   0x42

/* Define NUS UUID so scanners can see us in Scan Response */
#define NUS_UUID_Service_Val                                                             \
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00,    \
      0x40, 0x6E

/* Shared state: distances to three UWB anchors in mm (volatile ok for mock) */
static volatile uint16_t uwb_distances_mm[3] = {0, 0, 0};

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

static struct net_if* uwb_iface_found;

static void find_ieee802154_cb(struct net_if* iface, void* user_data) {
  uint16_t ch;
  if (uwb_iface_found != NULL) {
    return;
  }
  if (net_mgmt(NET_REQUEST_IEEE802154_GET_CHANNEL, iface, &ch, sizeof(ch)) == 0) {
    uwb_iface_found = iface;
  }
}

static void uwb_thread_entry(void* p1, void* p2, void* p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  struct sockaddr_in6 dst;
  int fd;
  int ret;

  uwb_iface_found = NULL;
  net_if_foreach(find_ieee802154_cb, NULL);
  if (uwb_iface_found == NULL || !net_if_is_up(uwb_iface_found)) {
    LOG_ERR("UWB: no IEEE 802.15.4 interface or not up");
    k_thread_suspend(k_current_get());
    return;
  }

  fd = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    LOG_ERR("UWB: socket create failed (%d)", fd);
    k_thread_suspend(k_current_get());
    return;
  }

  memset(&dst, 0, sizeof(dst));
  dst.sin6_family = AF_INET6;
  dst.sin6_port = htons(UWB_ANCHOR_PORT);
  /* ff02::1 (all-nodes multicast) */
  dst.sin6_addr.s6_addr[0] = 0xff;
  dst.sin6_addr.s6_addr[1] = 0x02;
  dst.sin6_addr.s6_addr[15] = 0x01;

  for (;;) {
    ret = zsock_sendto(
        fd, UWB_POLL_PAYLOAD, UWB_POLL_LEN, 0, (struct sockaddr*)&dst, sizeof(dst));
    if (ret < 0) {
      LOG_WRN("UWB: sendto failed (%d)", ret);
    }

    /* Mock: update distances (base + small increment to simulate movement) */
    static uint16_t tick;
    tick++;
    uwb_distances_mm[0] = 1000 + (tick % 200);
    uwb_distances_mm[1] = 2000 + (tick % 200);
    uwb_distances_mm[2] = 3000 + (tick % 200);

    k_msleep(UWB_POLL_INTERVAL_MS);
  }
}

K_THREAD_DEFINE(uwb_thread,
    UWB_THREAD_STACK_SIZE,
    uwb_thread_entry,
    NULL,
    NULL,
    NULL,
    UWB_THREAD_PRIORITY,
    0,
    0);

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
          uint8_t nus_buf[11];
          uint16_t d0 = uwb_distances_mm[0];
          uint16_t d1 = uwb_distances_mm[1];
          uint16_t d2 = uwb_distances_mm[2];

          nus_buf[0] = rx_buffer[0];
          nus_buf[1] = rx_buffer[1];
          nus_buf[2] = rx_buffer[2];
          nus_buf[3] = rx_buffer[3];
          nus_buf[4] = (uint8_t)(d0 >> 8);
          nus_buf[5] = (uint8_t)(d0 & 0xFF);
          nus_buf[6] = (uint8_t)(d1 >> 8);
          nus_buf[7] = (uint8_t)(d1 & 0xFF);
          nus_buf[8] = (uint8_t)(d2 >> 8);
          nus_buf[9] = (uint8_t)(d2 & 0xFF);
          nus_buf[10] = (rx_buffer[1] + rx_buffer[2] + rx_buffer[3] +
                         nus_buf[4] + nus_buf[5] + nus_buf[6] +
                         nus_buf[7] + nus_buf[8] + nus_buf[9]) & 0xFF;

          int err = bt_nus_send(current_conn, nus_buf, 11);
          if (err == 0) {
            last_nus_send_ms = now;
            nus_send_backoff_until_ms = 0;
          } else {
            /* -22 EINVAL: notifications not enabled; -128 ENOTCONN: link down.
             * Back off to avoid hammering the stack and contributing to drops. */
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

/*
 * UWB anchor: IEEE 802.15.4 receive loop.
 * Configures the radio for RX, blocks on incoming frames, logs length and payload bytes.
 * Uses a UDP socket on the 802.15.4 interface (CONFIG_NET_SOCKET_PACKET not in NCS).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/net/ieee802154_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>

#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define RECV_BUF_SIZE   128
#define LOG_PAYLOAD_LEN 8
#define DEFAULT_CHANNEL 11
#define DEFAULT_PAN_ID  0x1234
#define LISTEN_PORT     4242

static uint8_t recv_buf[RECV_BUF_SIZE];

static int configure_ieee802154_rx(struct net_if* iface) {
  uint16_t channel = DEFAULT_CHANNEL;
  uint16_t pan_id = DEFAULT_PAN_ID;
  int ret;

  ret = net_mgmt(NET_REQUEST_IEEE802154_SET_CHANNEL, iface, &channel, sizeof(channel));
  if (ret < 0) {
    LOG_ERR("Set channel failed: %d", ret);
    return ret;
  }

  ret = net_mgmt(NET_REQUEST_IEEE802154_SET_PAN_ID, iface, &pan_id, sizeof(pan_id));
  if (ret < 0) {
    LOG_ERR("Set PAN ID failed: %d", ret);
    return ret;
  }

  net_if_up(iface);
  LOG_INF("IEEE 802.15.4 RX: channel %u, PAN 0x%04x", channel, pan_id);
  return 0;
}

static struct net_if* ieee802154_iface;

static void find_ieee802154_iface_cb(struct net_if* iface, void* user_data) {
  uint16_t channel;

  if (ieee802154_iface != NULL) {
    return;
  }
  if (net_mgmt(NET_REQUEST_IEEE802154_GET_CHANNEL, iface, &channel, sizeof(channel)) ==
      0) {
    ieee802154_iface = iface;
  }
}

int main(void) {
  struct net_if* iface;
  struct sockaddr_in6 addr;
  int fd;
  ssize_t len;
  int ret;

  log_init();

  ieee802154_iface = NULL;
  net_if_foreach(find_ieee802154_iface_cb, NULL);
  iface = ieee802154_iface;
  if (iface == NULL) {
    LOG_ERR("No IEEE 802.15.4 interface");
    return -1;
  }

  ret = configure_ieee802154_rx(iface);
  if (ret < 0) {
    return ret;
  }

  fd = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    LOG_ERR("Socket create failed: %d", fd);
    return -1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin6_family = AF_INET6;
  addr.sin6_port = htons(LISTEN_PORT);
  ret = zsock_bind(fd, (struct sockaddr*)&addr, sizeof(addr));
  if (ret < 0) {
    LOG_ERR("Bind failed: %d", ret);
    zsock_close(fd);
    return -1;
  }

  LOG_INF("UWB anchor RX loop started (UDP port %u); waiting for frames...", LISTEN_PORT);

  for (;;) {
    len = zsock_recv(fd, recv_buf, sizeof(recv_buf), 0);
    if (len < 0) {
      LOG_WRN("recv: %d", (int)len);
      continue;
    }

    LOG_INF("frame len=%zd", len);

    if (len > 0) {
      unsigned int log_len = len < LOG_PAYLOAD_LEN ? (unsigned int)len : LOG_PAYLOAD_LEN;
      LOG_INF("payload (first %u): %02x %02x %02x %02x %02x %02x %02x %02x",
          log_len,
          log_len > 0 ? recv_buf[0] : 0,
          log_len > 1 ? recv_buf[1] : 0,
          log_len > 2 ? recv_buf[2] : 0,
          log_len > 3 ? recv_buf[3] : 0,
          log_len > 4 ? recv_buf[4] : 0,
          log_len > 5 ? recv_buf[5] : 0,
          log_len > 6 ? recv_buf[6] : 0,
          log_len > 7 ? recv_buf[7] : 0);
    }
  }

  zsock_close(fd);
  return 0;
}

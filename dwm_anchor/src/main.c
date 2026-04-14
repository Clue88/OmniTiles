// SPDX-License-Identifier: MIT
// (c) 2025-2026 Christopher Liu
//
// DWM Anchor — SS-TWR responder, based on Qorvo ex_06b_ss_twr_responder.
//
// Each anchor is identified by CONFIG_ANCHOR_ID (0, 1, or 2).
// Flash with: west build -- -DCONFIG_ANCHOR_ID=<n>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(anchor, LOG_LEVEL_INF);

// Must match tag exactly
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

// Frame format — matches SDK ex_06b exactly
// Poll:     [FC 0x41 0x88] [seq] [PAN 0xCA 0xDE] [dst 2B] [src 2B] [func 0xE0]
// Response: [FC 0x41 0x88] [seq] [PAN 0xCA 0xDE] [dst 2B] [src 2B] [func 0xE1]
//           [poll_rx_ts 4B] [resp_tx_ts 4B]
// + 2B CRC auto-appended

#define FUNC_POLL 0xE0
#define FUNC_RESP 0xE1

#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14

// Per-anchor antenna delay (calibrated against a reference tag). Anchor 0 is
// the "golden" reference and stays at nominal 16385; tags are calibrated
// against it. Anchors 1 and 2 absorb their own residual bias relative to
// anchor 0, and those values are tag-independent.
#if CONFIG_ANCHOR_ID == 0
  #define TX_ANT_DLY 16385
  #define RX_ANT_DLY 16385
#elif CONFIG_ANCHOR_ID == 1
  #define TX_ANT_DLY 16385
  #define RX_ANT_DLY 16385
#elif CONFIG_ANCHOR_ID == 2
  #define TX_ANT_DLY 16385
  #define RX_ANT_DLY 16385
#else
  #error "CONFIG_ANCHOR_ID must be 0, 1, or 2"
#endif

// Delay from poll RX to response TX — SDK uses 650 at 38MHz SPI, we need more at 8MHz
#define POLL_RX_TO_RESP_TX_DLY_UUS 1500

#define UUS_TO_DWT_TIME 63898

// Address format: dst[0] = anchor ID, dst[1] = 'A'; src = 'T','G' (tag)
// Anchor only responds to polls where dst[0] matches its own ID.
#define ADDR_DST_IDX 5
static uint8_t tx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'G', 0, 'A', FUNC_RESP, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 5  // Only validate FC + PAN (skip addresses)

#define RX_BUF_LEN 24  // Must fit any frame (poll=12, response=20, +CRC)
static uint8_t rx_buffer[RX_BUF_LEN];

static uint8_t frame_seq_nb = 0;
static uint32_t status_reg = 0;

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

static uint64_t get_rx_timestamp_u64(void) {
  uint8_t ts[5];
  dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
  return ((uint64_t)ts[0]) | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16) |
         ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

static void resp_msg_set_ts(uint8_t* ts_field, uint64_t ts) {
  ts_field[0] = (uint8_t)ts;
  ts_field[1] = (uint8_t)(ts >> 8);
  ts_field[2] = (uint8_t)(ts >> 16);
  ts_field[3] = (uint8_t)(ts >> 24);
}

int main(void) {
  LOG_INF("DWM Anchor %d starting (TX_ANT_DLY=%d, RX_ANT_DLY=%d)...",
      CONFIG_ANCHOR_ID,
      TX_ANT_DLY,
      RX_ANT_DLY);

  dw3000_hw_init();
  dw3000_hw_reset();
  k_sleep(K_MSEC(10));

  if (dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf) != DWT_SUCCESS) {
    LOG_ERR("dwt_probe failed");
    return -1;
  }

  if (dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID |
                     DWT_READ_OTP_BAT | DWT_READ_OTP_TMP) != DWT_SUCCESS) {
    LOG_ERR("dwt_initialise failed");
    return -1;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
    LOG_ERR("dwt_configure failed");
    return -1;
  }

  static dwt_txconfig_t txconfig = {0x34, 0xfdfdfdfd, 0x0};
  dwt_configuretxrf(&txconfig);

  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // Set this anchor's ID in the response frame src address
  tx_resp_msg[7] = CONFIG_ANCHOR_ID;

  LOG_INF("Anchor %d ready (SS-TWR). Listening...", CONFIG_ANCHOR_ID);

  while (1) {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_readsysstatuslo()) &
             (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    }

    if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
      uint16_t frame_len;

      dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

      frame_len = dwt_getframelength(NULL);
      if (frame_len <= sizeof(rx_buffer)) {
        dwt_readrxdata(rx_buffer, frame_len, 0);

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        // Check FC + PAN + addressed to this anchor + function code is POLL
        if (rx_buffer[0] == 0x41 && rx_buffer[1] == 0x88 && rx_buffer[3] == 0xCA &&
            rx_buffer[4] == 0xDE && rx_buffer[ADDR_DST_IDX] == CONFIG_ANCHOR_ID &&
            rx_buffer[9] == FUNC_POLL) {
          uint32_t resp_tx_time;
          int ret;

          poll_rx_ts = get_rx_timestamp_u64();

          resp_tx_time = (uint32_t)((poll_rx_ts + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS *
                                                      UUS_TO_DWT_TIME)) >>
                                    8);
          dwt_setdelayedtrxtime(resp_tx_time);

          resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

          resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
          resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

          tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
          dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
          dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
          ret = dwt_starttx(DWT_START_TX_DELAYED);

          if (ret == DWT_SUCCESS) {
            while (!((status_reg = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {
            }
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            frame_seq_nb++;
          } else {
            LOG_WRN("Resp TX late");
          }
        }
      }
    } else {
      dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
  }

  return 0;
}

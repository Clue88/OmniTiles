// SPDX-License-Identifier: MIT
// (c) 2025-2026 Christopher Liu
//
// DWM Anchor — DS-TWR responder for UWB ranging.
//
// Listens for Poll frames from a tag, participates in the DS-TWR exchange,
// and sends the computed distance back in a Distance Result frame.
//
// Each anchor is identified by CONFIG_ANCHOR_ID (0, 1, or 2).
// Flash with: west build -- -DCONFIG_ANCHOR_ID=<n>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(anchor, LOG_LEVEL_INF);

// ---------------------------------------------------------------------------
// UWB configuration — must match the tag exactly
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

// ---------------------------------------------------------------------------
// Frame definitions for the DS-TWR protocol
// ---------------------------------------------------------------------------
// All frames share a minimal 802.15.4 header:
//   [0..1] Frame control  (0x4188 = data frame, short addr, PAN compress)
//   [2]    Sequence number
//   [3..4] PAN ID
//   [5..6] Destination address (short)
//   [7..8] Source address (short)
//   [9]    Function code: POLL=0x21, RESP=0x10, FINAL=0x23, RESULT=0x24
//   [10..] Payload (timestamps, distance)
//   CRC is appended automatically by the DW3000 (2 bytes)

#define FC_0        0x41
#define FC_1        0x88
#define PAN_ID_0    0xCA  // PAN 0xDECA
#define PAN_ID_1    0xDE
#define FUNC_POLL   0x21
#define FUNC_RESP   0x10
#define FUNC_FINAL  0x23
#define FUNC_RESULT 0x24

#define TAG_ADDR_0 0x01  // Tag always uses short address 0x0001
#define TAG_ADDR_1 0x00

#define FRAME_HEADER_LEN 10  // bytes 0..9

// Response frame: header + 2 bytes reserved (for future use) + CRC = 12 bytes total on air
#define RESP_MSG_LEN (FRAME_HEADER_LEN + 2)

// Result frame: header + 4 bytes (uint32 distance_mm) + CRC = 14 bytes on air
#define RESULT_MSG_LEN (FRAME_HEADER_LEN + 4)

// Antenna delay — calibrate per board for best accuracy (default from Qorvo examples)
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

// Delay between RX of Poll and TX of Response, in UWB microseconds (~300 us)
#define POLL_RX_TO_RESP_TX_DLY_UUS 300
// Delay between RX of Final and TX of Result, in UWB microseconds
#define FINAL_RX_TO_RESULT_TX_DLY_UUS 300
// Receive timeout after sending Response (waiting for Final), in UWB microseconds
#define RESP_TX_TO_FINAL_RX_TIMEOUT_UUS 5000

// Speed of light in air (m/s)
#define SPEED_OF_LIGHT 299702547.0

// Convert UWB microseconds to DW3000 device time units.
// DW3000 counter runs at ~499.2 MHz * 128 = ~63.8976 GHz → 1 UUS ≈ 65536 DTU.
#define UUS_TO_DWT_TIME 65536

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------
static uint8_t rx_buf[24];

static uint8_t resp_msg[] = {
    FC_0,
    FC_1,
    0x00,  // FC + seq
    PAN_ID_0,
    PAN_ID_1,  // PAN
    TAG_ADDR_0,
    TAG_ADDR_1,  // Dest = tag
    0x00,
    0x00,       // Src = this anchor (filled at init)
    FUNC_RESP,  // Function code
    0x00,
    0x00,  // Reserved
};

static uint8_t result_msg[] = {
    FC_0,
    FC_1,
    0x00,
    PAN_ID_0,
    PAN_ID_1,
    TAG_ADDR_0,
    TAG_ADDR_1,
    0x00,
    0x00,  // Src = this anchor (filled at init)
    FUNC_RESULT,
    0x00,
    0x00,
    0x00,
    0x00,  // Distance in mm (uint32 LE)
};

static uint8_t seq_num = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static uint64_t get_rx_timestamp_u64(void) {
  uint8_t ts[5];
  dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
  return ((uint64_t)ts[0]) | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16) |
         ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

static uint64_t get_tx_timestamp_u64(void) {
  uint8_t ts[5];
  dwt_readtxtimestamp(ts);
  return ((uint64_t)ts[0]) | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16) |
         ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

static void set_anchor_addr(void) {
  uint8_t addr_lo = 0x10 + CONFIG_ANCHOR_ID;  // 0x10, 0x11, 0x12
  uint8_t addr_hi = 0x00;

  resp_msg[7] = addr_lo;
  resp_msg[8] = addr_hi;
  result_msg[7] = addr_lo;
  result_msg[8] = addr_hi;

  dwt_setaddress16((addr_hi << 8) | addr_lo);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void) {
  LOG_INF("DWM Anchor %d starting...", CONFIG_ANCHOR_ID);

  // Init DW3000 hardware (SPI, GPIOs, reset)
  dw3000_hw_init();
  dw3000_hw_reset();
  k_sleep(K_MSEC(2));

  if (dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf) != DWT_SUCCESS) {
    LOG_ERR("dwt_probe failed");
    return -1;
  }

  uint32_t dev_id = dwt_readdevid();
  LOG_INF("DW3000 Device ID: 0x%08X", dev_id);

  if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
    LOG_ERR("dwt_initialise failed");
    return -1;
  }

  if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
    LOG_ERR("dwt_configure failed — PLL lock error?");
    return -1;
  }

  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  set_anchor_addr();
  dwt_setpanid(0xDECA);

  // Enable frame filtering: accept data frames addressed to this anchor or broadcast
  dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);

  LOG_INF("Anchor %d ready. Waiting for Poll frames...", CONFIG_ANCHOR_ID);

  while (1) {
    // --- Step 1: Enable RX and wait for a Poll frame ---
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // Poll for RX good frame or error/timeout
    uint32_t status;
    while (!(
        (status = dwt_readsysstatuslo()) &
        (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK |
            DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK))) {
      // Spin — anchor has nothing else to do
    }

    if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
      // RX error or timeout — clear and retry
      dwt_writesysstatuslo(0xFFFFFFFF);
      continue;
    }

    // Clear good RX event
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    // Read received frame
    uint8_t rng_bit = 0;
    uint16_t rx_len = dwt_getframelength(&rng_bit);
    if (rx_len > sizeof(rx_buf)) {
      rx_len = sizeof(rx_buf);
    }
    dwt_readrxdata(rx_buf, rx_len, 0);

    // Check it's a Poll frame
    if (rx_buf[9] != FUNC_POLL) {
      continue;
    }

    // Record Poll RX timestamp
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    // --- Step 2: Send Response frame with delayed TX ---
    uint32_t resp_tx_time =
        (uint32_t)((poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
    dwt_setdelayedtrxtime(resp_tx_time);

    // Turn on RX after TX to catch the Final frame
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(RESP_TX_TO_FINAL_RX_TIMEOUT_UUS);

    resp_msg[2] = seq_num++;
    dwt_writetxdata(sizeof(resp_msg) + FCS_LEN, resp_msg, 0);
    dwt_writetxfctrl(sizeof(resp_msg) + FCS_LEN, 0, 1);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret != DWT_SUCCESS) {
      // Missed the TX window — restart
      LOG_WRN("Resp TX late");
      continue;
    }

    // Wait for TX done
    while (!((status = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {
    }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    uint64_t resp_tx_ts = get_tx_timestamp_u64();

    // --- Step 3: Wait for Final frame ---
    while (!(
        (status = dwt_readsysstatuslo()) &
        (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK |
            DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK))) {
    }

    if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
      dwt_writesysstatuslo(0xFFFFFFFF);
      continue;
    }
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    rx_len = dwt_getframelength(&rng_bit);
    if (rx_len > sizeof(rx_buf)) {
      rx_len = sizeof(rx_buf);
    }
    dwt_readrxdata(rx_buf, rx_len, 0);

    if (rx_buf[9] != FUNC_FINAL) {
      continue;
    }

    uint64_t final_rx_ts = get_rx_timestamp_u64();

    // Extract tag's timestamps embedded in the Final frame (bytes 10..24)
    uint32_t poll_tx_ts_tag = (uint32_t)rx_buf[10] | ((uint32_t)rx_buf[11] << 8) |
                              ((uint32_t)rx_buf[12] << 16) | ((uint32_t)rx_buf[13] << 24);
    uint32_t resp_rx_ts_tag = (uint32_t)rx_buf[14] | ((uint32_t)rx_buf[15] << 8) |
                              ((uint32_t)rx_buf[16] << 16) | ((uint32_t)rx_buf[17] << 24);
    uint32_t final_tx_ts_tag = (uint32_t)rx_buf[18] | ((uint32_t)rx_buf[19] << 8) |
                               ((uint32_t)rx_buf[20] << 16) |
                               ((uint32_t)rx_buf[21] << 24);

    // --- Step 4: Compute DS-TWR distance ---
    // All timestamps are 32-bit (lower 32 of 40-bit counter). Differences
    // handle wrap correctly for intervals < half the counter period (~17s).
    uint32_t t_round_1 = (uint32_t)(resp_rx_ts_tag - poll_tx_ts_tag);
    uint32_t t_reply_1 = (uint32_t)(resp_tx_ts - poll_rx_ts);
    uint32_t t_round_2 = (uint32_t)(final_rx_ts - resp_tx_ts);
    uint32_t t_reply_2 = (uint32_t)(final_tx_ts_tag - resp_rx_ts_tag);

    // DS-TWR formula: ToF = (R1*R2 - D1*D2) / (R1 + R2 + D1 + D2)
    int64_t tof_dtu = ((int64_t)t_round_1 * t_round_2 - (int64_t)t_reply_1 * t_reply_2) /
                      ((int64_t)t_round_1 + t_round_2 + t_reply_1 + t_reply_2);

    if (tof_dtu < 0) {
      tof_dtu = 0;
    }

    double tof_sec = tof_dtu * DWT_TIME_UNITS;
    double distance_m = tof_sec * SPEED_OF_LIGHT;
    uint32_t distance_mm = (uint32_t)(distance_m * 1000.0);

    LOG_INF("Range: %u mm", distance_mm);

    // --- Step 5: Send Distance Result frame ---
    uint32_t result_tx_time =
        (uint32_t)((final_rx_ts + (FINAL_RX_TO_RESULT_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                   8);
    dwt_setdelayedtrxtime(result_tx_time);

    result_msg[2] = seq_num++;
    result_msg[10] = (uint8_t)(distance_mm);
    result_msg[11] = (uint8_t)(distance_mm >> 8);
    result_msg[12] = (uint8_t)(distance_mm >> 16);
    result_msg[13] = (uint8_t)(distance_mm >> 24);

    dwt_writetxdata(sizeof(result_msg) + FCS_LEN, result_msg, 0);
    dwt_writetxfctrl(sizeof(result_msg) + FCS_LEN, 0, 1);

    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret != DWT_SUCCESS) {
      LOG_WRN("Result TX late");
      continue;
    }

    // Wait for TX complete
    while (!((status = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {
    }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
  }

  return 0;
}

// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <string.h>

LOG_MODULE_REGISTER(main);

#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define SPI_BUF_SIZE   128

static const struct device* const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);
static uint8_t tx_buffer[SPI_BUF_SIZE];
static uint8_t rx_buffer[SPI_BUF_SIZE];

static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE,
};

int main(void) {
  if (!device_is_ready(spi_dev)) {
    LOG_ERR("SPI device not ready");
    return -1;
  }

  LOG_INF("Basic SPI Slave Ready. Waiting for STM32 Master clock...");

  /* Load our test payload into the TX buffer */
  memset(tx_buffer, 0, SPI_BUF_SIZE);
  strcpy((char*)tx_buffer, "Hello from nRF52 SPI Slave!");

  while (1) {
    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    /* This function naturally blocks until the Master clocks out 128 bytes */
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

    if (ret == 0) {
      LOG_INF("Transaction Complete! STM32 sent: %02X %02X %02X...",
          rx_buffer[0],
          rx_buffer[1],
          rx_buffer[2]);
    } else {
      LOG_ERR("SPI Error: %d", ret);
    }
  }
  return 0;
}

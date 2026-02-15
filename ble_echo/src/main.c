// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define DRDY_GPIO_NODE DT_ALIAS(spis_drdy_gpios)
#define SPI_BUF_SIZE   128

static const struct device* const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);
static const struct gpio_dt_spec drdy_pin = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);

static uint8_t tx_buffer[SPI_BUF_SIZE];
static uint8_t rx_buffer[SPI_BUF_SIZE];

/* SPI Configuration: Slave Mode, 8-bit words, CPOL=0, CPHA=0 */
static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE,
    .frequency = 4000000,
    .slave = 0,
};

static void set_drdy(bool active) {
  gpio_pin_set_dt(&drdy_pin, active ? 1 : 0);
}

int main(void) {
  int ret;

  if (!gpio_is_ready_dt(&drdy_pin)) {
    LOG_ERR("DRDY GPIO not ready");
    return -1;
  }
  gpio_pin_configure_dt(&drdy_pin, GPIO_OUTPUT_INACTIVE);

  if (!device_is_ready(spi_dev)) {
    LOG_ERR("SPI Device not ready");
    return -1;
  }

  LOG_INF("SPI Slave Initialized");

  while (1) {
    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    // Signal the Master that we are ready for a transaction.
    // The Master should detect this rising edge and assert CS.
    set_drdy(true);

    // In SLAVE mode, this function blocks indefinitely until the Master asserts CS and clocks out
    // the data.
    ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

    // Transaction complete or aborted.
    set_drdy(false);

    if (ret == 0) {
      LOG_INF("RX: 0x%02X 0x%02X ...", rx_buffer[0], rx_buffer[1]);
    } else {
      LOG_ERR("SPI Error: %d", ret);
    }

    // Prevent tight loop if master is spamming.
    k_yield();
  }
  return 0;
}

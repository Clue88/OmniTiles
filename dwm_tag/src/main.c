// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <string.h>

LOG_MODULE_REGISTER(main);

#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define DRDY_GPIO_NODE DT_ALIAS(spis_drdy_gpios)
#define SPI_BUF_SIZE   128

static const struct device* const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);
static const struct gpio_dt_spec drdy_pin = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);

static uint8_t tx_buffer[SPI_BUF_SIZE];
static uint8_t rx_buffer[SPI_BUF_SIZE];

static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE,
};

int main(void) {
  if (!device_is_ready(spi_dev) || !gpio_is_ready_dt(&drdy_pin)) {
    LOG_ERR("Hardware init failed");
    return -1;
  }

  gpio_pin_configure_dt(&drdy_pin, GPIO_OUTPUT_INACTIVE);

  LOG_INF("SPI + DRDY Slave Ready.");

  int counter = 0;

  while (1) {
    // Prepare the payload
    memset(tx_buffer, 0, SPI_BUF_SIZE);
    snprintk((char*)tx_buffer, SPI_BUF_SIZE, "Data Packet #%d from nRF52!", counter++);

    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    // Handshake: Tell STM32 we have data
    gpio_pin_set_dt(&drdy_pin, 1);

    // This blocks until the STM32 finishes clocking the 128 bytes
    spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

    // Handshake: Clear the pin
    gpio_pin_set_dt(&drdy_pin, 0);

    LOG_INF("Sent: %s", tx_buffer);

    // Wait 1 second before generating the next "fix"
    k_msleep(1000);
  }
  return 0;
}

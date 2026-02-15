#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/* ---------------- Configuration ---------------- */
#define SPI_SLAVE_NODE DT_NODELABEL(my_spis)
#define DRDY_GPIO_NODE DT_ALIAS(spis_drdy_gpios)

/* Buffer Sizes */
#define SPI_BUF_SIZE 128

/* ---------------- Globals ---------------- */
/* SPI Device Struct (from Device Tree) */
static const struct device* const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);

/* Data Ready GPIO Struct */
static const struct gpio_dt_spec drdy_pin = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);

/* Buffers for SPI transaction */
static uint8_t tx_buffer[SPI_BUF_SIZE];
static uint8_t rx_buffer[SPI_BUF_SIZE];

/* Semaphore to signal main loop when transfer is done */
K_SEM_DEFINE(spi_done_sem, 0, 1);

/* ---------------- Helper Functions ---------------- */

/* 1. Toggle the Data Ready Pin */
void set_drdy(bool active) {
  if (active) {
    gpio_pin_set_dt(&drdy_pin, 1);
  } else {
    gpio_pin_set_dt(&drdy_pin, 0);
  }
}

/* 2. SPI Configuration Structure */
static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE,
    .frequency = 4000000,  // 4 MHz (Max for this slave driver is usually 8MHz)
    .slave = 0,            // Chip select 0
};

/* ---------------- Main ---------------- */
int main(void) {
  int ret;

  /* A. Initialize GPIO (Data Ready Pin) */
  if (!gpio_is_ready_dt(&drdy_pin)) {
    printk("Error: DRDY GPIO not ready\n");
    return 0;
  }
  gpio_pin_configure_dt(&drdy_pin, GPIO_OUTPUT_INACTIVE);

  /* B. Initialize SPI */
  if (!device_is_ready(spi_dev)) {
    printk("Error: SPI Device not ready\n");
    return 0;
  }
  printk("SPI Slave Initialized. Waiting for Master...\n");

  /* C. The Loop */
  while (1) {
    /* 1. Prepare Buffers */
    // For test: Fill TX with a counter or recognizable pattern
    tx_buffer[0] = 0xAA;  // Start byte
    tx_buffer[1] = 0xBB;
    tx_buffer[2] = 0xCC;

    struct spi_buf tx_buf = {.buf = tx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {.buf = rx_buffer, .len = SPI_BUF_SIZE};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    /* 2. Assert DRDY to tell STM32 we are ready to trade */
    printk("Asserting DRDY. Waiting for transfer...\n");
    set_drdy(true);

    /* 3. Blocking Call: Wait for Master to Clock Data */
    // This function blocks until CS goes Low -> High
    ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

    /* 4. Transfer Complete */
    set_drdy(false);  // Lower DRDY immediately

    if (ret == 0) {
      printk(
          "Transaction Complete! Received: 0x%02X 0x%02X\n", rx_buffer[0], rx_buffer[1]);
    } else {
      printk("SPI Error: %d\n", ret);
    }

    /* Wait a bit before next transaction (for testing) */
    k_msleep(100);
  }
  return 0;
}

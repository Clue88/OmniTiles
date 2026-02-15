#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

int main(void) {
  // 1. Initialize BLE
  // (We will add this in Step 1.4)

  // 2. Initialize SPI Slave
  // (We will add this in Step 1.2)

  printk("System Initialized.\n");

  while (1) {
    k_sleep(K_FOREVER);
  }

  return 0;
}

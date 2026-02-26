/*
 * UWB anchor application: initializes the DW3000 device and logs status.
 * When a DW3000 driver (decawave,dw3000) is added via ZEPHYR_EXTRA_MODULES,
 * add the device check back and use DEVICE_DT_GET(DT_NODELABEL(dw3000)).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	log_init();

	LOG_INF("UWB anchor: DW3000 driver not in build; add Zephyr module for decawave,dw3000 to enable device check.");
	return 0;
}

# DWM tag — BLE-to-SPI bridge

This folder is the firmware for the DWM tag (nRF52) on the OmniTiles PCB. The tag runs as a BLE peripheral using the Nordic UART Service (NUS) and advertises as **OmniTile_1**. Whatever it receives over BLE it forwards to the STM32 over SPI, with the nRF52 as SPI slave. So the [GUI](../gui/) (or any NUS client) can send command packets over the air, and the [OmniTiles firmware](../omnitiles/) on the STM32 reads them via SPI.

On the PCB the STM32 has no BLE; it polls a DRDY line and does SPI reads. The tag receives a packet from the host, enqueues it (up to 128 bytes), then in its main loop waits for that message, drives DRDY high, and blocks on an SPI transaction. The STM32 sees DRDY high, asserts chip-select, reads 128 bytes from the slave, then deasserts CS. When the transfer finishes, the tag drives DRDY low and goes back to waiting for the next BLE packet. The protocol (sync byte, message ID, checksum) is defined in the omnitiles crate; the tag just shuttles bytes.

**Hardware / devicetree:** SPI slave is under `DT_NODELABEL(my_spis)` (see your board overlay or `app.overlay`). The DRDY pin is the GPIO alias `spis_drdy_gpios`—output, high when a payload is ready and low after the SPI transaction. [prj.conf](prj.conf) enables BLE peripheral, NUS, device name, SPI slave, and logging (e.g. RTT), and sets MTU/buffer sizes for the packets we use.

## Building and flashing (nRF Connect for VS Code)

We use the **nRF Connect for VS Code** extension for building and flashing the DWM tag.

1. Install the [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-for-vscode) extension if you haven’t already.
2. Open the `dwm_tag` folder in VS Code (or open the repo and have `dwm_tag` as the active folder for the nRF extension).
3. In the nRF Connect sidebar, choose the correct board (e.g. nRF52832 or nRF52840, depending on your DWM tag hardware).
4. Use **Build** to compile the application.
5. Connect the DWM tag and use **Flash** to program it. The extension will use the configured board and probe.

If you need to change the board target or build configuration, use the extension’s configuration UI rather than editing things by hand so the extension and Zephyr stay in sync.

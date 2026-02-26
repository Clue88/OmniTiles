# DWM tag: BLE-to-SPI bridge

This folder contains the Zephyr firmware for the DWM tag (nRF52) on the OmniTiles. The tag runs as a
BLE peripheral using the Nordic UART Service (NUS). Whatever it receives over BLE it forwards to the
STM32 over SPI, with the nRF52 as SPI slave. The [GUI](../gui/) (or any NUS client) can send command
packets over the air, and the [OmniTiles firmware](../omnitiles/) on the STM32 reads them via SPI.

## Building and flashing (nRF Connect for VS Code)

We use the **nRF Connect for VS Code** extension for building and flashing the DWM tag.

1. Install the [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-for-vscode)
extension if you haven’t already.
2. Open the `dwm_tag` folder in VS Code (or open the repo and have `dwm_tag` as the active folder
for the nRF extension).
3. In the nRF Connect sidebar, add the corresponding build configuration (use `prj.conf` and
`app.overlay` when prompted).
4. Use **Build** to compile the application.
5. Connect the DWM tag and use **Flash** to program it. The extension will use the configured board
and probe.

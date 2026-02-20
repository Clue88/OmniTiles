# DWM tag — BLE-to-SPI bridge

Firmware for the **DWM tag** (nRF52) used on the OmniTiles PCB. It runs as a **BLE peripheral** using the Nordic UART Service (NUS), advertises as **OmniTile_1**, and forwards data to the main STM32 over **SPI slave**. This is the wireless link between the [debug GUI](../gui/) and the [OmniTiles firmware](../omnitiles/).

## Role in the project

On the PCB, the host (phone or PC running the GUI) connects over BLE to the nRF52. The STM32 does not have BLE; it talks to the nRF52 over SPI as **master**. The flow is:

1. Host sends a command packet over BLE (NUS) to the nRF52.
2. The nRF52 enqueues the payload (up to 128 bytes) in a message queue.
3. In its main loop, the nRF52 waits for a message, then drives **DRDY** high and blocks on an SPI transaction.
4. The STM32 firmware polls DRDY; when high, it asserts SPI chip-select, reads 128 bytes from the slave, then deasserts CS.
5. The nRF52 SPI transfer completes; it drives DRDY low and loops back to wait for the next BLE packet.

So the DWM tag is a **bridge**: BLE in, SPI out, with DRDY signaling “data ready for the STM32.” The protocol (sync byte, message ID, checksum) is defined in the [omnitiles protocol](../omnitiles/src/protocol/messages.rs); this app only forwards opaque bytes.

## Hardware / devicetree

- **SPI:** Runs as **slave**; the node is `DT_NODELABEL(my_spis)` (configure in your board overlay or `app.overlay`).
- **DRDY:** GPIO alias `spis_drdy_gpios` — output driven high when a payload is ready and low after the SPI transaction completes.

## Building and flashing

This is a **Zephyr** application. Use a Zephyr SDK and the appropriate board target (e.g. nRF52832/nRF52840, depending on your DWM tag hardware).

Example (adjust board and path to Zephyr as needed):

```bash
west build -b <your_nrf52_board> --pristine
west flash
```

Configuration is in [prj.conf](prj.conf): BLE peripheral, NUS, device name `OmniTile_1`, SPI slave, and logging (e.g. RTT). MTU and buffer sizes are set to support the expected packet sizes.

## Summary

| What | Detail |
|------|--------|
| **Platform** | Zephyr on nRF52 |
| **BLE** | Peripheral, NUS, advertises as `OmniTile_1` |
| **SPI** | Slave; 128-byte buffer; DRDY GPIO signals “data ready” |
| **Upstream** | [OmniTiles firmware](../omnitiles/) (STM32) reads via SPI when DRDY is high |
| **Downstream** | [GUI](../gui/) or any NUS client sends command packets over BLE |

# dwm_tag — nRF52 BLE-to-SPI bridge + UWB ranging

## Build and flash

Built and flashed via **nRF Connect for VS Code**. Board: DWM3001CDK (nRF52833).
Config files: `prj.conf` (Kconfig), `app.overlay` (device tree overlay).

## What it does

1. BLE peripheral with Nordic UART Service (NUS). Currently hardcoded as "OmniTile_1"
   but will need per-tile unique names for multi-tile setups.
2. Receives command packets from GUI over BLE, forwards them to STM32 over SPI (slave)
3. Runs a UWB ranging thread (DS-TWR initiator) against fixed anchors
4. Embeds UWB distances into telemetry packets before forwarding to GUI
5. Sends brake command on BLE disconnect or queue overflow (safety)

## Key pins and peripherals

- SPI slave: SCK=P0.31, MOSI=P0.26, MISO=P0.07, CS=P0.30
- DRDY (data ready for STM32): P0.28
- DW3000 on SPI3: CS=P1.6, IRQ=P0.12, RESET=P0.13
- BLE MTU: 247 bytes

## C style

clang-format with Google base style:
```
BasedOnStyle: Google, ColumnLimit: 90, AllowShortBlocksOnASingleLine: Never,
AllowShortIfStatementsOnASingleLine: Never, AllowShortLoopsOnASingleLine: false,
BinPackArguments: false, BinPackParameters: false, InsertNewlineAtEOF: true
```

## Notes

- Protocol changes must stay in sync with `omnitiles` and `gui`.
- Logging is via Segger RTT (not UART).
- The single source file is `src/main.c`.

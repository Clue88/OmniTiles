# dwm_tag — nRF52 BLE-to-SPI bridge + UWB ranging

## Build and flash

Built and flashed via **nRF Connect for VS Code**. Board: DWM3001CDK (nRF52833).
Config files: `prj.conf` (Kconfig), `Kconfig` (app-specific options), `app.overlay`
(device tree overlay).

Set tile ID at build time (1, 2, or 3):
```
west build -- -DCONFIG_TILE_ID=1
```
`CONFIG_TILE_ID` controls the BLE advertising name (`OmniTile_<n>`), the per-tile
UWB antenna delay calibration, the UWB poll source address (so anchors can echo it
back and tags can reject responses meant for other tiles), and a small startup
slot offset that staggers multiple tiles on the shared UWB channel.

## What it does

1. BLE peripheral with Nordic UART Service (NUS), advertising as `OmniTile_<CONFIG_TILE_ID>`.
2. Receives command packets from GUI over BLE, forwards them to STM32 over SPI (slave)
3. Runs a UWB ranging thread (DS-TWR initiator) against fixed anchors. Poll frames
   embed `CONFIG_TILE_ID` in the source address; responses are filtered by tile ID
   so multiple tiles can share the same anchors without cross-contamination.
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

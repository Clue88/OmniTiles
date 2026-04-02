# dwm_anchor — nRF52 UWB anchor (DS-TWR responder)

## Build and flash

Built and flashed via **nRF Connect for VS Code**. Board: DWM3001CDK (nRF52833).
Config files: `prj.conf` (Kconfig), `app.overlay` (device tree overlay).

Set anchor ID at build time:
```
west build -- -DCONFIG_ANCHOR_ID=0   # 0, 1, or 2
```

## What it does

DS-TWR (Double-Sided Two-Way Ranging) responder. Listens for Poll frames from tags,
exchanges Response/Final frames, computes distance, and sends a Distance Result frame
back. Each anchor has a unique short address (0x0010, 0x0011, 0x0012). Anchors are
shared infrastructure — multiple tiles will range against the same set of anchors.

## UWB configuration

- DW3000 channel 5, 6.8 Mbps, 128-symbol preamble
- PAN ID: 0xDECA
- Antenna delay: 16385 DTU
- Tag address: 0x0001

## C style

Same clang-format config as `dwm_tag` (Google base, 90 column limit).

## Notes

- Logging via Segger RTT.
- Single source file: `src/main.c`.
- Anchor positioning and trilateration setup is still evolving.

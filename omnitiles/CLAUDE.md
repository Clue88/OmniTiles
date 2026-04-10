# omnitiles — STM32F7 Rust firmware

## Build and flash

```bash
cargo run --release        # build + flash via probe-rs
cargo build --release      # build only
cargo doc --no-deps --open # local API docs
```

Target: `thumbv7em-none-eabihf` (STM32F777VITx). Config in `Embed.toml` and
`.cargo/config.toml`.

## Important: embedded_hal

The `stm32f7xx-hal` crate (v0.8) re-exports an older version of `embedded_hal`. Do NOT
add `embedded_hal` as a direct dependency or use its traits directly. Use the APIs and
types from `stm32f7xx-hal` — there are examples throughout the codebase.

## Pin configuration

Multiple pin configs exist in `src/hw/`: `pins_f767zi.rs` (Nucleo dev board),
`pins_v1.rs` (PCB v1 with RP2040 USB-UART bridge), `pins_v2.rs` (current custom PCB).
The active config is selected in `src/hw/mod.rs`. Currently using `pins_v2`.

## Code structure

- `src/main.rs` — init, main loop, command dispatch
- `src/protocol/` — binary packet parser and message ID constants
- `src/hw/` — peripheral setup (UART, SPI, I2C, ADC, encoder, CAN, LED)
- `src/drivers/` — device drivers (DRV8873, Actuonix P16/T16, GIM6010, FIT0185, VL53L0x)
- `src/control/` — PID controller, linear actuator closed-loop control

## Conventions

- This is `#![no_std]` bare-metal. No heap, no unwinding.
- Use `stm32f7xx-hal` types for all peripheral access.
- Motor drivers use IN1/IN2 PWM pins; see `main.rs` for wiring.
- Protocol changes must stay in sync with `dwm_tag` and `gui`.

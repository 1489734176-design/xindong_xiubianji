# Gear startup and reference flash-code regression

The runner compiles **current production C**, not a rewritten controller model:
- `user_gears_init()` and `get_gear_from_adc()`, including their shared threshold table.
- `LedError_Control()` compared to the actual `LedError_Control()` extracted from the supplied reference program. The reference tree is read-only.

Checks cover all 4096 ADC codes with all six previous gear values, unchanged stationary-knob behavior, ten-sample runtime confirmation/reset, all single/pair combinations of the reference's 25 fault bits, fault priority, and suppression of the added battery-communication flash code without clearing its fault latch.

```sh
python -I tests/ui_control/run_tests.py --keil "<Keil_v5>" --cmsis "<CMSIS/Core/Include>" --reference "<reference project root>" --compile-only
```

`--compile-only` checks compilation/linking only; **it does not execute assertions**.
Remove it to execute on a Cortex-M0 emulator when `unicorn` and `pyelftools` are already installed in that Python environment. The script does not install or download dependencies. All outputs are temporary and existing Keil artifacts are untouched.

## Board checks

- Keep the knob fixed in each gear (especially inside the wide hysteresis regions), power-cycle, and confirm the resulting gear and target speed are unchanged by old Flash records.
- Startup uses the existing **downshift thresholds including the 50 mV margin**: ADC boundaries 82/901/1720/2498/3522, nominally 0.10/1.10/2.10/3.05/4.30 V. Exactly at a boundary retains the higher gear, as in runtime downshifting.
- Normal running still uses independent up/down thresholds and ten consecutive 1 ms samples.
- Confirm fault flash counts 1–10 and reference fallback 11, including priority with multiple faults. `LED_FAULT_BAT_COM_EN=0` disables only the added fault's flash indication, not motor inhibit or fault latching.

No software-only result establishes actual ADC noise tolerance, timing, or correct LED wiring on the board.

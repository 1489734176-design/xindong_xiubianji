# Speed-transition regression checks

`run_tests.py` extracts the **current production C functions** for speed measurement,
reference ramp, PI, PWM tracking/current limiting, and gear targets. It compiles
with the project's ARMCC 5 toolchain and can execute on Keil's Cortex-M0 FVP.
Only `PwmDutyUpdate()` is replaced; these tests do not run the closed commutation
library, real ADC interrupts, IPD, or a physical motor.

Requirements: Python 3 standard library, Keil ARMCC 5, CMSIS headers. Executing
requires an installed Cortex-M0 FVP **and its separate valid license**. Nothing is
downloaded by this runner. Build outputs use a temporary directory unless
`--build-dir` is supplied; existing `KEIL_PRJ` artifacts are not changed.

```sh
python -I tests/speed_control/run_tests.py --keil "<Keil_v5>" --cmsis "<CMSIS/Core/Include>"
```

Without an FVP license, add `--compile-only`. That checks compilation/linking,
**not execution or test assertions**.

Coverage:
- Unchanged six eRPM targets and gear PWM ceilings.
- Bumpless PI entry, monotonic bounded reference ramp and tapered final approach.
- All 36 gear transitions, immediate downshift reference and bounded PI state.
- Bounded acceleration lead, its decay at constant speed, and small timing noise.
- Actuator-lag anti-windup, startup-current-limit freeze and Q15 fractional control.
- Stop/fault/invalid-BEMF reset and restart without stale transition history.
- Normal 1 ms PWM tracking and the unchanged fast peak-current limiting path.

## Board verification still required

Use the same board, parameter set, supply and load as before the change. Record
hardware revision, voltage, load and whether the motor was stopped or coasting.
Start at 20 V under the user's original conditions:

1. Each gear: cold start and repeated stop/restart, observing current **and speed**.
2. Adjacent up/down shifts, then 1→6 and 6→1 sweeps; compare peak current,
   peak eRPM, settling time and steady current/speed with the previous firmware.
3. Confirm a load step still recovers speed without slow hunting. Retain all
   current/voltage/thermal/watchdog/stall protections throughout testing.
4. Inspect `motor_control_list.motor_speed`, `Speed_Pid.refValue`,
   `Speed_Pid.error`, `Speed_Pid.output`, `pwm_duty_aim2`, and
   `mc_core_list.pwm_actual_runing_data` if the transient remains.

The 40 ms acceleration lookahead and 32 ms reference taper are initial tuning
values, not measured motor constants. Acceleration torque can legitimately make
current exceed its eventual steady value; a 760 mA steady reading is not a valid
startup current cap. Passing software checks does not establish a physical
current or speed-overshoot bound.

"""Run extracted production speed/PWM code on the installed Keil Cortex-M0 FVP.

Only PwmDutyUpdate is stubbed. No network packages or target board are needed.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent


def extract_function(path, signature):
    source = path.read_text(encoding="utf-8")
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for pos in range(opening, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[start : pos + 1]
    raise ValueError(f"Unbalanced function in {path}: {signature}")


def run(command):
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
    output = result.stdout + result.stderr
    if output:
        print(output, end="" if output.endswith("\n") else "\n")
    if result.returncode or "warning:" in output.lower():
        raise RuntimeError(f"Tool failed or emitted a warning: {command[0]}")
    return output


def build_and_run(keil, cmsis, output, compile_only=False):
    output.mkdir(parents=True, exist_ok=True)
    code = [
        '#include "parameter.h"',
        '#include "motor_config.h"',
        '#include "motor_control.h"',
        '#include "mc_core.h"',
        '#include "user_control.h"',
        '#include <string.h>',
        "Motor_Control_list motor_control_list;",
        "PidPI_t Speed_Pid;",
        "Mc_core_list mc_core_list;",
        "User_list_Struct user_list;",
        "Machine_ERROR_HOLD machine_error_hold;",
        "static u16 pwm_last;",
        "void PwmDutyUpdate(u16 duty) { pwm_last=duty; }",
    ]
    for name in (
        "void pwm_set(void)",
        "void phase_time_filter(void)",
        "void get_speed(void)",
        "void  Speed_PI_init(void)",
        "void  Speed_PI_reset(void)",
        "int32_t MC_PI_Handler(PidPI_t *pi, int32_t errorBack)",
        "void motor_speed_control_1ms(void)",
    ):
        code.append(extract_function(ROOT / "MOTOR_CONTROL/motor_control.c", name))
    for name in ("void pwm_duty_aim_deal(void)", "void user_gears_handle(void)"):
        code.append(extract_function(ROOT / "USER/user_control.c", name))
    code.append(f'#include "{(TEST_DIR / "test_speed_control.c").as_posix()}"')
    source = output / "speed_tests.c"
    source.write_text("\n\n".join(code) + "\n", encoding="utf-8")
    startup = output / "startup.s"
    startup.write_text("""
        PRESERVE8
        THUMB
        AREA STACK, NOINIT, READWRITE, ALIGN=3
        SPACE 0x1000
__initial_sp
        EXPORT __initial_sp
        AREA HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
        SPACE 0x100
__heap_limit
        EXPORT __heap_base
        EXPORT __heap_limit
        AREA RESET, DATA, READONLY
        EXPORT __Vectors
__Vectors
        DCD __initial_sp
        DCD Reset_Handler
        SPACE 56
        AREA |.text|, CODE, READONLY
        IMPORT __main
        EXPORT Reset_Handler
Reset_Handler PROC
        LDR r0, =__main
        BX r0
        ENDP
        END
""", encoding="ascii")
    scatter = output / "tests.sct"
    scatter.write_text("""
LR_CODE 0x00000000 0x100000 {
    ER_CODE 0x00000000 0x100000 {
        startup.o (RESET, +FIRST)
        *(InRoot$$Sections)
        .ANY (+RO)
    }
    RW_DATA 0x20000000 0x100000 { .ANY (+RW +ZI) }
}
""", encoding="ascii")
    tools = keil / "ARM/ARMCC/bin"
    compiler = [
        str(tools / "armcc.exe"), "-c", "--cpu", "Cortex-M0", "--c99", "-O1",
        "--apcs=interwork", "--split_sections", "-D__MICROLIB",
        "-DUSE_STDPERIPH_DRIVER", "-DUSE_5V",
        "-o", str(output / "tests.o"), str(source),
    ]
    for include in (
        ROOT / "MM32SPIN0230/Include", ROOT / "MM32SPIN0230/HAL_Lib/inc",
        ROOT / "USER/inc", ROOT / "MOTOR_CONTROL", ROOT / "MC_CORE", cmsis,
    ):
        compiler.extend(["-I", str(include)])
    run(compiler)
    run([str(tools / "armasm.exe"), "--cpu", "Cortex-M0", "--apcs=interwork",
         "-o", str(output / "startup.o"), str(startup)])
    image = output / "speed_tests.axf"
    run([str(tools / "armlink.exe"), "--cpu", "Cortex-M0", "--library_type=microlib",
         "--strict", "--scatter", str(scatter), "--entry", "Reset_Handler",
         str(output / "startup.o"), str(output / "tests.o"), "-o", str(image)])
    if compile_only:
        print("PASS: speed regression image compiled and linked without warnings (not executed)")
        return
    simulator = keil / "ARM/avh-fvp/bin/models/FVP_MPS2_Cortex-M0.exe"
    command = [str(simulator), "--quiet", "--timelimit", "60", "-a", str(image)]
    for port in range(3):
        # Disable terminal windows; local semihosting reports the test results.
        command.extend(["-C", f"fvp_mps2.telnetterminal{port}.mode=raw",
                        "-C", f"fvp_mps2.telnetterminal{port}.start_telnet=0"])
    result = run(command)
    if "PASS: speed control" not in result:
        raise RuntimeError("Simulator exited without passing the speed tests")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keil", type=Path, required=True)
    parser.add_argument("--cmsis", type=Path, required=True,
                        help="CMSIS/Core/Include containing core_cm0.h")
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--compile-only", action="store_true",
                        help="Build without running the separately licensed FVP simulator")
    args = parser.parse_args()
    if args.build_dir:
        build_and_run(args.keil.resolve(), args.cmsis.resolve(), args.build_dir.resolve(), args.compile_only)
    else:
        with tempfile.TemporaryDirectory(prefix="router-speed-tests-") as temp:
            build_and_run(args.keil.resolve(), args.cmsis.resolve(), Path(temp), args.compile_only)


if __name__ == "__main__":
    main()

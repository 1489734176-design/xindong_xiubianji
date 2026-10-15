"""Compile actual gear/LED C against the named reference, optionally run with Unicorn.

No packages are installed. --compile-only needs only Python, ARMCC and CMSIS.
Execution additionally requires existing unicorn and pyelftools installations.
"""

import argparse
from pathlib import Path
import struct
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent


def extract_function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for pos in range(opening, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[start:pos + 1]
    raise ValueError(f"Unbalanced function: {signature}")


def build(args, output):
    output.mkdir(parents=True, exist_ok=True)
    user = (ROOT / "USER/user_control.c").read_text(encoding="utf-8")
    led = (ROOT / "USER/led.c").read_text(encoding="utf-8")
    reference = (args.reference / "USER/led.c").read_text(encoding="gbk")
    reference = extract_function(reference, "uint8_t LedError_Control(void)")
    reference = reference.replace("LedError_Control", "reference_led_error_control")
    reference = reference.replace("user_led_control", "reference_led_control")
    # This unused local is in the reference, not needed for its enabled branch.
    reference = reference.replace("uint8_t i;", "")
    gear_start = user.index("static const u16 gear_down_adc[")
    gear_end = user.index("/**\n * @brief 按确认档位选择固定PWM目标", gear_start)
    code = [
        '#include "parameter.h"', '#include "user_control.h"', '#include "led.h"',
        "User_list_Struct user_list;", "Machine_ERROR_HOLD machine_error_hold;",
        "User_LED_Control_t user_led_control, reference_led_control;",
        user[gear_start:gear_end],
        extract_function(led, "uint8_t LedError_Control(void)"), reference,
        f'#include "{(TEST_DIR / "test_ui_control.c").as_posix()}"',
    ]
    source = output / "ui_tests.c"
    source.write_text("\n\n".join(code) + "\n", encoding="utf-8")
    tools = args.keil / "ARM/ARMCC/bin"
    obj = output / "ui_tests.o"
    command = [
        str(tools / "armcc.exe"), "-c", "--cpu", "Cortex-M0", "--c99", "-O1",
        "--apcs=interwork", "--split_sections", "-D__MICROLIB",
        "-DUSE_STDPERIPH_DRIVER", "-DUSE_5V", "-o", str(obj), str(source),
    ]
    for include in (
        ROOT / "MM32SPIN0230/Include", ROOT / "MM32SPIN0230/HAL_Lib/inc",
        ROOT / "USER/inc", ROOT / "MC_CORE", args.cmsis,
    ):
        command += ["-I", str(include)]
    run_tool(command)
    image = output / "ui_tests.axf"
    run_tool([
        str(tools / "armlink.exe"), "--cpu", "Cortex-M0", "--library_type=microlib",
        "--strict", "--ro_base=0x08000000", "--rw_base=0x20000000",
        "--entry=run_tests", str(obj), "-o", str(image),
    ])
    return image


def run_tool(command):
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
    output = result.stdout + result.stderr
    if output:
        print(output, end="")
    if result.returncode or "warning:" in output.lower():
        raise RuntimeError(f"Tool failed or warned: {command[0]}")


def execute(image):
    from elftools.elf.elffile import ELFFile
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

    emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emulator.mem_map(0x08000000, 0x100000)
    emulator.mem_map(0x20000000, 0x100000)
    with image.open("rb") as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                emulator.mem_write(segment["p_vaddr"], segment.data())
        symbols = {symbol.name: symbol["st_value"]
                   for symbol in elf.get_section_by_name(".symtab").iter_symbols()}
    stop = 0x080F0000
    emulator.reg_write(UC_ARM_REG_SP, 0x200FFFF0)
    emulator.reg_write(UC_ARM_REG_LR, stop | 1)
    emulator.emu_start(symbols["run_tests"] | 1, stop, timeout=60_000_000, count=200_000_000)
    if emulator.reg_read(UC_ARM_REG_PC) != stop:
        raise RuntimeError("Test did not return within its execution limit")

    def word(name):
        return struct.unpack("<I", emulator.mem_read(symbols[name], 4))[0]

    if word("test_failure_line"):
        raise AssertionError(f'Case {word("test_case")} failed at '
                             f'test_ui_control.c:{word("test_failure_line")}')
    print(f'PASS: {word("test_case")} groups, {word("test_checks")} checks '
          '(actual Cortex-M0 C functions; not a board test)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keil", type=Path, required=True)
    parser.add_argument("--cmsis", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--compile-only", action="store_true")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="router-ui-tests-") as temporary:
        image = build(args, Path(temporary))
        if args.compile_only:
            print("PASS: UI regression image compiled and linked without warnings; NOT EXECUTED")
        else:
            execute(image)


if __name__ == "__main__":
    main()

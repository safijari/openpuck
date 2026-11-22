#!/usr/bin/env python3
"""Host-side compile and UBSan test runner for OpenPuck Joy-Con 2 BLE mode.

Checks:
1. Dual-header syntax gate: every vendored libjoycon2 header compiles cleanly
   under both strict ISO C99 and C++17.
2. Pure C99 compile of vendored libjoycon2 sources + C++17 compile of OpenPuck
   Joy-Con 2 mode adapters under UndefinedBehaviorSanitizer.
3. Execution of tests/joycon2/test_joycon2_mode.cpp.
"""

from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parent.parent
LIB_DIR = ROOT / "OpenPuck" / "src" / "libjoycon2"
OPENPUCK_DIR = ROOT / "OpenPuck"
TEST_SRC = ROOT / "tests" / "joycon2" / "test_joycon2_mode.cpp"


def run(cmd, *, input_text=None):
    res = subprocess.run(
        cmd,
        input=input_text,
        text=True,
        capture_output=True,
        check=False,
    )
    if res.returncode != 0:
        print(f"FAIL: {' '.join(map(str, cmd))}", file=sys.stderr)
        if res.stdout:
            print(res.stdout, file=sys.stderr)
        if res.stderr:
            print(res.stderr, file=sys.stderr)
        sys.exit(res.returncode)
    return res.stdout


def main():
    headers = sorted(LIB_DIR.glob("*.h"))
    if not headers:
        print(f"No headers found in {LIB_DIR}", file=sys.stderr)
        sys.exit(1)

    for hdr in headers:
        inc = f'#include "{hdr.name}"\n'
        run(
            [
                "cc",
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wpedantic",
                "-DJC2_HOST_TEST=1",
                f"-I{LIB_DIR}",
                "-x",
                "c",
                "-fsyntax-only",
                "-",
            ],
            input_text=inc,
        )
        run(
            [
                "c++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-DJC2_HOST_TEST=1",
                f"-I{LIB_DIR}",
                "-x",
                "c++",
                "-fsyntax-only",
                "-",
            ],
            input_text=inc,
        )
    print(f"PASS: dual-header gate ({len(headers)} headers in C99 + C++17)")

    c_sources = sorted(LIB_DIR.glob("*.c"))
    cpp_sources = [
        OPENPUCK_DIR / "jc2_pad_mouse.cpp",
        OPENPUCK_DIR / "jc2_esb_coex.cpp",
        OPENPUCK_DIR / "mode_joycon2_ble.cpp",
        TEST_SRC,
    ]

    with tempfile.TemporaryDirectory(prefix="opk-jc2-test-") as td:
        tmp = Path(td)
        objs = []

        for src in c_sources:
            obj = tmp / f"{src.stem}.c.o"
            run(
                [
                    "cc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-Wpedantic",
                    "-O2",
                    "-g",
                    "-fsanitize=undefined",
                    "-fno-omit-frame-pointer",
                    "-DJC2_HOST_TEST=1",
                    f"-I{LIB_DIR}",
                    "-c",
                    str(src),
                    "-o",
                    str(obj),
                ]
            )
            objs.append(obj)

        for src in cpp_sources:
            obj = tmp / f"{src.stem}.cpp.o"
            run(
                [
                    "c++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-O2",
                    "-g",
                    "-fsanitize=undefined",
                    "-fno-omit-frame-pointer",
                    "-DOPK_HOST_TEST=1",
                    "-DJC2_HOST_TEST=1",
                    f"-I{OPENPUCK_DIR}",
                    "-c",
                    str(src),
                    "-o",
                    str(obj),
                ]
            )
            objs.append(obj)

        exe = tmp / "test_joycon2_mode"
        run(
            [
                "c++",
                "-fsanitize=undefined",
                "-fno-omit-frame-pointer",
                *[str(o) for o in objs],
                "-lm",
                "-o",
                str(exe),
            ]
        )
        out = run([str(exe)])
        print(out.strip())


if __name__ == "__main__":
    main()

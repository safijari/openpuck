#!/usr/bin/env python3
"""Build and run native XInput regression tests without the Arduino SDK."""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests" / "xinput"


def run(command):
    print("+", shlex.join(str(arg) for arg in command), flush=True)
    subprocess.run(command, check=True)


def fixture_header(destination):
    data = json.loads((TESTS / "fixtures.json").read_text())
    rows = []
    for console in data["consoles"]:
        fields = [json.dumps(console[key]) for key in
                  ("console", "nonce", "identification", "init", "init_response")]
        for key in ("verify", "verify_response"):
            fields.append("{" + ",".join(map(json.dumps, console[key])) + "}")
        rows.append("{" + ",".join(fields) + "}")
    text = "static const Fixture fixtures[] = {" + ",\n".join(rows) + "};\n"
    for key in ("mac_wrap", "salt_wrap"):
        text += f"static const char {key}[] = {json.dumps(data[key])};\n"
    destination.write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asan", action="store_true",
                        help="also enable AddressSanitizer (requires host runtime)")
    parser.add_argument("--check-fixtures", action="store_true",
                        help="regenerate/compare independent fixtures; needs OpenSSL")
    args = parser.parse_args()
    if args.check_fixtures:
        actual = subprocess.check_output(
            [os.sys.executable, str(TESTS / "reference.py")], text=True)
        if json.loads(actual) != json.loads((TESTS / "fixtures.json").read_text()):
            raise SystemExit("Independent fixture regeneration differs")
        print("PASS independent OpenSSL/Python fixture regeneration", flush=True)

    cc = shlex.split(os.environ.get("CC", "cc"))
    cxx = shlex.split(os.environ.get("CXX", "c++"))
    flags = ["-O1", "-g", "-Wall", "-Wextra", "-Werror",
             "-fno-omit-frame-pointer", "-fsanitize=undefined",
             "-fsanitize-undefined-trap-on-error"]
    if args.asan:
        flags.append("-fsanitize=address")
    with tempfile.TemporaryDirectory(prefix="openpuck-xinput-") as name:
        build = Path(name)
        fixture_header(build / "fixtures.h")
        includes = ["-I" + str(TESTS), "-I" + str(ROOT / "OpenPuck"),
                    "-I" + str(build)]
        sources = [ROOT / "OpenPuck" / "src" / "libxsm3" / name for name in
                   ("excrypt_des.c", "excrypt_sha.c", "excrypt_parve.c",
                    "usbdsec.c", "xsm3.c")]
        sources += [ROOT / "OpenPuck" / "xinput_auth.cpp", TESTS / "test.cpp"]
        objects = []
        for source in sources:
            obj = build / (source.name + ".o")
            compiler = cc if source.suffix == ".c" else cxx
            standard = "-std=c11" if source.suffix == ".c" else "-std=c++17"
            run(compiler + [standard] + flags + includes +
                ["-c", str(source), "-o", str(obj)])
            objects.append(str(obj))
        executable = build / "test-xinput"
        run(cxx + flags + objects + ["-o", str(executable)])
        run([str(executable)])
    print("PASS native XInput tests (UBSan trap" +
          (" + ASan" if args.asan else "") + ")", flush=True)


if __name__ == "__main__":
    main()
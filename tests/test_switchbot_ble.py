"""Run the BLE controller with asynchronous scanner and packet fixtures."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture(scope="module")
def ble_test(tmp_path_factory):
    compiler = shutil.which(os.environ.get("CXX", "g++"))
    assert compiler, "A C++ compiler is required for the BLE controller tests"
    root = Path(__file__).resolve().parents[1]
    build = tmp_path_factory.mktemp("switchbot-ble")
    # The harness supplies the SDK functions used by the controller.
    for header in ("esp_bt.h", "esp_heap_caps.h"):
        (build / header).write_text("#pragma once\n")
    binary = build / "switchbot-ble"
    subprocess.run([
        compiler, "-std=c++17", "-g", "-fsanitize=address,undefined",
        "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
        "-I", str(build), "-I", str(root / "src"),
        str(root / "tests/cpp/switchbot_ble_test.cpp"), "-o", str(binary),
    ], check=True)
    return binary


@pytest.mark.parametrize("scenario", [
    "identification", "discovery", "timeout", "rollover", "replacement",
    "disable", "transition", "ota", "service_only", "unidentified", "mixed_slots", "empty", "zero",
    "restart", "replace_saved", "unpair_disabled", "invalid_saved", "all_slots",
])
def test_switchbot_ble(ble_test, scenario):
    subprocess.run([str(ble_test), scenario], check=True)

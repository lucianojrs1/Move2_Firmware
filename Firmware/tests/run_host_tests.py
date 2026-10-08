"""Run portable C tests for the actual SD formatter and batch commit code.

Usage: python Firmware/tests/run_host_tests.py (requires clang or a host C compiler).
"""
import json
import os
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
build = root / "build" / "host_tests"
build.mkdir(parents=True, exist_ok=True)
compiler = os.environ.get("CC") or (shutil.which("cl") if os.name == "nt" else None) or shutil.which("clang") or shutil.which("cc")
if not compiler:
    raise SystemExit("Install a host C compiler or set CC.")
exe = build / ("test_sd.exe" if os.name == "nt" else "test_sd")
sources = [str(root / "main/sd_record.c"), str(root / "main/sd_batch.c"),
           str(root / "main/gps_format.c"), str(root / "main/gps_navigation.c"),
           str(root / "main/motion.c"), str(root / "main/imu_format.c"),
           str(root / "tests/test_metrics.c"), str(root / "tests/test_sd.c")]
if Path(compiler).stem.lower() == "cl":
    cmd = [compiler, "/nologo", "/std:c11", "/W4", "/WX", "/D_CRT_SECURE_NO_WARNINGS",
           "/I" + str(root / "main"), *sources, "/Fe:" + str(exe)]
else:
    cmd = [compiler]
    if os.name == "nt": cmd += ["--target=x86_64-pc-windows-msvc", "-D_CRT_SECURE_NO_WARNINGS"]
    cmd += ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(root / "main"), *sources, "-o", str(exe)]
    if os.name != "nt": cmd += ["-lm"]
subprocess.run(cmd, check=True, cwd=build)
result = subprocess.run([str(exe)], capture_output=True, text=True)
if result.returncode:
    raise SystemExit(result.stderr or f"C tests exited with code {result.returncode}")
records = [json.loads(line) for line in result.stdout.splitlines()]
assert len(records) == 16
can, rtr, dlc15, imu, gps, worst_gps, health = records[:7]
assert can["seq"] == 4294967297 and can["timestamp_ms"] == 1757800000123
assert can["mono_us"] == 987654321098 and can["boot"] == "123456789abcdef0"
assert can["id"] == 0x1803F3F4 and can["data"] == "0011223344556677"
assert can["extended"] and not can["rtr"]
assert rtr["rtr"] and rtr["data"] == "" and rtr["timestamp_ms"] is None
assert dlc15["dlc"] == 15 and dlc15["data"] == can["data"]
assert imu["acc_raw"] == [-32768, 32767, 0] and imu["sensor_time"] == 0xFFFFFFFF
assert imu["temp_raw"] == -123 and imu["acc_conf"] == 0x4028
assert gps["sentence"] == '$GNRMC,"quoted",\\,\t*00'
assert worst_gps["sentence"] == "\x01" * 191
assert health["dropped"] == [12, 0, 0] and health["io_errors"] == 2
assert health["last_data_us"] == 12345 and health["last_sync_us"] == 45678
assert health["committed_bytes"] == 4294967297
assert health["motion_queue_dropped"] == 0
no_data, no_fix, position, stale, zero = [r["value"] for r in records[7:12]]
assert no_data["positionStatus"] == "sem_dados" and no_data["ageMs"] is None
assert no_fix["positionStatus"] == "sem_fix" and no_fix["latitude"] is None
assert position["latitude"] == -8.055338 and position["longitude"] == -34.951803
assert position["valid"] and position["satellites"] == 8 and position["ageMs"] == 500
assert stale["positionStatus"] == "desatualizado" and stale["longitude"] is None
assert not stale["valid"] and stale["ageMs"] == 3000
assert zero["valid"] and zero["latitude"] == 0 and zero["longitude"] == 0
trip, sd_trip, motion, sd_motion = records[12:]
assert trip["value"]["speedKmh"] == 0 and trip["value"]["courseDeg"] is None
assert trip["value"]["altitudeM"] == 42.5 and trip["value"]["timeToFirstFixS"] == 1
assert sd_trip["type"] == "trip" and sd_trip["value"]["tripDistanceM"] == trip["value"]["tripDistanceM"]
assert motion["value"]["imuTemperatureC"] is None and motion["value"]["motionValid"]
assert motion["value"]["events"]["possibleFallCount"] == 1
assert motion["value"]["events"]["brakingCount"] == 1
assert motion["value"]["events"]["impactCount"] == 1
assert motion["value"]["accelerometer"]["z"] == 9.81
assert sd_motion["type"] == "motion" and sd_motion["value"]["events"] == motion["value"]["events"]
print(result.stderr.strip())
print("7 JSON records validated: 64-bit clocks, offline timestamps, CAN/RTR, raw IMU, escaped GPS, health.")
print("5 live GPS payloads validated: no data, no fix, fresh, stale and genuine zero coordinates.")
print("4 calculated payloads validated: GPS trip, IMU motion and their offline SD records.")

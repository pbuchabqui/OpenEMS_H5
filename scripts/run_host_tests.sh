#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${TMPDIR:-/tmp}/openems_host_tests"
CXX_BIN="${CXX:-g++}"
CXXFLAGS=(-std=c++17 -DEMS_HOST_TEST -Isrc -Wall -Wextra -Werror -Wno-attributes)

mkdir -p "${BUILD_DIR}"

run_test() {
  local name="$1"
  shift
  local exe="${BUILD_DIR}/${name}"
  echo ""
  echo "==> [${name}] build"
  "${CXX_BIN}" "${CXXFLAGS[@]}" "$@" -o "${exe}"
  echo "==> [${name}] run"
  "${exe}"
}

cd "${ROOT_DIR}"

echo "Running all host tests with ${CXX_BIN}"

run_test test_ckp \
  test/drv/test_ckp.cpp \
  src/drv/ckp.cpp

run_test test_scheduler \
  test/drv/test_scheduler.cpp \
  src/drv/scheduler.cpp \
  src/hal/tim.cpp

run_test test_sensors \
  test/drv/test_sensors.cpp \
  src/drv/sensors.cpp \
  src/hal/adc.cpp

run_test test_sensors_validation \
  test/drv/test_sensors_validation.cpp \
  src/drv/sensors.cpp \
  src/hal/adc.cpp

# test_ftm_arithmetic removed - was MK64F/Teensy specific, not ported to STM32H5

run_test test_fuel \
  test/engine/test_fuel.cpp \
  src/engine/fuel_calc.cpp \
  src/engine/table3d.cpp \
  src/hal/flash_nvm.cpp

# test_fuel_calc_assertions removed - file does not exist

run_test test_ign \
  test/engine/test_ign.cpp \
  src/engine/ign_calc.cpp \
  src/engine/table3d.cpp

run_test test_quick_crank \
  test/engine/test_quick_crank.cpp \
  src/engine/quick_crank.cpp

run_test test_auxiliaries \
  test/engine/test_auxiliaries.cpp \
  src/engine/auxiliaries.cpp \
  src/engine/table3d.cpp

run_test test_iacv \
  test/engine/test_iacv.cpp \
  src/engine/auxiliaries.cpp \
  src/engine/table3d.cpp

run_test test_knock \
  test/engine/test_knock.cpp \
  src/engine/knock.cpp \
  src/hal/dac.cpp \
  src/hal/flash_nvm.cpp

run_test test_can \
  test/app/test_can.cpp \
  src/app/can_stack.cpp \
  src/engine/knock.cpp \
  src/hal/dac.cpp \
  src/hal/flash_nvm.cpp \
  src/hal/fdcan.cpp

run_test test_flash_nvm \
  test/hal/test_flash_nvm.cpp \
  src/hal/flash_nvm.cpp

run_test test_cordic \
  test/hal/test_cordic.cpp \
  src/hal/cordic.cpp

run_test test_fuel_boundary \
  test/engine/test_fuel_boundary.cpp \
  src/engine/fuel_calc.cpp \
  src/engine/table3d.cpp \
  src/hal/flash_nvm.cpp

run_test test_scheduler_boundary \
  test/drv/test_scheduler_boundary.cpp \
  src/drv/scheduler.cpp \
  src/hal/tim.cpp

# test_ts_protocol, test_usb_cdc, test_ecu_sched, test_ecu_sched_fixes, test_pipeline_backbone
# removed - depend on stub_ecu_sched_ivc.cpp and engine/ecu_sched.h which don't exist

echo ""
echo "All host tests passed."

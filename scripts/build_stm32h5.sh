#!/usr/bin/env bash
# build_stm32h5.sh — Build completo do firmware OpenEMS para STM32H562RGT6
#
# Gera: build/openems.elf, build/openems.hex, build/openems.bin
# Requer: arm-none-eabi-g++ e arm-none-eabi-objcopy no PATH
#
# Se o cross-compiler não estiver disponível, executa verificação de sintaxe
# com g++ nativo (EMS_HOST_TEST mode) como fallback.
#
# Uso:
#   ./scripts/build_stm32h5.sh              # build completo
#   ARM_CXX=arm-none-eabi-g++ ./scripts/... # especificar toolchain
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
mkdir -p "${BUILD_DIR}"

cd "${ROOT_DIR}"

# ── Arquivos de código C++ do firmware ───────────────────────────────────────
STM32_SRCS=(
    src/main.cpp
    src/hal/system.cpp
    src/hal/tim.cpp
    src/hal/adc.cpp
    src/hal/fdcan.cpp
    src/hal/cordic.cpp
    src/hal/flash_nvm.cpp
    src/hal/usb_cdc.cpp
    src/drv/ckp.cpp
    src/drv/scheduler.cpp
    src/drv/sensors.cpp
    src/engine/fuel_calc.cpp
    src/engine/ign_calc.cpp
    src/engine/table3d.cpp
    src/engine/auxiliaries.cpp
    src/engine/ecu_sched.cpp
    src/engine/cycle_sched.cpp
    src/engine/knock.cpp
    src/engine/quick_crank.cpp
    src/app/tuner_studio.cpp
    src/app/can_stack.cpp
)

# Arquivo de startup assembly (Reset_Handler + tabela de vetores)
STARTUP_SRC="src/hal/startup_stm32h562.s"

# Linker script
LINKER_SCRIPT="src/hal/stm32h562rgt6.ld"

# ── Flags comuns de compilação ────────────────────────────────────────────────
COMMON_FLAGS=(
    -std=c++17
    -DTARGET_STM32H562
    -Isrc
    -Wall
    -Wextra
    -fno-exceptions
    -fno-rtti
    -ffunction-sections
    -fdata-sections
)

# Flags específicos do Cortex-M33 com FPU
ARM_FLAGS=(
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -O2
    -g
)

# ── Tentar cross-compiler ─────────────────────────────────────────────────────
ARM_CXX="${ARM_CXX:-arm-none-eabi-g++}"
ARM_GCC="${ARM_GCC:-arm-none-eabi-gcc}"
ARM_OBJCOPY="${ARM_OBJCOPY:-arm-none-eabi-objcopy}"
ARM_SIZE="${ARM_SIZE:-arm-none-eabi-size}"

if command -v "${ARM_CXX}" &>/dev/null; then
    echo "==> [build_stm32h5] compilando com ${ARM_CXX}"

    # ── 1. Compilar cada arquivo .cpp → .o ───────────────────────────────
    OBJ_FILES=()
    for src in "${STM32_SRCS[@]}"; do
        obj="${BUILD_DIR}/$(basename "${src%.cpp}").o"
        echo "    CC  ${src}"
        "${ARM_CXX}" \
            "${COMMON_FLAGS[@]}" \
            "${ARM_FLAGS[@]}" \
            -c "${src}" -o "${obj}"
        OBJ_FILES+=("${obj}")
    done

    # ── 2. Compilar startup assembly .s → .o ─────────────────────────────
    STARTUP_OBJ="${BUILD_DIR}/startup_stm32h562.o"
    echo "    AS  ${STARTUP_SRC}"
    "${ARM_GCC}" \
        "${ARM_FLAGS[@]}" \
        -c "${STARTUP_SRC}" -o "${STARTUP_OBJ}"
    OBJ_FILES+=("${STARTUP_OBJ}")

    # ── 3. Linkar tudo → .elf ─────────────────────────────────────────────
    ELF="${BUILD_DIR}/openems.elf"
    echo "    LD  ${ELF}"
    "${ARM_CXX}" \
        "${ARM_FLAGS[@]}" \
        -T "${LINKER_SCRIPT}" \
        -Wl,--gc-sections \
        -Wl,-Map="${BUILD_DIR}/openems.map" \
        -Wl,--print-memory-usage \
        -nostartfiles \
        "${OBJ_FILES[@]}" \
        -o "${ELF}"

    # ── 4. Gerar .hex (Intel HEX — para gravação com STM32CubeProgrammer) ─
    HEX="${BUILD_DIR}/openems.hex"
    echo "    HEX ${HEX}"
    "${ARM_OBJCOPY}" -O ihex "${ELF}" "${HEX}"

    # ── 5. Gerar .bin (binário puro — para st-flash) ──────────────────────
    BIN="${BUILD_DIR}/openems.bin"
    echo "    BIN ${BIN}"
    "${ARM_OBJCOPY}" -O binary "${ELF}" "${BIN}"

    # ── 6. Exibir tamanho do firmware ─────────────────────────────────────
    echo ""
    "${ARM_SIZE}" --format=berkeley "${ELF}"
    echo ""

    echo "[PASS] Build completo:"
    echo "       ELF: ${ELF}"
    echo "       HEX: ${HEX}"
    echo "       BIN: ${BIN}"
    echo "       MAP: ${BUILD_DIR}/openems.map"

else
    echo "[INFO] ${ARM_CXX} não encontrado — executando verificação de sintaxe"
    echo "[INFO] Para build completo: instale arm-none-eabi-g++ e reexecute"
    echo ""
    echo "==> [build_stm32h5] verificação de sintaxe com ${CXX:-g++}"

    CXX_BIN="${CXX:-g++}"
    "${CXX_BIN}" \
        -std=c++17 \
        -DEMS_HOST_TEST \
        -Isrc \
        -Wall -Wextra \
        -fsyntax-only \
        "${STM32_SRCS[@]}"

    echo "[PASS] Verificação de sintaxe passou (EMS_HOST_TEST mode)"
    echo "[WARN] Nenhum binário gerado — cross-compiler ausente"
fi

echo ""
echo "build_stm32h5.sh: concluído."

#!/usr/bin/env bash
# flash_stm32h562.sh — Grava o firmware OpenEMS no STM32H562RGT6 via SWD
#
# Tenta automaticamente: OpenOCD → STM32CubeProgrammer CLI → st-flash
# Se nenhum estiver disponível, instrui como instalar.
#
# Uso:
#   ./scripts/flash_stm32h562.sh           # grava build/openems.hex
#   ./scripts/flash_stm32h562.sh --verify  # grava e verifica
#   ./scripts/flash_stm32h562.sh --erase   # apaga toda a Flash antes
#
# Hardware necessário:
#   ST-Link V2 ou V3 conectado via USB ao computador
#   Pinos SWD no STM32H562RGT6:
#     PA13 = SWDIO (SWD Data)
#     PA14 = SWDCLK (SWD Clock)
#     GND, 3.3V
#
# Nota: PA13/PA14 são os pinos SWD default do STM32H5 — não redirecionar
# esses pinos para outras funções no firmware (GPIO AF padrão = SWD).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEX="${ROOT_DIR}/build/openems.hex"
BIN="${ROOT_DIR}/build/openems.bin"

VERIFY=0
ERASE=0
for arg in "$@"; do
    case "${arg}" in
        --verify) VERIFY=1 ;;
        --erase)  ERASE=1  ;;
        *) echo "[WARN] Argumento desconhecido: ${arg}" ;;
    esac
done

# Verificar que o binário existe
if [ ! -f "${HEX}" ]; then
    echo "[ERROR] ${HEX} não encontrado. Execute primeiro:"
    echo "        ./scripts/build_stm32h5.sh"
    exit 1
fi

echo "==> [flash_stm32h562] firmware: ${HEX}"
echo "    tamanho: $(wc -c < "${BIN}" 2>/dev/null || echo 'N/A') bytes (bin)"

# ── Opção A: OpenOCD ─────────────────────────────────────────────────────────
if command -v openocd &>/dev/null; then
    echo "==> Usando OpenOCD"

    ERASE_CMD=""
    if [ "${ERASE}" -eq 1 ]; then
        ERASE_CMD="flash erase_sector 0 0 last;"
    fi

    VERIFY_FLAG=""
    if [ "${VERIFY}" -eq 1 ]; then
        VERIFY_FLAG=" verify"
    fi

    openocd \
        -f "${ROOT_DIR}/scripts/openocd_stm32h562.cfg" \
        -c "program ${HEX}${VERIFY_FLAG} reset exit" \
        2>&1 | tee /tmp/openocd_flash.log

    if grep -q "** Programming Finished **" /tmp/openocd_flash.log; then
        echo "[PASS] Firmware gravado com sucesso (OpenOCD)"
    else
        echo "[FAIL] Verifique /tmp/openocd_flash.log"
        exit 1
    fi

# ── Opção B: STM32CubeProgrammer CLI ────────────────────────────────────────
elif command -v STM32_Programmer_CLI &>/dev/null; then
    echo "==> Usando STM32CubeProgrammer CLI"

    ERASE_OPT=""
    if [ "${ERASE}" -eq 1 ]; then
        ERASE_OPT="-e all"
    fi

    VERIFY_OPT=""
    if [ "${VERIFY}" -eq 1 ]; then
        VERIFY_OPT="-v"
    fi

    STM32_Programmer_CLI \
        -c port=SWD freq=4000 \
        -d "${HEX}" \
        ${ERASE_OPT} \
        ${VERIFY_OPT} \
        -rst

    echo "[PASS] Firmware gravado com sucesso (STM32CubeProgrammer)"

# ── Opção C: st-flash (open source) ─────────────────────────────────────────
elif command -v st-flash &>/dev/null; then
    echo "==> Usando st-flash"

    if [ "${ERASE}" -eq 1 ]; then
        echo "    Apagando Flash..."
        st-flash erase
    fi

    VERIFY_OPT=""
    if [ "${VERIFY}" -eq 1 ]; then
        VERIFY_OPT="--verify"
    fi

    st-flash ${VERIFY_OPT} write "${BIN}" 0x08000000

    echo "[PASS] Firmware gravado com sucesso (st-flash)"

# ── Nenhuma ferramenta disponível ────────────────────────────────────────────
else
    echo "[ERROR] Nenhuma ferramenta de gravação encontrada."
    echo ""
    echo "Instale uma das seguintes opções:"
    echo ""
    echo "  OpenOCD (recomendado):"
    echo "    Ubuntu/Debian: sudo apt install openocd"
    echo "    macOS:         brew install openocd"
    echo ""
    echo "  STM32CubeProgrammer:"
    echo "    Download: https://www.st.com/en/development-tools/stm32cubeprog.html"
    echo ""
    echo "  st-flash (libstlink):"
    echo "    Ubuntu/Debian: sudo apt install stlink-tools"
    echo "    macOS:         brew install stlink"
    echo ""
    echo "Após instalar, reexecute: ./scripts/flash_stm32h562.sh"
    exit 1
fi

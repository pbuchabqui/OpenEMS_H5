# =============================================================================
# Makefile — OpenEMS firmware para STM32H562RGT6 (Cortex-M33 @ 250 MHz)
#
# Uso rápido:
#   make            # compila → build/openems.elf, .hex, .bin
#   make flash      # grava no MCU via ST-Link + OpenOCD
#   make size       # exibe uso de Flash e RAM
#   make clean      # remove build/
#   make debug      # inicia servidor GDB (OpenOCD na porta 3333)
#
# Requer: arm-none-eabi-g++, arm-none-eabi-gcc, arm-none-eabi-objcopy, openocd
# =============================================================================

# ── Toolchain ─────────────────────────────────────────────────────────────────
CXX      := arm-none-eabi-g++
CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
SIZE     := arm-none-eabi-size

# ── Diretórios ────────────────────────────────────────────────────────────────
BUILD_DIR := build
SRC_DIR   := src

# ── Arquivos fonte ────────────────────────────────────────────────────────────
CPP_SRCS := \
    $(SRC_DIR)/main.cpp \
    $(SRC_DIR)/hal/system.cpp \
    $(SRC_DIR)/hal/timer.cpp \
    $(SRC_DIR)/hal/adc.cpp \
    $(SRC_DIR)/hal/can.cpp \
    $(SRC_DIR)/hal/uart.cpp \
    $(SRC_DIR)/hal/flexnvm.cpp \
    $(SRC_DIR)/drv/ckp.cpp \
    $(SRC_DIR)/drv/sensors.cpp \
    $(SRC_DIR)/engine/fuel_calc.cpp \
    $(SRC_DIR)/engine/ign_calc.cpp \
    $(SRC_DIR)/engine/table3d.cpp \
    $(SRC_DIR)/engine/auxiliaries.cpp \
    $(SRC_DIR)/engine/ecu_sched.cpp \
    $(SRC_DIR)/engine/cycle_sched.cpp \
    $(SRC_DIR)/engine/knock.cpp \
    $(SRC_DIR)/engine/quick_crank.cpp \
    $(SRC_DIR)/app/tuner_studio.cpp \
    $(SRC_DIR)/app/can_stack.cpp

ASM_SRCS := $(SRC_DIR)/hal/startup_stm32h562.s

# ── Objetos gerados ───────────────────────────────────────────────────────────
CPP_OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(CPP_SRCS))
ASM_OBJS := $(patsubst $(SRC_DIR)/%.s,   $(BUILD_DIR)/%.o, $(ASM_SRCS))
ALL_OBJS := $(CPP_OBJS) $(ASM_OBJS)

# ── Artefatos finais ──────────────────────────────────────────────────────────
ELF := $(BUILD_DIR)/openems.elf
HEX := $(BUILD_DIR)/openems.hex
BIN := $(BUILD_DIR)/openems.bin
MAP := $(BUILD_DIR)/openems.map

# ── Flags de compilação ───────────────────────────────────────────────────────
ARCH_FLAGS := \
    -mcpu=cortex-m33 \
    -mthumb \
    -mfpu=fpv5-sp-d16 \
    -mfloat-abi=hard

CXXFLAGS := \
    $(ARCH_FLAGS) \
    -std=c++17 \
    -DTARGET_STM32H562 \
    -I$(SRC_DIR) \
    -Wall \
    -Wextra \
    -fno-exceptions \
    -fno-rtti \
    -ffunction-sections \
    -fdata-sections \
    -O2 \
    -g

ASFLAGS := $(ARCH_FLAGS)

# ── Flags de linkagem ─────────────────────────────────────────────────────────
LDSCRIPT := $(SRC_DIR)/hal/stm32h562rgt6.ld

LDFLAGS := \
    $(ARCH_FLAGS) \
    -T $(LDSCRIPT) \
    -Wl,--gc-sections \
    -Wl,-Map=$(MAP) \
    -Wl,--print-memory-usage \
    -nostartfiles

# ── OpenOCD ───────────────────────────────────────────────────────────────────
OPENOCD_CFG := scripts/openocd_stm32h562.cfg

# =============================================================================
# Targets
# =============================================================================

.PHONY: all flash clean size debug

## all: compila o firmware completo (padrão)
all: $(ELF) $(HEX) $(BIN)
	@echo ""
	@$(SIZE) --format=berkeley $(ELF)
	@echo ""
	@echo "[PASS] Build concluído:"
	@echo "       ELF: $(ELF)"
	@echo "       HEX: $(HEX)"
	@echo "       BIN: $(BIN)"
	@echo "       MAP: $(MAP)"

## Linkar → .elf
$(ELF): $(ALL_OBJS)
	@echo "  LD  $@"
	@$(CXX) $(LDFLAGS) $^ -o $@

## Gerar .hex (Intel HEX — para STM32CubeProgrammer / OpenOCD)
$(HEX): $(ELF)
	@echo "  HEX $@"
	@$(OBJCOPY) -O ihex $< $@

## Gerar .bin (binário puro — para st-flash)
$(BIN): $(ELF)
	@echo "  BIN $@"
	@$(OBJCOPY) -O binary $< $@

## Compilar arquivos .cpp → .o
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

## Compilar arquivo .s (startup assembly) → .o
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s
	@mkdir -p $(dir $@)
	@echo "  AS  $<"
	@$(CC) $(ASFLAGS) -c $< -o $@

## flash: grava o firmware no MCU via ST-Link + OpenOCD
flash: $(HEX)
	openocd -f $(OPENOCD_CFG) \
	        -c "program $(HEX) verify reset exit"

## size: exibe uso de Flash e RAM
size: $(ELF)
	$(SIZE) --format=berkeley $(ELF)

## debug: inicia servidor GDB na porta 3333 (Ctrl+C para encerrar)
##        Em outro terminal: arm-none-eabi-gdb build/openems.elf
##                           (gdb) target remote :3333
debug: $(ELF)
	openocd -f $(OPENOCD_CFG)

## clean: remove todos os artefatos de build
clean:
	rm -rf $(BUILD_DIR)
	@echo "  Limpeza concluída."

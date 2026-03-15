# OpenEMS_H5
Firmware ECU open-source para **STM32H562RGT6** (ARM Cortex-M33 @ 250 MHz).

---

## Pré-requisitos

### 1. Instalar o toolchain ARM GCC
```bash
# Ubuntu / Debian
sudo apt update
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi make
```

### 2. Instalar o OpenOCD (para gravar e depurar)
```bash
sudo apt install openocd
```

### 3. Hardware necessário
| Item | Detalhe |
|------|---------|
| **ST-Link V2 ou V3** | Programador USB — conectado ao PC via USB |
| **Conexão SWD** | `PA13` = SWDIO · `PA14` = SWDCLK (pinos 46/49 no LQFP64) |
| **Alimentação** | 3,3 V no VDD do STM32 |

Esquema de conexão ST-Link → STM32:
```
ST-Link        STM32H562RGT6
--------        -------------
SWDIO   ──────  PA13 (pino 46)
SWDCLK  ──────  PA14 (pino 49)
GND     ──────  GND
3.3V    ──────  VDD  (opcional, se o STM32 não tiver alimentação externa)
```

---

## Como compilar

### Passo 1 — Clonar o repositório
```bash
git clone https://github.com/pbuchabqui/OpenEMS_H5.git
cd OpenEMS_H5
```

### Passo 2 — Compilar o firmware
```bash
make
```

O comando compila todos os arquivos e gera os artefatos em `build/`:

| Arquivo | Uso |
|---------|-----|
| `build/openems.elf` | Debug com GDB |
| `build/openems.hex` | Gravação via OpenOCD ou STM32CubeProgrammer |
| `build/openems.bin` | Gravação via `st-flash` |
| `build/openems.map` | Mapa de memória (diagnóstico) |

Ao final, o `make` exibe o uso de memória:
```
   text    data     bss     dec     hex filename
 123456    1234   56789  181479   2c4a7 build/openems.elf
```
- **text** = Flash utilizada (código + dados somente leitura)
- **bss/data** = RAM utilizada

---

## Como gravar no STM32

### Passo 3 — Conectar o ST-Link via USB
Verifique se o ST-Link é reconhecido pelo sistema:
```bash
lsusb | grep STMicro
# Deve aparecer algo como:
# Bus 001 Device 003: ID 0483:374b STMicroelectronics ST-LINK/V2.1
```

### Passo 4 — Gravar o firmware
```bash
make flash
```

O comando executa o OpenOCD, grava o `.hex`, verifica a gravação e reinicia o MCU.
Saída esperada:
```
** Programming Started **
** Programming Finished **
** Verify Started **
** Verified OK **
** Resetting Target **
```

---

## Outros comandos úteis

```bash
make clean    # Remove build/ e todos os arquivos compilados
make size     # Exibe uso de Flash e RAM sem recompilar
make debug    # Inicia servidor GDB via OpenOCD (porta 3333)
```

### Depuração com GDB
Em um terminal, inicie o servidor:
```bash
make debug
```

Em outro terminal, conecte o GDB:
```bash
arm-none-eabi-gdb build/openems.elf \
    -ex "target remote :3333" \
    -ex "monitor reset halt"
```

---

## Alternativa: STM32CubeProgrammer (GUI)

Se preferir uma interface gráfica:
1. Baixe o [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. Conecte o ST-Link
3. Abra `build/openems.hex`
4. Clique em **Download**

---

## Estrutura do projeto

```
OpenEMS_H5/
├── Makefile                        # Build system
├── scripts/
│   ├── build_stm32h5.sh            # Script bash alternativo
│   └── openocd_stm32h562.cfg       # Configuração OpenOCD
└── src/
    ├── main.cpp                    # Ponto de entrada
    ├── hal/                        # Hardware Abstraction Layer
    │   ├── system.cpp              # Clock PLL 250 MHz, SysTick, IWDG
    │   ├── timer.cpp               # TIM1/3/4/5/6
    │   ├── adc.cpp                 # ADC1 + ADC2
    │   ├── can.cpp                 # FDCAN1 (CAN FD)
    │   ├── uart.cpp                # USART1
    │   ├── flexnvm.cpp             # EEPROM emulada (Flash Bank2)
    │   ├── regs.h                  # Registradores STM32H562
    │   ├── startup_stm32h562.s     # Vetor de interrupções + Reset_Handler
    │   └── stm32h562rgt6.ld        # Linker script
    ├── drv/                        # Drivers de periféricos
    │   ├── ckp.cpp                 # Sensor de posição do virabrequim
    │   └── sensors.cpp             # Processamento de sensores
    ├── engine/                     # Lógica de controle do motor
    │   ├── ecu_sched.cpp           # Scheduler principal da ECU
    │   ├── fuel_calc.cpp           # Cálculo de injeção
    │   ├── ign_calc.cpp            # Cálculo de ignição
    │   └── ...
    └── app/                        # Aplicação
        ├── tuner_studio.cpp        # Protocolo TunerStudio
        └── can_stack.cpp           # Stack de mensagens CAN
```

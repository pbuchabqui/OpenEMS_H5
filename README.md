# OpenEMS_H5

![Platform](https://img.shields.io/badge/platform-STM32H5-blue)
![Language](https://img.shields.io/badge/language-C++17-green)
![Architecture](https://img.shields.io/badge/architecture-OpenEMS%20v2.2-orange)

Firmware ECU para **STM32H562RGT6** seguindo a arquitetura **OpenEMS v2.2**.

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

### Opção 1: Make (Recomendado)

#### Passo 1 — Clonar o repositório
```bash
git clone https://github.com/pbuchabqui/OpenEMS_H5.git
cd OpenEMS_H5
```

#### Passo 2 — Compilar o firmware
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

### Opção 2: PlatformIO

Se você já usa VS Code com PlatformIO:

```bash
# Compilar
pio run

# Gravar no MCU
pio run -t upload

# Monitor serial
pio device monitor -b 115200
```

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

### Gravação alternativa via script
```bash
./scripts/flash_stm32h562.sh
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

Comandos GDB úteis:
```gdb
break main              # Breakpoint no main
continue                # Continuar execução
print variable_name     # Imprimir variável
info registers          # Mostrar registradores
step                    # Passo a passo (dentro de funções)
next                    # Próxima linha
backtrace               # Mostrar call stack
```

---

## TunerStudio

O firmware suporta comunicação com TunerStudio para calibração e monitoramento em tempo real.

### Configuração
1. Abra o TunerStudio
2. Crie um novo projeto: **File → New Project**
3. Selecione **Custom / Other** como tipo
4. Carregue o arquivo de definição: `tunerstudio/openems.ini`
5. Configure a porta serial: `/dev/ttyACM0` (Linux) ou `COMx` (Windows)

### Canais disponíveis
| Canal | Descrição | Unidade |
|-------|-----------|---------|
| `rpm` | Rotação do motor | RPM |
| `map_kpa` | Pressão do coletor | kPa |
| `tps_pct` | Posição do acelerador | % |
| `clt_c` | Temperatura do líquido de arrefecimento | °C |
| `iat_c` | Temperatura do ar de admissão | °C |
| `o2_mv` | Sonda Lambda | mV |
| `advance` | Avanço de ignição | graus |
| `ve_cell` | Eficiência volumétrica | % |
| `status` | Bits de status | - |

---

## Alternativa: STM32CubeProgrammer (GUI)

Se preferir uma interface gráfica:
1. Baixe o [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. Conecte o ST-Link
3. Abra `build/openems.hex`
4. Clique em **Download**

---

## Arquitetura v2.2

- `TIM2` 32-bit: captura `CKP/CMP` em `PA0/PA1`
- `TIM5` 32-bit: base absoluta do scheduler
- `TIM1`: `INJ1-3` em `PA8/PA9/PA10`
- `TIM15`: `INJ4` em `PC12`
- `TIM8`: `IGN1-4` em `PC6/PC7/PC8/PC9`
- `TIM3`: PWM de `IACV/Wastegate` em `PB0/PB1`
- `TIM12`: PWM de `VVT` em `PB14/PB15`
- `FDCAN1`: `PB7/PB8`
- `USB CDC ACM`: `PA11/PA12`
- `CORDIC`: usado por `hal/cordic`

O backend de produção de `USB CDC ACM` ainda precisa ser implementado sobre o periférico USB do STM32H5. O transporte serial falso foi removido de propósito.

## Estrutura do projeto

```
OpenEMS_H5/
├── Makefile                        # Build system
├── platformio.ini                  # Configuração PlatformIO
├── scripts/
│   ├── build_stm32h5.sh            # Script bash alternativo
│   ├── flash_stm32h562.sh          # Script de gravação
│   ├── openocd_stm32h562.cfg       # Configuração OpenOCD
│   └── run_host_tests.sh           # Script para rodar testes
├── src/
│   ├── main.cpp                    # Ponto de entrada
│   ├── hal/                        # Hardware Abstraction Layer
│   │   ├── system.cpp              # Clock PLL 250 MHz, SysTick, IWDG
│   │   ├── tim.cpp                 # TIM2/TIM5/TIM1/TIM8/TIM15/TIM3/TIM12
│   │   ├── adc.cpp                 # ADC1 + ADC2
│   │   ├── fdcan.cpp               # FDCAN1 (CAN FD)
│   │   ├── usb_cdc.cpp             # USB CDC ACM (target backend pendente)
│   │   ├── cordic.cpp              # CORDIC hardware / referência host
│   │   ├── flash_nvm.cpp           # EEPROM emulada (Flash Bank2)
│   │   ├── regs.h                  # Registradores STM32H562
│   │   ├── startup_stm32h562.s     # Vetor de interrupções + Reset_Handler
│   │   └── stm32h562rgt6.ld        # Linker script
│   ├── drv/                        # Drivers de periféricos
│   │   ├── ckp.cpp                 # Sensor de posição do virabrequim
│   │   ├── scheduler.cpp           # Scheduler absoluto v2.2
│   │   └── sensors.cpp             # Processamento de sensores
│   ├── engine/                     # Lógica de controle do motor
│   │   ├── fuel_calc.cpp           # Cálculo de injeção
│   │   ├── ign_calc.cpp            # Cálculo de ignição
│   │   ├── knock.cpp               # Detecção de knock
│   │   ├── quick_crank.cpp         # Quick crank para partida rápida
│   │   ├── table3d.cpp             # Tabelas 3D para calibração
│   │   └── auxiliaries.cpp         # Controle de auxiliares (IACV, VVT, etc.)
│   └── app/                        # Aplicação
│       ├── tuner_studio.cpp        # Protocolo TunerStudio
│       └── can_stack.cpp           # Stack de mensagens CAN
├── test/                           # Testes unitários
│   ├── app/
│   │   ├── test_can.cpp
│   │   └── test_ts_protocol.cpp
│   ├── drv/
│   │   ├── test_ckp.cpp
│   │   ├── test_scheduler.cpp
│   │   ├── test_sensors.cpp
│   │   └── test_sensors_validation.cpp
│   ├── engine/
│   │   ├── test_auxiliaries.cpp
│   │   ├── test_fuel.cpp
│   │   ├── test_iacv.cpp
│   │   ├── test_ign.cpp
│   │   ├── test_knock.cpp
│   │   └── test_quick_crank.cpp
│   └── hal/
│       ├── test_cordic.cpp
│       ├── test_flash_nvm.cpp
│       ├── test_tim_32bit.cpp
│       └── test_usb_cdc.cpp
└── tunerstudio/
    └── openems.ini                 # Definição TunerStudio
```

---

## Testes

O projeto inclui testes unitários para validação das funcionalidades.

### Rodar testes no host (sem hardware)
```bash
./scripts/run_host_tests.sh
```

Os testes cobrem:
- **App**: Protocolo CAN e TunerStudio
- **Drivers**: CKP, Scheduler, Validação de sensores
- **Engine**: Cálculos de combustível, ignição, knock, auxiliares
- **HAL**: CORDIC, Flash NVM, TIM 32-bit, USB CDC

---

## Contribuição

Contribuições são bem-vindas! Para contribuir:

1. Fork o repositório
2. Crie uma branch para sua feature: `git checkout -b feature/nova-feature`
3. Commit suas mudanças: `git commit -m 'Add nova feature'`
4. Push para a branch: `git push origin feature/nova-feature`
5. Abra um Pull Request

### Padrões de código
- C++17 padrão
- Sem exceptions (`-fno-exceptions`)
- Sem RTTI (`-fno-rtti`)
- Warnings habilitados (`-Wall -Wextra`)

---

## Licença

Este projeto é de código aberto. Consulte o repositório para informações sobre licenciamento.

---

## Links úteis

- [STM32H562RG Datasheet](https://www.st.com/resource/en/datasheet/stm32h562rg.pdf)
- [STM32H5 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0481-stm32h5-series-armbased-32bit-mcus-stmicroelectronics.pdf)
- [OpenOCD Documentation](https://openocd.org/doc/html/index.html)
- [PlatformIO Documentation](https://docs.platformio.org/)
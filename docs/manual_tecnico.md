# Manual Tecnico — OpenEMS H5

**Plataforma:** STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz, LQFP64)
**Motor-alvo:** 4 cilindros, ciclo Otto, roda fonica 60-2
**Versao do firmware:** OpenEMS_v1.1

---

## Sumario

1. [Pipeline Geral](#1-pipeline-geral)
2. [Power-On e Reset Handler](#2-power-on-e-reset-handler)
3. [Inicializacao do Sistema](#3-inicializacao-do-sistema)
4. [Inicializacao dos Perifericos](#4-inicializacao-dos-perifericos)
5. [Sincronizacao CKP (60-2)](#5-sincronizacao-ckp-60-2)
6. [Loop Principal e Slots de Tempo](#6-loop-principal-e-slots-de-tempo)
7. [Pipeline de Calculo de Injecao](#7-pipeline-de-calculo-de-injecao)
8. [Pipeline de Calculo de Ignicao](#8-pipeline-de-calculo-de-ignicao)
9. [Scheduling no Dominio Angular](#9-scheduling-no-dominio-angular)
10. [Controles Auxiliares](#10-controles-auxiliares)
11. [Comunicacao](#11-comunicacao)
12. [Persistencia em Flash](#12-persistencia-em-flash)
13. [Protecoes e Seguranca](#13-protecoes-e-seguranca)
14. [Pinagem Completa do MCU](#14-pinagem-completa-do-mcu)

---

## 1. Pipeline Geral

O firmware segue um pipeline deterministic com dois dominios de execucao:

```
                        DOMINIO DE INTERRUPCAO (ISR)
                        ============================
  Dente CKP ──► TIM5_IRQ (prio 1) ──► ckp_decoder ──► ecu_sched_on_tooth_hook
                                            │                    │
                                            ▼                    ▼
                                       rpm_calc            arma TIM1 OC
                                       sensors_on_tooth    (ignicao/injecao)
                                            │
                                            ▼
                                  TIM1_CC_IRQ (prio 4) ──► disparo de bobina
                                                           abertura/fechamento
                                                           de injetores


                        DOMINIO DE BACKGROUND (main loop)
                        ==================================
  millis() ──► 2ms:  fuel_calc + ign_calc + commit_calibration
           ──► 10ms: auxiliaries_tick + can_stack_process
           ──► 20ms: tuner_studio_service
           ──► 50ms: sensors_tick_50ms
           ──► 100ms: sensors_tick_100ms + STFT + runtime_seed
           ──► 500ms: flash_flush (calibracao + LTFT)
```

**Hierarquia de prioridades NVIC:**

| Prioridade | IRQ | Periferico | Funcao |
|------------|-----|-----------|--------|
| 1 (maxima) | IRQ 48 | TIM5 | Captura CKP (dentes da roda fonica) |
| 4 | IRQ 44 | TIM1_CC | Output Compare (ignicao e injecao) |
| 5 | IRQ 37 | ADC1/ADC2 | Conversao analogica (EOC/EOS) |
| 11 | — | SysTick | Temporizacao millis()/micros() |

---

## 2. Power-On e Reset Handler

**Arquivo:** `src/hal/startup_stm32h562.s`

### 2.1 Tabela de Vetores

A tabela de vetores de interrupcao esta na secao `.isr_vector`, alinhada em 512 bytes no inicio da Flash (0x08000000).

```
Posicao 0:  _estack (0x20040000)     — ponteiro de stack inicial
Posicao 1:  Reset_Handler            — ponto de entrada apos reset
Posicao 2:  NMI_Handler
Posicao 3:  HardFault_Handler
Posicao 4:  MemManage_Handler        — falha de MPU
Posicao 5:  BusFault_Handler
Posicao 6:  UsageFault_Handler
Posicao 7:  SecureFault_Handler      — TrustZone (Cortex-M33)
Posicao 11: SVC_Handler
Posicao 14: PendSV_Handler
Posicao 15: SysTick_Handler          — implementado em system.cpp
```

ISRs criticos do STM32H562:

| Posicao | IRQ# | Handler | Uso |
|---------|------|---------|-----|
| 53 | 37 | ADC1_2_IRQHandler | Conversao ADC1/ADC2 |
| 60 | 44 | TIM1_CC_IRQHandler | Ignicao e injecao |
| 64 | 48 | TIM5_IRQHandler | Captura CKP |
| 76 | 60 | USART3_IRQHandler | UART TunerStudio |

Todos os handlers nao implementados apontam para `Default_Handler` (loop infinito), permitindo que o IWDG (100 ms) reinicie o sistema.

### 2.2 Sequencia do Reset_Handler

```
1. Zerar BSS (.bss)
   ├── r0 = _sbss, r1 = _ebss
   └── loop: str 0, [r0], #4 ate r0 >= r1

2. Copiar .data (Flash → SRAM)
   ├── r0 = _sidata (LMA na Flash)
   ├── r1 = _sdata  (VMA na SRAM)
   ├── r2 = _edata
   └── loop: ldr r3, [r0], #4 → str r3, [r1], #4

3. Chamar main()
   └── bl main
       b .  (loop infinito se main retornar)
```

### 2.3 Mapa de Memoria

```
Flash Bank1:  0x08000000 – 0x0807FFFF  (512 KB) — firmware
Flash Bank2:  0x08100000 – 0x0817FFFF  (512 KB) — NVM (LTFT, calibracao, seed)
SRAM1 (AXI): 0x20000000 – 0x2003FFFF  (256 KB) — dados + heap + stack

Stack: cresce para baixo a partir de 0x20040000
Heap minimo: 4 KB
Stack minimo: 8 KB
```

---

## 3. Inicializacao do Sistema

**Arquivo:** `src/hal/system.cpp`
**Funcao:** `system_stm32_init()`

### 3.1 Arvore de Clocks (PLL)

```
HSE (cristal 8 MHz)
 │
 ▼
PLL1 (DIVM1=1, DIVN1=125, VCO=1000 MHz)
 ├── PLL1P (/4) = 250 MHz ──► SYSCLK ──► HCLK (AHB) = 250 MHz
 │                                         ├── APB1 (/2) = 125 MHz
 │                                         │   └── Timer clock = 250 MHz (doubler)
 │                                         └── APB2 (/2) = 125 MHz
 │                                             └── Timer clock = 250 MHz (doubler)
 └── PLL1Q (/16) = 62.5 MHz ──► FDCAN1 clock
```

**Flash latency:** 5 Wait States + Prefetch + I-Cache + D-Cache habilitados.

### 3.2 SysTick

- **Reload:** 249.999 (250.000 ciclos @ 250 MHz = 1 ms)
- **Prioridade:** 11 (mais baixa que todos os perifericos)
- **ISR:** `SysTick_Handler` incrementa `g_systick_ms`

**millis():** Retorna `g_systick_ms` (volatile uint32_t).

**micros():** Leitura atomica de `g_systick_ms` + SysTick CVR (Current Value Register):
```
us = ms * 1000 + (249999 - CVR) / 250
```

### 3.3 IWDG (Independent Watchdog)

- **Clock:** LSI 32 kHz
- **Prescaler:** /32 → 1000 Hz
- **Reload boot:** 2999 → timeout 3 s (durante init)
- **Reload operacional:** 99 → timeout ~100 ms

`iwdg_kick()` escreve 0xAAAA no IWDG_KR (refresh).

### 3.4 MPU (Memory Protection Unit)

| Regiao | Base | Limite | Permissao | Uso |
|--------|------|--------|-----------|-----|
| 0 | 0x20000000 | 0x200000FF | NO_ACCESS, XN | Stack guard (256 bytes) |
| 1 | 0x08000000 | 0x0807FFFF | RO priv, exec | Flash Bank1 (firmware) |
| 2 | 0x40000000 | 0x5FFFFFFF | RW priv, XN | Perifericos (Device, nGnRE) |

---

## 4. Inicializacao dos Perifericos

**Funcao:** `openems_init()` em `src/main.cpp`

### 4.1 Timers

**Arquivo:** `src/hal/timer.cpp`

Todos os timers operam a 250 MHz com prescaler /4 → **62.5 MHz (16 ns/tick)**.

#### TIM5 — Captura CKP (equivalente ao FTM3 Kinetis)

| Parametro | Valor |
|-----------|-------|
| Modo | Input Capture, 32-bit free-running |
| ARR | 0xFFFFFFFF (sem overflow) |
| CH1 | TI1 (PA0) — borda de subida — sinal CKP |
| CH2 | TI2 (PA1) — borda de subida — sinal CMP (came) |
| Filtro | Nenhum (IC1F = 0, IC2F = 0) |
| Interrupcao | CC1IE + CC2IE habilitadas |
| Prioridade | 1 (maxima) |

**ISR `TIM5_IRQHandler`:**
- CC1IF → chama `ckp_ftm3_ch0_isr()` (dente CKP)
- CC2IF → chama `ckp_ftm3_ch1_isr()` (came/fase)

#### TIM1 — Output Compare Ignicao (equivalente ao FTM0)

| Parametro | Valor |
|-----------|-------|
| Modo | Output Compare, 16-bit free-running |
| ARR | 0xFFFF |
| CH1-CH4 | "Inactive on match" (OC mode = 010) |
| BDTR | MOE = 1 (Main Output Enable) |
| Interrupcao | CC1IE-CC4IE habilitadas |
| Prioridade | 4 |

**Mapeamento de canais:**

| Canal TIM1 | Pino | Funcao | Canal FTM0 (Kinetis) |
|------------|------|--------|---------------------|
| CH1 | PA8 | IGN4 | FTM0_CH4 |
| CH2 | PA9 | IGN3 | FTM0_CH5 |
| CH3 | PA10 | IGN2 | FTM0_CH6 |
| CH4 | PA11 | IGN1 | FTM0_CH7 |

**ISR `TIM1_CC_IRQHandler`:** Delega para `FTM0_IRQHandler()` em `ecu_sched.cpp`.

#### TIM3 — PWM IACV e Wastegate (equivalente ao FTM1)

| Canal | Pino | Funcao | Frequencia |
|-------|------|--------|-----------|
| CH1 | PA6 | IACV (valvula de marcha lenta) | 125 Hz |
| CH2 | PA7 | Wastegate (solenoide) | 125 Hz |

Modo PWM1 com preload. Duty: `CCR = (ARR+1) * duty_pct_x10 / 1000`.

#### TIM4 — PWM VVT (equivalente ao FTM2)

| Canal | Pino | Funcao | Frequencia |
|-------|------|--------|-----------|
| CH1 | PB6 | VVT Escape | 150 Hz |
| CH2 | PB7 | VVT Admissao | 150 Hz |

#### TIM6 — Trigger ADC (timer basico)

- **MMS:** Update Event → TRGO (dispara ADC1 + ADC2)
- **ARR padrao:** 249.999 (~1 ms)
- **Dinamico:** `adc_pdb_on_tooth()` ajusta ARR para amostrar na metade do periodo do dente

### 4.2 ADC

**Arquivo:** `src/hal/adc.cpp`

**Clock ADC:** HCLK/4 = 62.5 MHz
**Resolucao:** 12 bits
**Amostragem:** 47.5 ciclos (~0.95 us/canal)
**Trigger:** TIM6 TRGO (borda de subida)

#### ADC1 — 8 canais (sensores rapidos)

| Sequencia | Canal | Pino | Sensor |
|-----------|-------|------|--------|
| SQ1 | IN3 | PA2 | MAP (pressao do coletor) |
| SQ2 | IN4 | PA3 | MAF (fluxo de ar) |
| SQ3 | IN5 | PA4 | TPS (posicao borboleta) |
| SQ4 | IN6 | PA5 | O2 (sonda lambda — reservado) |
| SQ5 | IN7 | PB0 | AN1 (expansao) |
| SQ6 | IN8 | PB1 | AN2 (expansao) |
| SQ7 | IN9 | PC0 | AN3 (expansao) |
| SQ8 | IN10 | PC1 | AN4 (expansao) |

#### ADC2 — 4 canais (sensores lentos)

| Sequencia | Canal | Pino | Sensor |
|-----------|-------|------|--------|
| SQ1 | IN1 | PC2 | CLT (temperatura do liquido) |
| SQ2 | IN2 | PC3 | IAT (temperatura do ar) |
| SQ3 | IN13 | PC4 | Pressao de combustivel |
| SQ4 | IN14 | PC5 | Pressao de oleo |

#### ISR `ADC1_2_IRQHandler`

```
1. EOC (End of Conversion):
   ├── ADC1: g_adc1_raw[idx++] = ADC1_DR & 0xFFF
   └── ADC2: g_adc2_raw[idx++] = ADC2_DR & 0xFFF

2. EOS (End of Sequence):
   ├── Reset idx = 0
   ├── Limpa flag EOS (W1C)
   └── Re-arma: ADC_CR |= ADSTART
```

### 4.3 CAN (FDCAN1)

**Arquivo:** `src/hal/can.cpp`

**Bitrate:** 500 kbps
**Clock:** PLL1Q = 62.5 MHz

```
Prescaler = 5 → Tq = 80 ns
NTSEG1 = 18 Tq, NTSEG2 = 6 Tq, SJW = 4 Tq
Total = 1 + 18 + 6 = 25 Tq = 2000 ns → 500 kbps
Ponto de amostragem = 76%
```

**Filtro de RX:** 1 filtro standard ID, match exato em 0x180 (WBO2).

**Pinos:** PB8 (RX), PB9 (TX) — AF9.

**FIFO RX:** 3 elementos no Message RAM + FIFO circular de software (4 frames).

### 4.4 UART (USART3)

**Arquivo:** `src/hal/uart.cpp`

| Parametro | Valor |
|-----------|-------|
| Baud rate | 115.200 bps |
| Clock APB1 | 125 MHz |
| BRR | 1085 (erro < 0.01%) |
| Stop bits | 1 |
| Paridade | Nenhuma |
| Pinos | PB10 (TX), PB11 (RX) — AF7 |

**RX:** Buffer circular de 128 bytes, polling via `uart0_poll_rx()`.
**TX:** Polling com timeout (100.000 ciclos).

### 4.5 Sequencia Completa de Init

```
1.  system_stm32_init()     PLL 250 MHz + SysTick 1ms + IWDG 3s
2.  ftm0_init()             TIM1 Output Compare (ignicao)
3.  ftm3_init()             TIM5 Input Capture (CKP + CMP)
4.  ftm1_pwm_init(125)      TIM3 PWM @ 125 Hz (IACV + wastegate)
5.  ftm2_pwm_init(150)      TIM4 PWM @ 150 Hz (VVT)
6.  ECU_Hardware_Init()     Scheduler unificado
7.  adc_init()              ADC1/ADC2 + TIM6
8.  can0_init()             FDCAN1 @ 500 kbps
9.  uart0_init()            USART3 @ 115200
10. nvm_load_calibration()  Carrega page-0 da Flash Bank2
11. nvm_load_runtime_seed() Tenta re-sync rapido (se seed valido)
12. sensors_init()          Driver de sensores
13. fuel_reset_adaptives()  Zera adaptativos de combustivel
14. auxiliaries_init()      IACV, VVT, wastegate
15. knock_init()            Detonacao
16. quick_crank_reset()     Modo cranking
17. cycle_sched_init()      Scheduler de ciclo
18. ts_init()               Protocolo TunerStudio
19. can_stack_init()        Stack CAN (WBO2 ID = 0x180)
20. NVIC enable TIM5+TIM1  Habilita interrupcoes criticas
21. IWDG reload → 99       Reduz timeout para 100 ms operacional
22. Aguarda FULL_SYNC CKP  Timeout 5 s com watchdog kick
```

---

## 5. Sincronizacao CKP (60-2)

**Arquivo:** `src/drv/ckp.cpp`

### 5.1 Roda Fonica 60-2

```
Roda fonica: 60 posicoes angulares, 2 dentes ausentes (gap)
Espacamento: 360 / 60 = 6 graus por posicao
Dentes reais: 58 dentes presentes fisicamente
Gap: ~3x o periodo normal de um dente (18 graus)
```

### 5.2 Maquina de Estados

```
                    ┌─────────────┐
                    │  WAIT_GAP   │◄─── Power-On
                    └──────┬──────┘
                           │ gap detectado
                           ▼
                    ┌─────────────┐
                    │  HALF_SYNC  │ contando dentes ate proximo gap
                    └──────┬──────┘
                           │ gap no dente esperado (58)
                           ▼
                    ┌─────────────┐
                    │  FULL_SYNC  │◄─── operacao normal
                    └──────┬──────┘
                           │ gap ausente >61 dentes
                           │ OU gap prematuro
                           ▼
                    ┌──────────────────┐
                    │  LOSS_OF_SYNC    │──► volta para WAIT_GAP
                    └──────────────────┘
```

### 5.3 Deteccao do Gap

**Teste de razao:** `periodo_atual * 2 > media * 3` → gap detectado (ratio > 1.5x)

O gap real tem ratio ~3.0, dando margem de 100%.

**Filtro de ruido:** Periodos fora da banda ±20% da media movel (3 dentes) sao descartados sem atualizar o historico.

**Contagem minima:** `kGapThresholdTooth = 55` — rejeita gaps espurios causados por EMC antes de acumular dentes suficientes.

### 5.4 Calculo de RPM

```
rpm_x10 = 10.000.000.000 / tooth_period_ns
```

Derivacao: 60 posicoes/rev × 10 (escala x10) × 10^9 ns/s / periodo_ns.

Exemplo: periodo = 1.666.667 ns → rpm_x10 = 6000 → 600.0 RPM.

### 5.5 Deteccao de Fase (Came)

O sinal do sensor de came (CMP) entra em TIM5_CH2 (PA1).

Cada borda de subida alterna `phase_A` (true/false), identificando qual metade do ciclo de 4 tempos esta em execucao: cilindros 1,4 vs 2,3.

### 5.6 Snapshot Atomico

```c
struct CkpSnapshot {
    uint32_t tooth_period_ns;   // periodo do ultimo dente
    uint16_t tooth_index;       // 0-57 (desde o ultimo gap)
    uint32_t last_ftm3_capture; // timestamp TIM5 (32-bit)
    uint32_t rpm_x10;           // RPM x 10
    SyncState state;            // WAIT_GAP / HALF_SYNC / FULL_SYNC / LOSS
    bool phase_A;               // fase do came
};
```

`ckp_snapshot()` usa CPSID/CPSIE (desabilita/habilita interrupcoes) para garantir leitura atomica — todos os campos pertencem ao mesmo instante.

### 5.7 Re-Sync Rapido (Runtime Seed)

No desligamento, o firmware salva um `RuntimeSyncSeed` em Flash com:
- `tooth_index` da ultima posicao conhecida
- `phase_A` da ultima fase
- `decoder_tag = 0x3C02` (identificador 60-2)
- CRC-32 para validacao

No proximo boot, se o seed e valido:
1. `ckp_seed_arm(phase_A)` carrega a fase alvo
2. No primeiro gap detectado, pula direto para FULL_SYNC (sem esperar HALF_SYNC)
3. Janela de 2000 ms para confirmar via sinal de came
4. Se nao confirmado: retorna ao caminho normal (HALF_SYNC)

---

## 6. Loop Principal e Slots de Tempo

**Arquivo:** `src/main.cpp`, funcao `main()`

```c
for (;;) {
    iwdg_kick();                   // primeiro statement — alimenta watchdog
    g_datalog_us = micros();       // timestamp para datalog
    const uint32_t now = millis();

    // slots de tempo com elapsed()
    if (elapsed(now, g_t2ms_,   2))   { ... }
    if (elapsed(now, g_t10ms_,  10))  { ... }
    if (elapsed(now, g_t20ms_,  20))  { ... }
    if (elapsed(now, g_t50ms_,  50))  { ... }
    if (elapsed(now, g_t100ms_, 100)) { ... }
    if (elapsed(now, g_t500ms_, 500)) { ... }
}
```

### Slot 2 ms — Calculo de Combustivel e Ignicao

1. Captura snapshot CKP + sensores
2. Gerencia janela do runtime seed (expira apos 2000 ms)
3. Armazena snapshots de FULL_SYNC e HALF_SYNC para persistencia
4. **Se sincronizado e sem rev-cut:**
   - Calcula MAP, VE, PW base, correcoes CLT/IAT/Vbatt
   - Calcula PW final (base * correcoes + dead time)
   - Calcula avanco de ignicao (tabela + correcoes)
   - Calcula dwell (tempo de carga da bobina)
   - **Commit para o scheduler angular:** `ecu_sched_commit_calibration(advance, dwell_ticks, inj_pw_ticks, soi_lead)`
5. Atualiza estado de limp mode (MAP fault OU CLT fault)

### Slot 10 ms — Auxiliares e CAN

1. `auxiliaries_tick_10ms()` — PID de IACV, VVT, wastegate
2. Calcula STFT percentual
3. Monta status bits
4. `can_stack_process()` — processa RX CAN + TX 0x400 (10 ms) + TX 0x401 (100 ms)

### Slot 20 ms — TunerStudio

1. `ts_service()`:
   - `uart0_poll_rx(64)` — le ate 64 bytes do buffer RX
   - `ts_process()` — processa comandos do protocolo
   - TX ate 96 bytes para a UART

2. `auxiliaries_tick_20ms()` — tarefas auxiliares adicionais

### Slot 50 ms — Sensores Lentos

1. `sensors_tick_50ms()` — pressao de combustivel, pressao de oleo (media de 4)
2. Atualiza `g_datalog_us`

### Slot 100 ms — Sensores + STFT + Runtime Seed

1. `sensors_tick_100ms()` — CLT, IAT (media de 8), AN1-AN4
2. **STFT (Short-Term Fuel Trim):**
   - Lambda medido via `can_stack_lambda_milli_safe(now)` (WBO2 CAN)
   - Lambda alvo = 1000 (estequiometrico)
   - Valido se `can_stack_wbo2_fresh(now)` = true (frame < 500 ms)
3. **Runtime Seed — salva posicao para re-sync rapido:**
   - Se RPM cai para 0 e motor estava rodando:
     - Espera 100 ms apos RPM=0
     - Salva seed com tooth_index, phase_A, decoder_tag
     - `nvm_save_runtime_seed(&seed)`

### Slot 500 ms — Flush Flash

1. Se `g_calib_dirty`: grava calibracao page-0 (512 bytes) na Flash Bank2
2. LTFT e knock maps sao persistidos diretamente por `nvm_write_ltft()` e `nvm_write_knock()`

---

## 7. Pipeline de Calculo de Injecao

**Arquivo:** `src/engine/fuel_calc.cpp`

### 7.1 Formula Base

```
PW_base = REQ_FUEL × (VE / 100) × (MAP / MAP_REF)
```

| Parametro | Valor padrao | Descricao |
|-----------|-------------|-----------|
| REQ_FUEL | 8000 us | Fluxo de referencia do injetor |
| MAP_REF | 100 kPa | Pressao de referencia (1 atm) |
| VE | 50-110% | Eficiencia volumetrica (tabela 16x16) |

### 7.2 Tabela VE (Volumetric Efficiency)

Grade 16x16 com eixos:
- **RPM (x10):** 500, 750, 1000, 1250, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 12000
- **Carga (kPa):** 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 175, 200

Interpolacao bilinear com aritmetica Q8 (fracao de 256).

### 7.3 Correcao por Temperatura do Liquido (CLT)

```
Eixo CLT (C):    -40   -10    0    20    40    70    90   110
Fator (x256):    384   352   320   288   272   256   248   240
Fator real:      1.50  1.38  1.25  1.13  1.06  1.00  0.97  0.94
```

Motor frio → enriquecimento (ate 50% a mais).
Motor quente → empobrecimento leve (6% a menos).

### 7.4 Correcao por Temperatura do Ar (IAT)

```
Eixo IAT (C):    -20    0    20    40    60    80   100   120
Fator (x256):    272   264   256   248   240   232   224   216
Fator real:      1.06  1.03  1.00  0.97  0.94  0.91  0.88  0.84
```

Ar frio → enriquecimento (mais denso). Ar quente → empobrecimento.

### 7.5 Correcao por Tensao da Bateria (Dead Time)

```
Tensao (mV):   9000  10000  11000  12000  13000  14000  15000  16000
Dead time (us): 1400   1200   1050    900    800    700    650    600
```

Tensao baixa → tempo de abertura do injetor maior (solenoide mais lento).

### 7.6 Calculo Final

```
PW_final = (PW_base × corr_clt × corr_iat) >> 16  +  dead_time_us
```

A divisao por 65536 (>>16) compensa os dois fatores x256 multiplicados entre si.

### 7.7 Enriquecimento por Aceleracao (AE)

```
1. Calcula derivada do TPS: tpsdot = delta_tps / dt_ms
2. Se tpsdot > threshold (padrao 5 %/ms):
     ae_pw = tpsdot × sensibilidade[bucket_CLT]
3. Decay linear ao longo de N ciclos (taper)
```

Sensibilidade por temperatura: `{11, 10, 9, 8, 7, 6, 5, 4}` us por %TPS/ms.

### 7.8 STFT (Short-Term Fuel Trim)

Controlador PI que ajusta a mistura com base no lambda medido:

```
erro = lambda_alvo - lambda_medido  (em millesimos)

P = erro × 3 / 100       → 0.03 por unidade de erro
I += erro × 1 / 200      → 0.005 por amostra

Clamp: ±25 pontos (±2.5%)
```

**Condicoes de ativacao:**
- CLT > 70 C
- WBO2 valido (frame < 500 ms)
- Sem aceleracao ativa
- Sem rev-cut

### 7.9 LTFT (Long-Term Fuel Trim)

Grade 16x16 (mesmos eixos do VE). Atualizado lentamente:

```
celula += (stft - celula) / 64    (IIR com fator 1/64)
```

Persistido em Flash Bank2 Setor 0.

---

## 8. Pipeline de Calculo de Ignicao

**Arquivo:** `src/engine/ign_calc.cpp`

### 8.1 Tabela de Avanco

Grade 16x16 (mesmos eixos do VE):
- Valores: 0 a 70 graus BTDC (int8_t com offset de 40)
- Interpolacao bilinear Q8

```c
int16_t get_advance(rpm_x10, load_kpa);  // retorna graus x 10
```

### 8.2 Avanco Total

```
avanco_total = avanco_base + correcao_IAT + correcao_CLT - retardo_knock
avanco_total = clamp(avanco_total, -10, +40)  // limites rigidos
```

- **Negativo:** retardo apos TDC (raro, carga muito baixa)
- **Positivo:** avanco antes do TDC (operacao normal)
- **Maximo:** 40 BTDC (protecao contra detonacao)

### 8.3 Tempo de Dwell (Carga da Bobina)

```
Tensao (mV):   9000  10000  11000  12000  13000  14000  15000  16000
Dwell (ms×10):   42     38     35     30     28     25     23     22
Dwell (ms):     4.2    3.8    3.5    3.0    2.8    2.5    2.3    2.2
```

### 8.4 Angulo de Dwell

```
dwell_angle_x10 = (dwell_ms_x10 × RPM × 6) / 1000
```

Limitado a 359.9 graus (nao faz sentido mais de 1 rotacao).

### 8.5 Inicio do Dwell

```
dwell_start = spark_deg + dwell_angle
```

Exemplo: spark = 10 BTDC, dwell = 30 → dwell comeca 40 BTDC.

### 8.6 Ordem de Ignicao

```
Cilindros:         1      3      4      2
Offset TDC (graus): 0    180    360    540
```

4 cilindros, ciclo de 720 graus, disparo a cada 180 graus.

---

## 9. Scheduling no Dominio Angular

**Arquivo:** `src/engine/ecu_sched.cpp`

### 9.1 Arquitetura

O scheduling opera no **dominio angular** — cada evento e armazenado como `(tooth_index, sub_frac_x256, phase_A)` e disparado na ISR do dente correspondente.

**Vantagens sobre scheduling no dominio do tempo:**
- Elimina race condition entre ISR CKP (prio 1) e ISR TIM1 (prio 4)
- Timing exato em variacao de RPM (usa periodo do dente atual)
- ISR simplificada: sem gestao de fila ou retime

### 9.2 Hardware de Output Compare

**TIM1 (equivalente ao FTM0 Kinetis):**
- Clock: 250 MHz / PS=4 = 62.5 MHz
- Resolucao por tick: 16 ns
- Free-running 16-bit

**Mapeamento de canais:**

| Canal TIM1 | Funcao | Acao no match |
|------------|--------|---------------|
| CH0 (INJ3) | Injetor cyl 3 | SET (HIGH = aberto) |
| CH1 (INJ4) | Injetor cyl 4 | SET |
| CH2 (INJ1) | Injetor cyl 1 | SET |
| CH3 (INJ2) | Injetor cyl 2 | SET |
| CH4 (IGN4) | Bobina cyl 4 | CLEAR (LOW = disparo) |
| CH5 (IGN3) | Bobina cyl 3 | CLEAR |
| CH6 (IGN2) | Bobina cyl 2 | CLEAR |
| CH7 (IGN1) | Bobina cyl 1 | CLEAR |

### 9.3 Tabela de Eventos Angulares

```c
typedef struct {
    uint8_t tooth_index;     // 0-57 (dente alvo)
    uint8_t sub_frac_x256;   // 0-255 (fracao dentro do periodo do dente)
    uint8_t channel;         // TIM1 CH0-CH7
    uint8_t action;          // INJ_ON, INJ_OFF, DWELL_START, SPARK
    uint8_t phase_A;         // ECU_PHASE_A, B, ou ANY
    uint8_t valid;           // slot ocupado
} AngleEvent_t;
```

Capacidade: 20 eventos (4 cilindros x 4 eventos + margem).

### 9.4 Conversao Angulo → Evento

```
1. phase_A = (angulo < 360) ? PHASE_A : PHASE_B
2. angulo_na_rev = angulo % 360
3. pos_x256 = (angulo_na_rev × 256) / 6     // 60 posicoes × 6 graus
4. tooth = pos_x256 >> 8                     // parte inteira (0-59)
5. sub_frac = pos_x256 & 0xFF               // fracao (0-255)
6. Se tooth > 57: tooth = 57, sub_frac = 255 // regiao do gap
```

### 9.5 Disparo na ISR (on_tooth_hook)

A cada dente CKP, a ISR percorre a tabela de eventos:

```
Para cada evento:
  Se tooth_index == dente_atual E
     (phase_A == fase_atual OU phase_A == ANY):

    offset_ticks = (sub_frac × tooth_ftm0_ticks) >> 8
    target_cnv = TIM1_CNT + offset_ticks
    arma canal TIM1 com target_cnv e acao (SET/CLEAR)
```

Fator de conversao: `tooth_ftm0_ticks = tooth_period_ns × 3 / 1600`

### 9.6 Pre-Sync (HALF_SYNC)

Quando so um gap foi detectado (fase desconhecida):
- **Ignicao:** centelha perdida — pares (cyl 1,4) e (cyl 2,3) disparam toda revolucao
- **Injecao:** simultanea ou semi-sequencial (todos os 4 injetores)
- **Fase:** ECU_PHASE_ANY (dispara independente da fase do came)

### 9.7 Modo Sequencial (FULL_SYNC)

Apos fase confirmada pelo sinal de came:
- Cada cilindro dispara no angulo exato do seu TDC
- Injecao sequencial: cada injetor abre individualmente
- Restricao IVC: se PW excede angulo ate IVC, PW e recortado

### 9.8 Prime Pulse (Primeira Injecao no Cranking)

```
1. Dispara no 5o dente da partida
2. Todos os 4 injetores simultaneamente
3. PW = REQ_FUEL × multiplicador_CLT
4. Multiplicador: 3.0x @ -40C, 2.0x @ 20C, 1.15x @ 110C
```

---

## 10. Controles Auxiliares

**Arquivo:** `src/engine/auxiliaries.cpp`

### 10.1 IACV — Controle de Marcha Lenta (PID)

**Saida:** TIM3_CH1 (PA6) — PWM @ 125 Hz

```
Se CLT < 60C:
  duty = tabela_warmup[CLT]      // duty fixo por temperatura

Se CLT >= 60C:
  rpm_alvo = tabela_idle[CLT]    // 800-1200 RPM
  erro = rpm_alvo - rpm_atual

  P = 2 × erro / 100
  I += (erro × 6) / 1000         // clamp ±30
  D = (erro - erro_ant) × 5 / 2

  duty = duty_base + P + I + D   // clamp 0-1000
```

### 10.2 Wastegate — Controle de Boost (PID)

**Saida:** TIM3_CH2 (PA7) — PWM @ 125 Hz

```
Se MAP > alvo + 2.0 kPa:
  overboost_ms += dt
  Se overboost_ms >= 500 ms:
    failsafe = true → duty = 0   // corta boost imediatamente

Se nao failsafe:
  erro = alvo - MAP
  P = (erro × 8) / 100
  I += erro / 100                // clamp ±25
  duty = P + I                   // clamp 0-1000
```

### 10.3 VVT — Variable Valve Timing (PID duplo)

**Saidas:** TIM4_CH1 (PB6, escape), TIM4_CH2 (PB7, admissao) — PWM @ 150 Hz

```
posicao_deg = estimativa via tooth_index + phase_A
alvo_esc = tabela_vvt_esc[RPM][MAP]
alvo_adm = tabela_vvt_adm[RPM][MAP]

Para cada atuador (escape, admissao):
  erro = alvo - posicao
  P = (erro × 12) / 10
  I += erro / 20                 // clamp ±30
  duty = 500 + P + I             // 50% baseline + correcao, clamp 0-1000
```

### 10.4 Knock — Controle de Detonacao

**Arquivo:** `src/engine/knock.cpp`

Algoritmo PI por cilindro:

```
Se contagem_eventos > threshold:
  retardo += 2.0 graus           // retardo rapido
  vosel -= 2                     // reduz threshold do comparador
  ciclos_limpos = 0

Se nao:
  ciclos_limpos++
  Se ciclos_limpos >= 10:
    retardo -= 0.1 graus         // recuperacao lenta

  ciclos_limpos_global++
  Se ciclos_limpos_global >= 100:
    vosel += 1                   // aumenta threshold gradualmente
```

**Limites:**
- Retardo maximo: 10.0 graus
- VOSEL: 0-63 (6-bit DAC)
- VOSEL default: 32

### 10.5 Ventilador

```
Se CLT >= 95C: liga ventilador
Se CLT <= 90C: desliga ventilador
```

Histerese de 5 C. Saida GPIO (rele).

### 10.6 Bomba de Combustivel

```
Se ignition OFF:    bomba OFF
Se ignition ON < 2s: bomba ON  (prime)
Se RPM > 0:         bomba ON
Se RPM = 0 > 3s:    bomba OFF
```

Saida GPIO (rele).

---

## 11. Comunicacao

### 11.1 CAN — Protocolo de Mensagens

**Arquivo:** `src/app/can_stack.cpp`

#### Mensagem TX 0x400 — Dados em Tempo Real (~10 ms)

| Byte | Campo | Tipo | Unidade | Conversao |
|------|-------|------|---------|-----------|
| 0-1 | RPM | U16 LE | RPM | direto |
| 2 | MAP | U8 | kPa | map_kpa_x10 / 10 |
| 3 | TPS | U8 | % | tps_pct_x10 / 10 |
| 4 | CLT | S8 | C | (clt_degc_x10 / 10) + 40 |
| 5 | Avanco | S8 | graus | advance_deg + 40 |
| 6 | PW | U8 | 0.1 ms | pw_ms_x10 |
| 7 | Status | U8 | bitmask | ver tabela abaixo |

#### Mensagem TX 0x401 — Dados Auxiliares (~100 ms)

| Byte | Campo | Tipo | Unidade | Conversao |
|------|-------|------|---------|-----------|
| 0-1 | Pressao comb. | U16 LE | 0.1 kPa | fuel_press_kpa_x10 |
| 2-3 | Pressao oleo | U16 LE | 0.1 kPa | oil_press_kpa_x10 |
| 4 | IAT | S8 | C | (iat_degc_x10 / 10) + 40 |
| 5 | STFT | S8 | % | stft_pct + 100 |
| 6 | VVT admissao | U8 | % | vvt_intake_pct |
| 7 | VVT escape | U8 | % | vvt_exhaust_pct |

#### Mensagem RX 0x180 — WBO2 Lambda

| Byte | Campo | Tipo | Descricao |
|------|-------|------|-----------|
| 0-1 | Lambda x1000 | U16 LE | Ex: 1050 = lambda 1.050 |
| 2 | Status WBO2 | U8 | Status do controlador WBO2 |

**Timeout:** Se nenhum frame em 500 ms → `WBO2_FAULT` ativo, lambda fallback = 1.050.

#### Bits de Status (byte 7 do 0x400)

| Bit | Mascara | Nome | Significado |
|-----|---------|------|-------------|
| 0 | 0x01 | SYNC_FULL | Motor sincronizado |
| 1 | 0x02 | PHASE_A | Fase A do came |
| 2 | 0x04 | SENSOR_FAULT | Falha em sensor |
| 3 | 0x08 | LIMP_MODE | Modo limp ativo |
| 4 | 0x10 | SCHED_LATE | Evento atrasado do scheduler |
| 5 | 0x20 | SCHED_DROP | Evento descartado |
| 6 | 0x40 | SCHED_CLAMP | Parametro limitado |
| 7 | 0x80 | WBO2_FAULT | WBO2 offline (>500 ms) |

### 11.2 TunerStudio — Protocolo Serial

**Arquivo:** `src/app/tuner_studio.cpp`

Protocolo compativel com MegaSquirt (MS). Conexao via USART3 @ 115200 bps.

#### Comandos de Caractere Unico

| Comando | Resposta | Descricao |
|---------|----------|-----------|
| 'Q' / 'H' | "OpenEMS_v1.1" | Assinatura do firmware |
| 'S' | "OpenEMS_fw_1.1" | Versao do firmware |
| 'F' | "001" | Versao do protocolo |
| 'C' | 0x00 + 0xAA | Teste de comunicacao |
| 'A' / 'O' | 64 bytes | Stream de dados em tempo real (page 3) |

#### Leitura de Pagina ('r')

```
TX: 'r' + [page: 1B] + [offset: 2B LE] + [length: 2B LE]
RX: [data: length bytes]
```

#### Escrita de Pagina ('w')

```
TX: 'w' + [page: 1B] + [offset: 2B LE] + [length: 2B LE] + [data: length B]
RX: 0x00 (ACK_OK) ou 0x01 (ACK_ERR)
```

#### Paginas de Memoria

| Pagina | Tamanho | Conteudo | Acesso |
|--------|---------|----------|--------|
| 0 | 512 bytes | Configuracao principal (IVC etc.) | R/W |
| 1 | 256 bytes | Tabela VE (16x16) | R/W |
| 2 | 256 bytes | Tabela de avanco (16x16) | R/W |
| 3 | 64 bytes | Dados em tempo real | Somente leitura |

#### Estrutura da Page 3 (TsRealtimeData)

| Offset | Campo | Tipo | Unidade |
|--------|-------|------|---------|
| 0-1 | RPM | U16 | RPM |
| 2 | MAP | U8 | kPa |
| 3 | TPS | U8 | % |
| 4 | CLT + 40 | S8 | C |
| 5 | IAT + 40 | S8 | C |
| 6 | Lambda / 4 | U8 | millesimos/4 |
| 7 | PW x10 | U8 | 0.1 ms |
| 8 | Avanco + 40 | U8 | graus |
| 9 | VE | U8 | % |
| 10 | STFT + 100 | S8 | % |
| 11 | Status bits | U8 | bitmask |
| 12-63 | Diagnosticos | — | contadores internos |

---

## 12. Persistencia em Flash

**Arquivo:** `src/hal/flexnvm.cpp`

### 12.1 Layout da Flash Bank2 (0x08100000)

```
Setor 0 (0x08100000, 8 KB):
  ├── Offset 0x000: LTFT map (256 bytes, int8_t[16][16])
  ├── Offset 0x100: Knock map (64 bytes, int8_t[8][8])
  └── Offset 0x200: RuntimeSyncSeed (32 bytes)

Setor 1 (0x08102000, 8 KB): Calibracao pagina 0 (512 bytes)
Setor 2 (0x08104000, 8 KB): Calibracao pagina 1 (256 bytes)
Setor 3 (0x08106000, 8 KB): Calibracao pagina 2 (256 bytes)
Setores 4-7: Reservados para wear leveling
```

### 12.2 Procedimento de Escrita

```
1. Desbloquear Flash Bank2 (KEYR2: 0x45670123 + 0xCDEF89AB)
2. Apagar setor (SER + SNB + STRT)
3. Aguardar BSY clear (timeout 50 ms)
4. Programar em palavras de 32 bits (PG mode)
5. Verificar PGSERR e WRPERR
6. Re-travar (LOCK bit em FLASH_CR2)
```

### 12.3 Header de Calibracao

```c
struct CalHeader {
    uint32_t magic;   // 0xCA110E55
    uint32_t crc32;   // CRC-32 ISO 3309 dos dados
};
```

CRC-32: polinomio 0xEDB88320, inicial 0xFFFFFFFF, final ~crc.

### 12.4 RuntimeSyncSeed

```c
struct RuntimeSyncSeed {
    uint16_t magic;          // 0x5343 ("SC")
    uint8_t  version;        // 1
    uint8_t  flags;          // bit 0: VALID, bit 1: FULL_SYNC, bit 2: PHASE_A
    uint16_t tooth_index;    // 0-57 para decoder 60-2
    uint16_t decoder_tag;    // 0x3C02 (identificador 60-2)
    uint32_t sequence;       // contador de wear-leveling
    uint32_t crc32;          // CRC-32 dos campos [0:24]
};
```

---

## 13. Protecoes e Seguranca

### 13.1 IWDG (Watchdog)

- **Timeout operacional:** 100 ms
- **Kick:** primeiro statement de cada iteracao do loop
- **Falha:** reset automatico do MCU
- **Default Handler:** loop infinito → IWDG reseta em < 100 ms

### 13.2 Limp Mode

**Ativacao:** MAP fault (bit 0) OU CLT fault (bit 3) em `sensors.fault_bits`.

**Acoes:**
- Rev limiter: RPM < 3000 (kLimpRpmLimit_x10 = 30000)
- Corta injecao/ignicao se RPM excede o limite
- Sinalizado via STATUS_LIMP_MODE (bit 3) no CAN 0x400

### 13.3 Deteccao de Falha de Sensores

Cada sensor tem validacao de range:
- 3 leituras consecutivas fora do range → fault ativo
- Valores de fallback aplicados:
  - MAP: 101 kPa
  - TPS: 0%
  - CLT: 90 C
  - IAT: 25 C

### 13.4 Overboost Protection

- Se MAP > alvo + 2.0 kPa por mais de 500 ms:
  - Wastegate failsafe: duty = 0 (totalmente aberta)
  - Protege o motor contra pressao excessiva no turbo

### 13.5 Knock Protection

- Retardo rapido: +2.0 graus por evento de detonacao
- Recuperacao lenta: -0.1 graus apos 10 ciclos limpos
- Retardo maximo: 10.0 graus
- Persistido em Flash (sobrevive a reinicializacao)

### 13.6 MPU Stack Guard

- Regiao 0 do MPU: 256 bytes em 0x20000000 com NO_ACCESS
- Stack overflow causa MemManage fault → HardFault → IWDG reset

### 13.7 WBO2 Failsafe

- Se nenhum frame CAN do WBO2 em 500 ms:
  - Lambda fallback = 1.050 (levemente rico para protecao)
  - STATUS_WBO2_FAULT sinalizado
  - STFT desabilitado (evita aprendizado com lambda invalido)

---

## 14. Pinagem Completa do MCU

### STM32H562RGT6 — LQFP64

#### Funcoes Analogicas (ADC)

| Pino | Porta | Canal ADC | Sensor | Faixa |
|------|-------|-----------|--------|-------|
| 9 | PA2 | ADC1_IN3 | MAP | 0-250 kPa |
| 10 | PA3 | ADC1_IN4 | MAF | 0-1000 mV |
| 11 | PA4 | ADC1_IN5 | TPS | 0-100% |
| 12 | PA5 | ADC1_IN6 | O2 (reserva) | 0-1000 mV |
| 18 | PB0 | ADC1_IN7 | AN1 (expansao) | 0-4095 raw |
| 19 | PB1 | ADC1_IN8 | AN2 (expansao) | 0-4095 raw |
| 7 | PC0 | ADC1_IN9 | AN3 (expansao) | 0-4095 raw |
| 8 | PC1 | ADC1_IN10 | AN4 (expansao) | 0-4095 raw |
| 15 | PC2 | ADC2_IN1 | CLT | -40 a 150 C |
| 16 | PC3 | ADC2_IN2 | IAT | -40 a 150 C |
| 23 | PC4 | ADC2_IN13 | Pressao comb. | 0-250 kPa |
| 24 | PC5 | ADC2_IN14 | Pressao oleo | 0-250 kPa |

#### Funcoes de Timer (Ignicao/Injecao/PWM)

| Pino | Porta | Timer/Canal | AF | Funcao |
|------|-------|-------------|-----|--------|
| 29 | PA8 | TIM1_CH1 | AF1 | IGN4 (bobina cyl 4) |
| 30 | PA9 | TIM1_CH2 | AF1 | IGN3 (bobina cyl 3) |
| 31 | PA10 | TIM1_CH3 | AF1 | IGN2 (bobina cyl 2) |
| 32 | PA11 | TIM1_CH4 | AF1 | IGN1 (bobina cyl 1) |
| 13 | PA6 | TIM3_CH1 | AF2 | IACV PWM |
| 14 | PA7 | TIM3_CH2 | AF2 | Wastegate PWM |
| 58 | PB6 | TIM4_CH1 | AF2 | VVT Escape PWM |
| 59 | PB7 | TIM4_CH2 | AF2 | VVT Admissao PWM |

#### Funcoes de Captura (CKP/CMP)

| Pino | Porta | Timer/Canal | AF | Funcao |
|------|-------|-------------|-----|--------|
| 6 | PA0 | TIM5_CH1 | AF2 | CKP (roda fonica) |
| 5 | PA1 | TIM5_CH2 | AF2 | CMP (came) |

#### Comunicacao

| Pino | Porta | Periferico | AF | Funcao |
|------|-------|-----------|-----|--------|
| 25 | PB8 | FDCAN1_RX | AF9 | CAN RX |
| 26 | PB9 | FDCAN1_TX | AF9 | CAN TX |
| 21 | PB10 | USART3_TX | AF7 | UART TX (TunerStudio) |
| 22 | PB11 | USART3_RX | AF7 | UART RX (TunerStudio) |

#### SWD (Programacao/Debug)

| Pino | Porta | Funcao |
|------|-------|--------|
| 46 | PA13 | SWDIO |
| 49 | PA14 | SWDCLK |

### Diagrama Resumido de Conexoes

```
                        STM32H562RGT6 (LQFP64)
                     ┌──────────────────────────┐
                     │                          │
  CKP ──────────────►│ PA0  (TIM5_CH1)          │
  CMP ──────────────►│ PA1  (TIM5_CH2)          │
                     │                          │
  MAP ──────────────►│ PA2  (ADC1_IN3)          │──── PA8  IGN4 ──► Bobina 4
  MAF ──────────────►│ PA3  (ADC1_IN4)          │──── PA9  IGN3 ──► Bobina 3
  TPS ──────────────►│ PA4  (ADC1_IN5)          │──── PA10 IGN2 ──► Bobina 2
  O2  ──────────────►│ PA5  (ADC1_IN6)          │──── PA11 IGN1 ──► Bobina 1
                     │                          │
  AN1 ──────────────►│ PB0  (ADC1_IN7)          │──── PA6  IACV (PWM)
  AN2 ──────────────►│ PB1  (ADC1_IN8)          │──── PA7  Wastegate (PWM)
                     │                          │
  AN3 ──────────────►│ PC0  (ADC1_IN9)          │──── PB6  VVT Esc (PWM)
  AN4 ──────────────►│ PC1  (ADC1_IN10)         │──── PB7  VVT Adm (PWM)
                     │                          │
  CLT ──────────────►│ PC2  (ADC2_IN1)          │──── PB8  CAN RX
  IAT ──────────────►│ PC3  (ADC2_IN2)          │──── PB9  CAN TX
  Fuel Press ───────►│ PC4  (ADC2_IN13)         │
  Oil Press ────────►│ PC5  (ADC2_IN14)         │──── PB10 UART TX
                     │                          │──── PB11 UART RX
  SWDIO ────────────►│ PA13                     │
  SWDCLK ───────────►│ PA14                     │
                     └──────────────────────────┘
```

---

*Manual gerado a partir do codigo-fonte do firmware OpenEMS_H5 v1.1*
*Plataforma: STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz)*

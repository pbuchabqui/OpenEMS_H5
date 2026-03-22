# **OpenEMS**

## Engine Management System

### STM32H562RG · LQFP64 · 250 MHz · ARM Cortex-M33 + FPU + TrustZone

*Prompt de Engenharia — v2.2 (pinout verificado contra esquemático WeAct STM32H5 64-PIN CoreBoard V1.1)*

> **CHANGELOG**
>
> v2.2 — Pinout corrigido contra esquemático WeAct V1.1 (conflitos USB/MicroSD/SWD resolvidos):
>
> 📌 CONFLITOS RESOLVIDOS:
>
> • **PA11/PA12 = USB Type-C** (hardware fixo) → INJ4 migrado para **PC12 (TIM15_CH1)**, FDCAN1_TX para **PB7**
>
> • **MicroSD sacrificado** → libera PC8/PC9 (IGN3/4), PC10/PC11 (USART3 debug), PC12 (INJ4), PD2
>
> • **PA9/PA10 compartilham** header SWD P5 (TX/RX) → aceito, debug apenas via PA13/PA14 (SWD)
>
> • **PA8** tem R12 10k pull-up (MicroSD detect) → remover R12 ou aceitar para INJ1
>
> • **TunerStudio via USB CDC** (PA11/PA12) → USART1 eliminado, PB6/PB7 liberados
>
> • **Scheduler agora 3 timers**: TIM1 (INJ1-3) + TIM15 (INJ4) + TIM8 (IGN1-4)
>
> • **USART3** em PC10/PC11 (ex-MicroSD) adicionado para debug serial
>
> • **PB9/PB11** = VCAP (R7/R8 0Ω no esquemático, marcar NC) → NÃO USAR
>
> v2.1.1 — MCU: STM32H562RGT6 (LQFP64) com remapeamento para 64 pinos
>
> v2.1 — 13 otimizações de hardware STM32H5
>
> v2.0 — Migração de plataforma Teensy 3.5 → STM32H562
>
> ⚡ OTIMIZAÇÕES INTEGRADAS:
>
> • **OPT-1 TIM2 32-bit para CKP**: elimina overflow — 17.18s de range @ 250 MHz (era: 16-bit com overflow a cada 1.05 ms)
>
> • **OPT-2 TIM5 32-bit para scheduler**: timestamp absoluto de 32 bits, elimina recálculo a cada dente
>
> • **OPT-3 CORDIC hardware**: sin+cos em 116 ns (29 ciclos) para cálculo de ângulos — engine/ign_calc e ckp_angle_to_ticks
>
> • **OPT-4 FMAC hardware**: filtro IIR/FIR autônomo para knock sensor (CPU zero durante filtragem)
>
> • **OPT-5 GPDMA linked-list**: pipeline ADC → DMA → buffer circular sem intervenção de CPU
>
> • **OPT-6 ADC oversampling hardware**: ×16 → 14-bit efetivo, ×64 → 16-bit efetivo, zero CPU
>
> • **OPT-7 TIM1/TIM8 BKIN**: emergency shutdown assíncrono (funciona com CPU morta)
>
> • **OPT-8 Timer sync ITR**: TIM1 master → TIM8 slave, sincronização hardware sem jitter
>
> • **OPT-9 ICACHE 8 KB**: ISR de flash em 0-wait-state (4-5× mais rápido que sem cache)
>
> • **OPT-10 CAN-FD 64 bytes**: 1 frame = todos os dados de telemetria (era: 2 frames CAN clássico)
>
> • **OPT-11 Backup SRAM 4 KB**: crash log + freeze-frame sem desgaste de flash
>
> • **OPT-12 LPTIM em Stop mode**: monitoramento key-on/battery com MCU em ~2 µA
>
> • **OPT-13 DAC 12-bit para knock**: threshold adaptativo 64× mais preciso que CMP0 6-bit do MK64F
>
> v2.0 — Migração completa de plataforma: Teensy 3.5 (MK64FX512VMD12) → STM32H562RG (LQFP64)

---

# COMO USAR ESTE DOCUMENTO

> 11 prompts modulares · 1 sessão nova por prompt · ordem obrigatória

Este documento contém 11 prompts independentes, um por módulo de firmware. Cole cada prompt numa sessão completamente nova — sem histórico da sessão anterior. Cada prompt declara explicitamente quais módulos já existem; o LLM não precisa ver o código anterior, apenas as interfaces. Isso evita alucinação por contexto contaminado.

> **ℹ Disciplina de sessão**
>
> Sessão nova = contexto zerado. Um LLM com 100k tokens de contexto acumulado começa a 'lembrar' escolhas de sessões anteriores e contradiz interfaces definidas aqui. Não ceda à tentação de continuar na mesma sessão para 'poupar tempo' — vai custar reescrita.

---

# SEÇÃO 0 — CONTRATO GLOBAL

> leia antes de qualquer geração · aplica-se a todos os 11 prompts

## 0.1 Identidade e Toolchain

- MCU: **STM32H562RGT6** (LQFP64) — **Cortex-M33 @ 250 MHz**, FPU, **640 KB RAM**, **1 MB Flash**, **96 KB high-cycling data flash** (100K ciclos), **4 KB backup SRAM (VBAT)**

- Aceleradores: **CORDIC** (sin/cos/atan2 em 29 ciclos), **FMAC** (FIR/IIR hardware, 256×16-bit SRAM), **GPDMA** (linked-list, 16 canais)  ⚡ OPT-3/4/5

- Build: **PlatformIO + STM32CubeMX, framework = stm32cube, GCC ARM 12+**, C++17, flags: -O2 -fno-exceptions -fno-rtti -fno-threadsafe-statics -Wall

- Referências: RusEFI (scheduler, trigger decoder) + Speeduino (protocolo TunerStudio, estrutura de tabelas)

- Objetivo: superar ambas as referências em precisão de timing aproveitando **timers 32-bit (TIM2/TIM5) que eliminam overflow**, CORDIC para ângulos, GPDMA+ADC autônomo, e BKIN para safety

## 0.2 Regras Absolutas — NUNCA violar

> **🚫 delay() / millis() / micros() em código de tempo-real**
>
> Código de tempo-real usa registradores TIM diretamente (TIMx->CNT, TIMx->CCR, TIMx->SR).
>
> millis() é permitido APENAS no main loop de background (tarefas > 5 ms).
>
> **🚫 Alocação dinâmica de memória após init**
>
> new, malloc, std::vector, std::string, std::map — proibidos após setup().
>
> **🚫 Código HAL/LL de alto nível em hal/\***
>
> Acesso direto a registradores CMSIS: TIM2->CCR1, ADC1->SQR1, GPIOA->BSRR.
>
> Exceção: SystemClock_Config() pode usar HAL. CORDIC/FMAC: LL API permitida (LL_CORDIC_*).
>
> **🚫 float em ISRs de CKP e scheduler**
>
> ISR de CKP e scheduler: apenas uint32_t, aritmética inteira. float permitido em engine/\*.
>
> ⚡ **CORDIC usa q1.31 ou q1.15 fixed-point — compatível com esta regra. Usar CORDIC em ISR é permitido (29 ciclos determinísticos).**

## 0.3 Regras de Aritmética de Timestamp  ⚡ SIMPLIFICADO POR TIM2/TIM5 32-BIT

> **⚡ OPT-1/OPT-2: Timers 32-bit eliminam overflow para CKP e Scheduler**
>
> **TIM2 (CKP input capture)** e **TIM5 (scheduler output compare)** são 32-bit.
>
> PSC=0 (sem prescaler) → tick = 4 ns @ 250 MHz, overflow a cada **17.18 segundos**.
>
> A 50 RPM (idle mais lento possível), período de dente = 20 ms → 5.000.000 counts. SEM OVERFLOW.
>
> **Aritmética circular 32-bit**: `uint32_t delta = (uint32_t)(now - prev);` — funciona naturalmente.
>
> **Nenhuma ISR de overflow necessária. Nenhum counter extendido. Leitura atômica de 32 bits.**
>
> **TIM1/TIM8 permanecem 16-bit** (output compare para INJ/IGN), mas operam com timestamps
> convertidos do TIM5 32-bit. A conversão é: `uint16_t tim1_ticks = (uint16_t)(tim5_abs_ticks);`
> — funciona porque TIM1 e TIM5 compartilham o mesmo clock (via timer sync ITR).
>
> Referência: RusEFI issue #1488 — ISR atrasada + overflow 16-bit = falso gap. **32-bit elimina essa classe inteira de bugs.**

## 0.4 Pinout Canônico — Verificado contra DS14258 Rev 6

*Tabela definitiva. Fonte: DS14258 Rev 6, Tabelas 15/16 (AF0–AF15). Package: LQFP64.*

### Injeção — TIM1 (INJ1-3) + TIM15 (INJ4)  📌 v2.2: PA11=USB, INJ4 migrado para TIM15

| Pino | Port | AF | Função | Timer Canal | Pin LQFP64 | WeAct nota |
|---|---|---|---|---|---|---|
| PA8 | A | AF1 | INJ1 / TIM1_CH1 | TIM1 CH1 | 41 | ⚠ R12 10k pull-up (SD detect) — remover R12 |
| PA9 | A | AF1 | INJ2 / TIM1_CH2 | TIM1 CH2 | 42 | ⚠ Exposto em header P5 pin 5 (TX) |
| PA10 | A | AF1 | INJ3 / TIM1_CH3 | TIM1 CH3 | 43 | ⚠ Exposto em header P5 pin 7 (RX) |
| PC12 | C | AF2 | INJ4 / TIM15_CH1 | TIM15 CH1 | 53 | 📌 Era MicroSD CMD. **TIM15 synced via ITR** |

> 📌 **v2.2: INJ4 NÃO PODE usar PA11 (USB_DN na WeAct).** Alternativa: TIM15_CH1 em PC12 (liberado pelo sacrifício do MicroSD).
> TIM15 é 16-bit com 2 canais. PSC=0, ARR=0xFFFF, sincronizado com TIM5 via ITR — mesma base de tempo.
> **Scheduler agora gerencia 3 timers OC**: TIM1 (INJ1-3) + TIM15 (INJ4) + TIM8 (IGN1-4).

### Ignição — TIM8 Output Compare (16-bit, 4 canais)

| Pino | Port | AF | Função | Timer Canal | Pin LQFP64 |
|---|---|---|---|---|---|
| PC6 | C | AF3 | IGN1 / TIM8_CH1 | TIM8 CH1 | 37 |
| PC7 | C | AF3 | IGN2 / TIM8_CH2 | TIM8 CH2 | 38 |
| PC8 | C | AF3 | IGN3 / TIM8_CH3 | TIM8 CH3 | 39 |
| PC9 | C | AF3 | IGN4 / TIM8_CH4 | TIM8 CH4 | 40 |

### CKP/CMP — TIM2 Input Capture  ⚡ OPT-1 (era: TIM3 16-bit)

| Pino | Port | AF | Função | Timer Canal | Notas |
|---|---|---|---|---|---|
| PA0 | A | AF1 | CKP / TIM2_CH1 | TIM2 CH1 | pin 14 · ⚡ **32-bit capture, 4 ns tick, overflow 17s** |
| PA1 | A | AF1 | CMP / TIM2_CH2 | TIM2 CH2 | pin 15 · ⚡ **32-bit capture** |

### Scheduler Timebase — TIM5 Output Compare  ⚡ OPT-2 (NOVO)

| Pino | Port | AF | Função | Timer Canal | Notas |
|---|---|---|---|---|---|
| (interno) | — | — | Timebase 32-bit | TIM5 | ⚡ **32-bit absolute timestamps para scheduler** |

> ⚡ **OPT-2**: TIM5 roda free-running 32-bit @ 250 MHz. O scheduler armazena eventos com timestamps absolutos de 32 bits.
> Quando precisa programar TIM1/TIM8 (16-bit), converte: `uint16_t hw_ticks = (uint16_t)(abs_ticks);`
> TIM1 e TIM8 são sincronizados com TIM5 via ITR (OPT-8), garantindo que os 16 bits baixos coincidam.

### Auxiliares PWM — TIM3 (IACV/WG) + TIM12 (VVT)  📌 LQFP64: PD12/PD13 não existe → TIM3_CH3/CH4

| Pino | Port | AF | Função | Timer Canal | Pin LQFP64 |
|---|---|---|---|---|---|
| PB0 | B | AF2 | IACV / TIM3_CH3 | TIM3 CH3 | 26 |
| PB1 | B | AF2 | Wastegate / TIM3_CH4 | TIM3 CH4 | 27 |
| PB14 | B | AF2 | VVT Esc / TIM12_CH1 | TIM12 CH1 | 35 |
| PB15 | B | AF2 | VVT Adm / TIM12_CH2 | TIM12 CH2 | 36 |

> 📌 **IACV/WG migraram de TIM4 (PD12/PD13) para TIM3 CH3/CH4 (PB0/PB1).**
> TIM3 CH1/CH2 (PA6/PA7) permanecem livres — TIM2 assumiu CKP/CMP.
> Consequência: `tim4_pwm_init()` → `tim3_pwm_init()`, `tim4_set_duty()` → `tim3_set_duty()`.

### USB CDC — TunerStudio  📌 v2.2: PA11/PA12 fixos pela WeAct Type-C

| Pino | Port | Função | Pin LQFP64 | WeAct nota |
|---|---|---|---|---|
| PA11 | A | USB_DN (USB CDC) | 44 | Hardware fixo (Type-C connector J1) |
| PA12 | A | USB_DP (USB CDC) | 45 | Hardware fixo (Type-C connector J1) |

> 📌 **TunerStudio conecta via USB CDC.** Não é USART — implementar USB Device CDC ACM class.
> O USB já tem resistores de 5.1k (R9/R10) e proteção ESD no esquemático WeAct.

### FDCAN1  📌 v2.2: TX migrado de PA12(USB) para PB7

| Pino | Port | AF | Função | Pin LQFP64 |
|---|---|---|---|---|
| PB7 | B | AF9 | FDCAN1_TX | 59 |
| PB8 | B | AF9 | FDCAN1_RX | 61 |

> 📌 **PB7 era USART1_RX no v2.1.1 — não mais necessário (TunerStudio via USB CDC).**

### USART3 — Debug Serial  📌 v2.2: NOVO (pinos liberados do MicroSD)

| Pino | Port | AF | Função | Pin LQFP64 | WeAct nota |
|---|---|---|---|---|---|
| PC10 | C | AF7 | USART3_TX | 51 | Era MicroSD CLK (sacrificado) |
| PC11 | C | AF7 | USART3_RX | 52 | Era MicroSD D3/CD (sacrificado) |

> 📌 **115200 baud, 8N1. Para debug logging e diagnóstico. Não confundir com TunerStudio (USB).**

### BKIN — Emergency Shutdown  ⚡ OPT-7  📌 LQFP64: PE15 não existe → PB12 para TIM1_BKIN

| Pino | Port | AF | Função | Pin LQFP64 | Notas |
|---|---|---|---|---|---|
| PB12 | B | AF1 | TIM1_BKIN | 33 | ⚡ Desabilita INJ1-4 (overspeed/overcurrent) |
| PA6 | A | AF3 | TIM8_BKIN | 21 | ⚡ Desabilita IGN1-4 (overspeed/overcurrent) |

> ⚡ **Ambos BKIN disponíveis no LQFP64 sem conflito!** PB12 AF1=TIM1_BKIN, PA6 AF3=TIM8_BKIN.
> Conectar mesmo sinal de fault externo (LM393) a ambos os pinos via OR-wired.
> TIMx->BDTR: BKE=1, BKP=0 (active low), BKF=0000 (sem filtro digital = resposta em ns).
> LOCK level 2: congela configuração de break após init — firmware bugado não pode desabilitar.

### DAC — Knock Threshold  ⚡ OPT-13 (NOVO)

| Pino | Port | AF | Função | Notas |
|---|---|---|---|---|
| PA5 | A | AF0 | DAC1_OUT2 | ⚡ **12-bit threshold para knock (4096 níveis vs 64 do MK64F)** |

### GPIO Auxiliar  📌 v2.2: PB2=BOOT1/KEY na WeAct, usar PB3

| Pino | Função | Pin LQFP64 | WeAct nota |
|---|---|---|---|
| PB3 | Fan relay (GPIO push-pull) | 55 | Livre |
| PB10 | Bomba combustível relay (GPIO push-pull) | 29 | Livre |
| PC13 | LED status (Blue LED na WeAct) | 2 | ⚠ LED já soldado (active low, 5.1k R1) — usar como indicador EMS |

> ⚠ **PB2 (pin 28)**: BOOT1 + botão K3 na WeAct (pull-down 10k R4). NÃO usar como GPIO de saída.
> ⚠ **PB9 (pin 62)**: VCAP_1 via R7 (0Ω, marcado NC). Verificar se R7 está soldado antes de usar.
> ⚠ **PB11 (pin 30)**: VCAP_2 via R8 (0Ω, marcado NC). Verificar se R8 está soldado antes de usar.

> ⚡ **OPT-8: Timer Synchronization via ITR**
>
> TIM5 (32-bit master) → TRGO on Update → TIM1 slave reset + TIM8 slave reset.
>
> TIM1->SMCR: TS=ITR para TIM5, SMS=000 (desabilitado normalmente, reset on trigger).
>
> Isso garante que TIM1, TIM8 e TIM5 compartilham a mesma base de tempo.
>
> Quando TIM5 atinge 0xFFFF e volta a 0x0000, TIM1 e TIM8 fazem o mesmo — sincronia perfeita.

## 0.5 Alocação de Timers

| Timer | Tipo | Resolução | Função | Clock | Prioridade IRQ |
|---|---|---|---|---|---|
| TIM2 | GP | **32-bit** | ⚡ CKP/CMP input capture | 250 MHz, PSC=0 | **1** (mais alta) |
| TIM5 | GP | **32-bit** | ⚡ Scheduler timebase | 250 MHz, PSC=0 | 2 |
| TIM1 | Advanced | 16-bit | INJ1-3 output compare | 250 MHz, PSC=0 | 4 |
| TIM15 | GP | 16-bit | ⚡📌 INJ4 output compare (PC12) | 250 MHz, PSC=0 | 4 |
| TIM8 | Advanced | 16-bit | IGN1-4 output compare | 250 MHz, PSC=0 | 4 |
| TIM4 | GP | 16-bit | IACV/WG PWM | 250 MHz | 8 |
| TIM12 | GP | 16-bit | VVT PWM | 250 MHz | 8 |
| TIM6 | Basic | 16-bit | Datalog timestamp (1 MHz) | 250 MHz | 12 |
| TIM7 | Basic | 16-bit | Watchdog software (100 ms) | 250 MHz | 11 |
| LPTIM1 | LP | 16-bit | ⚡ OPT-12: Monitoramento Stop mode | LSE 32.768 kHz | 10 |

> **Clock gating (RCC)**:
> RCC->APB2ENR: TIM1EN, TIM8EN, TIM15EN  📌 v2.2: TIM15 para INJ4
> RCC->APB1ENR1: TIM2EN, TIM3EN, TIM5EN, TIM6EN, TIM7EN, TIM12EN  📌 TIM3 para CKP(v2.0)/IACV-WG(v2.1.1), TIM4 não usado
> RCC->AHB2ENR: GPIOAEN, GPIOBEN, GPIOCEN  📌 LQFP64: sem Port D GPIO (apenas PD2), sem Port E
> RCC->AHB1ENR: CORDICEN, FMACEN, GPDMA1EN  ⚡ OPT-3/4/5
> RCC->APB1ENR1: DAC1EN  ⚡ OPT-13

## 0.6 Interfaces Imutáveis — Contratos entre Módulos  ⚡ ATUALIZADO para 32-bit

```cpp
// ems/drv/ckp.h  ⚡ ATUALIZADO: 32-bit timestamps (TIM2)
namespace ems::drv {

enum class SyncState : uint8_t { WAIT, SYNCING, SYNCED };

struct CkpSnapshot {
    uint32_t tooth_period_ticks;  // ⚡ período em ticks TIM2 (4 ns cada) — sem conversão para ns
    uint16_t tooth_index;         // 0..57
    uint32_t last_tim2_capture;   // ⚡ valor raw 32-bit de TIM2->CCR1 — sem truncamento
    uint32_t rpm_x10;             // RPM × 10 sem float
    SyncState state;
    bool phase_A;
};

CkpSnapshot ckp_snapshot() noexcept;
uint32_t ckp_angle_to_ticks(uint16_t angle_x10,
                            uint32_t ref_capture) noexcept;  // ⚡ retorno 32-bit
}
```

```cpp
// ems/drv/scheduler.h  ⚡ v2.2: 3 timers (TIM1 INJ1-3 + TIM15 INJ4 + TIM8 IGN1-4)
namespace ems::drv {

enum class Channel : uint8_t {
    INJ1=0, INJ2=1, INJ3=2,               // TIM1_CH1..3 — Port A (PA8-PA10)
    INJ4=3,                                // 📌 TIM15_CH1 — PC12 (era TIM1_CH4/PA11, USB blocked)
    IGN1=4, IGN2=5, IGN3=6, IGN4=7,       // TIM8_CH1..4 — Port C (PC6-PC9)
};

enum class Action : uint8_t { SET, CLEAR };

// ⚡ abs_ticks é timestamp absoluto 32-bit (TIM5). Internamente convertido para TIM1/TIM8 16-bit.
bool sched_event(Channel ch, uint32_t abs_ticks, Action act) noexcept;  // ⚡ era: uint16_t
void sched_cancel(Channel ch) noexcept;
void sched_cancel_all() noexcept;
void sched_isr_tim1() noexcept;  // TIM1_CC_IRQHandler (INJ1-3)
void sched_isr_tim15() noexcept; // 📌 v2.2: TIM1_BRK_TIM15_IRQHandler (INJ4)
void sched_isr_tim8() noexcept;  // TIM8_CC_IRQHandler (IGN1-4)
void sched_recalc(const CkpSnapshot& snap) noexcept;
}
```

```cpp
// ems/drv/sensors.h — SEM ALTERAÇÃO na struct
namespace ems::drv {
struct SensorData { /* ... mesmo de antes ... */ };
const SensorData& sensors_get() noexcept;
}
```

```cpp
// ⚡ NOVO: ems/hal/cordic.h — OPT-3
namespace ems::hal {
// Input: angle em q1.31 [-1,+1] representando [-π,+π]. Output: sin/cos em q1.31.
void cordic_sincos_q31(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) noexcept;
// Input: angle_deg_x10 (0..3600). Output: sin/cos em q1.15 (±32767).
void cordic_sincos_deg(uint16_t angle_deg_x10, int16_t* sin_out, int16_t* cos_out) noexcept;
}
```

---

# PROMPT 1 — HAL: Timers + ICACHE + BKIN + Sync  ⚡ EXPANDIDO

> hal/tim.h + hal/tim.cpp + hal/cordic.h + hal/cordic.cpp

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ Você é engenheiro de firmware embarcado especializado em STM32H5.
2 │ Implemente os módulos HAL de Timers e CORDIC para o OpenEMS no STM32H562RG.

3 │

4 │ # HARDWARE ALVO
5 │ MCU: STM32H562RGT6, core 250 MHz (PLL1), APB1/APB2 250 MHz
6 │ Aceleradores: CORDIC coprocessor, FMAC, GPDMA com linked-list

7 │

8 │ # ⚡ OPT-1: TIM2 — 32-BIT CKP INPUT CAPTURE
9 │   CH1: CKP → PA0  AF1  TIM2_CH1  input capture rising edge (pin 14)
10│   CH2: CMP → PA1  AF1  TIM2_CH2  input capture rising edge (pin 15)
11│ PSC = 0 (sem prescaler) → tick = 4 ns, counter 32-bit → overflow em 17.18 s
12│ ARR = 0xFFFFFFFF (free-running 32-bit)
13│ CCMR1: CC1S=01, CC2S=01 (direct input)
14│ CCER: CC1P=0 (rising), CC2P=0 (rising), CC1E=1, CC2E=1
15│ DIER: CC1IE=1 (interrupt on CKP capture)
16│ ⚡ VANTAGEM: elimina TOTALMENTE bugs de overflow de timestamp.
17│ Um leitura atômica de TIM2->CCR1 (32-bit) dá o timestamp completo.

18│

19│ # ⚡ OPT-2: TIM5 — 32-BIT SCHEDULER TIMEBASE (NOVO)
20│ Free-running 32-bit @ 250 MHz. Não tem pinos externos — é apenas timebase interna.
21│ PSC = 0, ARR = 0xFFFFFFFF.
22│ Usado pelo scheduler para timestamps absolutos. Eventos são programados como:
23│   abs_ticks = tim5_now + delta_ticks
24│ Na hora de acionar TIM1/TIM8 compare: hw_ticks = (uint16_t)(abs_ticks)
25│ Funciona porque TIM1/TIM8 estão sincronizados via ITR (OPT-8).

26│

27│ # ⚡ OPT-8: SINCRONIZAÇÃO TIM5 → TIM1 + TIM8 via ITR
28│ TIM5 como master: TIM5->CR2: MMS = 010 (Update event as TRGO)
29│ TIM1 como slave: TIM1->SMCR: TS = ITR (selecionar fonte TIM5), SMS = 100 (reset mode)
30│   → TIM1 reseta seu counter quando TIM5 dá update (overflow a cada 17s)
31│   → Na prática, os 16 bits baixos de TIM5->CNT == TIM1->CNT a qualquer instante
32│ TIM8 como slave: mesma configuração.
33│ RESULTADO: scheduler pode converter abs_ticks 32-bit → TIM1/TIM8 16-bit por simples cast.

34│

35│ TIM1 — 3 canais INJ: PA8(CH1) PA9(CH2) PA10(CH3) AF1, output compare
36│   📌 v2.2: TIM1_CH4(PA11) BLOQUEADO — USB Type-C na WeAct
37│ TIM15 — 1 canal INJ4: PC12(CH1) AF2, output compare  📌 v2.2: NOVO
38│ TIM8 — 4 canais IGN: PC6(CH1) PC7(CH2) PC8(CH3) PC9(CH4) AF3, output compare
39│ TIM3 CH3/CH4 — PWM IACV(PB0)/WG(PB1) AF2
40│ TIM12 — PWM VVT: PB14(CH1)/PB15(CH2) AF2
41│ TIM6/TIM7 — basic timers

39│

40│ # ⚡ OPT-7: BKIN — EMERGENCY SHUTDOWN ASSÍNCRONO
41│ TIM1->BDTR: MOE=1, BKE=1, BKP=0 (active low), AOE=0 (manual re-arm only), BKF=0000
42│ TIM1->BDTR: LOCK=2 (congela BKE/BKP/MOE lock após init)
43│ TIM8->BDTR: mesma configuração
44│ Pinos: PB12 AF1 = TIM1_BKIN (pin 33), PA6 AF3 = TIM8_BKIN (pin 21)  📌 era: PE15
45│ ⚡ CRÍTICO: com LOCK=2, firmware bugado NÃO consegue desabilitar o break.
46│ Idle state quando break ativo: todos os canais → LOW (injetores/bobinas desligados).
47│ Re-arm manual: após fault clearado, firmware seta MOE=1 novamente.

48│

49│ # ⚡ OPT-9: ICACHE — HABILITAR NO INIT
50│ ICACHE->CR: EN=1 (habilitar instruction cache 8 KB, 2-way set-associative)
51│ Flash com 5 wait-states @ 250 MHz → sem ICACHE, ISR executa a ~42 MHz efetivo.
52│ COM ICACHE: ISR executa a 250 MHz após primeira invocação (cache hit).
53│ Habilitar ANTES de qualquer configuração de timer.
54│ FLASH->ACR: PRFTEN=1 (prefetch enable) — complementar ao ICACHE.

55│

56│ # ⚡ OPT-3: CORDIC COPROCESSOR (hal/cordic.h + hal/cordic.cpp)
57│ RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
58│ CORDIC->CSR: FUNC=0001 (cosine), PRECISION=6 (24 iterações, ~20-bit),
59│   NRES=1 (2 results: cos + sin), NARGS=1, ARGSIZE=0 (32-bit), RESSIZE=0 (32-bit)
60│ Uso: CORDIC->WDATA = angle_q31; // write triggers computation
61│       while (!(CORDIC->CSR & CORDIC_CSR_RRDY)); // 29 cycles max
62│       int32_t cos_result = CORDIC->RDATA; // primary
63│       int32_t sin_result = CORDIC->RDATA; // secondary
64│ Conversão graus→q31: angle_q31 = (int32_t)((angle_deg / 180.0) * 2147483648.0)
65│ Para inteiro: angle_q31 = (int32_t)(((int64_t)angle_deg_x10 * 11930465LL) / 10)
66│ ⚡ Usar em engine/ign_calc para converter ângulos de avanço em timestamps.

67│

68│ # INTERFACE A IMPLEMENTAR
69│ namespace ems::hal {
70│   // Timers
71│   void tim2_init();   // ⚡ 32-bit CKP capture (era: tim3_init 16-bit)
72│   void tim5_init();   // ⚡ 32-bit scheduler timebase (NOVO)
73│   void tim1_init();   // INJ1-3 output compare + BKIN
74│   void tim8_init();   // IGN1-4 output compare + BKIN
75│   void tim15_init();  // 📌 v2.2: INJ4 output compare (PC12) — synced via ITR
75│   void tim3_pwm_init(uint32_t freq_hz);  // 📌 LQFP64: era tim4, agora TIM3 CH3/CH4
76│   void tim12_pwm_init(uint32_t freq_hz);
77│   void icache_init(); // ⚡ ICACHE + flash prefetch
78│   uint32_t tim2_count() noexcept;   // ⚡ 32-bit (era: tim3_count 16-bit)
79│   uint32_t tim5_count() noexcept;   // ⚡ 32-bit (NOVO)
80│   void tim1_set_compare(uint8_t ch, uint16_t ticks) noexcept;
81│   void tim8_set_compare(uint8_t ch, uint16_t ticks) noexcept;
82│   void tim15_set_compare(uint16_t ticks) noexcept;  // 📌 v2.2: INJ4 (single channel)
83│   void tim1_clear_ccf(uint8_t ch) noexcept;
84│   void tim8_clear_ccf(uint8_t ch) noexcept;
85│   void tim15_clear_ccf() noexcept;  // 📌 v2.2: INJ4
84│   void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;  // 📌 CH3/CH4
85│   void tim12_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;
86│   void bkin_rearm_tim1() noexcept;  // ⚡ OPT-7: re-enable MOE após fault
87│   void bkin_rearm_tim8() noexcept;  // ⚡ OPT-7
88│   // CORDIC
89│   void cordic_init();
90│   void cordic_sincos_q31(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) noexcept;
91│   void cordic_sincos_deg(uint16_t angle_deg_x10, int16_t* sin_out, int16_t* cos_out) noexcept;
92│   // Handlers
93│   extern "C" void TIM1_CC_IRQHandler();
94│   extern "C" void TIM8_CC_IRQHandler();
95│   extern "C" void TIM1_BRK_TIM15_IRQHandler();  // 📌 v2.2: TIM15 shares IRQ with TIM1_BRK
96│   extern "C" void TIM2_IRQHandler();
96│ }

97│

98│ # ENTREGÁVEIS
99│  1. src/hal/tim.h + tim.cpp — TODOS os timers acima
100│ 2. src/hal/cordic.h + cordic.cpp — CORDIC sincos wrapper
101│ 3. test/hal/test_tim_32bit.cpp:
102│    - TIM2 32-bit: now=100, prev=0xFFFFFF00 → delta = 356 (wrap de 32-bit funciona)
103│    - TIM5→TIM1 sync: (uint16_t)(0x0001ABCD) == 0xABCD
104│ 4. test/hal/test_cordic.cpp: sin(90°) → ~32767 (q1.15), cos(0°) → ~32767
```

---

# PROMPT 2 — DRIVER: CKP Decoder 60-2  ⚡ 32-BIT (OVERFLOW ELIMINADO)

> drv/ckp.h + drv/ckp.cpp · ISR < 200 ns  ⚡ (era: < 500 ns — 2.5× mais rápido com ICACHE)

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ Implemente o CKP Decoder (roda fônica 60-2) para o OpenEMS.

2 │

3 │ # HAL DISPONÍVEL
4 │ ems::hal::tim2_count() → uint32_t   ⚡ 32-bit (era: tim3_count 16-bit)
5 │ TIM2->CCR1 → registrador de captura 32-bit (era: TIM3->CCR1 16-bit)
6 │ GPIOA->IDR bit 0 → confirmar rising CKP (PA0)  ⚡ (era: PA6)
7 │ GPIOA->IDR bit 1 → confirmar rising CMP (PA1)  ⚡ (era: PA7)

8 │

9 │ # ⚡ VANTAGEM 32-BIT PARA CKP
10│ Com TIM2 32-bit @ 250 MHz (tick = 4 ns):
11│ - A 10.000 RPM: tooth period = ~100 µs = 25.000 ticks (resolução 0.004%)
12│ - A 100 RPM: tooth period = ~10 ms = 2.500.000 ticks (cabe em uint32_t sem overflow)
13│ - Gap ratio: 1.5× threshold funciona perfeitamente com aritmética de 32-bit
14│ - NENHUMA ISR de overflow necessária
15│ - NENHUM counter extendido / combinação de registradores
16│ - Leitura atômica: uint32_t capture = TIM2->CCR1; // completo em 1 instrução

17│

18│ [... roda fônica, gap detection, estados — MESMA LÓGICA do v2.0, com uint32_t ...]

19│

20│ # STRUCT DE OUTPUT  ⚡ ATUALIZADO
21│ struct CkpSnapshot {
22│     uint32_t tooth_period_ticks;  // ⚡ ticks diretos (4 ns cada), não mais ns
23│     uint16_t tooth_index;
24│     uint32_t last_tim2_capture;   // ⚡ 32-bit completo
25│     uint32_t rpm_x10;
26│     SyncState state;
27│     bool phase_A;
28│ };
29│ // rpm_x10 = (60ULL * 250000000ULL * 10) / (58 * tooth_period_ticks)
30│ //         = 1500000000000ULL / (58 * ticks) — requer uint64_t no numerador

31│

32│ # FUNÇÃO ckp_angle_to_ticks()  ⚡ ATUALIZADO
33│ // Converte ângulo futuro → timestamp absoluto 32-bit (TIM5/TIM2)
34│ // abs_ticks = ref_capture + (angle_x10 * tooth_period_ticks) / TOOTH_ANGLE_x1000
35│ // Resultado é uint32_t — scheduler converte para TIM1/TIM8 16-bit internamente

36│

37│ # ENTREGÁVEIS
38│ 1. src/drv/ckp.h + ckp.cpp
39│ 2. test/drv/test_ckp.cpp — ADICIONAR teste de 32-bit: capture wrap de 0xFFFFFF00→0x100
```

---

# PROMPT 3 — DRIVER: Event Scheduler  ⚡ 32-BIT TIMESTAMPS

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ # INTERFACES EXISTENTES
2 │ ems::hal::tim5_count() → uint32_t   ⚡ 32-bit scheduler timebase
3 │ ems::hal::tim1_set_compare(ch, uint16_t ticks)
4 │ ems::hal::tim8_set_compare(ch, uint16_t ticks)
5 │ ems::drv::ckp_snapshot() → CkpSnapshot (com 32-bit timestamps)

6 │

7 │ # ⚡ VANTAGEM: SCHEDULER COM TIMESTAMPS ABSOLUTOS 32-BIT
8 │ Eventos armazenados como uint32_t abs_ticks (timestamp absoluto TIM5).
9 │ Verificação de "evento já passou": uint32_t delta = abs_ticks - tim5_count();
10│ if (delta > 0x80000000u) return false; // mais de 8.5s no passado — impossível
11│ Na hora de programar TIM1/TIM8 compare:
12│   uint16_t hw = (uint16_t)(abs_ticks); // 16 bits baixos = TIM1/TIM8->CNT (por sync ITR)
13│ ⚡ ELIMINA sched_recalc() a cada dente — timestamps absolutos não precisam recálculo.
14│ sched_recalc() ainda existe mas é NOP (compatibilidade de interface).

15│

16│ # MAPEAMENTO CANAL → GPIO (BSRR)  📌 v2.2: INJ4 em PC12 (TIM15)
17│ INJ1 = TIM1_CH1  → PA8  → GPIOA->BSRR bit 8/24
18│ INJ2 = TIM1_CH2  → PA9  → GPIOA->BSRR bit 9/25
19│ INJ3 = TIM1_CH3  → PA10 → GPIOA->BSRR bit 10/26
20│ INJ4 = TIM15_CH1 → PC12 → GPIOC->BSRR bit 12/28   📌 timer diferente!
21│ IGN1 = TIM8_CH1  → PC6  → GPIOC->BSRR bit 6/22
22│ IGN2 = TIM8_CH2  → PC7  → GPIOC->BSRR bit 7/23
23│ IGN3 = TIM8_CH3  → PC8  → GPIOC->BSRR bit 8/24
24│ IGN4 = TIM8_CH4  → PC9  → GPIOC->BSRR bit 9/25

25│

26│ [... lógica de fila, ISR dual TIM1/TIM8 — como v2.0, com uint32_t abs_ticks ...]

27│

28│ # ENTREGÁVEIS
29│ src/drv/scheduler.h + scheduler.cpp
30│ test/drv/test_scheduler.cpp: testar com abs_ticks 32-bit, conversão para 16-bit
```

---

# PROMPT 4 — DRIVER: Sensor Pipeline  ⚡ GPDMA + ADC OVERSAMPLING

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ # ⚡ OPT-5: GPDMA LINKED-LIST PARA ADC (ZERO CPU)
2 │ Pipeline autônomo: TIM2 TRGO (cada dente CKP) → triggers ADC1 → GPDMA transfere resultado
3 │ para buffer circular em SRAM → half-transfer interrupt para double-buffering.
4 │
5 │ GPDMA1 Channel 0: ADC1 → SRAM (DMA request de ADC1 EOC)
6 │   Linked-List Item: source = &ADC1->DR, dest = &adc1_buf[0], transfer_size = 6 (canais)
7 │   Circular mode: GPDMA1_C0LLR aponta de volta para primeiro LLI
8 │   Half-transfer interrupt: CPU processa buffer[0..2] enquanto DMA preenche buffer[3..5]
9 │
10│ GPDMA1 Channel 1: ADC2 → SRAM (mesmo esquema)
11│
12│ ⚡ RESULTADO: CPU não toca no ADC durante operação normal.
13│ Apenas lê valores filtrados do buffer quando precisa.

14│

15│ # ⚡ OPT-6: ADC HARDWARE OVERSAMPLING
16│ ADC1->CFGR2: OVSE=1, OVSR=0011 (×16), OVSS=0010 (right shift 2) → 14-bit efetivo
17│ ADC2->CFGR2: OVSE=1, OVSR=0011 (×16), OVSS=0010 (right shift 2) → 14-bit efetivo
18│ Para MAP (requer máxima precisão): OVSR=0101 (×64), OVSS=0010 → 16-bit efetivo
19│   ⚡ Substitui o modo 16-bit nativo do ADC0 do MK64F com resultado MELHOR (15.8 ENOB vs ~13 ENOB)
20│ Oversampling é global por ADC — usar ADC1 para canais que precisam ×64 (MAP),
21│ ADC2 para canais com ×16 (CLT, IAT, pressões).
22│ Throughput com ×16: 312.5 ksps (mais que suficiente para 12 amostras/revolução a 10k RPM)

23│

24│ # SENSORES E CANAIS  ⚡ ATUALIZADO  📌 LQFP64: PB0/PB1 ocupados por TIM3 IACV/WG
25│ MAP:   ADC1 (PC0 pin 8)  — ×64 oversampling → 16-bit efetivo, DMA circular
26│ TPS:   ADC1 (PC2 pin 10) — ×16 oversampling → 14-bit, DMA circular
27│ O2:    ADC1 (PA4 pin 19) — ×16 oversampling, DMA circular
28│ CLT:   ADC2 (PC4 pin 24) — ×16 oversampling, software poll 100 ms
29│ IAT:   ADC2 (PC5 pin 25) — ×16 oversampling, software poll 100 ms
30│ P.cmb: ADC2 (PA2 pin 16) — ×16 oversampling, software poll 50 ms  📌 era: PB1 (conflito TIM3)
31│ P.oil: ADC2 (PA3 pin 17) — ×16 oversampling, software poll 50 ms  📌 era: PB0 (conflito TIM3)

32│

33│ # ADC TRIGGER
34│ TIM2 TRGO trigger → ADC1 regular group + ADC2 regular group (simultâneo)
35│ ADC1->CFGR: EXTEN=01 (rising edge), EXTSEL para TIM2_TRGO
36│ TIM2->CR2: MMS=010 (Update as TRGO) — dispara ADC a cada dente de CKP

37│

38│ # FILTRO IIR — SOFTWARE (canais lentos) ou ⚡ FMAC (canal knock — ver Prompt 7)
39│ Para MAP/TPS/O2: IIR software continua (overhead trivial a 250 MHz)
40│ ⚡ FMAC reservado para knock sensor (Prompt 7) — único canal de alta taxa

41│

42│ [... linearização, detecção de falha — MESMA LÓGICA ...]

43│

44│ # ENTREGÁVEIS
45│ 1. src/hal/adc.h + adc.cpp — ⚡ config ADC1/ADC2 + GPDMA + oversampling + TIM2 trigger
46│ 2. src/drv/sensors.h + sensors.cpp
47│ 3. test/drv/test_sensors.cpp — adicionar: verificar que oversampling ×16 raw > 4095
```

---

# PROMPT 5 — ENGINE: Fuel + Ignition  ⚡ CORDIC PARA ÂNGULOS

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ [... tabelas VE, fórmula de pulso, correções — MESMA LÓGICA ...]

2 │

3 │ # ⚡ OPT-3: USAR CORDIC PARA CONVERSÃO ÂNGULO → TICKS
4 │ engine/ign_calc: calcular dwell_angle e spark_angle usando CORDIC para conversões
5 │ trigonométricas quando necessário (ex: compensação de distorção angular da roda fônica).
6 │ ems::hal::cordic_sincos_deg(angle_deg_x10, &sin_val, &cos_val) — 116 ns por chamada.
7 │ ⚡ Permite implementar interpolação harmônica de velocidade angular entre dentes
8 │ (correção de ~3.9° offset documentado em RusEFI issue #778) em tempo real.

9 │

10│ # Conversão µs → ticks  ⚡ ATUALIZADO PARA TIM5 32-bit
11│ pw_ticks = pw_final_us * 250; // 250 ticks/µs @ 250 MHz PSC=0, resultado uint32_t
12│ ⚡ (era: pw_final_us × 120 / 2 @ 120MHz/PSC2, resultado uint16_t)
13│ O scheduler aceita uint32_t abs_ticks diretamente.

14│

15│ [... entregáveis — mesmos, com teste de CORDIC angle conversion ...]
```

---

# PROMPT 7 — Knock Control  ⚡ DAC 12-BIT + FMAC

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ # ⚡ OPT-13: DAC 12-BIT PARA KNOCK THRESHOLD
2 │ DAC1_OUT2 (PA5): 12-bit (4096 níveis) → 0.8 mV/step @ 3.3V
3 │   ⚡ era: CMP0_DACCR[VOSEL] 6-bit (64 níveis) → 52 mV/step no MK64F
4 │   = 64× mais resolução para threshold adaptativo
5 │ DAC1->DHR12R2 = threshold_value; // 0..4095
6 │ DAC1->CR: EN2=1, TEN2=0 (software trigger), BOFF2=0 (buffer enable)
7 │
8 │ Conectar DAC1_OUT2 (PA5) ao pino inversor de comparador EXTERNO (LM393).
9 │ Knock sensor → pino não-inversor do LM393 (após filtro passa-banda 6-15 kHz).
10│ Saída do LM393 → GPIO com EXTI (interrupt na borda rising).
11│
12│ ⚡ Threshold adaptativo via DMA: tabela knock_threshold[cyl][rpm_bucket]
13│ TIM2 TRGO (por dente) → GPDMA → atualiza DAC1->DHR12R2 automaticamente.
14│ Zero CPU para ajuste de threshold durante operação normal.

15│

16│ # ⚡ OPT-4: FMAC PARA FILTRAGEM DE KNOCK
17│ FMAC configurado com IIR passa-banda (6-15 kHz, 4ª ordem, Direct Form 1):
18│ Pipeline: ADC knock → DMA → FMAC input buffer (X1) → FMAC computa → output (Y) → DMA → SRAM
19│ FMAC->X1BUFCFG: X1_BUF_SIZE, X1_WATERMARK
20│ FMAC->X2BUFCFG: coeficientes do filtro (pré-carregados no init)
21│ FMAC->PARAM: P=4 (feedback taps), Q=4 (feedforward taps), FUNC=IIR
22│ ⚡ CPU zero durante filtragem — só processa eventos de knock (EXTI interrupt).

23│

24│ # JANELA DE DETECÇÃO — 10°-90° ATDC (mesma lógica)
25│ Habilitar/desabilitar DAC output + EXTI conforme ângulo do cilindro.
26│ Usar TIM1/TIM8 output compare para abrir/fechar janela.

27│

28│ # ALGORITMO — mesma lógica (knock_count, retard, recovery, adaptive threshold)
29│ Diferença: threshold_value é 12-bit (0..4095) em vez de 6-bit (0..63)
30│ Increment/decrement: VOSEL±2 → DAC_value±128 (escala proporcional)

31│

32│ # ENTREGÁVEIS
33│ src/hal/dac.h + dac.cpp  ⚡ NOVO
34│ src/engine/knock.h + knock.cpp
35│ test/engine/test_knock.cpp: verificar DAC output linear, EXTI mock, retardo aplicado
```

---

# PROMPT 9 — CAN Stack  ⚡ CAN-FD 64 BYTES

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ # HARDWARE
2 │ FDCAN1: 500 kbps arbitration, ⚡ 2 Mbps data phase (BRS enabled)
3 │ TX: PB7 AF9 (pin 59), RX: PB8 AF9 (pin 61). Transceiver: MCP2562FD.  📌 v2.2: PB7 (era PA12=USB)

4 │

5 │ # ⚡ OPT-10: CAN-FD COM FRAMES DE 64 BYTES
6 │ TX ID 0x400 (10 ms): ⚡ FRAME ÚNICO DE 48 BYTES (era: 2 frames de 8 bytes)
7 │   Bytes [0:1] RPM, [2:3] MAP, [4:5] TPS, [6:7] CLT, [8:9] IAT,
8 │   [10:11] advance, [12:13] PW, [14:15] lambda, [16:17] STFT,
9 │   [18:19] fuel_press, [20:21] oil_press, [22:23] vbatt,
10│   [24:27] abs_crank_angle, [28:31] ckp_period, [32:33] knock_retard[0..3],
11│   [34] status_bits, [35] fault_bits, [36:39] uptime_ms, [40:47] reserved
12│ ⚡ VANTAGEM: 1 frame a 10 ms = 100 Hz full telemetry (era: 50 Hz parcial + 10 Hz parcial)
13│ TX ID 0x401 (100 ms): eliminado — tudo cabe em 0x400

14│

15│ RX ID 0x180: wideband O2 (mesma lógica)
16│ ⚡ ATENÇÃO: FDCAN no STM32H5 tem FIFO de apenas 3 elementos e SEM suporte a DMA.
17│ Servir RX via interrupt (FDCAN1_IT0_IRQHandler) imediatamente.
18│ Se FIFO overflow: frame perdido — configurar FDCAN1->RXGFC para rejeitar non-matching IDs.

19│

20│ # ENTREGÁVEIS
21│ src/hal/fdcan.h + fdcan.cpp — config CAN-FD com BRS, Message RAM, FIFO
22│ src/app/can_stack.h + can_stack.cpp — serialização 48-byte frame
23│ test/app/test_can.cpp: serializar/desserializar frame de 48 bytes
```

---

# PROMPT 10 — Flash Persistence  ⚡ BACKUP SRAM

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ [... high-cycling flash + wear-leveling — MESMA LÓGICA do v2.0 ...]

2 │

3 │ # ⚡ OPT-11: BACKUP SRAM 4 KB (VBAT-MAINTAINED)
4 │ PWR->DBPCR |= PWR_DBPCR_DBP; // unlock backup domain
5 │ RCC->AHB4ENR |= RCC_AHB4ENR_BKPSRAMEN; // enable clock
6 │ Endereço base: 0x38800000 (4 KB, acessível como SRAM normal)
7 │
8 │ Uso:
9 │   - Crash log: últimos 256 bytes — RPM, MAP, TPS, fault_bits no momento do crash
10│   - Freeze-frame: snapshot completo de SensorData quando fault_bits != 0
11│   - Boot counter: uint32_t incrementado a cada power-on
12│   - Last-known-good: último estado do motor antes de desligar (RPM=0 confirmado)
13│
14│ Layout:
15│   [0x000..0x003] magic (0xDEADBEEF = dados válidos)
16│   [0x004..0x007] boot_count
17│   [0x008..0x0FF] crash_log (último fault)
18│   [0x100..0x1FF] freeze_frame (SensorData + CkpSnapshot no momento do fault)
19│   [0x200..0x3FF] reserved
20│
21│ ⚡ VANTAGEM: writes ilimitados (é SRAM, não flash). Sobrevive reset e power-off (com VBAT).
22│ Consumo: ~1.5 µA com VBAT. Tamper detection: TAMP pode auto-erase se intrusão detectada.

23│

24│ # API ATUALIZADA
25│ void bkpsram_write_crash(const SensorData& s, const CkpSnapshot& ckp);
26│ bool bkpsram_read_crash(SensorData& s, CkpSnapshot& ckp);
27│ uint32_t bkpsram_boot_count();

28│

29│ # ENTREGÁVEIS
30│ src/hal/flash_nvm.h + flash_nvm.cpp — flash + backup SRAM
31│ test/hal/test_flash_nvm.cpp — incluir teste de backup SRAM read/write
```

---

# PROMPT 11 — Main Loop + Integration  ⚡ ICACHE + LPTIM + BKIN

**PROMPT — COPIE E COLE EM SESSÃO NOVA**

```
1 │ # SEQUÊNCIA DE INIT (ordem obrigatória)  ⚡ ATUALIZADO
2 │ 1.  SystemClock_Config(): HSE → PLL1 → 250 MHz (HAL permitido APENAS aqui)
3 │ 2.  ⚡ ems::hal::icache_init() — ANTES de tudo (acelera todo o init subsequente)
4 │ 3.  RCC clock gating: habilitar TODOS os periféricos (incluindo CORDIC, FMAC, GPDMA, DAC, USB)
5 │ 4.  ems::hal::tim2_init(), tim5_init(), tim1_init(), tim15_init(), tim8_init(),
6 │     tim3_pwm_init(125), tim12_pwm_init(150)  ⚡📌 tim15 NOVO para INJ4
7 │ 5.  ems::hal::cordic_init()  ⚡ OPT-3
8 │ 6.  ems::hal::adc_init() — com GPDMA linked-list + oversampling  ⚡ OPT-5/6
9 │ 7.  ems::hal::dac_init()   ⚡ OPT-13 (knock threshold)
10│ 8.  ems::hal::fdcan_init(), usb_cdc_init(), usart3_init()  📌 v2.2: USB CDC + USART3 debug
11│ 9.  ems::hal::flash_nvm_init() → carregar calibração + ler crash log de backup SRAM
12│ 10. ems::drv::ckp_init(), sched_init(), sensors_init()
13│ 11. ems::engine::fuel_init(), ign_init(), aux_init(), knock_init()
14│ 12. ems::app::ts_init(), can_stack_init()
15│ 13. NVIC prioridades:
16│     TIM2 (CKP)     = 1 (mais alta)
17│     TIM5            = 2
18│     TIM1_CC (INJ1-3) = 4
19│     TIM1_BRK/TIM15 (INJ4) = 4  📌 v2.2: shared IRQ vector
20│     TIM8_CC (IGN)   = 4
20│     ADC1/2          = 5
21│     FDCAN1_IT0      = 6  ⚡ (servir FIFO imediatamente — só 3 slots)
22│     TIM7 (watchdog) = 11
23│     TIM6 (datalog)  = 12
24│ 14. Aguardar state == SYNCED antes de habilitar sched_event()

25│

26│ # MAIN LOOP — tarefas de background
27│ 5 ms: calcular fuel_calc + ign_calc → sched_event() com abs_ticks 32-bit
28│ 10 ms: IACV PID, VVT PID, Boost PID
29│ 20 ms: processar buffer TunerStudio RX
30│ 50 ms: transmitir CAN-FD 0x400 (48 bytes, frame único)  ⚡ OPT-10
31│ 100 ms: atualizar LTFT se condições ok
32│ 500 ms: flush calibração para flash se dirty flag
33│ A cada fault: ⚡ bkpsram_write_crash() — salvar snapshot em backup SRAM

34│

35│ # ⚡ OPT-12: LPTIM1 — MONITORAMENTO EM STOP MODE
36│ Quando RPM=0 por 30s E ignition key still on:
37│   - Entrar em Stop mode (consumo ~2 µA com LPTIM + LSE + BKPSRAM)
38│   - LPTIM1 wakeup a cada 1 s: verificar VBAT, key state via GPIO, CAN bus activity
39│   - Se key-off detectado: gravar estado final em BKPSRAM, manter Stop
40│   - Se key-on + cranking: wakeup imediato via EXTI (CKP edge)
41│ Configuração: LPTIM1->CFGR: CKSEL=0 (LSI), PRESC=128 → ~256 Hz
42│ LPTIM1->ARR = 255 → wakeup a cada ~1 s

43│

44│ # TIM7 WATCHDOG — como v2.0

45│

46│ # ENTREGÁVEIS
47│ src/main.cpp — init + main loop + stop mode entry/exit
48│ platformio.ini para STM32H562RG (LQFP64)
```

---

# CRITÉRIOS DE ACEITAÇÃO  ⚡ ATUALIZADO COM OTIMIZAÇÕES

| Módulo | Critério obrigatório | Como verificar |
|---|---|---|
| hal/tim ⚡ | TIM2/TIM5 32-bit sem overflow. TIM1↔TIM8 sync via ITR. BKIN ativa e LOCK=2. ICACHE enabled. | Teste 32-bit wrap + scope sync |
| hal/cordic ⚡ | sin(90°)→32767±1, cos(0°)→32767±1, latência ≤ 29 ciclos. | Teste q1.15 + cycle count |
| drv/ckp ⚡ | Sync ≤ 2 rev. Falso gap rejeitado. RPM correto. **32-bit capture sem overflow.** | Simulação 120 pulsos host |
| drv/scheduler ⚡ | Eventos em ordem. Cancel limpo. **32-bit abs_ticks → 16-bit compare correto.** | Teste unitário + mock |
| drv/sensors ⚡ | Valores corretos. **ADC oversampling ×16 retorna >4095.** DMA circular funcional. | Tabela valores + DMA check |
| engine/fuel_calc | PW ±2%. AE dispara threshold. | Cálculo manual |
| engine/ign_calc ⚡ | Dwell > spark. Clamp -10°/+55°. **CORDIC angle usado.** | Valores extremos |
| engine/knock ⚡ | Retardo após threshold. **DAC 12-bit linear.** FMAC filter convergente. | Mock EXTI + DAC output scope |
| app/tuner_studio | Frames r/w/A corretos. | Captura serial |
| app/can_stack ⚡ | **Frame CAN-FD de 48 bytes** decodificável. BRS funcional. | Loopback hardware |
| hal/flash_nvm ⚡ | LTFT sobrevive power cycle. **Backup SRAM crash log funcional.** | Write/read loop |
| main.cpp ⚡ | Sync < 3 rev. **ICACHE hit rate > 90%.** LPTIM wakeup funcional. BKIN testado. | Scope + power analyzer |

---

> **ℹ Filosofia v2.1**
>
> Este prompt explora 13 otimizações de hardware do STM32H5 que não existiam no MK64F.
> A mais impactante é o uso de **TIM2/TIM5 32-bit que elimina a classe inteira de bugs de overflow**
> que afetou RusEFI (#1488), Speeduino, e toda EMS baseada em timers de 16-bit.
> CORDIC, FMAC, e GPDMA movem computação de software para hardware dedicado.
> BKIN com LOCK=2 garante safety mesmo com firmware corrompido.

---

# RESUMO DAS 13 OTIMIZAÇÕES E MÓDULOS AFETADOS

| # | Otimização | Periférico | Prompt(s) afetado(s) | Impacto |
|---|---|---|---|---|
| 1 | CKP 32-bit | TIM2 | 1, 2 | **Elimina overflow — classe inteira de bugs** |
| 2 | Scheduler 32-bit | TIM5 | 1, 3 | Timestamps absolutos, sem recálculo por dente |
| 3 | CORDIC sin/cos | CORDIC | 1, 5 | Ângulos em 116 ns, interpolação harmônica viável |
| 4 | FMAC filter | FMAC | 7 | Filtragem knock autônoma, CPU zero |
| 5 | DMA linked-list | GPDMA | 4 | Pipeline ADC completamente autônomo |
| 6 | ADC oversampling | ADC1/ADC2 | 4 | 14-16 bit efetivo em hardware, zero CPU |
| 7 | BKIN emergency | TIM1/TIM8 BDTR | 1, 11 | Shutdown assíncrono, LOCK=2 anti-tamper |
| 8 | Timer sync ITR | TIM5→TIM1/TIM8 | 1, 3 | Conversão 32→16 bit por simple cast |
| 9 | ICACHE 8 KB | ICACHE | 1, 11 | ISR 4-5× mais rápido, jitter reduzido |
| 10 | CAN-FD 64B | FDCAN1 | 9 | 1 frame = telemetria completa a 100 Hz |
| 11 | Backup SRAM | BKPSRAM | 10, 11 | Crash log sem desgaste de flash |
| 12 | LPTIM Stop mode | LPTIM1 | 11 | Monitoramento a ~2 µA com motor parado |
| 13 | DAC 12-bit knock | DAC1 | 7 | 64× mais resolução que CMP0 6-bit do MK64F |

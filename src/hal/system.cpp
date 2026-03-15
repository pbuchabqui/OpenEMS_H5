/**
 * @file hal/stm32h562/system.cpp
 * @brief Clock (PLL → 250 MHz), SysTick (1 ms), IWDG (100 ms)
 *        para STM32H562RGT6 — substitui runtime Teensyduino.
 *
 * Configuração de clock:
 *   HSE  =  8 MHz (cristal externo)
 *   PLL1: M=1, N=62, P=2  →  (8 / 1) × 62 / 2 = 248 MHz  ≈ 250 MHz *
 *         M=1, N=125, P=4 →  (8 / 1) × 125/ 4 = 250 MHz  (exato)
 *   SYSCLK = 250 MHz
 *   HCLK   = 250 MHz  (AHB prescaler = 1)
 *   APB1   = 125 MHz  (APB1 prescaler = 2)
 *   APB2   = 125 MHz  (APB2 prescaler = 2)
 *   Timers APB2 (TIM1): 250 MHz  (timer doubler ativo quando PPRE2 ≠ 1)
 *   Timers APB1 (TIM3/4/5/6): 250 MHz
 *
 * IWDG a 32 kHz LSI:
 *   Prescaler /32 → 1000 Hz; Reload = 99 → timeout ≈ 100 ms
 */

#ifndef EMS_HOST_TEST

#include "hal/system.h"
#include "hal/regs.h"

// ── Variável global de contagem SysTick ─────────────────────────────────────
static volatile uint32_t g_systick_ms = 0u;
// Reload value para 1 ms a 250 MHz (SysTick usa HCLK)
static constexpr uint32_t kSysTickReload = 250000u - 1u;  // 1 ms

// ── SysTick_Handler ──────────────────────────────────────────────────────────
// Chamado a cada 1 ms pelo SysTick timer (ARM core, prioridade configurável).
extern "C" void SysTick_Handler(void) noexcept {
    ++g_systick_ms;
}

// ── API pública ───────────────────────────────────────────────────────────────

uint32_t millis(void) noexcept {
    // Leitura atômica: uint32_t em ARM é leitura de uma instrução (LDR).
    return g_systick_ms;
}

uint32_t micros(void) noexcept {
    // Combina contagem de ms com o valor atual do SysTick (conta regressiva).
    // SysTick CVR: valor atual do contador (decresce de kSysTickReload a 0).
    // µs = ms × 1000 + (kSysTickReload - CVR) / 250
    //   onde 250 = kSysTickReload / 1000 = ciclos por µs @ 250 MHz.
    //
    // FIX: leitura de g_systick_ms e CVR em loop até consistentes.
    // Se SysTick disparar entre as duas leituras, g_systick_ms muda → retry.
    // Sem retry, micros() poderia retornar um valor até ~1ms menor que o real,
    // causando jitter nos cálculos de datalog e timing.
    uint32_t ms, cvr;
    do {
        ms  = g_systick_ms;
        // ARM SysTick CVR em 0xE000E018 (leitura atômica de 32 bits)
        cvr = *reinterpret_cast<volatile uint32_t*>(0xE000E018u);
    } while (g_systick_ms != ms);  // retry se SysTick aconteceu entre as duas leituras
    const uint32_t elapsed_us = (kSysTickReload - cvr) / 250u;
    return ms * 1000u + elapsed_us;
}

void iwdg_kick(void) noexcept {
    IWDG_KR = IWDG_KR_REFRESH;
}

// ── MPU: proteção de memória básica ──────────────────────────────────────────
// Configura 3 regiões MPU no Cortex-M33 do STM32H562:
//   Região 0: Stack guard — 256 bytes na base do SRAM1 (no-access, XN)
//             Detecta stack overflow via MemManage fault.
//   Região 1: Flash Bank1 código — read/execute only (impede writes acidentais)
//   Região 2: Periféricos APB/AHB — privileged RW, execute-never
//
// Nota: endereço da stack guard (0x20000000) é o início do SRAM1.
// Se o linker script colocar .data/.bss imediatamente acima, o guard
// capturará overflows que ultrapassam todo o espaço de dados. Para
// proteção mais granular, refinar após linker script disponível.
static void mpu_init() noexcept {
    // Verificar se MPU está presente (TYPE.DREGION != 0)
    if ((MPU_TYPE >> 8) == 0u) { return; }  // sem MPU — não configurar

    MPU_CTRL = 0u;  // desabilitar MPU para reconfigurar com segurança

    // MAIR0: atributos de memória
    //   Byte 0 (AttrIndx=0): Normal memory, outer=WB+RA+WA, inner=WB+RA+WA → 0xFF
    //   Byte 1 (AttrIndx=1): Device memory, nGnRE → 0x04
    MPU_MAIR0 = (0xFFu << 0)   // Attr0: Normal, Write-Back, cacheable (Flash + SRAM)
              | (0x04u << 8);  // Attr1: Device, nGnRE (periféricos)

    // ── Região 0: Stack guard (256 bytes @ base SRAM1 = 0x20000000) ──────────
    // Stack descende de endereços altos para baixos. Se ultrapassar os dados
    // e atingir esta região, gera MemManage fault — detectável via HardFault.
    MPU_RNR  = 0u;
    MPU_RBAR = MPU_RBAR_ADDR(0x20000000u)  // base alinhada a 32 bytes
             | MPU_RBAR_AP_NO_ACCESS        // AP=00: qualquer acesso → fault
             | MPU_RBAR_XN;                 // XN=1: execute-never
    MPU_RLAR = MPU_RLAR_LIMIT(0x200000FFu) // limit: 0x200000E0 (inclui até FF)
             | MPU_RLAR_ATTR_NORMAL         // AttrIndx=0: Normal memory
             | MPU_RLAR_EN;                 // região habilitada

    // ── Região 1: Flash Bank1 (firmware) — read/execute only ─────────────────
    // Impede writes acidentais ao código. Tentativa de escrita → MemManage fault.
    // STM32H562: Bank1 = 0x08000000–0x0807FFFF (512 KB)
    MPU_RNR  = 1u;
    MPU_RBAR = MPU_RBAR_ADDR(0x08000000u)
             | MPU_RBAR_AP_RO_PRIV;        // AP=10: RO privileged (sem XN → executável)
    MPU_RLAR = MPU_RLAR_LIMIT(0x0807FFFFu)
             | MPU_RLAR_ATTR_NORMAL
             | MPU_RLAR_EN;

    // ── Região 2: Periféricos (APB1+APB2+AHB1+AHB2) — priv RW, execute-never ─
    // Acesso a periféricos por código não-privilegiado → MemManage fault.
    // Cobre 0x40000000–0x5FFFFFFF (periféricos STM32H5).
    MPU_RNR  = 2u;
    MPU_RBAR = MPU_RBAR_ADDR(0x40000000u)
             | MPU_RBAR_AP_RW_PRIV         // AP=01: RW privileged only
             | MPU_RBAR_XN;                // XN=1: execute-never
    MPU_RLAR = MPU_RLAR_LIMIT(0x5FFFFFFFu)
             | MPU_RLAR_ATTR_DEVICE        // AttrIndx=1: Device memory
             | MPU_RLAR_EN;

    // Habilitar MPU com mapa default para código privilegiado
    // PRIVDEFENA=1: código privilegiado pode acessar regiões fora das configuradas
    // (necessário para SysTick, NVIC, etc. no espaço 0xE0000000+)
    MPU_CTRL = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;

    // Barreira de instrução para garantir que MPU está ativo antes do retorno
    asm volatile("dsb" ::: "memory");
    asm volatile("isb" ::: "memory");
}

// ── Inicialização do sistema ──────────────────────────────────────────────────

void system_stm32_init(void) noexcept {
    // ── 0. IWDG antes de tudo (LSI independente de cristal externo) ──────
    // FIX: IWDG era inicializado APÓS os loops de HSE e PLL. Se o cristal HSE
    // falhasse, o sistema travava para sempre sem possibilidade de reset por watchdog.
    // Solução: inicializar IWDG com timeout longo (3 s) antes de ligar HSE;
    // reduzir para 100 ms após a inicialização completa.
    IWDG_KR  = IWDG_KR_START;    // Inicia IWDG (habilita LSI automaticamente)
    IWDG_KR  = IWDG_KR_ACCESS;   // Desbloqueia PR e RLR
    IWDG_PR  = IWDG_PR_DIV32;    // Prescaler /32 → 1000 Hz (LSI 32 kHz)
    IWDG_RLR = 2999u;            // Reload = 2999 → 3 s timeout durante init
    IWDG_KR  = IWDG_KR_REFRESH;  // Primeiro kick

    // ── 1. Habilitar HSE e aguardar estabilização ─────────────────────────
    // FIX: loop sem timeout → se HSE travar, IWDG agora reseta após 3 s.
    // Timeout de software adicional (~10 ms @ HSI 64 MHz) para log futuro.
    static constexpr uint32_t kHseTimeoutCycles = 640000u;
    RCC_CR |= RCC_CR_HSEON;
    {
        uint32_t t = kHseTimeoutCycles;
        while ((RCC_CR & RCC_CR_HSERDY) == 0u) {
            if (--t == 0u) {
                // HSE não respondeu: aguardar IWDG reset (3 s) — mais seguro
                // do que tentar operar com PLL instável.
                for (;;) { IWDG_KR = IWDG_KR_REFRESH; /* aguarda reset */ }
            }
        }
    }

    // ── 2. Flash latency + cache antes de aumentar clock ─────────────────
    // 5 WS exigidos para HCLK > 210 MHz @ VOS0 (RM0481 §9.3.3)
    FLASH_ACR = (FLASH_ACR & ~0xFu)
              | FLASH_ACR_LATENCY_5WS
              | FLASH_ACR_PRFTEN
              | FLASH_ACR_ICEN
              | FLASH_ACR_DCEN;
    // Aguarda que o novo valor seja aplicado
    while ((FLASH_ACR & 0xFu) != 5u) { /* aguarda */ }

    // ── 3. Configurar PLL1: HSE=8 MHz / M=1 × N=125 / P=4 = 250 MHz ────
    // PLL1CFGR: PLLSRC=HSE (01b), DIVM1=1 (M-1 = 0)
    RCC_PLL1CFGR = (1u << 0)   // PLLSRC = HSE (bits [1:0] = 01)
                 | (0u << 8);  // DIVM1 = 1 (valor M-1 = 0, bits [13:8])
    // PLL1DIVR: N=125 (DIVN1 = N-1 = 124), P=4 (DIVP1 = P/2-1 = 1)
    // Bits [8:0]  = DIVN1 (N-1) = 124 = 0x7C
    // Bits [14:9] = DIVP1 (P/2-1) = 1
    // Bits [22:16]= DIVQ1 — não usado, manter 0
    // Bits [30:24]= DIVR1 — não usado, manter 0
    RCC_PLL1DIVR = (124u << 0)   // DIVN1 = 124 → N = 125
                 | (1u << 9);    // DIVP1 = 1   → P = 4

    // ── 4. Ligar PLL1 e aguardar lock ────────────────────────────────────
    // FIX: loop sem timeout → IWDG reseta após 3 s se PLL não fizer lock.
    static constexpr uint32_t kPllTimeoutCycles = 100000u;
    RCC_CR |= RCC_CR_PLL1ON;
    {
        uint32_t t = kPllTimeoutCycles;
        while ((RCC_CR & RCC_CR_PLL1RDY) == 0u) {
            if (--t == 0u) {
                for (;;) { IWDG_KR = IWDG_KR_REFRESH; /* aguarda reset */ }
            }
        }
    }

    // ── 5. Configurar prescalers APB (manter AHB = SYSCLK) ───────────────
    // CFGR1: AHB prescaler = 1 (HPRE=0), APB1=/2 (PPRE1=100b), APB2=/2 (PPRE2=100b)
    // Para simplificar timers: usar APB1=APB2=HCLK/2 → timer clock = 2×APB = HCLK
    RCC_CFGR1 = (0u << 4)    // HPRE = 0  → HCLK = SYSCLK
              | (4u << 8)    // PPRE1 = 4 → APB1 = HCLK/2 = 125 MHz
              | (4u << 11);  // PPRE2 = 4 → APB2 = HCLK/2 = 125 MHz

    // ── 6. Selecionar PLL1 como SYSCLK ───────────────────────────────────
    RCC_CFGR1 = (RCC_CFGR1 & ~0x7u) | RCC_CFGR1_SW_PLL1;
    while ((RCC_CFGR1 & (7u << 3)) != RCC_CFGR1_SWS_PLL1) { /* aguarda */ }

    // ── 7. Habilitar clocks dos GPIOs ────────────────────────────────────
    // STM32H562RGT6 (LQFP64): apenas GPIOA/B/C disponíveis no package
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN
                  | RCC_AHB2ENR1_GPIOBEN
                  | RCC_AHB2ENR1_GPIOCEN;

    // ── 8. Configurar SysTick @ 1 ms ─────────────────────────────────────
    // ARM SysTick registers (CMSIS):
    //   0xE000E010 = STK_CTRL  (ENABLE | TICKINT | CLKSRC=processor)
    //   0xE000E014 = STK_LOAD
    //   0xE000E018 = STK_VAL
    volatile uint32_t* stk_load = reinterpret_cast<volatile uint32_t*>(0xE000E014u);
    volatile uint32_t* stk_val  = reinterpret_cast<volatile uint32_t*>(0xE000E018u);
    volatile uint32_t* stk_ctrl = reinterpret_cast<volatile uint32_t*>(0xE000E010u);

    *stk_load = kSysTickReload;
    *stk_val  = 0u;
    // CLKSRC=1 (processor clock), TICKINT=1 (interrupt enable), ENABLE=1
    *stk_ctrl = (1u << 2) | (1u << 1) | (1u << 0);

    // SysTick priority: equivalente ao PIT0 do Kinetis (prio 11)
    // ARM: SCB->SHP[11] = priority for SysTick (offset 0xE000ED23)
    *reinterpret_cast<volatile uint8_t*>(0xE000ED23u) = static_cast<uint8_t>(11u << 4u);

    // ── 9. Reduzir IWDG para 100 ms (operação normal) ────────────────────
    // Boot completo: reduzir timeout de 3 s para 100 ms operacional.
    IWDG_KR  = IWDG_KR_ACCESS;   // Desbloqueia PR e RLR para reconfiguração
    IWDG_RLR = IWDG_RLR_100MS;   // Reload = 99 → 99/1000 ≈ 99 ms
    IWDG_KR  = IWDG_KR_REFRESH;  // Kick após reconfiguração

    // ── 10. Configurar MPU (proteção de stack + Flash + periféricos) ──────
    mpu_init();
}

#else  // EMS_HOST_TEST

#include "hal/system.h"
#include <cstdint>
static uint32_t g_mock_ms = 0u;
void system_stm32_init(void) noexcept { }
void iwdg_kick(void) noexcept { }
uint32_t millis(void) noexcept { return g_mock_ms; }
uint32_t micros(void) noexcept { return g_mock_ms * 1000u; }

#endif  // EMS_HOST_TEST

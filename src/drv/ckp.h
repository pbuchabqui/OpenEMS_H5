/**
 * @file drv/ckp.h
 * @brief Decodificador de roda fônica 60-2 e máquina de sincronismo — OpenEMS
 *
 * RODA FÔNICA 60-2
 * ────────────────
 *   60 posições angulares; 2 dentes consecutivos ausentes = 58 dentes reais.
 *   Espaçamento normal: 360°/60 = 6,0° por posição.
 *   Gap: ≈ 3 × período normal (18°).
 *
 * MÁQUINA DE ESTADOS (SyncState)
 * ───────────────────────────────
 *
 *                      gap && count≥55
 *   WAIT      ─────────────────────────►  SYNCING
 *       ▲                                     │  gap && count≥55
 *       │   gap detected                      ▼
 *   LOSS_OF_SYNC  ◄─── count>61 ────  SYNCED
 *       │                                     │
 *       └──────────── gap detected ───────────┘
 *                       (re-sync)
 *
 * HARDWARE: TIM2/TIM5 input capture no pinout STM32H5 v2.2.
 *   ISR: entrada CKP/CMP servida pela HAL de timers/capture.
 *   Prioridade NVIC: 1 (mais alta do sistema) — §CLAUDE.md tabela IRQ
 */

#pragma once

#include <cstdint>

namespace ems::drv {

/**
 * @brief Estado da máquina de sincronismo CKP.
 *
 * @note Os valores numéricos são estáveis — não alterar sem verificar
 *       todo código que usa comparação direta com o inteiro subjacente.
 */
enum class SyncState : uint8_t {
    WAIT = 0,       ///< Contrato v2.2: aguardando sincronismo
    SYNCING = 1,    ///< Contrato v2.2: sincronismo em progresso
    SYNCED = 2,     ///< Contrato v2.2: sincronizado
    LOSS_OF_SYNC = 3,
};

/**
 * @brief Instantâneo do decodificador CKP (sem estado mutável).
 *
 * Todos os campos são consistentes entre si no momento da chamada a
 * ckp_snapshot() (captura atômica via seção crítica).
 */
struct CkpSnapshot {
    uint32_t tooth_period_ticks;  ///< Contrato v2.2: período em ticks do timer CKP
    uint16_t tooth_index;         ///< Índice do dente (0–57) contado desde o último gap; válido em SYNCED
    uint32_t last_tim2_capture;   ///< Contrato v2.2: captura crua 32-bit
    uint32_t rpm_x10;            ///< RPM × 10 (ex: 8000 = 800,0 RPM); 0 antes de dados suficientes
    SyncState state;             ///< Estado corrente da máquina de sincronismo
    bool phase_A;                ///< Fase do ciclo de 4 tempos — alterna a cada evento no cam sensor (CH1)
};

/**
 * @brief Retorna instantâneo atômico do estado CKP.
 *
 * Seguro para chamada de qualquer contexto (main loop, ISR de menor
 * prioridade). Usa seção crítica CPSID/CPSIE internamente.
 */
CkpSnapshot ckp_snapshot() noexcept;

/**
 * @brief Converte ângulo de virabrequim em tick-alvo absoluto.
 *
 * @param angle_x10   Ângulo em décimos de grau.
 * @param ref_capture Timestamp de referência.
 * @return            Timestamp absoluto no mesmo domínio do capture.
 */
uint32_t ckp_angle_to_ticks(uint16_t angle_x10, uint32_t ref_capture) noexcept;

// ── Hooks ─────────────────────────────────────────────────────────────────────
// Chamados pela ISR de CKP a cada dente (símbolos fracos — sobrescreva para
// adicionar comportamento sem modificar este módulo).
//
// sensors_on_tooth  → drv/sensors.cpp  (amostragem sincronizada ao dente)
// schedule_on_tooth → engine/cycle_sched.cpp (agendamento injeção/ignição)
// prime_on_tooth    → engine/quick_crank.cpp (prime pulse — 5º dente de cranking)

void sensors_on_tooth(const CkpSnapshot& snap) noexcept;
void schedule_on_tooth(const CkpSnapshot& snap) noexcept;
void prime_on_tooth(const CkpSnapshot& snap) noexcept;

// ── ISR handlers (chamados da HAL de timer/capture) ──────────────────────────
void ckp_capture_primary_isr() noexcept;   ///< CKP rising edge
void ckp_capture_secondary_isr() noexcept; ///< Cam sensor rising edge

/**
 * @brief Arm a persisted sync seed for fast reacquire on next valid gap.
 *
 * Safety note: this does not bypass gap validation; it only allows promotion
 * WAIT/LOSS_OF_SYNC -> SYNCED at the first accepted gap if a persisted seed is armed.
 */
void ckp_seed_arm(bool phase_A) noexcept;
void ckp_seed_disarm() noexcept;

uint32_t ckp_seed_loaded_count() noexcept;
uint32_t ckp_seed_confirmed_count() noexcept;
uint32_t ckp_seed_rejected_count() noexcept;

// ── API de teste (somente em build host) ──────────────────────────────────────
#if defined(EMS_HOST_TEST)
void     ckp_test_reset() noexcept;
uint32_t ckp_test_rpm_x10_from_period_ticks(uint32_t period_ticks) noexcept;
#endif

}  // namespace ems::drv

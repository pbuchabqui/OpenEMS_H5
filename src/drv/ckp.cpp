/**
 * @file drv/ckp.cpp
 * @brief Módulo 1 (DECODE) + Módulo 2 (SYNC) — Engine Position Core — OpenEMS
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * MÓDULO 1: DECODE via timer input capture (Crank)
 * ───────────────────────────────────────────────
 *   Hardware: input capture do timer de crank.
 *             Em targets embarcados o caminho é mapeado via hal/tim.cpp.
 *             Em host tests, o registrador de captura é um volatile mock.
 *
 *   Fluxo da ISR (ckp_capture_primary_isr):
 *     1. Verificação anti-glitch: pino PA0 ainda HIGH? (não é noise falling edge)
 *     2. Leitura do registrador de captura travado pelo HW.
 *        ► NÃO lemos o contador livre: ele avançou enquanto a CPU atendia a IRQ.
 *          O registrador de captura contém o timestamp EXATO da borda.
 *     3. delta_ticks = (uint32_t)(capture_now - prev_capture)  ← aritmética circular
 *        Correto no TIM2 32-bit sem ISR de overflow.
 *     4. Conversão auxiliar para nanossegundos: period_ns = delta_ticks × 4
 *        Captura CKP: TIM2 @ 250 MHz, PSC=0 → 4 ns/tick
 *     5. Cálculo de médias → classificação: GAP | NORMAL_TOOTH | NOISE
 *     6. Atualização da máquina de estados (Módulo 2)
 *     7. Disparo dos hooks sensors_on_tooth() / schedule_on_tooth()
 *
 * VANTAGEM DO INPUT CAPTURE vs GPIO/EXTI:
 *   O periférico de captura registra o timestamp da borda em hardware no exato
 *   instante do evento, independente do atraso de atendimento da IRQ
 *   (tipicamente 12–20 ciclos = 0,1–0,17 µs @ 120 MHz no Cortex-M4).
 *   A 6000 RPM, 0,2 µs de jitter ≈ 0,07° — inaceitável sem input capture.
 *   Com input capture: resolução = 1 tick = 4 ns ≈ 0,0015° @ 6000 RPM.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * MÓDULO 2: SYNC — Máquina de Estados
 * ─────────────────────────────────────
 *   Roda fônica 60-2: 60 posições × 6°; 2 dentes ausentes consecutivos.
 *   O gap ocorre 1× por revolução; a ISR identifica-o por razão de período.
 *
 *   Estados (enum SyncState — definido em ckp.h):
 *     WAIT         → inicial / pós-falha: aguarda qualquer gap
 *     SYNCING      → 1º gap detectado; contando dentes para confirmar
 *     SYNCED       → 2º gap na posição correta: tooth_index válido
 *     LOSS_OF_SYNC → gap ausente por >61 dentes, ou gap prematuro (<55 dentes)
 *
 *   Transições:
 *     WAIT         + gap              → SYNCING   (tooth_count reset=0)
 *     SYNCING      + gap, count≥55    → SYNCED    (tooth_index reset=0)
 *     SYNCING      + gap, count<55    → LOSS_OF_SYNC (pulso espúrio)
 *     SYNCING      + count>61         → LOSS_OF_SYNC (gap ausente)
 *     SYNCED       + gap, count≥55    → SYNCED    (gap confirmado, reinicia)
 *     SYNCED       + gap, count<55    → LOSS_OF_SYNC (wheel slip / ruído)
 *     SYNCED       + count>61         → LOSS_OF_SYNC (gap ausente)
 *     LOSS_OF_SYNC + gap              → SYNCING   (tentativa re-sync)
 *
 * FILTRO DINÂMICO ±20% (rejeição de ruído):
 *   Dentes normais aceitos apenas se:  0,8×avg ≤ period ≤ 1,2×avg
 *   Gap identificado por razão:        period × 2 > avg × 3  (≡ period > 1,5×avg)
 *   Período fora das duas faixas:      descartado como ruído (hist não atualizado)
 *
 *   Para roda 60-2: gap ≈ 3×normal → 3,0 >> 1,5: margem 100% acima do limiar.
 *   O filtro ±20% cobre desaceleração típica (<15%/dente no ciclo NEDC).
 */

#include "drv/ckp.h"

#include <cstdint>

// ── Remapeamento de registradores para STM32H562 ─────────────────────────────
// Capture CH0 → TIM2_CCR1 (0x40000034), CH1 → TIM2_CCR2 (0x40000038)
// GPIO input mirror → GPIOA_IDR (0x42020010)
// TIM2 @ 250 MHz (PSC=0) → 4 ns/tick
#define TICKS_TO_NS_FACTOR  4000u
#define TICKS_TO_NS_DIVISOR 1000u

// ── Mock de registradores para testes host ───────────────────────────────────
#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_ckp_capture_ch0 = 0u;
volatile uint32_t ems_test_ckp_capture_ch1 = 0u;
volatile uint32_t ems_test_ckp_gpio_idr = 0u;
#endif

// ── FIX-15: FASTRUN — coloca ISRs críticas em SRAM (zero cache miss) ─────────
// Em Teensyduino, FASTRUN = __attribute__((section(".fastrun"))) — definido em
// WProgram.h / core_pins.h. Em host tests a macro é indefinida; defini-la vazia
// garante que o código compile sem modificações.
#if !defined(FASTRUN)
#  if defined(TARGET_STM32H562) && !defined(EMS_HOST_TEST)
#    define FASTRUN  __attribute__((optimize("O3")))
#  else
#    define FASTRUN
#  endif
#endif

namespace {

// ── Constantes da roda fônica 60-2 ───────────────────────────────────────────

// 60-2: 60 posições, 2 dentes ausentes consecutivos = 58 dentes reais.
// Espaçamento por posição: 360°/60 = 6,0°.
// Gap: 3 posições ausentes × 6° = 18° ≈ 3× período normal.
static constexpr uint16_t kRealTeethPerRev    = 58u;  // dentes físicos — uso exclusivo da máquina de estados (gap detection)
static constexpr uint16_t kTeethPositionsPerRev = 60u; // posições angulares uniformes — uso em cálculos de RPM e ângulo

// Ângulo por dente normal em miligraus (× 1000).
// 6,0° × 1000 = 6000. Usado em ckp_angle_to_ticks().
// NOTA: o nome kToothAngleX1000 sugere ×1000, que é "miligraus".
//   angle_x10 passado à função é TAMBÉM em miligraus (não em ×10 como o nome diz).
static constexpr uint16_t kToothAngleX1000 = 6000u;

// Mínimo de dentes contados desde o último gap para aceitar novo gap.
// 55 << 58: descarta pulsos espúrios no início de cada revolução.
static constexpr uint16_t kGapThresholdTooth  = 55u;

// Máximo de dentes sem gap antes de declarar LOSS_OF_SYNC.
// SCH-04: margem aumentada de 60 para 63 (58 real + 5 de margem).
// A margem anterior de 2 dentes era insuficiente durante desaceleração brusca:
// se o filtro ±20% rejeitar 2 dentes consecutivos como "muito lentos" (período
// crescendo > 20%/dente), o contador de dentes válidos ficaria curto de 58,
// disparando LOSS_OF_SYNC espuriamente ("tropeço" em desaceleração agressiva).
// Com 5 dentes de margem, suporta até 5 rejeições consecutivas por ruído antes
// de declarar perda — cobre condições normais de desaceleração em estrada.
static constexpr uint16_t kMaxTeethBeforeLoss = 63u;

// ── Limiares do filtro ───────────────────────────────────────────────────────
// Detecção de gap por razão:  period × kDen > avg × kNum  ≡  period > 1,5 × avg
// Para gap 60-2 (≈3×normal): separação real ≈ 3,0 >> 1,5 → margem robusta.
static constexpr uint32_t kGapRatioNum = 3u;
static constexpr uint32_t kGapRatioDen = 2u;

// Tolerância ±20% para dentes normais:
//   period ∈ [avg × 4/5, avg × 6/5]  →  dente aceito
//   period fora desta faixa e < limiar de gap  →  ruído, descartado
static constexpr uint32_t kTolNumLow  = 4u;   // 80% = 4/5
static constexpr uint32_t kTolDenLow  = 5u;
static constexpr uint32_t kTolNumHigh = 6u;   // 120% = 6/5
static constexpr uint32_t kTolDenHigh = 5u;

// Tamanho da janela deslizante de histórico (em número de períodos).
// 3 amostras são suficientes para filtrar ruído transitório.
static constexpr uint8_t kHistSize = 3u;

// ── Acesso a registradores de captura ────────────────────────────────────────
// CRÍTICO: Lemos o registrador de CAPTURA travado pelo hardware,
//   não o contador livre que avançou durante o atendimento da ISR.
//   Esta distinção elimina o jitter de software: o valor capturado reflete o
//   exato instante da borda de subida, independente da latência da IRQ.
//   Referência: RusEFI issue #1488 ("timestamp corruption from CNT vs CnV").
#if defined(EMS_HOST_TEST)
#define CKP_CAPTURE_CH0      ems_test_ckp_capture_ch0
#define CKP_CAPTURE_CH1      ems_test_ckp_capture_ch1
#define CKP_GPIO_IDR         ems_test_ckp_gpio_idr
#else
// STM32H562: TIM2_CCR1 (TIM2 base 0x40000000, CCR1 offset 0x34)
#define CKP_CAPTURE_CH0      (*reinterpret_cast<volatile uint32_t*>(0x40000034u))
// STM32H562: TIM2_CCR2 (CCR2 offset 0x38)
#define CKP_CAPTURE_CH1      (*reinterpret_cast<volatile uint32_t*>(0x40000038u))
// GPIOA_IDR (GPIOA base 0x42020000, IDR offset 0x10) — verificação anti-glitch PA0
#define CKP_GPIO_IDR         (*reinterpret_cast<volatile uint32_t*>(0x42020010u))
#endif

// ── Estado interno do decodificador ──────────────────────────────────────────
//
// INVARIANTE DE ACESSO — NUNCA VIOLAR:
//   g_state é escrito EXCLUSIVAMENTE pela ISR ckp_capture_primary_isr() (prioridade 1).
//   Qualquer outro contexto (main loop, ISRs de prioridade < 1) DEVE usar
//   ckp_snapshot() para ler g_state.snap — que aplica seção crítica CPSID/CPSIE.
//
//   Acessar g_state.snap diretamente fora da ISR de prioridade 1 é PROIBIDO
//   porque a leitura pode observar um snapshot parcialmente actualizado
//   (ex: tooth_index actualizado mas rpm_x10 ainda com valor anterior).
//
//   Se uma nova ISR de prioridade < 1 for adicionada e precisar de dados CKP,
//   ela DEVE chamar ckp_snapshot() ou ser elevada para prioridade 1 (com
//   revisão cuidadosa das implicações de latência para as demais ISRs).
struct DecoderState {
    ems::drv::CkpSnapshot snap;
    // TIM2 é 32-bit; o delta usa aritmética circular uint32_t diretamente.
    uint32_t prev_capture;              // último timestamp TIM2_CCR1 (para delta circular)
    uint32_t tooth_hist[kHistSize];     // janela deslizante de períodos (ticks) — dentes normais
    uint8_t  hist_ready;                // quantas entradas válidas em tooth_hist (máx kHistSize)
    uint16_t tooth_count;               // dentes desde o último gap aceito
    uint8_t  cmp_confirms;              // confirmações do cam sensor (CH1)
};

static DecoderState g_state = {
    ems::drv::CkpSnapshot{0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT, false},
    0u,
    {0u, 0u, 0u},
    0u,
    0u,
    0u,
};
// FIX-5: volatile nas variáveis escritas pela ISR de captura CKP (prio 1) e lidas pelo
// background loop sem seção crítica. Sem volatile, o compilador pode elevar
// as leituras para fora de loops ou cacheá-las em registradores, observando
// valores desatualizados. volatile força um fresh load de memória a cada acesso.
static volatile bool g_seed_armed = false;
static volatile bool g_seed_phase_a = false;
static volatile bool g_seed_probation = false;
static volatile uint16_t g_seed_probation_teeth = 0u;
static volatile uint32_t g_seed_loaded_count = 0u;
static volatile uint32_t g_seed_confirmed_count = 0u;
static volatile uint32_t g_seed_rejected_count = 0u;
static constexpr uint16_t kSeedCamConfirmMaxTeeth = 70u;

// ── Utilitários inline ────────────────────────────────────────────────────────

// Conversão auxiliar de ticks TIM2 para nanossegundos. O contrato principal v2.2
// usa tooth_period_ticks; period_ns permanece apenas como derivado local.
inline uint32_t ticks_to_ns(uint32_t ticks) noexcept {
#if defined(TICKS_TO_NS_FACTOR) && (TICKS_TO_NS_FACTOR == 4000u) && (TICKS_TO_NS_DIVISOR == 1000u)
    return ticks * 4u;
#elif defined(TICKS_TO_NS_FACTOR)
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ticks) * TICKS_TO_NS_FACTOR) / TICKS_TO_NS_DIVISOR);
#else
    return ticks * 4u;
#endif
}

// ⚡ CORREÇÃO C1/C7: Numerador correto da fórmula de RPM
// Fonte: OpenEMS v2.2 Prompt 2, linha 29:
//   rpm_x10 = (uint32_t)(1500000000000ULL / (58ULL * tooth_period_ticks))
// Simplificação: 1500000000000 / 58 = 25862068.97 ≈ 25862068
// O valor anterior (2500000000) estava ~96.7× maior que o correto.
static constexpr uint64_t kRpmNumeratorTicks = 25862068ULL;
inline uint32_t rpm_x10_from_period_ticks(uint32_t period_ticks) noexcept {
    if (period_ticks == 0u) { return 0u; }
    return static_cast<uint32_t>(kRpmNumeratorTicks / period_ticks);
}

// Insere novo período na janela deslizante (shift FIFO).
// Chamado APENAS para períodos aceitos como dente normal.
inline void hist_push(uint32_t period_ticks) noexcept {
    g_state.tooth_hist[2] = g_state.tooth_hist[1];
    g_state.tooth_hist[1] = g_state.tooth_hist[0];
    g_state.tooth_hist[0] = period_ticks;
    if (g_state.hist_ready < kHistSize) {
        ++g_state.hist_ready;
    }
}

// Média dos períodos no histórico (em ticks).
// Retorna 1 para evitar divisão por zero antes do histórico estar pronto.
inline uint32_t hist_avg() noexcept {
    if (g_state.hist_ready == 0u) { return 1u; }
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < g_state.hist_ready; ++i) {
        sum += g_state.tooth_hist[i];
    }
    return sum / static_cast<uint32_t>(g_state.hist_ready);
}

// Teste de gap por razão (sem divisão — operação pura de multiplicação).
// Equivalente a: period > 1,5 × avg   →   period × 2 > avg × 3
inline bool is_gap(uint32_t period, uint32_t avg) noexcept {
    return (period * kGapRatioDen > avg * kGapRatioNum);
}

// Teste de dente normal dentro da janela de tolerância ±20%.
// 0,8×avg ≤ period ≤ 1,2×avg → dente aceito para atualização do histórico.
inline bool is_normal_tooth(uint32_t period, uint32_t avg) noexcept {
    const uint32_t lo = (avg * kTolNumLow)  / kTolDenLow;   // 80% de avg
    const uint32_t hi = (avg * kTolNumHigh) / kTolDenHigh;  // 120% de avg
    return (period >= lo && period <= hi);
}

// ── Seção crítica ARM Cortex-M4 ──────────────────────────────────────────────
// CPSID I: mascara todas as interrupções maskable (PRIMASK=1).
// Uso: proteger leitura coerente de g_state.snap pelo main loop (ckp_snapshot).
// A própria ISR CKP (prioridade 1) NÃO usa seção crítica interna.
inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

// ── Processamento de gap na máquina de estados ───────────────────────────────
// Chamado pela ISR quando period > 1,5 × avg E tooth_count satisfaz a condição.
// Retorna true se o gap foi aceito (transição válida).
inline bool process_gap_event(uint32_t capture_now) noexcept {
    switch (g_state.snap.state) {

        case ems::drv::SyncState::WAIT:
        case ems::drv::SyncState::LOSS_OF_SYNC:
            // Em qualquer estado de "não sincronizado", um gap inicia a tentativa de sync.
            //
            // CORREÇÃO CKP-02: exigir tooth_count >= kGapThresholdTooth mesmo aqui.
            // Durante os primeiros kHistSize dentes (bootstrap do histórico), o filtro
            // de razão ainda não está activo e tooth_count pode ser < 55. Um spike de
            // EMC que gere um período longo (aparentemente gap) sem que tenham passado
            // dentes suficientes não deve avançar o estado — seria uma sincronização falsa.
            //
            // Excepção: no estado puro WAIT com histórico ainda não preenchido
            // (hist_ready < kHistSize), a ISR retorna antes de chegar aqui, portanto
            // qualquer gap que chegue a process_gap_event() já passou pelo filtro externo
            // ou é o primeiro evento após hist_ready == kHistSize. A guarda abaixo cobre
            // o caso em que tooth_count foi incrementado mas ainda está abaixo do limiar.
            if (g_state.tooth_count < kGapThresholdTooth) {
                // Gap prematuro: < 55 dentes desde o último reset de contagem.
                // Pode ser spike de EMC, ruído mecânico ou arranque com histórico parcial.
                // Reseta a contagem mas mantém o estado — aguarda gap legítimo.
                g_state.tooth_count = 0u;
                return false;
            }
            if (g_seed_armed) {
                g_state.snap.state = ems::drv::SyncState::SYNCED;
                g_state.snap.phase_A = g_seed_phase_a;
                g_seed_armed = false;
                g_seed_probation = true;
                g_seed_probation_teeth = 0u;
            } else {
                g_state.snap.state = ems::drv::SyncState::SYNCING;
            }
            g_state.tooth_count         = 0u;
            g_state.snap.tooth_index    = 0u;
            g_state.snap.last_tim2_capture = capture_now;
            return true;

        case ems::drv::SyncState::SYNCING:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // 2º gap na posição correta → sincronismo completo.
                // A partir deste ponto, tooth_index rastreia a posição angular
                // com resolução de 6° por dente.
                g_state.snap.state          = ems::drv::SyncState::SYNCED;
                g_state.tooth_count         = 0u;
                g_state.snap.tooth_index    = 0u;
                g_state.snap.last_tim2_capture = capture_now;
                return true;
            }
            // Gap antes de kGapThresholdTooth dentes: pulso espúrio (EMC, dente danificado).
            // Cai para LOSS_OF_SYNC e aguarda novo gap para tentar outra vez.
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            return false;

        case ems::drv::SyncState::SYNCED:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // Gap na posição esperada → mantém SYNCED, reinicia contagem.
                g_state.tooth_count         = 0u;
                g_state.snap.tooth_index    = 0u;
                g_state.snap.last_tim2_capture = capture_now;
                return true;
            }
            // Gap inesperado: wheel slip, dente duplo, interferência severa.
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            return false;

        default:
            return false;
    }
}

}  // namespace

// ── Símbolos fracos (hooks) ───────────────────────────────────────────────────
namespace ems::drv {

#if defined(__GNUC__)
__attribute__((weak))
#endif
void sensors_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

#if defined(__GNUC__)
__attribute__((weak))
#endif
void schedule_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

#if defined(__GNUC__)
__attribute__((weak))
#endif
void prime_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

}  // namespace ems::drv

// ── API pública ───────────────────────────────────────────────────────────────
namespace ems::drv {

CkpSnapshot ckp_snapshot() noexcept {
    CkpSnapshot out;
    enter_critical();
    out = g_state.snap;
    exit_critical();
    return out;
}

uint32_t ckp_angle_to_ticks(uint16_t angle_x10, uint32_t ref_capture) noexcept {
    static constexpr uint32_t kToothAngleX10 = 60u;
    // ⚡ CORREÇÃO M3: Guard de divisão por zero
    if (g_state.snap.tooth_period_ticks == 0u) {
        return ref_capture;  // sem dados de período → retorna referência sem avanço
    }
    const uint32_t delta = (static_cast<uint32_t>(angle_x10) *
                            g_state.snap.tooth_period_ticks) / kToothAngleX10;
    return ref_capture + delta;  // wrap-around uint32_t correto para aritmética de ticks
}

// ── ISR do CKP: canal primário de captura (rising edge) ──────────────────────
//
// CONTEXTO: chamada pelo handler de captura em hal/tim.cpp, NVIC prioridade 1.
// Não há chamada direta por código de usuário.
//
// Setup: canal de captura por borda de subida, configurado em hal/tim.cpp.
//
// VANTAGEM vs GPIO/EXTI:
//   O periférico de captura trava o valor do contador no registrador de captura no exato
//   instante da borda de subida, em hardware. A CPU pode atender a IRQ
//   vários ciclos depois — o timestamp em C0V permanece válido.
//   Isso é impossível com GPIO/EXTI onde a CPU leria o contador atual (atrasado).
FASTRUN void ckp_capture_primary_isr() noexcept {
    // ── 1. Timestamp sem jitter de ISR ────────────────────────────────────
    // CRÍTICO: lemos o registrador de captura ANTES de qualquer outra operação.
    // O registrador de captura foi travado pelo HW no instante exato da borda;
    // leituras posteriores de GPIO etc. não afetam o valor capturado.
    // NÃO ler o contador livre: ele avançou durante a latência de IRQ.
    //
    const uint32_t capture_now = CKP_CAPTURE_CH0;  // TIM2_CCR1: timestamp 32-bit travado pelo HW

    // ── 2. Delta de ticks (aritmética circular uint32_t) ──────────────────
    // Subtração circular unsigned: correto mesmo com overflow do contador 32-bit.
    // Ex: 0x00000005 - 0xFFFFFFFD = 0x00000008 (delta = 8) ✓
    const uint32_t delta_ticks = capture_now - g_state.prev_capture;

    // ── 3. Anti-glitch por período mínimo ────────────────────────────────
    // Estratégia preferida a checar o GPIO depois da captura: a 250 MHz
    // o pino pode ter retornado LOW antes da CPU ler o registrador de GPIO,
    // descartando capturas legítimas em alta rotação (> 4000 RPM).
    // Período mínimo aceitável: dente a 20000 RPM → 250 MHz / (58 * 20000/60)
    //   ≈ 3228 ticks; usamos 50 como limite inferior conservador para
    //   rejeitar glitches de EMC (< ~800 ns) sem afetar operação normal.
    static constexpr uint32_t kMinToothTicks = 50u;
    if (delta_ticks < kMinToothTicks) {
        return;  // pulso muito curto → ruído EMC, descartar
    }

    g_state.prev_capture                = capture_now;
    g_state.snap.last_tim2_capture      = capture_now;

    const uint32_t period_ticks = delta_ticks;

    // ── 4. Construção do histórico (primeiros kHistSize dentes) ───────────
    // Antes do histórico estar completo (hist_ready < kHistSize), aceitamos
    // todos os dentes incondicionalmente para inicializar a janela de média.
    // Não há referência de velocidade ainda para filtrar.
    if (g_state.hist_ready < kHistSize) {
        hist_push(delta_ticks);
        ++g_state.tooth_count;
        g_state.snap.tooth_period_ticks = period_ticks;
        g_state.snap.rpm_x10 = rpm_x10_from_period_ticks(period_ticks);
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 5. Classificação do evento por razão ──────────────────────────────
    // Calcula média dos últimos kHistSize períodos normais (em ticks).
    // avg é computado em ticks (não ns) para evitar conversão adicional.
    const uint32_t avg = hist_avg();

    // ── 5a. Teste de GAP ──────────────────────────────────────────────────
    // Condição: delta_ticks × 2 > avg × 3  ≡  delta > 1,5 × avg
    // Para 60-2: gap ≈ 3 × avg → ratio ≈ 3,0 >> 1,5 (margem de 100%)
    // O limiar 1,5× separa o gap de qualquer dente normal válido (máx 120% de avg).
    //
    // NOTA (CKP-02): A verificação de tooth_count >= kGapThresholdTooth é feita
    // DENTRO de process_gap_event() para todos os estados, incluindo WAIT e
    // LOSS_OF_SYNC. Isso garante proteção uniforme contra gaps prematuros durante
    // bootstrap do histórico (hist_ready < kHistSize) onde o filtro de razão acima
    // ainda não estava activo nas iterações anteriores.
    if (is_gap(delta_ticks, avg)) {
        // Delega para a máquina de estados — a validação de tooth_count
        // e todas as transições estão encapsuladas em process_gap_event().
        static_cast<void>(process_gap_event(capture_now));
        // Dispara hooks mesmo após gap: permite ao agendador / sensores reagir
        // à mudança de estado (ex: cancelar eventos pendentes após LOSS_OF_SYNC).
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 5b. Filtro ±20%: dente normal vs ruído ────────────────────────────
    // Período dentro de [80%, 120%] do avg → dente normal válido.
    // Fora desta janela (mas < limiar de gap) → ruído transitório: descartar.
    if (!is_normal_tooth(delta_ticks, avg)) {
        // Ruído: não atualiza hist, não incrementa tooth_count.
        // Apenas dispara hooks para que camadas superiores possam monitorar.
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 6. Processamento de dente normal ──────────────────────────────────
    hist_push(delta_ticks);

    g_state.snap.tooth_period_ticks = period_ticks;
    g_state.snap.rpm_x10 = rpm_x10_from_period_ticks(period_ticks);

    // Incrementa tooth_count e tooth_index apenas em estados sincronizados/tentando.
    if (g_state.snap.state != ems::drv::SyncState::WAIT &&
        g_state.snap.state != ems::drv::SyncState::LOSS_OF_SYNC) {
        ++g_state.tooth_count;
        // tooth_index: posição angular dentro da revolução (0 = após gap, 57 = último dente)
        g_state.snap.tooth_index =
            (g_state.snap.tooth_index < (kRealTeethPerRev - 1u))
                ? static_cast<uint16_t>(g_state.snap.tooth_index + 1u)
                : 0u;
    } else {
        // Em WAIT / LOSS_OF_SYNC: incrementa tooth_count para detectar
        // o limiar mínimo de dentes antes de aceitar o próximo gap.
        ++g_state.tooth_count;
    }

    // ── 7. Verificação de perda de sincronia por contagem excessiva ───────
    // Se passaram mais de kMaxTeethBeforeLoss dentes sem um gap:
    //   → o gap foi perdido (interferência, aceleração brusca, falha de sensor)
    if (g_state.tooth_count > kMaxTeethBeforeLoss) {
        if (g_state.snap.state == ems::drv::SyncState::SYNCING ||
            g_state.snap.state == ems::drv::SyncState::SYNCED) {
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
        }
    }

    // ── 8. Hooks ──────────────────────────────────────────────────────────
    if (g_seed_probation) {
        ++g_seed_probation_teeth;
        if (g_seed_probation_teeth > kSeedCamConfirmMaxTeeth) {
            // Seed could not be validated by cam edge in time: fallback to safe sync path.
            g_seed_probation = false;
            g_seed_probation_teeth = 0u;
            ++g_seed_rejected_count;
            g_state.snap.state = ems::drv::SyncState::SYNCING;
            g_state.tooth_count = 0u;
            g_state.snap.tooth_index = 0u;
        }
    }
    sensors_on_tooth(g_state.snap);
    schedule_on_tooth(g_state.snap);
    prime_on_tooth(g_state.snap);
}

// ── ISR do cam sensor: canal secundário de captura (rising edge) ────────────
// Cada borda de subida do cam sensor indica meio ciclo de motor (180° de virabrequim).
// phase_A alterna para permitir ao agendador identificar qual par de cilindros está
// no tempo de injeção (cilindros 1/4 vs 2/3 para motor 4 cilindros em linha).
FASTRUN void ckp_capture_secondary_isr() noexcept {
    if ((CKP_GPIO_IDR & (1u << 1u)) == 0u) {
        return;  // anti-glitch: apenas rising edges reais
    }
    static_cast<void>(CKP_CAPTURE_CH1);   // leitura limpa o flag do canal
    g_state.snap.phase_A = !g_state.snap.phase_A;
    if (g_seed_probation) {
        g_seed_probation = false;
        g_seed_probation_teeth = 0u;
        ++g_seed_confirmed_count;
    }
    if (g_state.cmp_confirms < 2u) {
        ++g_state.cmp_confirms;
    }
}

void ckp_seed_arm(bool phase_A) noexcept {
    g_seed_armed = true;
    g_seed_phase_a = phase_A;
    ++g_seed_loaded_count;
}

void ckp_seed_disarm() noexcept {
    g_seed_armed = false;
}

uint32_t ckp_seed_loaded_count() noexcept {
    return g_seed_loaded_count;
}

uint32_t ckp_seed_confirmed_count() noexcept {
    return g_seed_confirmed_count;
}

uint32_t ckp_seed_rejected_count() noexcept {
    return g_seed_rejected_count;
}

// ── API de teste (host only) ──────────────────────────────────────────────────
#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept {
    g_state = DecoderState{
        CkpSnapshot{0u, 0u, 0u, 0u, SyncState::WAIT, false},
        0u,
        {0u, 0u, 0u},
        0u,
        0u,
        0u,
    };
    ems_test_ckp_capture_ch0 = 0u;
    ems_test_ckp_capture_ch1 = 0u;
    ems_test_ckp_gpio_idr = 0u;
    g_seed_armed = false;
    g_seed_phase_a = false;
    g_seed_probation = false;
    g_seed_probation_teeth = 0u;
    g_seed_loaded_count = 0u;
    g_seed_confirmed_count = 0u;
    g_seed_rejected_count = 0u;
}

uint32_t ckp_test_rpm_x10_from_period_ticks(uint32_t period_ticks) noexcept {
    return rpm_x10_from_period_ticks(period_ticks);
}
#endif

}  // namespace ems::drv

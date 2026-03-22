#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "app/can_stack.h"
#include "hal/fdcan.h"

namespace {

int g_tests_run    = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d  cond: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d  expected=%u  got=%u\n", \
                    __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

// ─── helpers ────────────────────────────────────────────────────────────────

ems::drv::CkpSnapshot make_ckp(uint32_t rpm_x10) {
    return ems::drv::CkpSnapshot{0u, 0u, 0u, rpm_x10,
                                  ems::drv::SyncState::SYNCED, false};
}

ems::drv::SensorData make_sensors() {
    return ems::drv::SensorData{
        998u,    // map_kpa_x10
        0u,      // maf_gps_x100
        457u,    // tps_pct_x10
        930,     // clt_degc_x10   →  93 °C
        250,     // iat_degc_x10   →  25 °C
        3510u,   // fuel_press_kpa_x10
        4020u,   // oil_press_kpa_x10
        13800u,  // vbatt_mv
        0u,      // fault_bits
        0u       // o2_mv
    };
}

// ─── test 1: bit timing ──────────────────────────────────────────────────────

// test 1: frame com ID ≠ 0x180 não deve atualizar lambda nem limpar fault.
// Valida defesa-em-profundidade no software: can_stack_rx() deve ignorar IDs
// inesperados mesmo que o filtro hardware (SFEC/ANFS) falhe por regressão.
void test_rx_ignores_non_wbo2_id() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(5000u);
    const ems::drv::SensorData  sensors = make_sensors();

    // Injetar frame com ID=0x181 (não é o WBO2 0x180), dados plausíveis de λ
    ems::hal::FdcanFrame intruder{};
    intruder.id       = 0x181u;
    intruder.dlc_bytes = 3u;
    intruder.extended = false;
    intruder.brs      = false;
    intruder.data[0]  = 0xE8u;  // λ=1000 milli se fosse 0x180
    intruder.data[1]  = 0x03u;
    intruder.data[2]  = 0x01u;
    TEST_ASSERT_TRUE(ems::hal::fdcan_test_inject_rx(intruder));

    // can_stack_process() consome a fila RX internamente
    ems::app::can_stack_process(50u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0x00u);

    // Lambda deve permanecer no valor seguro e fault ativo — frame intruso ignorado
    TEST_ASSERT_EQ_U32(ems::app::WBO2_SAFE_LAMBDA_MILLI,
                       ems::app::can_stack_lambda_milli());
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fault());
}

// ─── test 2: serialização 0x400 CAN-FD 48 bytes ─────────────────────────────

void test_tx_0x400_serialization() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(32150u);
    const ems::drv::SensorData  sensors = make_sensors();

    // WBO2 nunca recebeu frame → fault deve estar ativo
    ems::app::can_stack_process(10u, ckp, sensors,
        12,    // advance_deg
        45u,   // pw_ms_x10
        -3,    // stft_pct
        0u, 0u,
        0x25u  // status_bits do chamador (sem bit7)
    );

    ems::hal::FdcanFrame out = {};
    TEST_ASSERT_TRUE(ems::hal::fdcan_test_pop_tx(out));
    TEST_ASSERT_EQ_U32(0x400u, out.id);
    TEST_ASSERT_EQ_U32(48u,     out.dlc_bytes);
    TEST_ASSERT_EQ_U32(3215u & 0xFFu,         out.data[0]);
    TEST_ASSERT_EQ_U32((3215u >> 8u) & 0xFFu, out.data[1]);
    TEST_ASSERT_EQ_U32(998u & 0xFFu,          out.data[2]);
    TEST_ASSERT_EQ_U32((998u >> 8u) & 0xFFu,  out.data[3]);
    TEST_ASSERT_EQ_U32(457u & 0xFFu,          out.data[4]);
    TEST_ASSERT_EQ_U32((457u >> 8u) & 0xFFu,  out.data[5]);
    TEST_ASSERT_EQ_U32(930u & 0xFFu,          out.data[6]);
    TEST_ASSERT_EQ_U32((930u >> 8u) & 0xFFu,  out.data[7]);
    TEST_ASSERT_EQ_U32(120u, out.data[10]);   // advance_x10
    TEST_ASSERT_EQ_U32(0x94u, out.data[12]);  // pw_us = 4500
    TEST_ASSERT_EQ_U32(0x11u, out.data[13]);
    TEST_ASSERT_EQ_U32(0xE2u, out.data[16]);  // stft -3.0% -> -30 int16
    TEST_ASSERT_EQ_U32(0xFFu, out.data[17]);
    TEST_ASSERT_EQ_U32(0x25u | ems::app::STATUS_WBO2_FAULT, out.data[36]);
}

void test_tx_0x400_carries_pressures_and_uptime() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(10000u);
    const ems::drv::SensorData  sensors = make_sensors();

    ems::app::can_stack_process(100u, ckp, sensors, 0, 0u, -7, 32u, 64u, 0u);

    ems::hal::FdcanFrame out = {};
    TEST_ASSERT_TRUE(ems::hal::fdcan_test_pop_tx(out));
    TEST_ASSERT_EQ_U32(0x400u, out.id);
    TEST_ASSERT_EQ_U32(3510u & 0xFFu,          out.data[18]);
    TEST_ASSERT_EQ_U32((3510u >> 8u) & 0xFFu,  out.data[19]);
    TEST_ASSERT_EQ_U32(4020u & 0xFFu,          out.data[20]);
    TEST_ASSERT_EQ_U32((4020u >> 8u) & 0xFFu,  out.data[21]);
    TEST_ASSERT_EQ_U32(13800u & 0xFFu,         out.data[22]);
    TEST_ASSERT_EQ_U32((13800u >> 8u) & 0xFFu, out.data[23]);
    TEST_ASSERT_EQ_U32(100u, out.data[38]);    // uptime_ms LSB
}

// ─── test 4: RX WBO2 + safe lambda + timeout ─────────────────────────────────

void test_rx_wbo2_safe_lambda_and_timeout() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(9000u);
    const ems::drv::SensorData  sensors = make_sensors();

    // Estado inicial: nenhum frame recebido → fault ativo, lambda seguro
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fault());
    TEST_ASSERT_EQ_U32(ems::app::WBO2_SAFE_LAMBDA_MILLI,
                       ems::app::can_stack_lambda_milli_safe(0u));

    // Injeta frame WBO2: lambda = 0x041A = 1050
    ems::hal::FdcanFrame in = {};
    in.id       = 0x180u;
    in.dlc_bytes = 8u;
    in.extended = false;
    in.brs      = false;
    in.data[0]  = 0x1Au;
    in.data[1]  = 0x04u;   // 1050 little-endian
    in.data[2]  = 0x55u;   // status WBO2 externo
    TEST_ASSERT_TRUE(ems::hal::fdcan_test_inject_rx(in));

    // Processa em t=100 ms
    ems::app::can_stack_process(100u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0u);

    // Sensor fresco → lambda raw = 1050, safe = 1050, sem fault
    TEST_ASSERT_EQ_U32(1050u, ems::app::can_stack_lambda_milli());
    TEST_ASSERT_EQ_U32(1050u, ems::app::can_stack_lambda_milli_safe(100u));
    TEST_ASSERT_EQ_U32(0x55u, ems::app::can_stack_wbo2_status());
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fresh(100u));
    TEST_ASSERT_TRUE(!ems::app::can_stack_wbo2_fault());

    // Ainda fresco em t=600 ms (600 - 100 = 500 ms, não excede)
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fresh(600u));
    TEST_ASSERT_EQ_U32(1050u, ems::app::can_stack_lambda_milli_safe(600u));

    // Timeout em t=601 ms (601 - 100 = 501 ms > 500 ms)
    TEST_ASSERT_TRUE(!ems::app::can_stack_wbo2_fresh(601u));
    // Sem fallback para narrow: retorna lambda fixo de segurança
    TEST_ASSERT_EQ_U32(ems::app::WBO2_SAFE_LAMBDA_MILLI,
                       ems::app::can_stack_lambda_milli_safe(601u));
}

// ─── test 5: status_bits — bit7 WBO2_FAULT reflete estado real ───────────────

void test_status_bits_wbo2_fault_flag() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(5000u);
    const ems::drv::SensorData  sensors = make_sensors();

    // t=10: sem frame → fault forçado no payload único
    ems::app::can_stack_process(10u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0x00u);
    ems::hal::FdcanFrame f = {};
    TEST_ASSERT_TRUE(ems::hal::fdcan_test_pop_tx(f));
    TEST_ASSERT_EQ_U32(0x400u, f.id);
    TEST_ASSERT_TRUE((f.data[36] & ems::app::STATUS_WBO2_FAULT) != 0u);

    // Injeta frame WBO2 e processa em t=20
    ems::hal::FdcanFrame in = {};
    in.id = 0x180u; in.dlc_bytes = 3u; in.extended = false; in.brs = false;
    in.data[0] = 0xE8u; in.data[1] = 0x03u; // 1000 = λ1.00
    in.data[2] = 0x01u;
    ems::hal::fdcan_test_inject_rx(in);

    ems::app::can_stack_process(20u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0x00u);
    while (ems::hal::fdcan_test_pop_tx(f)) {
        if (f.id == 0x400u) { break; }
    }
    TEST_ASSERT_EQ_U32(0x400u, f.id);
    TEST_ASSERT_TRUE((f.data[36] & ems::app::STATUS_WBO2_FAULT) == 0u);
}

void test_status_bits_scheduler_flags_passthrough() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(5000u);
    const ems::drv::SensorData  sensors = make_sensors();

    const uint8_t status = static_cast<uint8_t>(
        ems::app::STATUS_SYNC_FULL |
        ems::app::STATUS_SCHED_LATE |
        ems::app::STATUS_SCHED_DROP |
        ems::app::STATUS_SCHED_CLAMP);

    // Provide a fresh WBO2 frame so bit7 is not auto-forced.
    ems::hal::FdcanFrame in = {};
    in.id = 0x180u; in.dlc_bytes = 3u; in.extended = false; in.brs = false;
    in.data[0] = 0xE8u; in.data[1] = 0x03u;
    in.data[2] = 0x01u;
    ems::hal::fdcan_test_inject_rx(in);

    ems::app::can_stack_process(10u, ckp, sensors, 0, 0u, 0, 0u, 0u, status);

    ems::hal::FdcanFrame f = {};
    while (ems::hal::fdcan_test_pop_tx(f)) {
        if (f.id == 0x400u) { break; }
    }
    TEST_ASSERT_EQ_U32(0x400u, f.id);
    TEST_ASSERT_EQ_U32(status, f.data[36]);
}

} // namespace

int main() {
    test_rx_ignores_non_wbo2_id();
    test_tx_0x400_serialization();
    test_tx_0x400_carries_pressures_and_uptime();
    test_rx_wbo2_safe_lambda_and_timeout();
    test_status_bits_wbo2_fault_flag();
    test_status_bits_scheduler_flags_passthrough();

    std::printf("\ntests=%d  failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}

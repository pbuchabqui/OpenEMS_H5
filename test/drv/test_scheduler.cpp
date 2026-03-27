#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "drv/scheduler.h"
#include "hal/tim.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

void reset_all() {
    ems::hal::tim_test_clear_all();
    ems::drv::sched_init();
}

void test_tim1_routing_uses_low_16_bits() {
    reset_all();
    const uint32_t abs_ticks = 0x1234ABCDu;
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::INJ2, abs_ticks, ems::drv::Action::SET));
    TEST_ASSERT_EQ_U32(0xABCDu, ems::hal::tim_test_get_compare(1u, 1u));
    TEST_ASSERT_TRUE(ems::drv::sched_test_is_armed(ems::drv::Channel::INJ2));
}

void test_tim15_routing_programs_inj4() {
    reset_all();
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::INJ4, 0x00017777u, ems::drv::Action::CLEAR));
    TEST_ASSERT_EQ_U32(0x7777u, ems::hal::tim_test_get_compare(15u, 0u));
}

void test_tim8_routing_programs_ign_channels() {
    reset_all();
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::IGN3, 0x00015555u, ems::drv::Action::SET));
    TEST_ASSERT_EQ_U32(0x5555u, ems::hal::tim_test_get_compare(8u, 2u));
}

void test_cancel_disarms_channel() {
    reset_all();
    static_cast<void>(ems::drv::sched_event(ems::drv::Channel::IGN1, 0xA5A5u, ems::drv::Action::SET));
    ems::drv::sched_cancel(ems::drv::Channel::IGN1);
    TEST_ASSERT_TRUE(!ems::drv::sched_test_is_armed(ems::drv::Channel::IGN1));
}

void test_isr_disarms_timer_groups() {
    reset_all();
    static_cast<void>(ems::drv::sched_event(ems::drv::Channel::INJ1, 0x1111u, ems::drv::Action::SET));
    static_cast<void>(ems::drv::sched_event(ems::drv::Channel::INJ4, 0x2222u, ems::drv::Action::SET));
    static_cast<void>(ems::drv::sched_event(ems::drv::Channel::IGN4, 0x3333u, ems::drv::Action::SET));

    ems::drv::sched_isr_tim1();
    TEST_ASSERT_TRUE(!ems::drv::sched_test_is_armed(ems::drv::Channel::INJ1));
    TEST_ASSERT_TRUE(ems::drv::sched_test_is_armed(ems::drv::Channel::INJ4));

    ems::drv::sched_isr_tim15();
    TEST_ASSERT_TRUE(!ems::drv::sched_test_is_armed(ems::drv::Channel::INJ4));

    ems::drv::sched_isr_tim8();
    TEST_ASSERT_TRUE(!ems::drv::sched_test_is_armed(ems::drv::Channel::IGN4));
}

void test_recalc_is_noop_for_absolute_scheduler() {
    reset_all();
    const ems::drv::CkpSnapshot snap = {0u, 0u, 0u, 80000u, ems::drv::SyncState::SYNCED, false};
    static_cast<void>(ems::drv::sched_event(ems::drv::Channel::INJ3, 0x0000BABEu, ems::drv::Action::SET));
    ems::drv::sched_recalc(snap);
    TEST_ASSERT_EQ_U32(0xBABEu, ems::hal::tim_test_get_compare(1u, 2u));
    TEST_ASSERT_TRUE(ems::drv::sched_test_is_armed(ems::drv::Channel::INJ3));
}

/**
 * @brief Prompt 3, Entregável #2: Teste de conversão 32→16 bit para scheduler
 * 
 * Verifica que timestamps absolutos 32-bit são corretamente convertidos para 16-bit
 * via cast simples, graças à sincronização hardware TIM5→TIM1/TIM8 via ITR.
 */
void test_scheduler_32bit_to_16bit_conversion() {
    reset_all();
    
    // Testar vários valores 32-bit que devem ser convertidos corretamente
    const uint32_t test_values[] = {
        0x0000ABCDu,  // Valor baixo
        0x0001ABCDu,  // Valor com upper 16 bits
        0x1234ABCDu,  // Valor alto
        0xFFFFFFFFu,  // Overflow
        0x00000000u   // Zero
    };
    
    for (size_t i = 0; i < sizeof(test_values) / sizeof(test_values[0]); ++i) {
        const uint32_t abs_ticks = test_values[i];
        const uint16_t expected_hw = static_cast<uint16_t>(abs_ticks);
        
        static_cast<void>(ems::drv::sched_event(ems::drv::Channel::INJ1, abs_ticks, ems::drv::Action::SET));
        TEST_ASSERT_EQ_U32(expected_hw, ems::hal::tim_test_get_compare(1u, 0u));
        ems::drv::sched_cancel(ems::drv::Channel::INJ1);
    }
}

/**
 * @brief Teste de eventos no passado (CORREÇÃO C2/H8)
 * 
 * Verifica que eventos no passado são rejeitados pelo scheduler
 */
void test_scheduler_rejects_past_events() {
    reset_all();
    
    // Simular tempo atual
    ems::hal::tim_test_set_counter(5u, 1000u);
    
    // Evento no futuro: deve ser aceito
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::INJ1, 2000u, ems::drv::Action::SET));
    TEST_ASSERT_TRUE(ems::drv::sched_test_is_armed(ems::drv::Channel::INJ1));
    ems::drv::sched_cancel(ems::drv::Channel::INJ1);
    
    // Evento no passado: deve ser rejeitado
    TEST_ASSERT_TRUE(!ems::drv::sched_event(ems::drv::Channel::INJ1, 500u, ems::drv::Action::SET));
    TEST_ASSERT_TRUE(!ems::drv::sched_test_is_armed(ems::drv::Channel::INJ1));
}

}  // namespace

int main() {
    test_tim1_routing_uses_low_16_bits();
    test_tim15_routing_programs_inj4();
    test_tim8_routing_programs_ign_channels();
    test_cancel_disarms_channel();
    test_isr_disarms_timer_groups();
    test_recalc_is_noop_for_absolute_scheduler();
    test_scheduler_32bit_to_16bit_conversion();
    test_scheduler_rejects_past_events();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}

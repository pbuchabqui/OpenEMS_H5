#include "drv/scheduler.h"

#include "hal/tim.h"

namespace {

using ems::drv::Action;
using ems::drv::Channel;

struct Slot {
    uint32_t abs_ticks = 0u;
    Action action = Action::SET;
    bool armed = false;
};

Slot g_slots[8] = {};

constexpr uint8_t to_index(Channel ch) noexcept {
    return static_cast<uint8_t>(ch);
}

constexpr bool is_tim1(Channel ch) noexcept {
    return ch <= Channel::INJ3;
}

constexpr bool is_tim15(Channel ch) noexcept {
    return ch == Channel::INJ4;
}

constexpr uint8_t tim1_channel(Channel ch) noexcept {
    return to_index(ch);
}

constexpr uint8_t tim8_channel(Channel ch) noexcept {
    return static_cast<uint8_t>(to_index(ch) - to_index(Channel::IGN1));
}

void program_compare(Channel ch, uint32_t abs_ticks) noexcept {
    // Hardware sync via ITR cascade: TIM5→TIM1→TIM15, TIM5→TIM8
    // All slave timers' 16-bit counters match TIM5_CNT[15:0],
    // so a simple cast gives the correct compare value.
    const uint16_t hw_ticks = static_cast<uint16_t>(abs_ticks);
    if (is_tim1(ch)) {
        ems::hal::tim1_set_compare(tim1_channel(ch), hw_ticks);
    } else if (is_tim15(ch)) {
        ems::hal::tim15_set_compare(hw_ticks);
    } else {
        ems::hal::tim8_set_compare(tim8_channel(ch), hw_ticks);
    }
}

void clear_channel_flag(Channel ch) noexcept {
    if (is_tim1(ch)) {
        ems::hal::tim1_clear_ccf(tim1_channel(ch));
    } else if (is_tim15(ch)) {
        ems::hal::tim15_clear_ccf();
    } else {
        ems::hal::tim8_clear_ccf(tim8_channel(ch));
    }
}

void disarm(Channel ch) noexcept {
    g_slots[to_index(ch)].armed = false;
}

void service_group(Channel first, Channel last) noexcept {
    for (uint8_t idx = to_index(first); idx <= to_index(last); ++idx) {
        const Channel ch = static_cast<Channel>(idx);
        if (!g_slots[idx].armed) {
            continue;
        }
        clear_channel_flag(ch);
        g_slots[idx].armed = false;
    }
}

}  // namespace

namespace ems::drv {

void sched_init() noexcept {
    sched_cancel_all();
}

bool sched_event(Channel ch, uint32_t abs_ticks, Action act) noexcept {
    // ⚡ CORREÇÃO C2/H8: Rejeitar eventos no passado
    // Fonte: OpenEMS v2.2 Prompt 3, linhas 9-10:
    //   uint32_t delta = abs_ticks - tim5_count();
    //   if (delta > 0x80000000u) return false; // mais de 8.5s no passado
    // Sem esta verificação, eventos atrasados disparam após wrap de 16-bit (~65ms),
    // causando erro de timing de injeção/ignição de até 65ms.
    const uint32_t now = ems::hal::tim5_count();
    const uint32_t delta = abs_ticks - now;
    if (delta > 0x80000000u) {
        return false;  // evento no passado
    }

    Slot& slot = g_slots[to_index(ch)];
    slot.abs_ticks = abs_ticks;
    slot.action = act;
    slot.armed = true;
    program_compare(ch, abs_ticks);
    return true;
}

void sched_cancel(Channel ch) noexcept {
    disarm(ch);
}

void sched_cancel_all() noexcept {
    for (auto& slot : g_slots) {
        slot = {};
    }
}

void sched_isr_tim1() noexcept {
    service_group(Channel::INJ1, Channel::INJ3);
}

void sched_isr_tim15() noexcept {
    service_group(Channel::INJ4, Channel::INJ4);
}

void sched_isr_tim8() noexcept {
    service_group(Channel::IGN1, Channel::IGN4);
}

void sched_recalc(const CkpSnapshot&) noexcept {
    // v2.2 stores absolute TIM5 timestamps, so there is nothing to retime
    // when a new tooth arrives.
}

#if defined(EMS_HOST_TEST)
bool sched_test_is_armed(Channel ch) noexcept {
    return g_slots[to_index(ch)].armed;
}
#endif

}  // namespace ems::drv

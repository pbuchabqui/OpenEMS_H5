#pragma once

#include <cstdint>

#include "drv/ckp.h"

namespace ems::drv {

enum class Channel : uint8_t {
    INJ1 = 0,
    INJ2 = 1,
    INJ3 = 2,
    INJ4 = 3,
    IGN1 = 4,
    IGN2 = 5,
    IGN3 = 6,
    IGN4 = 7,
};

enum class Action : uint8_t {
    SET = 0,
    CLEAR = 1,
};

void sched_init() noexcept;
bool sched_event(Channel ch, uint32_t abs_ticks, Action act) noexcept;
void sched_cancel(Channel ch) noexcept;
void sched_cancel_all() noexcept;
void sched_isr_tim1() noexcept;
void sched_isr_tim15() noexcept;
void sched_isr_tim8() noexcept;
void sched_recalc(const CkpSnapshot& snap) noexcept;

#if defined(EMS_HOST_TEST)
bool sched_test_is_armed(Channel ch) noexcept;
#endif

}  // namespace ems::drv

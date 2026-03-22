#include "hal/cordic.h"

#if defined(EMS_HOST_TEST)

#include <cmath>

namespace {

constexpr int32_t kQ31One = 0x7FFFFFFF;
constexpr int16_t kQ15One = 32767;
constexpr double kPi = 3.14159265358979323846;

inline double wrap_turn(double turns) noexcept {
    while (turns < 0.0) {
        turns += 1.0;
    }
    while (turns >= 1.0) {
        turns -= 1.0;
    }
    return turns;
}

}  // namespace

namespace ems::hal {

void cordic_init() noexcept {}

void cordic_sincos_q31(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) noexcept {
    if (sin_out == nullptr || cos_out == nullptr) {
        return;
    }
    const double turns = wrap_turn(static_cast<double>(static_cast<uint32_t>(angle_q31)) / 4294967296.0);
    const double angle = turns * (2.0 * kPi);
    *sin_out = static_cast<int32_t>(std::lround(std::sin(angle) * static_cast<double>(kQ31One)));
    *cos_out = static_cast<int32_t>(std::lround(std::cos(angle) * static_cast<double>(kQ31One)));
}

void cordic_sincos_deg(uint16_t angle_deg_x10, int16_t* sin_out, int16_t* cos_out) noexcept {
    if (sin_out == nullptr || cos_out == nullptr) {
        return;
    }
    const double angle = (static_cast<double>(angle_deg_x10 % 3600u) / 10.0) * (kPi / 180.0);
    *sin_out = static_cast<int16_t>(std::lround(std::sin(angle) * static_cast<double>(kQ15One)));
    *cos_out = static_cast<int16_t>(std::lround(std::cos(angle) * static_cast<double>(kQ15One)));
}

}  // namespace ems::hal

#else

#include "hal/regs.h"

namespace {

constexpr int16_t kQ15One = 32767;

inline void cordic_wait_ready() noexcept {
    while ((CORDIC_CSR & CORDIC_CSR_RRDY) == 0u) {}
}

}  // namespace

namespace ems::hal {

void cordic_init() noexcept {
    RCC_AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    CORDIC_CSR = CORDIC_CSR_FUNC_COSINE |
                 CORDIC_CSR_PRECISION_24CYC |
                 CORDIC_CSR_NARGS_1 |
                 CORDIC_CSR_NRES_2 |
                 CORDIC_CSR_ARGSIZE_32BIT |
                 CORDIC_CSR_RESSIZE_32BIT;
}

void cordic_sincos_q31(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) noexcept {
    if (sin_out == nullptr || cos_out == nullptr) {
        return;
    }

    CORDIC_WDATA = static_cast<uint32_t>(angle_q31);
    cordic_wait_ready();
    *cos_out = static_cast<int32_t>(CORDIC_RDATA);
    *sin_out = static_cast<int32_t>(CORDIC_RDATA);
}

void cordic_sincos_deg(uint16_t angle_deg_x10, int16_t* sin_out, int16_t* cos_out) noexcept {
    if (sin_out == nullptr || cos_out == nullptr) {
        return;
    }

    const uint32_t angle_q31 = (static_cast<uint32_t>(angle_deg_x10 % 3600u) << 31) / 1800u;
    int32_t sin_q31 = 0;
    int32_t cos_q31 = 0;
    cordic_sincos_q31(static_cast<int32_t>(angle_q31), &sin_q31, &cos_q31);
    *sin_out = static_cast<int16_t>(sin_q31 / 65536);
    *cos_out = static_cast<int16_t>(cos_q31 / 65536);
    if (*sin_out == -32768) {
        *sin_out = -kQ15One;
    }
    if (*cos_out == -32768) {
        *cos_out = -kQ15One;
    }
}

extern "C" void CORDIC_IRQHandler() {}

}  // namespace ems::hal

#endif

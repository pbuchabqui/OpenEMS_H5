#pragma once

#include <cstdint>

namespace ems::hal {

void cordic_init() noexcept;
void cordic_sincos_q31(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) noexcept;
void cordic_sincos_deg(uint16_t angle_deg_x10, int16_t* sin_out, int16_t* cos_out) noexcept;

}  // namespace ems::hal

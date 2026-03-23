#pragma once

#include <cstdint>

namespace ems::util {

// 3-param range clamp (wide input for implicit narrowing from uint32_t callers)
inline uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi) noexcept {
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<uint16_t>(v);
}

// 3-param range clamp (wide input for implicit narrowing from int32_t callers)
inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<int16_t>(v);
}

// Saturating cast to uint16_t (0..65535)
inline uint16_t saturate_u16(uint32_t v) noexcept {
    return static_cast<uint16_t>((v > 65535u) ? 65535u : v);
}

// Saturating cast to int16_t (-32768..32767)
inline int16_t saturate_i16(int32_t v) noexcept {
    if (v < -32768) { return -32768; }
    if (v > 32767)  { return 32767; }
    return static_cast<int16_t>(v);
}

inline bool elapsed(uint32_t now, uint32_t last, uint32_t period) noexcept {
    return static_cast<uint32_t>(now - last) >= period;
}

}  // namespace ems::util

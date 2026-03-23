#pragma once

#include <cstdint>

namespace ems::hal {

enum class NvmError : uint8_t {
    OK = 0,
    INVALID_PARAM,
    TIMEOUT,
    ERASE_FAIL,
    PROGRAM_FAIL,
    CRC_MISMATCH,
    NOT_FOUND,
    BUSY,
    BUFFER_OVERFLOW,
};

}  // namespace ems::hal

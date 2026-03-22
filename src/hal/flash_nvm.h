#pragma once

#include <cstdint>

#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::hal {

void flash_nvm_init() noexcept;

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept;
int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept;

bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept;
int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept;
void nvm_reset_knock_map() noexcept;

bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept;
bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept;
bool nvm_flush_adaptive_maps() noexcept;

void bkpsram_write_crash(const ems::drv::SensorData& s,
                         const ems::drv::CkpSnapshot& ckp) noexcept;
bool bkpsram_read_crash(ems::drv::SensorData& s,
                        ems::drv::CkpSnapshot& ckp) noexcept;
uint32_t bkpsram_boot_count() noexcept;

#if defined(EMS_HOST_TEST)
void nvm_test_reset() noexcept;
void nvm_test_set_ccif_busy_polls(uint32_t polls) noexcept;
uint32_t nvm_test_erase_count() noexcept;
uint32_t nvm_test_program_count() noexcept;
void nvm_test_corrupt_calibration_crc(uint8_t page) noexcept;
#endif

}  // namespace ems::hal

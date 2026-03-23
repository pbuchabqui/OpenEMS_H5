#include "hal/flash_nvm.h"

#include "hal/runtime_seed.h"

#include <cstring>

namespace {

struct CrashStore {
    ems::drv::SensorData sensors;
    ems::drv::CkpSnapshot ckp;
    bool valid;
};

static CrashStore g_crash_store = {};
static uint32_t g_boot_count = 0u;

}  // namespace

#ifndef EMS_HOST_TEST

#include "hal/regs.h"
#include "hal/system.h"

namespace {

static int8_t g_ltft_ram[16][16] = {};
static int8_t g_knock_ram[8][8] = {};
static bool g_ltft_dirty = false;
static bool g_knock_dirty = false;

static constexpr uint32_t kSectorLtft = 0u;
static constexpr uint32_t kSectorCal0 = 1u;
static constexpr uint32_t kSectorCal1 = 2u;
static constexpr uint32_t kSectorCal2 = 3u;

static constexpr uint32_t kBank2Base = FLASH_BANK2_BASE;
static constexpr uint32_t kSectorSize = FLASH_SECTOR_SIZE;

static constexpr uint32_t kFlashKey1 = 0x45670123u;
static constexpr uint32_t kFlashKey2 = 0xCDEF89ABu;

struct CalHeader {
    uint32_t magic;
    uint32_t crc32;
};

static constexpr uint32_t kCalMagic = 0xCA110E55u;
static constexpr uint32_t kCalHdrSz = static_cast<uint32_t>(sizeof(CalHeader));
static constexpr uint32_t kSeedOffset = 512u;

static uint32_t crc32_buffer(const uint8_t* data, uint32_t len) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; ++b) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static void flash_unlock_bank2() noexcept {
    if (FLASH_CR2 & FLASH_CR_LOCK) {
        FLASH_KEYR2 = kFlashKey1;
        FLASH_KEYR2 = kFlashKey2;
    }
}

static void flash_lock_bank2() noexcept {
    FLASH_CR2 |= FLASH_CR_LOCK;
}

static bool flash_wait_ready() noexcept {
    static constexpr uint32_t kFlashTimeoutUs = 50000u;
    const uint32_t deadline = micros() + kFlashTimeoutUs;
    while (FLASH_SR2 & (FLASH_SR_BSY | FLASH_SR_WBNE | FLASH_SR_DBNE)) {
        if (static_cast<int32_t>(micros() - deadline) > 0) {
            return false;
        }
    }
    return true;
}

static bool flash_erase_sector(uint32_t sector_num) noexcept {
    if (!flash_wait_ready()) {
        return false;
    }
    FLASH_CCR2 = 0xFFFFFFFFu;

    FLASH_CR2 = FLASH_CR_SER | ((sector_num & 0xFu) << FLASH_CR_SNB_SHIFT) | FLASH_CR_STRT;

    if (!flash_wait_ready()) {
        return false;
    }
    FLASH_CR2 &= ~(FLASH_CR_SER | (0xFu << FLASH_CR_SNB_SHIFT));

    return (FLASH_SR2 & (FLASH_SR_PGSERR | FLASH_SR_WRPERR)) == 0u;
}

static bool flash_write_words(uint32_t dest_addr,
                              const uint8_t* src,
                              uint32_t len_bytes) noexcept {
    if (!flash_wait_ready()) {
        return false;
    }
    FLASH_CR2 |= FLASH_CR_PG;

    const uint32_t* src32 = reinterpret_cast<const uint32_t*>(src);
    volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(dest_addr);
    const uint32_t nwords = len_bytes / 4u;

    for (uint32_t i = 0u; i < nwords; ++i) {
        dst32[i] = src32[i];
        if (!flash_wait_ready()) {
            FLASH_CR2 &= ~FLASH_CR_PG;
            return false;
        }
        if (FLASH_SR2 & (FLASH_SR_PGSERR | FLASH_SR_WRPERR)) {
            FLASH_CR2 &= ~FLASH_CR_PG;
            return false;
        }
    }

    FLASH_CR2 &= ~FLASH_CR_PG;
    return true;
}

}  // namespace

namespace ems::hal {

void flash_nvm_init() noexcept {
    ++g_boot_count;
}

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) {
        return false;
    }
    if (g_ltft_ram[rpm_i][load_i] != val) {
        g_ltft_ram[rpm_i][load_i] = val;
        g_ltft_dirty = true;
    }
    return true;
}

int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) {
        return 0;
    }
    return g_ltft_ram[rpm_i][load_i];
}

bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) {
        return false;
    }
    if (g_knock_ram[rpm_i][load_i] != retard_deci_deg) {
        g_knock_ram[rpm_i][load_i] = retard_deci_deg;
        g_knock_dirty = true;
    }
    return true;
}

int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) {
        return 0;
    }
    return g_knock_ram[rpm_i][load_i];
}

void nvm_reset_knock_map() noexcept {
    std::memset(g_knock_ram, 0, sizeof(g_knock_ram));
    g_knock_dirty = true;
}

NvmError nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept {
    if (page > 2u || data == nullptr || len == 0u) {
        return NvmError::INVALID_PARAM;
    }
    if (static_cast<uint32_t>(len) + kCalHdrSz > FLASH_SECTOR_SIZE) {
        return NvmError::BUFFER_OVERFLOW;
    }

    const uint32_t sector = kSectorCal0 + page;
    const uint32_t dest = kBank2Base + sector * kSectorSize;

    static uint8_t cal_write_buf[FLASH_SECTOR_SIZE] = {};
    std::memset(cal_write_buf, 0xFFu, sizeof(cal_write_buf));

    CalHeader hdr = {};
    hdr.magic = kCalMagic;
    hdr.crc32 = crc32_buffer(data, static_cast<uint32_t>(len));
    std::memcpy(cal_write_buf, &hdr, kCalHdrSz);
    std::memcpy(cal_write_buf + kCalHdrSz, data, len);

    const uint32_t total = kCalHdrSz + static_cast<uint32_t>(len);
    const uint32_t total32 = (total + 3u) & ~3u;

    flash_unlock_bank2();
    if (!flash_erase_sector(sector)) {
        flash_lock_bank2();
        return NvmError::ERASE_FAIL;
    }
    if (!flash_write_words(dest, cal_write_buf, total32)) {
        flash_lock_bank2();
        return NvmError::PROGRAM_FAIL;
    }
    flash_lock_bank2();
    return NvmError::OK;
}

NvmError nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept {
    if (page > 2u || data == nullptr || len == 0u) {
        return NvmError::INVALID_PARAM;
    }

    const uint32_t sector = kSectorCal0 + page;
    const uint32_t src = kBank2Base + sector * kSectorSize;

    CalHeader hdr = {};
    std::memcpy(&hdr, reinterpret_cast<const void*>(src), kCalHdrSz);
    if (hdr.magic != kCalMagic) {
        return NvmError::NOT_FOUND;
    }

    const uint8_t* flash_data = reinterpret_cast<const uint8_t*>(src + kCalHdrSz);
    const uint32_t computed_crc = crc32_buffer(flash_data, static_cast<uint32_t>(len));
    if (computed_crc != hdr.crc32) {
        return NvmError::CRC_MISMATCH;
    }

    std::memcpy(data, flash_data, len);
    return NvmError::OK;
}

NvmError nvm_flush_adaptive_maps() noexcept {
    if (!g_ltft_dirty && !g_knock_dirty) {
        return NvmError::OK;
    }

    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};
    std::memcpy(sector_buf, reinterpret_cast<const void*>(kBank2Base), sizeof(sector_buf));
    std::memcpy(sector_buf + 0u, g_ltft_ram, 256u);
    std::memcpy(sector_buf + 256u, g_knock_ram, 64u);

    flash_unlock_bank2();
    if (!flash_erase_sector(kSectorLtft)) {
        flash_lock_bank2();
        return NvmError::ERASE_FAIL;
    }
    if (!flash_write_words(kBank2Base, sector_buf, sizeof(sector_buf))) {
        flash_lock_bank2();
        return NvmError::PROGRAM_FAIL;
    }
    flash_lock_bank2();

    g_ltft_dirty = false;
    g_knock_dirty = false;
    return NvmError::OK;
}

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept {
    if (seed == nullptr) {
        return false;
    }

    RuntimeSyncSeed write_seed = *seed;
    write_seed.magic = RUNTIME_SYNC_SEED_MAGIC;
    write_seed.version = RUNTIME_SYNC_SEED_VERSION;
    write_seed.crc32 = runtime_seed_crc32(write_seed);

    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};
    std::memcpy(sector_buf, reinterpret_cast<const void*>(kBank2Base), sizeof(sector_buf));
    std::memcpy(sector_buf + kSeedOffset, &write_seed, sizeof(RuntimeSyncSeed));

    flash_unlock_bank2();
    const bool ok = flash_erase_sector(kSectorLtft) &&
                    flash_write_words(kBank2Base, sector_buf, sizeof(sector_buf));
    flash_lock_bank2();
    return ok;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept {
    if (seed_out == nullptr) {
        return false;
    }
    const uint32_t addr = kBank2Base + kSeedOffset;
    std::memcpy(seed_out, reinterpret_cast<const void*>(addr), sizeof(RuntimeSyncSeed));
    return runtime_seed_boot_compatible_60_2(*seed_out);
}

bool nvm_clear_runtime_seed() noexcept {
    RuntimeSyncSeed blank{};
    return nvm_save_runtime_seed(&blank);
}

void bkpsram_write_crash(const ems::drv::SensorData& s,
                         const ems::drv::CkpSnapshot& ckp) noexcept {
    g_crash_store.sensors = s;
    g_crash_store.ckp = ckp;
    g_crash_store.valid = true;
}

bool bkpsram_read_crash(ems::drv::SensorData& s,
                        ems::drv::CkpSnapshot& ckp) noexcept {
    if (!g_crash_store.valid) {
        return false;
    }
    s = g_crash_store.sensors;
    ckp = g_crash_store.ckp;
    return true;
}

uint32_t bkpsram_boot_count() noexcept {
    return g_boot_count;
}

}  // namespace ems::hal

#else

namespace ems::hal {

static constexpr uint8_t kTestSeedSlots = 8u;

struct CalHeader {
    uint32_t magic;
    uint32_t crc32;
};

static constexpr uint32_t kCalMagic = 0xCA110E55u;
static constexpr uint32_t kCalHdrSz = static_cast<uint32_t>(sizeof(CalHeader));

static int8_t g_ltft[16][16] = {};
static int8_t g_knock[8][8] = {};
static uint8_t g_cal[3][520] = {};
static uint32_t g_erase_cnt = 0u;
static uint32_t g_prog_cnt = 0u;
static bool g_flash_busy = false;
static uint32_t g_ccif_busy_polls = 0u;
static RuntimeSyncSeed g_seed_slots[kTestSeedSlots] = {};
static bool g_seed_slot_valid[kTestSeedSlots] = {};

static uint32_t crc32_buffer(const uint8_t* data, uint32_t len) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; ++b) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void flash_nvm_init() noexcept {
    ++g_boot_count;
}

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) {
        return false;
    }
    if (g_flash_busy) {
        return false;
    }
    g_ltft[rpm_i][load_i] = val;
    return true;
}

int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) {
        return 0;
    }
    return g_ltft[rpm_i][load_i];
}

bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) {
        return false;
    }
    if (g_flash_busy) {
        return false;
    }
    g_knock[rpm_i][load_i] = retard_deci_deg;
    return true;
}

int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) {
        return 0;
    }
    return g_knock[rpm_i][load_i];
}

void nvm_reset_knock_map() noexcept {
    std::memset(g_knock, 0, sizeof(g_knock));
}

NvmError nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept {
    if (page > 2u || data == nullptr || len == 0u) {
        return NvmError::INVALID_PARAM;
    }
    if (static_cast<uint32_t>(len) + kCalHdrSz > sizeof(g_cal[0])) {
        return NvmError::BUFFER_OVERFLOW;
    }
    if (g_flash_busy) {
        return NvmError::BUSY;
    }

    ++g_erase_cnt;
    ++g_prog_cnt;

    CalHeader hdr = {};
    hdr.magic = kCalMagic;
    hdr.crc32 = crc32_buffer(data, static_cast<uint32_t>(len));
    std::memset(g_cal[page], 0xFFu, sizeof(g_cal[page]));
    std::memcpy(g_cal[page], &hdr, kCalHdrSz);
    std::memcpy(g_cal[page] + kCalHdrSz, data, len);
    return NvmError::OK;
}

NvmError nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept {
    if (page > 2u || data == nullptr || len == 0u) {
        return NvmError::INVALID_PARAM;
    }

    CalHeader hdr = {};
    std::memcpy(&hdr, g_cal[page], kCalHdrSz);
    if (hdr.magic != kCalMagic) {
        return NvmError::NOT_FOUND;
    }
    const uint32_t computed_crc = crc32_buffer(g_cal[page] + kCalHdrSz, static_cast<uint32_t>(len));
    if (computed_crc != hdr.crc32) {
        return NvmError::CRC_MISMATCH;
    }

    std::memcpy(data, g_cal[page] + kCalHdrSz, len);
    return NvmError::OK;
}

NvmError nvm_flush_adaptive_maps() noexcept {
    return NvmError::OK;
}

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept {
    if (seed == nullptr) {
        return false;
    }

    uint32_t max_seq = 0u;
    uint8_t write_slot = 0u;
    bool found_any = false;
    for (uint8_t i = 0u; i < kTestSeedSlots; ++i) {
        if (!g_seed_slot_valid[i]) {
            if (!found_any) {
                write_slot = i;
            }
            break;
        }
        if (g_seed_slots[i].sequence >= max_seq) {
            max_seq = g_seed_slots[i].sequence;
            write_slot = static_cast<uint8_t>((i + 1u) % kTestSeedSlots);
            found_any = true;
        }
    }

    RuntimeSyncSeed write_seed = *seed;
    write_seed.magic = RUNTIME_SYNC_SEED_MAGIC;
    write_seed.version = RUNTIME_SYNC_SEED_VERSION;
    write_seed.sequence = found_any ? max_seq + 1u : 0u;
    write_seed.crc32 = runtime_seed_crc32(write_seed);
    g_seed_slots[write_slot] = write_seed;
    g_seed_slot_valid[write_slot] = true;
    return true;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept {
    if (seed_out == nullptr) {
        return false;
    }

    bool found = false;
    uint32_t best_seq = 0u;
    const RuntimeSyncSeed* best = nullptr;
    for (uint8_t i = 0u; i < kTestSeedSlots; ++i) {
        if (!g_seed_slot_valid[i]) {
            continue;
        }
        const RuntimeSyncSeed& slot = g_seed_slots[i];
        if (slot.crc32 != runtime_seed_crc32(slot)) {
            continue;
        }
        if (!runtime_seed_boot_compatible_60_2(slot)) {
            continue;
        }
        if (!found || static_cast<int32_t>(slot.sequence - best_seq) > 0) {
            best_seq = slot.sequence;
            best = &slot;
            found = true;
        }
    }

    if (!found || best == nullptr) {
        return false;
    }
    *seed_out = *best;
    return true;
}

bool nvm_clear_runtime_seed() noexcept {
    std::memset(g_seed_slots, 0, sizeof(g_seed_slots));
    std::memset(g_seed_slot_valid, 0, sizeof(g_seed_slot_valid));
    return true;
}

void bkpsram_write_crash(const ems::drv::SensorData& s,
                         const ems::drv::CkpSnapshot& ckp) noexcept {
    g_crash_store.sensors = s;
    g_crash_store.ckp = ckp;
    g_crash_store.valid = true;
}

bool bkpsram_read_crash(ems::drv::SensorData& s,
                        ems::drv::CkpSnapshot& ckp) noexcept {
    if (!g_crash_store.valid) {
        return false;
    }
    s = g_crash_store.sensors;
    ckp = g_crash_store.ckp;
    return true;
}

uint32_t bkpsram_boot_count() noexcept {
    return g_boot_count;
}

void nvm_test_reset() noexcept {
    std::memset(g_ltft, 0, sizeof(g_ltft));
    std::memset(g_knock, 0, sizeof(g_knock));
    std::memset(g_cal, 0, sizeof(g_cal));
    g_erase_cnt = 0u;
    g_prog_cnt = 0u;
    g_flash_busy = false;
    g_ccif_busy_polls = 0u;
    std::memset(g_seed_slots, 0, sizeof(g_seed_slots));
    std::memset(g_seed_slot_valid, 0, sizeof(g_seed_slot_valid));
    g_crash_store = {};
    g_boot_count = 0u;
}

void nvm_test_set_ccif_busy_polls(uint32_t polls) noexcept {
    g_ccif_busy_polls = polls;
    g_flash_busy = (polls > 0u);
}

uint32_t nvm_test_erase_count() noexcept {
    return g_erase_cnt;
}

uint32_t nvm_test_program_count() noexcept {
    return g_prog_cnt;
}

bool nvm_test_runtime_seed_inject_slot(uint8_t slot,
                                       const RuntimeSyncSeed* seed,
                                       bool recompute_crc) noexcept {
    if (seed == nullptr || slot >= kTestSeedSlots) {
        return false;
    }
    RuntimeSyncSeed write_seed = *seed;
    if (recompute_crc) {
        write_seed.crc32 = runtime_seed_crc32(write_seed);
    }
    g_seed_slots[slot] = write_seed;
    g_seed_slot_valid[slot] = true;
    return true;
}

uint8_t nvm_test_runtime_seed_slot_count() noexcept {
    return kTestSeedSlots;
}

void nvm_test_corrupt_calibration_crc(uint8_t page) noexcept {
    if (page > 2u) {
        return;
    }
    g_cal[page][4] ^= 0x01u;
}

}  // namespace ems::hal

#endif

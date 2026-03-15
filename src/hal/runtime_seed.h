#pragma once

#include <cstdint>

namespace ems::hal {

struct RuntimeSyncSeed {
    uint16_t magic;
    uint8_t version;
    uint8_t flags;
    uint16_t tooth_index;
    uint16_t decoder_tag;
    uint32_t sequence;
    uint32_t crc32;  // CRC-32 (ISO 3309) de todos os campos exceto crc32 si mesmo
};

static constexpr uint16_t RUNTIME_SYNC_SEED_MAGIC = 0x5343u; /* "SC" */
static constexpr uint8_t RUNTIME_SYNC_SEED_VERSION = 1u;
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_VALID = (1u << 0u);
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_FULL_SYNC = (1u << 1u);
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_PHASE_A = (1u << 2u);
static constexpr uint16_t RUNTIME_SYNC_SEED_DECODER_TAG_60_2 = 0x3C02u;  // 60 teeth, 2 missing
static constexpr uint16_t RUNTIME_SYNC_SEED_MAX_TOOTH_INDEX_60_2 = 57u;

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept;
bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept;
bool nvm_clear_runtime_seed() noexcept;

// ── CRC-32 compartilhado (ISO 3309 / Ethernet, polinômio 0xEDB88320) ─────────
// Disponível em ambos os contextos (produção e host test) para garantir
// que a validação em runtime_seed_boot_compatible_60_2() use o mesmo
// algoritmo que nvm_save_runtime_seed() ao gravar na Flash.
//
// Cobre todos os bytes do struct exceto o campo crc32 no final:
//   sizeof(RuntimeSyncSeed) - sizeof(uint32_t) bytes
//
// FIX: a função era definida apenas em #ifdef EMS_HOST_TEST (flexnvm.cpp),
// causando divergência: produção não validava CRC, mock validava.
inline uint32_t runtime_seed_crc32(const RuntimeSyncSeed& seed) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&seed);
    const uint16_t sz = static_cast<uint16_t>(sizeof(seed) - sizeof(seed.crc32));
    for (uint16_t i = 0u; i < sz; ++i) {
        crc ^= p[i];
        for (uint8_t b = 0u; b < 8u; ++b) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// ── Validação completa de compatibilidade (inclui CRC) ───────────────────────
// FIX: versão anterior verificava apenas flags/tooth_index/decoder_tag,
// ignorando magic, version e o campo crc32 — Flash corrompida era aceita.
inline bool runtime_seed_boot_compatible_60_2(const RuntimeSyncSeed& seed) noexcept {
    if (seed.magic != RUNTIME_SYNC_SEED_MAGIC) {
        return false;
    }
    if (seed.version != RUNTIME_SYNC_SEED_VERSION) {
        return false;
    }
    if ((seed.flags & RUNTIME_SYNC_SEED_FLAG_FULL_SYNC) == 0u) {
        return false;
    }
    if (seed.tooth_index > RUNTIME_SYNC_SEED_MAX_TOOTH_INDEX_60_2) {
        return false;
    }
    if (seed.decoder_tag != RUNTIME_SYNC_SEED_DECODER_TAG_60_2) {
        return false;
    }
    // Validar integridade dos dados — detecta Flash corrompida por power-loss
    if (seed.crc32 != runtime_seed_crc32(seed)) {
        return false;
    }
    return true;
}

inline bool runtime_seed_fast_reacquire_compatible_60_2(const RuntimeSyncSeed& seed) noexcept {
    if (!runtime_seed_boot_compatible_60_2(seed)) {
        return false;
    }
    // Fast-gap promotion path anchors at the next detected gap; require a gap-aligned seed.
    return (seed.tooth_index == 0u);
}

#if defined(EMS_HOST_TEST)
bool nvm_test_runtime_seed_inject_slot(uint8_t slot,
                                       const RuntimeSyncSeed* seed,
                                       bool recompute_crc) noexcept;
uint8_t nvm_test_runtime_seed_slot_count() noexcept;
#endif

}  // namespace ems::hal

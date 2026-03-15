/**
 * @file hal/stm32h562/flash.cpp
 * @brief Emulação de EEPROM via Flash Bank2 para STM32H562RGT6
 *        Substitui hal/flexnvm.cpp da versão Kinetis.
 *
 * Layout da Flash Bank2 (base 0x08100000):
 *   Setor 0 (0x08100000, 8 KB): LTFT map + Knock map (página quente)
 *   Setor 1 (0x08102000, 8 KB): Calibração página 0 (512 bytes)
 *   Setor 2 (0x08104000, 8 KB): Calibração página 1 (256 bytes)
 *   Setor 3 (0x08106000, 8 KB): Calibração página 2 (256 bytes)
 *   Setores 4-7: reservados para wear leveling de calibração
 *
 * Emulação de FlexRAM:
 *   No Kinetis, LTFT e knock maps vivem em FlexRAM (SRAM dedicada 0x14000000).
 *   No STM32, usamos um buffer em SRAM normal (g_ltft_ram / g_knock_ram)
 *   e escrevemos na Flash periodicamente (quando dirty bit ativo).
 *
 * Procedimento de escrita:
 *   1. Aguardar BSY
 *   2. Desbloquear Flash Bank2 (KEYR2 com sequência)
 *   3. Apagar setor (SER + SNB + STRT)
 *   4. Programar em palavras de 32 bits (PG mode)
 *   5. Re-travar
 */

#ifndef EMS_HOST_TEST

#include "hal/flexnvm.h"
#include "hal/regs.h"
#include "hal/system.h"  // micros() para timeout em flash_wait_ready()
#include <cstring>

// ── Buffers SRAM para LTFT e Knock maps ─────────────────────────────────────
// Espelham os dados da Flash; modificados em RAM e flushed periodicamente.
static int8_t  g_ltft_ram[16][16] = {};     // 256 bytes (equivale FlexRAM 0x14000000)
static int8_t  g_knock_ram[8][8]  = {};     // 64 bytes  (equivale FlexRAM 0x14000100)
static bool    g_ltft_dirty  = false;
static bool    g_knock_dirty = false;

// ── Endereços dos setores Bank2 ───────────────────────────────────────────────
static constexpr uint32_t kSectorLtft  = 0u;   // Setor 0: LTFT + knock
static constexpr uint32_t kSectorCal0  = 1u;   // Setor 1: Cal page 0
static constexpr uint32_t kSectorCal1  = 2u;   // Setor 2: Cal page 1
static constexpr uint32_t kSectorCal2  = 3u;   // Setor 3: Cal page 2

static constexpr uint32_t kBank2Base   = FLASH_BANK2_BASE;
static constexpr uint32_t kSectorSize  = FLASH_SECTOR_SIZE;

// Chaves de desbloqueio da Flash (RM0481 §7.4)
static constexpr uint32_t kFlashKey1 = 0x45670123u;
static constexpr uint32_t kFlashKey2 = 0xCDEF89ABu;

// ── Cabeçalho de integridade para dados de calibração ────────────────────────
// Prefixo de 8 bytes gravado antes de cada página de calibração na Flash.
// Permite detectar dados corrompidos por power-loss durante escrita.
// O setor tem 8 KB — o header de 8 bytes é negligível.
struct CalHeader {
    uint32_t magic;   // identificador fixo kCalMagic
    uint32_t crc32;   // CRC-32 (ISO 3309) dos bytes de dados (sem o header)
};
static constexpr uint32_t kCalMagic  = 0xCA110E55u;  // "CALL EMS"
static constexpr uint32_t kCalHdrSz  = static_cast<uint32_t>(sizeof(CalHeader));  // 8 bytes

// CRC-32 sobre buffer arbitrário (mesmo polinômio que runtime_seed_crc32).
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

// ── Funções auxiliares ───────────────────────────────────────────────────────

static void flash_unlock_bank2() noexcept {
    if (FLASH_CR2 & FLASH_CR_LOCK) {
        FLASH_KEYR2 = kFlashKey1;
        FLASH_KEYR2 = kFlashKey2;
    }
}

static void flash_lock_bank2() noexcept {
    FLASH_CR2 |= FLASH_CR_LOCK;
}

// FIX: versão anterior era void e loop infinito — se o Flash controller travar,
// o IWDG não consegue resetar o sistema pois o main loop está bloqueado aqui.
// Agora usa micros() com timeout de 50 ms e retorna false em caso de travamento.
static bool flash_wait_ready() noexcept {
    static constexpr uint32_t kFlashTimeoutUs = 50000u;  // 50 ms
    const uint32_t deadline = micros() + kFlashTimeoutUs;
    while (FLASH_SR2 & (FLASH_SR_BSY | FLASH_SR_WBNE | FLASH_SR_DBNE)) {
        if (static_cast<int32_t>(micros() - deadline) > 0) {
            return false;  // timeout — Flash controller travado
        }
    }
    return true;
}

static bool flash_erase_sector(uint32_t sector_num) noexcept {
    if (!flash_wait_ready()) { return false; }  // timeout antes de começar
    FLASH_CCR2 = 0xFFFFFFFFu;  // limpa todos os flags de erro

    FLASH_CR2 = FLASH_CR_SER
              | ((sector_num & 0xFu) << FLASH_CR_SNB_SHIFT)
              | FLASH_CR_STRT;

    if (!flash_wait_ready()) { return false; }  // timeout durante apagamento
    FLASH_CR2 &= ~(FLASH_CR_SER | (0xFu << FLASH_CR_SNB_SHIFT));

    return (FLASH_SR2 & (FLASH_SR_PGSERR | FLASH_SR_WRPERR)) == 0u;
}

static bool flash_write_words(uint32_t dest_addr,
                              const uint8_t* src,
                              uint32_t len_bytes) noexcept {
    // len_bytes deve ser múltiplo de 4
    if (!flash_wait_ready()) { return false; }  // timeout antes de começar
    FLASH_CR2 |= FLASH_CR_PG;

    const uint32_t* src32 = reinterpret_cast<const uint32_t*>(src);
    volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(dest_addr);
    const uint32_t nwords = len_bytes / 4u;

    for (uint32_t i = 0u; i < nwords; ++i) {
        dst32[i] = src32[i];
        if (!flash_wait_ready()) {        // timeout durante escrita de palavra
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

namespace ems::hal {

// ── LTFT map ─────────────────────────────────────────────────────────────────

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) { return false; }
    if (g_ltft_ram[rpm_i][load_i] != val) {
        g_ltft_ram[rpm_i][load_i] = val;
        g_ltft_dirty = true;
    }
    return true;
}

int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) { return 0; }
    return g_ltft_ram[rpm_i][load_i];
}

// ── Knock map ─────────────────────────────────────────────────────────────────

bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) { return false; }
    if (g_knock_ram[rpm_i][load_i] != retard_deci_deg) {
        g_knock_ram[rpm_i][load_i] = retard_deci_deg;
        g_knock_dirty = true;
    }
    return true;
}

int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) { return 0; }
    return g_knock_ram[rpm_i][load_i];
}

void nvm_reset_knock_map() noexcept {
    std::memset(g_knock_ram, 0, sizeof(g_knock_ram));
    g_knock_dirty = true;
}

// ── Calibração (páginas) ──────────────────────────────────────────────────────

bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept {
    if (page > 2u || data == nullptr || len == 0u) { return false; }

    const uint32_t sector = kSectorCal0 + page;
    const uint32_t dest   = kBank2Base + sector * kSectorSize;

    // Montar buffer: [CalHeader (8 bytes) | dados | padding para múltiplo de 4]
    // Usar sector_buf local estático (8 KB, em BSS — não na stack).
    // FIX: versão anterior gravava dados sem magic/CRC — power-loss durante escrita
    // retornava lixo silenciosamente. Agora detectável na leitura via CalHeader.
    static uint8_t cal_write_buf[FLASH_SECTOR_SIZE] = {};
    std::memset(cal_write_buf, 0xFFu, sizeof(cal_write_buf));

    CalHeader hdr = {};
    hdr.magic = kCalMagic;
    hdr.crc32 = crc32_buffer(data, static_cast<uint32_t>(len));
    std::memcpy(cal_write_buf, &hdr, kCalHdrSz);
    std::memcpy(cal_write_buf + kCalHdrSz, data, len);

    // Total a gravar: header + dados, arredondado para múltiplo de 4
    const uint32_t total = kCalHdrSz + static_cast<uint32_t>(len);
    const uint32_t total32 = (total + 3u) & ~3u;

    flash_unlock_bank2();
    const bool ok = flash_erase_sector(sector) &&
                    flash_write_words(dest, cal_write_buf, total32);
    flash_lock_bank2();
    return ok;
}

bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept {
    if (page > 2u || data == nullptr || len == 0u) { return false; }

    const uint32_t sector = kSectorCal0 + page;
    const uint32_t src    = kBank2Base + sector * kSectorSize;

    // Ler e validar o CalHeader antes de copiar dados para o buffer de destino.
    // FIX: versão anterior copiava dados sem verificação — Flash corrompida era aceita.
    CalHeader hdr = {};
    std::memcpy(&hdr, reinterpret_cast<const void*>(src), kCalHdrSz);

    if (hdr.magic != kCalMagic) {
        return false;  // setor nunca foi gravado ou foi corrompido
    }

    const uint8_t* flash_data = reinterpret_cast<const uint8_t*>(src + kCalHdrSz);
    const uint32_t computed_crc = crc32_buffer(flash_data, static_cast<uint32_t>(len));
    if (computed_crc != hdr.crc32) {
        return false;  // dados corrompidos (power-loss durante escrita)
    }

    std::memcpy(data, flash_data, len);
    return true;
}

// ── Flush LTFT + Knock para Flash ─────────────────────────────────────────────
// Chamado a cada 500 ms do loop principal (equivalente ao flexnvm flush do Kinetis)

bool nvm_flush_adaptive_maps() noexcept {
    if (!g_ltft_dirty && !g_knock_dirty) { return true; }

    // Setor 0 contém LTFT (256 bytes) + Knock (64 bytes)
    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};

    // Carregar conteúdo atual do setor (para preservar outros dados)
    std::memcpy(sector_buf,
                reinterpret_cast<const void*>(kBank2Base),
                sizeof(sector_buf));

    // Atualizar LTFT e knock no buffer
    std::memcpy(sector_buf + 0,   g_ltft_ram,  256u);
    std::memcpy(sector_buf + 256, g_knock_ram, 64u);

    flash_unlock_bank2();
    const bool ok = flash_erase_sector(kSectorLtft) &&
                    flash_write_words(kBank2Base,
                                      sector_buf,
                                      sizeof(sector_buf));
    flash_lock_bank2();

    if (ok) {
        g_ltft_dirty  = false;
        g_knock_dirty = false;
    }
    return ok;
}

// ── RuntimeSyncSeed (boot rápido) ────────────────────────────────────────────
// Armazena seed na região final do Setor 0 (bytes 512-543 = 32 bytes)
#include "hal/runtime_seed.h"

static constexpr uint32_t kSeedOffset = 512u;

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept {
    if (seed == nullptr) { return false; }

    // FIX: versão anterior gravava o seed como recebido, sem preencher magic,
    // version nem crc32 — campos ficavam com lixo se o chamador não os definisse.
    // Agora sempre forçamos magic e version corretos e computamos o CRC antes de gravar.
    RuntimeSyncSeed w = *seed;
    w.magic   = RUNTIME_SYNC_SEED_MAGIC;
    w.version = RUNTIME_SYNC_SEED_VERSION;
    w.crc32   = runtime_seed_crc32(w);  // CRC de todos os campos exceto crc32

    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};
    std::memcpy(sector_buf,
                reinterpret_cast<const void*>(kBank2Base),
                sizeof(sector_buf));
    std::memcpy(sector_buf + kSeedOffset, &w, sizeof(RuntimeSyncSeed));

    flash_unlock_bank2();
    const bool ok = flash_erase_sector(kSectorLtft) &&
                    flash_write_words(kBank2Base, sector_buf, sizeof(sector_buf));
    flash_lock_bank2();
    return ok;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept {
    if (seed_out == nullptr) { return false; }
    const uint32_t addr = kBank2Base + kSeedOffset;
    std::memcpy(seed_out, reinterpret_cast<const void*>(addr),
                sizeof(RuntimeSyncSeed));
    return runtime_seed_boot_compatible_60_2(*seed_out);
}

bool nvm_clear_runtime_seed() noexcept {
    RuntimeSyncSeed blank{};
    return nvm_save_runtime_seed(&blank);
}

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/flexnvm.h"
#include "hal/runtime_seed.h"
#include <cstring>

// runtime_seed_crc32() agora definida em hal/runtime_seed.h (inline, compartilhada).
// Removida duplicata local que existia apenas no bloco EMS_HOST_TEST.

namespace ems::hal {

static constexpr uint8_t kTestSeedSlots = 8u;
// CalHeader para mock: mesma struct que produção (replica comportamento real)
struct CalHeader {
    uint32_t magic;
    uint32_t crc32;
};
static constexpr uint32_t kCalMagic = 0xCA110E55u;
static constexpr uint32_t kCalHdrSz = static_cast<uint32_t>(sizeof(CalHeader));

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

static int8_t g_ltft[16][16] = {};
static int8_t g_knock[8][8]  = {};
// g_cal: layout [CalHeader (8 bytes) | dados (512 bytes)] por página
static uint8_t g_cal[3][520]  = {};
static uint32_t g_erase_cnt   = 0u, g_prog_cnt = 0u;
static bool     g_flash_busy      = false;  // simulates flash BSY timeout when set
static uint32_t g_ccif_busy_polls = 0u;     // non-zero → simulate timeout on next op

// Runtime seed: slot array (mirrors Kinetis FlexRAM layout for test compatibility)
static RuntimeSyncSeed g_seed_slots[kTestSeedSlots] = {};
static bool g_seed_slot_valid[kTestSeedSlots] = {};

bool nvm_write_ltft(uint8_t r, uint8_t l, int8_t v) noexcept {
    if (r >= 16u || l >= 16u) { return false; }
    if (g_flash_busy) { return false; }
    g_ltft[r][l] = v; return true;
}
int8_t nvm_read_ltft(uint8_t r, uint8_t l) noexcept {
    if (r >= 16u || l >= 16u) { return 0; }
    return g_ltft[r][l];
}
bool nvm_write_knock(uint8_t r, uint8_t l, int8_t v) noexcept {
    if (r >= 8u || l >= 8u) { return false; }
    if (g_flash_busy) { return false; }
    g_knock[r][l] = v; return true;
}
int8_t nvm_read_knock(uint8_t r, uint8_t l) noexcept {
    if (r >= 8u || l >= 8u) { return 0; }
    return g_knock[r][l];
}
void nvm_reset_knock_map() noexcept { std::memset(g_knock, 0, sizeof(g_knock)); }

bool nvm_save_calibration(uint8_t pg, const uint8_t* d, uint16_t l) noexcept {
    if (pg > 2u || d == nullptr || l == 0u) return false;
    if (g_flash_busy) { return false; }
    ++g_erase_cnt; ++g_prog_cnt;
    // Mock replica o comportamento de produção: [CalHeader | dados]
    CalHeader hdr = {};
    hdr.magic = kCalMagic;
    hdr.crc32 = crc32_buffer(d, static_cast<uint32_t>(l));
    std::memset(g_cal[pg], 0xFFu, sizeof(g_cal[pg]));
    std::memcpy(g_cal[pg], &hdr, kCalHdrSz);
    std::memcpy(g_cal[pg] + kCalHdrSz, d, l);
    return true;
}
bool nvm_load_calibration(uint8_t pg, uint8_t* d, uint16_t l) noexcept {
    if (pg > 2u || d == nullptr || l == 0u) return false;
    // Verificar CalHeader antes de copiar (mesma lógica que produção)
    CalHeader hdr = {};
    std::memcpy(&hdr, g_cal[pg], kCalHdrSz);
    if (hdr.magic != kCalMagic) { return false; }
    const uint32_t computed = crc32_buffer(g_cal[pg] + kCalHdrSz, static_cast<uint32_t>(l));
    if (computed != hdr.crc32) { return false; }
    std::memcpy(d, g_cal[pg] + kCalHdrSz, l);
    return true;
}
bool nvm_flush_adaptive_maps() noexcept { return true; }

bool nvm_save_runtime_seed(const RuntimeSyncSeed* s) noexcept {
    if (!s) { return false; }
    // Find slot with highest sequence number to determine next write slot
    uint32_t max_seq = 0u;
    uint8_t write_slot = 0u;
    bool found_any = false;
    for (uint8_t i = 0u; i < kTestSeedSlots; ++i) {
        if (!g_seed_slot_valid[i]) {
            if (!found_any) { write_slot = i; }
            break;
        }
        if (g_seed_slots[i].sequence >= max_seq) {
            max_seq = g_seed_slots[i].sequence;
            write_slot = static_cast<uint8_t>((i + 1u) % kTestSeedSlots);
            found_any = true;
        }
    }
    RuntimeSyncSeed w = *s;
    w.magic    = RUNTIME_SYNC_SEED_MAGIC;
    w.version  = RUNTIME_SYNC_SEED_VERSION;
    w.sequence = found_any ? max_seq + 1u : 0u;
    w.crc32    = runtime_seed_crc32(w);  // usa função compartilhada de runtime_seed.h
    g_seed_slots[write_slot] = w;
    g_seed_slot_valid[write_slot] = true;
    return true;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* s) noexcept {
    if (!s) { return false; }
    // Find valid slot with highest sequence (wrap-aware)
    bool found = false;
    uint32_t best_seq = 0u;
    const RuntimeSyncSeed* best = nullptr;
    for (uint8_t i = 0u; i < kTestSeedSlots; ++i) {
        if (!g_seed_slot_valid[i]) { continue; }
        const RuntimeSyncSeed& sl = g_seed_slots[i];
        if (sl.crc32 != runtime_seed_crc32(sl)) { continue; }
        if (!runtime_seed_boot_compatible_60_2(sl)) { continue; }
        if (!found || static_cast<int32_t>(sl.sequence - best_seq) > 0) {
            best_seq = sl.sequence;
            best = &sl;
            found = true;
        }
    }
    if (!found || best == nullptr) { return false; }
    *s = *best;
    return true;
}

bool nvm_clear_runtime_seed() noexcept {
    std::memset(g_seed_slots, 0, sizeof(g_seed_slots));
    std::memset(g_seed_slot_valid, 0, sizeof(g_seed_slot_valid));
    return true;
}

void nvm_test_reset() noexcept {
    std::memset(g_ltft, 0, sizeof(g_ltft));
    std::memset(g_knock, 0, sizeof(g_knock));
    std::memset(g_cal, 0, sizeof(g_cal));
    g_erase_cnt = g_prog_cnt = 0u;
    g_flash_busy = false;
    g_ccif_busy_polls = 0u;
    std::memset(g_seed_slots, 0, sizeof(g_seed_slots));
    std::memset(g_seed_slot_valid, 0, sizeof(g_seed_slot_valid));
}
void nvm_test_set_ccif_busy_polls(uint32_t polls) noexcept {
    g_ccif_busy_polls = polls;
    g_flash_busy = (polls > 0u);
}
uint32_t nvm_test_erase_count() noexcept { return g_erase_cnt; }
uint32_t nvm_test_program_count() noexcept { return g_prog_cnt; }

bool nvm_test_runtime_seed_inject_slot(uint8_t slot,
                                       const RuntimeSyncSeed* seed,
                                       bool recompute_crc) noexcept {
    if (seed == nullptr || slot >= kTestSeedSlots) { return false; }
    RuntimeSyncSeed w = *seed;
    if (recompute_crc) { w.crc32 = runtime_seed_crc32(w); }
    g_seed_slots[slot] = w;
    g_seed_slot_valid[slot] = true;
    return true;
}

uint8_t nvm_test_runtime_seed_slot_count() noexcept {
    return kTestSeedSlots;
}

void nvm_test_corrupt_calibration_crc(uint8_t page) noexcept {
    if (page > 2u) { return; }
    // CalHeader layout: [magic (4B) | crc32 (4B)] — flip bit 0 do campo crc32
    // para simular corrupção por power-loss ou bit-flip em Flash.
    g_cal[page][4] ^= 0x01u;  // byte 4 = LSB do campo crc32 em CalHeader
}

} // namespace ems::hal

#endif  // EMS_HOST_TEST

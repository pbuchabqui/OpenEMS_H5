# OpenEMS v2.2 — Adversarial Peer Review Report

**Date**: 2026-03-22  
**Reviewer**: Cline (Adversarial Agent)  
**Scope**: Full codebase review against `OpenEMS_Engineering_Prompt_STM32_v2.2.md` and `OpenEMS_v2.2.json`  
**Severity Levels**: 🔴 CRITICAL (safety/correctness), 🟡 HIGH (spec violation), 🟠 MEDIUM (robustness), 🔵 LOW (style/best practice)

---

## EXECUTIVE SUMMARY

The codebase has **incomplete STM32H5 porting** — several modules still reference Teensy/MK64F peripherals. Critical bugs include an RPM formula that produces values ~100× too high and a scheduler that doesn't reject past events. The ADC/sensor pipeline is entirely unported (still uses MK64F ADC0/ADC1/PDB).

| Category | 🔴 Critical | 🟡 High | 🟠 Medium | 🔵 Low |
|----------|------------|---------|----------|--------|
| HAL      | 3          | 2       | 1        | 0      |
| Driver   | 2          | 2       | 1        | 0      |
| Engine   | 0          | 1       | 0        | 0      |
| App      | 0          | 1       | 1        | 0      |
| Main     | 2          | 4       | 2        | 0      |
| **Total**| **7**      | **10**  | **5**    | **0**  |

---

## 🔴 CRITICAL FINDINGS

### C1/C7. RPM Formula Produces Values ~100× Too High
**Files**: `src/drv/ckp.cpp` (lines ~196 and ~200)  
**Issue**: The numerator `2500000000ULL` is wrong.  
**Spec says**: `rpm_x10 = 1500000000000ULL / (58 * tooth_period_ticks)`  
**Simplified**: `rpm_x10 ≈ 25862068ULL / period_ticks`  
**Code uses**: `rpm_x10 = 2500000000ULL / period_ticks` ← **~96.7× too high**

**Impact**: At 1000 RPM (period_ticks ≈ 2,586,207), the code reports ~9670 RPM instead of 1000 RPM. This causes:
- Incorrect fuel calculation (VE lookup at wrong RPM)
- Incorrect ignition advance (spark table lookup at wrong RPM)
- Limp mode may engage incorrectly
- CAN telemetry reports wrong RPM

**Fix**: Change `kRpmNumeratorTicks` to `25862068ULL`.

---

### C2/H8. Scheduler Doesn't Reject Past Events
**File**: `src/drv/scheduler.cpp` (`sched_event()`)  
**Issue**: No check for events scheduled in the past.  
**Spec says**: `uint32_t delta = abs_ticks - tim5_count(); if (delta > 0x80000000u) return false;`  
**Code does**: Always programs compare, returns `true`.

**Impact**: Late events fire after 16-bit wrap (~65ms) → injection/ignition timing corruption, potential engine damage.

**Fix**: Add past-event rejection before `program_compare()`.

---

### C3. TIM1 BKIN Pin Not Configured
**File**: `src/hal/tim.cpp` (`init_tim1_compare()`)  
**Issue**: PB12 (TIM1_BKIN, pin 33) not configured as AF1 input.  
**Impact**: Emergency shutdown for INJ1-4 does NOT work → risk of fire, hydrolock, ECU destruction.

**Fix**: Add `gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 12u, GPIO_AF1);`

---

### C4. TIM1/TIM8 ITR Synchronization Missing
**File**: `src/hal/tim.cpp`  
**Issue**: `TIM1_SMCR = 0u; TIM8_SMCR = 0u;` — no ITR slave mode.  
**Impact**: 32→16 bit conversion `(uint16_t)(abs_ticks)` produces WRONG values → timing errors up to 16ms.

**Fix**: Configure TIM5 as master (MMS=010), TIM1/TIM8 as ITR slaves (SMS=100 reset mode).

---

### C5. ADC/Sensor Pipeline Not Ported to STM32H5
**Files**: `src/drv/sensors.cpp`, `src/hal/adc.h`  
**Issue**: Uses MK64F/Teensy APIs (adc0_read, Adc0Channel::MAP_SE10, PDB).  
**Impact**: **FIRMWARE WILL NOT COMPILE** for STM32H5.

**Fix**: Complete rewrite for STM32H5 ADC1/ADC2 with GPDMA.

---

### C6. TIM15 IRQ Handler Name Wrong
**File**: `src/hal/tim.h`  
**Issue**: Declares `TIM15_IRQHandler()` but STM32H5 uses `TIM1_BRK_TIM15_IRQHandler`.  
**Impact**: INJ4 (PC12) never triggers → Cylinder 4 gets no fuel.

**Fix**: Rename to `TIM1_BRK_TIM15_IRQHandler`.

---

## 🟡 HIGH SEVERITY FINDINGS

| ID | File | Issue |
|----|------|-------|
| H1 | main.cpp | Main loop 2ms vs spec 5ms |
| H2 | main.cpp | millis()/micros() used for timing |
| H3 | main.cpp | dac_init() not called |
| H4 | main.cpp | usart3_init() not called |
| H5 | main.cpp | NVIC priorities incomplete |
| H6 | main.cpp | LPTIM stop mode not implemented |
| H7 | main.cpp | BKIN not tested at init |
| H9 | tim.cpp | TIM2 CC2IE enabled unnecessarily |
| H10 | main.cpp | No ICACHE hit rate monitoring |

---

## 🟠 MEDIUM SEVERITY FINDINGS

| ID | File | Issue |
|----|------|-------|
| M1 | ckp.cpp | MK64F GPIO references in comments |
| M2 | sensors.cpp | VBATT hardcoded to 12000mV |
| M3 | ckp.cpp | No divide-by-zero guard in ckp_angle_to_ticks |
| M4 | scheduler.cpp | Always returns true |
| M5 | main.cpp | Backup SRAM not initialized |

---

## SPECIFICATION COMPLIANCE MATRIX

| Requirement | Status | Notes |
|-------------|--------|-------|
| TIM2 32-bit CKP | ✅ | PSC=0, ARR=0xFFFFFFFF |
| TIM5 32-bit scheduler | ✅ | PSC=0, ARR=0xFFFFFFFF |
| TIM1 INJ1-3 + BKIN | ⚠️ | BKIN pin missing |
| TIM8 IGN1-4 + BKIN | ⚠️ | BKIN pin configured |
| TIM15 INJ4 | ❌ | Wrong IRQ handler |
| ITR sync | ❌ | SMCR=0 |
| ICACHE | ✅ | FLASH_ACR configured |
| CORDIC | ✅ | Both variants implemented |
| ADC+GPDMA | ❌ | Not ported |
| DAC 12-bit | ❌ | Not in main init |
| CAN-FD 48B | ✅ | Implemented |
| USB CDC | ✅ | Implemented |
| Backup SRAM | ⚠️ | Exists but not initialized |
| LPTIM | ❌ | Not implemented |
| RPM formula | ❌ | 100× too high |
| Scheduler check | ❌ | Missing |
| BKIN LOCK=2 | ✅ | BDTR correct |
| 60-2 decoder | ✅ | State machine correct |
| Fuel calc | ✅ | Formula matches spec |
| Ign calc | ✅ | Advance clamping present |
| TunerStudio | ✅ | USB CDC transport |

---

## RECOMMENDATIONS

### Immediate (Before Any Hardware Testing)
1. Fix RPM formula (C1/C7) — `2500000000ULL` → `25862068ULL`
2. Fix TIM15 IRQ handler (C6) — `TIM15_IRQHandler` → `TIM1_BRK_TIM15_IRQHandler`
3. Add TIM1 BKIN pin config (C3) — PB12 AF1
4. Add ITR sync (C4) — TIM5→TIM1/TIM8 slave mode
5. Add past-event rejection (C2/H8)

### Short-Term (Before Dynamic Testing)
6. Port ADC/sensor pipeline (C5)
7. Add DAC/usart3 init (H3/H4)
8. Complete NVIC priorities (H5)
9. Fix main loop to 5ms (H1)

### Medium-Term (Before Production)
10. Implement LPTIM stop mode (H6)
11. Add BKIN test at init (H7)
12. Add backup SRAM init (M5)

---

## CONCLUSION

The firmware is **NOT ready for hardware testing**. Minimum fixes required: C1, C3, C4, C5, C6 before attempting to start an engine.

The **RPM formula bug (C1)** is the most dangerous correctness issue — all engine calculations use wrong RPM values.

The **missing ITR sync (C4)** and **missing BKIN pin (C3)** are safety-critical — emergency shutdown is non-functional.
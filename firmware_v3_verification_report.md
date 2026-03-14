# Firmware v3 — Relatório de Verificação

**Alvo:** STM32H562RGT6 — ECU inline-4, roda 60-2 + sensor de cam
**Arquivos analisados:** 60 arquivos (223 KB)
**Data:** 2026-03-14

---

## Resumo Executivo

| Severidade | Quantidade |
|------------|-----------|
| CRÍTICO    | 3         |
| MAJOR      | 9         |
| MINOR      | 5         |
| **Total**  | **17**    |

---

## BUGS CRÍTICOS

### 1. Truncamento do timer no HAL — `hal/hal.c` (~linha 263)

```c
/* BUG: TIM2 é 32-bit, mas o CCR é truncado para 16 bits */
*ccr = (uint16_t)(timestamp & 0xFFFFUL);
```

**Problema:** Qualquer evento agendado após ~65 ms do início wrapa incorretamente. Todos os timings de ignição e injeção ficam errados.
**Correção:** Usar o registrador de 32 bits diretamente: `*ccr = timestamp;`
**Impacto:** Risco de dano ao motor por timing de faísca/injeção aleatório.

---

### 2. Race condition em leitura de float — `engine/angle_tracker.c` (~linhas 86, 104-106, 132, 144)

`s_cycle_base_angle` é `volatile float` mas é lido fora de seção crítica em pontos críticos. No Cortex-M33, leitura de float de 32 bits pode não ser atômica dependendo do alinhamento.

**Impacto:** Ângulo corrompido em alta RPM → ignição/injeção fora de sincronismo.
**Correção:** Envolver todas as leituras de `s_cycle_base_angle` e `s_pll_period` em `hal_enter_critical()` / `hal_exit_critical()`.

---

### 3. Offset CAN TX incorreto — `communication/comm.c` (~linhas 126-127)

```c
/* BUG: FDCAN1->TXBC não é um offset direto em bytes */
volatile uint32_t *tb = (volatile uint32_t *)
    (SRAMCAN_BASE + FDCAN1->TXBC * 4U);
```

**Problema:** O registrador `TXBC` codifica o endereço de início do buffer como offset de palavra; a multiplicação por 4 é simplificada e incorreta. O código escreve em posição de memória errada.
**Impacto:** Corrupção de memória ou frames CAN inválidos.
**Correção:** Usar o offset real conforme RM0481 seção FDCAN message RAM, ou inicializar via STM32CubeH5 e usar ponteiro retornado pelo HAL.

---

## BUGS MAJOR

### 4. Constante de fluxo de injetor errada por 1000× — `fuel/fuel_control.c` (~linhas 107, 118)

```c
float injected_mg = (float)base_pw * 0.0055f;  /* deveria ser 5.5f */
float extra_us    = extra_mg / 0.0055f;         /* deveria ser 5.5f */
```

440 cc/min = 5,5 g/s = **5,5 µg/µs**, não 0,0055 µg/µs. O modelo de wall-wetting (Aquino) opera com valores 1000× menores que a realidade.
**Impacto:** Correção de transiente e partida a frio completamente incorretas.

---

### 5. Acesso fora dos limites no ISR do scheduler — `engine/event_scheduler.c` (~linha 147)

```c
int8_t slot = s_channel_to_slot[hw_channel];  /* sem bounds check de hw_channel */
```

`hw_channel` vem direto da ISR sem validação. Se `hw_channel >= MAX_HW_CHANNELS (8)`, acesso fora do array.
**Correção:** Adicionar `if (hw_channel >= MAX_HW_CHANNELS) return;` antes do acesso.

---

### 6. NaN em leitura de termistor — `sensors/sensors.c` (~linhas 68-76)

```c
float r = THERMISTOR_RPULL * v / denom;
/* se v > ADC_VREF por ruído, denom < 0, r < 0 */
float inv_t = (1.0f / T_REF_K) + (1.0f / beta) * logf(r / r25);
/* logf(valor negativo) = NaN */
```

**Correção:** `if (r <= 0.0f) return -40.0f;` antes do `logf`.

---

### 7. Wrap-around de timer no timeout de crank — `diagnostics/safe_state.c` (~linha 88)

```c
uint32_t elapsed_ms = now_ms - s_last_tooth_ts_ms;
```

Subtração unsigned é correta para wrap, **mas** a granularidade é em ms (`hal_timer_get() / HAL_TIMER_TICKS_PER_MS`). Se `HAL_TIMER_TICKS_PER_MS` não está definido corretamente, o timeout pode ser falso.
**Verificação necessária:** Confirmar valor de `HAL_TIMER_TICKS_PER_MS` contra clock real do TIM2.

---

### 8. Frequência do filtro de knock não validada — `knock/knock_dsp.c` (~linhas 100-104)

```c
calibration_get(CAL_ID_KNOCK_FREQ_HZ, &freq_hz);
compute_coefficients(freq_hz);  /* sem validar se 0 < freq < Nyquist */
```

Se `freq_hz = 0` ou `> 25 kHz` (Nyquist para 50 kHz de amostragem), o filtro biquad fica instável.
**Correção:** `if (freq_hz < 100.0f || freq_hz > 24000.0f) freq_hz = 7500.0f;`

---

### 9. Race condition no ring buffer de log — `logging/logging.c` (~linhas 53-67)

```c
void log_write(uint16_t signal_id, float value, uint32_t timestamp)
{
    /* sem seção crítica — pode ser interrompido por ISR no meio da escrita */
    s_ring[s_head].value = value;
    __asm volatile ("" ::: "memory");  /* barrier de compilador não é suficiente */
    s_head = next_head;
}
```

Se chamado de tarefa e de ISR, os campos do slot podem ser escritos parcialmente.
**Correção:** Envolver em `hal_enter_critical()` / `hal_exit_critical()`.

---

### 10. `HYSTERESIS_RPM` definido mas não usado — `ignition/rev_limiter.c`

```c
#define HYSTERESIS_RPM  100U   /* nunca aplicado na lógica */
```

Sem histerese, o limitador de rotação oscila continuamente no limiar, causando comportamento áspero.
**Correção:** Implementar histerese: cortar injeção ao atingir `RPM_LIMIT`, reativar ao cair abaixo de `RPM_LIMIT - HYSTERESIS_RPM`.

---

### 11. `lambda_ctrl_update()` não valida `dt_s > 0` — `fuel/lambda_ctrl.c` (~linha 64)

```c
s_integrator += error * KI * dt_s;
```

Se chamado com `dt_s = 0` (por bug no caller) ou valor negativo, o integrador acumula incorretamente.
**Correção:** `if (dt_s <= 0.0f) return;`

---

### 12. `engine_control_update()` chamado apenas em tooth 0 mas g_new_tooth_flag em todo dente — `main.c` (~linhas 86, 152-157)

`g_new_tooth_flag` é setado em cada dente, mas `engine_control_update()` retorna no início se `crank_get_tooth_index() != 0`. Isso consome CPU em cada dente para nada, e pode atrasar outros loops se a frequência de ISR for alta. É um desperdício de ciclos mas não é criticamente errado.

---

## BUGS MINOR

### 13. Retorno de `calibration_load()` ignorado — `main.c` linha 254

Se a flash estiver corrompida, o sistema usa defaults sem alertar ninguém.
**Correção:** Checar retorno e registrar DTC de falha de calibração.

---

### 14. Dois CRC32 independentes — `calibration/cal_flash.c` e `calibration/calibration.c`

Duplicação de código. Se um for corrigido e o outro não, checksums ficam incompatíveis.
**Correção:** Consolidar em `hal.h` / `hal.c` ou arquivo utilitário compartilhado.

---

### 15. Stubs sem assert/warning — `comm.c`, `cal_flash.c`, `diagnostics.c`, `sensors.c`

O README lista 4 módulos com stubs inacabados. Nenhum tem `#error` ou aviso de compilação para impedir uso acidental em produção.
**Correção:** Adicionar `#warning "STUB: verificar antes de usar em hardware real"` nos cabeçalhos correspondentes.

---

## Stubs / Código Incompleto (conforme README)

| Módulo | Descrição | Risco |
|--------|-----------|-------|
| `cal_flash.c` | Offset de message RAM simplificado | Alto — flash write/erase pode falhar |
| `comm.c` | Acesso ao message RAM do FDCAN com offset placeholder | Alto — CAN inoperante |
| `sensors.c` | Init do ADC não implementado | Alto — sem leitura de sensores |
| `diagnostics.c` | DTCs confirmados apenas em RAM, perdem ao reset | Médio |

---

## Prioridade de Correção

```
IMEDIATO (antes de ligar em hardware):
  1. Bug #1 — Timer truncado 16-bit (hal.c)
  2. Bug #3 — Offset CAN TX (comm.c)
  3. Bug #4 — Constante de fluxo ×1000 (fuel_control.c)
  4. Stubs de sensors.c e cal_flash.c

ALTA PRIORIDADE:
  5. Bug #2 — Race condition angle tracker
  6. Bug #5 — Bounds check no scheduler ISR
  7. Bug #6 — NaN em termistor
  8. Bug #9 — Race condition logging

MÉDIA PRIORIDADE:
  9. Bug #8 — Validação de frequência knock
  10. Bug #10 — Histerese do limitador de rotação
  11. Bugs #11-#15
```

---

*Análise estática — verificação em hardware recomendada após correções.*

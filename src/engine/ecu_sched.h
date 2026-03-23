#pragma once

#include <cstdint>

// =============================================================================
// ECU Scheduler — OpenEMS v2.2
//
// Gerencia o ângulo IVC (Inlet Valve Close) em graus ABDC (After Bottom Dead Center).
// Usado pelo cálculo de combustível para determinar o ponto de corte de injeção.
// =============================================================================

namespace ems::engine {

// Define o ângulo IVC em graus ABDC (0-180°)
// Chamado por TunerStudio quando o calibrador ajusta o parâmetro ivc_abdc_deg
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg) noexcept;

// Retorna o ângulo IVC atual em graus ABDC
uint8_t ecu_sched_get_ivc() noexcept;

}  // namespace ems::engine
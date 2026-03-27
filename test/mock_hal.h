/**
 * @file test/mock_hal.h
 * @brief Mock CMSIS registers for native testing
 * 
 * Este arquivo fornece mocks estáticos para os registradores CMSIS
 * usados pelos módulos HAL, permitindo testes nativos sem hardware.
 * 
 * ⚡ OPT-9: Mock ICACHE para testes de performance
 * ⚡ OPT-3: Mock CORDIC para testes de trigonometria
 * ⚡ OPT-1/OPT-2: Mock TIM2/TIM5 32-bit para testes de timestamp
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace mock {

// ──────────────────────────────────────────────────────────────────────────────
// Mock CMSIS registers (estáticos para testes nativos)
// ──────────────────────────────────────────────────────────────────────────────

// TIM2 (CKP 32-bit input capture)
namespace tim2 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CCR1 = 0u;
    static volatile uint32_t CCR2 = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFFFFFu;
    static volatile uint32_t CCMR1 = 0u;
    static volatile uint32_t CCER = 0u;
    static volatile uint32_t DIER = 0u;
    static volatile uint32_t SR = 0u;
    static volatile uint32_t EGR = 0u;
    static volatile uint32_t CR2 = 0u;
}

// TIM5 (Scheduler 32-bit timebase)
namespace tim5 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFFFFFu;
    static volatile uint32_t CR2 = 0u;
    static volatile uint32_t SMCR = 0u;
    static volatile uint32_t EGR = 0u;
}

// TIM1 (INJ1-3 output compare)
namespace tim1 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CCR1 = 0u;
    static volatile uint32_t CCR2 = 0u;
    static volatile uint32_t CCR3 = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFu;
    static volatile uint32_t CCMR1 = 0u;
    static volatile uint32_t CCMR2 = 0u;
    static volatile uint32_t CCER = 0u;
    static volatile uint32_t BDTR = 0u;
    static volatile uint32_t DIER = 0u;
    static volatile uint32_t SR = 0u;
    static volatile uint32_t EGR = 0u;
    static volatile uint32_t SMCR = 0u;
    static volatile uint32_t CR2 = 0u;
}

// TIM8 (IGN1-4 output compare)
namespace tim8 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CCR1 = 0u;
    static volatile uint32_t CCR2 = 0u;
    static volatile uint32_t CCR3 = 0u;
    static volatile uint32_t CCR4 = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFu;
    static volatile uint32_t CCMR1 = 0u;
    static volatile uint32_t CCMR2 = 0u;
    static volatile uint32_t CCER = 0u;
    static volatile uint32_t BDTR = 0u;
    static volatile uint32_t DIER = 0u;
    static volatile uint32_t SR = 0u;
    static volatile uint32_t EGR = 0u;
    static volatile uint32_t SMCR = 0u;
}

// TIM15 (INJ4 output compare)
namespace tim15 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CCR1 = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFu;
    static volatile uint32_t CCMR1 = 0u;
    static volatile uint32_t CCER = 0u;
    static volatile uint32_t DIER = 0u;
    static volatile uint32_t SR = 0u;
    static volatile uint32_t EGR = 0u;
    static volatile uint32_t SMCR = 0u;
}

// TIM3 (IACV/WG PWM)
namespace tim3 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CCR3 = 0u;
    static volatile uint32_t CCR4 = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFu;
    static volatile uint32_t CCMR2 = 0u;
    static volatile uint32_t CCER = 0u;
    static volatile uint32_t DIER = 0u;
    static volatile uint32_t EGR = 0u;
}

// TIM12 (VVT PWM)
namespace tim12 {
    static volatile uint32_t CNT = 0u;
    static volatile uint32_t CCR1 = 0u;
    static volatile uint32_t CCR2 = 0u;
    static volatile uint32_t CR1 = 0u;
    static volatile uint32_t PSC = 0u;
    static volatile uint32_t ARR = 0xFFFFu;
    static volatile uint32_t CCMR1 = 0u;
    static volatile uint32_t CCER = 0u;
    static volatile uint32_t DIER = 0u;
    static volatile uint32_t EGR = 0u;
}

// GPIOA (INJ1-3, CKP/CMP, BKIN)
namespace gpioa {
    static volatile uint32_t MODER = 0u;
    static volatile uint32_t AFRL = 0u;
    static volatile uint32_t AFRH = 0u;
    static volatile uint32_t OSPEEDR = 0u;
    static volatile uint32_t IDR = 0u;
    static volatile uint32_t BSRR = 0u;
}

// GPIOB (IACV/WG, VVT, BKIN)
namespace gpiob {
    static volatile uint32_t MODER = 0u;
    static volatile uint32_t AFRL = 0u;
    static volatile uint32_t AFRH = 0u;
    static volatile uint32_t OSPEEDR = 0u;
    static volatile uint32_t IDR = 0u;
    static volatile uint32_t BSRR = 0u;
}

// GPIOC (IGN1-4, INJ4)
namespace gpioc {
    static volatile uint32_t MODER = 0u;
    static volatile uint32_t AFRL = 0u;
    static volatile uint32_t AFRH = 0u;
    static volatile uint32_t OSPEEDR = 0u;
    static volatile uint32_t IDR = 0u;
    static volatile uint32_t BSRR = 0u;
}

// ADC1 (MAP, TPS, O2)
namespace adc1 {
    static volatile uint32_t DR = 0u;
    static volatile uint32_t CFGR2 = 0u;
    static volatile uint32_t SQR1 = 0u;
    static volatile uint32_t CR = 0u;
    static volatile uint32_t ISR = 0u;
    static volatile uint32_t IER = 0u;
}

// ADC2 (CLT, IAT, Fuel Press, Oil Press)
namespace adc2 {
    static volatile uint32_t DR = 0u;
    static volatile uint32_t CFGR2 = 0u;
    static volatile uint32_t SQR1 = 0u;
    static volatile uint32_t CR = 0u;
    static volatile uint32_t ISR = 0u;
    static volatile uint32_t IER = 0u;
}

// DAC1 (Knock threshold)
namespace dac1 {
    static volatile uint32_t DHR12R2 = 0u;
    static volatile uint32_t CR = 0u;
}

// CORDIC (Trigonometric coprocessor)
namespace cordic {
    static volatile uint32_t CSR = 0u;
    static volatile uint32_t WDATA = 0u;
    static volatile uint32_t RDATA = 0u;
}

// FMAC (Filter coprocessor)
namespace fmac {
    static volatile uint32_t CSR = 0u;
    static volatile uint32_t X1BUFCFG = 0u;
    static volatile uint32_t X2BUFCFG = 0u;
    static volatile uint32_t PARAM = 0u;
    static volatile uint32_t WDATA = 0u;
    static volatile uint32_t RDATA = 0u;
}

// GPDMA (General Purpose DMA)
namespace gpdma {
    static volatile uint32_t C0LLR = 0u;
    static volatile uint32_t C1LLR = 0u;
}

// RCC (Reset and Clock Control)
namespace rcc {
    static volatile uint32_t APB2ENR = 0u;
    static volatile uint32_t APB1LENR = 0u;
    static volatile uint32_t AHB1ENR = 0u;
    static volatile uint32_t AHB2ENR = 0u;
}

// FLASH (Flash memory control)
namespace flash {
    static volatile uint32_t ACR = 0u;
}

// ICACHE (Instruction Cache)
namespace icache {
    static volatile uint32_t CR = 0u;
    static volatile uint32_t SR = 0u;
    static volatile uint32_t MMONR = 0u;
}

// PWR (Power control)
namespace pwr {
    static volatile uint32_t DBPCR = 0u;
}

// ──────────────────────────────────────────────────────────────────────────────
// Buffer de DMA para ADC (alinhado para GPDMA)
// ──────────────────────────────────────────────────────────────────────────────
static uint16_t __attribute__((aligned(32))) adc1_buf[6] = {0};
static uint16_t __attribute__((aligned(32))) adc2_buf[4] = {0};

// ──────────────────────────────────────────────────────────────────────────────
// Funções de reset para testes
// ──────────────────────────────────────────────────────────────────────────────
void reset_all() noexcept {
    // Reset TIM2
    tim2::CNT = 0u;
    tim2::CCR1 = 0u;
    tim2::CCR2 = 0u;
    tim2::CR1 = 0u;
    tim2::PSC = 0u;
    tim2::ARR = 0xFFFFFFFFu;
    tim2::CCMR1 = 0u;
    tim2::CCER = 0u;
    tim2::DIER = 0u;
    tim2::SR = 0u;
    tim2::EGR = 0u;
    tim2::CR2 = 0u;

    // Reset TIM5
    tim5::CNT = 0u;
    tim5::CR1 = 0u;
    tim5::PSC = 0u;
    tim5::ARR = 0xFFFFFFFFu;
    tim5::CR2 = 0u;
    tim5::SMCR = 0u;
    tim5::EGR = 0u;

    // Reset TIM1
    tim1::CNT = 0u;
    tim1::CCR1 = 0u;
    tim1::CCR2 = 0u;
    tim1::CCR3 = 0u;
    tim1::CR1 = 0u;
    tim1::PSC = 0u;
    tim1::ARR = 0xFFFFu;
    tim1::CCMR1 = 0u;
    tim1::CCMR2 = 0u;
    tim1::CCER = 0u;
    tim1::BDTR = 0u;
    tim1::DIER = 0u;
    tim1::SR = 0u;
    tim1::EGR = 0u;
    tim1::SMCR = 0u;
    tim1::CR2 = 0u;

    // Reset TIM8
    tim8::CNT = 0u;
    tim8::CCR1 = 0u;
    tim8::CCR2 = 0u;
    tim8::CCR3 = 0u;
    tim8::CCR4 = 0u;
    tim8::CR1 = 0u;
    tim8::PSC = 0u;
    tim8::ARR = 0xFFFFu;
    tim8::CCMR1 = 0u;
    tim8::CCMR2 = 0u;
    tim8::CCER = 0u;
    tim8::BDTR = 0u;
    tim8::DIER = 0u;
    tim8::SR = 0u;
    tim8::EGR = 0u;
    tim8::SMCR = 0u;

    // Reset TIM15
    tim15::CNT = 0u;
    tim15::CCR1 = 0u;
    tim15::CR1 = 0u;
    tim15::PSC = 0u;
    tim15::ARR = 0xFFFFu;
    tim15::CCMR1 = 0u;
    tim15::CCER = 0u;
    tim15::DIER = 0u;
    tim15::SR = 0u;
    tim15::EGR = 0u;
    tim15::SMCR = 0u;

    // Reset GPIO
    gpioa::MODER = 0u;
    gpioa::AFRL = 0u;
    gpioa::AFRH = 0u;
    gpioa::OSPEEDR = 0u;
    gpioa::IDR = 0u;
    gpioa::BSRR = 0u;
    
    gpiob::MODER = 0u;
    gpiob::AFRL = 0u;
    gpiob::AFRH = 0u;
    gpiob::OSPEEDR = 0u;
    gpiob::IDR = 0u;
    gpiob::BSRR = 0u;
    
    gpioc::MODER = 0u;
    gpioc::AFRL = 0u;
    gpioc::AFRH = 0u;
    gpioc::OSPEEDR = 0u;
    gpioc::IDR = 0u;
    gpioc::BSRR = 0u;

    // Reset ADC
    adc1::DR = 0u;
    adc1::CFGR2 = 0u;
    adc1::SQR1 = 0u;
    adc1::CR = 0u;
    adc1::ISR = 0u;
    adc1::IER = 0u;
    
    adc2::DR = 0u;
    adc2::CFGR2 = 0u;
    adc2::SQR1 = 0u;
    adc2::CR = 0u;
    adc2::ISR = 0u;
    adc2::IER = 0u;

    // Reset DAC
    dac1::DHR12R2 = 0u;
    dac1::CR = 0u;

    // Reset CORDIC
    cordic::CSR = 0u;
    cordic::WDATA = 0u;
    cordic::RDATA = 0u;

    // Reset FMAC
    fmac::CSR = 0u;
    fmac::X1BUFCFG = 0u;
    fmac::X2BUFCFG = 0u;
    fmac::PARAM = 0u;
    fmac::WDATA = 0u;
    fmac::RDATA = 0u;

    // Reset GPDMA
    gpdma::C0LLR = 0u;
    gpdma::C1LLR = 0u;

    // Reset RCC
    rcc::APB2ENR = 0u;
    rcc::APB1LENR = 0u;
    rcc::AHB1ENR = 0u;
    rcc::AHB2ENR = 0u;

    // Reset FLASH
    flash::ACR = 0u;

    // Reset ICACHE
    icache::CR = 0u;
    icache::SR = 0u;
    icache::MMONR = 0u;

    // Reset PWR
    pwr::DBPCR = 0u;

    // Reset buffers
    std::memset(adc1_buf, 0, sizeof(adc1_buf));
    std::memset(adc2_buf, 0, sizeof(adc2_buf));
}

}  // namespace mock

// ──────────────────────────────────────────────────────────────────────────────
// Redefinição de macros para testes nativos
// ──────────────────────────────────────────────────────────────────────────────
#ifdef UNIT_TEST

// Redefinir registradores CMSIS para apontar para mocks
#define TIM2    ((TIM_TypeDef*)&mock::tim2)
#define TIM5    ((TIM_TypeDef*)&mock::tim5)
#define TIM1    ((TIM_TypeDef*)&mock::tim1)
#define TIM8    ((TIM_TypeDef*)&mock::tim8)
#define TIM15   ((TIM_TypeDef*)&mock::tim15)
#define TIM3    ((TIM_TypeDef*)&mock::tim3)
#define TIM12   ((TIM_TypeDef*)&mock::tim12)

#define GPIOA   ((GPIO_TypeDef*)&mock::gpioa)
#define GPIOB   ((GPIO_TypeDef*)&mock::gpiob)
#define GPIOC   ((GPIO_TypeDef*)&mock::gpioc)

#define ADC1    ((ADC_TypeDef*)&mock::adc1)
#define ADC2    ((ADC_TypeDef*)&mock::adc2)

#define DAC1    ((DAC_TypeDef*)&mock::dac1)

#define CORDIC  ((CORDIC_TypeDef*)&mock::cordic)
#define FMAC    ((FMAC_TypeDef*)&mock::fmac)
#define GPDMA1  ((GPDMA_TypeDef*)&mock::gpdma)

#define RCC     ((RCC_TypeDef*)&mock::rcc)
#define FLASH   ((FLASH_TypeDef*)&mock::flash)
#define ICACHE  ((ICACHE_TypeDef*)&mock::icache)
#define PWR     ((PWR_TypeDef*)&mock::pwr)

// Redefinir funções de NVIC para testes
#define nvic_set_priority(irq, priority)  ((void)0)
#define nvic_enable_irq(irq)              ((void)0)

// Redefinir IRQ numbers
#define IRQ_TIM2        0
#define IRQ_TIM1_CC     1
#define IRQ_TIM8_CC     2
#define IRQ_TIM15       3

#endif  // UNIT_TEST
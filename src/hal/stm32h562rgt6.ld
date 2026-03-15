/**
 * @file stm32h562rgt6.ld
 * @brief GNU Linker Script para STM32H562RGT6 (Cortex-M33, LQFP64)
 *
 * Mapa de memória (RM0481):
 *   Flash Bank1:  0x08000000 – 0x0807FFFF  (512 KB) ← código + dados RO
 *   Flash Bank2:  0x08100000 – 0x0817FFFF  (512 KB) ← NVM (flexnvm.cpp)
 *   SRAM1 (AXI): 0x20000000 – 0x2003FFFF  (256 KB) ← dados + heap + stack
 *
 * Layout do stack:
 *   Stack cresce para baixo a partir do topo da SRAM1 (0x20040000).
 *   _estack = topo da SRAM1. O linker reserva 4 KB mínimos de stack.
 *   MPU stack guard está configurado em system.cpp; o endereço deve ser
 *   atualizado para _estack - kExpectedMaxStackDepth após primeiro link.
 */

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM1  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

/* Topo do stack = fim da SRAM1 */
_estack = ORIGIN(SRAM1) + LENGTH(SRAM1);   /* 0x20040000 */

/* Tamanho mínimo de heap e stack — o linker gera erro se não couber */
_Min_Heap_Size  = 0x1000;   /* 4 KB */
_Min_Stack_Size = 0x2000;   /* 8 KB */

ENTRY(Reset_Handler)

SECTIONS
{
    /* ── Tabela de vetores de interrupção (deve estar no início da Flash) ── */
    .isr_vector :
    {
        . = ALIGN(512);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* ── Código e dados read-only ─────────────────────────────────────── */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Read-only data (constantes, strings) */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ARM exception unwind (necessário para C++ mesmo com -fno-exceptions) */
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    /* Construtores/destrutores estáticos C++ */
    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } > FLASH

    /* ── Dados inicializados (.data): LMA=Flash, VMA=SRAM ─────────────── */
    /* _sidata = Load Memory Address dos dados (na Flash)                    */
    /* _sdata/_edata = Virtual Memory Address (na SRAM, destino da cópia)   */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM1 AT > FLASH

    /* ── BSS (variáveis não inicializadas, zeramos no Reset_Handler) ───── */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > SRAM1

    /* ── Heap (alocação dinâmica — pouco usada neste firmware) ─────────── */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        __heap_start = .;
        . += _Min_Heap_Size;
        . += _Min_Stack_Size;
        . = ALIGN(8);
    } > SRAM1

    /* Verifica que o firmware cabe na Flash Bank1 */
    ASSERT(SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.data) < 512K,
           "ERROR: Firmware excede 512 KB da Flash Bank1")
}

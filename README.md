# stm32L432kc-nucleo32-demo

SRAM2 Demo on STM32L452KC Nucleo-32 Board (no fault)

Demo program to show that the problem with STM32L452 running code in SRAM2
does not affect the STM32L432KC. This demo is designed to run on an
STM32L432KC Nucleo-32 board.

Overview
====

This is an Eclipse project generated via STM32CubeMX.  It uses FreeRTOS to 
spawn two threads.  One thread blinks the green LED on the Nucleo board at
1Hz.  The other thread starts the ADC DMA process reading into a circular
buffer at 26400 sps.  These are placed on a message queue in the DMA
interrupt.  The thread then reads each block of these samples from the
queue and sends them to a FIR filter running in SRAM2.  The filter results
are discarded.

There are 3 GPIOs defined:

 * ADC_STATUS (PA11)
 * FIR_STATUS (PB5)
 * ERROR (PB4)
 
ADC_STATUS will flip between high and low as the DMA moves between
half-complete and complete.

FIR_STATUS will go high during FIR processing and low otherwise.

ERROR is raised in the hard fault handler. (This does not occur on the
STM32L432KC.)

This same code will run fine on the STM32L432KC Nucleo32 board and will
fail when using an STM32L452CE or STM32L452RE.

Placing Code in SRAM2
====

1. The section "bss2" is defined in the STM32L432RE_FLASH.ld linker script.
2. Code and data in bss2 is copied to SRAM2 by the
./startup/startup_stm32l432xx.S startup code.
3. The FIR filter code is placed in the bss2 section by modifying
./Drivers/CMSIS/Include/arm_math.h:

```c++
    void arm_fir_f32(
      const arm_fir_instance_f32 * S,
      float32_t * pSrc,
      float32_t * pDst,
      uint32_t blockSize) __attribute__((section(".bss2")));
```

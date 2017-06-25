# SRAM2 Demo on STM32L432KC Nucleo-32 Board

Demo program to show that the [problem with STM32L452 running code in SRAM2](https://github.com/mobilinkd/nucleo-l452-sram2-fault)
does not affect the STM32L432KC. This demo is designed to run on an
[STM32L432KC Nucleo-32](https://developer.mbed.org/platforms/ST-Nucleo-L432KC/) board.

This demo does show off a couple of features of the STM32L4 MCU that may be
of general interest.

 * ADC DMA into a circular buffer
 * How to put data and code in SRAM2
 * Double buffering the DMA data
 * Using FreeRTOS queues to move data from an interrupt to a thread 

# Overview

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

# Placing Code in SRAM2

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

SRAM2, when accessed at its standard 0x10000000 address has a direct
connection to the CPU core via the I and D (instruction and data) busses.
There are no wait states or buffering involved in processing code and data
from SRAM2 as there are from SRAM1 and Flash.

The same attribute shown above with the arm_fir_f32() function can be used
with data that needs to be accessed frequently such as the filter coefficients
or the filter input and output buffers.

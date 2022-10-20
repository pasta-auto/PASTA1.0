/*
 * MIT License
 *
 * Copyright (c) 2019 LandF Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// -*-c++-*-
// $RCSfile: StartupLFY63N.c,v $
// $Revision: 1.3 $
// $Date: 2016/03/14 01:17:29 $
//
//  YellowIDE7
//  Startup Routine, CSC63N.C
//
//  For LFY-RX63N1
//
// Copyright (c) 2015 LandF Corporation.
//
//  A user needs to modify mainly three parts
//  labeled as <<MODIFY1>>, <<MODIFY2>>, and <<MODIFY3>>.
//
//--------------------------------------------------------------------
//=============================header==============================
#include "YIDESYM.H" // STACK_SIZE HEAP_SIZE STRUCT_SIZE

// <<MODIFY1>>
#define RAM_BASE   0x0000000 // PLEASE SET RAM STARTING ADDRESS
#define CLOCK      96000000L // CPU clock (Hz)
#define PCLOCK     48000000L // Embedded peripheral clock (Hz)
#define MAX_VECTNO 256       // PLEASE SET MAX INTERRUPT NUM + 1
// See more details in the hardware manual.
// If you do not know MAX_VECTNO, use 256

#define DBG_PORT SCI1     // Serial port settings for debugging
//#define  DBG_PORT USB_R // Serial port settings for debugging
#define DBG_PORT_BPS 38400L // Baudrate for the debugging port
#define PIN_SIZE                                                               \
  144 // The number of pins: 144 (145 pins as well), 100, 64, 48, or 177

// Not use SDRAM
#define _SDRAM_NOUSE_

/*
 * USB Serial Number Definition
 *
 *   If you connect multiple boards to a single PC,
 *   you need to change a serial number to avoid conflicts.
 *   Please change the following serial number definition if necessary.
 */
#define USB_SERIAL_NO 0

// end of <<MODIFY1>>

#include <csheader.h>

/*
 * Simplified Memory Map for CPU R5F563NEDDFB
 *
 *   (On-chip ROM extended mode)
 *              0x00000000  - 0x0001FFFF        On-chip SRAM (128KB)
 *              0x00080000  - 0x000FFFFF        I/O
 *              0x08000000  - 0x09FFFFFF        SDRAM
 *              0xFFE00000  - 0xFFFFFFFF        On-chip ROM (2048KB)
 */

void __CSTARTUP_INIT2__(void);
extern long _STACK_TOP[];
extern long _USTACK_TOP[];
static unsigned long temp;
static unsigned char *p1, *p2;

// Macros to run a monitor on RAM (DO NOT CHANGE) 
#ifdef __TEXT_RAM__
extern unsigned char TEXT_RAM_rom, TEXT_RAM_start, DATA_CONST_RAM_rom,
    DATA_CONST_RAM_start;
extern unsigned long TEXT_RAM_size, DATA_CONST_RAM_size;
#endif // __TEXT_RAM__ 

#define _B1(a) (1 << (a))
#define _B0(a) (0)
#define _B(a, b) ((a) << (b))

#define BIT_4 _B1(4)
#define BIT_6 _B1(6)
#define BIT_7 _B1(7)
/*===================================================================
 * Option Setting Memory
 *===================================================================
 * Option setting memory areas are registers that control the microcontroller's state
 * after a reset. Since the option setting memory is allocated on ROM, settings
 * are determined on flashing the ROM image. Please use the following macros to
 * configure OFS0, OFS1, and MDES registers. Note that MDES is automatically
 * configured.
 */
/*****************************************
 * Option Function Select Register0 (OFS0)
****************************************/

// IWDT behavior after a reset
#define OFS0_IWDTSTART_AUTO                                                    \
  _B0(1) // IWDT is automatically started in Auto-Start mode.
#define OFS0_IWDTSTART_STOP _B1(1)   // IWDT is in the stop state
// IWDT time-out period
#define OFS0_IWDTTOPS_1024  _B(0, 2) // 1024 cycles
#define OFS0_IWDTTOPS_4096  _B(1, 2) // 409 cycles
#define OFS0_IWDTTOPS_8192  _B(2, 2) // 8192 cycles
#define OFS0_IWDTTOPS_16384 _B(3, 2) // 16384 cycles
// IWDT clock division ratio
#define OFS0_IWDTCKS_1   _B(0, 4)  // 1-divisor   (1 cycle = 131ms)
#define OFS0_IWDTCKS_16  _B(2, 4)  // 16-divisor  (1 cycle = 2.10s)
#define OFS0_IWDTCKS_32  _B(3, 4)  // 32-divisor  (1 cycle = 4.19s)
#define OFS0_IWDTCKS_64  _B(4, 4)  // 64-divisor  (1 cycle = 8.39s)
#define OFS0_IWDTCKS_128 _B(15, 4) // 128-divisor (1 cycle = 16.8s)
#define OFS0_IWDTCKS_256 _B(5, 4)  // 256-divisor (1 cycle = 33.6s)
// IWDT window end position
#define OFS0_IWDTRPES_75 _B(0, 8) // 75%
#define OFS0_IWDTRPES_50 _B(1, 8) // 50%
#define OFS0_IWDTRPES_25 _B(2, 8) // 25%
#define OFS0_IWDTRPES_0  _B(3, 8) // 0% (without window end position)
// IWDT window start position
#define OFS0_IWDTRPSS_25  _B(0, 10) // 25%
#define OFS0_IWDTRPSS_50  _B(1, 10) // 50%
#define OFS0_IWDTRPSS_75  _B(2, 10) // 75%
#define OFS0_IWDTRPSS_100 _B(3, 10) // 100% (without window start position)
// IWDT reset interupt request
#define OFS0_IWDTRSTIRQS_NMI   _B0(12) // Allow non-maskable interrupts
#define OFS0_IWDTRSTIRQS_RESET _B1(12) // "Reset" is enabled
// IWDT sleep mode count stop
#define OFS0_IWDTSLCSTP_DIS   _B0(14) // Sleep mode count stop flag
#define OFS0_IWDTSLCSTP_SLEEP _B1(14) // Enable count stop in sleep
/* mode, software standby mode
 * deep software standby mode,
 * and all module clock stop mode.
 * WDT behavior after a reset
 */
#define OFS0_WDTSTRT_AUTO                                                      \
  _B0(17) // WDT is automatically started in Auto-Start mode.
#define OFS0_WDTSTRT_STOP _B1(17) // WDT is in the stop state

// WDT time-out period
#define OFS0_WDTTOPS_1024  _B(0, 18) // 1024 cycles
#define OFS0_WDTTOPS_4096  _B(1, 18) // 4096 cycles
#define OFS0_WDTTOPS_8192  _B(2, 18) // 8192 cycles
#define OFS0_WDTTOPS_16384 _B(3, 18) // 16384 cycles
// WDT clock division ratio
#define OFS0_WDTCKS_4    _B(1, 20)  // Clock division ratio: 4
#define OFS0_WDTCKS_64   _B(4, 20)  // Clock division ratio: 64
#define OFS0_WDTCKS_128  _B(15, 20) // Clock division ratio: 128
#define OFS0_WDTCKS_512  _B(6, 20)  // Clock division ratio: 512
#define OFS0_WDTCKS_2048 _B(7, 20)  // Clock division ratio: 2048
#define OFS0_WDTCKS_8192 _B(8, 20)  // Clock division ratio: 8192
// WDT window end position
#define OFS0_WDTRPES_75 _B(0, 24) // 75%
#define OFS0_WDTRPES_50 _B(1, 24) // 50%
#define OFS0_WDTRPES_25 _B(2, 24) // 25%
#define OFS0_WDTRPES_0  _B(3, 24) // 0% (widhout window end position)
// WDT window start position
#define OFS0_WDTRPSS_25  _B(0, 26) // 25%
#define OFS0_WDTRPSS_50  _B(1, 26) // 50%
#define OFS0_WDTRPSS_75  _B(2, 26) // 75%
#define OFS0_WDTRPSS_100 _B(3, 26) // 100% (without window start position)
// WDT reset interrupt
#define OFS0_WDTRSTIRQS_NMI   _B0(28) // Allow non-maskable interrupts
#define OFS0_WDTRSTIRQS_RESET _B1(28) // "Reset" is enabled

#define OFS0_DEF 0xe001a001

// OFS0 settings 
#define OFS0                                                                   \
  (OFS0_DEF | OFS0_IWDTSTART_STOP | OFS0_IWDTTOPS_16384 | OFS0_IWDTCKS_128 |   \
   OFS0_IWDTRPES_0 | OFS0_IWDTRPSS_100 | OFS0_IWDTRSTIRQS_RESET |              \
   OFS0_IWDTSLCSTP_SLEEP | OFS0_WDTSTRT_STOP | OFS0_WDTTOPS_16384 |            \
   OFS0_WDTCKS_128 | OFS0_WDTRPES_0 | OFS0_WDTRPSS_100 |                       \
   OFS0_WDTRSTIRQS_RESET)

/*****************************************
   Option Function Select Register 1 (OFS1)
****************************************/

// Voltage monitoring 0 startup bit
#define OFS1_LVDAS_VALID                                                       \
  _B0(2) // the voltage monitoring 0 reset is enabled after a reset
#define OFS1_LVDAS_INVALID                                                     \
  _B1(2) // the voltage monitoring 0 reset is disabled after a reset
// High-speed clock oscillator (HOCO) enable bit
#define OFS1_HOCOEN_VALID   _B0(8) // HOCO is enabled after a reset
#define OFS1_HOCOEN_INVALID _B1(8) // HOCO is disabled after a reset

#define OFS1_DEF 0xfffffefb

// OFS1 settings 
#define OFS1 (OFS1_DEF | OFS1_LVDAS_INVALID | OFS1_HOCOEN_INVALID)

/* ----------------------------------------------------------------------------------------
 *  Endian select register S (MDES)
 * ----------------------------------------------------------------------------------------*/
// Auto-configured by compiler definition macros 
#define MDES_BIG 0xfffffff8
#define MDES_LTTTLE 0xffffffff
// MDES settings 
#ifdef __LITTLE_ENDIAN__
#define MDES MDES_LTTTLE
#else
#define MDES MDES_BIG
#endif // __LITTLE_ENDIAN__ 

//===================================================================
//  I/O definitions
//===================================================================
// System clock control register
#define SCKCR (*((volatile unsigned long *)0x80020))
#define SCKCR_MASK 0x00000011
#define SCKCR_PCKB1  _B(0, 8)  // Peripheral module clock B,  1-divisor
#define SCKCR_PCKB2  _B(1, 8)  //                             2-divisor
#define SCKCR_PCKB4  _B(2, 8)  //                             4-divisor
#define SCKCR_PCKB8  _B(3, 8)  //                             8-divisor
#define SCKCR_PCKB16 _B(4, 8)  //                            16-divisor
#define SCKCR_PCKB32 _B(5, 8)  //                            32-divisor
#define SCKCR_PCKB64 _B(6, 8)  //                            64-divisor
#define SCKCR_PCKA1  _B(0, 12) // Peripheral module clock A,  1-divisor
#define SCKCR_PCKA2  _B(1, 12) //                             2-divisor
#define SCKCR_PCKA4  _B(2, 12) //                             4-divisor
#define SCKCR_PCKA8  _B(3, 12) //                             8-divisor
#define SCKCR_PCKA16 _B(4, 12) //                            16-divisor
#define SCKCR_PCKA32 _B(5, 12) //                            32-divisor
#define SCKCR_PCKA64 _B(6, 12) //                            64-divisor
#define SCKCR_BCK1   _B(0, 16) // External bus clock,         1-divisor
#define SCKCR_BCK2   _B(1, 16) //                             2-divisor
#define SCKCR_BCK4   _B(2, 16) //                             4-divisor
#define SCKCR_BCK8   _B(3, 16) //                             8-divisor
#define SCKCR_BCK16  _B(4, 16) //                            16-divisor
#define SCKCR_BCK32  _B(5, 16) //                            32-divisor
#define SCKCR_BCK64  _B(6, 16) //                            64-divsior

#define PSTOP0_BIT   _B1(22)  // SDCLK pin output control bit:
                              // 0 --- enabled, 1 --- disabled
#define PSTOP1_BIT   _B1(23)  // BCLK pin output control bit:
                              // 0 --- enabled, 1 --- disabled
#define SCKCR_ICK1  _B(0, 24) // System clock          1-divisor
#define SCKCR_ICK2  _B(1, 24) //                       2-divisor
#define SCKCR_ICK4  _B(2, 24) //                       4-divisor
#define SCKCR_ICK8  _B(3, 24) //                       8-divisor
#define SCKCR_ICK16 _B(4, 24) //                      16-divisor
#define SCKCR_ICK32 _B(5, 24) //                      32-divisor
#define SCKCR_ICK64 _B(6, 24) //                      64-divisor
#define SCKCR_FCK1  _B(0, 28) // FlashIF Clock         1-divisor
#define SCKCR_FCK2  _B(1, 28) //                       2-divisor
#define SCKCR_FCK4  _B(2, 28) //                       4-divisor
#define SCKCR_FCK8  _B(3, 28) //                       8-divisor
#define SCKCR_FCK16 _B(4, 28) //                      16-divisor
#define SCKCR_FCK32 _B(5, 28) //                      32-divisor
#define SCKCR_FCK64 _B(6, 28) //                      64-divisor

/* Settings for embedded clock frequency division ratios
 * b31:b28      FCK         FlashIF clock             - 4-divisor
 * b27:b24      ICK         System clock              - 2-divisor
 * b23          PSTOP1      BCLK output control       - Disabled
 * b22          PSTOP0      SDCLK output control      - Disabled
 * b21:b20      Reserved    Reserved
 * b19:b16      BCK         External bus clock        - 4-divisor
 * b15:b12      PCLKA       Peripheral module clock A - 2-divisor
 * b11:b8       PCLKB       Peripheral module clock B - 4-divisor
 */
#define SCKCR_VAL                                                              \
  (SCKCR_MASK | SCKCR_FCK4 | SCKCR_ICK2 | SCKCR_PCKB4 | SCKCR_PCKA2 |          \
   SCKCR_BCK4 | PSTOP1_BIT | PSTOP0_BIT)

// System clock control register2
#define SCKCR2 (*((volatile unsigned short *)0x80024))

// System clock control register3
#define SCKCR3 (*((volatile unsigned short *)0x80026))
#define SCKCR3_LOCO _B(0, 8) // Clock source select bit LOCO
#define SCKCR3_HOCO _B(1, 8) //                         HOCO
#define SCKCR3_MAIN _B(2, 8) //                         Main clock
#define SCKCR3_SUB  _B(3, 8) //                         Sub clock
#define SCKCR3_PLL  _B(4, 8) //                         PLL

// External bus control register
#define BCKCR (*((volatile unsigned char *)0x80030))

// High-speed on-chip oscillator control register
#define HOCOCR (*((volatile unsigned char *)0x80036))
#define HCSTP_BIT _B1(0)

// High-speed on-chip oscillator power supply control register
#define HOCOPCR (*((volatile unsigned char *)0x8C294))

// System control register 0
#define SYSCR0 (*((volatile unsigned short *)0x80006))
#define SYSCR0_ROME _B1(0)     // On-chip ROM enable bit
#define SYSCR0_EXBE _B1(1)     // External BUS enable bit
#define SYSCR0_KEY _B(0x5A, 8) // SYSCR0 key code
#define SYSCR1_RAME _B1(0)     // On-chip RAM enbale bit

// Main Clock Oscillator Wait Control Register
#define MOSCWTCR (*((volatile unsigned char *)0x800A2))

// Main Clock Oscillator Control Register
#define MOSCCR (*((volatile unsigned char *)0x80032))

//  PLL control register (PLLCR)
#define PLLCR (*((volatile unsigned short *)0x80028))
//  PLL input frequency division ratio
#define PLLCR_PLIDIV_1 _B(0, 0) // 1-divisor
#define PLLCR_PLIDIV_2 _B(1, 0) // 2-divisor
#define PLLCR_PLIDIV_4 _B(2, 0) // 4-divisor
// PLL frequency multiplication factor
#define PLLCR_STC_8  _B(7 , 8) // x8
#define PLLCR_STC_10 _B(9 , 8) // x10
#define PLLCR_STC_12 _B(11, 8) // x12
#define PLLCR_STC_16 _B(15, 8) // x16
#define PLLCR_STC_20 _B(19, 8) // x20
#define PLLCR_STC_24 _B(23, 8) // x24
#define PLLCR_STC_25 _B(24, 8) // x25
#define PLLCR_STC_50 _B(49, 8) // x50

//  PLL control register 2 (PLLCR2)
#define PLLCR2 (*((volatile unsigned char *)0x8002a))

//  PLL wait control register (PLLWTCR)
#define PLLWTCR (*((volatile unsigned char *)0x800A6))
#define PLLWTCR_16_CYL      _B(0 , 0) //      16 cycles
#define PLLWTCR_32_CYL      _B(1 , 0) //      32 cycles
#define PLLWTCR_64_CYL      _B(2 , 0) //      64 cycles
#define PLLWTCR_512_CYL     _B(3 , 0) //     512 cycles
#define PLLWTCR_1024_CYL    _B(4 , 0) //    1024 cycles
#define PLLWTCR_2048_CYL    _B(5 , 0) //    2048 cycles
#define PLLWTCR_4096_CYL    _B(6 , 0) //    4096 cycles
#define PLLWTCR_16384_CYL   _B(7 , 0) //   16384 cycles
#define PLLWTCR_32768_CYL   _B(8 , 0) //   32768 cycles
#define PLLWTCR_65536_CYL   _B(9 , 0) //   65536 cycles
#define PLLWTCR_131072_CYL  _B(10, 0) //  131072 cycles
#define PLLWTCR_262144_CYL  _B(11, 0) //  262144 cycles
#define PLLWTCR_524288_CYL  _B(12, 0) //  524288 cycles
#define PLLWTCR_1048576_CYL _B(13, 0) // 1048576 cycles
#define PLLWTCR_2097152_CYL _B(14, 0) // 2097152 cycles
#define PLLWTCR_4194304_CYL _B(15, 0) // 4194304 cycles

// Port direction register (PDR)
#define P0DR                                                                   \
  (*((volatile unsigned char *)0x8c000)) // PORT0 port direction regster
#define P1DR                                                                   \
  (*((volatile unsigned char *)0x8c001)) // PORT1 port direction regster
#define P2DR                                                                   \
  (*((volatile unsigned char *)0x8c002)) // PORT2 port direction regster
#define P3DR                                                                   \
  (*((volatile unsigned char *)0x8c003)) // PORT3 port direction regster
#define P4DR                                                                   \
  (*((volatile unsigned char *)0x8c004)) // PORT4 port direction regster
#define P5DR                                                                   \
  (*((volatile unsigned char *)0x8c005)) // PORT5 port direction regster
#define P6DR                                                                   \
  (*((volatile unsigned char *)0x8c006)) // PORT6 port direction regster
#define P7DR                                                                   \
  (*((volatile unsigned char *)0x8c007)) // PORT7 port direction regster
#define P8DR                                                                   \
  (*((volatile unsigned char *)0x8c008)) // PORT8 port direction regster
#define P9DR                                                                   \
  (*((volatile unsigned char *)0x8c009)) // PORT9 port direction regster
#define PADR                                                                   \
  (*((volatile unsigned char *)0x8c00A)) // PORTA port direction regster
#define PBDR                                                                   \
  (*((volatile unsigned char *)0x8c00B)) // PORTB port direction regster
#define PCDR                                                                   \
  (*((volatile unsigned char *)0x8c00C)) // PORTC port direction regster
#define PDDR                                                                   \
  (*((volatile unsigned char *)0x8c00D)) // PORTD port direction regster
#define PEDR                                                                   \
  (*((volatile unsigned char *)0x8c00E)) // PORTE port direction regster
#define PFDR                                                                   \
  (*((volatile unsigned char *)0x8c00F)) // PORTF port direction regster
#define PGDR                                                                   \
  (*((volatile unsigned char *)0x8c010)) // PORTG port direction regster
#define PJDR                                                                   \
  (*((volatile unsigned char *)0x8c012)) // PORTJ port direction regster

// port output data register (PODR)
#define P0ODR (*((volatile unsigned char *)0x8c020))
#define P1ODR (*((volatile unsigned char *)0x8c021))
#define P2ODR (*((volatile unsigned char *)0x8c022))

// port input data register (PIDR)
#define P0IDR (*((volatile unsigned char *)0x8c040))
#define PFIDR (*((volatile unsigned char *)0x8c04f))
#define PCIDR (*((volatile unsigned char *)0x8c04c))
#define PJIDR (*((volatile unsigned char *)0x8c052))

// port mode register (PMR)
#define P0MR (*((volatile unsigned char *)0x8c060))
#define P1MR (*((volatile unsigned char *)0x8c061))
#define P2MR (*((volatile unsigned char *)0x8c062))
#define P3MR (*((volatile unsigned char *)0x8c063))

#if PIN_SIZE == 144
#define NE_P1PDR 0x03 // does not exist: P10, P11
#define NE_P5PDR 0x80 // does not exist: P57
#define NE_P8PDR 0x30 // does not exist: P84,P85
#define NE_P9PDR 0xF0 // does not exist: P94 to P97
#define NE_PFPDR 0x1F // does not exist: PF0 to PF4
#define NE_PGPDR 0xFF // does not exist: PG0 to PG7
#elif PIN_SIZE == 100
#define NE_P0PDR 0x0F // does not exist: P00 to P03
#define NE_P1PDR 0x03 // does not exist: P10,P11
#define NE_P5PDR 0xC0 // does not exist: P56,P57
#define NE_P6PDR 0xFF // does not exist: P60 to P67
#define NE_P7PDR 0xFF // does not exist: P70 to P77
#define NE_P8PDR 0xFF // does not exist: P80 to P87
#define NE_P9PDR 0xFF // does not exist: P90 to P97
#define NE_PFPDR 0x3F // does not exist: PF0 to PF5
#define NE_PGPDR 0xFF // does not exist: PG0 to PG7
#define NE_PJPDR 0x20 // does not exist: PJ5
#endif

// Module stop control registers
#define MSTPCRA                                                                \
  (*((volatile unsigned long *)0x80010)) // Module stop control register A
#define MSTPCRB                                                                \
  (*((volatile unsigned long *)0x80014)) // Module stop control register B
#define MSTPCRC                                                                \
  (*((volatile unsigned long *)0x80018)) // Module stop control register C

#define MSTPCRA_DMAC_BIT 28   // DMAC
#define MSTPCRA_EXDMAC_BIT 29 // EXDMAC
#define MSTPCRB_USB1_BIT                                                       \
  18 // Universal serial bus interface 1 (BGA package only)
#define MSTPCRB_USB0_BIT 19 // Universal serial bus interface 0
#define MSTPCRB_SCI6_BIT 25 // Serial communication interface 6
#define MSTPCRB_SCI5_BIT 26 // Serial communication interface 5
#define MSTPCRB_SCI3_BIT 28 // Serial communication interface 3
#define MSTPCRB_SCI2_BIT 29 // Serial communication interface 2
#define MSTPCRB_SCI1_BIT 30 // Serial communication interface 1
#define MSTPCRB_SCI0_BIT 31 // Serial communication interface 0

// Protect register (PRCR)
#define PRCR (*((volatile unsigned short *)0x803fe))
#define PRCR_KEY 0xA500
#define PRCR_PRC0 _B1(0) // Clock generation
#define PRCR_PRC1 _B1(1) // Low power, and module stop control
#define PRCR_PRC3 _B1(3) // LVD

// System configuration control register (SYSCFG) Use USB0
#define SYSCFG0 (*((volatile unsigned short *)0xa0000))
#define SYSCFG0_USBE_ENA   _B1(0)  // USB enable
#define SYSCFG0_DPRPU_ENA  _B1(4)  // Pull-up enable
#define SYSCFG0_DRPD_ENA   _B1(5)  // Pull-down enable
#define SYSCFG0_DCFM_F     _B0(6)  // Use peripheral control function
#define SYSCFG0_DCFM_H     _B1(6)  // Use host control function NEED TO REVIEW
#define SYSCFG0_USBCLK_DIS _B0(10) // Disable generating USB clock
#define SYSCFG0_USBCLK_ENA _B1(10) // Enable generating USB clock

// Bus error monitoring enable register (BEREN)
#define BEREN (*((volatile unsigned char *)0x81304))
#define BEREN_IGAEN_DIS _B0(0) // Illegal address access detection enable bit
#define BEREN_TOEN_DIS  _B0(1) // Timeout detection enable bit

#define BUSPRI                                                                 \
  (*((volatile unsigned short *)0x81310)) // Bus Priority Control Register    
                                          // (BUSPRI)
#define BUSPRI_BPRA_FIX _B(0, 0)  // Memory bus 1 (RAM)  priority fix
#define BUSPRI_BPRA_TOG _B(1, 0)  //                     priority toggle
#define BUSPRI_BPRO_FIX _B(0, 2)  // Memory bus 2 (ROM)  priority fix
#define BUSPRI_BPRO_TOG _B(1, 2)  //                     priority toggle
#define BUSPRI_BPIB_FIX _B(0, 4)  // Internal peripheral bus 1    priority fix
#define BUSPRI_BPIB_TOG _B(1, 4)  //                              priority toggle
#define BUSPRI_BPGB_FIX _B(0, 6)  // Internal peripheral bus 2,3  priority fix
#define BUSPRI_BPGB_TOG _B(1, 6)  //                              priority toggle
#define BUSPRI_BPHB_FIX _B(0, 8)  // Internal peripheral bus 4,5  priority fix
#define BUSPRI_BPHB_TOG _B(1, 8)  //                              priority toggle
#define BUSPRI_BPFB_FIX _B(0, 10) // Internal peripheral bus 6   priority fix
#define BUSPRI_BPFB_TOG _B(1, 10) //                             priority toggle
#define BUSPRI_BPEB_FIX _B(0, 12) // External bus                priority fix
#define BUSPRI_BPEB_TOG _B(1, 12) //                             priority toggle

// Multi-Function pin controller (MPC)
#define PFAOE0                                                                 \
  (*((volatile unsigned char *)0x8C104)) // Address output enable register1
#define PFAOE1                                                                 \
  (*((volatile unsigned char *)0x8C105)) // Address output enable register2
#define PFBCR0                                                                 \
  (*((volatile unsigned char *)0x8C106)) // External bus control register1
#define PFBCR1                                                                 \
  (*((volatile unsigned char *)0x8C107)) // External bus control register2
#define PWPR (*((volatile unsigned char *)0x8C11F)) // Write-protect register
#define B0WI_BIT _B1(7)
#define PFSWE_BIT _B1(6)

#define P00FS (*((volatile unsigned char *)0x8C140)) // P00 pin function control
// register
#define P01FS (*((volatile unsigned char *)0x8C141)) // P01 pin function control
// register
#define P14FS (*((volatile unsigned char *)0x8C14C)) // P14 pin function control
// register
#define P16FS (*((volatile unsigned char *)0x8C14E)) // P16 pin function control
// register
#define P26FS (*((volatile unsigned char *)0x8C156)) // P26 pin function control
// register
#define P30FS (*((volatile unsigned char *)0x8C158)) // P30 pin function control
// register
#define PFUSB0 (*((volatile unsigned char *)0x8C114)) // USB0 control register
#define PUPHZS_BIT _B1(2) // PUPHZ select bit
#define PDHZS_BIT  _B1(3) // PDHZ select bit

#define SDCLKE_DIS _B0(7) // SDCLK disable
#define SDCLKE_ENA _B1(7) // SDCLK enable

#define SDRFCR                                                                 \
  (*((volatile unsigned short *)0x83C14)) // SDRAM refresh control register
// (SDRFCR)
#define SDCMOD                                                                 \
  (*((volatile unsigned char *)0x83C01))  // SDC mode register (SDCMOD)
#define SDAMOD                                                                 \
  (*((volatile unsigned char *)0x83C02))  // SDRAM access mode register
// (SDAMOD)
#define SDRFEN                                                                 \
  (*((volatile unsigned char *)0x83C16))  // SDRAM auto-refresh control
// register (SDRFEN)
#define SDIR                                                                   \
  (*((volatile unsigned short *)0x83C24)) // SDRAM initialization register
// (SDIR)
#define SDADR                                                                  \
  (*((volatile unsigned char *)0x83C40))  // SDRAM address register (SDADR)
#define SDTR                                                                   \
  (*((volatile unsigned long *)0x83C44))  // SDRAM timing register (SDTR)
#define SDMOD                                                                  \
  (*((volatile unsigned short *)0x83C48)) // SDRAM mode register (SDMOD)
#define SDSR                                                                   \
  (*((volatile unsigned char *)0x83C50))  // SDRAM status register (SDSR)
#define SDICR                                                                  \
  (*((volatile unsigned char *)0x83C20))  // SDRAM initialization sequence
// control register (SDICR)
#define SDCCR                                                                  \
  (*((volatile unsigned char *)0x83C00)) // SDC control register(SDCCR)
#define SDC_EXENB_DIS _B0(0)             // Disable SDRAM Operation
#define SDC_EXENB_ENA _B1(0)             // Enable SDRAM Operation
#define SDC_BSIZE_16  _B(0, 4)           // SDRAM bus width 16 bits
#define SDC_BSIZE_32  _B(1, 4)           // SDRAM bus width 32 bits
#define SDC_BSIZE_8   _B(2, 4)           // SDRAM bus width  8 bits

#define RCR1 (*((volatile unsigned char *)0x8C422)) // RTC control register 1
#define RCR2 (*((volatile unsigned char *)0x8C424)) // RTC control register 2
#define RCR3 (*((volatile unsigned char *)0x8C426)) // RTC control register 3
#define RCR4 (*((volatile unsigned char *)0x8C428)) // RTC control register 4
#define SOSCCR                                                                 \
  (*((volatile unsigned char *)0x80033)) // Sub-clock oscillator control
// register

//=================Program starts from here========================
void __startup_func__ START(void);
void interrupt __vectno__ { 24 } _restart_pos(void)
{
    START();
}
void __startup_func__ START(void)
{
    int wait;
    int i;
/*---------------------------------------------------------------------
 *  Stack pointer and PSW initialization (DO NOT CHANGE)
 *---------------------------------------------------------------------*/

#ifndef __YIDE_MON_MAKE__
#ifdef __SUB_PROJECT__
_asm MVTC #_SYSTEM_STACK_TOP  + 4, ISP 
_asm MVTC #_SYSTEM_USTACK_TOP + 4, USP // Stack for monitor and downloader
#else
_asm MVTC #0x3CFFC, ISP // Interrupt handler stack
_asm MVTC #0x39FFC, USP // User stack
#endif
#else
_asm extern ___mm_monitor_stack ___mm_monitor_ustack 
_asm MVTC #___mm_monitor_stack, ISP // Monitor stack
_asm MVTC #___mm_monitor_ustack,USP // Monitor stack
#endif
_asm MVTC #0, PSW // PSW initialization

_asm extern _main_start 
_asm MVTC #_main_start, INTB // INTB

/* ---------------------------------------------------------------------
 *  Clock configuration
 * ---------------------------------------------------------------------
 *
 *    I phi=96MHz, P phi=48MHz, B phi=48MHz,SDCLK enabled
 */
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__)) //<- Required for
    // ROMization and
    // ROM debbuging

    // Unlock register protection
    PRCR = (PRCR_KEY | PRCR_PRC1 | PRCR_PRC0);

    // Disable peripheral functions (DMAC/DTC,EXDMAC)
    MSTPCRA |= (_B1(MSTPCRA_DMAC_BIT) | _B1(MSTPCRA_EXDMAC_BIT));
// Output settings: set "1" to non-existent ports
#if PIN_SIZE == 144
    P1DR = (P1DR | NE_P1PDR);
    P5DR = (P5DR | NE_P5PDR);
    P8DR = (P8DR | NE_P8PDR);
    P9DR = (P9DR | NE_P9PDR);
    PFDR = (PFDR | NE_PFPDR);
    PGDR = (PGDR | NE_PGPDR);
#endif

    // Main-clock oscillator settings
    P3MR |= (BIT_6 | BIT_7);

    // Main-clock oscillator wait control
    MOSCWTCR = 0x0d; // wait 131072 cycles ( 10ms )

    /* Main-clock oscillator
     * b7:b1  Reserved
     * b0     Stop main-clock oscillator - main-clock oscillator operation =
     * 0
     */
    MOSCCR = 0x00;
    do {
    } while (0x00 != MOSCCR);

    wait = 575; // 23000/(8us x 5(cycle)
    do {
    } while (--wait);

    // PLL settings

    /* PLL control register PLL input clock division selection bit:
     * 1-divisor
     * and frequency multiplication: x16
     */
    PLLCR = (PLLCR_STC_16 | PLLCR_PLIDIV_1);
    
    /* PLLF 192MHz PLL clock stable period: 500usec
     * 500 ~ 192 = 96000 => wait time 131072
     */
    PLLWTCR = PLLWTCR_131072_CYL;

    // PLL ON
    PLLCR2 = 0x00;

    wait = 300;
    do {
    } while (--wait);

    // Clock divisor settings

    /* Configure internal clock divisor
     * b31:b28      FCK       FlashIF clock        - 4-divisor
     * b27:b24      ICK       System clock         - 2-dovisor
     * b23          PSTOP1    BCLK output control  - stop
     * b22          PSTOP0    SDCLK output control - stop
     * b21:b20      Reserved  Reserved
     * b19:b16      BCK       External bus clock       - 4-divisor
     * b15:b12      PCLKA     Peripheral module clockA - 2-divisor
     * b11:b8       PCLKB     Peripheral module clockB - 4-divisor
     */
    SCKCR = SCKCR_VAL;
    do {
    } while (SCKCR_VAL != SCKCR);

    SCKCR2 = 0x0032;
    do {
    } while (0x0032 != SCKCR2);

    /* External bus clock control
     * No divisor */
    BCKCR = 0x00;
    do {
    } while (0x00 != BCKCR);

    // Clock source selection
    SCKCR3 = SCKCR3_PLL;
    do {
    } while (SCKCR3_PLL != SCKCR3);

    // Stop HOCO
    HOCOCR = HCSTP_BIT;
    // HOCO power supply off
    HOCOPCR = 0x01;

    // Protect register
    PRCR = PRCR_KEY;

#endif // #if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__))

/*<<MODIFY>>
 *======================================================================
 *  Initialize off-chip RAM
 *  ------Please modify this section for your CPU board -----
 *  It is required to initialize off-chip RAM (bus settings) when
 *  you use off-chip RAM. Please comment-out or remove this section when
 *  you use only on-chip RAM.
 *======================================================================*/
/*======================================================================
 * I/O address definition
 * Please check I/O address definition for CPUs other than LFY-63N1.
 *======================================================================*/

#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__)) //<- Only for
// ROMization and ROM debugging

// External bus settings are defined here 
#ifndef _SDRAM_NOUSE_
    /* SDRAM initialization
     * Unlock register protect */
    PRCR = PRCR_KEY | PRCR_PRC1 | PRCR_PRC0;

    // Stop SDCLK,BCLK
    SCKCR |= (PSTOP1_BIT | PSTOP0_BIT);

    // Stop bus error monitoring functions
    BEREN = (BEREN_IGAEN_DIS | BEREN_TOEN_DIS);

    // Bus priority
    BUSPRI =
      (BUSPRI_BPRA_FIX | BUSPRI_BPRO_FIX | BUSPRI_BPIB_FIX | BUSPRI_BPGB_FIX |
       BUSPRI_BPHB_FIX | BUSPRI_BPFB_FIX | BUSPRI_BPEB_FIX);

    /* SDRAM pin function
     * ADRLE  = 1 : Configure PA0-PA7 as external address bus A0-A7
     * ADRHMS = 0 : Configure PC0-PC7 as external address bus A16-A23
     * DHE    = 1 : Configure PE0-PE7 as external data bus D8-D15
     * DH32E  = 0 : Configure PG0-PG7, P90-P97 as I/O ports */
    PFBCR0 = 0x11;

    /* MDSDE  = 1 : Enable CKE,SDCS#,RAS#,CAS#,WE#,DQM0
     * DQM1E  = 1 : Enable DQM1
     * SDCLKE = 1 : Enable SDCLK */
    PFBCR1 = 0xD0;

    /* A15E=1 : Disable A15
     * A14E=1 : Enable A14
     * A13E=1 : Enable A13
     * A12E=1 : Enable A12
     * A11E=1 : Enable A11
     * A10E=1 : Enable A10
     *  A9E=1 : Enable A9
     *  A8E=1 : Enable A8 */
    PFAOE0 = 0xFF;

    /* A23E=0 : Disable A23
     * A22E=0 : Disable A22
     * A21E=0 : Disable A21
     * A20E=0 : Disable A20
     * A19E=0 : Disable A19
     * A18E=0 : Disable A18
     * A17E=0 : Disable A17
     * A16E=0 : Disable A16 */
    PFAOE1 = 0x00;

    // Configure input ports
    PADR = 0x00;   // Use input ports as A0-A7
    PBDR = 0x00;   // Use input ports as A8-A14
    PDDR = 0x00;   // Use input ports as D0-D7
    PEDR = 0x00;   // Use input ports as D8-D15
    P6DR = 0x00;   // Use input ports as CKE,SDCS#,RAS#,CAS#,W#,DQM0,DQM1
    P7DR &= ~0x01; // P70 = SDCLK pin input port

    // Enable On-chop ROM/External bus
    SYSCR0 = (SYSCR0_KEY | SYSCR0_EXBE | SYSCR0_ROME); // 0x5A03

    while (!(SYSCR0 & SYSCR0_EXBE))
      ;

    // Enable SDCLK
    SCKCR &= ~PSTOP0_BIT;

    // Protect register
    PRCR = PRCR_KEY;

    // MUST wait for more than 100usec
    wait = 686; // 100usec
    do {
    } while (--wait); // 1 loop = 7 clocks

    /* SDRAM initialization register
     * ARFI=1: initialization auto-refresh period - 4 cycles
     * ARFC=2: initialization auto-refresh counts - 2 counts
     *  PRC=0: initialization pre-charge cycles   - 3 cycles */
    SDIR = 0x0021;

    /* SDRAM initialization sequence control register (SDICR)
     * INIRQ=1:start initialization sequence */
    SDICR = 0x01;

    // Check SDRAM status
    while (SDSR)
      ;

    // Set SDRAM bus width
    SDCCR = SDC_BSIZE_16; // 16 bit, Disable SDRAM

    /* SDRAM mode select
     * A12-A10 Reserved
     * A9      WriteBurstMode - Single Bit (1)
     * A8-A7   TestMode       - Normal (00)
     * A6-A4   CAS Latency    - 2 (010)
     * A3      BurstType      - Sequential (0)
     * A2-A0   BurstLength    - 1 (000) */
    SDMOD = 0x0220;

    /* SDRAM timing register (SDTR)
     * CL=2 :SDRAMC column latency setting 2 cycles
     * WR=2 :write recovery      2 cycles 42ns(48MHz)
     * RP=1 :Row pre-charge      2 cycles 21ns(48MHz)
     * RCD=1:Row column latency  2 cycles 21ns(48MHz)
     * RAS=3:Row active interval 3 cycles 62ns(48MHz) */
    SDTR = 0x00020102;

    /* Address multiplex setting
     * Row address 13 bits, Column address 9 bits, 256M bit chip, one 16-bit
     * bus SDRAM -> shift 9 */
    SDADR = 0x01;

    /* Endian setting
     * Same endian as the operation mode
     * EMODE(b0) = 0:Endian in SDRAM address space is same as the one in the
     * operation mode. */
    SDCMOD = 0x00;

    /* Access mode setting
     *  BE=0: Disable continous access */
    SDAMOD = 0x00;

    /* Auto-refresh timing setting
     * RFC=369 cycles tREF/# of row addresses=63ns/8192=7.690us
     * 7.690/(1/48Mhz)=369 cycles(0x171)
     * REF=4 cycles */
    SDRFCR = 0x3171;

    /* Auto-refresh setting
     * RFEN=1:enable auto-refresh */
    SDRFEN = 0x01;

    // Eanble SDRAM address space
    SDCCR |= SDC_EXENB_ENA; // Enable SDRAM

#endif // defined(_SDRAM_USE_)

#endif // #if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__))

    // Enable on-chip ROM/disable external bus
    SYSCR0 = (SYSCR0_KEY | SYSCR0_ROME); // 0x5A01

// Segment copy in the case of RAM execution (DO NOT CHANGE) 
#ifdef __TEXT_RAM__
    // Copy text segment 
    p1 = (unsigned char *)&TEXT_RAM_rom;
    p2 = (unsigned char *)&TEXT_RAM_start;
    temp = (unsigned long)&TEXT_RAM_size;
    do {
      *p2++ = *p1++;
    } while (--temp);
    // Copy constant segment 
    p1 = (unsigned char *)&DATA_CONST_RAM_rom;
    p2 = (unsigned char *)&DATA_CONST_RAM_start;
    temp = (unsigned long)&DATA_CONST_RAM_size;
    do {
      *p2++ = *p1++;
    } while (--temp);
#endif // __TEXT_RAM__ 

    __CSTARTUP_INIT2__();
}

/*<<MODIFY BELOW>>
 *======================================================================
 *  Serial communication initialization
 *  This section initializes serial channels for debugging, prtinf, etc.
 *  You do not need to initialize serial channels which are specific to
 *  your application. Please initialize such serial channeles in your
 *  main function.
 *======================================================================*/
/*======================================================================
 * I/O address definition
 * Please check address definition in the case of a CPU other than RX63N
 *======================================================================*/
#define SMR1  (*((volatile unsigned char *)0x8A020)) // Serial mode register
#define BRR1  (*((volatile unsigned char *)0x8A021)) // Bit rate register
#define SCR1  (*((volatile unsigned char *)0x8A022)) // Serial control register
#define TDR1  (*((volatile unsigned char *)0x8A023)) // Transmit data register
#define SSR1  (*((volatile unsigned char *)0x8A024)) // Serial status register
#define RDR1  (*((volatile unsigned char *)0x8A025)) // Receive data register
#define SCMR1 (*((volatile unsigned char *)0x8A026)) // Smart card mode register
#define SEMR1                                                                  \
  (*((volatile unsigned char *)0x8A027)) // Serial extended mode register

#define SMR6  (*((volatile unsigned char *)0x8A0C0)) 
#define BRR6  (*((volatile unsigned char *)0x8A0C1))
#define SCR6  (*((volatile unsigned char *)0x8A0C2))
#define TDR6  (*((volatile unsigned char *)0x8A0C3))
#define SSR6  (*((volatile unsigned char *)0x8A0C4))
#define RDR6  (*((volatile unsigned char *)0x8A0C5))
#define SCMR6 (*((volatile unsigned char *)0x8A0C6))
#define SEMR6 (*((volatile unsigned char *)0x8A0C7))

// Serial status regsiter (bit symbol)
#define SSR_ORER _B1(5) // Overrun error flag
#define SSR_FER _B1(4)  // Framing error flag
#define SSR_PER _B1(3)  // Parity error flag
#define SSR_TEND _B1(2) // Trasmit end flag

// Serial control register (bit symbol)
#define CKE_BIT _B(3, 0) // Clock enable bit
#define TEIE_BIT _B1(2)  // Transmit end interrupt enable bit
#define MPIE_BIT _B1(3)  // Multiplexer interrupt enable bit
#define RE_BIT _B1(4)    // Receive enable bit
#define TE_BIT _B1(5)    // Transmit enable bit
#define RIE_BIT _B1(6)   // Receive interrupt enable bit
#define TIE_BIT _B1(7)   // Transmit interrupt enable bit

#define INT_REQ_BASE 0x87000 // Interrupt reguest register address
#define INT_REQ(a) *((volatile unsigned char *)(INT_REQ_BASE + (a)))

#define INT_IPR_BASE 0x87300 // Interrupt priroty register address
#define INT_IPR(n, a) *((volatile unsigned char *)(INT_IPR_BASE + (n))) = (a)

#define IER04 (*((volatile unsigned char *)0x87204))
#define IER1B (*((volatile unsigned char *)0x8721B))
#define IER1D (*((volatile unsigned char *)0x8721D))

/*====================================================================
 * Macros for various serial ports
 * THESE NEED TO CHANGE WHEN YOU USE A CPU OTHER THAN RX62N.
 *====================================================================*/
// RXD:P30, TXD:P26
#define SCI1_RXINT 217     // Serial receive interrupt vector #
#define SCI1_TXINT 218     // Serial transmit interrupt vector #
#define SCI1_MSTCR MSTPCRB // Module stop register
#define SCI1_MSTCR_BIT _B1(MSTPCRB_SCI1_BIT) // Module stop register bit 1
#define SCI1_IPR(a)                                                            \
  INT_IPR(0xD9, a) // Macro for interrupt level setting 0xd9=217
#define SCI1_PORT_TX (P26FS |= _B(10, 0)) // Macro to enable TXD pin
#define SCI1_PORT_RX (P30FS |= _B(10, 0)) // Macro to enable RXD pin
#define SCI1_IER_RX IER1B
#define SCI1_IER_RX_BIT _B1(1)
#define SCI1_IER_TX IER1B
#define SCI1_IER_TX_BIT _B1(2)

#define SCI1_PODR_TX P2ODR // TXD pin output data (HIGH)
#define SCI1_PDR_TX P2DR
#define SCI1_PDR_RX P3DR
#define SCI1_PMR_TX P2MR
#define SCI1_PMR_RX P3MR
#define PORT_TXD_BIT _B1(6) // P26
#define PORT_RXD_BIT _B1(0) // P30

#if (DBG_PORT == SCI1)
#define DBG_PODR_TX SCI1_PODR_TX
#define DBG_PDR_TX SCI1_PDR_TX
#define DBG_PDR_RX SCI1_PDR_RX
#define DBG_PMR_TX SCI1_PMR_TX
#define DBG_PMR_RX SCI1_PMR_RX
#define DBG_SEMR SEMR1
#define DBG_IER_TX SCI1_IER_TX
#define DBG_IER_TX_BIT SCI1_IER_TX_BIT
#endif

// RXD:P01, TXD:P00
#define SCI6_RXINT 232     // Serial receive interrupt vector #
#define SCI6_TXINT 233     // Serial transmit interrupt vector #
#define SCI6_MSTCR MSTPCRB // Module stop register
#define SCI6_MSTCR_BIT _B1(MSTPCRB_SCI6_BIT) // Module stop register bit 1
#define SCI6_IPR(a) INT_IPR(0xE8, a)         // Macro for interrupt level
#define SCI6_PODR_TX P0PODR |= _B1(0)        // TXD pint output data (HIGH)
#define SCI6_PORT_TX P00FS  |= _B(10, 0)     // Macro to enable TXD pin
#define SCI6_PORT_RX P01FS  |= _B(10, 0)     // Macro to enable RXD pin
#define SCI6_IER_RX IER1D
#define SCI6_IER_RX_BIT _B1(0)
#define SCI6_IER_TX IER1D
#define SCI6_IER_TX_BIT _B1(1)

#if (DBG_PORT == SCI6)
#define DBG_PODR_TX SCI6_PODR_TX
#define DBG_PDR_TX SCI6_PDR_TX
#define DBG_PDR_RX SCI6_PDR_RX
#define DBG_PMR_TXGEN SCI6_PMR_TXGEN
#define DBG_PMR_RXGEN SCI6_PMR_RXGEN
#define DBG_PMR_TXPER SCI6_PMR_TXPER
#define DBG_PMR_RXPER SCI6_PMR_RXPER
#define DBG_SEMR SEMR1
#endif

/*====================================================================
 * USB port definitions
 *====================================================================*/

#define USB_MSTCR MSTPCRB                   // Describe USB module stop register
#define USB_MSTCR_BIT _B1(MSTPCRB_USB0_BIT) // Module stop register bit position
#define IPR_USB(a) INT_IPR(0x23, a)
#define USB_IER IER04
#define USB_IER_BIT _B1(3)

#ifndef __YIDE_MON_MAKE__
#define USB_INT_UEVEL 15 // USB other than monitor
#else
#define USB_INT_UEVEL 14
#endif

int _YDrvUsbInit(char *pSerialNo, void *pRxIntFunc);
void _YDrvUsbSoftInit(char *pSerialNo, void *pRxIntFunc);
int _YDrvUsbStart();
int _YDrvUsbEnd();

/*====================================================================
 * Echo back setting
 * fgetc won't echo back if this line is commented out.
 *====================================================================*/
#define ECHO_BACK_ON // Enable stdin echo back

/*====================================================================
 *  Configure the number of input/output devices in C.
 *  For example, when you set the number of the input/output devices to 2
 *      and assign SCI0 and SCI1 to input/ouput devices, you can control
 *      output devices like
 *  fputc('A', stdout) -> output to SCI0
 *  fputc('B', std_sio1) -> output to SCI1
 *====================================================================*/
#define DEVICE_COUNT 2 // # of input/output devices; min 1/max 6

/*
 * IO device 0 initial setting
 *         
 * IO device 0 allocates a serial channel for downloading/
 * debugging a program as well as stdin/stdout in printf/fgetc.
 * In general, IO device 0 allocates a serial channel for ROM
 * writing.
 */

// Macro to execute a monitor program on RAM 
#ifdef __TEXT_RAM__
#pragma seg_text TEXT_RAM
#pragma seg_const DATA_CONST_RAM
#define TEXT TEXT_RAM
#endif // __TEXT_RAM__ 

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_init0
 *   
 *  Overview
 *      Initialization function; define serial communication initialization
 *   
 *  Argument
 *      None
 *   
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/

// Baud rate error check
#if (BRR_CHK(DBG_PORT_BPS, 2))
#error                                                                         \
    "Error rate exceeds 2% of the serial port for debugging. Please specify baud rate with less than 2% error rate."
#endif

void ___mm_usb_init(void);

void ___mm_sio_init0(void)
{
/* Do nothing since a monitor already initializes serial ports in
 * remote-debugging case */
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__)) //<- Required in
// ROMization and ROM debugging
#if (DBG_PORT == USB_R)

    //////////////////////////////////////
    //  Embedded USB port

    // Hard reset 
    ___mm_usb_init();

#else

    //////////////////////////////////////
    //  Serial port
  
    int wait;

    // Unlock register protect
    PRCR = PRCR_KEY | PRCR_PRC1;
    // Unlock module stop
    DBG_MSTCR &= ~DBG_MSTCR_BIT;
    // Set register protect
    PRCR = PRCR_KEY;

    // Configure RXI interrupt level; need to specify 14 in YellowScope
    DBG_IPR(14);

    // Stop transmit/receive and configure clock divisor
    DBG_SCR = 0; // Set 0 to SCR register
    do {
    } while (0x00 != (DBG_SCR & 0xf0));

    // Configure port output data TXD:HIGH
    DBG_PODR_TX |= PORT_TXD_BIT;

    // Configure port direction
    DBG_PDR_TX |= PORT_TXD_BIT;
    DBG_PDR_RX &= ~(PORT_RXD_BIT);

    /* Set a port mode to general IO port
     * Must set "0" to the PMR register when we the PmnPFS register is
     * configured. */
    DBG_PMR_TX &= ~(PORT_TXD_BIT);
    DBG_PMR_RX &= ~(PORT_RXD_BIT);

    // Configure MPC write-protect register
    PWPR &= ~B0WI_BIT;
    PWPR |= PFSWE_BIT;

    // Configure pin function controllers
    DBG_PORT_TX;
    DBG_PORT_RX;

    // Configure MPC write-protection register
    PWPR &= ~PFSWE_BIT;
    PWPR |= B0WI_BIT;

    // Set port modes to peripheral functions
    DBG_PMR_RX |= PORT_RXD_BIT;
    DBG_PMR_TX |= PORT_TXD_BIT;

    // Initialize SCI
    // CKE=00 Internal baud-rate generator
    DBG_SCR &= ~(CKE_BIT);

    /* Serial mode register
     * b7    CM    communication mode   - Asynchronous mode (0)
     * b6    CHR   character length     - 8 bits (0)
     * b5    PE    parity enable        - no-parity (0)
     * b4    PM    parity mode          - ignore (0)
     * b3    STOP  stop bits            - 1 stop bit (0)
     * b2    MP    multi-processor mode - Multi-processor commmunication
     * function is disabled (0)
     * b1:b0 CKS   clock select         - PCLK clock (n=0) (0) */
    DBG_SMR = 0; // 8 bits, no-parity, 1 stop-bit

    DBG_SCMR = 0xF2; // Serial communication I/F mode
    /* Transmit the content in TDR register
     * LSB first */
    DBG_SEMR = 0x00; // Disable RXD noise reduction
    // 16 basic clock cycle

    // Configure baud-rate by BRR register
    DBG_BRR = BRR_CAL(DBG_PORT_BPS);
    // Wait for the completion of 1 bit transmit. 1 loop = 7 clocks  
    wait = BRR_WAIT(DBG_PORT_BPS);
    do {
    } while (--wait); // 1 loop = 7 clocks

    // Reset interrupt request register
    INT_REQ(DBG_SCI_TXINT) = 0;
    INT_REQ(DBG_SCI_RXINT) = 0;

    /* Enable transmit/receive and interrupts. To disable interrupts, the
     * INTC instruction should be called. */
    DBG_SCR = 0xf0;
    do {
    } while (0xf0 != (DBG_SCR & 0xf0));
#endif
#endif // #if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__))
}

/* ----------------------------------------------------------------------------------------
 * ___mm_usb_init 
 *  
 *  Overview
 *      USB initialization function
 *        
 *  Argument
 *      None
 *    
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
/*
 * Serial number definition
 *
 *  Serial number is defined in 10-digit unicode strings.
 */

// Macro to convert an integer number to a unicode string 
#define _UD(a) ((USB_SERIAL_NO / (a)) % 10) + '0', 0

// Serial number string 
static const char USB_SerialNo[] = {22,3,
_UD(1000000000),_UD(100000000),_UD(10000000),_UD(1000000),_UD(100000),_UD(10000),_UD(1000),_UD(100),_UD(10),_UD(1)
};

extern int  __ei();

/*
 * Will not initialize USB when USB module stop is already unlocked,
 * assuming that initialization of USB is done in the earlier stage.
 */
void ___mm_usb_init(void)
{
    // Unlock register protect
    PRCR = PRCR_KEY | PRCR_PRC1;

    // Unlock USB module stop 
    USB_MSTCR &= ~USB_MSTCR_BIT;
 
    // Protect register
    PRCR = PRCR_KEY;

    // Configure MPC write-protect register
    PWPR &= ~B0WI_BIT;
    PWPR |= PFSWE_BIT;

    // Configure port modes of P14 and P16 to stdin/stdout
    P1MR &= ~(BIT_4 | BIT_6);

    // Configure pin function controllers
    P14FS = 0x11; // USB0_DPUPE
    P16FS = 0x11; // USB0_VBUS
  
    // Configure port modes of P14 and P16 to peripheral function modes.
    P1MR |= (BIT_4 | BIT_6);

    PFUSB0 |= PUPHZS_BIT;

    // Configure MPC write-protect register
    PWPR &= ~PFSWE_BIT;
    PWPR |= B0WI_BIT;

    // Initialize priority of USB interrupt vector 
    IPR_USB(USB_INT_UEVEL);

    // Enable USB interrupt 
    USB_IER |= USB_IER_BIT;

    // Initialize a USB driver 
    _YDrvUsbInit(USB_SerialNo, 0);
    _YDrvUsbStart();
    __ei();
}

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_in0
 *  
 *  Overview 
 *      1 byte input function (polling)
 *  
 *  Argument
 *    None
 *   
 *  Return
 *     input data        
 * ----------------------------------------------------------------------------------------*/
#if (DBG_PORT == USB_R)
int ___mm_sio_in0(void);
#ifndef __YIDE_MON_MAKE__
#asmb {
extern _YDrvUsbRcvBytePoll _YDrvUsbSndBytePoll _YDrvUsbSndFlushPoll 
    segment TEXT ATR_CODE 
    public ___mm_sio_in0 
___mm_sio_in0:
    PUSH R15
    MOV #0, R15 // Disable timeout
    BSR _YDrvUsbSndFlushPoll // Flush here if you call a input function.
    // Without CR in a output function, output string will be diplayed by this flush.
    MOV #0, R15 // Disable timeout
    BSR _YDrvUsbRcvBytePoll 
    RTSD #1*4,R15-R15 
    
    segment TEXT ATR_CODE 
    public ___mm_sio_in0_nf 
___mm_sio_in0_nf: 
    PUSH R15
    MOV #0, R15 // Disable timeout
    BSR _YDrvUsbRcvBytePoll 
    RTSD #1*4, R15-R15
}
#else
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_in0 
___mm_sio_in0:
    RTS
}
#endif // __YIDE_MON_MAKE__ 
#else
int ___mm_sio_in0(void)
{
    unsigned char c;

    // Wait for interrupt requests
    do {
    } while ((INT_REQ(DBG_SCI_RXINT) == 0));

    if (!(DBG_IER_RX & DBG_IER_RX_BIT)) {
    /*
     * The state of disabling interrupt request (polling receive)
     * turns off interrupt request flag.
     */
    _asm pushc psw;
    _asm clrpsw I; // Assuming that we are in the supervisor mode

    INT_REQ(DBG_SCI_RXINT) = 0;

    _asm popc psw;
    }

    c = DBG_RDR;
    return c;
}
#endif // ( DBG_PORT == USB_R ) 

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_ind0 
 *  
 *  Overview
 *      1 byte input function (without polling)
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      Input data
 * ----------------------------------------------------------------------------------------*/
/*
 * The interrupt request register is automatically cleared for edge
 * detection in serial communication.
*/
#if (DBG_PORT == USB_R)
int ___mm_sio_ind0(void);
#ifndef __YIDE_MON_MAKE__
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_ind0 
___mm_sio_ind0: 
    PUSH R15
    MOV #1, R15 // Disable timeout
    BSR _YDrvUsbRcvBytePoll 
    RTSD #1*4, R15-R15
}
#else
#asmb {
    segment TEXT ATR_CODE
    public ___mm_sio_ind0
___mm_sio_ind0: 
    RTS
}
#endif // __YIDE_MON_MAKE__ 
#else
int ___mm_sio_ind0(void)
{
    return DBG_RDR;
}
#endif // ( DBG_PORT == USB_R ) 

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_out0 
 *  
 *  Overview    
 *      1 byte output function
 *
 *  Argument
 *      Output data
 *
 *  Return
 *      Output data
 * ----------------------------------------------------------------------------------------*/

#if (DBG_PORT == USB_R)
int ___mm_sio_out0(unsigned char data);

#define INTSTS0_ADR 0xA0040
#define INTSTS0_VBSTS 7

#ifndef __YIDE_MON_MAKE__
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_out0 
___mm_sio_out0: 
    PUSHM R14-R15 
    MOV #INTSTS0_ADR,R1 
    MOV.W [R1], R1 
    BTST #INTSTS0_VBSTS,R1 // Check if USB is connected
    BEQ ___mm_sio_out0_L00
        MOV #0,R14 // Disable timeout
        BSR _YDrvUsbSndBytePoll
        CMP #'\n', R3 // Execute 'flush' with "\n" in trasmit data.
        BNE ___mm_sio_out0_L00
        MOV #0, R15 // Disable timeout
        BSR _YDrvUsbSndFlushPoll 
___mm_sio_out0_L00:
    MOVU.B R3, R3 
    RTSD #2*4, R14-R15
}
#else
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_out0 
___mm_sio_out0: 
    RTS
}
#endif // __YIDE_MON_MAKE__ 
#else
int ___mm_sio_out0(unsigned char data)
{
    do {
    } while ((INT_REQ(DBG_SCI_TXINT) == 0));

    // Write data to the TDR register
    DBG_TDR = data;
    return data;
}
#endif // ( DBG_PORT == USB_R ) 

/* ----------------------------------------------------------------------------------------
 * Flush function
 * ----------------------------------------------------------------------------------------*/
// Wait for the completion of data trasmit.
#if (DBG_PORT == USB_R)
void ___mm_sio_flush0(void);
#ifndef __YIDE_MON_MAKE__
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_flush0 
___mm_sio_flush0: 
    PUSH R15 
    MOV #0, R15 // disable timeout
    BSR _YDrvUsbSndFlushPoll 
    RTSD #1 * 4, R15 - R15
}
#else
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_flush0 
___mm_sio_flush0: 
    RTS
}
#endif // __YIDE_MON_MAKE__ 
#else
void ___mm_sio_flush0(void)
{
    do {
    } while ((DBG_SSR & SSR_TEND) == 0);
}
#endif // ( DBG_PORT == USB_R ) 

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_ei0
 *  
 *  Overview
 *      Enable receive interrupt
 *
 *  Argument
 *      Flag (1 to clear the interrupt flag) 
 *
 *  Return
 *      None   
 * ----------------------------------------------------------------------------------------*/
#if (DBG_PORT == USB_R)
void ___mm_sio_ei0(int flag);
#ifndef __YIDE_MON_MAKE__
#asmb {
    extern _YDrvUsbEnaRcvInt 
    segment TEXT ATR_CODE 
    public ___mm_sio_ei0 
___mm_sio_ei0:
    BRA _YDrvUsbEnaRcvInt
}
#else
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_ei0 
___mm_sio_ei0: 
    RTS
}
#endif // __YIDE_MON_MAKE__ 
#else
void ___mm_sio_ei0(int flag)
{
    if (flag)
      INT_REQ(DBG_SCI_RXINT) = 0;
    DBG_IER_RX |= DBG_IER_RX_BIT;
}
#endif // ( DBG_PORT == USB_R ) 

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_di0 
 *  
 *  Overview
 *      Disable receive interrupt
 *
 *  Argument
 *      None   
 *
 *  Return
 *      None   
 * ----------------------------------------------------------------------------------------*/
#if (DBG_PORT == USB_R)
void ___mm_sio_di0(void);
#ifndef __YIDE_MON_MAKE__
#asmb {
    extern _YDrvUsbDisRcvInt 
    segment TEXT ATR_CODE 
    public ___mm_sio_di0 
___mm_sio_di0:
    BRA _YDrvUsbDisRcvInt
}
#else
#asmb {
    segment TEXT ATR_CODE 
    public ___mm_sio_di0 
___mm_sio_di0: 
    RTS
}
#endif // __YIDE_MON_MAKE__ 
#else
void ___mm_sio_di0(void)
{ 
    DBG_IER_RX &= ~DBG_IER_RX_BIT;
}
#endif // ( DBG_PORT == USB_R ) 

/* ------ In the normal configuraion, users only need to modify the above  -------
 * ----------- If you have 2 or more IO devices, please define below ------------
 *
 * IO device 1 settings
 *         std_sio1 is the file descriptor for IO device 1.
 *         In this example, we assign SCI2 to IO device 1's I/O ports.
 *         Thus, the following functions output or input data to/from SCI2.
 *         
 *         printf(std_sio1, ....);
 *         fgetc(std_sio1);
 *         
 *         You do not need to configure the following settings
 *         if you only use stdin, stdout, and stderr.
*/
/* ----------------------------------------------------------------------------------------
 * ___mm_sio_init1 
 *  
 *  Overview
 *      Serial communication initialization function
 *
 *  Argument
 *      None
 *
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/

// Baud rate of SC16
#define SCI6_BPS 38400L

// Baud rate error check
#if (BRR_CHK(SCI6_BPS, 2))
#error                                                                         \
    "Error rate in serial port 2 exceeds 2% of the serial port for debugging. Please specify baud rate with less than 2% error rate."
#endif

void ___mm_sio_init1(void)
{
     int wait;

    // Unlock module stop
    SCI6_MSTCR &= ~SCI6_MSTCR_BIT;
    // Configure interrupt level. We will use 14 in YellowScope.
    SCI6_IPR(14);
    /* Configure transmit-receive stop; Stop transmit/receive and configure
     * clock divisor. */
    SCR6 = 0; // Set 0 to the SMR register
    // Configure pin function controller
    SCI6_PORT_TX;
    SCI6_PORT_RX;
    SMR6 = 0; // 8bit,parity-non,stop 1
    // Configure baud rate with the BRR register
    BRR6 = BRR_CAL(SCI6_BPS);
    // Wait for the completion of 1-bit transmit. 1 loop = 7 clocks 
    wait = BRR_WAIT(SCI6_BPS);
    do {
    } while (--wait); // 7 cloks per loop
    /* Enable transmit/receive and interrupts. To disable interrupts, the
     * INTC instruction should be called. */
    SCR6 = 0xf0;
}

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_in1 
 *  
 *  Overview
 *      1 byte input function (polling)
 *
 *  Arguments
 *      None
 *
 *  Return
 *      Input data
 * ----------------------------------------------------------------------------------------*/
int ___mm_sio_in1(void)
{
    return RDR6;
}

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_ind1 
 * 
 *  Overview
 *      1 byte input function (no polling)
 *
 *  Argument
 *      None
 *
 *  Return
 *      Input data
 * ----------------------------------------------------------------------------------------*/
int ___mm_sio_ind1(void)
{
    return RDR6;
}

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_out1
 *  
 *  Overview
 *      1 byte output function
 *
 *  Argument
 *      Output data
 *
 *  Return
 *      Output data
 * ----------------------------------------------------------------------------------------*/
int ___mm_sio_out1(int data)
{
    TDR6 = data;
    return data;
}

/* ----------------------------------------------------------------------------------------
 * Flush function
 * ----------------------------------------------------------------------------------------*/
// Wait for the completion of data transmit.
void ___mm_sio_flush1(void)
{
    do {
    } while ((SSR6 & SSR_TEND) == 0);
}

/* ----------------------------------------------------------------------------------------
 *
 *  Enable receive interrupts
 *
 *  Argument
 *      flag (1 to clear interrupt vector flag)
 *
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void ___mm_sio_ei1(int flag)
{
    if (flag) {
      INT_REQ(SCI6_RXINT) = 0;
    }
    SCI6_IER_RX |= SCI6_IER_RX_BIT;
}

/* ----------------------------------------------------------------------------------------
 * ___mm_sio_di1 
 *  Overview
 *      Disable receive interrupts
 *
 *  Argument
 *      None
 *
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void ___mm_sio_di1(void)
{ 
    SCI6_IER_RX &= ~SCI6_IER_RX_BIT; 
}

/* ----------------------------------------------------------------------------------------
 * DTC configuration
 *    Please refer to Help for more information about DTC
 * ----------------------------------------------------------------------------------------*/
#pragma dtc_info 4, 256, 0x000, 0x0, 0x17fff // DTC information. Please refer
// to help for more information
#define DTCVBR 0x82404 // DTCVBR register address

/* ----------------------------------------------------------------------------------------
 * ___mm_set_dtcvbr 
 *  Overview
 *      DTCVBR configuration
 *
 *  Argument
 *      DTCVBR register address
 *
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void ___mm_set_dtcvbr(unsigned long *vbr)
{
    *((volatile unsigned long *)DTCVBR) = (unsigned long)vbr;
}

/* ----------  Users can modify the above code. ----------------------
 * ----------  The following code should be kept as it is  -------------- */
#if defined(__YIDE_REM_DEBUG__) || defined(__YIDE_MON_MAKE__) ||               \
    defined(__YIDE_LOAD_MAKE__)
#asmb {
INCLUDE(YIDESYM.DEF)
__MAX_VECTNO__ MAX_VECTNO + 32 /*  The number of interrupt vectors (fixed
                                *  interrupt vectors should be placed here)
                                */
}
#else
#asmb {
INCLUDE(YIDESYM.DEF)
__MAX_VECTNO__ MAX_VECTNO /*  The number of interrupt vectors
                             */
}
#endif

// Must include the USB library here
#deflib                                                                        \
    "USB\YDRVUSB_RX_01.OBJ" // Specify a library for embedded USB of Renesas chips 

#if (DBG_PORT == USB_R)
#if !(defined(__YIDE_MON_MAKE__) || defined(__YIDE_ROM_DEBUG__))
#deflib "USB\YDRVUSB_RX_01.OBJ" // Specify a library for embedded USB for Renesas chips 
#endif
#endif

#ifdef __YIDE_ROM_DEBUG__
#error "Cannot perform ROM debugging"
#endif

#ifdef __YIDE_MON_MAKE__
#ifndef __TEXT_RAM__
#if (DBG_PORT == USB_R)
#deflib                                                                        \
    "REM-MON\RX600_02\REMRX.LIB" // Specify a monitor library for embedded USB remote debugging 
#else
#deflib                                                                        \
    "REM-MON\RX600_00\REMRX.LIB" // Specify a monitor library for remote debugging 
#endif
#else
#if (DBG_PORT == USB_R)
#error "No found: USB monitor running on on-chip SRAM"
#else
#deflib                                                                        \
    "REM-MON\RX600_80\REMRX.LIB" // Specify a libray monitor for remote debugging 
#endif
#endif // __TEXT_RAM__ 
#endif // __YIDE_MON_MAKE__  

#ifdef __YIDE_LOAD_MAKE__
#if (DBG_PORT == USB_R)
#deflib "LOADER\RX600_01\LOADRX.LIB" // Specify a monitor library for the embedded USB loader 
#else
#deflib "LOADER\RX600_00\LOADRX.LIB" // Specify a monitor library for the loader 
#endif
#endif // __YIDE_LOAD_MAKE__  

#define NULL ((void *)0)

#ifdef __INT_RAM__
#undef RAM_BASE
#define RAM_BASE 0x0000000 // Specify your on-chip SRAM
// starting address here. The 12
// LSB of the address should be
// always "0".
#endif

#ifdef __SUB_PROJECT__
#define _USTACK_TOP _SYSTEM_USTACK_TOP
#define _USTACK_BTM _SYSTEM_USTACK_BTM
#define _STACK_TOP _SYSTEM_STACK_TOP
#define _STACK_BTM _SYSTEM_STACK_BTM
extern long _SYSTEM_USTACK_TOP[];
extern long _SYSTEM_STACK_TOP[];
#asmb {
DEFINE vector_table_top system_vector_table_top
}
#endif

extern void ___mm_monitor_stack;
extern void ___mm_monitor_istack;

#if defined(__YIDE_MON_MAKE__) || defined(__YIDE_LOAD_MAKE__)
#rambase RAM_BASE
#endif

__vect_table__ {

/* --------------------------------------------------------------------
 * Interrupt Vector Table
 * DO NOT CHANGE. YellowIDE will configure it properly
 * -------------------------------------------------------------------- */
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__) ||                 \
     (defined __YIDE_SIM_DEBUG__))
#endif

#ifdef __YIDE_ROM_DEBUG__ // ROM debugging case
#if (DBG_PORT != USB_R)
  DBG_SCI_RXINT, ___mm_sci_int // SCI receive interrupt; DO NOT CHANGE
#endif
#endif
#if (DBG_PORT != USB_R)
#ifdef __YIDE_MON_MAKE__ // Monitor case
      DBG_SCI_RXINT,
      ___mm_sci_int // SCI receive interrupt; DO NOT CHANGE
#endif
#endif
}

#ifdef __YIDE_REM_DEBUG__ // remote debugging case
#if (DBG_PORT == USB_R)
#asmb {
    __EXC_VECTNO__ 13 // For undefined fixed interrupt vector detection
    __EXC_VECTNO__ 14 // For undefined interrupt vector detection
    __EXC_VECTNO__ 15 // For system calls
    __DBG_VECTTBL__
}
#else
#asmb {
    __EXC_VECTNO__ 13 // For undefined fixed interrupt vector detection
    __EXC_VECTNO__ 14 // For undefined interrupt vector detection
    __EXC_VECTNO__ 15 // For system calls
    __EXC_VECTNO__ DBG_SCI_RXINT 
    __DBG_VECTTBL__
}
#endif
#endif // __YIDE_REM_DEBUG__ 

#ifdef __YIDE_RAM__ // Downloader
#asmb {
__DBG_VECTTBL__
}
#endif

//-----------------End of Vector Table-------------------------------

static void (*dp)(void);
extern int _main(int, int);
extern void __exit(void);
extern void __exit2(int);
extern void monitor(void);
extern unsigned char DATA_rom, DATA_start;
extern unsigned long DATA_size;
extern unsigned int __mm_dtc_vect_addr;
extern unsigned char __Heapbase[];
extern void _WriteSP(long *);
extern void __init_intb__(void);
extern unsigned char * const __smr_adr[];
extern void ___mm_putc_yscope(unsigned char);
extern int ___mm_getc_yscope(void);

void __CSTARTUP_INIT2__(void)
{
    /* ----------  DO NOT NEED TO CHANGE BELOW  --------------
     *=====================================================================
     *  Initialize serial ports
     *=====================================================================*/
    for (temp = 1; temp < 8; temp++) {
        dp = (void (*)(void))__smr_adr[temp];
        if (dp != NULL) {
            dp();
        }
    }
    dp = (void (*)(void))__smr_adr[0];
    if (dp != NULL)
        dp();
    /* ---------------------------------------------------------------------
     *  Configure INTB register (base address of interrupt vector)
     * --------------------------------------------------------------------- */

#ifndef __SUB_PROJECT__
#ifndef __YIDE_MON_MAKE__
    __init_intb__();
#endif // __YIDE_MON_MAKE__   
#endif // __SUB_PROJECT__    

/* ---------------------------------------------------------------------
 *  Copy data segment
 * --------------------------------------------------------------------- */
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__) ||                 \
     (defined __YIDE_SIM_DEBUG__) || (defined __YIDE_REM_DEBUG__))
    p1 = (unsigned char *)&DATA_rom;
    p2 = (unsigned char *)&DATA_start;
    for (temp = 0; temp < (unsigned long)&DATA_size; temp++) {
        *p2++ = *p1++;
    }
#endif

/* --------------------------------------------------------------------
 *  Initialize heap
 * -------------------------------------------------------------------- */
#if HEAP_SIZE
    for (temp = 0; temp < HEAP_SIZE + 8; temp++) {
        __Heapbase[temp++] = 0;
    }
#endif

    /* ---------------------------------------------------------------------
     *  Configure DTCVBR register (base address of DTC vector)
     * --------------------------------------------------------------------- */
    ___mm_set_dtcvbr((unsigned long *)&__mm_dtc_vect_addr);

/* ---------------------------------------------------------------------
 *  Call minitor
 * --------------------------------------------------------------------- */
#ifdef __YIDE_ROM_DEBUG__
    monitor();
#endif

    /* ---------------------------------------------------------------------
     *  Call main function
     * --------------------------------------------------------------------- */
    temp = _main(0, 0);

    /* --------------------------------------------------------------------
     *  post process of main function
     *  raise function requires __exit label
     * -------------------------------------------------------------------- */
    __exit2(temp);
}

/*====================================================================
 * TEXT segment (functions)
 * Exit function
 *====================================================================*/
void __exit(void)
{
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
    _asm MOV #0, R15 
    _asm INT #15 
    _asm DC.B 0xC8
#else
    while (1) {
        ;
    }
#endif
}

void __exit2(int ret)
{
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
    _asm INT #15 
    _asm DC.B 0xC8
#else
    while (1) {
        ;
    }
#endif
}

/*====================================================================
 * TEXT segment (functions)
 * Define proceduer in stack overflows here
 *====================================================================*/
void _stack_over_task(void)
{
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
    _asm MOV SP, R1 // R1 = ErrorFuncSP
    _asm INT #15    // R2 = ErrorFuncPtr
    _asm DC.B 0xCC
#else
    extern void _puts(const char *);
    extern void _TransitSuperMode(void);
    //------------------------------------------------------------------------
    _TransitSuperMode();
    _asm MVTC #_STACK_TOP + 4, ISP // Interrupt stack
    _asm MVTC #_USTACK_TOP + 4,USP // User stack
    _puts("stack overflow");
    //------------------------------------------------------------------------
    __exit(); // Eixt

// NOTE: THIS FUCNTION CANNOT RETURN
#endif
}

/* --------------------------------------------------------------------
 *  Device drivers
 * -------------------------------------------------------------------- */
#if (DEVICE_COUNT < 2)
#define DEV_INIT1 NULL
#define DEV_IN1 NULL
#define DEV_IND1 NULL
#define DEV_OUT1 NULL
#define DEV_FLS1 NULL
#define DEV_EI1 NULL
#define DEV_DI1 NULL
#else
#define DEV_INIT1 ___mm_sio_init1
#define DEV_IN1   ___mm_sio_in1
#define DEV_IND1  ___mm_sio_ind1
#define DEV_OUT1  ___mm_sio_out1
#define DEV_FLS1  ___mm_sio_flush1
#define DEV_EI1   ___mm_sio_ei1
#define DEV_DI1   ___mm_sio_di1
#endif
#if (DEVICE_COUNT < 3)
#define DEV_INIT2 NULL
#define DEV_IN2   NULL
#define DEV_IND2  NULL
#define DEV_OUT2  NULL
#define DEV_FLS2  NULL
#define DEV_EI2   NULL
#define DEV_DI2   NULL
#else
#define DEV_INIT2 ___mm_sio_init2
#define DEV_IN2   ___mm_sio_in2
#define DEV_IND2  ___mm_sio_ind2
#define DEV_OUT2  ___mm_sio_out2
#define DEV_FLS2  ___mm_sio_flush2
#define DEV_EI2   ___mm_sio_ei2
#define DEV_DI2   ___mm_sio_di2
#endif
#if (DEVICE_COUNT < 4)
#define DEV_INIT3 NULL
#define DEV_IN3   NULL
#define DEV_IND3  NULL
#define DEV_OUT3  NULL
#define DEV_FLS3  NULL
#define DEV_EI3   NULL
#define DEV_DI3   NULL
#else
#define DEV_INIT3 ___mm_sio_init3
#define DEV_IN3   ___mm_sio_in3
#define DEV_IND3  ___mm_sio_ind3
#define DEV_OUT3  ___mm_sio_out3
#define DEV_FLS3  ___mm_sio_flush3
#define DEV_EI3   ___mm_sio_ei3
#define DEV_DI3   ___mm_sio_di3
#endif
#if (DEVICE_COUNT < 5)
#define DEV_INIT4 NULL
#define DEV_IN4   NULL
#define DEV_IND4  NULL
#define DEV_OUT4  NULL
#define DEV_FLS4  NULL
#define DEV_EI4   NULL
#define DEV_DI4   NULL
#else
#define DEV_INIT4 ___mm_sio_init4
#define DEV_IN4   ___mm_sio_in4
#define DEV_IND4  ___mm_sio_ind4
#define DEV_OUT4  ___mm_sio_out4
#define DEV_FLS4  ___mm_sio_flush4
#define DEV_EI4   ___mm_sio_ei4
#define DEV_DI4   ___mm_sio_di4
#endif
#if (DEVICE_COUNT < 6)
#define DEV_INIT5 NULL
#define DEV_IN5   NULL
#define DEV_IND5  NULL
#define DEV_OUT5  NULL
#define DEV_FLS5  NULL
#define DEV_EI5   NULL
#define DEV_DI5   NULL
#else
#define DEV_INIT5 ___mm_sio_init5
#define DEV_IN5   ___mm_sio_in5
#define DEV_IN5   ___mm_sio_ind5
#define DEV_OUT5  ___mm_sio_out5
#define DEV_FLS5  ___mm_sio_flush5
#define DEV_EI5   ___mm_sio_ei5
#define DEV_DI5   ___mm_sio_di5
#endif

/* --------------------------------------------------------------------
 * Assign input/output devices for standard input/output functions
 * -------------------------------------------------------------------- */
unsigned char *const __smr_adr[] = {
    // Initialization
    NULL,      // stdin
    NULL,      // stdout
    NULL,      // stderr
    DEV_INIT1, // std_sio1
    DEV_INIT2, // std_sio2
    DEV_INIT3, // std_sio3
    DEV_INIT4, // std_sio4
    DEV_INIT5, // std_sio5
// Input
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
    ___mm_getc_yscope, // stdin
#else
    NULL,     // stdin
#endif
    NULL,     // stdout
    NULL,     // stderr
    DEV_IN1,  // std_sio1
    DEV_IN2,  // std_sio2
    DEV_IN3,  // std_sio3
    DEV_IN4,  // std_sio4
    DEV_IN5,  // std_sio5
    // Input w/o polling
    NULL,     // stdin
    NULL,     // stdout
    NULL,     // stderr
    DEV_IND1, // std_sio1
    DEV_IND2, // std_sio2
    DEV_IND3, // std_sio3
    DEV_IND4, // std_sio4
    DEV_IND5, // std_sio5
    // Output
    NULL, // stdin
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
    ___mm_putc_yscope, // stdout
    ___mm_putc_yscope, // stderr
#else
    NULL,      // stderr
    NULL,      // stderr
#endif
    DEV_OUT1, // std_sio1
    DEV_OUT2, // std_sio2
    DEV_OUT3, // std_sio3
    DEV_OUT4, // std_sio4
    DEV_OUT5, // std_sio5
    // Flush
    ___mm_sio_flush0, // stdin
    ___mm_sio_flush0, // stdout
    ___mm_sio_flush0, // stderr
    DEV_FLS1,         // std_sio1
    DEV_FLS2,         // std_sio2
    DEV_FLS3,         // std_sio3
    DEV_FLS4,         // std_sio4
    DEV_FLS5,         // std_sio5
    // Enable receive interrupt
    ___mm_sio_ei0, // stdin
    ___mm_sio_ei0, // stdout
    ___mm_sio_ei0, // stderr
    DEV_EI1,       // std_sio1
    DEV_EI2,       // std_sio2
    DEV_EI3,       // std_sio3
    DEV_EI4,       // std_sio4
    DEV_EI5,       // std_sio5
    // Disable receive interrupt
    ___mm_sio_di0, // stdin
    ___mm_sio_di0, // stdout
    ___mm_sio_di0, // stderr
    DEV_DI1,       // std_sio1
    DEV_DI2,       // std_sio2
    DEV_DI3,       // std_sio3
    DEV_DI4,       // std_sio4
    DEV_DI5,       // std_sio5
    // Reserved 1
    NULL, // stdin
    NULL, // stdout
    NULL, // stderr
    NULL, // std_sio1
    NULL, // std_sio2
    NULL, // std_sio3
    NULL, // std_sio4
    NULL, // std_sio5
    // Reserved 2
    NULL, // stdin
    NULL, // stdout
    NULL, // stderr
    NULL, // std_sio1
    NULL, // std_sio2
    NULL, // std_sio3
    NULL, // std_sio4
    NULL, // std_sio5
};

/* --------------------------------------------------------------------
 *  DATA_CONST segment (data with 'const')
 * -------------------------------------------------------------------- */
#if HEAP_SIZE
const unsigned short __HeapSize = HEAP_SIZE;
#endif
#ifdef ECHO_BACK_ON
const char __dbg_flag = 0x80;
#else
const char __dbg_flag = 0xC0;
#endif

/* --------------------------------------------------------------------
 *    BSS segment (data without 'const')
 * -------------------------------------------------------------------- */
#if RET_STRUCT_SIZE
unsigned char __struct_ret[RET_STRUCT_SIZE];
#endif

/* --------------------------------------------------------------------
 * HEAP segment
 * You can delete it if you never use heap management functions like malloc.
 * -------------------------------------------------------------------- */
#if HEAP_SIZE
#pragma seg_bss HEAP
unsigned char __Heapbase[HEAP_SIZE + 8]; // 8 = sizeof(HEADER)
#pragma seg_bss_end
#endif

/* --------------------------------------------------------------------
 * Stack segment
 * (DO NOT MODIFY)
 * -------------------------------------------------------------------- */
#pragma seg_bss STACK          // For regular stack
static char ___stack_free[64]; // Extra buffer
char _STACK_BTM[STACK_SIZE - 4];
long _STACK_TOP[1];
#pragma seg_bss_end

#pragma seg_bss USTACK          // For regular user stack
static char ___ustack_free[64]; // Extra buffer
char _USTACK_BTM[USTACK_SIZE - 4];
long _USTACK_TOP[1];
#pragma seg_bss_end

/* ----------------------------------------------------------------------------------------
 * Fixed Interrupt Vector Exception Handler 
* ----------------------------------------------------------------------------------------*/
#if defined(__YIDE_MON_MAKE__) || defined(__YIDE_LOAD_MAKE__)
#asmb {
MACRO EXC_EXE(no)
    /*
     * Fixed exception handlers are configured to jump to 32 vectors from
     * interrupt vector number 256 to 287.
     * This is the macro to jump to those vectors
     */
    SUB #4, SP    // Allocate a trap address for RTS
    PUSH R7       // Save R7 (address calculation register) to the stack
    MVFC INTB, R7 // Retreive start address of variable interrupt vector
    /* Calculate the address where (the specified vector number + 256) * 4
     * Store calculated interrupt vector address to RTS trap address in the
     * stack. */
    MOV.L((no)+256)*4[R7],4[SP] 
    RTSD #4, R7-R7 // Jump by the RTSC instruction
ENDM
}
#else
#asmb {
MACRO EXC_EXE(no) LOCAL(L0) 
L0: BRA L0 
ENDM
}
#endif

#asmb {
    segment TEXT 
___mm_fi_nmi_def: 
    EXC_EXE(30) 
    
    segment TEXT 
___mm_fi_fpuexc_def: 
    EXC_EXE(25) 
    
    segment TEXT 
___mm_fi_undefinst_def: 
    EXC_EXE(23) 
    
    segment TEXT 
___mm_fi_accexc_def: 
    EXC_EXE(21) 
    
    segment TEXT 
___mm_fi_superexc_def: 
    EXC_EXE(20) 
    
    segment TEXT 
___mm_fi_undefexc_def: 
    EXC_EXE(8)
}

/*************************************
 **                                 **
 **    Fixed Interrupt Vector       **
 **                                 **
 *************************************
 *
 * Default Definitions
 */

#asmb {
extern ___mm_fi_undefexc ___mm_fi_superexc ___mm_fi_accexc ___mm_fi_undefinst ___mm_fi_fpuexc ___mm_fi_nmi
}

#ifdef __YIDE_ROM__
#asmb {
externdef ___mm_fi_undefexc  ___mm_fi_undefexc_def 
externdef ___mm_fi_superexc  ___mm_fi_superexc_def 
externdef ___mm_fi_accexc    ___mm_fi_accexc_def 
externdef ___mm_fi_undefinst ___mm_fi_undefinst_def 
externdef ___mm_fi_fpuexc    ___mm_fi_fpuexc_def 
externdef ___mm_fi_nmi       ___mm_fi_nmi_def
}
#else
#asmb {
extern ___mm_fi_undefexc_yscope8 ___mm_fi_undefexc_yscope20 ___mm_fi_undefexc_yscope21 ___mm_fi_undefexc_yscope23 ___mm_fi_undefexc_yscope25 ___mm_fi_undefexc_yscope30 

externdef ___mm_fi_undefexc  ___mm_fi_undefexc_yscope8  ___mm_fi_undefexc_def 
externdef ___mm_fi_superexc  ___mm_fi_undefexc_yscope20 ___mm_fi_superexc_def 
externdef ___mm_fi_accexc    ___mm_fi_undefexc_yscope21 ___mm_fi_accexc_def 
externdef ___mm_fi_undefinst ___mm_fi_undefexc_yscope23 ___mm_fi_undefinst_def 
externdef ___mm_fi_fpuexc    ___mm_fi_undefexc_yscope25 ___mm_fi_fpuexc_def 
externdef ___mm_fi_nmi       ___mm_fi_undefexc_yscope30 ___mm_fi_nmi_def
}
#endif

#asmb {
segment FIX_INTV            // DO NOT ADD 'ATR_VECT'
    DC.L MDES               // Endian select register S
    DC.L ___mm_fi_undefexc  // 1
    DC.L OFS1               // Option function select register 1
    DC.L OFS0               // Option function select register 0
    DC.L ___mm_fi_undefexc  // 4
    DC.L ___mm_fi_undefexc  // 5
    DC.L ___mm_fi_undefexc  // 6
    DC.L ___mm_fi_undefexc  // 7
    DC.L ___mm_fi_undefexc  // 8
    DC.L ___mm_fi_undefexc  // 9
    DC.L ___mm_fi_undefexc  // 10
    DC.L ___mm_fi_undefexc  // 11
    DC.L ___mm_fi_undefexc  // 12
    DC.L ___mm_fi_undefexc  // 13
    DC.L ___mm_fi_undefexc  // 14
    DC.L ___mm_fi_undefexc  // 15
    DC.L ___mm_fi_undefexc  // 16
    DC.L ___mm_fi_undefexc  // 17
    DC.L ___mm_fi_undefexc  // 18
    DC.L ___mm_fi_undefexc  // 19
    DC.L ___mm_fi_superexc  // 20  Priviledged instruction exceptions
    DC.L ___mm_fi_accexc    // 21  Access exceptions (not supported on RX610)
    DC.L ___mm_fi_undefexc  // 22
    DC.L ___mm_fi_undefinst // 23  Undefined instruction exceptions
    DC.L ___mm_fi_undefexc  // 24
    DC.L ___mm_fi_fpuexc    // 25  Floating point exceptions
    DC.L ___mm_fi_undefexc  // 26
    DC.L ___mm_fi_undefexc  // 27
    DC.L ___mm_fi_undefexc  // 28
    DC.L ___mm_fi_undefexc  // 29
    DC.L ___mm_fi_nmi       // 30  NMI
    DC.L START              // 31  Reset
}

//----------  YellowScope Related Routines  ----------
/* --------------------------------------------------------------------
 *    Debug Port Identifier
 * -------------------------------------------------------------------- */
const unsigned long ___mm_RM_INTB = RAM_BASE;
const unsigned char ___mm_RMDebugPort = DBG_PORT | 0x80;
#if (DBG_PORT != USB_R)
const unsigned char ___mm_RM_PortIntOff = DBG_SCI_RXINT;
#endif

#ifdef __LITTLE_ENDIAN__
#asmb {
__CPUOPT__ 0, 2 // RX600 Family CPU Setting Instruction, Little Endian
}
#else
#asmb {
__CPUOPT__ 0, 3 // RX600 Family CPU Setting Instruction, Big Endian
}
#endif

#ifdef __TEXT_RAM__
#pragma seg_text_end
#pragma seg_const_end
#undef TEXT
#endif // __TEXT_RAM__ 

#ifndef __CPU_RX600__
#error "CPU type is not RX600"
#endif

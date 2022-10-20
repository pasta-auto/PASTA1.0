/*******************************************************************************
*  Copyright (C) 2011 Renesas Electronics Corporation. All rights reserved.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in
*  all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
******************************************************************************/
/******************************************************************************
* File Name    : r_flash_api_rx600_private.h
* Version      : 2.20
* Device       : RX600 Series
* Tool-Chain   : RX Family C Compiler
* H/W Platform : RSKRX62N, RSKRX610, YRDKRX62N, RSKRX630, RSKRX63N, RSKRX62T,
*                YRDKRX63N
* Description  : Flash programming for the RX600 Group
*******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.12.2009 1.00    First Release
*         : 13.01.2010 1.10    Made function names and variables RAPI compliant
*         : 11.02.2010 1.20    Fixed other RAPI issues and fixed I flag issue
*         : 29.04.2010 1.30    Added support for RX621/N Group. Moved most
*                              device specific data to header file.
*         : 26.05.2010 1.40    Added support for RX62T Group
*         : 28.07.2010 1.41    Fixed bug when performing a blank check on an
*                              entire data flash block.  Also declared
*                              functions not in the API as 'static'.
*         : 23.08.2010 1.42    Updated source to raise the coding standard, to
*                              meet GSCE & RSK standards.
*         : 15.02.2011 1.43    Fixed bug in blank check routine when handling
*                              input arguments and moved _Flash_Init() to
*                              _Enter_PE_Mode() function.
*         : 21.04.2011 2.00    Added BGO capabilities for data flash. Made
*                              some more changes to fit coding standard. Added
*                              ability to do ROM to ROM or DF to DF transfers.
*                              Added the ability to use the lock bit feature
*                              on ROM blocks.  Added BGO capabilities for
*                              ROM operations.
*         : 06.07.2011 2.10    Added support for RX630, RX631, and RX63N.
*                              Also added R_FlashEraseRange() for devices like
*                              RX63x that have finer granularity data flash.
*                              Various bug fixes as well. Example bug fix was
*                              removing DATA_FLASH_OPERATION_PIPL and
*                              ROM_OPERATION_PIPL #defines since the IPL was
*                              not restored when leaving flash ready interrupt.
*         : 29.11.2011 2.20    Renamed private functions according to new
*                              Coding Standard. Removed unused 'bytes' argument
*                              from enter_pe_mode() function. Removed 'far'
*                              keyword since it is not needed. Fixed where some
*                              functions were being placed in RAM when this was
*                              not needed. Uses platform.h now instead of
*                              having multiple iodefine_rxXXX.h's. Added
*                              __evenaccess directive to FCU accesses. This
*                              ensures proper bus width accesses. Added
*                              R_FlashCodeCopy() function. When clearing the
*                              FENTRYR register, the register is read back to
*                              ensure its value is 0x0000. Added call to
*                              exit_pe_mode() when enter_pe_mode() function
*                              fails to protect against reading ROM in P/E
*                              mode. Added option to use r_bsp package.
*****************************************************************************/

#ifndef _FLASH_API_RX600_PRIVATE_H
#define _FLASH_API_RX600_PRIVATE_H

// This catches to make sure the user specified a CPU clock 
#if !defined(ICLK_HZ)
#error "ERROR !!! You must specify the System Clock Frequency (ICLK_HZ) !";
#endif

// This checks to make sure an IPL for the flash ready interrupt was defined 
#if (defined(ROM_BGO) || defined(DATA_FLASH_BGO)) && !defined(FLASH_READY_IPL)
#error "ERROR !!! You must specify an IPL for the flash ready interrupt" \
    in r_flash_api_rx600_config.h when using BGO."
#endif

/* Define the clock frequency supplied to the FCU. On the RX610 and Rx62x
 * this is the PCLK. On the RX63x it is the FCLK. */
#if defined(MCU_RX610) || defined(MCU_RX62T) || defined(MCU_RX62N)
#define FLASH_CLOCK_HZ PCLK_HZ
#elif  defined(MCU_RX630) || defined(MCU_RX63N)
#define FLASH_CLOCK_HZ FCLK_HZ
#endif

/******************************************************************************
*  MCU Specific Items
*  Configuration setttings set in r_flash_api_rx600_config.h
*****************************************************************************/
// This flag is used for setting/clearing the I bit in the PSW 
#define I_FLAG  (0x00010000)

/* Below is memory information that is specific to device families (RX610,
 * RX62N, etc...).  While memory sizes will differ between family members,
 * the memory layout should remain the same */

/* Do not edit below this line unless you are editting for another RX with
 * a different memory layout. */

/*  Bottom of User Flash Area
 *#define ROM_PE_ADDR     ((0x100000000-ROM_SIZE_BYTES)&(0x00FFFFFF))*/
#define ROM_PE_ADDR     0xF00000

#if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) || defined(MCU_RX62T)
/*  According to HW Manual the Max Programming Time for 256 bytes (ROM)
 *  is 12ms. This is with a PCLK of 50MHz. The calculation below
 *  calculates the number of ICLK ticks needed for the timeout delay.
 *  The 12ms number is adjusted linearly depending on the PCLK frequency.
 */
#define WAIT_MAX_ROM_WRITE \
    ((int32_t)(12000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

/*  According to HW Manual the Max Programming Time for 128 bytes
 *  (Data Flash) is 5ms. This is with a PCLK of 50MHz. The calculation
 *  below calculates the number of ICLK ticks needed for the timeout delay.
 *  The 5ms number is adjusted linearly depending on the PCLK frequency.
 */
#define WAIT_MAX_DF_WRITE \
    ((int32_t)(5000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

/*  According to HW Manual the Max Blank Check time for 2k bytes
 *  (Data Flash) is 0.7ms. This is with a PCLK of 50MHz. The calculation
 *  below calculates the number of ICLK ticks needed for the timeout delay.
 *  The 0.7ms number is adjusted linearly depending on the PCLK frequency.
 */
#define WAIT_MAX_BLANK_CHECK \
    ((int32_t)(700 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

/*  According to HW Manual the max timeout value when using the peripheral
 *  clock notification command is 60us. This is with a PCLK of 50MHz. The
 *  calculation below calculates the number of ICLK ticks needed for the
 *  timeout delay. The 10us number is adjusted linearly depending on
 *  the PCLK frequency.
 */
#define WAIT_MAX_NOTIFY_FCU_CLOCK \
    ((int32_t)(60 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

#elif defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N)

/*  According to HW Manual the Max Programming Time for 128 bytes (ROM)
 *  is 12ms. This is with a FCLK of 50MHz. The calculation below
 *  calculates the number of ICLK ticks needed for the timeout delay.
 *  The 12ms number is adjusted linearly depending on the FCLK frequency.
 */
#define WAIT_MAX_ROM_WRITE \
    ((int32_t)(12000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

/*  According to HW Manual the Max Programming Time for 2 bytes
 *  (Data Flash) is 2ms. This is with a FCLK of 50MHz. The calculation
 *  below calculates the number of ICLK ticks needed for the timeout delay.
 *  The 5ms number is adjusted linearly depending on the FCLK frequency.
 */
#define WAIT_MAX_DF_WRITE \
    ((int32_t)(2000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

/*  According to HW Manual the Max Blank Check time for 2k bytes
 *  (Data Flash) is 0.7ms. This is with a FCLK of 50MHz. The calculation
 *  below calculates the number of ICLK ticks needed for the timeout delay.
 *  The 0.7ms number is adjusted linearly depending on the FCLK frequency.
 */
#define WAIT_MAX_BLANK_CHECK \
    ((int32_t)(700 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

/*  According to HW Manual the max timeout value when using the peripheral
 *  clock notification command is 60us. This is with a FCLK of 50MHz. The
 *  calculation below calculates the number of ICLK ticks needed for the
 *  timeout delay. The 10us number is adjusted linearly depending on
 *  the FCLK frequency.
 */
#define WAIT_MAX_NOTIFY_FCU_CLOCK \
    ((int32_t)(60 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

#else // if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) || defined(MCU_RX62T)
#error "ERROR !!! Define timeout values for this device \
    in r_flash_api_rx600.h !!!"
#endif // if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) || defined(MCU_RX62T)

#if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) || defined(MCU_RX62T) \
    || defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N)
/* FCU-RAM address define
 * FCU F/W Store Address*/
#define FCU_PRG_TOP     0xFEFFE000
// FCU RAM Address 
#define FCU_RAM_TOP     0x007F8000
// FCU RAM Size 
#define FCU_RAM_SIZE    0x2000
#else // if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) ||
// defined(MCU_RX62T) || defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N) 
#error "ERROR !!! Set memory locations for FCU RAM and ROM \
    in r_flash_api_rx600.h !!!"
#endif // if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) ||
// defined(MCU_RX62T) || defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N) 

// Memory specifics for the RX610 group 
#if defined(MCU_RX610)

// Defines the number of flash areas 
#define NUM_ROM_AREAS       2
// Defines the start program/erase address for the different flash areas 
#define ROM_AREA_0          (0x00F00000)
#define ROM_AREA_1          (0x00E00000)

//  Bottom of DF Area 
#define DF_ADDRESS      0x00100000
// Used for getting DF block 
#define DF_MASK         0xFFFF6000
// Used for getting erase boundary in DF block when doing blank checking 
#define DF_ERASE_BLOCK_SIZE   0x00002000
// Defines how many DF blocks are on this part 
#define DF_NUM_BLOCKS   4

// Defines how many ROM blocks are on this part 
#if ROM_SIZE_BYTES   == 2097152
#define ROM_NUM_BLOCKS  28  // 2MB part 
#elif ROM_SIZE_BYTES == 1572864
#define ROM_NUM_BLOCKS  24  // 1.5MB part 
#elif ROM_SIZE_BYTES == 1048576
#define ROM_NUM_BLOCKS  20  // 1MB part 
#elif ROM_SIZE_BYTES == 786432
#define ROM_NUM_BLOCKS  18  // 768KB part 
#endif // if ROM_SIZE_BYTES == 2097152

// Programming size for ROM in bytes 
#define ROM_PROGRAM_SIZE      256
/* Programming sizes for data flash in bytes. Some MCUs have two sizes
* (e.g. 8-bytes or 128-bytes) that's why there is a LARGE and SMALL */
#define DF_PROGRAM_SIZE_LARGE 128
#define DF_PROGRAM_SIZE_SMALL 8

/*  According to HW Manual the Max Erasure Time for a 128kB block
 *  is 1750ms. This is with a PCLK of 50MHz. The calculation below
 *  calculates the number of ICLK ticks needed for the timeout delay.
 *  The 1750ms number is adjusted linearly depending on the PCLK frequency.
 */
#define WAIT_MAX_ERASE \
    ((int32_t)(1750000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

// Memory specifics for the RX621/N group 
#elif defined(MCU_RX62N) || defined(MCU_RX621)

// Defines the number of flash areas 
#define NUM_ROM_AREAS       1
// Defines the start program/erase address for the different flash areas 
#define ROM_AREA_0          (0x00F80000)

//  Bottom of DF Area 
#define DF_ADDRESS      0x00100000
// Used for getting DF block 
#define DF_MASK         0xFFFFF800
// Used for getting erase boundary in DF block when doing blank checking 
#define DF_ERASE_BLOCK_SIZE   0x00000800
// Defines how many DF blocks are on this part 
#define DF_NUM_BLOCKS   16

// Defines how many ROM blocks are on this part 
#if ROM_SIZE_BYTES   == 524288
#define ROM_NUM_BLOCKS  38 // 512KB part 
#elif ROM_SIZE_BYTES == 393216
#define ROM_NUM_BLOCKS  30 // 384KB part 
#elif ROM_SIZE_BYTES == 262144
#define ROM_NUM_BLOCKS  22 // 256KB part 
#endif // if ROM_SIZE_BYTES == 524288

// Programming size for ROM in bytes 
#define ROM_PROGRAM_SIZE      256
// Programming size for data flash in bytes 
#define DF_PROGRAM_SIZE_LARGE 128
#define DF_PROGRAM_SIZE_SMALL 8

/*  According to HW Manual the Max Erasure Time for a 16kB block is
 *  around 300ms. This is with a PCLK of 50MHz. The calculation below
 *  calculates the number of ICLK ticks needed for the timeout delay.
 *  The 300ms number is adjusted linearly depending on the PCLK frequency.
 */
#define WAIT_MAX_ERASE \
    ((int32_t)(300000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

// Memory specifics for the RX62T group 
#elif defined(MCU_RX62T)

// Defines the number of flash areas 
#define NUM_ROM_AREAS       1
// Defines the start program/erase address for the different flash areas 
#define ROM_AREA_0          (0x00FC0000)

//  Bottom of DF Area 
#define DF_ADDRESS          0x00100000
// Used for getting DF block 
#define DF_MASK             0xFFFFF800
// Used for getting erase boundary in DF block when doing blank checking 
#define DF_ERASE_BLOCK_SIZE 0x00000800

// Defines how many DF blocks are on this part 
#if DF_SIZE_BYTES == 32768
#define DF_NUM_BLOCKS   16 //32KB DF part 
#else
#define DF_NUM_BLOCKS   4  // 8KB DF part 
#endif

// Defines how many ROM blocks are on this part 
#if ROM_SIZE_BYTES   == 262144
#define ROM_NUM_BLOCKS  22  // 256KB part 
#elif ROM_SIZE_BYTES == 131072
#define ROM_NUM_BLOCKS  14  // 128KB part 
#elif ROM_SIZE_BYTES == 65536
#define ROM_NUM_BLOCKS  10  // 64KB part 
#endif // if ROM_SIZE_BYTES == 262144

// Programming size for ROM in bytes 
#define ROM_PROGRAM_SIZE      256
// Programming size for data flash in bytes 
#define DF_PROGRAM_SIZE_LARGE 128
#define DF_PROGRAM_SIZE_SMALL 8

/*  According to HW Manual the Max Erasure Time for a 16kB block is
 *  around 300ms. This is with a PCLK of 50MHz. The calculation below
 *  calculates the number of ICLK ticks needed for the timeout delay.
 *  The 300ms number is adjusted linearly depending on the PCLK frequency.
 */
#define WAIT_MAX_ERASE \
    ((int32_t)(300000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

// Memory specifics for the RX630/631/63N group 
#elif defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N)

// Defines the number of flash areas 
#define NUM_ROM_AREAS       4
// Defines the start program/erase addresses for the different flash areas 
#define ROM_AREA_0          (0x00F80000)
#define ROM_AREA_1          (0x00F00000)
#define ROM_AREA_2          (0x00E80000)
#define ROM_AREA_3          (0x00E00000)

//  Bottom of DF Area 
#define DF_ADDRESS          0x00100000
// Used for getting DF block 
#define DF_MASK             0xFFFFF800
// Used for getting erase boundary in DF block when doing blank checking 
#define DF_ERASE_BLOCK_SIZE 0x00000020
// This is used to get the boundary of the 'fake' blocks that are 2KB. 
#define DF_BLOCK_SIZE_LARGE 0x00000800
// Defines how many DF blocks are on this part 
#define DF_NUM_BLOCKS 16

// Defines how many ROM blocks are on this part 
#if ROM_SIZE_BYTES   == 2097152
#define ROM_NUM_BLOCKS  70 // 2MB part 
#elif ROM_SIZE_BYTES == 1572864
#define ROM_NUM_BLOCKS  62 // 1.5MB part 
#elif ROM_SIZE_BYTES == 1048576
#define ROM_NUM_BLOCKS  54 // 1MB part 
#elif ROM_SIZE_BYTES == 786432
#define ROM_NUM_BLOCKS  46 // 768KB part 
#elif ROM_SIZE_BYTES == 524288
#define ROM_NUM_BLOCKS  38 // 512KB part 
#elif ROM_SIZE_BYTES == 393216
#define ROM_NUM_BLOCKS  30 // 384KB part 
#endif // if ROM_SIZE_BYTES == 2097152

// Programming size for ROM in bytes 
#define ROM_PROGRAM_SIZE    128
/* Programming size for data flash in bytes
 * NOTE: RX630/631/63N only programs in 2-byte intervals*/
#define DF_PROGRAM_SIZE_SMALL     2

/*  According to HW Manual the Max Erasure Time for a 64kB block is
 *  around 1152ms. This is with a FCLK of 50MHz. The calculation below
 *  calculates the number of ICLK ticks needed for the timeout delay.
 *  The 1152ms number is adjusted linearly depending on the FCLK frequency.
 */
#define WAIT_MAX_ERASE \
    ((int32_t)(1152000 * (50.0/(FLASH_CLOCK_HZ/1000000)))*(ICLK_HZ/1000000))

#else // if defined(MCU_RX610)
#error "!!! Need to define memory specifics for this RX600 family \
    in r_flash_api_rx600_private.h !!!"
#endif // if defined(MCU_RX610)


#endif // _FLASH_API_RX600_PRIVATE_H

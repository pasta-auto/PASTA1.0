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
* File Name    : r_flash_api_rx600.h
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

#ifndef _FLASH_API_RX600_H
#define _FLASH_API_RX600_H

#ifndef  _MCU_INFO
#include "mcu_info.h"
#endif

// User specific options for Flash API 
#include "r_flash_api_rx600_config.h"

// Version Number of API. 
#define RX600_FLASH_API_VERSION_MAJOR           (2)
#define RX600_FLASH_API_VERSION_MINOR           (20)

// Pointer definitions for what should be sent in to R_FlashWrite 
#define FLASH_PTR_TYPE uint32_t
#define   BUF_PTR_TYPE uint32_t

/* *** Function Return Values ***
 * Operation was successful*/
#define FLASH_SUCCESS           (0x00)
/* Flash area checked was blank, making this 0x00 as well to keep existing
 * code checking compatibility */
#define FLASH_BLANK             (0x00)
// The address that was supplied was not on aligned correctly for ROM or DF 
#define FLASH_ERROR_ALIGNED     (0x01)
/* Flash area checked was not blank, making this 0x01 as well to keep existing
 * code checking compatibility */
#define FLASH_NOT_BLANK         (0x01)
// The number of bytes supplied to write was incorrect 
#define FLASH_ERROR_BYTES       (0x02)
// The address provided is not a valid ROM or DF address 
#define FLASH_ERROR_ADDRESS     (0x03)
// Writes cannot cross the 1MB boundary on some parts 
#define FLASH_ERROR_BOUNDARY    (0x04)
// Flash is busy with another operation 
#define FLASH_BUSY              (0x05)
// Operation failed 
#define FLASH_FAILURE           (0x06)
// Lock bit was set for the block in question 
#define FLASH_LOCK_BIT_SET      (0x07)
// Lock bit was not set for the block in question 
#define FLASH_LOCK_BIT_NOT_SET  (0x08)

#if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) || defined(MCU_RX62T)
// 'size' parameter for R_FlashDataAreaBlankCheck 
#define BLANK_CHECK_8_BYTE       0
#elif defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N)
// 'size' parameter for R_FlashDataAreaBlankCheck 
#define BLANK_CHECK_2_BYTE       0
#endif // if defined(MCU_RX610) || defined(MCU_RX621) || defined(MCU_RX62N) || defined(MCU_RX62T)
// 'size' parameter for R_FlashDataAreaBlankCheck 
#define BLANK_CHECK_ENTIRE_BLOCK 1

uint8_t R_FlashErase(uint8_t block );
#if defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N)
uint8_t R_FlashEraseRange(uint32_t start_addr, uint32_t bytes);
#endif
uint8_t R_FlashWrite(uint32_t flash_addr, uint32_t buffer_addr, uint16_t bytes);
uint8_t R_FlashProgramLockBit(uint8_t block);
uint8_t R_FlashReadLockBit(uint8_t block);
uint8_t R_FlashSetLockBitProtection(uint8_t lock_bit);
uint8_t R_FlashGetStatus(void);
// Data Flash Only Functions 
void    R_FlashDataAreaAccess(uint16_t read_en_mask, uint16_t write_en_mask);
uint8_t R_FlashDataAreaBlankCheck(uint32_t address, uint8_t size);

// These functions are only used when BGO (non-blocking) mode is enabled 
#if defined(DATA_FLASH_BGO) || defined(ROM_BGO)
// Callback function to call when flash erase is finished 
void FlashEraseDone(void);
// Callback function to call when flash write is finished 
void FlashWriteDone(void);
// Function to take care of flash errors 
void FlashError(void);
/* Callback function to call when flash blank check is finished. 'result'
 * argument is 0 if block was blank and 1 if it was not */
void FlashBlankCheckDone(uint8_t result);
#endif // if defined(DATA_FLASH_BGO) || defined(ROM_BGO)

#ifdef COPY_CODE_BY_API
void R_FlashCodeCopy(void);
#endif

// Memory specifics for the RX610 group 
#if defined(MCU_RX610)

// User ROM Block Area           Size: Start Addr -   End Addr 
#define BLOCK_0     0           //    8KB: 0xFFFFE000 - 0xFFFFFFFF 
#define BLOCK_1     1           //    8KB: 0xFFFFC000 - 0xFFFFDFFF 
#define BLOCK_2     2           //    8KB: 0xFFFFA000 - 0xFFFFBFFF 
#define BLOCK_3     3           //    8KB: 0xFFFF8000 - 0xFFFF9FFF 
#define BLOCK_4     4           //    8KB: 0xFFFF6000 - 0xFFFF7FFF 
#define BLOCK_5     5           //    8KB: 0xFFFF4000 - 0xFFFF5FFF 
#define BLOCK_6     6           //    8KB: 0xFFFF2000 - 0xFFFF3FFF 
#define BLOCK_7     7           //    8KB: 0xFFFF0000 - 0xFFFF1FFF 
#define BLOCK_8     8           //   64KB: 0xFFFE0000 - 0xFFFEFFFF 
#define BLOCK_9     9           //   64KB: 0xFFFD0000 - 0xFFFDFFFF 
#define BLOCK_10    10          //   64KB: 0xFFFC0000 - 0xFFFCFFFF 
#define BLOCK_11    11          //   64KB: 0xFFFB0000 - 0xFFFBFFFF 
#define BLOCK_12    12          //   64KB: 0xFFFA0000 - 0xFFFAFFFF 
#define BLOCK_13    13          //   64KB: 0xFFF90000 - 0xFFF9FFFF 
#define BLOCK_14    14          //   64KB: 0xFFF80000 - 0xFFF8FFFF 
#define BLOCK_15    15          //   64KB: 0xFFF70000 - 0xFFF7FFFF 
#define BLOCK_16    16          //   64KB: 0xFFF60000 - 0xFFF6FFFF 
#define BLOCK_17    17          //  128KB: 0xFFF40000 - 0xFFF5FFFF 
#define BLOCK_18    18          //  128KB: 0xFFF20000 - 0xFFF3FFFF 
#define BLOCK_19    19          //  128KB: 0xFFF00000 - 0xFFF1FFFF 
#define BLOCK_20    20          //  128KB: 0xFFEE0000 - 0xFFEFFFFF 
#define BLOCK_21    21          //  128KB: 0xFFEC0000 - 0xFFEDFFFF 
#define BLOCK_22    22          //  128KB: 0xFFEA0000 - 0xFFEBFFFF 
#define BLOCK_23    23          //  128KB: 0xFFE80000 - 0xFFE9FFFF 
#define BLOCK_24    24          //  128KB: 0xFFE60000 - 0xFFE7FFFF 
#define BLOCK_25    25          //  128KB: 0xFFE40000 - 0xFFE5FFFF 
#define BLOCK_26    26          //  128KB: 0xFFE20000 - 0xFFE3FFFF 
#define BLOCK_27    27          //  128KB: 0xFFE00000 - 0xFFE1FFFF 

// Data Flash Block Area         Size: Start Addr -   End Addr 
#define BLOCK_DB0   28          //    8KB: 0x00100000 - 0x00101FFF 
#define BLOCK_DB1   29          //    8KB: 0x00102000 - 0x00103FFF 
#define BLOCK_DB2   30          //    8KB: 0x00104000 - 0x00105FFF 
#define BLOCK_DB3   31          //    8KB: 0x00106000 - 0x00107FFF 

// Array of flash addresses used for writing 
#if defined(FLASH_BLOCKS_DECLARE)
const uint32_t g_flash_BlockAddresses[32] = {
    // Caution. ID CODE(FFFFFFA0-FFFFFFAF) is excluded. 
    0x00FFE000,          // EB00 
    0x00FFC000,          // EB01 
    0x00FFA000,          // EB02 
    0x00FF8000,          // EB03 
    0x00FF6000,          // EB04 
    0x00FF4000,          // EB05 
    0x00FF2000,          // EB06 
    0x00FF0000,          // EB07 
    0x00FE0000,          // EB08 
    0x00FD0000,          // EB09 
    0x00FC0000,          // EB10 
    0x00FB0000,          // EB11 
    0x00FA0000,          // EB12 
    0x00F90000,          // EB13 
    0x00F80000,          // EB14 
    0x00F70000,          // EB15 
    0x00F60000,          // EB16 
    0x00F40000,          // EB17 
    0x00F20000,          // EB18 
    0x00F00000,          // EB19 
    0x00EE0000,          // EB20 
    0x00EC0000,          // EB21 
    0x00EA0000,          // EB22 
    0x00E80000,          // EB23 
    0x00E60000,          // EB24 
    0x00E40000,          // EB25 
    0x00E20000,          // EB26 
    0x00E00000,          // EB27 
    0x00100000,          // DB00 
    0x00102000,          // DB01 
    0x00104000,          // DB02 
    0x00106000};         // DB03 
#else // if defined(FLASH_BLOCKS_DECLARE)
extern const uint32_t g_flash_BlockAddresses[32];
#endif // if defined(FLASH_BLOCKS_DECLARE)

// Memory specifics for the RX621/N group 
#elif defined(MCU_RX62N) || defined(MCU_RX621)

// User ROM Block Area           Size: Start Addr -   End Addr 
#define BLOCK_0     0           //    4KB: 0xFFFFF000 - 0xFFFFFFFF 
#define BLOCK_1     1           //    4KB: 0xFFFFE000 - 0xFFFFEFFF 
#define BLOCK_2     2           //    4KB: 0xFFFFD000 - 0xFFFFDFFF 
#define BLOCK_3     3           //    4KB: 0xFFFFC000 - 0xFFFFCFFF 
#define BLOCK_4     4           //    4KB: 0xFFFFB000 - 0xFFFFBFFF 
#define BLOCK_5     5           //    4KB: 0xFFFFA000 - 0xFFFFAFFF 
#define BLOCK_6     6           //    4KB: 0xFFFF9000 - 0xFFFF9FFF 
#define BLOCK_7     7           //    4KB: 0xFFFF8000 - 0xFFFF8FFF 
#define BLOCK_8     8           //   16KB: 0xFFFF4000 - 0xFFFF7FFF 
#define BLOCK_9     9           //   16KB: 0xFFFF0000 - 0xFFFF3FFF 
#define BLOCK_10    10          //   16KB: 0xFFFEC000 - 0xFFFEFFFF 
#define BLOCK_11    11          //   16KB: 0xFFFE8000 - 0xFFFEBFFF 
#define BLOCK_12    12          //   16KB: 0xFFFE4000 - 0xFFFE7FFF 
#define BLOCK_13    13          //   16KB: 0xFFFE0000 - 0xFFFE3FFF 
#define BLOCK_14    14          //   16KB: 0xFFFDC000 - 0xFFFDFFFF 
#define BLOCK_15    15          //   16KB: 0xFFFD8000 - 0xFFFDBFFF 
#define BLOCK_16    16          //   16KB: 0xFFFD4000 - 0xFFFD7FFF 
#define BLOCK_17    17          //   16KB: 0xFFFD0000 - 0xFFFD3FFF 
#define BLOCK_18    18          //   16KB: 0xFFFCC000 - 0xFFFCFFFF 
#define BLOCK_19    19          //   16KB: 0xFFFC8000 - 0xFFFCBFFF 
#define BLOCK_20    20          //   16KB: 0xFFFC4000 - 0xFFFC7FFF 
#define BLOCK_21    21          //   16KB: 0xFFFC0000 - 0xFFFC3FFF 
#define BLOCK_22    22          //   16KB: 0xFFFBC000 - 0xFFFBFFFF 
#define BLOCK_23    23          //   16KB: 0xFFFB8000 - 0xFFFBBFFF 
#define BLOCK_24    24          //   16KB: 0xFFFB4000 - 0xFFFB7FFF 
#define BLOCK_25    25          //   16KB: 0xFFFB0000 - 0xFFFB3FFF 
#define BLOCK_26    26          //   16KB: 0xFFFAC000 - 0xFFFAFFFF 
#define BLOCK_27    27          //   16KB: 0xFFFA8000 - 0xFFFABFFF 
#define BLOCK_28    28          //   16KB: 0xFFFA4000 - 0xFFFA7FFF 
#define BLOCK_29    29          //   16KB: 0xFFFA0000 - 0xFFFA3FFF 
#define BLOCK_30    30          //   16KB: 0xFFF9C000 - 0xFFF9FFFF 
#define BLOCK_31    31          //   16KB: 0xFFF98000 - 0xFFF9BFFF 
#define BLOCK_32    32          //   16KB: 0xFFF94000 - 0xFFF97FFF 
#define BLOCK_33    33          //   16KB: 0xFFF90000 - 0xFFF93FFF 
#define BLOCK_34    34          //   16KB: 0xFFF8C000 - 0xFFF8FFFF 
#define BLOCK_35    35          //   16KB: 0xFFF88000 - 0xFFF8BFFF 
#define BLOCK_36    36          //   16KB: 0xFFF84000 - 0xFFF87FFF 
#define BLOCK_37    37          //   16KB: 0xFFF80000 - 0xFFF83FFF 

// Data Flash Block Area         Size: Start Addr -   End Addr 
#define BLOCK_DB0    38         //    2KB: 0x00100000 - 0x001007FF 
#define BLOCK_DB1    39         //    2KB: 0x00100800 - 0x00100FFF 
#define BLOCK_DB2    40         //    2KB: 0x00101000 - 0x001017FF 
#define BLOCK_DB3    41         //    2KB: 0x00101800 - 0x00101FFF 
#define BLOCK_DB4    42         //    2KB: 0x00102000 - 0x001027FF 
#define BLOCK_DB5    43         //    2KB: 0x00102800 - 0x00102FFF 
#define BLOCK_DB6    44         //    2KB: 0x00103000 - 0x001037FF 
#define BLOCK_DB7    45         //    2KB: 0x00103800 - 0x00103FFF 
#define BLOCK_DB8    46         //    2KB: 0x00104000 - 0x001047FF 
#define BLOCK_DB9    47         //    2KB: 0x00104800 - 0x00104FFF 
#define BLOCK_DB10   48         //    2KB: 0x00105000 - 0x001057FF 
#define BLOCK_DB11   49         //    2KB: 0x00105800 - 0x00105FFF 
#define BLOCK_DB12   50         //    2KB: 0x00106000 - 0x001067FF 
#define BLOCK_DB13   51         //    2KB: 0x00106800 - 0x00106FFF 
#define BLOCK_DB14   52         //    2KB: 0x00107000 - 0x001077FF 
#define BLOCK_DB15   53         //    2KB: 0x00107800 - 0x00107FFF 

// Array of flash addresses used for writing 
#if defined(FLASH_BLOCKS_DECLARE)
const uint32_t g_flash_BlockAddresses[54] = { \
    /* Caution. ID CODE(FFFFFFA0-FFFFFFAF) is excluded.  \
     *     //     0x00FFF000,          // EB00  \
     *     //     0x00FFE000,          // EB01  \
     *     //     0x00FFD000,          // EB02  \
     *     //     0x00FFC000,          // EB03  \
     *     //     0x00FFB000,          // EB04  \
     *     //     0x00FFA000,          // EB05  \
     *     //     0x00FF9000,          // EB06  \
     *     //     0x00FF8000,          // EB07  \
     *     //     0x00FF4000,          // EB08  \
     *     //     0x00FF0000,          // EB09  \
     *     //     0x00FEC000,          // EB10  \
     *     //     0x00FE8000,          // EB11  \
     *     //     0x00FE4000,          // EB12  \
     *     //     0x00FE0000,          // EB13  \
     *     //     0x00FDC000,          // EB14  \
     *     //     0x00FD8000,          // EB15  \
     *     //     0x00FD4000,          // EB16  \
     *     //     0x00FD0000,          // EB17  \
     *     //     0x00FCC000,          // EB18  \
     *     //     0x00FC8000,          // EB19  \
     *     //     0x00FC4000,          // EB20  \
     *     //     0x00FC0000,          // EB21  \
     *     //     0x00FBC000,          // EB22  \
     *     //     0x00FB8000,          // EB23  \
     *     //     0x00FB4000,          // EB24  \
     *     //     0x00FB0000,          // EB25  \
     *     //     0x00FAC000,          // EB26  \
     *     //     0x00FA8000,          // EB27  \
     *     //     0x00FA4000,          // EB28  \
     *     //     0x00FA0000,          // EB29  \
     *     //     0x00F9C000,          // EB30  \
     *     //     0x00F98000,          // EB31  \
     *     //     0x00F94000,          // EB32  \
     *     //     0x00F90000,          // EB33  \
     *     //     0x00F8C000,          // EB34  \
     *     //     0x00F88000,          // EB35  \
     *     //     0x00F84000,          // EB36  \
     *     //     0x00F80000,          // EB37  \
     *     //     0x00100000,          // DB00  \
     *     //     0x00100800,          // DB01  \
     *     //     0x00101000,          // DB02  \
     *     //     0x00101800,          // DB03  \
     *     //     0x00102000,          // DB04  \
     *     //     0x00102800,          // DB05  \
     *     //     0x00103000,          // DB06  \
     *     //     0x00103800,          // DB07  \
     *     //     0x00104000,          // DB08  \
     *     //     0x00104800,          // DB09  \
     *     //     0x00105000,          // DB10  \
     *     //     0x00105800,          // DB11  \
     *     //     0x00106000,          // DB12  \
     *     //     0x00106800,          // DB13  \
     *     //     0x00107000,          // DB14  \
     *     //     0x00107800};         // DB15 */
#else // if defined(FLASH_BLOCKS_DECLARE)
extern const uint32_t g_flash_BlockAddresses[54];
#endif // if defined(FLASH_BLOCKS_DECLARE)

// Memory specifics for the RX62T group 
#elif defined(MCU_RX62T)

// User ROM Block Area           Size: Start Addr -   End Addr 
#define BLOCK_0     0           //    4KB: 0xFFFFF000 - 0xFFFFFFFF 
#define BLOCK_1     1           //    4KB: 0xFFFFE000 - 0xFFFFEFFF 
#define BLOCK_2     2           //    4KB: 0xFFFFD000 - 0xFFFFDFFF 
#define BLOCK_3     3           //    4KB: 0xFFFFC000 - 0xFFFFCFFF 
#define BLOCK_4     4           //    4KB: 0xFFFFB000 - 0xFFFFBFFF 
#define BLOCK_5     5           //    4KB: 0xFFFFA000 - 0xFFFFAFFF 
#define BLOCK_6     6           //    4KB: 0xFFFF9000 - 0xFFFF9FFF 
#define BLOCK_7     7           //    4KB: 0xFFFF8000 - 0xFFFF8FFF 
#define BLOCK_8     8           //   16KB: 0xFFFF4000 - 0xFFFF7FFF 
#define BLOCK_9     9           //   16KB: 0xFFFF0000 - 0xFFFF3FFF 
#define BLOCK_10    10          //   16KB: 0xFFFEC000 - 0xFFFEFFFF 
#define BLOCK_11    11          //   16KB: 0xFFFE8000 - 0xFFFEBFFF 
#define BLOCK_12    12          //   16KB: 0xFFFE4000 - 0xFFFE7FFF 
#define BLOCK_13    13          //   16KB: 0xFFFE0000 - 0xFFFE3FFF 
#define BLOCK_14    14          //   16KB: 0xFFFDC000 - 0xFFFDFFFF 
#define BLOCK_15    15          //   16KB: 0xFFFD8000 - 0xFFFDBFFF 
#define BLOCK_16    16          //   16KB: 0xFFFD4000 - 0xFFFD7FFF 
#define BLOCK_17    17          //   16KB: 0xFFFD0000 - 0xFFFD3FFF 
#define BLOCK_18    18          //   16KB: 0xFFFCC000 - 0xFFFCFFFF 
#define BLOCK_19    19          //   16KB: 0xFFFC8000 - 0xFFFCBFFF 
#define BLOCK_20    20          //   16KB: 0xFFFC4000 - 0xFFFC7FFF 
#define BLOCK_21    21          //   16KB: 0xFFFC0000 - 0xFFFC3FFF 

// Data Flash Block Area         Size: Start Addr -   End Addr 
#define BLOCK_DB0    22         //    2KB: 0x00100000 - 0x001007FF 
#define BLOCK_DB1    23         //    2KB: 0x00100800 - 0x00100FFF 
#define BLOCK_DB2    24         //    2KB: 0x00101000 - 0x001017FF 
#define BLOCK_DB3    25         //    2KB: 0x00101800 - 0x00101FFF 
#define BLOCK_DB4    26         //    2KB: 0x00102000 - 0x001027FF 
#define BLOCK_DB5    27         //    2KB: 0x00102800 - 0x00102FFF 
#define BLOCK_DB6    28         //    2KB: 0x00103000 - 0x001037FF 
#define BLOCK_DB7    29         //    2KB: 0x00103800 - 0x00103FFF 
#define BLOCK_DB8    30         //    2KB: 0x00104000 - 0x001047FF 
#define BLOCK_DB9    31         //    2KB: 0x00104800 - 0x00104FFF 
#define BLOCK_DB10   32         //    2KB: 0x00105000 - 0x001057FF 
#define BLOCK_DB11   33         //    2KB: 0x00105800 - 0x00105FFF 
#define BLOCK_DB12   34         //    2KB: 0x00106000 - 0x001067FF 
#define BLOCK_DB13   35         //    2KB: 0x00106800 - 0x00106FFF 
#define BLOCK_DB14   36         //    2KB: 0x00107000 - 0x001077FF 
#define BLOCK_DB15   37         //    2KB: 0x00107800 - 0x00107FFF 

// Array of flash addresses used for writing 
#if defined(FLASH_BLOCKS_DECLARE)
const uint32_t g_flash_BlockAddresses[38] = {
    // Caution. ID CODE(FFFFFFA0-FFFFFFAF) is excluded. 
    0x00FFF000,          // EB00 
    0x00FFE000,          // EB01 
    0x00FFD000,          // EB02 
    0x00FFC000,          // EB03 
    0x00FFB000,          // EB04 
    0x00FFA000,          // EB05 
    0x00FF9000,          // EB06 
    0x00FF8000,          // EB07 
    0x00FF4000,          // EB08 
    0x00FF0000,          // EB09 
    0x00FEC000,          // EB10 
    0x00FE8000,          // EB11 
    0x00FE4000,          // EB12 
    0x00FE0000,          // EB13 
    0x00FDC000,          // EB14 
    0x00FD8000,          // EB15 
    0x00FD4000,          // EB16 
    0x00FD0000,          // EB17 
    0x00FCC000,          // EB18 
    0x00FC8000,          // EB19 
    0x00FC4000,          // EB20 
    0x00FC0000,          // EB21 
    0x00100000,          // DB00 
    0x00100800,          // DB01 
    0x00101000,          // DB02 
    0x00101800,          // DB03 
    0x00102000,          // DB04 
    0x00102800,          // DB05 
    0x00103000,          // DB06 
    0x00103800,          // DB07 
    0x00104000,          // DB08 
    0x00104800,          // DB09 
    0x00105000,          // DB10 
    0x00105800,          // DB11 
    0x00106000,          // DB12 
    0x00106800,          // DB13 
    0x00107000,          // DB14 
    0x00107800};         // DB15 
#else // if defined(FLASH_BLOCKS_DECLARE)
extern const uint32_t g_flash_BlockAddresses[38];
#endif // if defined(FLASH_BLOCKS_DECLARE)

// Memory specifics for the RX630/631/63N group 
#elif defined(MCU_RX630) || defined(MCU_RX631) || defined(MCU_RX63N)

// User ROM Block Area           Size: Start Addr -   End Addr 
#define BLOCK_0     0           //    4KB: 0xFFFFF000 - 0xFFFFFFFF 
#define BLOCK_1     1           //    4KB: 0xFFFFE000 - 0xFFFFEFFF 
#define BLOCK_2     2           //    4KB: 0xFFFFD000 - 0xFFFFDFFF 
#define BLOCK_3     3           //    4KB: 0xFFFFC000 - 0xFFFFCFFF 
#define BLOCK_4     4           //    4KB: 0xFFFFB000 - 0xFFFFBFFF 
#define BLOCK_5     5           //    4KB: 0xFFFFA000 - 0xFFFFAFFF 
#define BLOCK_6     6           //    4KB: 0xFFFF9000 - 0xFFFF9FFF 
#define BLOCK_7     7           //    4KB: 0xFFFF8000 - 0xFFFF8FFF 
#define BLOCK_8     8           //   16KB: 0xFFFF4000 - 0xFFFF7FFF 
#define BLOCK_9     9           //   16KB: 0xFFFF0000 - 0xFFFF3FFF 
#define BLOCK_10    10          //   16KB: 0xFFFEC000 - 0xFFFEFFFF 
#define BLOCK_11    11          //   16KB: 0xFFFE8000 - 0xFFFEBFFF 
#define BLOCK_12    12          //   16KB: 0xFFFE4000 - 0xFFFE7FFF 
#define BLOCK_13    13          //   16KB: 0xFFFE0000 - 0xFFFE3FFF 
#define BLOCK_14    14          //   16KB: 0xFFFDC000 - 0xFFFDFFFF 
#define BLOCK_15    15          //   16KB: 0xFFFD8000 - 0xFFFDBFFF 
#define BLOCK_16    16          //   16KB: 0xFFFD4000 - 0xFFFD7FFF 
#define BLOCK_17    17          //   16KB: 0xFFFD0000 - 0xFFFD3FFF 
#define BLOCK_18    18          //   16KB: 0xFFFCC000 - 0xFFFCFFFF 
#define BLOCK_19    19          //   16KB: 0xFFFC8000 - 0xFFFCBFFF 
#define BLOCK_20    20          //   16KB: 0xFFFC4000 - 0xFFFC7FFF 
#define BLOCK_21    21          //   16KB: 0xFFFC0000 - 0xFFFC3FFF 
#define BLOCK_22    22          //   16KB: 0xFFFBC000 - 0xFFFBFFFF 
#define BLOCK_23    23          //   16KB: 0xFFFB8000 - 0xFFFBBFFF 
#define BLOCK_24    24          //   16KB: 0xFFFB4000 - 0xFFFB7FFF 
#define BLOCK_25    25          //   16KB: 0xFFFB0000 - 0xFFFB3FFF 
#define BLOCK_26    26          //   16KB: 0xFFFAC000 - 0xFFFAFFFF 
#define BLOCK_27    27          //   16KB: 0xFFFA8000 - 0xFFFABFFF 
#define BLOCK_28    28          //   16KB: 0xFFFA4000 - 0xFFFA7FFF 
#define BLOCK_29    29          //   16KB: 0xFFFA0000 - 0xFFFA3FFF 
#define BLOCK_30    30          //   16KB: 0xFFF9C000 - 0xFFF9FFFF 
#define BLOCK_31    31          //   16KB: 0xFFF98000 - 0xFFF9BFFF 
#define BLOCK_32    32          //   16KB: 0xFFF94000 - 0xFFF97FFF 
#define BLOCK_33    33          //   16KB: 0xFFF90000 - 0xFFF93FFF 
#define BLOCK_34    34          //   16KB: 0xFFF8C000 - 0xFFF8FFFF 
#define BLOCK_35    35          //   16KB: 0xFFF88000 - 0xFFF8BFFF 
#define BLOCK_36    36          //   16KB: 0xFFF84000 - 0xFFF87FFF 
#define BLOCK_37    37          //   16KB: 0xFFF80000 - 0xFFF83FFF 
#define BLOCK_38    38          //   32KB: 0xFFF78000 - 0xFFF7FFFF 
#define BLOCK_39    39          //   32KB: 0xFFF70000 - 0xFFF77FFF 
#define BLOCK_40    40          //   32KB: 0xFFF68000 - 0xFFF6FFFF 
#define BLOCK_41    41          //   32KB: 0xFFF60000 - 0xFFF67FFF 
#define BLOCK_42    42          //   32KB: 0xFFF58000 - 0xFFF5FFFF 
#define BLOCK_43    43          //   32KB: 0xFFF50000 - 0xFFF57FFF 
#define BLOCK_44    44          //   32KB: 0xFFF48000 - 0xFFF4FFFF 
#define BLOCK_45    45          //   32KB: 0xFFF40000 - 0xFFF47FFF 
#define BLOCK_46    46          //   32KB: 0xFFF38000 - 0xFFF3FFFF 
#define BLOCK_47    47          //   32KB: 0xFFF30000 - 0xFFF37FFF 
#define BLOCK_48    48          //   32KB: 0xFFF28000 - 0xFFF2FFFF 
#define BLOCK_49    49          //   32KB: 0xFFF20000 - 0xFFF27FFF 
#define BLOCK_50    50          //   32KB: 0xFFF18000 - 0xFFF1FFFF 
#define BLOCK_51    51          //   32KB: 0xFFF10000 - 0xFFF17FFF 
#define BLOCK_52    52          //   32KB: 0xFFF08000 - 0xFFF0FFFF 
#define BLOCK_53    53          //   32KB: 0xFFF00000 - 0xFFF07FFF 
#define BLOCK_54    54          //   64KB: 0xFFEF0000 - 0xFFEFFFFF 
#define BLOCK_55    55          //   64KB: 0xFFEE0000 - 0xFFEEFFFF 
#define BLOCK_56    56          //   64KB: 0xFFED0000 - 0xFFEDFFFF 
#define BLOCK_57    57          //   64KB: 0xFFEC0000 - 0xFFECFFFF 
#define BLOCK_58    58          //   64KB: 0xFFEB0000 - 0xFFEBFFFF 
#define BLOCK_59    59          //   64KB: 0xFFEA0000 - 0xFFEAFFFF 
#define BLOCK_60    60          //   64KB: 0xFFE90000 - 0xFFE9FFFF 
#define BLOCK_61    61          //   64KB: 0xFFE80000 - 0xFFE8FFFF 
#define BLOCK_62    62          //   64KB: 0xFFE70000 - 0xFFE7FFFF 
#define BLOCK_63    63          //   64KB: 0xFFE60000 - 0xFFE6FFFF 
#define BLOCK_64    64          //   64KB: 0xFFE50000 - 0xFFE5FFFF 
#define BLOCK_65    65          //   64KB: 0xFFE40000 - 0xFFE4FFFF 
#define BLOCK_66    66          //   64KB: 0xFFE30000 - 0xFFE3FFFF 
#define BLOCK_67    67          //   64KB: 0xFFE20000 - 0xFFE2FFFF 
#define BLOCK_68    68          //   64KB: 0xFFE10000 - 0xFFE1FFFF 
#define BLOCK_69    69          //   64KB: 0xFFE00000 - 0xFFE0FFFF 

/* NOTE:
 * The RX630/631/63N actually has 1024 x 32 byte blocks instead of the
 * 16 x 2Kbyte blocks shown below. These are grouped into 16 blocks to
 * make it easier for the user to delete larger sections of the data
 * flash. The user can still delete individual blocks but they will
 * need to use the new flash erase function that takes addresses
 * instead of blocks. */

// Data Flash Block Area         Size: Start Addr -   End Addr 
#define BLOCK_DB0    70         //    2KB: 0x00100000 - 0x001007FF 
#define BLOCK_DB1    71         //    2KB: 0x00100800 - 0x00100FFF 
#define BLOCK_DB2    72         //    2KB: 0x00101000 - 0x001017FF 
#define BLOCK_DB3    73         //    2KB: 0x00101800 - 0x00101FFF 
#define BLOCK_DB4    74         //    2KB: 0x00102000 - 0x001027FF 
#define BLOCK_DB5    75         //    2KB: 0x00102800 - 0x00102FFF 
#define BLOCK_DB6    76         //    2KB: 0x00103000 - 0x001037FF 
#define BLOCK_DB7    77         //    2KB: 0x00103800 - 0x00103FFF 
#define BLOCK_DB8    78         //    2KB: 0x00104000 - 0x001047FF 
#define BLOCK_DB9    79         //    2KB: 0x00104800 - 0x00104FFF 
#define BLOCK_DB10   80         //    2KB: 0x00105000 - 0x001057FF 
#define BLOCK_DB11   81         //    2KB: 0x00105800 - 0x00105FFF 
#define BLOCK_DB12   82         //    2KB: 0x00106000 - 0x001067FF 
#define BLOCK_DB13   83         //    2KB: 0x00106800 - 0x00106FFF 
#define BLOCK_DB14   84         //    2KB: 0x00107000 - 0x001077FF 
#define BLOCK_DB15   85         //    2KB: 0x00107800 - 0x00107FFF 

// Array of flash addresses used for writing 
#if defined(FLASH_BLOCKS_DECLARE)
const uint32_t g_flash_BlockAddresses[86] = {
    // Caution. ID CODE(FFFFFFA0-FFFFFFAF) is excluded. 
    0x00FFF000,          // EB00 
    0x00FFE000,          // EB01 
    0x00FFD000,          // EB02 
    0x00FFC000,          // EB03 
    0x00FFB000,          // EB04 
    0x00FFA000,          // EB05 
    0x00FF9000,          // EB06 
    0x00FF8000,          // EB07 
    0x00FF4000,          // EB08 
    0x00FF0000,          // EB09 
    0x00FEC000,          // EB10 
    0x00FE8000,          // EB11 
    0x00FE4000,          // EB12 
    0x00FE0000,          // EB13 
    0x00FDC000,          // EB14 
    0x00FD8000,          // EB15 
    0x00FD4000,          // EB16 
    0x00FD0000,          // EB17 
    0x00FCC000,          // EB18 
    0x00FC8000,          // EB19 
    0x00FC4000,          // EB20 
    0x00FC0000,          // EB21 
    0x00FBC000,          // EB22 
    0x00FB8000,          // EB23 
    0x00FB4000,          // EB24 
    0x00FB0000,          // EB25 
    0x00FAC000,          // EB26 
    0x00FA8000,          // EB27 
    0x00FA4000,          // EB28 
    0x00FA0000,          // EB29 
    0x00F9C000,          // EB30 
    0x00F98000,          // EB31 
    0x00F94000,          // EB32 
    0x00F90000,          // EB33 
    0x00F8C000,          // EB34 
    0x00F88000,          // EB35 
    0x00F84000,          // EB36 
    0x00F80000,          // EB37 
    0x00F78000,          // EB38 
    0x00F70000,          // EB39 
    0x00F68000,          // EB40 
    0x00F60000,          // EB41 
    0x00F58000,          // EB42 
    0x00F50000,          // EB43 
    0x00F48000,          // EB44 
    0x00F40000,          // EB45 
    0x00F38000,          // EB46 
    0x00F30000,          // EB47 
    0x00F28000,          // EB48 
    0x00F20000,          // EB49 
    0x00F18000,          // EB50 
    0x00F10000,          // EB51 
    0x00F08000,          // EB52 
    0x00F00000,          // EB53 
    0x00EF0000,          // EB54 
    0x00EE0000,          // EB55 
    0x00ED0000,          // EB56 
    0x00EC0000,          // EB57 
    0x00EB0000,          // EB58 
    0x00EA0000,          // EB59 
    0x00E90000,          // EB60 
    0x00E80000,          // EB61 
    0x00E70000,          // EB62 
    0x00E60000,          // EB63 
    0x00E50000,          // EB64 
    0x00E40000,          // EB65 
    0x00E30000,          // EB66 
    0x00E20000,          // EB67 
    0x00E10000,          // EB68 
    0x00E00000,          // EB69 
    0x00100000,          // DB00 
    0x00100800,          // DB01 
    0x00101000,          // DB02 
    0x00101800,          // DB03 
    0x00102000,          // DB04 
    0x00102800,          // DB05 
    0x00103000,          // DB06 
    0x00103800,          // DB07 
    0x00104000,          // DB08 
    0x00104800,          // DB09 
    0x00105000,          // DB10 
    0x00105800,          // DB11 
    0x00106000,          // DB12 
    0x00106800,          // DB13 
    0x00107000,          // DB14 
    0x00107800};         // DB15 

#else // if defined(FLASH_BLOCKS_DECLARE)
extern const uint32_t g_flash_BlockAddresses[86];
#endif // if defined(FLASH_BLOCKS_DECLARE)

#else // if defined(MCU_RX610)
#error "!!! Need to define memory specifics for this RX600 family \
    in r_flash_api_rx600.h !!!"
#endif // if defined(MCU_RX610)


#endif // _FLASH_API_RX600_H

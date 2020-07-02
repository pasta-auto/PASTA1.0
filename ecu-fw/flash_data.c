/* ---------------------------------------------------------------------------------------

*  Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.
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
* ---------------------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------------------

* File Name     : flash_data.c
* Version       : 1.00
* Device        : R5F563NB
* Tool-Chain    : Renesas RX Standard 1.2.0.0
* H/W Platform  : RSK+RX63N
* Description   : Defines flash data functions used in this sample.
* ---------------------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------------------
* History       : 13 Aug. 2012  Ver. 1.00 First Release
* ---------------------------------------------------------------------------------------*/

// Provides standard string function definitions used in this file 
#include <string.h>
#include "altypes.h"
#include "iodefine.h"

// Declares prototypes for functions defined in this file 
#include "flash_data.h"
// Defines flash API functions used in this file 
#include "r_Flash_API_RX600.h"

// Flash write buffer array
uint8_t gFlashWriteBuffer[6];
// Flash write address location global variable 
uint32_t gFlashWriteAddr;
// Total number of flash data blocks 
uint16_t gNumFlashBlocks = sizeof(g_flash_BlockAddresses)/4u;

// Switch callback function prototype delcaration
void CB_Switch(void);

void Exit_FlashData(void)
{
    // Enable MCU access to the flash data area 
    R_FlashDataAreaAccess(0x0, 0x0);
}

/* ---------------------------------------------------------------------------------------
* Init_FlashData
*
* Description
*     This function initialises the MCU flash area, allowing it to be
*     read and written to by user code. The function then calls the
*     InitADC_FlashData function to configure the ADC unit used in
*     the project. Finally the function erases the contents of the
*     flash data.
*
* Argument
*     None
*
* Return
*     None
* ---------------------------------------------------------------------------------------*/
void Init_FlashData(void)
{
    // Enable MCU access to the flash data area 
    R_FlashDataAreaAccess(0xFFFF, 0xFFFF);
}

/* ---------------------------------------------------------------------------------------
* Erase_FlashData
*
* Description
*     This function enters a for loop, and erases a block of 
*     flash data memory each iteration until all blocks have been erased.
*
* Argument
*     None
*
* Return
*     None
* ---------------------------------------------------------------------------------------*/
void Erase_FlashData(void)
{
    // Declare flash API error flag 
    uint8_t ret = 0u;

    // Declare current flash data block variable 
    uint8_t current_block;

    // Initialise a for loop to erase each of the flash data blocks 
    for (
        current_block = BLOCK_DB0; current_block < gNumFlashBlocks;
        current_block++
    ) {
        // Fetch beginning address of DF block 
        uint32_t address = g_flash_BlockAddresses[current_block];

        // Erase flash data block 
        ret |= R_FlashErase(current_block);

        // Halt here if erase was unsuccessful 
        while (R_FlashGetStatus() != FLASH_SUCCESS) {
            ;
        }

        // Check Blank Checking 
        ret |=  R_FlashDataAreaBlankCheck(
                    address,
                    BLANK_CHECK_ENTIRE_BLOCK
        );

        // Halt here if check was unsuccessful 
        while (R_FlashGetStatus() != FLASH_SUCCESS) {
            ;
        }
    }

    return;
}

/* ---------------------------------------------------------------------------------------
* Write_FlashData

* Description
*     This function writes the contents of gFlashWriteBuffer to the
*     flash data, at the location pointed by gFlashWriteAddr. If the
*     number of bytes to write to flash is not a multiple of 8, the
*     data is padded with null bytes (0x00) until the length is the
*     nearest multiple of 8.
*
* Argument
*     None
*
* Return
*     None
* ---------------------------------------------------------------------------------------*/
int Write_FlashData(void)
{
    // Declare flash API error flag 
    uint8_t ret = 0;

    // Declare data padding array and loop counter 
    uint8_t pad_buffer[256];

    // Declare pointer to flash write location 
    uint32_t * flash_ptr = (uint32_t *)gFlashWriteAddr;

    /* Declare the number of bytes variable, and initialise with the number of
     * bytes the variable gFlashWriteBuffer contains */
    uint8_t num_bytes = sizeof(gFlashWriteBuffer);

    // Clear the contents of the flash write buffer array 
    memset(pad_buffer, 0x00, 256);

    // Copy contents of the write buffer to the padding buffer 
    memcpy((char*)pad_buffer, (char*)gFlashWriteBuffer, num_bytes);

    // Write contents of write buffer to flash data 
    ret |=  R_FlashWrite(
                gFlashWriteAddr,
                (uint32_t)pad_buffer,
                (uint16_t)num_bytes
    );

    // Compare memory locations to verify written data 
    ret |= memcmp(gFlashWriteBuffer, flash_ptr, (size_t)num_bytes);

    return ret;
}

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

/* ----------------------------------------------------------------------------------------
 *
 * REPROGRAM Protocol processing
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2017/02/13 Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ----------------------------------------------------------------------------------------
 */

#include <sysio.h>
#include <string.h>
#include <stdio.h>
#include "iodefine.h"
#include "altypes.h"
#include "timer.h"
#include "flash_data.h"
#include "r_Flash_API_RX600.h"
#include "flash_rom.h"
#include "ecu.h"            // ECU common definition 
#include "can3_spi2.h"      // CAN3 definition 

/*
 *  Reprogram processing overview
 *
 *  CAN frame definition
 *
 *  ECU
 *  CAN-ID 0x7C0 to 7C7
 *
 * ECU query received (0x7DF) *All ECUs are processed
 *              +---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
 *      Byte    |       0       |       1       |       2       |       3       |       4       |       5       |       6       |       7       |
 *              +---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
 *      Meaning |  Data length  |    Command    |Additional Info|               |               |               |               |               |
 *              +---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
 *      Range   |      0-7      |    00 - FF    |    00 - FF    |               |               |               |               |               |
 *              +---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
 *
 *  CAN-ID 0x7C0 Request  to ECU0 (powertrain system)
 *         0x7C1 Request for ECU1 (body system)
 *         0x7C2 Request for ECU2 (chassis system)
 *         0x7C7 Request  to ECU3 (CGW-DTC)
 *
 *  ECU Response ID
 *  CAN-ID 0x7C8 Response from ECU0 (powertrain system)
 *         0x7C9 Response from ECU1 (body system)
 *         0x7CA Response from ECU2 (chassis system)
 *         0x7CF Response from ECU3 (CGW-DTC)
 */

extern void restart_pos(void);

/* ----------------------------------------------------------------------------------------
 * OBD2 packet union definition
 * ---------------------------------------------------------------------------------------- */
typedef union   __repro_query_frame__ {
    unsigned char BYTE[8];
    struct  {
        unsigned char   LEN;    // Data length 
        unsigned char   CMD;    // Command 
        unsigned char   ADH;    // Upper 8 bits of address 
        unsigned char   ADL;    // Lower 8 bits of address 
        unsigned char   DAT[4]; // Additional information 
    }   PACK;
}   REPRO_QUERY_FRAME;

/* ----------------------------------------------------------------------------------------
 * Command code definition
 * ---------------------------------------------------------------------------------------- */
#define     ALL_CONFIG_SAVE         0x07 // Batch save operation data to data flash 
#define     GET_ROUTING_MAP         0x08 // Get    routing map 
#define     SET_ROUTING_MAP         0x0A // Change routing map 
#define     ERA_ROUTING_MAP         0x0E // Delete routing map ROM 
#define     SAV_ROUTING_MAP         0x0F // Save   routing map ROM 

#define     DEL_CYCEVE_LIST         0x11 // Delete the specified ID from the list of active periodic transmissions and events 
#define     NEW_CYCEVE_LIST         0x12 // Add a new ID to the active periodic transmission / event list 
#define     GET_CYCEVE_LIST         0x13 // Get periodic    transmission / event list 
#define     SET_CYCEVE_LIST         0x14 // Change periodic transmission / event list 
#define     GET_CYCEVE_LIST1        0x18 // Get periodic    transmission / event list ID 
#define     GET_CYCEVE_LIST2        0x19 // Get periodic    transmission / event list timer 
#define     SET_CYCEVE_LIST1        0x1A // Change periodic transmission / event list ID 
#define     SET_CYCEVE_LIST2        0x1B // Change periodic transmission / event list timer 
#define     ERA_CYCEVE_LIST         0x1E // Delete periodic transmission / event list ROM 
#define     SAV_CYCEVE_LIST         0x1F // Save periodic   transmission / event list ROM 

#define     GET_EXT_IO_LIST         0x28        // Get    external I/O list 
#define     SET_EXT_IO_LIST         0x2A        // Save   external I/O list 
#define     ERA_EXT_IO_LIST         0x2E        // Change external I/O list 
#define     SAV_EXT_IO_LIST         0x2F        // Save   external I/O list ROM 

#define     READ_FIRMWARE           0x38        // Get    firmware 
#define     UPDATE_FIRMWARE         0x3C        // Save   firmware ROM 
#define     REMOVE_FIRMWARE         0x3D        // Delete firmware ROM 
#define     COPY_FIRMWARE           0x3E        // Transfer to firmware ROM writing buffer 
#define     WRITE_FIRMWARE          0x3F        // Firmware ROM writing (delete first) 

/* ----------------------------------------------------------------------------------------
 * Variable definition
 * ---------------------------------------------------------------------------------------- */
REPRO_QUERY_FRAME   repro_req;      // Variable definition 
REPRO_QUERY_FRAME   repro_ret;      // Response data 
unsigned char       fw_image[128];  // Write-only memory buffer 
unsigned long       fw_address;     // Write-only memory address 

/* ----------------------------------------------------------------------------------------
 * boot_copy
 * 
 *  Outline
 *      Copy APP to ROM
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      None
 *----------------------------------------------------------------------------------------*/
int bootcopy(void)
{
    unsigned long   rom = 0xFFF40000;
    unsigned long   app = 0x00000000;
    unsigned long   ape = 0x00020000;
    unsigned long   stp = 128;

    *((unsigned long *)0x60) = (unsigned long)restart_pos;

    for (; app < ape; app += stp, rom += stp) {
        _di();
        if (R_FlashWrite(rom, app, (unsigned short)stp) != FLASH_SUCCESS) {
            _ei();
            return 0;
        }
        _ei();
    }
    return 1;
}

/* ----------------------------------------------------------------------------------------
 * erace_rom
 *  
 *  Outline
 *      Erase specified area of ROM (0xFFF00000 to 0xFFF7FFFF)
 *  
 *  Argument
 *      sadr  Starting address
 *      eadr  Ending address
 *  
 *  Return
 *      char*    Message
 * ----------------------------------------------------------------------------------------*/
int bootclear(void)
{
    int blk;

    for (blk = BLOCK_53; blk > BLOCK_37; blk--) {
        _di();
        if (R_FlashErase(blk) != FLASH_SUCCESS) {
            _ei();
            return 0;
        }
        _ei();
    }
    return 1;
}

/* ----------------------------------------------------------------------------------------
 * Batch storage of ECU operation data
 * ---------------------------------------------------------------------------------------- */
int ecu_data_write(void)
{
    int bk, i;
    int wp  = 0;
    int fe  = 0;

    // Execute area erase 
    for (i = 0, bk = BLOCK_DB0; bk <= BLOCK_DB2; bk++, i++) {
        if (R_FlashDataAreaBlankCheck(
                                    g_flash_BlockAddresses[bk], 
                                    BLANK_CHECK_ENTIRE_BLOCK
        ) == FLASH_NOT_BLANK) { // With writing 
            while (R_FlashGetStatus() != FLASH_SUCCESS) {
                ;
            }
            if (R_FlashErase(bk) == FLASH_SUCCESS) {
                while (R_FlashGetStatus() != FLASH_SUCCESS) {
                    ;
                }
                if (R_FlashDataAreaBlankCheck(
                                            g_flash_BlockAddresses[bk],
                                            BLANK_CHECK_ENTIRE_BLOCK
                ) == FLASH_BLANK) { // Erase completed 
                    fe |= (1 << i);
                }
            }
        } else {
            fe |= (1 << i);
        }
    }

    // MAP writing 
    if ((fe & 1) != 0) { // Erase confirmed 
        if (R_FlashWrite(
                        ADDRESS_OF_ROOTMAP,
                        (int)&rout_map,
                        sizeof(ECU_ROUT_MAP)
        ) == FLASH_SUCCESS) { // Writing completed 
            wp |= 1;
        }
    }
    // CONF writing 
    if ((fe & 2) != 0) { // Erase confirmed 
        if (R_FlashWrite(
                        ADDRESS_OF_CYCEVE,
                        (int)&conf_ecu.LIST[0],
                        (sizeof(ECU_CYC_EVE) * MESSAGE_MAX)
        ) == FLASH_SUCCESS) { // Writing completed 
            wp |= 2;
        }
    }
    // I/O writing 
    if ((fe & 4) != 0) { // Erase confirmed 
        if (R_FlashWrite(
                        ADDRESS_OF_IOLIST,
                        (int)&ext_list[0],
                        sizeof(ext_list)
        ) == FLASH_SUCCESS) { // Writing completed 
            wp |= 4;
        }
    }
    while (R_FlashGetStatus() != FLASH_SUCCESS) {
        ;
    }
    return wp;
}

/* ----------------------------------------------------------------------------------------
 * Batch deletion of ECU operation data
 * ---------------------------------------------------------------------------------------- */
int ecu_data_erase(void)
{
    int bk, i;
    int fe = 0;

    for (i = 0, bk = BLOCK_DB0; bk <= BLOCK_DB15; bk++, i++) {
        if (R_FlashDataAreaBlankCheck(
                                    g_flash_BlockAddresses[bk],
                                    BLANK_CHECK_ENTIRE_BLOCK
        ) == FLASH_NOT_BLANK) { // With writing 
            while (R_FlashGetStatus() != FLASH_SUCCESS) {
                ;
            }
            if (R_FlashErase(bk) == FLASH_SUCCESS) {
                while (R_FlashGetStatus() != FLASH_SUCCESS) {
                    ;
                }
                if (R_FlashDataAreaBlankCheck(
                                            g_flash_BlockAddresses[bk],
                                            BLANK_CHECK_ENTIRE_BLOCK
                ) == FLASH_BLANK) { // Erase completed 
                    fe |= (1 << i);
                }
            }
        }
    }
    return fe;
}

/* ----------------------------------------------------------------------------------------
 * Checking the writing status of ECU operation data
 * ---------------------------------------------------------------------------------------- */
int ecu_data_check(void)
{
    int bk, i;
    int wf = 0;

    for (i = 0, bk = BLOCK_DB0; bk <= BLOCK_DB15; bk++, i++) {
        if (R_FlashDataAreaBlankCheck(
                                    g_flash_BlockAddresses[bk],
                                    BLANK_CHECK_ENTIRE_BLOCK
        ) == FLASH_NOT_BLANK) { // With writing 
            wf |= (1 << i);
        }
    }
    return wf;
}

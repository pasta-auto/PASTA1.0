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

/* ________________________________________________________________________________________
 *
 * UDS-CAN Protocol processing
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2017/08/13 Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ________________________________________________________________________________________
 */

#include <sysio.h>
#include <string.h>
#include <stdio.h>
#include "iodefine.h"
#include "timer.h"
#include "ecu.h"            // ECU common definition 
#include "can3_spi2.h"      // CAN3 definition 
#include "cantp.h"          // CAN-TP definition 
#include "uds.h"            // CAN-UDS definition 
#include "altypes.h"
#include "r_flash_api_rx600_config.h"
#include "mcu_info.h"
#include "r_flash_API_RX600.h"
#include "r_flash_api_rx600_private.h"

/*
 *  Overview of UDS (Unified Diagnostics Service) processing
 *
 *  UDS is a protocol that provides an integrated diagnostic service that sends and receives variable-length packets.
 *  In this firmware, only some services are implemented.
 *
 *  UDS service list (X = not implemented / O = implemented * but minimum)
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * | Function group    |Request|Respons|       Service                                 | Impl. |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * |    Diagnostic and |    10 |    50 |    Diagnostic Session Control                 |   O   |
 * +    Communications +-------+-------+-----------------------------------------------+-------+
 * |    Management     |    11 |    51 |    ECU Reset                                  |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    27 |    67 |    Security Access                            |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    28 |    68 |    Communication Control                      |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    3E |    7E |    Tester Present                             |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    83 |    C3 |    Access Timing Parameters                   |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    84 |    C4 |    Secured Data Transmission                  |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    85 |    C5 |    Control DTC Setting                        |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    86 |    C6 |    Response On Event                          |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    87 |    C7 |    Link Control                               |   X   |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * |    Data           |    22 |    62 |    Read Data By Identifier                    |   O   |
 * +    Transmission   +-------+-------+-----------------------------------------------+-------+
 * |                   |    23 |    63 |    Read Memory By Address                     |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    24 |    64 |    Read Scaling Data By Identifier            |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    2A |    6A |    Read Data By Identifire Periodic           |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    2C |    6C |    Dynamically Define Data Identifire         |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    2E |    6E |    Write Data By Identifire                   |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    3D |    7D |    Write Memory By Address                    |   O   |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * |    Stored Data    |    14 |    54 |    Clear Diagnostic Information               |   X   |
 * +    Transmission   +-------+-------+-----------------------------------------------+-------+
 * |                   |    19 |    59 |    Read DTC Information                       |   X   |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * |    I/O Control    |    2F |    6F |    Input Output Control By Identifire         |   X   |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * |    Remote         |       |       |                                               |       |
 * |    Activation of  |    31 |    71 |    Routine Control                            |   X   |
 * |    Routine        |       |       |                                               |       |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 * |    Upload /       |    34 |    74 |    Request Download                           |   O   |
 * +    Download       +-------+-------+-----------------------------------------------+-------+
 * |                   |    35 |    75 |    Request Upload                             |   X   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    36 |    76 |    Transfer Data                              |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    37 |    77 |    Request Transfer Exit                      |   O   |
 * +                   +-------+-------+-----------------------------------------------+-------+
 * |                   |    38 |    78 |    Request File Transfer                      |   X   |
 * +-------------------+-------+-------+-----------------------------------------------+-------+
 */

/* ----------------------------------------------------------------------------------------
 * Variable definition
 * ---------------------------------------------------------------------------------------- */
int uds_diag_session        = 0; // Session control 
int uds_p2_can_server_max   = 0; // P2 Time 
int uds_p2e_can_server_max  = 0; // P2E Time 
int uds_reset_request       = 0; // Reset request 
int uds_security_access     = 0; // Security access 

UDS_LOAD_STR uds_load;           // Download / upload management 

/*
 *  Repro regulations
 *
 *  ROM area setting
 *  E2DataFlash
 *  0x00100000 to 0x00107FFF  8K*4  Parameter      <--- 0x00100000 to 0x00107FFF Download permission  / Erase size 0x00002000
 *  Program Block
 *  0xFFE00000 to 0xFFEFFFFF 64K*16 Data Area      <--- 0xFFE00000 to 0xFFEFFFFF Download permission  / Erase size 0x00010000
 *  0xFFF00000 to 0xFFF3FFFF 32K*8  Download F/W   <--- 0xFFF00000 to 0xFFF3FFFF Download permission  / Erase size 0x00008000
 *  0xFFF40000 to 0xFFF7FFFF 32K*8  Default F/W    <--- 0xFFF40000 to 0xFFF7FFFF Download prohibition / Erase size 0x00008000
 *  0xFFF80000 to 0xFFFF7FFF 16K*29 Bootloader F/W <--- 0xFFF80000 to 0xFFFEFFFF Download prohibition / Erase size 0x00004000
 *  0xFFFF8000 to 0xFFFFFFFF  4K*8  Configuration  <--- 0xFFFF0000 to 0xFFFFFFFF Download prohibition / Erase size 0x00001000
 */

/* ----------------------------------------------------------------------------------------
 * CAN-UDS Variable initialization
 * ---------------------------------------------------------------------------------------- */
void can_uds_init(void)
{
    memset(&uds_load, 0, sizeof(UDS_LOAD_STR));
}

/* ----------------------------------------------------------------------------------------
 * UDS Duration time up
 * ---------------------------------------------------------------------------------------- */
void uds_timeup(void)
{
    uds_diag_session    = 0; // Session control 
    uds_security_access = 0; // Security access 
    can_uds_init();
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x10 Diagnostic Session Control
 * ---------------------------------------------------------------------------------------- */
int uds_sid_10(unsigned char *req, int sz, unsigned char *res, int *len)
{
    // Session code 01: Default / 02: Programming / 03: Extended dialog 
    switch (req[1] & 0x3F) {
    case 1: // Default session 
        uds_diag_session = 1;
        break;
    case 2: // ECU programming session 
        uds_diag_session = 2;
        break;
    case 3: // ECU extended diagnostic session 
        uds_diag_session = 3;
        break;
    default:
        return UDS_EC_SFNS;
    }
    res[0] = req[0] | UDS_RES_SID;
    res[1] = req[1];
    *len   = 2;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x11 ECU Reset
 * ---------------------------------------------------------------------------------------- */
int uds_sid_11(unsigned char *req, int sz, unsigned char *res, int *len)
{
    if (uds_diag_session < 2) { // Session low 
        return UDS_EC_GR; // General rejection 
    }
    // Session code 01: Hard / 02: KeyOnOff / 03: Soft / 04: Enable high-speed shutdown / 05: Disable high-speed shutdown 
    switch (req[1] & 0x3F) {
    case 1: // Hard reset 
        uds_reset_request = 1; // Hard reset 
        break;
    case 2: // Key-On/Off 
        uds_reset_request = 2; // Key reset 
        break;
    case 3: // Soft reset 
        uds_reset_request = 3; // Soft reset 
        break;
    default:
        return UDS_EC_SFNS;
    }
    res[0] = req[0] | UDS_RES_SID;
    res[1] = req[1];
    *len   = 2;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x27 Security Access
 * ---------------------------------------------------------------------------------------- */
int uds_sid_27(unsigned char *req, int sz, unsigned char *res, int *len)
{
    if (uds_diag_session < 2) { // Session low 
        return UDS_EC_GR;       // General rejection 
    }
    // Session code 01: Seed request / 02: Key transmission / 03 ~ Odd: Seed request / 04 ~ Even: Key transmission 
    switch (req[1] & 0x3F) {
    case 1: // Seed request 
        if (uds_security_access == 0) {
            res[2] = 0x12;
            res[3] = 0x34;
        } else {
            res[2] = 0x00;
            res[3] = 0x00;
        }
        *len = 4;
        break;
    case 2: // Key setting 
        if (
            req[2] == 0x17 &&
            req[3] == 0xC0 &&
            uds_security_access == 0
        ) { // 0x17C0 Release 
            uds_security_access = 1; // Unlocked 
            *len = 2;
        } else { // Key mismatch 
            uds_security_access = 0; // Locked 
            return UDS_EC_IK;
        }
        break;
    case 3: // Seed request 
        if (uds_security_access == 1) {
            res[2] = 0x34;
            res[3] = 0x56;
            *len   = 4;
        } else {
            return UDS_EC_SAD;
        }
        break;
    case 4: // Key setting 
        if (
            req[2] == 0x17 && 
            req[3] == 0xC1 && 
            uds_security_access == 1
        ) { // 0x17C1 Release 
            uds_security_access = 2; // Unlocked 
            *len = 2;
        } else { // Key mismatch 
            return UDS_EC_IK;
        }
        break;
    case 5: // Seed request 
        if (uds_security_access == 2) {
            res[2] = 0x56;
            res[3] = 0x78;
            *len   = 4;
        } else {
            return UDS_EC_SAD;
        }
        break;
    case 6: // Key setting 
        if (
            req[2] == 0x17 &&
            req[3] == 0xC2 &&
            uds_security_access == 2
        ) { // 0x17C2 Release 
            uds_security_access = 3; // Unlocked 
            *len = 2;
        } else { // Key mismatch 
            return UDS_EC_IK;
        }
        break;
    default:
        uds_security_access = 0;    // Locked 
        return UDS_EC_SFNS;
    }
    res[0] = req[0] | UDS_RES_SID;
    res[1] = req[1];
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x3E Tester Present
 * ---------------------------------------------------------------------------------------- */
int uds_sid_3e(unsigned char *req, int sz, unsigned char *res, int *len)
{
    // Maintain while the tester is connected 
    res[0] = req[0] | UDS_RES_SID;
    *len   = 1;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x22 Read Data By Identifier
 * ---------------------------------------------------------------------------------------- */
extern const char def_ecu_corp[]; // 16] = "PASTA"; 
extern const char def_ecu_name[]; // 16] = "CAN2ECU"; 
extern const char def_ecu_vars[]; // 16] = "Ver1.3.0"; 
extern const char def_ecu_date[]; // 16] = __DATE__; 
extern const char def_ecu_time[]; // 16] = __TIME__; 
int uds_sid_22(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int             i, k, b;
    unsigned long   d;

    res[0] = req[0] | UDS_RES_SID;
    res[1] = req[1]; // CID 
    res[2] = req[2]; // CID 
    i = 3;
    switch (req[1]) {
    default:
        return UDS_EC_SFNS;
    case 0xF1:  // ECU information 
        switch (req[2]) {
        default:
            return UDS_EC_SNS;
        case 0x00:  // Manufacturer's name 
            memcpy(&res[3], &def_ecu_corp[0], 16);
            i += 16;
            break;
        case 0x01:  // Vehicle code 
            memcpy(&res[3], &def_ecu_name[0], 16);
            i += 16;
            break;
        case 0x02:  // ECU version 
            memcpy(&res[3], &def_ecu_vars[0], 16);
            i += 16;
            break;
        case 0x03:  // F/W date 
            memcpy(&res[3], &def_ecu_date[0], 16);
            i += 16;
            break;
        case 0x04:  // F/W time 
            memcpy(&res[3], &def_ecu_time[0], 16);
            i += 16;
            break;
        }
        break;
    case 0xF2: // Memory map information 
        switch (req[2]) {
        default:
            return UDS_EC_SNS;
        case 0x00: // Routing map address 
            d = (unsigned long)&rout_map;
            b = 1;
            break;
        case 0x01: // Period / event / remote management definition variables 
            d = (unsigned long)&conf_ecu;
            b = sizeof(ECU_CYC_EVE);
            break;
        case 0x02: // ECU input / output checklist 
            d   = (unsigned long)&ext_list;
            b   = sizeof(EXTERNUL_IO);
            break;
        case 0x03: // CAN-ID -> EX-I/O-ID Conversion table 
            d   = (unsigned long)&can_to_exio;
            b   = 1;
            break;
        }
        res[3] = (d >> 16);  // Address H 
        res[4] = (d >> 8);   // Address M 
        res[5] = (d & 0xFF); // Address L 
        res[6] = (b & 0xFF); // Address size (byte) 
        i      += 4;
        break;
    case 0xF3: // Parameter access 
        k = ((int)req[3] << 8) | ((int)req[4] & 0xFF);
        res[3] = req[3];
        res[4] = req[4];
        i += 2;
        switch (req[2]) {
        default:
            return UDS_EC_SNS;
        case 0x00: // Routing map 
            if (k >= CAN_ID_MAX) {
                return UDS_EC_ROOR;
            }
            res[5] = rout_map.ID[k].BYTE;
            i++;
            break;
        case 0x01: // Period / event / remote management definition variables 
            if (k >= MESSAGE_MAX) {
                return UDS_EC_ROOR;
            }
            memcpy(&res[5], &conf_ecu.LIST[k], sizeof(ECU_CYC_EVE));
            i += sizeof(ECU_CYC_EVE);
            break;
        case 0x02:  // ECU input / output checklist 
            if (k >= ECU_EXT_MAX) {
                return UDS_EC_ROOR;
            }
            memcpy(&res[5], &ext_list[k], sizeof(EXTERNUL_IO));
            i += sizeof(EXTERNUL_IO);
            break;
        case 0x03:  // CAN-ID -> EX-I/O-ID Conversion table 
            if (k >= CAN_ID_MAX) {
                return UDS_EC_ROOR;
            }
            res[5] = can_to_exio[k];
            i++;
            break;
        }
        break;
    case 0xF5:  // Data flash operation 
        switch (req[2]) {
        default:
            return UDS_EC_SNS;
        case 0x00:  // Check writing status 
            k = ecu_data_check();
            res[3] = (unsigned char)(k >> 8);
            res[4] = (unsigned char)(k & 0xFF);
            i += 2;
            break;
        }
        break;
    }
    *len = i;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x23 Read Memory By Address
 * ---------------------------------------------------------------------------------------- */
int uds_sid_23(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int             z;
    unsigned char * p;

    p = (unsigned char *)(((int)req[1] << 16) | ((int)req[2] << 8) | ((int)req[3] & 0xFF));
    z = (int)req[4];
    if (z > 64) { // Batch read over 
        return UDS_EC_IML_IF;
    }
    res[0] = req[0] | UDS_RES_SID;
    memcpy(&res[1], p, z);
    *len = z + 1;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x2E Write Data By Identifier
 * ---------------------------------------------------------------------------------------- */
int uds_sid_2e(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int             i, k;
    unsigned long   d;

    res[0] = req[0] | UDS_RES_SID;
    res[1] = req[1]; // CID 
    res[2] = req[2]; // CID 
    i      = 3;
    switch (req[1]) {
    default:
        return UDS_EC_SFNS;
    case 0xF3:  // Parameter access 
        if (uds_diag_session < 2) {    // Session low 
            return UDS_EC_GR;          // General rejection 
        }
        if (uds_security_access < 1) { // Unauthorized 
            return UDS_EC_SAD;         // Security denied 
        }
        k = ((int)req[3] << 8) | ((int)req[4] & 0xFF);
        res[3] = req[3];
        res[4] = req[4];
        i += 2;
        switch (req[2]) {
        default:
            return UDS_EC_SNS;
        case 0x00:  // Routing map 
            if (k >= CAN_ID_MAX) {
                return UDS_EC_ROOR;
            }
            rout_map.ID[k].BYTE = req[5];
            res[5] = rout_map.ID[k].BYTE;
            i++;
            break;
        case 0x01:  // Period / event / remote management definition variables 
            if (k >= MESSAGE_MAX) {
                return UDS_EC_ROOR;
            }
            memcpy(&conf_ecu.LIST[k], &req[5], sizeof(ECU_CYC_EVE));
            memcpy(&res[5], &conf_ecu.LIST[k], sizeof(ECU_CYC_EVE));
            i += sizeof(ECU_CYC_EVE);
            break;
        case 0x02:  // ECU input / output checklist 
            if (k >= ECU_EXT_MAX) {
                return UDS_EC_ROOR;
            }
            memcpy(&ext_list[k], &req[5], sizeof(EXTERNUL_IO));
            memcpy(&res[5], &ext_list[k], sizeof(EXTERNUL_IO));
            i += sizeof(EXTERNUL_IO);
            break;
        case 0x03:  // CAN-ID -> EX-I/O-ID Conversion table 
            if (k >= CAN_ID_MAX) {
                return UDS_EC_ROOR;
            }
            can_to_exio[k] = req[5];
            res[5] = can_to_exio[k];
            i++;
            break;
        }
        break;
    case 0xF5: // Data flash operation 
        switch (req[2]) {
        default:
            return UDS_EC_SNS;
        case 0x01: // Save 
            k = ecu_data_write();
            res[3] = (unsigned char)(k >> 8);
            res[4] = (unsigned char)(k & 0xFF);
            i += 2;
            break;
        case 0x02: // Erase 
            k = ecu_data_erase();
            res[3] = (unsigned char)(k >> 8);
            res[4] = (unsigned char)(k & 0xFF);
            i += 2;
            break;
        }
        break;
    }
    *len = i;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x3D Write Memory By Address
 * ---------------------------------------------------------------------------------------- */
int uds_sid_3d(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int i;
    int z;
    unsigned char *p, *r;

    if (uds_diag_session < 2) {    // Session low 
        return UDS_EC_GR;          // General rejection 
    }
    if (uds_security_access < 1) { // Unauthorized 
        return UDS_EC_SAD;         // Security denied 
    }
    p   = (unsigned char *)(((int)req[1] << 16) | ((int)req[2] << 8) | ((int)req[3] & 0xFF));
    z   = (int)req[4];
    if (z > 64) { // Batch write over 
        return UDS_EC_IML_IF;
    }
    memcpy(p, &req[5], z);
    memcpy(res, req, 5);
    res[0] |= UDS_RES_SID;
    *len = 5;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x34 Request Download
 * ---------------------------------------------------------------------------------------- */
int uds_sid_34(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int i, j, sb, eb, bs;
    int siz;
    unsigned long adr;

    if (uds_diag_session < 2) { // Session low 
        return UDS_EC_GR;       // General rejection 
    }
    if (uds_security_access == 0) { // Lock status 
        return UDS_EC_UDNA;
    }
    if (uds_load.MODE) {  // Running error 
        return UDS_EC_BRR;
    }
    if (req[1] != 0x00) { // No support for compression 
        return UDS_EC_SFNS;
    }
    switch (req[2]) { // Format 
    case 0x44:
        for (adr = 0, i = 3; i < 7; i++) {
            adr <<= 8;
            adr |= (unsigned long)req[i] & 0xFF;
        }
        for (siz = 0, i = 7; i < 11; i++) {
            siz <<= 8;
            siz |= (int)req[i] & 0xFF;
        }
        break;
    default:
        return UDS_EC_SFNS;
    }
    if (adr >= 0x00000000ul && adr <= 0x0003FFFF) { // RAM 
        return UDS_EC_CNC;  // Range error 
    } else if (adr >= 0x00100000ul && adr <= 0x00107FFF) { // E2Data 
        if (siz > (0x00108000 - adr) || siz < 0) { // Size error 
            return UDS_EC_UDNA;
        }
        if ((adr & 0x00001FFF) == 0) { // Erase 
            i = BLOCK_DB0 + (adr >> 13); // Erase block 
            for (j = siz; j > 0; j -= 0x2000, i--) { // Erase block to be written first 
                if (
                    R_FlashDataAreaBlankCheck(
                        g_flash_BlockAddresses[i], 
                        BLANK_CHECK_ENTIRE_BLOCK
                    ) == FLASH_NOT_BLANK
                ) { // Erase 
                    while (R_FlashGetStatus() != FLASH_SUCCESS) {
                        ;
                    }
                    if (R_FlashErase(i) != FLASH_SUCCESS) {
                        return UDS_EC_GPF;
                    }
                    while (R_FlashGetStatus() != FLASH_SUCCESS) {
                        ;
                    }
                    if (
                        R_FlashDataAreaBlankCheck(
                            g_flash_BlockAddresses[i],
                            BLANK_CHECK_ENTIRE_BLOCK
                        ) != FLASH_BLANK
                    ) { // Erase failed 
                        return UDS_EC_GPF;
                    }
                }
            }
        }
    } else if (adr >= 0xFFE00000ul) { // Program Flash ROM 
        if (uds_security_access < 2 && (adr >= 0xFFF40000ul || (adr + (unsigned long)siz) >= 0xFFF40000ul)) {
            return UDS_EC_CNC;  // Range error 
        }
        if (uds_security_access < 3 && (adr >= 0xFFF80000ul || (adr + (unsigned long)siz) >= 0xFFF80000ul)) {
            return UDS_EC_CNC;  // Range error 
        }
        if (siz > (0x40000 - (adr & 0x0003FFFF)) || siz < 0) {
            return UDS_EC_UDNA; // Size error 
        }
        if ((adr & 0x0001FFFF) == 0) { // Erase 
            if (adr < 0xFFF00000ul) {  // Erase failed 64K block (data ROM area) 
                sb = BLOCK_69 - ((adr - 0xFFE00000ul) >> 16);
                eb = sb - ((siz + 0xFFFF) >> 16);
                bs = 0x10000;
                if (eb < BLOCK_53) {
                    return UDS_EC_CNC; // Range error 
                }
            } else if (adr < 0xFFF80000ul) { // 32K block (program ROM area) 
                sb = BLOCK_53 - ((adr - 0xFFF00000ul) >> 15);
                eb = sb - ((siz + 0x7FFF) >> 15);
                bs = 0x8000;
                if (eb < BLOCK_37) {
                    return UDS_EC_CNC; // Range error 
                }
            } else if (adr < 0xFFFF8000ul) { // 16K block (boot loader ROM area) 
                sb = BLOCK_37 - ((adr - 0xFFF80000ul) >> 14);
                eb = sb - ((siz + 0x3FFF) >> 14);
                bs = 0x4000;
                if (eb < BLOCK_7) {
                    return UDS_EC_CNC; // Range error 
                }
            } else { // 4K block (boot initialization ROM area) 
                sb = BLOCK_7 - ((adr - 0xFFFF8000ul) >> 12);
                eb = sb - ((siz + 0x0FFF) >> 12);
                bs = 0x1000;
                if (eb < -1) {
                    return UDS_EC_CNC;  // Range error 
                }
            }
            for (i = sb, j = siz; j > 0; j -= bs, i--) { // Erase block to be written first 
                _di();
                if (R_FlashErase(i) != FLASH_SUCCESS) {
                    _ei();
                    return UDS_EC_GPF;
                }
                _ei();
            }
        }
    } else { // Program range error 
        return UDS_EC_CNC;
    }
    // Download status 
    uds_load.ADDR   = adr;
    uds_load.SIZE   = siz;
    uds_load.MODE   = UDS_TD_DOWNLOAD;
    uds_load.BLKL   = 1 + 128;  // SID + Data[128] 
    uds_load.CNT    = 0;
    // Notify block size 
    res[0]  = req[0] | UDS_RES_SID;
    res[1]  = 0x20; // word block size record 
    res[2]  = (uds_load.BLKL >> 8);
    res[3]  = (uds_load.BLKL & 0xFF);
    *len    = 4;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x35 Request Upload
 * ---------------------------------------------------------------------------------------- */
int uds_sid_35(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int i, blk;
    int siz;
    unsigned long adr;

    if (uds_diag_session < 2) { // Session low 
        return UDS_EC_GR;       // General rejection 
    }
    if (uds_security_access == 0) { // Lock status 
        return UDS_EC_UDNA;
    }
    if (uds_load.MODE) {  // Running error 
        return UDS_EC_BRR;
    }
    if (req[1] != 0x00) { // No support for compression 
        return UDS_EC_SFNS;
    }
    switch (req[2]) {     // Format 
    case 0x44:
        for (adr = 0, i = 3; i < 7; i++) {
            adr <<= 8;
            adr |= (unsigned long)req[i] & 0xFF;
        }
        for (siz = 0, i = 7; i < 11; i++) {
            siz <<= 8;
            siz |= (int)req[i] & 0xFF;
        }
        if (sz > 12) { // Specified size 
            blk = (((int)req[11]) << 8) & 0xFF00;
            blk |= ((int)req[12]) & 0xFF;;
        } else { // F/W size 
            blk = 129;
        }
        break;
    default:
        return UDS_EC_SFNS;
    }
    if (blk > 0xFFF && blk < 6) {   // Block size error 
        return UDS_EC_UDNA;
    }
    if (siz > 0x40000 || siz < 0) { // Size error 
        return UDS_EC_UDNA;
    }
    // Download status 
    uds_load.ADDR = adr;
    uds_load.SIZE = siz;
    uds_load.MODE = UDS_TD_UPLOAD;
    uds_load.BLKL = 1 + 128;  // SID + Data[128] 
    uds_load.CNT  = 0;
    if (blk < uds_load.BLKL) {
        uds_load.BLKL = blk;
    }
    // Notify block size 
    res[0] = req[0] | UDS_RES_SID;
    res[1] = 0x20; // word record 
    res[2] = (uds_load.BLKL >> 8);
    res[3] = (uds_load.BLKL & 0xFF);
    *len   = 4;
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x36 Transfer Data
 * ---------------------------------------------------------------------------------------- */
int uds_sid_36(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int             i;
    unsigned long   adr;
    unsigned char * p;

    if (uds_diag_session < 2) { // Session low 
        return UDS_EC_GR;       // General rejection 
    }
    if (uds_security_access == 0) { // Lock status 
        return UDS_EC_UDNA;
    }
    if (uds_load.MODE == UDS_TD_DOWNLOAD) { // Downloading (Tools-> ECU) 
        if (sz > uds_load.BLKL) {           // Size over 
            uds_load.MODE = UDS_TD_NONE;    // Cancel transfer 
            return UDS_EC_IML_IF;
        }
        sz--;   // Reduce by command code byte 
        if ((uds_load.CNT + sz) > uds_load.SIZE) {
            sz = uds_load.SIZE - uds_load.CNT;
        }
        if ((uds_load.ADDR & 0xFFFC0000) == 0) { // Write to RAM 
            p = (unsigned char *)uds_load.ADDR;
            uds_load.ADDR += sz;
            uds_load.CNT  += sz;
            memcpy(p, &req[1], sz);
        } else if ((uds_load.ADDR & 0xFFFF8000ul) == 0x00100000ul) { // E2Data 
            i = sz;
            if ((i & (DF_PROGRAM_SIZE_SMALL - 1)) != 0) { // Fill with 00 if the minimum write byte has not been reached 
                for (; (i & (DF_PROGRAM_SIZE_SMALL - 1)) != 0; i++) {
                    req[1 + i] = 0;
                }
            }
            if (R_FlashWrite(uds_load.ADDR, (int)&req[1], i) != FLASH_SUCCESS) { // Write failed 
                uds_load.MODE = UDS_TD_NONE;
                return UDS_EC_GPF;
            }
            uds_load.ADDR += sz;
            uds_load.CNT  += sz;
        } else if (uds_load.ADDR >= 0xFFE00000ul) { // Program Flash ROM 
            i = sz;
            if (i < ROM_PROGRAM_SIZE) { // Fill in the program unit shortfall with FF 
                for (; i < ROM_PROGRAM_SIZE; i++) {
                    req[1 + i] = 0xFF;
                }
            }
            _di();
            if (R_FlashWrite(uds_load.ADDR, (int)&req[1], i) != FLASH_SUCCESS) { // Write failed 
                _ei();
                uds_load.MODE = UDS_TD_NONE;
                return UDS_EC_GPF;
            }
            _ei();
            uds_load.ADDR   += sz;
            uds_load.CNT    += sz;
        }
        if (uds_load.CNT >= uds_load.SIZE) { // Download completed 
            uds_load.MODE = UDS_TD_NONE;
        }
        i = (int)(uds_load.SIZE - uds_load.CNT);
        // Normal response (notify remaining bytes) 
        res[0] = req[0] | UDS_RES_SID;
        res[1] = 0x04; // long counter record 
        res[2] = (i >> 24);
        res[3] = (i >> 16);
        res[4] = (i >> 8);
        res[5] = (i & 0xFF);
        *len   = 6;
        return UDS_EC_NONE;
    } else if (uds_load.MODE == UDS_TD_UPLOAD) { // Uploading (ECU Tool)
        i      = uds_load.BLKL; // Size adjustment 
        res[0] = req[0] | UDS_RES_SID;
        p      = (unsigned char *)uds_load.ADDR;
        sz     = (int)(uds_load.SIZE - uds_load.CNT) + 1;
        if (sz > uds_load.BLKL) {
            sz = uds_load.BLKL;
        }
        for (i = 1; i < sz; i++) {
            res[i] = *p++;
            uds_load.ADDR++;
            uds_load.CNT++;
        }
        *len = i;
        return UDS_EC_NONE;
    }
    return UDS_EC_RSE; // Procedure error 
}

/* ----------------------------------------------------------------------------------------
 * UDS 0x37 Request Transfer Exit
 * ---------------------------------------------------------------------------------------- */
int uds_sid_37(unsigned char *req, int sz, unsigned char *res, int *len)
{
    int i;
    if (uds_diag_session < 2) { // Session low 
        return UDS_EC_GR; // General rejection 
    }
    if (uds_security_access == 0) { // Lock status 
        return UDS_EC_UDNA;
    }
    if (uds_load.MODE != UDS_TD_NONE) {
        if (uds_load.MODE == UDS_TD_DOWNLOAD) { // Downloading (Tools-> ECU) 
            if (uds_load.ADDR >= 0xFFF00000ul && uds_load.ADDR < 0xFFF20000ul) { // Erase ROM to interrupt user firmware area 
                R_FlashErase(BLOCK_53);
            }
        }
        uds_load.MODE = UDS_TD_NONE;
        i = (int)(uds_load.SIZE - uds_load.CNT);
        // Report remaining bytes at break 
        res[0] = req[0] | UDS_RES_SID;
        res[1] = 0x04; // long counter record 
        res[2] = (i >> 24);
        res[3] = (i >> 16);
        res[4] = (i >> 8);
        res[5] = (i & 0xFF);
        *len   = 6;
    } else { // End 
        res[0] = req[0] | UDS_RES_SID;
        res[1] = 0x00;
        *len   = 2;
    }
    return UDS_EC_NONE;
}

/* ----------------------------------------------------------------------------------------
 * UDS processing
 * ---------------------------------------------------------------------------------------- */
int uds_job(unsigned char *msg, int len, unsigned char *res)    //int ch, int id, void *frame) 
{
    int size = 0;
    int ercd = UDS_EC_NONE;
    // Execute service 
    switch (msg[0]) {
    default: // Service not supported 
        res[0] = UDS_ERR_SID;
        res[1] = msg[0];
        res[2] = UDS_EC_SNS;
        return 3;
    case 0x10: // Diagnostic Session Control 
        ercd = uds_sid_10(msg, len, res, &size);
        break;
    case 0x11: // ECU Reset 
        ercd = uds_sid_11(msg, len, res, &size);
        break;
    case 0x27: // Security Access 
        ercd = uds_sid_27(msg, len, res, &size);
        break;
    case 0x3E: // Tester Present 
        ercd = uds_sid_3e(msg, len, res, &size);
        break;
    case 0x22: // Read Data By Identifier 
        ercd = uds_sid_22(msg, len, res, &size);
        break;
    case 0x23: // Read Memory By Address 
        ercd = uds_sid_23(msg, len, res, &size);
        break;
    case 0x2E: // Write Data By Identifier 
        ercd = uds_sid_2e(msg, len, res, &size);
        break;
    case 0x3D: // Write Memory By Address 
        ercd = uds_sid_3d(msg, len, res, &size);
        break;
    case 0x34: // Request Download 
        ercd = uds_sid_34(msg, len, res, &size);
        break;
    case 0x35: // Request Upload 
        ercd = uds_sid_35(msg, len, res, &size);
        break;
    case 0x36: // Transfer Data 
        ercd = uds_sid_36(msg, len, res, &size);
        break;
    case 0x37: // Request Transfer Exit 
        ercd = uds_sid_37(msg, len, res, &size);
        break;
    }
    if (ercd != UDS_EC_NONE) { // With error 
        res[0] = UDS_ERR_SID;
        res[1] = msg[0];
        res[2] = ercd;
        return 3;
    }
    after_call(DTC_TIMER_ID, 10000, uds_timeup); // Connection maintaining 10 seconds timer 
    return size;
}

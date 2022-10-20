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

// #include "uds.h"   // CAN-UDS definition 

#ifndef     __CAN_UDS_PROTOCOL__
#define     __CAN_UDS_PROTOCOL__

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
 * Code definition
 * ---------------------------------------------------------------------------------------- */
#define UDS_ERR_SID 0x7F // Error service ID 
#define UDS_RES_SID 0x40 // Normal response addition ID 

/* ----------------------------------------------------------------------------------------
 * Error response code (attached to SID = 0x7F)
 * ---------------------------------------------------------------------------------------- */
#define UDS_EC_NONE   0x00 // No error 
#define UDS_EC_GR     0x10 // General rejection 
#define UDS_EC_SNS    0x11 // Unsupported services 
#define UDS_EC_SFNS   0x12 // Subfeature not supported 
#define UDS_EC_IML_IF 0x13 // Incorrect message length or invalid format 
#define UDS_EC_RTL    0x14 // Response too long 
#define UDS_EC_BRR    0x21 // Busy repeat request 
#define UDS_EC_CNC    0x22 // Condition not correct 
#define UDS_EC_RSE    0x24 // Request sequence error 
#define UDS_EC_NRFSNC 0x25 // No response from subnet component 
#define UDS_EC_FPERA  0x26 // Failure prevents execution of the requested action 
#define UDS_EC_ROOR   0x31 // Request out of range 
#define UDS_EC_SAD    0x33 // Security access denied 
#define UDS_EC_IK     0x35 // Invalid key 
#define UDS_EC_ENOA   0x36 // Exceeded number of attempts 
#define UDS_EC_RTDNE  0x37 // Travel time has not expired 
#define UDS_EC_UDNA   0x70 // Unable to upload / download 
#define UDS_EC_TDS    0x71 // Pause transfer data 
#define UDS_EC_GPF    0x72 // General programming failure 
#define UDS_EC_WBSC   0x73 // Wrong block sequence counter 
#define UDS_EC_RCR    0x78 // Request received correctly, response pending 
#define UDS_EC_SFNSAS 0x7E // Subsession not supported in active session 
#define UDS_EC_SNSAS  0x7F // Service not supported for active sessions 

/* ----------------------------------------------------------------------------------------
 * Variable definition
 * ---------------------------------------------------------------------------------------- */
extern int uds_diag_session;       // Session control 
extern int uds_p2_can_server_max;  // P2 Time 
extern int uds_p2e_can_server_max; // P2E Time 
extern int uds_reset_request;      // Reset request 
extern int uds_security_access;    // Security access 

// Download / upload management structure 
#define UDS_TD_NONE     0 // Waiting for upload / download request 
#define UDS_TD_DOWNLOAD 1 // Downloading 
#define UDS_TD_UPLOAD   2 // Uploading 
typedef struct  __uds_load_control_str__ {
    int           MODE; // Transfer mode 
    unsigned long ADDR; // Address 
    int           SIZE; // Size 
    int           BLKL; // Block size 
    int           CNT;  // Transfer counter 
}   UDS_LOAD_STR;

extern UDS_LOAD_STR uds_load; // Download / upload management 

#define UDS_BUFFER_MAX (128+1) // Maximum data size that can be sent and received by UDS 

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
extern void can_uds_init(void);
/* ----------------------------------------------------------------------------------------
 * UDS 0x10 Diagnostic Session Control
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_10(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x11 ECU Reset
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_11(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x27 Security Access
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_27(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x3E Tester Present
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_3e(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x22 Read Data By Identifier
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_22(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x23 Read Memory By Address
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_23(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x2E Write Data By Identifier
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_2e(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x3D Write Memory By Address
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_3d(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x34 Request Download
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_34(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x35 Request Upload
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_35(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x36 Transfer Data
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_36(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS 0x37 Request Transfer Exit
 * ---------------------------------------------------------------------------------------- */
extern int uds_sid_37(unsigned char *req, int sz, unsigned char *res, int *len);
/* ----------------------------------------------------------------------------------------
 * UDS processing
 * ---------------------------------------------------------------------------------------- */
extern int uds_job(unsigned char *msg, int len, unsigned char *res);

#endif //__CAN_UDS_PROTOCOL__

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
 * OBD2-CAN Protocol processing
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2017/02/13 Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ________________________________________________________________________________________
 */
#ifndef __CAN_OBD2_PROTOCOL__
#define __CAN_OBD2_PROTOCOL__

/*
 *    OBD2 processing overview
 *
 *    OBD2 CAN frame definition
 *
 *    Broadcast
 *    CAN-ID 0x7DF
 *
 *                    ECU query received (0x7DF) * All ECUs are processed
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        PID-type / Byte |           0           |           1           |           2           |           3           |           4           |           5           |           6           |           7           |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        SAE standard    |Number of additional   |Mode                   |PID code               |not used                                                                                                               |
 *                        | data bytes:2          |01=show current data   |(e.g.:05=Engine coolant|(may be 55h)                                                                                                           |
 *                        |                       |02=freeze frame        | temperature)          |                                                                                                                       |
 *                        |                       |etc.                   |                       |                                                                                                                       |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        Vehicle speific |Number of additional   |Custom mode:           |PID code                                       |not used                                                                                       |
 *                        | data bytes:3          |(e.g.:22=enhanced data |(e.g.:4980h)                                   |(may be 00h or 55h)                                                                            |
 *                        |                       |                       |                                               |                                                                                               |
 *                        |                       |                       |                                               |                                                                                               |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *
 *
 *    Multiframe
 *    CAN-ID 0x7E0 Request       to ECU0 (powertrain system)
 *           0x7E1 Request      for ECU1 (body system)
 *           0x7E2 Requirements for ECU2 (chassis system)
 *           0x7E3 Request       to ECU3 (CGW-DTC)
 *
 *                    ECU query reception (0x7E0 to 7E7) *OBD2 request for individual ECU
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        PID-type / Byte |           0           |           1           |           2           |           3           |           4           |           5           |           6           |           7           |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        SAE standard    |Number of additional   |Mode                   |PID code               |not used                                                                                                               |
 *                        | data bytes:2          |01=show current data   |(e.g.:05=Engine coolant|(may be 55h)                                                                                                           |
 *                        |                       |02=freeze frame        | temperature)          |                                                                                                                       |
 *                        |                       |etc.                   |                       |                                                                                                                       |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        Vehicle speific |Number of additional   |Custom mode:           |PID code                                       |not used                                                                                       |
 *                        | data bytes:3          |(e.g.:22=enhanced data |(e.g.:4980h)                                   |(may be 00h or 55h)                                                                            |
 *                        |                       |                       |                                               |                                                                                               |
 *                        |                       |                       |                                               |                                                                                               |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *
 *    CAN-ID 0x7E8 Response from ECU0 (powertrain system)
 *           0x7E9 Response from ECU1 (body system)
 *           0x7EA Response from ECU2 (chassis system)
 *           0x7EB Response from ECU3 (CGW-DTC)
 *
 *                    ECU query transmission (0x7E8 to 7EF)
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        PID-type / Byte |           0           |           1           |           2           |           3           |           4           |           5           |           6           |           7           |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        SAE standard    |Number of additional   |Custom mode (+40h)     |PID code               |value of the           |value.                 |value.                 |value.                 |not used               |
 *                        | data bytes:3 to 6     |41=show current data   |(e.g.:05=Engine coolant|specified parameter.   |byte 1                 |byte 2                 |byte 3                 |(may be 00h or 55h)    |
 *                        |                       |42=freeze frame        |(temperature)          |byte 0                 |(optional)             |(optional)             |(optional)             |                       |
 *                        |                       |etc.                   |                       |                       |                       |                       |                       |                       |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        Vehicle speific |Number of additional   |Custom mode (+40h)     |PID code                                       |value of the           |value.                 |value.                 |value.                 |
 *                        | data bytes:4 to 7     |(e.g.:62h=response to  |(e.g.:4980h)                                   |specified parameter.   |byte 1                 |byte 2                 |byte 3                 |
 *                        |                       | mode 22h request)     |                                               |byte 0                 |(optional)             |(optional)             |(optional)             |
 *                        |                       |                       |                                               |                       |                       |                       |                       |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *        Vehicle speific |Number of additional   |7Fh this a general     |Custom mode:           |31h                    |not used                                                                                       |
 *                        | data bytes:3          |response usually       |(e.g.:22h=enhanced     |                       |(may be 00h)                                                                                   |
 *                        |                       |indicating the module  |diagnostic data by PID,|                       |                                                                                               |
 *                        |                       | doesn't recognize the |21h=enhanced data by   |                       |                                                                                               |
 *                        |                       | request.              |offset                 |                       |                                                                                               |
 *                        +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *
 *
 */

/* ----------------------------------------------------------------------------------------
 * OBD2 packet union definition
 * ---------------------------------------------------------------------------------------- */
typedef union __obd2_query_frame__ {
    unsigned char BYTE[8];
    struct {
        unsigned char   MODE;  // Mode 
        unsigned char   PID;   // Parameter number 
        unsigned char   NU[6]; // Unused [55h] 
    } SAE_OBD;
    struct {
        unsigned char   MODE;  // Mode (0 to A) 
        unsigned char   PIDH;  // Upper 8 bits of parameter number 
        unsigned char   PIDL;  // Lower 8 bits of parameter number 
        unsigned char   NU[5]; // Unused [55h] 
    } VS_OBD;
    struct {
        unsigned char MODE;   // Mode (+40h)
        unsigned char PID;    // Parameter number
        unsigned char VAL[5]; // Parameter value
        unsigned char NU;     // Unused [00h or 55h]
    } SAE_ECU;
    struct {
        unsigned char MODE;   // Mode (+40h)
        unsigned char PIDH;   // Upper 8 bits of parameter number
        unsigned char PIDL;   // Lower 8 bits of parameter number
        unsigned char VAL[5]; // Parameter value
    } VS_ECU;
    struct {
        unsigned char MODE;   // Mode (+40h)
        unsigned char VAL[7]; // Parameter value
    } MD3_ECU;
    struct {
        unsigned char   MODE;  // Mode (+40h) 
        unsigned char   NU[7]; // Parameter value 
    } MD4_ECU;
    struct {
        unsigned char   X7F;   // Unsupported request code: Fixed [7Fh] 
        unsigned char   MODE;  // Receive mode 
        unsigned char   X31;   // Fixed [31h] 
        unsigned char   NU[5]; // Unused [00h] 
    } NOT_ECU;
} OBD2_QUERY_FRAME;

/* ----------------------------------------------------------------------------------------
 * MODE code definition
 * ---------------------------------------------------------------------------------------- */
#define SHOW_CURRENT_DATA    0x01 // Get current value 
#define SHOW_FREEZE_FDATA    0x02 // Get pause frame 
#define SHOW_STORED_DTC      0x03 // Get DTC (Diagnostic Trouble Codes) (diagnosis log) 
#define CLEAR_STORED_DTC     0x04 // Clear DTC information 
#define TEST_RESULT_NON_CAN  0x05 // Test result (does not work with CAN) 
#define TEST_RESULT_ONLY_CAN 0x06 // Test result (CAN only oxygen sensor monitoring) 
#define SHOW_PENDING_DTC     0x07 // Get unknown diagnostic trouble code information 
#define CTRL_OPERATION_SYS   0x08 // On-board system support 
#define REQUEST_VEHICLE_INFO 0x09 // Get vehicle information 
#define PERMANENT_DTC        0x0A // Persistent DTC information 

/* ----------------------------------------------------------------------------------------
 * Diagnostic trouble code (DTC) definition
 * ---------------------------------------------------------------------------------------- */
typedef union __obd2_dtc_str__ {
    unsigned char BYTE[2];
    struct {
        struct {
            unsigned char ECU : 2;   /* 1st DTC character 0:[P] Powertrain / 1:[C]
                                      * Chassis / 2:[B] Body / 3:[U] Network*/
            unsigned char   CH  : 2; // 2nd DTC character 0 to 3 
            unsigned char   CL  : 4; // 3rd DTC character 0 to F 
        } A;
        struct {
            unsigned char   CH  : 4; // 4th DTC character 00 to FF 
            unsigned char   CL  : 4; // 5th DTC character 00 to FF 
        } B;
    } BIT;
} OBD2_DTC_STR;

#define DTC_ECU_CODE_POW 0 // Powertrain diagnostic code 
#define DTC_ECU_CODE_CHA 1 // Chassis diagnostic code 
#define DTC_ECU_CODE_BDY 2 // Body diagnostic code 
#define DTC_ECU_CODE_NET 3 // Network diagnostic code 

/* ----------------------------------------------------------------------------------------
 * Variable definition
 * ---------------------------------------------------------------------------------------- */
extern OBD2_QUERY_FRAME obd2_req; // Request data 
extern OBD2_QUERY_FRAME obd2_ret; // Response data 

extern int obd2_ret_counter;
/* ----------------------------------------------------------------------------------------
 * MODE1 processing  0x0C, 0x0D, 0x1C, 0x2F, 0x31, 0x49, 0x51
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode1(int len);
/* ----------------------------------------------------------------------------------------
 * MODE2 processing (DTC record reply request)
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode2(int len);
/* ----------------------------------------------------------------------------------------
 * MODE3 processing (Reply saved DTC record)
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode3(int len);
/* ----------------------------------------------------------------------------------------
 * MODE4 processing (Delete DTC record)
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode4(int len);
/* ----------------------------------------------------------------------------------------
 * MODE5 processing
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode5(int len);
/* ----------------------------------------------------------------------------------------
 * MODE6 Test result (CAN only oxygen sensor monitoring)
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode6(int len);
/* ----------------------------------------------------------------------------------------
 * MODE7 Obtain unknown diagnostic trouble code information
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode7(int len);
/* ----------------------------------------------------------------------------------------
 * MODE8 On-board system support
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode8(int len);
/* ----------------------------------------------------------------------------------------
 * MODE9 processing
 * ---------------------------------------------------------------------------------------- */
extern int obd2_mode9(int len);
/* ----------------------------------------------------------------------------------------
 * MODE10 Persistent DTC information
 * ---------------------------------------------------------------------------------------- */
extern int obd2_modeA(int len);
/* ----------------------------------------------------------------------------------------
 * OBD2 processing
 * ---------------------------------------------------------------------------------------- */
extern int obd2_job(unsigned char *msg, int len, unsigned char *res);

#endif //__CAN_OBD2_PROTOCOL__

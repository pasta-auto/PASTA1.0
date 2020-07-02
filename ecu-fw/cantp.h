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
 * CAN-TP transport layer protocol processing
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2020/03/15  Buffer size modification
 *                 Change CAN-TP buffer size to 4096 bytes, change segment definition and stack size
 * 08/01/2017  Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ________________________________________________________________________________________
 */

#ifndef     __CAN_TRANSE_PORT_PROTOCOL__
#define     __CAN_TRANSE_PORT_PROTOCOL__

/*
 *  Overview of CAN-TP
 *
 *  Link single or multiple packets of TP -> Pass to upper layer -> Receive from upper layer -> Decompose into single or multiple packets and transfer
 *
 *  CAN-TP frame definition
 *
 *  Application
 *  Broadcast               CAN-ID  0x7DF
 *  For designated ECU      CAN-ID  0x7E0 to 0x7E7
 *  Designated ECU response CAN-ID  0x7E8 to 0x7EF
 *
 *  Request frame
 *
 *                       +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Byte             |          0            |          1            |          2            |          3            |          4            |          5            |          6            |          7            |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Bit(LE)          |  7..4     | 3..0      |        15..8          |        23..16         |        31..24         |                       |                       |                       |                       |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Single           |  Type 0   | Size 0..7 |       Data A          |       Data B          |        Data C         |        Data D         |        Data E         |        Data F         |        Data G         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      First            |  Type 1   | Size 8..4095                      |       Data A          |        Data B         |        Data C         |        Data D         |        Data E         |        Data F         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Consecutive      |  Type 2   |Index 0..15|       Data A          |       Data B          |        Data C         |        Data D         |        Data E         |        Data F         |        Data G         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Flow             |  Type 3   |FCflag0,1,2|       Block Size      |          ST           |                       |                       |                       |                       |                       |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Reserve          |  4..15    | 0..15     |       Data A          |        Data B         |        Data C         |        Data D         |        Data E         |        Data F         |        Data G         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *
 *  Response (Request CAN-ID + 0x008)
 *                       +-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Byte             |          0            |          1            |          2            |          3            |          4            |          5            |          6            |          7            |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Bit(LE)          |  7..4     |  3..0     |        15..8          |        23..16         |        31..24         |                       |                       |                       |                       |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Single           |  Type 0   | Size 0..7 |       Data A          |        Data B         |        Data C         |        Data D         |        Data E         |        Data F         |        Data G         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      First            |  Type 1   | Size 8..4095                      |        Data A         |        Data B         |        Data C         |        Data D         |        Data E         |        Data F         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Consecutive      |  Type 2   |Index 0..15|       Data A          |        Data B         |        Data C         |        Data D         |        Data E         |        Data F         |        Data G         |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 *      Flow             |  Type 3   |FCflag0,1,2|      Block Size       |          ST           |                       |                       |                       |                       |                       |
 *                       +-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 */

/* ----------------------------------------------------------------------------------------
 * TYPE code definition
 * ---------------------------------------------------------------------------------------- */
#define CAN_TP_SINGLE 0x00 // Single frame 
#define CAN_TP_FIRST  0x01 // Start of multi-frame 
#define CAN_TP_CONT   0x02 // Continue multi-frame 
#define CAN_TP_FLOW   0x03 // Flow control 

/* ----------------------------------------------------------------------------------------
 * Mode flag bit synthesis
 * ---------------------------------------------------------------------------------------- */
#define CANTP_MODE_RECV 1  // TP operation mode (Receiving) 
#define CANTP_MODE_SEND 2  // TP operation mode (Transmitting) 
#define CANTP_MODE_WFL  4  // TP operation mode (Wait for flow response) 
#define CANTP_MODE_WTE  8  // TP operation mode (Waiting for transmission completion) 
#define CANTP_MODE_WTU  16 // TP operation mode (Waiting for time up) 

/* ----------------------------------------------------------------------------------------
 * Buffer size
 * ---------------------------------------------------------------------------------------- */
#define     CAN_TP_BUF_SIZE    256
#define     CAN_TP_MAX_BUF     4096 

// Multi-frame buffer type definition 
typedef struct  __can_tp_buffer__ {
    int RPOS; // Read position 
    int WPOS; // Write position 
    unsigned char BUF[CAN_TP_MAX_BUF]; // Buffer 
}   CAN_TP_BUF;

// Single frame type definition 
typedef union   __can_tp_single__ {
    unsigned long  L[2];
    unsigned short W[4];
    unsigned char  B[8];
    struct  {
        union   {
            unsigned char BYTE;
            struct  {
                unsigned char CODE : 4; // Frame type (0) 
                unsigned char SIZE : 4; // Number of data bytes (0 to 7) 
            }   HEAD;
        }   PCI;
        unsigned char DATA[7]; // Data 
    }   FRAME;
}   TP_FRM_SINGL;

// Multi-frame head type definition 
typedef union   __can_tp_first__ {
    unsigned long  L[2];
    unsigned short W[4];
    unsigned char  B[8];
    struct  {
        union   {
            unsigned char BYTE;
            struct  {
                unsigned char CODE : 4; // Frame type (1) 
                unsigned char SIZE : 4; // Upper 4 bits of data bytes (0 to F) 
            }   HEAD;
        }   PCI;
        unsigned char SIZEL;   // Lower 8 bits of data bytes (00 to FF) 
        unsigned char DATA[6]; // Data 
    }   FRAME;
}   TP_FRM_FIRST;

// Multi-frame subsequent type definition 
typedef union   __can_tp_consec__ {
    unsigned long  L[2];
    unsigned short W[4];
    unsigned char  B[8];
    struct  {
        union   {
            unsigned char BYTE;
            struct  {
                unsigned char CODE  : 4; // Frame type (2) 
                unsigned char INDEX : 4; // Frame index number (0 to 15) 
            }   HEAD;
        }   PCI;
        unsigned char DATA[7]; // Data 
    }   FRAME;
}   TP_FRM_CONSEC;

// Multi-frame flow control type definition 
typedef union   __can_tp_flow__ {
    unsigned long  L[2];
    unsigned short W[4];
    unsigned char  B[8];
    struct  {
        union   {
            unsigned char BYTE;
            struct  {
                unsigned char CODE : 4; // Frame type (3) 
                unsigned char FC   : 4; // Flow control (0=transmission allowed, 1=WAIT, 2=overflow) 
            }   HEAD;
        }   PCI;
        unsigned char BS;      // Block size (0=no reception delay, 1 to number of receivable frames) 
        unsigned char ST;      // Frame division time (0 to 127msec, 0xF1 to 0xF9=100 to 900msec) 
        unsigned char DATA[5]; // Invalid 
    }   FRAME;
}   TP_FRM_FLOW;
#define CANTP_FC_CTS   0 // Transmit permission 
#define CANTP_FC_WAIT  1 // Wait for transmission 
#define CANTP_FC_OVER  2 // Buffer over flow 
#define CANTP_FC_ABORT 2 // Abort 
#define CANTP_FC_INDEX 3 // Index does not match 

// Frame union definition 
typedef union   __can_tp_frame__ {
    unsigned long  L[2];
    unsigned short W[4];
    unsigned char  B[8];
    TP_FRM_SINGL   SINGLE; // Single frame 
    TP_FRM_FIRST   FIRST;  // Multi-frame head 
    TP_FRM_CONSEC  CONSEC; // Multi-frame subsequent 
    TP_FRM_FLOW    FLOW;   // Multi-frame flow control 
}   CAN_TP_FRAME;

// TP control buffer type definition 
typedef struct  __can_tp_packet__ {
    int          MODE;  // TP operation mode (0:request waiting, bit combining (1:receiving, 2:transmitting, 4:flow response waiting, 8:transmission completion waiting, 16:time-up waiting)) 
    int          TIME;  // DTC duration (0=release, 0 <ongoing) *Regularly receive continuation request and update 
    int          CH;    // CAN port number 
    int          ID;    // CAN Frame ID 
    int          INDEX; // Multi-frame index number 
    int          SIZE;  // Multi-frame data size 
    int          BC;    // Block counter 
    int          FC;    // Flow control (0=transmission allowed, 1=WAIT, 2=overflow) 
    int          BS;    // Block size (0=no reception delay, 1 to number of receivable frames) 
    int          ST;    // Frame division time (0 to 127msec, 0xF1 to 0xF9=100 to 900msec) 
    int          TXIF;  // Transmission completion processing request flag 
    int          TXID;  // Retain transmission CAN-ID 
    CAN_TP_FRAME RXF;   // Receive frame 
    CAN_TP_FRAME TXF;   // Transmit frame 
    CAN_TP_BUF   RXD;   // Receive buffer 
    CAN_TP_BUF   TXD;   // Transmit buffer 
}   CAN_TP_PACK;

extern CAN_TP_PACK tp_pack;         // TP control variable 

/* ----------------------------------------------------------------------------------------
 * CAN-TP variable initialization
 * ---------------------------------------------------------------------------------------- */
extern void can_tp_init(void);
/* ----------------------------------------------------------------------------------------
 * CAN-TP data stacking process
 * ---------------------------------------------------------------------------------------- */
extern int can_tp_build(unsigned char *dp, int sz);
/* ----------------------------------------------------------------------------------------
 * CAN-TP data transmission processing
 * ---------------------------------------------------------------------------------------- */
extern int can_tp_send(void);
/* ----------------------------------------------------------------------------------------
 * Continuous transmission processing
 * ---------------------------------------------------------------------------------------- */
extern void can_tp_consecutive(void);
/* ----------------------------------------------------------------------------------------
 * CAN-TP transmission completion processing
 * ---------------------------------------------------------------------------------------- */
extern void can_tp_txendreq(void);
/* ----------------------------------------------------------------------------------------
 * CAN-TP transmission completion wait release check
 * ---------------------------------------------------------------------------------------- */
extern void can_tp_txecheck(int ch, int id);
/* ----------------------------------------------------------------------------------------
 * CAN-TP processing
 * ---------------------------------------------------------------------------------------- */
extern int can_tp_job(int ch, int id, void *frame);

#endif //__CAN_TRANSE_PORT_PROTOCOL__

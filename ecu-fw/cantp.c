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
 * 08/01/2017    Start coding (by Tachibana)
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
#include "ecu.h"            //    ECU common definition 
#include "can3_spi2.h"      //    CAN3 definition 
#include "cantp.h"          //    CAN-TP definition 
#include "obd2.h"           //    CAN-OBDII definition 
#include "uds.h"            //    CAN-UDS definition 

/*
 *  Overview of CAN-TP
 *
 *  Link single or multiple packets of TP -> Pass to upper layer -> Receive from upper layer -> Decompose into single or multiple packets and transfer
 *
 *  CAN-TP frame definition
 *
 *  Application
 *  Broadcast               CAN-ID    0x7DF
 *  For designated ECU      CAN-ID    0x7E0 to 0x7E7
 *  Designated ECU response CAN-ID    0x7E8 to 0x7EF
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

CAN_TP_PACK tp_pack;         // TP control variable 

/* ----------------------------------------------------------------------------------------
 * CAN-TP variable initialization
 * ---------------------------------------------------------------------------------------- */
void can_tp_init(void)
{
    memset(&tp_pack, 0, sizeof(CAN_TP_PACK));
}

/* ----------------------------------------------------------------------------------------
 * CAN-TP data stacking process
 * ---------------------------------------------------------------------------------------- */
int can_tp_build(unsigned char *dp, int sz)
{
    int i;
    int f = 0;
    // Data accumulation 
    if (sz > 0) { // With data 
        for (i = 0; i < sz && tp_pack.RXD.WPOS < tp_pack.SIZE; i++) {
            tp_pack.RXD.BUF[tp_pack.RXD.WPOS++] = *dp++;
        }
        if (tp_pack.RXD.WPOS == tp_pack.SIZE) { // All data reception completed 
            if (tp_pack.RXD.BUF[0] < 0x10) {    // OBD2 protocol 
                f = obd2_job(tp_pack.RXD.BUF, tp_pack.RXD.WPOS, tp_pack.TXD.BUF);
            } else {  // UDS protocol 
                f = uds_job(tp_pack.RXD.BUF, tp_pack.RXD.WPOS, tp_pack.TXD.BUF);
            }
            if (f > 0) {
                tp_pack.TXD.RPOS    = 0;
                tp_pack.TXD.WPOS    = f;
            }
        }
    }
    return f;
}

/* ----------------------------------------------------------------------------------------
 * CAN-TP data transmission processing
 * ---------------------------------------------------------------------------------------- */
int can_tp_send(void)
{
    int             i, sz;
    unsigned char * dp;
    // Data accumulation 
    if (tp_pack.TXD.RPOS < tp_pack.TXD.WPOS) { // With data 
        memset(&tp_pack.TXF, 0, sizeof(CAN_TP_FRAME));
        if (tp_pack.TXD.WPOS < 8) { // Tramsmit in single frame 
            tp_pack.TXF.SINGLE.FRAME.PCI.HEAD.CODE  = CAN_TP_SINGLE;
            tp_pack.TXF.SINGLE.FRAME.PCI.HEAD.SIZE  = tp_pack.TXD.WPOS;
            tp_pack.SIZE                            = tp_pack.TXD.WPOS;
            tp_pack.INDEX                           = 1;
            tp_pack.BC                              = 0;
            tp_pack.MODE                            = CANTP_MODE_SEND;
            dp                                      = tp_pack.TXF.SINGLE.FRAME.DATA;
            sz                                      = 7;
        } else if (tp_pack.TXD.RPOS == 0) {  // Transmit in multiframe, first 
            tp_pack.TXF.FIRST.FRAME.PCI.HEAD.CODE   = CAN_TP_FIRST;
            tp_pack.TXF.FIRST.FRAME.PCI.HEAD.SIZE   = (tp_pack.TXD.WPOS >> 8) & 0x0F;
            tp_pack.TXF.FIRST.FRAME.SIZEL           = tp_pack.TXD.WPOS & 0xFF;
            tp_pack.SIZE                            = tp_pack.TXD.WPOS;
            tp_pack.BC                              = 0;
            tp_pack.INDEX                           = 1;
            tp_pack.MODE                            = CANTP_MODE_SEND | CANTP_MODE_WFL;  // Waiting for flow 
            dp                                      = tp_pack.TXF.FIRST.FRAME.DATA;
            sz                                      = 6;
        } else {  // Transmit in multiframe, continuous 
            tp_pack.TXF.CONSEC.FRAME.PCI.HEAD.CODE  = CAN_TP_CONT;
            tp_pack.TXF.CONSEC.FRAME.PCI.HEAD.INDEX = tp_pack.INDEX++;
            tp_pack.INDEX                           &= 15;
            tp_pack.BC++;
            tp_pack.MODE = CANTP_MODE_SEND | CANTP_MODE_WTE;   // Waiting for transmission completion 
            if (tp_pack.BC >= tp_pack.BS && tp_pack.BS > 0) {  // Reach continuous block count 
                tp_pack.BC      = 0;
                tp_pack.FC      = CANTP_FC_WAIT;
                tp_pack.MODE    |= CANTP_MODE_WFL;  // Waiting for flow 
            }
            dp  = tp_pack.TXF.CONSEC.FRAME.DATA;
            sz  = tp_pack.TXD.WPOS - tp_pack.TXD.RPOS;
            if (sz > 7) {
                sz = 7;
            }
        }
        // Data copy 
        for (i = 0; i < sz && tp_pack.TXD.RPOS < tp_pack.TXD.WPOS; i++) {
            *dp++ = tp_pack.TXD.BUF[tp_pack.TXD.RPOS++];
        }
        if (tp_pack.TXD.RPOS == tp_pack.SIZE) {  // All data transmission completed 
            tp_pack.MODE = 0;
        }
        return 1;    // With transmission 
    }
    return 0;    // No transmission 
}

/* ----------------------------------------------------------------------------------------
 * CAN-TP flow transmission processing
 * ---------------------------------------------------------------------------------------- */
int can_tp_flow(void)
{
    // Data accumulation 
    memset(&tp_pack.TXF, 0, sizeof(CAN_TP_FRAME));
    tp_pack.TXF.FLOW.FRAME.PCI.HEAD.CODE    = CAN_TP_FLOW;   // Flow control 
    tp_pack.TXF.FLOW.FRAME.PCI.HEAD.FC      = CANTP_FC_CTS;  // Transmit permission 
    tp_pack.TXF.FLOW.FRAME.BS               = 1;             // Block size (0=continuous) 
    tp_pack.TXF.FLOW.FRAME.ST               = 0;             // Frame division time (1ms) 
    tp_pack.BS                              = tp_pack.TXF.FLOW.FRAME.BS;
    tp_pack.BC                              = 0;
    tp_pack.MODE                            = 0;
    return 1;
}

/* ----------------------------------------------------------------------------------------
 * Continuous transmission processing
 * ---------------------------------------------------------------------------------------- */
void can_tp_consecutive(void)
{
    int id  = SELECT_ECU_UNIT + 0x7E8;
    int f   = 0;

    switch (tp_pack.FC) {
    case CANTP_FC_CTS:   // Transmit permission 
        if (tp_pack.MODE & CANTP_MODE_SEND) {     // Transmitting 
            if (tp_pack.MODE & CANTP_MODE_WTE) {  // Waiting for transmission completion 
                return;
            }
            if (tp_pack.MODE & CANTP_MODE_WTU) {
                if (tp_pack.ST > 0) { // Timer enable 
                    if (check_timer(TP_TIMER_ID) == 0) {  // Waiting for time up 
                        return;
                    }
                }
                tp_pack.MODE ^= CANTP_MODE_WTU;
            }
            // Generate continuous transmission frame 
            f = can_tp_send();
        }
        break;
    case CANTP_FC_WAIT:        // Waiting for permission 
        if (tp_pack.ST > 0) {  // Timer enable 
            if (check_timer(TP_TIMER_ID)) {  // Time over 
                tp_pack.MODE = 0;
                return;
            }
        }
        break;
    case CANTP_FC_ABORT:      // Abort 
        tp_pack.MODE    = 0;  // Wait 
        tp_pack.CH      = -1;
        tp_pack.ID      = -1;
        break;
    }
    if (f != 0) { // Reply 
        memcpy(&can_buf.ID[id], tp_pack.TXF.B, 8);
        add_mbox_frame(tp_pack.CH, 8, CAN_DATA_FRAME, id);   // Stack buffer for transmission 
    }
}

/* ----------------------------------------------------------------------------------------
 * CAN-TP transmission complete processing (call from main)
 * ---------------------------------------------------------------------------------------- */
void can_tp_txendreq(void)
{
    if (tp_pack.ST > 0) { // Set time until next transmission 
        if (tp_pack.FC == 0 && (tp_pack.BS > tp_pack.BC || tp_pack.BS == 0)) {  // Transmission permission 
            tp_pack.MODE |= CANTP_MODE_WTU;  // Waiting for time up 
            after_call(TP_TIMER_ID, tp_pack.ST, can_tp_consecutive); // Timer set 
        }
    } else {  // No separation time 
        if (tp_pack.FC == 0 && (tp_pack.BS > tp_pack.BC || tp_pack.BS == 0)) {  // Transmission permission 
            can_tp_consecutive();
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * Check for CAN-TP transmist complete
 * ---------------------------------------------------------------------------------------- */
void can_tp_txecheck(int ch, int id)
{
    if (tp_pack.CH == ch && tp_pack.TXID == id) {
        if (tp_pack.MODE & CANTP_MODE_WTE) {    // Waiting for transmission completion 
            tp_pack.MODE    ^= CANTP_MODE_WTE;  // Release wait 
            tp_pack.TXIF    = 1;                // Transmission complete processing request flag 
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * CAN-TP processing
 * ---------------------------------------------------------------------------------------- */
int can_tp_job(int ch, int id, void *frame)
{
    int             sw = SELECT_ECU_UNIT + 0x7E0;
    int             f = 0;
    int             sz, i;
    unsigned char * dp;

    if (id != 0x7DF && id != sw) {
        return 0;
    }

    memcpy(&tp_pack.RXF, frame, sizeof(CAN_TP_FRAME));
    memset(&tp_pack.TXF, 0x00, sizeof(CAN_TP_FRAME));

    switch (tp_pack.RXF.SINGLE.FRAME.PCI.HEAD.CODE) {
    case CAN_TP_SINGLE:  // Single frame 
        if (tp_pack.MODE == 0) {  // Waiting 
            sz              = tp_pack.RXF.SINGLE.FRAME.PCI.HEAD.SIZE;
            tp_pack.CH      = ch;
            tp_pack.ID      = id;
            tp_pack.INDEX   = 0;
            tp_pack.SIZE    = sz;
            memset(&tp_pack.RXD, 0, sizeof(CAN_TP_BUF));
            dp  = tp_pack.RXF.SINGLE.FRAME.DATA;
            f   = can_tp_build(dp, sz);
            if (f > 0) {  // Transmission processing 
                if (can_tp_send() == 0) {
                    f = 0;
                }
            }
        }
        break;
    case CAN_TP_FIRST:   // Multi first frame 
        if (tp_pack.MODE == 0) {  // Waiting 
            sz              =  (tp_pack.RXF.FIRST.FRAME.PCI.HEAD.SIZE) << 8;
            sz              |= (tp_pack.RXF.FIRST.FRAME.SIZEL) & 0xFF;
            tp_pack.CH      = ch;
            tp_pack.ID      = id;
            tp_pack.INDEX   = 1;
            tp_pack.SIZE    = sz;
            memset(&tp_pack.RXD, 0, sizeof(CAN_TP_BUF));
            dp  = tp_pack.RXF.FIRST.FRAME.DATA;
            sz  = 6;
            f   = can_tp_build(dp, sz);
            if (f > 0) { // Transmission processing 
                if (can_tp_send() == 0) {
                    f = 0;
                }
            } else { // Transmit response 
                f = can_tp_flow();
            }
        }
        break;
    case CAN_TP_CONT:    // Multi-continuation frame 
        if (tp_pack.MODE == 0) {  // Waiting 
            if (tp_pack.CH == ch && tp_pack.ID == id) {  // Port, ID match 
                i = tp_pack.RXF.CONSEC.FRAME.PCI.HEAD.INDEX;
                if (i != tp_pack.INDEX) {  // Index does not match 
                    tp_pack.TXF.FLOW.FRAME.PCI.HEAD.CODE    = CAN_TP_CONT;    // Flow control 
                    tp_pack.TXF.FLOW.FRAME.PCI.HEAD.FC      = CANTP_FC_ABORT; // Abort 
                    tp_pack.TXF.FLOW.FRAME.BS               = 0;              // 
                    tp_pack.TXF.FLOW.FRAME.ST               = 0;              // 
                    tp_pack.TXF.FLOW.FRAME.DATA[0]          = tp_pack.INDEX;  // Current index 
                    f                                       = 1;
                } else {  // Index matches 
                    tp_pack.BC++;
                    tp_pack.INDEX++;
                    tp_pack.INDEX   &= 15;
                    dp              = tp_pack.RXF.CONSEC.FRAME.DATA;
                    sz              = 7;
                    f               = can_tp_build(dp, sz);
                    if (f > 0) {  // Transmission processing 
                        if (can_tp_send() == 0) {
                            f = 0;
                        }
                    } else if (tp_pack.BC >= tp_pack.BS && tp_pack.BS > 0) {     // Block number reached, Flow control transmission 
                        tp_pack.TXF.FLOW.FRAME.PCI.HEAD.CODE    = CAN_TP_FLOW;   // Flow control 
                        tp_pack.TXF.FLOW.FRAME.PCI.HEAD.FC      = CANTP_FC_CTS;  // Transmission permission 
                        tp_pack.TXF.FLOW.FRAME.BS               = tp_pack.BS;    // Block size (0=Continuous) 
                        tp_pack.TXF.FLOW.FRAME.ST               = 0;             // Frame division time (1ms) 
                        tp_pack.BC                              = 0;
                        f                                       = 1;
                    }
                }
            }
        }
        break;
    case CAN_TP_FLOW:    // Flow control frame reception 
        if (tp_pack.MODE & CANTP_MODE_WFL) { // Waiting for flow 
            tp_pack.MODE ^= CANTP_MODE_WFL;  // Release waiting 
            if (tp_pack.CH == ch && tp_pack.ID == id) {  /* Port, ID match
                                                          * Flow control*/
                tp_pack.BC  = 0;
                tp_pack.BS  = 0;
                tp_pack.ST  = 0;
                tp_pack.FC  = tp_pack.RXF.FLOW.FRAME.PCI.HEAD.FC;
                switch (tp_pack.FC) {
                case CANTP_FC_CTS:       // Continue transmission 
                    tp_pack.BS = tp_pack.RXF.FLOW.FRAME.BS; // Block size 
                    if (tp_pack.BS > 0) {  // Specified number of blocks continuous + delay valid 
                        tp_pack.ST = tp_pack.RXF.FLOW.FRAME.ST; // Separation time (ms) 
                        if (tp_pack.ST > 0xF0) {
                            tp_pack.ST = (tp_pack.ST - 0xF0) * 100; // 100 to 900ms 
                        }
                    }
                    f = can_tp_send();
                    break;
                case CANTP_FC_WAIT: // Waiting transmission 
                    tp_pack.MODE    |= CANTP_MODE_WFL | CANTP_MODE_WTU;
                    tp_pack.BS      = -1;    // Transmission disable 
                    tp_pack.ST      = 10000; // Timeout rules (10sec) 
                    after_call(TP_TIMER_ID, tp_pack.ST, can_tp_consecutive);
                    break;
                default:                     // Abort error (overflow, abort) 
                    tp_pack.MODE    = 0;     // Waiting 
                    tp_pack.CH      = -1;
                    tp_pack.ID      = -1;
                    break;
                }
            }
        }
        break;
    default:     // Unsupported mode 
        return 0;
    }
    if (tp_pack.TXF.B[0] != 0 && f != 0) {  // Reply 
        sw              += 8;
        tp_pack.TXID    = sw;
        memcpy(&can_buf.ID[sw], tp_pack.TXF.B, 8);
        if (ch >= 0) {
            add_mbox_frame(ch, 8, CAN_DATA_FRAME, sw); // Stack buffer for transmission 
        }
        return sw;
    }
    return 0;
}

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

/* -*-c++-*-
 * $RCSfile: ecu.c,v $
 * $Revision: 1.00 $
 * $Date: 2016/12/15 14:14:48 $
 *
 * Copyright (c) 2016 LandF Corporation.
 *
 * History:
 **/

/*
 * System definition
 **/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sysio.h>
#include "altypes.h"
#include "iodefine.h"
#include "timer.h"
#include "sci.h"
#include "usb.h"
#include "r_can_api.h"
#include "flash_data.h"
#include "r_Flash_API_RX600.h"

/*
 * User difinition
 **/
#include "memo.h"   // ECU development memo 
#include "ecu.h"    // ECU common definition 
#include "ecu_io.h" // ECU I/O port definition 
#include "can3_spi2.h"
#include "uSD_rspi1.h"
#include "cantp.h"  // CAN-TP definition 

// Built-in initial value setting header 
#include "ecu_def_config.h"

/* ---------------------------------------------------------------------------------------
 * Variable declaration 
 *
 * << E2DATA flash save variables >>
 * Routing map*/
ECU_ROUT_MAP rout_map; // Map variables 
// Definition holding buffer 
CYCLE_EVENTS conf_ecu; // Period/event/remote management definition variables 
// ECU I/O checklist 
EXTERNUL_IO ext_list[ECU_EXT_MAX];
int         ext_list_count; // Number of registered external I/O processes 

/* << RAM-only variables >>
 * Time-up waiting buffer*/
CYCLE_EVENTS wait_tup; // Cycle/event wait variable 
// Transmission waiting buffer for each message box 
SEND_WAIT_BUF send_msg[CAN_CH_MAX];
// CAN data buffer variables 
CAN_FRAME_BUF can_buf;
CAN_FRAME_BUF can_random_mask;
// Message box range 
MBOX_SELECT_ID mbox_sel;

// Data length conversion table 
const int DLC_VALUE_TABLE[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8};

// Repro mode flag 
int repro_mode = 0; // 0=Normal mode / 1=Repro mode 

// 1ms timer counter 
int timer_count;
// Status transmission timer 
int status_timer;

// Log function 
void logging(char *fmt, ...);
void SendPC(char *msg);

// CAN module list (CH0 to 3) 
extern const can_st_ptr CAN_CHANNELS[];

// External I/O device status 
EXT_IO_STATUS exiosts;
unsigned char exio_chg[EX_IO_MAX];
int           exio_chg_mark;
// CAN-ID -> EX-I/O-ID conversion table 
unsigned char can_to_exio[CAN_ID_MAX]; // The data indicated by the CAN-ID is the external I/O management number. 00-3F=Applicable, FF=Not supported 

// ECU sequence variables 
int job             = 0; // ECU processing flow 
int led             = 0; // LED flashing 
int stat_update_id  = 0; // ID position to notify LCD 
#ifdef      __LFY_RX63N__
int stat_comm = 1;       // Communication port number 0 to 6 to notify LCD 
#else
int stat_comm = 0;       // Communication port number 0 to 6 to notify LCD 
#endif
int ds_xross_pt_index   = -1; // DS conflict I/O list number retention 
int ds_x_lost_counter   = 0;  // Driving simulator competition end detection counter 

RX_MB_BUF rxmb_buf[3]; // Receive sub-buffer 

// LED monitoring ID setting 
int           led_monit_id     = 0;          // Monitor ID 
unsigned char led_monit_ch     = 0;          // Monitor CH bit set 
int           led_monit_first  = 0x7FFFFFFF; // Minimum time 
int           led_monit_slow   = 0;          // Longest time 
int           led_monit_time   = 0;          // Average time 
int           led_monit_count  = 0;          // Averaging times 
int           led_monit_sample = 0;          // Number of samples 

// Hex string definition 
const char HEX_CHAR[] = "0123456789ABCDEF";

/* ---------------------------------------------------------------------------------------
 * search_target_id
 * 
 * Outline
 *     Check if the ID is managed by your own station
 *
 * Argument
 *     int id  Search ID number
 *
 * Description
 *     Searches the specified ID number from the definition buffer
 *
 * Return
 *     int  0 or more: Definition buffer number / -1: Not applicable ID
 *---------------------------------------------------------------------------------------*/
int search_target_id(int id)
{
    int i, sid;
    CAN_ID_FORM *idf;

    for (i = 0; i < MESSAGE_MAX && i < conf_ecu.CNT; i++) {
        sid = conf_ecu.LIST[i].ID.BIT.SID & CAN_ID_MASK;
        if (sid == id) {
            return i; // ID match 
        }
    }
    return -1;
}
/* ---------------------------------------------------------------------------------------
 * search_wait_id
 * 
 * Outline
 *     Search from the waiting list
 *
 * Argument
 *     int id  Search ID number
 *
 * Description
 *     Searches the specified ID number from the definition buffer
 *
 * Return
 *     int  0 or more: Definition buffer number / -1: Not applicable ID
 *---------------------------------------------------------------------------------------*/
int search_wait_id(int id)
{
    int i, j, n, sid;
    CAN_ID_FORM *idf;

    n = wait_tup.TOP;
    if (n < 0) {
        return -1; // No waiting 
    }
    for (i = 0; i < MESSAGE_MAX && n < MESSAGE_MAX; i++) {
        sid = wait_tup.LIST[n].ID.BIT.SID & CAN_ID_MASK;
        if (sid == id) {
            return n; // ID match 
        }
        n = wait_tup.LIST[n].ID.BIT.NXT; // Next waiting message number 
    }
    return -1;
}

/* ---------------------------------------------------------------------------------------
 * can_check_mb
 * 
 * Outline
 *     Get Mailbox Status
 *
 * Argument
 *     int ch CAN port number
 *     int mb mail box number
 *
 * Description
 *     Transfers the frame of the transmission waiting buffer to the CAN register and starts transmission
 *
 * Return
 *     Status 0=Unused (can be sent/received) / 1=In use
 *---------------------------------------------------------------------------------------*/
int can_check_mb(int ch, int mb)
{
    unsigned char f;
    switch (ch) {
    default:
        return 1;
    case 0: // CAN0 
        f = CAN0.MCTL[mb].BYTE;
        break;
    case 1: // CAN1 
        f = CAN1.MCTL[mb].BYTE;
        break;
    case 2: // CAN2 
        f = CAN2.MCTL[mb].BYTE;
        break;
    case 3: // CAN3(MCSP2515-CAN controller) 
        f = CAN3_GetTxMCTL(mb);
        break;
    }
    return (((f & 0xC0) == 0) ? 0 : 1);
}

/* ---------------------------------------------------------------------------------------
 * can_do_txmb_ch0
 * 
 * Outline
 *     Send to CAN port
 *
 * Argument
 *     SEND_WAIT_FLAME *act Pointer to the waiting list structure
 *     int mb               Outgoing mailbox number
 *
 * Description
 *     There are empty frames in the transmission waiting buffer to be deleted due to timeout
 *     Transfer to CAN register and start transmission
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
int can_do_txmb_ch0(SEND_WAIT_FLAME *act, int mb)
{
    int i;

    while (CAN0.MCTL[mb].BYTE != 0) {
        CAN0.MCTL[mb].BYTE = 0x00;
    }
    CAN0.MB[mb].ID.LONG     = 0;
    CAN0.MB[mb].ID.BIT.SID  = act->ID.BIT.SID;
    CAN0.MB[mb].ID.BIT.RTR  = act->ID.BIT.RTR;
    CAN0.MB[mb].DLC         = act->ID.BIT.DLC;
    for (i = 0; i < 8; i++) {
        CAN0.MB[mb].DATA[i] = act->FD.BYTE[i];
    }
    CAN0.MCTL[mb].BYTE = 0x80;
    return R_CAN_OK;
}
int can_do_txmb_ch1(SEND_WAIT_FLAME *act, int mb)
{
    int i;

    while (CAN1.MCTL[mb].BYTE != 0) {
        CAN1.MCTL[mb].BYTE = 0x00;
    }
    CAN1.MB[mb].ID.LONG     = 0;
    CAN1.MB[mb].ID.BIT.SID  = act->ID.BIT.SID;
    CAN1.MB[mb].ID.BIT.RTR  = act->ID.BIT.RTR;
    CAN1.MB[mb].DLC         = act->ID.BIT.DLC;
    for (i = 0; i < 8; i++) {
        CAN1.MB[mb].DATA[i] = act->FD.BYTE[i];
    }
    CAN1.MCTL[mb].BYTE = 0x80;
    return R_CAN_OK;
}
int can_do_txmb_ch2(SEND_WAIT_FLAME *act, int mb)
{
    int i;

    while (CAN2.MCTL[mb].BYTE != 0) {
        CAN2.MCTL[mb].BYTE = 0x00;
    }
    CAN2.MB[mb].ID.LONG     = 0;
    CAN2.MB[mb].ID.BIT.SID  = act->ID.BIT.SID;
    CAN2.MB[mb].ID.BIT.RTR  = act->ID.BIT.RTR;
    CAN2.MB[mb].DLC         = act->ID.BIT.DLC;
    for (i = 0; i < 8; i++) {
        CAN2.MB[mb].DATA[i] = act->FD.BYTE[i];
    }
    CAN2.MCTL[mb].BYTE = 0x80;
    return R_CAN_OK;
}

/* ---------------------------------------------------------------------------------------
 * can_powtx_delmb
 * 
 * Outline
 *     Empty mailbox sending process for deleted messages
 *
 * Argument
 *     int ch  CAN port number
 *     int mb  Message box number (Mail box number)
 *     int mi  Message number
 *
 * Description
 *     There are empty frames in the transmission waiting buffer to be deleted due to timeout
 *     Transfer to CAN register and start transmission
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void can_powtx_delmb(int ch, int mb, int mi)
{
    int i, id, bkmb;
    SEND_WAIT_FLAME *act;

    if (ch >= 3) {
        return; // Valid only for CPU built-in CH 
    }
    // Free MB search 
    for (bkmb = 3; bkmb < 16; bkmb++) {
        if (can_check_mb(ch, bkmb) == 0) {
            break;
        }
    }

    if (bkmb < 16) { // Mailbox available 
        if (mi >= 0 && mi < MESSAGE_MAX) { // Waiting 
            act = &send_msg[ch].BOX[mb].MSG[mi];
            id  = act->ID.BIT.SID;
            if (act->ID.BIT.ENB != 0) { // Transmit permission 
                switch (ch) {
                case 0: //CAN0 
                    can_do_txmb_ch0(act, bkmb);
                    break;
                case 1: //CAN1 
                    can_do_txmb_ch1(act, bkmb);
                    break;
                case 2: //CAN2 
                    can_do_txmb_ch2(act, bkmb);
                    break;
                }
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * delete_mbox_frame
 * 
 * Outline
 *     Message box message deletion processing
 *
 * Argument
 *     int ch  Transmit CAN channel number (0 to 3)
 *     int mb  Message box number
 *     int mi  Message number
 *
 * Description
 *     Delete the specified frame in the transmission waiting mailbox of the specified CAN channel
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void delete_mbox_frame(int ch, int mb, int mi)
{
    int i, id, nxt;
    CAN_ID_FORM *idf;

    idf       = &send_msg[ch].BOX[mb].MSG[mi].ID;
    id        = idf->BIT.SID;
    nxt       = idf->BIT.NXT; // Continuous chain retention 
    idf->LONG = 0; // Message disable 
    // Chain search 
    if (send_msg[ch].BOX[mb].TOP == mi && mi != nxt) { // First message 
        send_msg[ch].BOX[mb].TOP = nxt;
    } else { // Intermediate message 
        for (i = 0; i < MESSAGE_MAX; i++) {
            idf = &send_msg[ch].BOX[mb].MSG[i].ID;
            if (idf->BIT.ENB == 0) {
                continue;
            }
            if (idf->BIT.NXT == mi) { // Connection source chain discovery 
                idf->BIT.NXT = nxt;   // Remove from chain 
                break;
            }
        }
    }
    // Counter -1 
    if (send_msg[ch].BOX[mb].CNT > 0) {
        send_msg[ch].BOX[mb].CNT--;
    }
}

/* ---------------------------------------------------------------------------------------
 * can_tx_mb
 * 
 * Outline
 *     Mailbox transmission processing for messages waiting to be sent
 *
 * Argument
 *     int ch CAN port number
 *     int mb Message box number (Mail box number)
 *
 * Description
 *     Transfers the frame of the transmission waiting buffer to the CAN register and starts transmission
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void can_tx_mb(int ch, int mb)
{
    int i;
    int mp;
    SEND_WAIT_FLAME *act;
    uint32_t lwk;

    if (can_check_mb(ch, mb) == 0) {       // Mail box enable 
        mp = send_msg[ch].BOX[mb].TOP;     // Transmit waiting chain 
        if (mp >= 0 && mp < MESSAGE_MAX) { // Waiting 
            act = &send_msg[ch].BOX[mb].MSG[mp];
            if (act->ID.BIT.ENB != 0) {    // Transmit enable 
                switch (ch) {
                case 0: //CAN0 
                    lwk = can_do_txmb_ch0(act, mb);
                    break;
                case 1: //CAN1 
                    lwk = can_do_txmb_ch1(act, mb);
                    break;
                case 2: //CAN2 
                    lwk = can_do_txmb_ch2(act, mb);
                    break;
                case 3: //CAN3(MCP2515) 
                    lwk = CAN3_TxSet(mb, act);
                    break;
                }
                if (lwk == R_CAN_OK) { // Setup OK 
#ifdef  SORT_TXWAITLIST_ENABLE
                    send_msg[ch].BOX[mb].TOP = act->ID.BIT.NXT; // Next transmission frame 
#else
                    send_msg[ch].BOX[mb].TOP++;
                    send_msg[ch].BOX[mb].TOP &= MESSAGE_MSK;    // Read pointer update 
#endif
                    send_msg[ch].BOX[mb].CNT--;
                    act->ID.LONG = 0;       // Delete 
                } else { // Error occurred 
                    logging("CAN_TxSet Err = %08lX\r",lwk);
                }
            } else { // Chain error 
                send_msg[ch].BOX[mb].WP  =  0;
                send_msg[ch].BOX[mb].TOP = -1;
                send_msg[ch].BOX[mb].CNT =  0;
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * send_mbox_frame
 * 
 * Outline
 *     Message box CAN frame transmission processing
 *
 * Argument
 *     None
 *
 * Description
 *     Transfers the frame of the transmission waiting buffer to the CAN register and starts transmission
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void send_mbox_frame(void)
{
    int ch;
    int mb;

#ifdef  __LFY_RX63N__
    ch = CAN_TEST_LFY_CH;
#else
    for (ch = 0; ch < CAN_CH_MAX; ch++)
#endif
    {   // CAN port number 
        for (mb = 0; mb < MESSAGE_BOXS; mb++) { // MBOX number 
            can_tx_mb(ch, mb);  // Waiting for transmission and execution of transmission 
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * add_mbox_frame
 * 
 * Outline
 *     Stacking of message box CAN frame transmission buffer
 *
 * Argument
 *     int ch  Transmit CAN channel number (0 to 3)
 *     int dlc Transmit data length
 *     int rtr Select transmission frame (0=data / 1=remote)
 *     int id  Transmit ID
 *
 * Description
 *     Stack the specified data in the transmission waiting buffer of the specified CAN channel
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void add_mbox_frame(int ch, int dlc, int rtr, int id)
{
    int                 i;
    int                 mb, mi, mp;
    SEND_WAIT_FLAME *   msg, *old, *act;
    // Select MBOX 
    mb = (id < mbox_sel.CH[ch].MB1) ? 0 : (id < mbox_sel.CH[ch].MB2) ? 1 : 2;

    mi                      = send_msg[ch].BOX[mb].WP;  // Get write pointer 
    send_msg[ch].BOX[mb].WP = (mi + 1) & MESSAGE_MSK;   // Update write pointer 
    act                     = &send_msg[ch].BOX[mb].MSG[mi]; // Get buffer 
    // Check if in use 
    if (act->ID.BIT.ENB != 0) { // Timeout and deletion processing for 256 unsent messages 
        can_powtx_delmb(ch, mb, mi);    // Attempt to transmit message forcibly 
        delete_mbox_frame(ch, mb, mi);  // Delete message 
    }
    // Register message 
    act->ID.LONG    = 0;
    act->ID.BIT.SID = id;
    act->ID.BIT.RTR = rtr;  // Frame setting 
    act->ID.BIT.ENB = 1;    // Transmittion valid 
    act->ID.BIT.NXT = MESSAGE_END;
    act->ID.BIT.DLC = dlc;
    act->FD.LONG[0] = can_buf.ID[id].LONG[0];
    act->FD.LONG[1] = can_buf.ID[id].LONG[1];
#ifdef  SORT_TXWAITLIST_ENABLE
    // Transmit waiting chain 
    mp = send_msg[ch].BOX[mb].TOP;
    if (mp < 0 || mp >= MESSAGE_MAX) { // As there is no waiting, make it top 
        send_msg[ch].BOX[mb].TOP = mi; // First message 
    } else { // As there is waiting, chain connect 
        msg = &send_msg[ch].BOX[mb].MSG[mp];
        if (msg->ID.BIT.SID > id) { // Prior to first message 
            send_msg[ch].BOX[mb].TOP = mi;
            act->ID.BIT.NXT          = mp;
        } else { // Connect behind 
            for (i = 0; i < MESSAGE_MAX; i++) {
                old = msg;  // Keep previous one 
                mp  = msg->ID.BIT.NXT;  // Next message 
                if (mp >= MESSAGE_MAX && i != mi) { // As there is no continuation, append at the end 
                    msg->ID.BIT.NXT = mi;
                    break;
                }
                // Continuation message 
                msg = &send_msg[ch].BOX[mb].MSG[mp];
                if (msg->ID.BIT.SID > id) { // As it is low priority message, interrupt before this 
                    act->ID.BIT.NXT = mp;
                    old->ID.BIT.NXT = mi;
                    break;
                }
            }
        }
    }
#endif // ifdef  SORT_TXWAITLIST_ENABLE
    send_msg[ch].BOX[mb].CNT++;
}

/* ---------------------------------------------------------------------------------------
 * can_recv_frame
 * 
 * Outline
 *     CAN data reception processing
 *
 * Argument
 *     int ch Receive CAN channel number (0 to 3)
 *     CAN_MBOX *mbox Pointer to incoming message
 *
 * Description
 *     Process a frame that arrived in message box.
 *     Transfer the data frame after copying to buffer.
 *     Remote frame returns target data.
 *
 * Return
 *     Data update 0=No / 1=Yes
 *---------------------------------------------------------------------------------------*/
extern int retport;
int can_recv_frame(int ch, CAN_MBOX *mbox)
{
    int           ret = 0; // Return value 
    int           i;
    int           id;    // Get ID 
    int           dlc;   // Number of data bytes 
    CAN_DATA_BYTE data;  // Data buffer 
    unsigned char rxmsk; // Channel receiving mask 
    unsigned char txmsk; // Channel transmission mask 
    unsigned char cgw;   // Transfer flag 
    unsigned char c1, c2, c3; // Counter 

    id = mbox->ID.BIT.SID;

    dlc   = DLC_VALUE_TABLE[mbox->DLC & 15];
    rxmsk = 0x10 << ch;       // Receiving channel bit 
    txmsk = 0x01 << ch;       // Transmission channel bit 
    cgw   = rout_map.ID[id].BYTE; // Transfer MAP bit 

    if (mbox->ID.BIT.RTR == 0) { // Data frame 
        if ((cgw & rxmsk) == 0 && (cgw & txmsk) != 0) { // Transmit only is rejected 
            return 0;
        }
        for (i = 0; i < dlc; i++) {
            data.BYTE[i] = mbox->DATA[i];
        }
        for (; i < 8; i++) {
            data.BYTE[i] = 0; //can_buf.ID[id].BYTE[i]; 
        }
        /* ---------------------------------
         * Driving simulator competition processing
         *---------------------------------*/
        if (id == DS_X_POWERTRAIN_ID) { // Receive drive simulator conflict ID 
            if (SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN) { // Only power trains are processed 
                if (ds_xross_pt_index >= 0) { // Detect 
                    if (ext_list[ds_xross_pt_index].PORT.BIT.MODE < 4) { // ECU external input -> Apply in CAN output mode 
                        ext_list[ds_xross_pt_index].PORT.BIT.MODE |= 4;  // Change from CAN input to ECU external output 
                    }
                    ds_x_lost_counter = 0;  // Initialize continuation counter 
                    // Invert MSB of first byte of received data to signal contention 
                    data.BYTE[0] |= 0x80;
                }
            }
        }
        if (
            data.LONG[0] == can_buf.ID[id].LONG[0] &&
            data.LONG[1] == can_buf.ID[id].LONG[1] && id < 0x700
        ) { // No data change 
            return 0;
        }
        can_buf.ID[id].LONG[0] = data.LONG[0];
        can_buf.ID[id].LONG[1] = data.LONG[1];
        ret                    = 1;
        /* ---------------------------------
         * Transport layer processing
         * ---------------------------------*/
        if (id >= 0x7DF && id <= 0x7EF) {
            //CAN-TP only ID 
            if (can_tp_job(ch, id, data.BYTE) > 0 && id != 0x7DF) {
                return 0; // Single TP (UDS,OBD2) processing 
            }
        }
        /* ---------------------------------
         * Other port forwarding processing
         * ---------------------------------*/
        if ((cgw & rxmsk) != 0) { // Transfer processing target 
            txmsk = cgw & ~txmsk;
            if ((txmsk & 0x01) != 0) { // CAN0 transfer enable 
                add_mbox_frame(0, dlc, CAN_DATA_FRAME, id);
            }
            if ((txmsk & 0x02) != 0) { // CAN1 transfer enable 
                add_mbox_frame(1, dlc, CAN_DATA_FRAME, id);
            }
            if ((txmsk & 0x04) != 0) { // CAN2 transfer enable 
                add_mbox_frame(2, dlc, CAN_DATA_FRAME, id);
            }
            if ((txmsk & 0x08) != 0) { // CAN3 transfer enable 
                add_mbox_frame(3, dlc, CAN_DATA_FRAME, id);
            }
        }
    } else { // Remote frame 
        if ((cgw & rxmsk) != 0) { // Processing object 
            if (search_target_id(id) >= 0) { // As it is target ID, reply data frame 
                add_mbox_frame(ch, dlc, CAN_DATA_FRAME, id);
            }
            // Confirm transfer target 
            txmsk = cgw & ~txmsk;
            if ((txmsk & 0x01) != 0) { // CAN0 transfer enable 
                add_mbox_frame(0, dlc, CAN_REMOTE_FRAME, id);
            }
            if ((txmsk & 0x02) != 0) { // CAN1 transfer enable 
                add_mbox_frame(1, dlc, CAN_REMOTE_FRAME, id);
            }
            if ((txmsk & 0x04) != 0) { // CAN2 transfer enable 
                add_mbox_frame(2, dlc, CAN_REMOTE_FRAME, id);
            }
            if ((txmsk & 0x08) != 0) { // CAN3 transfer enable 
                add_mbox_frame(3, dlc, CAN_REMOTE_FRAME, id);
            }
        }
    }
    return ret;
}

/* ---------------------------------------------------------------------------------------
 * can_send_proc
 * 
 * Outline
 *     Stacking processing waiting for CAN data transmission
 *
 * Argument
 *     ECU_CYC_EVE *ev Period/event information
 *
 * Description
 *     Stack up to the transmission buffer according to the cycle/event information.
 *     Perform data frame transmission or remote frame transmission processing.
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void can_send_proc(ECU_CYC_EVE *ev)
{
    int            id;  // Get ID 
    int            dlc; // Number of data bytes 
    unsigned char  msk; // Channel mask 
    CAN_DATA_BYTE *act; // Active data 
    CAN_DATA_BYTE *rms; // Random data mask 
    CAN_DATA_BYTE  val; // Random data 

    id  = ev->ID.BIT.SID;
    dlc = DLC_VALUE_TABLE[ev->ID.BIT.DLC];
    msk = rout_map.ID[id].BYTE & ((repro_mode == 0) ? 0x0F : 0xF0);

    if (ev->ID.BIT.RTR == 0) { // Data frame 
        if (id == DS_X_POWERTRAIN_ID) {
            if (ds_xross_pt_index >= 0) {
                if (ext_list[ds_xross_pt_index].PORT.BIT.MODE > 3) { // DS detected 
                    if (ds_x_lost_counter < 1000) { // Wait 10 seconds without receiving 
                        ds_x_lost_counter++;
                        if (ds_x_lost_counter >= 1000) {
                            ext_list[ds_xross_pt_index].PORT.BIT.MODE &= 3;
                            // Data initialization 
                            exiosts.DATA[ext_list[ds_xross_pt_index].PORT.BIT.NOM].LONG = 0;
                            can_buf.ID[id].LONG[0] = 0;
                            can_buf.ID[id].LONG[1] = 0;
                        } else {
                            return; // Cancel transmission during DS 
                        }
                    }
                }
            }
        }
        if (id < 0x700) { /* 700-7FF and DS competitive ID do not carry random information
                           * Random data generation additional processing*/
            act          = &can_buf.ID[id];
            rms          = &can_random_mask.ID[id];
            val.WORD[0]  = rand();
            val.WORD[3]  = val.WORD[0];
            val.WORD[1]  = rand();
            val.WORD[2]  = val.WORD[1];
            act->LONG[0] = 
                ((act->LONG[0] + val.LONG[0]) & (~rms->LONG[0])) |
                 (act->LONG[0] & rms->LONG[0]);
            act->LONG[1] = 
                ((act->LONG[1] - val.LONG[1]) & (~rms->LONG[1])) |
                 (act->LONG[1] & rms->LONG[1]);
        }
        if ((msk & 0x01) != 0) { // CAN0 transfer enable 
            add_mbox_frame(0, dlc, CAN_DATA_FRAME, id);
        }
        if ((msk & 0x02) != 0) { // CAN1 transfer enable 
            add_mbox_frame(1, dlc, CAN_DATA_FRAME, id);
        }
        if ((msk & 0x04) != 0) { // CAN2 transfer enable 
            add_mbox_frame(2, dlc, CAN_DATA_FRAME, id);
        }
        if ((msk & 0x08) != 0) { // CAN3 transfer enable 
            add_mbox_frame(3, dlc, CAN_DATA_FRAME, id);
        }
    } else { // Remote frame 
        if ((msk & 0x01) != 0) { // CAN0 transfer enable 
            add_mbox_frame(0, dlc, CAN_REMOTE_FRAME, id);
        }
        if ((msk & 0x02) != 0) { // CAN1 transfer enable 
            add_mbox_frame(1, dlc, CAN_REMOTE_FRAME, id);
        }
        if ((msk & 0x04) != 0) { // CAN2 transfer enable 
            add_mbox_frame(2, dlc, CAN_REMOTE_FRAME, id);
        }
        if ((msk & 0x08) != 0) { // CAN3 transfer enable 
            add_mbox_frame(3, dlc, CAN_REMOTE_FRAME, id);
        }
    }
    // Transmission processing 
    send_mbox_frame();
}

/* ---------------------------------------------------------------------------------------
 * can_timer_send
 * 
 * Outline
 *     CAN data time-up transmission processing (1msec cycle call)
 *
 * Argument
 *     int tcnt 1ms timer count value
 *
 * Description
 *     Transmission timing wait period/event transmission processing called every 1 ms
 *     Stack messages with remaining time 0 in transmit buffer
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void can_timer_send(int tcnt)
{
    int i, n, c; // Pointer 
    int t;
    int id; // Get ID 
    ECU_CYC_EVE *act; // Period/event information 
    ECU_CYC_EVE *old; // Period/event information 

    i = wait_tup.TOP;
    if (i < 0) {
        return; // No waiting 
    }
    old = 0;
    c   = 0;
    while (i < MESSAGE_MAX && i >= 0) { // Continue 
        c++;
        act = &wait_tup.LIST[i]; // Time-up queue information 
        n   = act->ID.BIT.NXT;   // Continuation pointer 
        if (act->ID.BIT.ENB == 0) {   // Remove unknown disable wait 
            if (i == wait_tup.TOP) {
                if (n < MESSAGE_MAX) { // Continuation first 
                    wait_tup.TOP = n;
                } else { // No waiting 
                    wait_tup.TOP = -1;
                }
            } else { // Processing enable 
                if (old != 0) {
                    old->ID.BIT.NXT = n;
                }
            }
            act->ID.LONG = 0;   // Abort 
            wait_tup.CNT--;
        } else { // Processing enable 
            if (act->TIMER.WORD.TIME == 0) { // Immediate transmission (TIME = 0 does not exist in periodic messages) 
                can_send_proc(act);          // Stack transmission buffer 
                if (i == wait_tup.TOP) {     // Remove head 
                    if (n < MESSAGE_MAX) {   // Continuation first 
                        wait_tup.TOP = n;
                    } else { // No waiting 
                        wait_tup.TOP = -1;
                    }
                } else { // Delete middle 
                    if (old != 0) {
                        old->ID.BIT.NXT = n;
                    }
                }
                act->ID.LONG = 0; // Abort 
                wait_tup.CNT--;
            } else {
                // Has remaining time 
                t   = (int)act->TIMER.WORD.CNT;
                t   -= tcnt;
                if (t <= 0) {                   // Transmission timing reached 
                    can_send_proc(act);         // Stack transmission buffer 
                    if (act->ID.BIT.REP != 0) { // Timer reset for periodic messages 
                        t += (int)act->TIMER.WORD.TIME;
                    } else { // Delete event 
                        if (old != 0) {
                            old->ID.BIT.NXT = n;
                        }
                        act->ID.LONG = 0; // Abort 
                        wait_tup.CNT--;
                    }
                }
                act->TIMER.WORD.CNT = t;
            }
        }
        old = act;
        if (i == n || c >= MESSAGE_MAX) { // Queue disorder 
            wait_tup.TOP = -1;
            logging("Chain Error\r");
            break;
        }
        i = n;
    }
}

/* ---------------------------------------------------------------------------------------
 * add_cyceve_list
 * 
 * Outline
 *     Add periodic event data
 *
 * Argument
 *     int rtr   Remote frame specification     0/1
 *     int id    CAN message number             0 to 2047
 *     int dlc   Data byte length               0 to 8
 *     int enb   Processing permission flag     0/1
 *     int rep   Periodic frame specification   0/1
 *     int time  Period time or delay time (ms) 0 to 65535
 *     int cnt   Delay increase time (ms)       0 to 65535
 *
 * Description
 *     Registration of cycle/event
 *
 * Return
 *     Buffer addition position
 *---------------------------------------------------------------------------------------*/
int add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt)
{
    int i;
    int mb, mi, mp;
    ECU_CYC_EVE *msg, *old, *act;

    // Find a free buffer 
    for (i = 0; i < MESSAGE_MAX; i++) {
        if (conf_ecu.LIST[i].ID.BIT.ENB == 0) {
            break;
        }
    }
    if (i < MESSAGE_MAX) {
        mi = i;
    } else {
        mi          = conf_ecu.WP;            // Get write pointer 
        conf_ecu.WP = (mi + 1) & MESSAGE_MSK; // Update write pointer 
    }
    act = &conf_ecu.LIST[mi]; // Get buffer 
    // Register message 
    act->ID.LONG         = 0;
    act->ID.BIT.RTR      = rtr; // Frame setting 
    act->ID.BIT.SID      = id;  // ID setting 
    act->ID.BIT.DLC      = dlc; // Data byte number 
    act->ID.BIT.ENB      = enb; // Enable 
    act->ID.BIT.REP      = rep; // Periodic message setting 
    act->ID.BIT.NXT      = MESSAGE_END;
    act->TIMER.WORD.TIME = time;
    act->TIMER.WORD.CNT  = cnt;
    // Transmission waiting chain 
    mp = conf_ecu.TOP;
    if (mp < 0) { // As it is no waiting, make it to beginning 
        conf_ecu.TOP = mi;  // First message 
        logging("conf new %d\r", mi);
    } else { // As it has waiting, connect chain 
        msg = &conf_ecu.LIST[mp];
        if (msg->ID.BIT.SID > id) { // Priorize than beginning message 
            conf_ecu.TOP    = mi;
            act->ID.BIT.NXT = mp;
            logging("conf top %d\r", mi);
        } else { // Connect behind 
            for (i = 0; i < MESSAGE_MAX; i++) {
                old = msg;               // Keep previous one 
                mp  = msg->ID.BIT.NXT;   // Next message 
                if (mp >= MESSAGE_MAX) { // As there is no continuation, append at the end 
                    msg->ID.BIT.NXT = mi;
                    logging("conf add %d\r", mi);
                    break;
                }
                // Continuation message 
                msg = &conf_ecu.LIST[mp];
                if (msg->ID.BIT.SID > id) { // As it is low priority message, insert before this 
                    act->ID.BIT.NXT = mp;
                    old->ID.BIT.NXT = mi;
                    logging("conf ins %d\r", mi);
                    break;
                }
            }
        }
    }
    conf_ecu.CNT++;
    return mi;
}

/* ---------------------------------------------------------------------------------------
 * insert_cyceve_list
 * 
 * Outline
 *     Add cyclic event data in the middle
 *
 * Argument
 *     int mi  Buffer number  0 to 255
 *
 * Description
 *     Link registered information of cycle/event
 *
 * Return
 *     Buffer addition position
 *---------------------------------------------------------------------------------------*/
void insert_cyceve_list(int mi)
{
    int i, id;
    int mb, mp;
    ECU_CYC_EVE *msg, *old, *act;

    act = &conf_ecu.LIST[mi]; // Get buffer 
    if (act->ID.BIT.ENB == 0) {
        return; // Do not link information that does not drive 
    }
    id              = act->ID.BIT.SID;
    act->ID.BIT.NXT = MESSAGE_END;
    // Transmission waiting chain 
    mp = conf_ecu.TOP;
    if (mp < 0) { // As it has no waiting messages, make it the first message
        conf_ecu.TOP = mi; // First message 
        logging("conf new %d\r", mi);
    } else { // As there are waitint messages, connect chain 
        msg = &conf_ecu.LIST[mp];
        if (msg->ID.BIT.SID > id) { // Prioritize first message 
            conf_ecu.TOP    = mi;
            act->ID.BIT.NXT = mp;
            logging("conf top %d\r", mi);
        } else { // Connect behind 
            for (i = 0; i < MESSAGE_MAX; i++) {
                old = msg;               // Keep previous one 
                mp  = msg->ID.BIT.NXT;   // Next message 
                if (mp >= MESSAGE_MAX) { // As there is no continuation, append at the end 
                    msg->ID.BIT.NXT = mi;
                    logging("conf add %d\r", mi);
                    break;
                }
                // Continuation message 
                msg = &conf_ecu.LIST[mp];
                if (msg->ID.BIT.SID > id) { // As it is low priority message, insert before this 
                    act->ID.BIT.NXT = mp;
                    old->ID.BIT.NXT = mi;
                    logging("conf ins %d\r", mi);
                    break;
                }
            }
        }
    }
    conf_ecu.CNT++;
}

/* ---------------------------------------------------------------------------------------
 * delete_cyceve_list
 * 
 * Outline
 *     Delete cyclic event data
 *
 * Argument
 *     int id CAN message number  0 to 2047
 *
 * Description
 *     Delete cycle/event
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void delete_cyceve_list(int id)
{
    int             mi, mp;
    ECU_CYC_EVE *   msg, *old;

    mp = conf_ecu.TOP;
    while (mp >= 0) {
        mi  = mp;
        msg = &conf_ecu.LIST[mp];
        mp  = msg->ID.BIT.NXT;       // Next message 
        if (msg->ID.BIT.SID == id) { // Match 
            if (conf_ecu.TOP == mi) {// Delete beginning 
                conf_ecu.TOP = mp;
            } else { // Delete middle 
                old->ID.BIT.NXT = mp;
            }
            conf_ecu.CNT--;
            return;
        }
        old = msg;
    }
}

/* ---------------------------------------------------------------------------------------
 * can_id_event
 * 
 * Outline
 *     Data update event generation processing
 *
 * Argument
 *     int id  ID number where the change occurred
 *     int tp  Additional wait time (ms)
 *
 * Description
 *     Called by a data update request from an external device other than CAN
 *     Add the ID of the event processing target to the time-up queue
 *
 * Return
 *     Negative number is registration failure / 0 or more delay time (ms)
*---------------------------------------------------------------------------------------*/
int can_id_event(int id, int tp)
{
    int i, n, p; // Pointer 
    int at; // Delay time (ms) 
    ECU_CYC_EVE *new; // Period / event information 
    ECU_CYC_EVE *act; // Period / event information 
    ECU_CYC_EVE *old; // Period / event information 

    // Investigate if target ID 
    i = search_target_id(id);
    if (i < 0) {
        return -1; // Excluded ID 
    }
    act = &conf_ecu.LIST[i];
    if (act->ID.BIT.REP != 0) { // Multiple registration of periodic messages is prohibited 
        n = search_wait_id(id);
        if (n >= 0) {
            return -2; // Registered 
        }
    }
    p           = wait_tup.WP++;
    wait_tup.WP &= MESSAGE_MSK;
    new         = &wait_tup.LIST[p];
    if (new->ID.BIT.ENB != 0) {
        // Unused search 
        for (p = 0; p < MESSAGE_MAX; p++) {
            if (wait_tup.LIST[p].ID.BIT.ENB == 0) {
                break; // Find unused number 
            }
        }
        if (p >= MESSAGE_MAX) {
            return -3; // No space 
        }
    }
    // Register 
    new                 = &wait_tup.LIST[p];
    new->ID.LONG        = act->ID.LONG;              // Copy message condition 
    new->TIMER.LONG     = act->TIMER.LONG;           // Copy setting value 
    at                  = (int)act->TIMER.WORD.CNT;  // Delay time (ms) 
    new->TIMER.WORD.CNT = new->TIMER.WORD.TIME + tp; // Wait time (ms) 
    new->ID.BIT.NXT     = MESSAGE_END;               // No continuation 
    new->ID.BIT.ENB     = 1;                         // Enable processing 
    // Confirm waiting top 
    i = wait_tup.TOP;
    if (i < 0) { // No waiting (top) 
        wait_tup.TOP = p; // Make it the beginning 
        wait_tup.CNT = 1; // One waiting now 
        logging("Wait new %08lX:%d\r", new->ID.LONG, p);
        return at; // Delay time of continuous registration (ms) 
    }
    // Insert destination search 
    old = 0;
    while (i < MESSAGE_MAX) {
        act = &wait_tup.LIST[i];         // Time-up queue information 
        n   = act->ID.BIT.NXT;           // Continuation pointer 
        if (act->ID.BIT.ENB != 0) {      // Processing enabled 
            if (act->ID.BIT.SID > id) {  // Find low priority message 
                if (i == wait_tup.TOP) { // Swap top 
                    wait_tup.TOP    = p;
                    new->ID.BIT.NXT = i;
                    wait_tup.CNT++; // Increase waiting number 
                    logging("Wait top %08lX:%d→%d\r", new->ID.LONG, p, i);
                    return at;
                } else { // Add in the middle 
                    old->ID.BIT.NXT = p;
                    new->ID.BIT.NXT = i;
                    wait_tup.CNT++; // Increase waiting number 
                    logging("Wait ins %08lX:%d→%d\r", new->ID.LONG, p, i);
                    return at;
                }
            } else if (n >= MESSAGE_MAX) { // Add to the end 
                act->ID.BIT.NXT = p;
                wait_tup.CNT++; // Increase waiting number 
                logging("Wait add %08lX:→%d\r", new->ID.LONG, p);
                return at;
            }
        }
        old = act;  // Previous information 
        if (i == n) { // Chain error 
            logging("Wait chain error %08lX:%d→%d\r", new->ID.LONG, i, n);
            break;
        }
        i = n; // Continuation pointer 
    }
    return -4;  // Destroy chain list 
}

/* ---------------------------------------------------------------------------------------
 * delete_waiting_list
 * 
 * Outline
 *     Deletion of data waiting for periodic event time-up
 *
 * Argument
 *     int id CAN message number  0 to 2047
 *
 * Description
 *     Deletion of data waiting for cycle / event time-up
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void delete_waiting_list(int id)
{
    int             mi, mp;
    ECU_CYC_EVE *   msg, *old;

    mp = wait_tup.TOP;
    while (mp >= 0) {
        mi  = mp;
        msg = &wait_tup.LIST[mp];
        mp  = msg->ID.BIT.NXT;        // Next message 
        if (msg->ID.BIT.SID == id) {  // Match 
            if (wait_tup.TOP == mi) { // Delete top 
                wait_tup.TOP = mp;
            } else { // Delete middle 
                old->ID.BIT.NXT = mp;
            }
            wait_tup.CNT--;
            return;
        }
        old = msg;
    }
}

/* ---------------------------------------------------------------------------------------
 *
 * Outline
 *     Acquire external input value via communication
 *
 * Argument
 *     int nom Buffer number   0 to 63
 *     int md  Mode            0:BIT / 1:BYTE / 2:WORD / 3:LONG
 *
 * Description
 *     Get input status of ECU external I/O connector
 *
 * Return
 *     int Input value (0/1 for digital, 0 to FFF for analog)
*---------------------------------------------------------------------------------------*/
int port_input_ex(int nom, int md)
{
    switch (md) {
    default:
        break;
    case 0: // Bit information 
        return ((int)exiosts.DATA[nom].BIT.B0);
    case 1: // Byte information 
        return ((int)exiosts.DATA[nom].BYTE[0]);
    case 2: // Word information 
        return ((int)exiosts.DATA[nom].WORD[0]);
    case 3: // Long word information 
        return (exiosts.DATA[nom].INTE);
    }
    return 0;
}
/* ---------------------------------------------------------------------------------------
 * port_output_ex
 * 
 * Outline
 *     Update external output value
 *
 * Argument
 *     int nom Buffer number 0 to 63
 *     int val Output value
 *     int md  Mode 0:BIT / 1:BYTE / 2:WORD / 3:LONG
 *
 * Description
 *     Update the output status of the ECU external I/O connector
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void port_output_ex(int nom, int val, int md)
{
    switch (md) {
    case 4: // Bit information 
        val = (val == 0) ? 0 : 1;
        if (exiosts.DATA[nom].BIT.B0 != val) {
            if (exio_chg[nom] < 100) {
                exio_chg[nom]++;// Data update notification 
            }
            exio_chg_mark++; // Update mark 
        }
        exiosts.DATA[nom].BIT.B0 = (val == 0) ? 0 : 1;
        break;
    case 5: // Byte information 
        if (exiosts.DATA[nom].BYTE[0] != val) {
            if (exio_chg[nom] < 100) {
                exio_chg[nom]++; // Data update notification 
            }
            exio_chg_mark++; // Update mark 
        }
        exiosts.DATA[nom].BYTE[0] = (unsigned char)val;
        break;
    case 6: // Word information 
        if (exiosts.DATA[nom].WORD[0] != val) {
            if (exio_chg[nom] < 100) {
                exio_chg[nom]++; // Data update notification 
            }
            exio_chg_mark++; // Update mark 
        }
        exiosts.DATA[nom].WORD[0] = (unsigned short)val;
        break;
    case 7: // Long word information 
        if (exiosts.DATA[nom].INTE != val) {
            if (exio_chg[nom] < 100) {
                exio_chg[nom]++; // Data update notification 
            }
            exio_chg_mark++; // Update mark 
        }
        exiosts.DATA[nom].INTE = val;
        break;
    }
}

/* ---------------------------------------------------------------------------------------
 * extern_io_update_ex
 * 
 * Outline
 *     External I/O update processing via communication
 *
 * Argument
 *     None
 *
 * Description
 *     Acquires the input status of the ECU external I/O connector 
 *     and updates the application data buffer
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void extern_io_update_ex(void)
{
    int i, d, m, tp;
    int id;             // ID number 
    int md;             // I/O processing mode 
    int bp;             // Byte position 
    int nm;             // I/O port number 
    int msk;            // Bit mode mask 
    EXTERNUL_IO *act;   // Active definition pointer 
    CAN_DATA_BYTE *buf; // Frame data buffer 
    CAN_DATA_BYTE  val; // Copy frame data 

    tp = 0; // Time difference setting for simultaneous input change 

    for (i = 0; i < ext_list_count; i++) {
        act = &ext_list[i];
        id  = act->SID;
        if (id >= 0 && id < CAN_ID_MAX) {     // Setting enable 
            nm          = act->PORT.BIT.NOM;  // Buffer number 
            md          = act->PORT.BIT.MODE; // Mode   Input 0:bit / 1:byte / 2:word / 3:long Output 4:bit / 5:byte / 6:word / 7:long 
            bp          = act->PORT.BIT.BPOS; // Byte position 
            msk         = act->PORT.BIT.MSK;  // Mask pattern 
            buf         = &can_buf.ID[id];    // Pointer to data buffer 
            val.LONG[0] = buf->LONG[0];       // Get data 1 
            val.LONG[1] = buf->LONG[1];       // Get data 2 
            // Get status 
            d = port_input_ex(nm, (md & 3));  // Acquisition of input status via communication 
            // Process mode 
            switch (md) {
            default:        // Disable 
                break;
            case 0: // Input bit 
                val.BYTE[bp] &= ~msk;
                val.BYTE[bp] |= (d == 0) ? 0 : msk;
                break;
            case 4: // Output bit 
                d = ((val.BYTE[bp] & msk) == 0) ? 0 : 1;
                port_output_ex(nm, d, md);
                break;
            case 1: // Input byte 
                val.BYTE[bp] = (unsigned char)d;
                break;
            case 5: // Output byte 
                d = (int)((unsigned long)val.BYTE[bp]);
                port_output_ex(nm, d, md);
                break;
            case 2: // Input word 
                val.BYTE[bp]     = (unsigned char)(d >> 8);
                val.BYTE[bp + 1] = (unsigned char)d;
                break;
            case 6: // Output word 
                d = (int)((((unsigned long)val.BYTE[bp]) << 8) | ((unsigned long)val.BYTE[bp + 1]));
                port_output_ex(nm, d, md);
                break;
            case 3: // Input long word 
                val.BYTE[bp]     = (unsigned char)(d >> 24);
                val.BYTE[bp + 1] = (unsigned char)(d >> 16);
                val.BYTE[bp + 2] = (unsigned char)(d >> 8);
                val.BYTE[bp + 3] = (unsigned char)d;
                break;
            case 7: // Output long word 
                d = (int)
                      ((((unsigned long)val.BYTE[bp])  << 24)    |
                       (((unsigned long)val.BYTE[bp + 1]) << 16) |
                       (((unsigned long)val.BYTE[bp + 2]) << 8)  |
                        ((unsigned long)val.BYTE[bp + 3])
                );
                port_output_ex(nm, d, md);
                break;
            }
            if (val.LONG[0] != buf->LONG[0] || val.LONG[1] != buf->LONG[1]) { // With change 
                buf->LONG[0] = val.LONG[0]; // Data update 1 
                buf->LONG[1] = val.LONG[1]; // Data update 2 
                if (exio_chg[nm] < 100) {
                    exio_chg[nm]++; // CAN data update count 
                }
                exio_chg_mark++; // Update mark 
                tp = can_id_event(id, tp); // CAN data update notification 
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * port_input
 * 
 * Outline
 *     External input value acquisition
 *
 * Argument
 *     int nom Port number (0 to 33)
 *
 * Description
 *     Get input status of ECU external I/O connector
 *
 * Return
 *     int Input value (0/1 for digital, 0 to FFF for analog)
*---------------------------------------------------------------------------------------*/
int port_input(int nom)
{
    switch (nom) {
    case 0:  return (X_DB_0  == 0) ? 0 : -1;
    case 1:  return (X_DB_1  == 0) ? 0 : -1;
    case 2:  return (X_DB_2  == 0) ? 0 : -1;
    case 3:  return (X_DB_3  == 0) ? 0 : -1;
    case 4:  return (X_DB_4  == 0) ? 0 : -1;
    case 5:  return (X_DB_5  == 0) ? 0 : -1;
    case 6:  return (X_DB_6  == 0) ? 0 : -1;
    case 7:  return (X_DB_7  == 0) ? 0 : -1;
    case 8:  return (X_DB_8  == 0) ? 0 : -1;
    case 9:  return (X_DB_9  == 0) ? 0 : -1;
    case 10: return (X_DB_10 == 0) ? 0 : -1;
    case 11: return (X_DB_11 == 0) ? 0 : -1;
    case 12: return (X_DB_12 == 0) ? 0 : -1;
    case 13: return (X_DB_13 == 0) ? 0 : -1;
    case 14: return (X_DB_14 == 0) ? 0 : -1;
    case 15: return (X_DB_15 == 0) ? 0 : -1;
    case 16: return (X_DB_16 == 0) ? 0 : -1;
    case 17: return (X_DB_17 == 0) ? 0 : -1;
    case 18: return (X_DB_18 == 0) ? 0 : -1;
    case 19: return (X_DB_19 == 0) ? 0 : -1;
    case 20: return (X_DB_20 == 0) ? 0 : -1;
    case 21: return (X_DB_21 == 0) ? 0 : -1;
    case 22: return (X_DB_22 == 0) ? 0 : -1;
    case 23: return (X_DB_23 == 0) ? 0 : -1;
    case 24: return (X_AN_0 & 0x0FFF);
    case 25: return (X_AN_1 & 0x0FFF);
    case 26: return (X_AN_2 & 0x0FFF);
    case 27: return (X_AN_3 & 0x0FFF);
    case 28: return (X_AN_4 & 0x0FFF);
    case 29: return (X_AN_5 & 0x0FFF);
    case 30: return (X_AN_6 & 0x0FFF);
    case 31: return (X_AN_7 & 0x0FFF);
    }
    return 0;
}
/* ---------------------------------------------------------------------------------------
 * port_output
 * 
 * Outline
 *     External output value update
 *
 * Argument
 *     int nom Port number 0 to 33
 *     int val Output value
 *
 * Description
 *     Update the output status of the ECU external I/O connector
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void port_output(int nom, int val)
{
    switch (nom) {
    case 0:  Y_DB_0  = (val == 0) ? 0 : 1; break;
    case 1:  Y_DB_1  = (val == 0) ? 0 : 1; break;
    case 2:  Y_DB_2  = (val == 0) ? 0 : 1; break;
    case 3:  Y_DB_3  = (val == 0) ? 0 : 1; break;
    case 4:  Y_DB_4  = (val == 0) ? 0 : 1; break;
    case 5:  Y_DB_5  = (val == 0) ? 0 : 1; break;
    case 6:  Y_DB_6  = (val == 0) ? 0 : 1; break;
    case 7:  Y_DB_7  = (val == 0) ? 0 : 1; break;
    case 8:  Y_DB_8  = (val == 0) ? 0 : 1; break;
    case 9:  Y_DB_9  = (val == 0) ? 0 : 1; break;
    case 10: Y_DB_10 = (val == 0) ? 0 : 1; break;
    case 11: Y_DB_11 = (val == 0) ? 0 : 1; break;
    case 12: Y_DB_12 = (val == 0) ? 0 : 1; break;
    case 13: Y_DB_13 = (val == 0) ? 0 : 1; break;
    case 14: Y_DB_14 = (val == 0) ? 0 : 1; break;
    case 15: Y_DB_15 = (val == 0) ? 0 : 1; break;
    case 16: Y_DB_16 = (val == 0) ? 0 : 1; break;
    case 17: Y_DB_17 = (val == 0) ? 0 : 1; break;
    case 18: Y_DB_18 = (val == 0) ? 0 : 1; break;
    case 19: Y_DB_19 = (val == 0) ? 0 : 1; break;
    case 20: Y_DB_20 = (val == 0) ? 0 : 1; break;
    case 21: Y_DB_21 = (val == 0) ? 0 : 1; break;
    case 22: Y_DB_22 = (val == 0) ? 0 : 1; break;
    case 23: Y_DB_23 = (val == 0) ? 0 : 1; break;
    case 24: Y_AN_0  = (val >> 2) & 0x03FF; break;
    case 25: Y_AN_1  = (val >> 2) & 0x03FF; break;
    }
}

/* ---------------------------------------------------------------------------------------
 * extern_io_update
 * 
 * Outline
 *     External I/O update processing
 *
 * Argument
 *     None
 *
 * Description
 *     Acquires the input status of the ECU external I/O connector
 *     and updates the application data buffer
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void extern_io_update(void)
{
    int i, d, m, tp;
    int id;             // ID number 
    int md;             // I/O processing mode 
    int ng;             // Data inversion flag 
    int sz;             // Access size 
    int bp;             // Byte position 
    int dlc;            // Data length 
    int nm;             // I/O port number 
    int msk;            // Bit mode mask 
    unsigned char *pat; // Pattern data 
    EXTERNUL_IO   *act; // Active definition pointer 
    CAN_DATA_BYTE *buf; // Frame data buffer 
    CAN_DATA_BYTE  val; // Copy frame data 

    tp = 0; // Time difference setting for simultaneous input change 

    for (i = 0; i < ext_list_count; i++) {
        act = &ext_list[i];
        id  = act->SID;
        if (id >= 0 && id < CAN_ID_MAX) {     // Setting enable 
            nm  = act->PORT.BIT.NOM;     // Port number 
            md  = act->PORT.BIT.MODE;    // Mode 0:Disable / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit value / 6:3bit value / 7:4bit value 
            ng  = act->PORT.BIT.NEG;     // Inversion flag 
            sz  = act->PORT.BIT.SIZE;    // Access size 0:BIT / 1 to 7:nBYTE 
            bp  = act->PORT.BIT.BPOS;    // Byte position 
            dlc = act->PORT.BIT.DLC;     // Comparison data length 
            msk = act->PORT.BIT.MSK;     // Mask pattern 
            pat = act->PAT;              // Pattern data 
            buf = &can_buf.ID[act->SID]; // Pointer to data buffer 
            val.LONG[0] = buf->LONG[0];  // Get data 1 
            val.LONG[1] = buf->LONG[1];  // Get data 2 
            // Get status 
            m = 1;
            d = port_input(nm); // Get input status 
            if (md > 2 && md < 5) { // ADC word processing 
                m = 0x0FFF; // 12bit 
            } else { // Bit processing 
                if (md > 4) { // Multiple bit 
                    d = ((d & 1) << 1) | (port_input(nm + 1) & 1);
                    m = (m << 1) | 1;
                    if (md > 5) { // Multiple bit 
                        d = ((d & 1) << 1) | (port_input(nm + 2) & 1);
                        m = (m << 1) | 1;
                        if (md > 6) { // Multiple bit 
                            d = ((d & 1) << 1) | (port_input(nm + 3) & 1);
                            m = (m << 1) | 1;
                        }
                    }
                }
            }
            // Request inversion 
            if (ng) { // Data inversion 
                d = ~d;
                d &= m;
            }
            // Mode processing 
            switch (md) {
            default:        // Disable 
                break;
            case 5:         // 2bit value 
            case 6:         // 3bit value 
            case 7:         // 4bit value 
            case 1:         // Input bit 
                if (sz == 0) { // Operate bit 
                    val.BYTE[bp]    &= ~msk;
                    val.BYTE[bp]    |= (d == 0) ? 0 : msk;
                } else { // Transger pattern 
                    memcpy(&val.BYTE[bp], &pat[sz * d], sz);
                }
                tp = can_id_event(id, tp); // Notify to CAN bus 
                break;
            case 2: // Output bit 
                if (sz == 0) { // Operate bit 
                    d = ((val.BYTE[bp] & msk) == 0) ? 0 : 1;
                    if (ng) {
                        d = ~d & 1;
                    }
                    port_output(nm, d);
                } else { // Compare pattern 
                    for (d = 0; d < 2; d++) {
                        if (memcmp(&val.BYTE[bp], &pat[(sz * d)], sz) == 0) { // Match 
                            if (ng) {
                                d = ~d & 1;
                            }
                            port_output(nm, d);
                            break;
                        }
                    }
                }
                break;
            case 3:         // Input ADC word 
                val.BYTE[bp]     = (unsigned char)(d >> 8);
                val.BYTE[bp + 1] = (unsigned char)(d & 0xFF);
                tp = can_id_event(id, tp); // Notify to CAN bus 
                break;
            case 4:         // Output DAC word 
                d = (((unsigned long)val.BYTE[bp]) << 8) | ((unsigned long)val.BYTE[bp + 1]);
                if (act->PORT.BIT.MODE & 0x08) { // Inversion 
                    d = (0 - d) & 0x0FFF;        // Change to 12bit 
                }
                port_output(nm, d);
                break;
            }
            if (
                val.LONG[0] != buf->LONG[0] ||
                val.LONG[1] != buf->LONG[1]
            ) { // With change 
                buf->LONG[0] = val.LONG[0];  // Data update 1 
                buf->LONG[1] = val.LONG[1];  // Data update 2 
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * add_extern_io
 * 
 * Outline
 *     External I/O registration processing
 *
 * Argument
 *     int id   Frame ID number         0 to 2047(000 to 7FF)
 *     int mode I/O processing mode     0:Disable / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit value / 6:3bit value / 7:4bit value
 *     int neg  Specify data inversion  0:Normal / 1:Reverse
 *     int size Access size             0:BIT / 1 to 7:nBYTE
 *     int bpos Byte position           0 to 7
 *     int dlc  Data byte length        0 to 8
 *     int nom  Applicable port number  0 to 63
 *     int msk  Mask pattern            00 to FF
 *     unsigned char *pat  Pattern data 24byte
 *
 * Description
 *     Acquires the input status of the ECU external I/O connector and updates the application data buffer
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat)
{
    int             i;
    EXTERNUL_IO *   act;
    CAN_DATA_BYTE * cmk;

    i = ext_list_count;
    if (i >= 0 && i < ECU_EXT_MAX) {
        ext_list_count++;          // Total update 
        can_to_exio[id] = i;       // Reverse map setting 
        act = &ext_list[i];        // Registration pointer 
        act->SID = id;             // Frame ID number 
        __break__
        act->PORT.BIT.MODE = mode; // I/O processing mode 
        act->PORT.BIT.NEG  = neg;  // Data inversion specification 
        act->PORT.BIT.SIZE = size; // Access size 
        act->PORT.BIT.BPOS = bpos; // Byte position 
        act->PORT.BIT.DLC  = dlc;  // Data byte length 
        act->PORT.BIT.MSK  = msk;  // Mask pattern 
        act->PORT.BIT.NOM  = nom;  // Applicable port number 
        if (pat != 0) {
            memcpy(act->PAT, pat, 24);  // Pattern data 
        }
        cmk = &can_random_mask.ID[id];
        // Mask processing 
        switch (mode) {
        default: // Mask invalid 
            cmk->LONG[0] = -1;
            cmk->LONG[1] = -1;
            break;
        case 0: // Bit input 
            cmk->BYTE[bpos] = msk;
            break;
        case 1: // Byte input 
            cmk->BYTE[bpos] = 0xFF;
            break;
        case 2: // Word input 
            cmk->BYTE[bpos]     = 0xFF;
            cmk->BYTE[bpos + 1] = 0xFF;
            break;
        case 3:     // Long word input 
            cmk->BYTE[bpos]     = 0xFF;
            cmk->BYTE[bpos + 1] = 0xFF;
            cmk->BYTE[bpos + 2] = 0xFF;
            cmk->BYTE[bpos + 3] = 0xFF;
            break;
        }
        // DS conflict ID list number 
        if (id == DS_X_POWERTRAIN_ID) { // DS-only rules 
            ds_xross_pt_index    = i;   // Retain 
            rout_map.ID[id].BYTE = 0x11;
        } else { // Routing map setting 
            /* Define transmit / receive flags in input / output direction 
             * (0..3:  input value -> CAN transmission 
             *  4..7: output value <- CAN reception)  */
            rout_map.ID[id].BYTE = (mode < 4) ? 0x01 : 0x10; 
        }
        return i;
    }
    return -1;
}

/* ---------------------------------------------------------------------------------------
 * set_frame_data
 * 
 * Outline
 *     Data buffer direct setting
 *
 * Argument
 *     int id             Frame ID number  0 to 2047(000 to 7FF)
 *     int dlc            Data length
 *     unsigned char *dat Pointer to data buffer
 *
 * Description
 *     Set value to CAN data buffer
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void set_frame_data(int id, int dlc, unsigned char *dat)
{
    int i;
    CAN_DATA_BYTE *act;

    act = &can_buf.ID[id];  // Select data buffer 
    memcpy(act, dat, dlc);  // Copy 
}

/* ---------------------------------------------------------------------------------------
 * ecu_init
 * 
 * Outline
 *     ECU initialization processing
 *
 * Argument
 *     None
 *
 * Description
 *     Initialization of ECU data area and registration of periodic message
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void ecu_init(void)
{
    int                     i, j, addr;
    POINTER_MULTI_ACCESS    s, d;
    EXTERNUL_IO *           act;
    CAN_DATA_BYTE *         cmk;

    Init_FlashData();

    // Variable initialization 
    memset(&wait_tup, 0, sizeof(wait_tup)); // Period / event wait initialization 
    memset(&send_msg, 0, sizeof(send_msg)); // Initialize the transmission waiting buffer for each message box 
    memset(&can_buf, 0, sizeof(can_buf));   // Initialize CAN data buffer 
    memset(&mbox_sel, 0, sizeof(mbox_sel)); // Initialize message box range 
    memset(&exiosts, 0, sizeof(exiosts));   // Initialize external I/O state 
    memset(&exio_chg, 0, sizeof(exio_chg)); // Initialize external I/O state 
    memset(&can_random_mask, 0, sizeof(can_random_mask)); // Initialize random code mask 
    memset(&rxmb_buf, 0, sizeof(rxmb_buf)); // Receive buffer 
    memset(&conf_ecu, 0, sizeof(conf_ecu)); // Event list 
    memset(ext_list, 0, sizeof(ext_list));  // ECU I/O checklist initialization 
    memset(can_to_exio, -1, sizeof(can_to_exio)); // Initialization of ECU I/O conversion table 

    exio_chg_mark = 0;

    ext_list_count = 0; // Checklist number reset 
    wait_tup.TOP   = -1;
    for (i = 0; i < CAN_CH_MAX; i++) {
        // MBOX0 : ID=000 to MBOX_POINT_1
        mbox_sel.CH[i].MB1 = MBOX_POINT_1; 
        // MBOX1 : ID=MBOX_POINT_1 to MBOX_POINT_2 , if more, MBOX2  
        mbox_sel.CH[i].MB2 = MBOX_POINT_2; 
        for (j = 0; j < MESSAGE_BOXS; j++) {
            send_msg[i].BOX[j].TOP = -1;
        }
    }

    // Get map 
    addr = g_flash_BlockAddresses[BLOCK_DB0];
    // Blank check 
    if (
        (i = R_FlashDataAreaBlankCheck(addr, BLANK_CHECK_ENTIRE_BLOCK)
       ) == FLASH_NOT_BLANK
    ) { /* Data flash enabled (8KB: 0x00100000 - 0x00101FFF)
         * Read routing map*/
        s.LONG = ADDRESS_OF_ROOTMAP;
        d.MAP  = &rout_map;
        memcpy(d.UB, s.UB, sizeof(rout_map)); // Initialize map 
        // Read event list 
        s.LONG = ADDRESS_OF_CYCEVE;
        d.CYE  = &conf_ecu.LIST[0];
         // Initialization of cycle / event / remote management definition 
        memcpy(d.UB, s.UB, sizeof(ECU_CYC_EVE) * MESSAGE_MAX); 
        j = CAN_ID_MAX;
        for (i = 0; i < MESSAGE_MAX; i++) {
            if (conf_ecu.LIST[i].ID.LONG == 0) { //ID.BIT.ENB != 0) 
                // Terminus 
                conf_ecu.WP = i;
                break;
            }
            if (conf_ecu.LIST[i].ID.BIT.SID < j) {
                j = conf_ecu.LIST[i].ID.BIT.SID;
                conf_ecu.TOP = i;
            }
            conf_ecu.CNT++;
        }
        // Read I/O setting 
        s.LONG = ADDRESS_OF_IOLIST;
        d.EXL  = ext_list;
        memcpy(d.UB, s.UB, sizeof(ext_list)); // ECU I/O checklist initialization 
        // Check list valid registration number 
        for (i = 0; i < ECU_EXT_MAX; i++, ext_list_count++) {
            if (ext_list[i].PORT.LONG == 0) {
                // Terminus 
                break;
            }
            ext_list_count++; // Update total number 
            act = &ext_list[i]; // Registration pointer 
            can_to_exio[act->SID] = i; // Reverse map setting 
            cmk = &can_random_mask.ID[act->SID];
            // Mask processing 
            switch (act->PORT.BIT.MODE) {
            default:    // Mask disable 
                cmk->LONG[0] = -1;
                cmk->LONG[1] = -1;
                break;
            case 0: // Bit input 
                cmk->BYTE[act->PORT.BIT.BPOS] = act->PORT.BIT.MSK;
                break;
            case 1: // Byte input 
                cmk->BYTE[act->PORT.BIT.BPOS] = 0xFF;
                break;
            case 2: // Word input 
                cmk->BYTE[act->PORT.BIT.BPOS]     = 0xFF;
                cmk->BYTE[act->PORT.BIT.BPOS + 1] = 0xFF;
                break;
            case 3: // Long word input 
                cmk->BYTE[act->PORT.BIT.BPOS]     = 0xFF;
                cmk->BYTE[act->PORT.BIT.BPOS + 1] = 0xFF;
                cmk->BYTE[act->PORT.BIT.BPOS + 2] = 0xFF;
                cmk->BYTE[act->PORT.BIT.BPOS + 3] = 0xFF;
                break;
            }
            // DS conflict ID list number 
            if (act->SID == DS_X_POWERTRAIN_ID) { // DS-only rules 
                ds_xross_pt_index = i;  // Retain 
                rout_map.ID[act->SID].BYTE = 0x11;
            }
        }
    } else if (i == FLASH_BLANK) { /* No saved information
                                    * Default setting call*/
        defset_rootmap();    // Map initial value 
        defset_confecu();    // Period / event initial value 
        defset_extlist_ex(); // External I/O definition initial value via communication 
    }
    // Frame data initial value 
    defset_framedat();
    // First event registration 
    for (i = 0; i < conf_ecu.CNT; i++) {
        if (conf_ecu.LIST[i].ID.BIT.ENB != 0) { // Period / Event 
            j = conf_ecu.LIST[i].ID.BIT.SID;
            can_id_event(j, 0);
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * can_pin_init
 * 
 * Outline
 *     CAN port setting
 *
 * Argument
 *     None
 *
 * Description
 *     Initialization of port conditions used for CAN
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void can_pin_init(void)
{
    SYSTEM.PRCR.WORD   = 0xA503; // Port setting permission 
    MPC.PWPR.BIT.B0WI  = 0;
    MPC.PWPR.BIT.PFSWE = 1;

#ifdef  __LFY_RX63N__
    // CAN1 pin setting 
    PORT5.PMR.BIT.B5  = 0;    // Peripheral functions 
    PORT5.PMR.BIT.B4  = 0;    // Peripheral functions 
    PORT5.PODR.BIT.B5 = 1;    // P55 -- CRX1 
    PORT5.PODR.BIT.B4 = 1;    // P54 -- CTX1 
    PORT5.PDR.BIT.B5  = 0;    // P55 -- CRX1 
    PORT5.PDR.BIT.B4  = 1;    // P54 -- CTX1 
    MPC.P55PFS.BYTE   = 0x10; // CRX1 
    MPC.P54PFS.BYTE   = 0x10; // CTX1 
    PORT5.PMR.BIT.B5  = 1;    // Peripheral functions 
    PORT5.PMR.BIT.B4  = 1;    // Peripheral functions 
#else // ifdef  __LFY_RX63N__
    // CAN driver S terminal control port 
    PORT6.PODR.BYTE  = 0xF0; // Port initialization 
    PORT6.PDR.BIT.B6 = 1;
    PORT6.PDR.BIT.B7 = 0;
    PORT6.PDR.BIT.B0 = 1;    // P60 -- Port-out CAN0S 
    PORT6.PDR.BIT.B1 = 1;    // P61 -- Port-out CAN1S 
    PORT6.PDR.BIT.B2 = 1;    // P62 -- Port-out CAN2S 
    PORT6.PDR.BIT.B3 = 1;    // P63 -- Port-out CAN3S 
    PORT6.PDR.BIT.B5 = 1;    // P65 -- Port-out LED 

    // ID check LED 
    PORTE.PMR.BIT.B0  = 0;   // PORT 
    PORTE.PODR.BIT.B0 = 1;   // PE0 
    PORTE.PDR.BIT.B0  = 1;   // LED output 

    // CAN0 pin setting 
    PORT3.PMR.BIT.B3  = 0;    // Peripheral functions 
    PORT3.PMR.BIT.B2  = 0;    // Peripheral functions 
    PORT3.PODR.BIT.B3 = 1;    // P32 -- CRX0 
    PORT3.PODR.BIT.B2 = 1;    // P32 -- CTX0 
    PORT3.PDR.BIT.B3  = 0;    // P33 -- CRX0 
    PORT3.PDR.BIT.B2  = 1;    // P32 -- CTX0 
    MPC.P33PFS.BYTE   = 0x10; // CRX0 
    MPC.P32PFS.BYTE   = 0x10; // CTX0 
    PORT3.PMR.BIT.B3  = 1;    // Peripheral functions 
    PORT3.PMR.BIT.B2  = 1;    // Peripheral functions 

    // CAN1 pin setting 
    PORT5.PMR.BIT.B5  = 0;    // Peripheral functions 
    PORT5.PMR.BIT.B4  = 0;    // Peripheral functions 
    PORT5.PODR.BIT.B5 = 1;    // P55 -- CRX1 
    PORT5.PODR.BIT.B4 = 1;    // P54 -- CTX1 
    PORT5.PDR.BIT.B5  = 0;    // P55 -- CRX1 
    PORT5.PDR.BIT.B4  = 1;    // P54 -- CTX1 
    MPC.P55PFS.BYTE   = 0x10; // CRX1 
    MPC.P54PFS.BYTE   = 0x10; // CTX1 
    PORT5.PMR.BIT.B5  = 1;    // Peripheral functions 
    PORT5.PMR.BIT.B4  = 1;    // Peripheral functions 

    // CAN2 pin setting 
    PORT6.PMR.BIT.B7  = 0;    // Peripheral functions 
    PORT6.PMR.BIT.B6  = 0;    // Peripheral functions 
    PORT6.PODR.BIT.B7 = 1;    // P67 -- CRX2 
    PORT6.PODR.BIT.B6 = 1;    // P66 -- CTX2 
    PORT6.PDR.BIT.B7  = 0;    // P67 -- CRX2 
    PORT6.PDR.BIT.B6  = 1;    // P66 -- CTX2 
    MPC.P67PFS.BYTE   = 0x10; // CRX2 
    MPC.P66PFS.BYTE   = 0x10; // CTX2 
    PORT6.PMR.BIT.B7  = 1;    // Peripheral functions 
    PORT6.PMR.BIT.B6  = 1;    // Peripheral functions 
#endif // ifdef  __LFY_RX63N__

    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI  = 1;
    SYSTEM.PRCR.WORD   = 0xA500; // Port setting prohibited 
}

/* ---------------------------------------------------------------------------------------
 * can_init
 * 
 * Outline
 *     CAN module register setting
 *
 * Argument
 *     int ch  Initialization channel
 *     int bps Communication speed (bps) For cars:500kbps
 *
 * Description
 *     Initialization of CAN module operating conditions
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void can_init(int ch, int bps)
{
    int i, j;
    int x, tbit, tseg1, tseg2, sjw, brp, fc;
    uint32_t lwk;
    volatile struct st_can  __evenaccess 
        *can_block_p = CAN_CHANNELS[ch];

    if (ch < 3) { /* Valid only for CPU built-in CAN0, 1, 2
                   * CAN port initialization*/
        logging("CAN%d Init\r",ch);
        lwk = R_CAN_Create(ch); // CAN module initialization 
        if (lwk != R_CAN_OK) {
            logging("CAN_Create = %08lX\r",lwk);
        } else {
            logging("CAN_Create OK\r");
        }
        // Reset execution 
        lwk = R_CAN_Control(ch, RESET_CANMODE); //RESET mode setting 
        if (lwk != R_CAN_OK) {
            logging("CAN_Control = %08lX\r",lwk);
        } else {
            logging("R_CAN_Control RESET_CANMODE\r");
        }
        // Setting mode 
        lwk = R_CAN_Control(ch, HALT_CANMODE); //HALT mode setting 
        if (lwk != R_CAN_OK) {
            logging("CAN_Control = %08lX\r",lwk);
        } else {
            logging("R_CAN_Control HALT_CANMODE\r");
        }

        /* Communication speed setting
         * BRP = 1 to 1024(0 to 1023)
         * SS = 1
         * TSEG1 = 4 to 16(3 to 15) : TSEG2 = 2 to 8(1 to 7)
         * SJW = 1 to 4(0 to 3)
         * *TSEG1 > TSEG2 ≧ SJW
         * Minimum 4 : 2 : 1 TCANBIT = SS + TSEG1 + TSEG2 = 1 + 4 + 2 = 5 〜 1 + 16 + 8 = 25
         *
         * PCLK = 48,000,000 Hz
         * BPS  =    500,000 bps
         *
         * BRP = PCLK / BPS / TBIT  *Ask to be TBIT = 16TQ (5 to 25TQ)
         *  = 48000000 / 500000 = 96 / 16 = 6
         *  = 6
         *
         * tBit= PCLK / BPS / BRP = 96 / 6 = 16Tq
         *
         * TSEG2 = (tBit - SS) / 4 = (16 - 1) / 4 = 3.75
         *    = 3
         * TSEG1 = tBit - SS - TSEG2 = 16 - 1 - 3 = 12
         *    = 12
         * SJW   = TSEG2 / 2 = 3 / 2 = 1.5
         *    = 1
         **/
        for (x = 25; x > 4; x--) {
            if ((48000000 % x) != 0) {
                continue;
            }
            fc = 48000000 / x;
            if ((fc % bps) > (bps * 17 / 1000)) {
                continue; // Error less than 1.7% 
            }
            brp = 48000000 / bps / x; // 1025 > brp > 0 
            if (brp < 1) {
                continue; // Out of range 
            }
            if (brp > 1024) {
                continue; // Out of range 
            }
            tbit = 48000000 / bps / brp; // 26 > tbit > 4 
            if (tbit != x) {
                continue; // Back calculation mismatch 
            }
            if (tbit < 5) {
                continue; // Out of range 
            }
            if (tbit > 25) {
                continue; // Out of range 
            }
            tseg2 = tbit / 3; // 9 > tseg2 > 1 
            if (tseg2 < 2) {
                continue; // Out of range 
            }
            if (tseg2 > 8) {
                continue; // Out of range 
            }
            tseg1 = tbit - 1 - tseg2; // 17 > tseg1 > 3 
            if (tseg1 <= tseg2) {
                continue; // Out of range 
            }
            if (tseg1 > 16) {
                continue; // Out of range 
            }
            sjw = (tseg2 + 1) / 2; // 5 > sjw > 0 
            if (sjw < 1) {
                continue; // Out of range 
            }
            if (sjw > 4) {
                continue; // Out of range 
            }
            break;
        }
        if (x == 4) { // As it failed, initialize to 500kbps 
            logging("can_init: invalid parameter (%ld)\r", bps);
            brp   = 6;
            tbit  = 16;
            tseg1 = 10;
            tseg2 = 5;
            sjw   = 4;
        }
        can_block_p->BCR.BIT.CCLKS = 0;    // PCLK(48MHz) 
        can_block_p->BCR.BIT.BRP   = brp - 1;
        can_block_p->BCR.BIT.TSEG1 = tseg1 - 1;
        can_block_p->BCR.BIT.TSEG2 = tseg2 - 1;
        can_block_p->BCR.BIT.SJW   = sjw - 1;
        logging(
            "BPS=%d SS=1 BRP=%d TSEG1=%d TSEG2=%d SJW=%d TBIT=%d\r",
            bps, brp, tseg1, tseg2, sjw, tbit
        );

        // Mailbox data initialization 
        for (i = 0; i < 32; i++) {
            can_block_p->MKR[i].LONG   = 0;
            can_block_p->MB[i].ID.LONG = 0;
        }
        // Mask, filter disable 
        can_block_p->MKIVLR.LONG = 0;   //0x0000FFFF; 

        // Mailbox control initialization 
        for (i = 0; i < 32; i++) {
            while (can_block_p->MCTL[i].BYTE != 0) {
                can_block_p->MCTL[i].BYTE = 0;
            }
        }
        // Start operation 
        can_block_p->MIER.LONG = 0xFFFFFFFF;      // Interrupt enable 
        lwk = R_CAN_Control(ch, OPERATE_CANMODE); // OPERATE mode setting 
        if (lwk != R_CAN_OK) {
            logging("R_CAN_Control = %08lX\r", lwk);
        } else {
            logging("R_CAN_Control OPERATE_CANMODE\r");
        }
        // Receive permission 
        for (i = 16; i < 32; i++) {
            can_block_p->MCTL[i].BYTE = 0x40; // MB16 to 32 are for reception only 
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * ecu_timeup
 * 
 * Outline
 *     ECU timer counter update processing
 *
 * Argument
 *     None
 *
 * Description
 *     Called by 1ms timer
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void ecu_timeup(void)
{
    timer_count++;
    status_timer++;
    after_call(0, -1, ecu_timeup);  // Fast timer call 
}

void ecu_rxmb_proc(void)
{
    int ch;
    for (ch = 0; ch < 3; ch++) {
        while (rxmb_buf[ch].WP != rxmb_buf[ch].RP) {
            can_recv_frame(
                ch,
                (void *)&rxmb_buf[ch].MB[rxmb_buf[ch].RP++]
            ); // Get received data 
            rxmb_buf[ch].RP &= (RX_MB_BUF_MAX-1);
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * ecu_job
 * 
 * Outline
 *     ECU operation sequence
 *
 * Argument
 *     None
 *
 * Description
 *     Call from main function. Performs CAN frame transmission management,
 *     scheduled processing, and event processing.
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void ecu_job(void)
{
    int i, ch, t;

    if (job > 1) {
        ecu_rxmb_proc();   // Check reception every time 
        send_mbox_frame(); // Check transmission every time 
    }

    switch (job) {
    case 0: // Initialization process 
        job++;
        logging("Init ECU\r");
        ecu_init();     // ECU buffer initialization 
        can_pin_init(); // CPU pin setting 
        timer_count  = 0;
        status_timer = 0;
        ecu_timeup();
    case 1:
        if (timer_count > 0) { // With timer update 
            led         += timer_count;
            timer_count = 0;   // Time count initialization 
            if (led >= 500) {
                led -= 500;
                PORT6.PODR.BYTE ^= 0x20; // LED inversion 
                logging("Start ECU\r");
                job++;
#ifdef  __LFY_RX63N__
                i = CAN_TEST_LFY_CH;
#else
                for (i = 0; i < CAN_CH_MAX; i++)
#endif
                {
                    can_init(i, 500000); // CAN port initialization 
                }
            }
        }
        break;
    /* ------------------------------------------
     * The following is iterative process
     *------------------------------------------*/
    case 2: // I/O update 
        job++;
        extern_io_update_ex(); // I/O via RS-232C communication 
        break;
    case 3: // 1ms cycle processing 
        job = 5;
        if (timer_count > 0) { // With timer update 
            t = timer_count;
            timer_count = 0;   // Time count initialization 
            led += t;
            if (led >= 500) {
                led -= 500;
                PORT6.PODR.BYTE ^= 0x20; // LED inversion 
            }
            can_timer_send(t); // Time-up processing 
        }
        break;
    case 4: // CAN transmission processing 
        job++;
        break;
    case 5: // CAN receiving processing 
        job++;
#ifndef CAN_RX_INT_ENB
        CAN0.MSMR.BYTE = 0; // SENTDATA search for received MB 
        while (CAN0.MSSR.BIT.SEST == 0) { // With results 
            i = (int)CAN0.MSSR.BIT.MBNST;
            if (!CAN0.MCTL[i].BIT.RX.INVALDATA) {
                if (CAN0.MCTL[i].BIT.RX.NEWDATA) {
                    can_recv_frame(0, (void *)&CAN0.MB[i]); // Get received data 
                }
                CAN0.MCTL[i].BYTE = 0x40; // Resuming reception 
            }
        }
        CAN1.MSMR.BYTE = 0; // Search SENTDATA for received MB 
        while (CAN1.MSSR.BIT.SEST == 0) { // With results 
            i = (int)CAN1.MSSR.BIT.MBNST;
            if (!CAN1.MCTL[i].BIT.RX.INVALDATA) {
                if (CAN1.MCTL[i].BIT.RX.NEWDATA) {
                    can_recv_frame(1, (void *)&CAN1.MB[i]); // Get received data 
                }
                CAN1.MCTL[i].BYTE = 0x40; // Resuming reception 
            }
        }
        CAN2.MSMR.BYTE = 0; // Search SENTDATA for received MB 
        while (CAN2.MSSR.BIT.SEST == 0) { // With results 
            i = (int)CAN2.MSSR.BIT.MBNST;
            if (!CAN2.MCTL[i].BIT.RX.INVALDATA) {
                if (CAN2.MCTL[i].BIT.RX.NEWDATA) {
                    can_recv_frame(2, (void *)&CAN2.MB[i]); // Get received data 
                }
                CAN2.MCTL[i].BYTE = 0x40; // Resuming reception 
            }
        }
        break;
#endif // ifndef CAN_RX_INT_ENB
    case 6:     // RS-232C transmission processing 
        job = 2;
        if (stat_update_id < EX_IO_MAX) { // Send continuation notification 
            if (sci_txbytes(stat_comm) == 0) { // Free transmission buffer 
                int j, k;
                int r;
                char buf[128];
                r = 0;
                // One message is sent as about 80 characters 
                for (; r < 80 && stat_update_id < EX_IO_MAX; stat_update_id++) { // Send one line 
                    if (exio_chg[stat_update_id] != 0) { // Data update ID 
                        exio_chg[stat_update_id] = 0;
                        if (r == 0) { // Beginning of line starts with "EXU" 
                            buf[r++] = 'E';
                            buf[r++] = 'X';
                            buf[r++] = 'U';
                        }
                        // 2-digit I/O-ID code 
                        k           = stat_update_id;
                        buf[r++]    = ' ';
                        buf[r++]    = HEX_CHAR[((k >> 4) & 15)];
                        buf[r++]    = HEX_CHAR[(k & 15)];
                        // Update data 4 bytes 
                        k   = exiosts.DATA[stat_update_id].INTE;
                        j   = 8;
                        if (((k >> 28) & 15) == 0) {
                            j--;
                            if (((k >> 24) & 15) == 0) {
                                j--;
                                if (((k >> 20) & 15) == 0) {
                                    j--;
                                    if (((k >> 16) & 15) == 0) {
                                        j--;
                                        if (((k >> 12) & 15) == 0) {
                                            j--;
                                            if (((k >> 8) & 15) == 0) {
                                                j--;
                                                if (((k >> 4) & 15) == 0) {
                                                    j--;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        switch (j) {
                        case 8:
                            buf[r++] = HEX_CHAR[((k >> 28) & 15)];
                        case 7:
                            buf[r++] = HEX_CHAR[((k >> 24) & 15)];
                        case 6:
                            buf[r++] = HEX_CHAR[((k >> 20) & 15)];
                        case 5:
                            buf[r++] = HEX_CHAR[((k >> 16) & 15)];
                        case 4:
                            buf[r++] = HEX_CHAR[((k >> 12) & 15)];
                        case 3:
                            buf[r++] = HEX_CHAR[((k >> 8) & 15)];
                        case 2:
                            buf[r++] = HEX_CHAR[((k >> 4) & 15)];
                        case 1:
                            buf[r++] = HEX_CHAR[(k & 15)];
                        case 0:
                            break;
                        }
                    }
                }
                if (r > 0) {
                    buf[r++]    = '\r';
                    buf[r]      = 0;
                    sci_puts(stat_comm, buf);       // Transmit execution 
                }
            }
        } else if (status_timer >= 500) { // Forced all transmission at 500ms cycle 
            status_timer    = 0;    // Timer clear 
            stat_update_id  = 0;    // Transmission start ID set 
            exio_chg_mark   = 0;    // Change flag clear 
            for (i = 0; i < ext_list_count; i++) {
                exio_chg[(ext_list[i].PORT.BIT.NOM)] = 1;
            }
        } else if (sci_txbytes(stat_comm) == 0 && exio_chg_mark > 0) { // Free transmission buffer, Individual transmission with status change 
            status_timer    = 0;    // Timer clear 
            stat_update_id  = 0;    // Transmission start ID set 
            exio_chg_mark   = 0;    // Change flag clear 
        }
        break;
    default:
        job = 2;
        break;
    }
}

/* ---------------------------------------------------------------------------------------
 * ecu_status
 * 
 * Outline
 *     ECU operating parameter notification
 *
 * Argument
 *     None
 *
 * Description
 *     Transmit current parameter value to COM1
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void SendPC(char *msg);

void ecu_status(char *cmd)
{
    int     i, j;
    int     ch, mb, id;
    char    tx[39], c;

    if (cmd == 0 || *cmd == 0) {
        return;
    }
    while (*cmd == ' ') {
        cmd++;
    }
    switch (*cmd++) {
    case 'L':   // conf_ecu variable display 
        logging("conf_ecu WP=%d TOP=%d CNT=%d\r", conf_ecu.WP, conf_ecu.TOP, conf_ecu.CNT);
        for (i = 0; i < MESSAGE_MAX; i++) {
            if (conf_ecu.LIST[i].ID.BIT.ENB != 0) {
                logging(
                            "No.%d RTR=%d SID=%03X DLC=%d ENB=%d REP=%d NXT=%d TIM=%d CNT=%d "
                            "DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r",
                            i,
                            (int)conf_ecu.LIST[i].ID.BIT.RTR,
                            (int)conf_ecu.LIST[i].ID.BIT.SID,
                            (int)conf_ecu.LIST[i].ID.BIT.DLC,
                            (int)conf_ecu.LIST[i].ID.BIT.ENB,
                            (int)conf_ecu.LIST[i].ID.BIT.REP,
                            (int)conf_ecu.LIST[i].ID.BIT.NXT,
                            (int)conf_ecu.LIST[i].TIMER.WORD.TIME,
                            (int)conf_ecu.LIST[i].TIMER.WORD.CNT,
                            (int)can_buf.ID[i].BYTE[0], (int)can_buf.ID[i].BYTE[1], 
                            (int)can_buf.ID[i].BYTE[2], (int)can_buf.ID[i].BYTE[3],
                            (int)can_buf.ID[i].BYTE[4], (int)can_buf.ID[i].BYTE[5], 
                            (int)can_buf.ID[i].BYTE[6], (int)can_buf.ID[i].BYTE[7]
                );
            }
        }
        break;
    case 'W':   // wait_tup variable display 
        logging("wait_tup WP=%d TOP=%d CNT=%d\r", wait_tup.WP, wait_tup.TOP, wait_tup.CNT);
        for (i = 0; i < MESSAGE_MAX; i++) {
            if (wait_tup.LIST[i].ID.BIT.ENB != 0) {
                logging(
                            "No.%d RTR=%d SID=%03X DLC=%d ENB=%d REP=%d NXT=%d TIM=%d CNT=%d "
                            "DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r",
                            i,
                            (int)wait_tup.LIST[i].ID.BIT.RTR,
                            (int)wait_tup.LIST[i].ID.BIT.SID,
                            (int)wait_tup.LIST[i].ID.BIT.DLC,
                            (int)wait_tup.LIST[i].ID.BIT.ENB,
                            (int)wait_tup.LIST[i].ID.BIT.REP,
                            (int)wait_tup.LIST[i].ID.BIT.NXT,
                            (int)conf_ecu.LIST[i].TIMER.WORD.TIME,
                            (int)conf_ecu.LIST[i].TIMER.WORD.CNT,
                            (int)can_buf.ID[i].BYTE[0], (int)can_buf.ID[i].BYTE[1], 
                            (int)can_buf.ID[i].BYTE[2], (int)can_buf.ID[i].BYTE[3],
                            (int)can_buf.ID[i].BYTE[4], (int)can_buf.ID[i].BYTE[5], 
                            (int)can_buf.ID[i].BYTE[6], (int)can_buf.ID[i].BYTE[7]
                );
            }
        }
        break;

    case 'S':   // send_msg variable display 
        for (ch = 0; ch < CAN_CH_MAX; ch++) {
            for (mb = 0; mb < MESSAGE_BOXS; mb++) {
                logging(
                    "send_msg[%d].BOX[%d] WP=%d TOP=%d CNT=%d\r",
                    ch, mb, send_msg[ch].BOX[mb].WP, 
                    send_msg[ch].BOX[mb].TOP, send_msg[ch].BOX[mb].CNT
                );
                for (i = 0; i < MESSAGE_MAX; i++) {
                    if (send_msg[ch].BOX[mb].MSG[i].ID.BIT.ENB != 0) {
                        logging(
                                    "No.%d RTR=%d SID=%03X DLC=%d ENB=%d NXT=%d "
                                    "DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r",
                                    i,
                                    (int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.RTR,
                                    (int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.SID,
                                    (int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.DLC,
                                    (int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.ENB,
                                    (int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.NXT,
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[0], 
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[1],
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[2], 
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[3],
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[4], 
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[5],
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[6], 
                                    (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[7]
                        );
                    }
                }
            }
        }
        break;
    }
}

/* ---------------------------------------------------------------------------------------
 * ecu_get_command
 * 
 * Outline
 *     Get ECU frame data
 *
 * Argument
 *     char *cmd Command string
 *
 * Description
 *     Returns the current specified ID frame data
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void ecu_get_command(char *cmd)
{
    int i, j;
    int id;
    char tx[39], c;

    if (cmd == 0 || *cmd == 0) {
        return;
    }
    while (*cmd == ' ') {
        cmd++;
    }
    if (*cmd < '0' || *cmd > '7') {
        return;
    }
    SendPC("ECU ");
    while (*cmd >= '0' && *cmd <= '7') {
        id = 0;
        for (i = 0; i < 4; i++) {
            c = *cmd;
            if (c >= 'a' && c <= 'f') {
                c -= 0x27;
            }
            if (c >= 'A' && c <= 'F') {
                c -= 7;
            }
            if (c < 0x30 || c > 0x3F) {
                break;
            }
            id = (id << 4) | (int)(c & 0x0F);
            cmd++;
        }
        if (i == 3 && id < CAN_ID_MAX) { // ID normal 
            j       = 0;
            tx[j++] = HEX_CHAR[(id >> 8) & 0x0F];
            tx[j++] = HEX_CHAR[(id >> 4) & 0x0F];
            tx[j++] = HEX_CHAR[id & 0x0F];
            for (i = 0; i < 8; i++) {
                c = can_buf.ID[id].BYTE[i];
                tx[j++] = HEX_CHAR[(c >> 4) & 0x0F];
                tx[j++] = HEX_CHAR[c & 0x0F];
            }
            if (*cmd >= ' ') {
                tx[j++] = ' ';
            }
            tx[j++] = 0;
            SendPC(tx);
        }
        while (*cmd == ' ') {
            cmd++;
        }
    }
    SendPC("\r");
}

/* ---------------------------------------------------------------------------------------
 * ecu_set_command
 * 
 * Outline
 *     ECU frame data rewriting
 *
 * Argument
 *     char *cmg Command string
 *
 * Description
 *     Rewrite frame data of specified ID
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void ecu_set_command(char *cmd)
{
    int i, j, f;
    int id, tp;
    int xid[32];
    int xwp = 0;
    char tx[39], c;
    unsigned char d;

    if (cmd == 0 || *cmd == 0) {
        return;
    }
    while (*cmd == ' ') {
        cmd++;
    }
    if (*cmd < '0' || *cmd > '7') {
        return;
    }
    // Rewriting process 
    tp = 0; // Initialize delay time 
    while (*cmd >= '0' && *cmd <= '7') {
        // Get target ID 
        id = 0;
        // Get binary data hexadecimal 3 digits (0x000 to FFF) from hexadecimal character string 
        for (i = 0; i < 3; i++) {
            c = *cmd;
            if (c >= 'a' && c <= 'f') {
                c -= 0x27;
            }
            if (c >= 'A' && c <= 'F') {
                c -= 7;
            }
            if (c < 0x30 || c > 0x3F) {
                break;
            }
            id = (id << 4) | (int)(c & 0x0F);
            cmd++;
        }
        // ID check 
        if (i == 3 && id < CAN_ID_MAX) { // ID normal, data processing 
            f = 0;
            // Get binary data from hexadecimal string 
            for (i = 0; i < 8; i++) {
                c = *cmd;
                if (c >= 'a' && c <= 'f') {
                    c -= 0x27;
                }
                if (c >= 'A' && c <= 'F') {
                    c -= 7;
                }
                if (c < 0x30 || c > 0x3F) {
                    break;
                }
                d = (unsigned char)(c & 0x0F);
                cmd++;
                c = *cmd;
                if (c >= 'a' && c <= 'f') {
                    c -= 0x27;
                }
                if (c >= 'A' && c <= 'F') {
                    c -= 7;
                }
                if (c < 0x30 || c > 0x3F) {
                    break;
                }
                d = (d << 4) | (unsigned char)(c & 0x0F);
                cmd++;
                can_buf.ID[id].BYTE[i] = d;
                f++;
            }
            if (f > 0) { // With data rewriting 
                i = can_tp_job(-1, id, &can_buf.ID[id].BYTE);
                if (i > 0) { // With response 
                    xid[xwp++] = i;
                } else { // Return as is 
                    tp          += can_id_event(id, tp);
                    xid[xwp++]  = id;
                }
                if (xwp >= 32) {
                    break; // Quantity limit 
                }
            }
        }
        while (*cmd == ' ') {
            cmd++;
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * ecu_put_command
 * 
 * Outline
 *     ECU frame data transmission
 *
 * Argument
 *     char *cmg Command string
 *
 * Description
 *     Rewrite frame data of specified ID
 *
 * Return
 *     None
 *---------------------------------------------------------------------------------------*/
void ecu_put_command(char *cmd)
{
    ECU_CYC_EVE mbox;
    int i, j, f;
    int id, tp;
    char c;
    unsigned char d;

    if (cmd == 0 || *cmd == 0) {
        return;
    }
    while (*cmd == ' ') {
        cmd++;
    }
    if (*cmd < '0' || *cmd > '7') {
        return;
    }
    // Rewriting process 
    while (*cmd >= '0' && *cmd <= '7') {
        // Get target ID 
        id = 0;
        for (i = 0; i < 3; i++) {
            c = *cmd;
            if (c >= 'a' && c <= 'f') {
                c -= 0x27;
            }
            if (c >= 'A' && c <= 'F') {
                c -= 7;
            }
            if (c < 0x30 || c > 0x3F) {
                break;
            }
            id = (id << 4) | (int)(c & 0x0F);
            cmd++;
        }
        if (i == 3 && id < CAN_ID_MAX) { // ID normal, data processing 
            f = 0;
            for (i = 0; i < 8; i++) {
                c = *cmd;
                if (c >= 'a' && c <= 'f') {
                    c -= 0x27;
                }
                if (c >= 'A' && c <= 'F') {
                    c -= 7;
                }
                if (c < 0x30 || c > 0x3F) {
                    break;
                }
                d = (unsigned char)(c & 0x0F);
                cmd++;
                c = *cmd;
                if (c >= 'a' && c <= 'f') {
                    c -= 0x27;
                }
                if (c >= 'A' && c <= 'F') {
                    c -= 7;
                }
                if (c < 0x30 || c > 0x3F) {
                    break;
                }
                d = (d << 4) | (unsigned char)(c & 0x0F);
                cmd++;
                can_buf.ID[id].BYTE[i] = d;
                f++;
            }
            if (f > 0) { // With data rewriting 
                i = can_tp_job(-1, id, &can_buf.ID[id].BYTE);
                if (i == 0) { // No response 
                    mbox.ID.LONG    = 0;
                    mbox.TIMER.LONG = 0;
                    mbox.ID.BIT.SID = id;
                    mbox.ID.BIT.ENB = 1;
                    mbox.ID.BIT.DLC = f;
                    can_send_proc(&mbox);
                }
            } else { // Remote frame issuance 
                mbox.ID.LONG    = 0;
                mbox.TIMER.LONG = 0;
                mbox.ID.BIT.SID = id;
                mbox.ID.BIT.ENB = 1;
                mbox.ID.BIT.RTR = 1;
                mbox.ID.BIT.DLC = 8;
                can_send_proc(&mbox);
            }
        }
        while (*cmd == ' ') {
            cmd++;
        }
    }
}

/* ---------------------------------------------------------------------------------------
 * ecu_input_update
 * 
 * Outline
 *     Updating external input information via communication
 *
 * Argument
 *     char *cmg Command string
 *
 * Description
 *     Rewrite data in specified input buffer
 *
 * Return
 *     None
*---------------------------------------------------------------------------------------*/
void ecu_input_update(char *cmd)
{
    int i, f;
    int id;
    char c;
    unsigned long d;

    if (cmd == 0 || *cmd == 0) {
        return;
    }
    // Rewriting process 
    while (*cmd != 0) {
        while (*cmd == ' ') {
            cmd++;
        }
        // Get target ID 
        id = 0;
        for (i = 0; i < 2; i++) {
            c = *cmd;
            if (c >= 'a' && c <= 'f') {
                c -= 0x27;
            }
            if (c >= 'A' && c <= 'F') {
                c -= 7;
            }
            if (c < 0x30 || c > 0x3F) {
                break;
            }
            id = (id << 4) | (int)(c & 0x0F);
            cmd++;
        }
        if (i == 2 && id < EX_IO_MAX) { // ID normal, data processing 
            f = 0;
            d = 0;
            for (i = 0; i < 8; i++) {
                c = *cmd;
                if (c >= 'a' && c <= 'f') {
                    c -= 0x27;
                }
                if (c >= 'A' && c <= 'F') {
                    c -= 7;
                }
                if (c < 0x30 || c > 0x3F) {
                    break;
                }
                d = (d << 4) | (unsigned char)(c & 0x0F);
                cmd++;
                f++;
            }
            if (f > 0) { // With data rewriting 
                exiosts.DATA[id].LONG = d;
            }
        } else {
            break;
        }
    }
}

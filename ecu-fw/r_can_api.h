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
/***********************************************************************************************************************
* File Name    : r_can_api.h
* Version      : 2.00
* Description  : Exports the Renesas CAN API
************************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 22.3.2010  1.00     For RX62N with new CAN API.
*         : 15.4.2010           R_CAN_Control 'Enter Sleep' added.
*                               R_CAN_TxStopMsg added.
*         : 15.06.2011          Added typedefs and defines to support multiple CAN channel selection in RX630
*         : 8.12.2011  2.00     Added support for extended ID frame mode.
**********************************************************************************************************************/

#ifndef _R_CAN_API_H
#define _R_CAN_API_H


#include "config_r_can_rapi.h"

#define CAN_RX_INT_ENB


// Standard data frame message definition object. 
typedef struct {
    uint32_t    id;
    uint8_t     dlc;
    uint8_t     data[8];
    uint16_t    time_st;
} can_frame_t;

#define __evenaccess

// A pointer to the CAN peripheral registers memory map structure. 
typedef volatile struct st_can __evenaccess * can_st_ptr;

// A set of pointers to the registers of the I/O ports assigned to CAN pins. 
typedef struct {
    volatile __evenaccess unsigned char *   p_CAN_Rx_Pin_MPC;
    volatile __evenaccess unsigned char *   p_CAN_Rx_Pin_PMR;
    volatile __evenaccess unsigned char *   p_CAN_Rx_Pin_PDR;
    volatile __evenaccess unsigned char *   p_CAN_Rx_Pin_PIDR;
    volatile uint8_t                        Rx_Pin_mask;
    volatile __evenaccess unsigned char *   p_CAN_Tx_Pin_MPC;
    volatile __evenaccess unsigned char *   p_CAN_Tx_Pin_PMR;
    volatile __evenaccess unsigned char *   p_CAN_Tx_Pin_PDR;
    volatile __evenaccess unsigned char *   p_CAN_Tx_Pin_PODR;
    volatile uint8_t                        Tx_Pin_mask;
} CAN_port_map_t;

/******************************************************************************
*  Definitions Used to make the port pin selection logic work
*****************************************************************************/
#define    P14        14
#define    P15        15
#define    P32        32
#define    P33        33
#define    P54        54
#define    P55        55
#define    P66        66
#define    P67        67

/* Define information necessary to stuff the CAN pin map based on user selection of port/pin number
 * CAN0 RX port configuration*/
#ifdef CAN0_RX_PORT
#if (CAN0_RX_PORT) && (CAN0_RX_PORT == P33)
#define p_CAN0_RX_PIN_MPC         &MPC.P33PFS.BYTE
#define p_CAN0_RX_PIN_PMR         &PORT3.PMR.BYTE
#define p_CAN0_RX_PIN_PDR         &PORT3.PDR.BYTE
#define p_CAN0_RX_PIN_PIDR        &PORT3.PIDR.BYTE
#define   CAN0_RX_PIN_MASK        0x08     // bit 3 
#elif (CAN0_RX_PORT) && (CAN0_RX_PORT == PD2)
#define p_CAN0_RX_PIN_MPC         &MPC.PD2PFS.BYTE
#define p_CAN0_RX_PIN_PMR         &PORTD.PMR.BYTE
#define p_CAN0_RX_PIN_PDR         &PORTD.PDR.BYTE
#define p_CAN0_RX_PIN_PIDR        &PORTD.PIDR.BYTE
#define   CAN0_RX_PIN_MASK        0x04     // bit 2 
#endif // if (CAN0_RX_PORT) && (CAN0_RX_PORT == P33)
#else // ifdef CAN0_RX_PORT
#define p_CAN0_RX_PIN_MPC         0
#define p_CAN0_RX_PIN_PMR         0
#define p_CAN0_RX_PIN_PDR         0
#define p_CAN0_RX_PIN_PIDR        0
#define   CAN0_RX_PIN_MASK        0
#endif // ifdef CAN0_RX_PORT

// CAN0 TX port configuration 
#ifdef CAN0_TX_PORT
#if (CAN0_TX_PORT) && (CAN0_TX_PORT == P32)
#define p_CAN0_TX_PIN_MPC         &MPC.P32PFS.BYTE
#define p_CAN0_TX_PIN_PMR         &PORT3.PMR.BYTE
#define p_CAN0_TX_PIN_PDR         &PORT3.PDR.BYTE
#define p_CAN0_TX_PIN_PODR        &PORT3.PODR.BYTE
#define   CAN0_TX_PIN_MASK        0x04     // bit 2 
#elif (CAN0_TX_PORT) && (CAN0_TX_PORT == PD1)
#define p_CAN0_TX_PIN_MPC         &MPC.PD1PFS.BYTE
#define p_CAN0_TX_PIN_PMR         &PORTD.PMR.BYTE
#define p_CAN0_TX_PIN_PDR         &PORTD.PDR.BYTE
#define p_CAN0_TX_PIN_PODR        &PORTD.PODR.BYTE
#define   CAN0_TX_PIN_MASK        0x02     // bit 1 
#endif // if (CAN0_TX_PORT) && (CAN0_TX_PORT == P32)
#else // ifdef CAN0_TX_PORT
#define p_CAN0_TX_PIN_MPC         0
#define p_CAN0_TX_PIN_PMR         0
#define p_CAN0_TX_PIN_PDR         0
#define p_CAN0_TX_PIN_PODR        0
#define   CAN0_TX_PIN_MASK        0
#endif // ifdef CAN0_TX_PORT

// CAN1 RX port configuration 
#if (CAN1_RX_PORT) && (CAN1_RX_PORT == P15)
#define p_CAN1_RX_PIN_MPC         &MPC.P15PFS.BYTE
#define p_CAN1_RX_PIN_PMR         &PORT1.PMR.BYTE
#define p_CAN1_RX_PIN_PDR         &PORT1.PDR.BYTE
#define p_CAN1_RX_PIN_PIDR        &PORT1.PIDR.BYTE
#define   CAN1_RX_PIN_MASK        0x20     // bit 5 
#elif (CAN1_RX_PORT) && (CAN1_RX_PORT == P55)
#define p_CAN1_RX_PIN_MPC         &MPC.P55PFS.BYTE
#define p_CAN1_RX_PIN_PMR         &PORT5.PMR.BYTE
#define p_CAN1_RX_PIN_PDR         &PORT5.PDR.BYTE
#define p_CAN1_RX_PIN_PIDR        &PORT5.PIDR.BYTE
#define   CAN1_RX_PIN_MASK        0x20     // bit 5 
#else // if (CAN1_RX_PORT) && (CAN1_RX_PORT == P15)
#define p_CAN1_RX_PIN_MPC         0
#define p_CAN1_RX_PIN_PMR         0
#define p_CAN1_RX_PIN_PDR         0
#define p_CAN1_RX_PIN_PIDR        0
#define   CAN1_RX_PIN_MASK        0
#endif // if (CAN1_RX_PORT) && (CAN1_RX_PORT == P15)
// CAN1 TX port configuration 
#if (CAN1_TX_PORT) && (CAN1_TX_PORT == P14)
#define p_CAN1_TX_PIN_MPC         &MPC.P14PFS.BYTE
#define p_CAN1_TX_PIN_PMR         &PORT1.PMR.BYTE
#define p_CAN1_TX_PIN_PDR         &PORT1.PDR.BYTE
#define p_CAN1_TX_PIN_PODR        &PORT1.PODR.BYTE
#define   CAN1_TX_PIN_MASK        0x10     // bit 4 
#elif (CAN1_TX_PORT) && (CAN1_TX_PORT == P54)
#define p_CAN1_TX_PIN_MPC         &MPC.P54PFS.BYTE
#define p_CAN1_TX_PIN_PMR         &PORT5.PMR.BYTE
#define p_CAN1_TX_PIN_PDR         &PORT5.PDR.BYTE
#define p_CAN1_TX_PIN_PODR        &PORT5.PODR.BYTE
#define   CAN1_TX_PIN_MASK        0x10     // bit 4 
#else // if (CAN1_TX_PORT) && (CAN1_TX_PORT == P14)
#define p_CAN1_TX_PIN_MPC         0
#define p_CAN1_TX_PIN_PMR         0
#define p_CAN1_TX_PIN_PDR         0
#define p_CAN1_TX_PIN_PODR        0
#define   CAN1_TX_PIN_MASK        0
#endif // if (CAN1_TX_PORT) && (CAN1_TX_PORT == P14)

#ifdef CAN2_RX_PORT
// CAN2 RX port configuration 
#if (CAN2_RX_PORT) && (CAN2_RX_PORT == P67)
#define p_CAN2_RX_PIN_MPC         &MPC.P67PFS.BYTE
#define p_CAN2_RX_PIN_PMR         &PORT6.PMR.BYTE
#define p_CAN2_RX_PIN_PDR         &PORT6.PDR.BYTE
#define p_CAN2_RX_PIN_PIDR        &PORT6.PIDR.BYTE
#define   CAN2_RX_PIN_MASK        0x80     // bit 7 
#else // if (CAN2_RX_PORT) && (CAN2_RX_PORT == P67)
#define p_CAN2_RX_PIN_MPC         0
#define p_CAN2_RX_PIN_PMR         0
#define p_CAN2_RX_PIN_PDR         0
#define p_CAN2_RX_PIN_PIDR        0
#define   CAN2_RX_PIN_MASK        0
#endif // if (CAN2_RX_PORT) && (CAN2_RX_PORT == P67)
#endif // ifdef CAN2_RX_PORT
// CAN2 TX port configuration 
#ifdef CAN2_TX_PORT
#if (CAN2_TX_PORT == P66)
#define p_CAN2_TX_PIN_MPC         &MPC.P66PFS.BYTE
#define p_CAN2_TX_PIN_PMR         &PORT6.PMR.BYTE
#define p_CAN2_TX_PIN_PDR         &PORT6.PDR.BYTE
#define p_CAN2_TX_PIN_PODR        &PORT6.PODR.BYTE
#define   CAN2_TX_PIN_MASK        0x40     // bit 6 
#else // if (CAN2_TX_PORT == P66)
#define p_CAN2_TX_PIN_MPC         0
#define p_CAN2_TX_PIN_PMR         0
#define p_CAN2_TX_PIN_PDR         0
#define p_CAN2_TX_PIN_PODR        0
#define   CAN2_TX_PIN_MASK        0
#endif // if (CAN2_TX_PORT == P66)
#endif // ifdef CAN2_TX_PORT
#define PINFUNC_CAN 0x10    // Value to set MPC PFS registers for CAN operation. 

//** CAN API ACTION TYPES ** 
#define DISABLE                         0
#define ENABLE                          1
// Periph CAN modes 
#define EXITSLEEP_CANMODE               2
#define ENTERSLEEP_CANMODE              3
#define RESET_CANMODE                   4
#define HALT_CANMODE                    5
#define OPERATE_CANMODE                 6
// Port mode actions 
#define CANPORT_TEST_LISTEN_ONLY        7
#define CANPORT_TEST_0_EXT_LOOPBACK     8
#define CANPORT_TEST_1_INT_LOOPBACK     9
#define CANPORT_RETURN_TO_NORMAL        10

// Local sleep mode for CAN module 
#define CAN_NOT_SLEEP   0
#define CAN_SLEEP       1

// Tranceiver port pin macros. 
#define CAN_TRX_DDR(x, y)               CAN_TRX_DDR_PREPROC(x, y)
#define CAN_TRX_DDR_PREPROC(x, y)       (PORT ## x.PDR.BIT.B ## y)
#define CAN_TRX_DR(x, y)                CAN_TRX_DR_PREPROC(x, y)
#define CAN_TRX_DR_PREPROC(x, y)        (PORT ## x.PODR.BIT.B ## y)

//** CAN API return values ******************** 
#define     R_CAN_OK                (uint32_t)0x00000000
#define     R_CAN_NOT_OK            (uint32_t)0x00000001
#define     R_CAN_MODULE_STOP_ERR   (uint32_t)0x00000002
#define     R_CAN_MSGLOST           (uint32_t)0x00000004
#define     R_CAN_NO_SENTDATA       (uint32_t)0x00000008
#define     R_CAN_RXPOLL_TMO        (uint32_t)0x00000010
#define     R_CAN_BAD_CH_NR         (uint32_t)0x00000020
#define     R_CAN_SW_BAD_MBX        (uint32_t)0x00000040
#define     R_CAN_BAD_ACTION_TYPE   (uint32_t)0x00000080
// CAN peripheral timeout reasons. 
#define     R_CAN_SW_WAKEUP_ERR     (uint32_t)0x00000100
#define     R_CAN_SW_SLEEP_ERR      (uint32_t)0x00000200
#define     R_CAN_SW_HALT_ERR       (uint32_t)0x00000400
#define     R_CAN_SW_RST_ERR        (uint32_t)0x00000800
#define     R_CAN_SW_TSRC_ERR       (uint32_t)0x00001000
#define     R_CAN_SW_SET_TX_TMO     (uint32_t)0x00002000
#define     R_CAN_SW_SET_RX_TMO     (uint32_t)0x00004000
#define     R_CAN_SW_ABORT_ERR      (uint32_t)0x00008000
// CAN STATE CODES 
#define     R_CAN_STATUS_ERROR_ACTIVE   (uint32_t)0x0000001
#define     R_CAN_STATUS_ERROR_PASSIVE  (uint32_t)0x0000002
#define     R_CAN_STATUS_BUSOFF         (uint32_t)0x0000004
//****************************************************** 

// CAN Frame ID modes 
#define     STD_ID_MODE     0
#define     EXT_ID_MODE     1
#define     MIXED_ID_MODE   2

// Mailbox search modes. 
#define     RX_SEARCH       0
#define     TX_SEARCH       1
#define     MSGLOST_SEARCH  2
#define     CHANNEL_SEARCH  3

// CAN1 Control Register (CTLR) b9, b8 CANM[1:0] CAN Operating Mode Select. 
#define CAN_OPERATION       0    //CAN operation mode 
#define CAN_RESET           1    //CAN reset mode 
#define CAN_HALT            2    //CAN halt mode 
#define CAN_RESET_FORCE     3    //CAN reset mode (forcible transition) 

// Frame types 
#define DATA_FRAME          0
#define REMOTE_FRAME        1

// Bit set defines 
#define        MBX_0        0x00000001
#define        MBX_1        0x00000002
#define        MBX_2        0x00000004
#define        MBX_3        0x00000008
#define        MBX_4        0x00000010
#define        MBX_5        0x00000020
#define        MBX_6        0x00000040
#define        MBX_7        0x00000080
#define        MBX_8        0x00000100
#define        MBX_9        0x00000200
#define        MBX_10       0x00000400
#define        MBX_11       0x00000800
#define        MBX_12       0x00001000
#define        MBX_13       0x00002000
#define        MBX_14       0x00004000
#define        MBX_15       0x00008000
#define        MBX_16       0x00010000
#define        MBX_17       0x00020000
#define        MBX_18       0x00040000
#define        MBX_19       0x00080000
#define        MBX_20       0x00100000
#define        MBX_21       0x00200000
#define        MBX_22       0x00400000
#define        MBX_23       0x00800000
#define        MBX_24       0x01000000
#define        MBX_25       0x02000000
#define        MBX_26       0x04000000
#define        MBX_27       0x08000000
#define        MBX_28       0x10000000
#define        MBX_29       0x20000000
#define        MBX_30       0x40000000
#define        MBX_31       0x80000000

/*****************************************************************
 *              R X   C A N   A P I
 *****************************************************************/
// INITIALIZATION 
uint32_t    R_CAN_Create(const uint32_t ch_nr);
uint32_t    R_CAN_PortSet(const uint32_t ch_nr, const uint32_t action_type);
uint32_t    R_CAN_Control(const uint32_t ch_nr, const uint32_t action_type);
void        R_CAN_SetBitrate(const uint32_t ch_nr);
void        R_CAN_ModuleStopStateCancel(const uint32_t ch_nr);

// TRANSMIT 
uint32_t    R_CAN_TxSet(const uint32_t ch_nr, const uint32_t mbox_nr, const can_frame_t* frame_p, const uint32_t frame_type);
uint32_t    R_CAN_TxSetXid(const uint32_t ch_nr, const uint32_t mbox_nr,       can_frame_t* frame_p, const uint32_t frame_type);
uint32_t    R_CAN_Tx(const uint32_t ch_nr, const uint32_t mbox_nr);
uint32_t    R_CAN_TxCheck(const uint32_t ch_nr, const uint32_t mbox_nr);
uint32_t    R_CAN_TxStopMsg(const uint32_t ch_nr, const uint32_t mbox_nr);

// RECEIVE 
uint32_t    R_CAN_RxSet(const uint32_t ch_nr, const uint32_t mbox_nr, const uint32_t sid, const uint32_t frame_type);
uint32_t    R_CAN_RxSetXid(const uint32_t ch_nr, const uint32_t mbox_nr,       uint32_t xid, const uint32_t frame_type);
uint32_t    R_CAN_RxPoll(const uint32_t ch_nr, const uint32_t mbox_nr);
uint32_t    R_CAN_RxRead(const uint32_t ch_nr, const uint32_t mbox_nr, can_frame_t * const frame_p);
void        R_CAN_RxSetMask(const uint32_t ch_nr, const uint32_t mbox_nr, const uint32_t mask_value);

// ERRORS 
uint32_t R_CAN_CheckErr(const uint32_t ch_nr);

#endif // R_CAN_API.H

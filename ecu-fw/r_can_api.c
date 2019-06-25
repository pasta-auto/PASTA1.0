/*******************************************************************************
Copyright (C) 2011 Renesas Electronics Corporation. All rights reserved.    

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*******************************************************************************/
/***********************************************************************************************************************
* File Name    : r_can_api.c
* Version      : 2.00
* Description  : CAN API function definitions. See application note r01an0339eu_rx for how to use.
*
*    //////////////////////////////////////////////////////////
*    //   Warning! Do not make changes to this code!         //
*    //  User customizations are confined to config_r_can.h  // 
*    //////////////////////////////////////////////////////////
*                    
************************************************************************************************************************
* History : DD.MM.YYYY  Version Description
*         : 22.3.2010   1.00    For RX62N with new CAN API.
*         : 15.4.2010           Remote frame handling added.
*                               Port configuration moved to config.h. User just 
*                                   sets port and pin number for the transceiver control 
*                                   ports. No need to change driver.
*                               R_CAN_Control: Enter Sleep added.
*                               R_CAN_PortSet: Modes automatically now enter and exit
*                               Halt mode, so user need just one call to change
*                                   mode.
*                               R_CAN_TxStopMsg(): TRMREQ to 0, then TRMABT clear.
*                               R_CAN_RxSetMask(): Halt CAN before mask change, resume 
*                                   after.
*                               R_CAN_RxPoll(): Function rewritten to use INVALDATA flag.
*                               R_CAN_NOT_OK added; "No message waiting or currently                                               
*                                   being written".
*                               All RETURN values spelled out in all function headers.
*                               Return values added and changed for some APIs.
*         : 7.6.2010            Changed TRM_ACTIVE -> SENTDATA in can_wait_tx_rx() since 
*                               TRM_ACTIVE is low for a while after TRMREQ is set high.            
*                               Increased MAX_CAN_SW_DELAY from 0x1000 to 0x2000. If the
*                                   TxCheck function is not used, the timer could time out
*                                   and the mailbox will not send properly if the user ignores 
*                                   the api_status warning that can_wait_tx_rx timed out.
*         : 9.6.2011            Ported to RSKRX630.
*                               Replaced hard-coded channel number references in function 
*                                   calls, with dynamic channel number labels. 
*                               Added other support for dynamic channel number handling.
*         : 27.6.2011           Adjusted #ifndef  USE_CAN_POLL sections. 
*         : 10.1.2012  2.00     Added support for CAN Extended IDs (29-bit ID).
*                               In can_wait_tx_rx changed to while (...SENTDATA == 0)...
*         : 22.3.2012           Removed RESET_CAN_SW_TMR. Counter reset at function start.
*         : 12.10.12            Changed R_CAN_Control() sleep mode, to ensure that Halt mode
*                               is entered before exiting sleep.
*                               Got rid of can_std_frame_t. can_frame_t to cover both
*                               Standard and Ext frames.
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include	<string.h>
#include	"altypes.h"
#include	"iodefine.h"
//#include	<machine.h>
#include	"config_r_can_rapi.h"
//#include	"platform.h"
#include	"r_can_api.h"
//#include	"typedefine.h"
#include	"libs.h"
#include	"ecu.h"			/*	ECU 共通定義			*/
#include	"timer.h"
#include	"cantp.h"		/*	CAN-TP 定義				*/

/******************************************************************************
Typedef definitions
******************************************************************************/
void	logging(char *fmt, ...);
/******************************************************************************
Macro definitions
******************************************************************************/
/* These macros are for determining if some can while loop times out. If they do,
the canx_sw_err variable will be non zero. This is to prevent an error in the 
can peripheral and driver from blocking the CPU indefinatly. */
#define DEC_CHK_CAN_SW_TMR      (--can_tmo_cnt != 0)
#define MAX_CAN_SW_DELAY        0x2000

#define CHECK_MBX_NR            {if (mbox_nr > 31) return R_CAN_SW_BAD_MBX;}

/* Define a mask for MSB of a long to serve as an extended ID mode bit flag. */
/* Extended ID occupies lower 29 bits, so use this to mask off upper 3 bits. */ 
#define XID_MASK                0xE0000000

/* Define a mask for the 11 bits that make up a standard ID. */ 
#define SID_MASK                0x000007FF

/******************************************************************************
Global variables and functions imported (externs)
******************************************************************************/
/******************************************************************************
Constant definitions
*****************************************************************************/
/* Mem. area for bit set defines */
static const uint32_t  bit_set[32] = 
{
    MBX_0,  MBX_1,  MBX_2,  MBX_3, 
    MBX_4,  MBX_5,  MBX_6,  MBX_7,
    MBX_8,  MBX_9,  MBX_10, MBX_11, 
    MBX_12, MBX_13, MBX_14, MBX_15,
    MBX_16, MBX_17, MBX_18, MBX_19,
    MBX_20, MBX_21, MBX_22, MBX_23, 
    MBX_24, MBX_25, MBX_26, MBX_27,
    MBX_28, MBX_29, MBX_30, MBX_31, 
};

const can_st_ptr CAN_CHANNELS[] = 
{
    #ifdef CAN0
         &CAN0,
    #endif
    #ifdef CAN1      
         &CAN1,
    #endif
    #ifdef CAN2      
         &CAN2 
    #endif     
};
/* make sure that MAX_CHANNELS = the number of CAN channels */
#define MAX_CHANNELS sizeof(CAN_CHANNELS)/sizeof(can_st_ptr)

/* initialize CAN port map */
const CAN_port_map_t CAN_CHNL_PORTS[] = {
	//#if defined(CAN0) && defined(CAN0_RX_PORT)
#if defined(CAN0)
        {
        p_CAN0_RX_PIN_MPC,
        p_CAN0_RX_PIN_PMR,
        p_CAN0_RX_PIN_PDR,
        p_CAN0_RX_PIN_PIDR,
          CAN0_RX_PIN_MASK,
        p_CAN0_TX_PIN_MPC, 
        p_CAN0_TX_PIN_PMR,
        p_CAN0_TX_PIN_PDR,
        p_CAN0_TX_PIN_PODR,
          CAN0_TX_PIN_MASK,
        },
    #endif
#if defined(CAN1) && defined(CAN1_RX_PORT)
        {
        p_CAN1_RX_PIN_MPC,
        p_CAN1_RX_PIN_PMR,
        p_CAN1_RX_PIN_PDR,
        p_CAN1_RX_PIN_PIDR,
          CAN1_RX_PIN_MASK,
        p_CAN1_TX_PIN_MPC, 
        p_CAN1_TX_PIN_PMR,
        p_CAN1_TX_PIN_PDR,
        p_CAN1_TX_PIN_PODR,
          CAN1_TX_PIN_MASK,
        },
    #endif
#if defined(CAN2) && defined(CAN2_RX_PORT)
        {      
        p_CAN2_RX_PIN_MPC,
        p_CAN2_RX_PIN_PMR,
        p_CAN2_RX_PIN_PDR,
        p_CAN2_RX_PIN_PIDR,
          CAN2_RX_PIN_MASK,
        p_CAN2_TX_PIN_MPC, 
        p_CAN2_TX_PIN_PMR,
        p_CAN2_TX_PIN_PDR,
        p_CAN2_TX_PIN_PODR,
          CAN2_TX_PIN_MASK,
        }
    #endif     
    };


/******************************************************************************
Global variables and functions private to the file
******************************************************************************/
/* Data */
/* Functions */
static void     can_clear_sent_data(const uint32_t ch_nr, const uint32_t mbox_nr);
static uint32_t can_wait_tx_rx(const uint32_t ch_nr, const uint32_t mbox_nr);
static void     config_can_interrupts(const uint32_t ch_nr);

//________________________________________________________________________________________
//
//  monit_timeover
//----------------------------------------------------------------------------------------
//  機能説明
//      CMT1時差モニタータイムアップ
//  引数
//      無し
//  戻り
//      無し
//________________________________________________________________________________________
//
void monit_timeover(void)
{
	int t;
	t = cmt1_stop();
	if(t < led_monit_first) led_monit_first = t;
	if(t > led_monit_slow) led_monit_slow = t;
	led_monit_time += t;
	led_monit_count++;
	if(led_monit_count >= led_monit_sample)
	{
		led_monit_time /= led_monit_count;
		logging("MON %03X %02X F=%dus A=%dus S=%dus\r", led_monit_id, led_monit_ch, led_monit_first, led_monit_time, led_monit_slow);
		led_monit_id = 0;
		led_monit_ch = 0;
	}
}

/******************************************************************************

                        C A N 0   F U N C T I O N S

******************************************************************************/
/*******************************************************************************
Function Name:  R_CAN_Create
Description:    Configure the CAN peripheral.
Arguments:      ch_nr

Return value:   R_CAN_OK                Action completed successfully.
                R_CAN_SW_BAD_MBX        Bad mailbox number.
                R_CAN_BAD_CH_NR         The channel number does not exist.
                R_CAN_SW_RST_ERR        The CAN peripheral did not enter Reset mode.
                R_CAN_MODULE_STOP_ERR   Module in stop state. PRCR register perhaps 
                                        not used to unlock the module stop register.
*******************************************************************************/
uint32_t R_CAN_Create(const uint32_t ch_nr)
{
    uint32_t api_status = R_CAN_OK;
    uint32_t i, j;
    
    /* A faulty CAN peripheral block, due to HW, FW could potentially block (hang) 
    the program at a while-loop. To prevent this, a sw timer in the while-loops
    will time out enabling the CPU to continue. */
    uint32_t can_tmo_cnt = MAX_CAN_SW_DELAY;   

    volatile struct st_can __evenaccess * can_block_p;

    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }
    R_CAN_ModuleStopStateCancel(ch_nr); /* exit module stop state */
    
    if(SYSTEM.MSTPCRB.LONG & (1 << ch_nr)) /* Check bits 0, 2, or 4 for channel 0, 1, or 2 */
    {   /* Module stop state bit did not clear. PRCR is probably locked. */
        return R_CAN_MODULE_STOP_ERR;      
    }
    
    /* Exit Sleep mode. This will also take us from HALT mode to OPERATE_CANMODE. */
    api_status |= R_CAN_Control(ch_nr, EXITSLEEP_CANMODE);
    
    /* Go to RESET mode. **********************************************************/
    api_status |= R_CAN_Control(ch_nr, RESET_CANMODE);
    
    /*** Setting of CAN1 Control register.***/
    /* BOM:    Bus Off recovery mode acc. to IEC11898-1 */
    can_block_p->CTLR.BIT.BOM = 0;
    /* MBM: Select normal mailbox mode. */
    can_block_p->CTLR.BIT.MBM = 0;
    
    /* IDFM: Select Frame ID mode. */
    can_block_p->CTLR.BIT.IDFM = FRAME_ID_MODE;

    /*     :    0 = Overwrite mode: Latest message overwrites old.
            1 = Overrun mode: Latest message discarded. */
    can_block_p->CTLR.BIT.MLM = 0;
    /* TPM: ID priority mode. */
    can_block_p->CTLR.BIT.TPM = 0;
    /* TSRC: Only to be set to 1 in operation mode */
    can_block_p->CTLR.BIT.TSRC = 0;
    /* TSPS: Update every 8 bit times */
    can_block_p->CTLR.BIT.TSPS = 3;

    /* Set BAUDRATE */
    R_CAN_SetBitrate(ch_nr);

    /* Mask invalid for all mailboxes by default. */
    can_block_p->MKIVLR.LONG = 0xFFFFFFFF;

    /* Configure CAN interrupts. */ 
    config_can_interrupts(ch_nr);

    /* Reset -> HALT mode ************************************************************/
    api_status |= R_CAN_Control(ch_nr, HALT_CANMODE);
    
    /* Configure mailboxes in Halt mode. */
    for (i = 0; i < 32; i++)
    {
        can_block_p->MB[i].ID.LONG = 0x00;
        can_block_p->MB[i].DLC = 0x0000;
        for (j = 0; j < 8; j++)
        {
            can_block_p->MB[i].DATA[j] = 0x00;
        }
        for (j = 0; j < 2; j++)
        {
            can_block_p->MB[i].TS = 0x0000;
        }
            
    }    

    /* Halt -> OPERATION mode *********************************************************/
    /* Note: EST and BLIF flag go high here when stepping code in debugger. */
    api_status |= R_CAN_Control(ch_nr, OPERATE_CANMODE);

    /* Time Stamp Counter reset. Set the TSRC bit to 1 in CAN Operation mode. */
    can_block_p->CTLR.BIT.TSRC = 1;
    while ((can_block_p->CTLR.BIT.TSRC) && DEC_CHK_CAN_SW_TMR) {;}
    if (can_tmo_cnt == 0)
    {
        api_status |= R_CAN_SW_TSRC_ERR;
    }
    /* Check for errors so far, report, and clear. */
    if (can_block_p->STR.BIT.EST)
    {
        api_status |= R_CAN_SW_RST_ERR;
    }
    /* Clear Error Interrupt Factor Judge Register. */
    if (can_block_p->EIFR.BYTE)
    {
        api_status |= R_CAN_SW_RST_ERR;
    }
    can_block_p->EIFR.BYTE = 0x00;

    /* Clear Error Code Store Register. */
    if (can_block_p->ECSR.BYTE)
    {
        api_status |= R_CAN_SW_RST_ERR;
    }
    can_block_p->ECSR.BYTE = 0x00;

    return api_status;
}/* end R_CAN_Create() */

/***********************************************************************************
Function Name:  R_CAN_PortSet
Description:    Configures the MCU and transceiver port pins. This function is 
                responsible for configuring the MCU and transceiver port pins. 
                Transceiver port pins such as Enable will vary depending on design, 
                and this fucntion must then be modified. The function is also used 
                to enter the CAN port test modes, such as Listen Only.
            
                Typical TJA1041 transceiver voltages with CAN active for RX/62N 
                RSK (ROK5562N0C):
                    PIN:        Voltage
                    1    TXD    3.25
                    2    GND    0.00
                    3    VCC    5.08
                    4    RXD    3.20
                    5    VIO    3.25
                    6    EN     3.25
                    7    INH    5.08

                    8    ERR    0.10
                    9    WAKE   0.00
                    10   VBAT   5.08
                    11   SPLIT  2.57
                    12   CANL   2.56
                    13   CANH   2.56
                    14   STB    3.25
                    
Arguments:      ch_nr 
                action_types: ENABLE, DISABLE, CANPORT_TEST_LISTEN_ONLY, 
                CANPORT_TEST_0_EXT_LOOPBACK, CANPORT_TEST_1_INT_LOOPBACK, and
                CANPORT_RETURN_TO_NORMAL which is the default; no need to call 
                unless another test mode was invoked previously.
Return value:   R_CAN_OK                Action completed successfully.
                R_CAN_SW_BAD_MBX        Bad mailbox number.
                R_CAN_BAD_CH_NR         The channel number does not exist.
                R_CAN_BAD_ACTION_TYPE   No such action type exists for this function.
                R_CAN_SW_HALT_ERR       The CAN peripheral did not enter Halt mode.
                R_CAN_SW_RST_ERR        The CAN peripheral did not enter Reset mode.
                See also R_CAN_Control return values.
***********************************************************************************/
uint32_t R_CAN_PortSet(const uint32_t ch_nr, const uint32_t action_type)
{  
    uint32_t api_status = R_CAN_OK;
    volatile struct st_can __evenaccess * can_block_p;
    CAN_port_map_t * pst_can_pin;

	//logging(LOG_DEBUG, NULL, "P_CAN_PortSet(ch_nr=%d, action_type=%d)\n", ch_nr, action_type);
    if (ch_nr < MAX_CHANNELS) 
    { 
        can_block_p = CAN_CHANNELS[ch_nr];
        pst_can_pin = (CAN_port_map_t *)&CAN_CHNL_PORTS[ch_nr];
    }    
    else 
    {
        return R_CAN_BAD_CH_NR;
    }
        
    switch (action_type)
    {
        case ENABLE:
    
            /* check for null pointer */
            if (!pst_can_pin->p_CAN_Rx_Pin_MPC) 
            {
                return R_CAN_BAD_CH_NR;    /* channel port not defined */
            }

            /* Initialize the RSK630 CTXn and CRXn pins. */

            /* RX630 Port pin function select register setting */
            MPC.PWPR.BYTE = 0x00;    /* PWPR.PFSWE write protect off */
            MPC.PWPR.BYTE = 0x40;    /* PFS register write protect off */

            /* When setting up pins, follow this sequence to avoid glitches */
            *(pst_can_pin->p_CAN_Rx_Pin_PMR) &= ~pst_can_pin->Rx_Pin_mask; /* Clear RX bit to general I/O */
            *(pst_can_pin->p_CAN_Rx_Pin_PDR) &= ~pst_can_pin->Rx_Pin_mask; /* Set RX pin as input:  0  */
            *(pst_can_pin->p_CAN_Tx_Pin_PMR) &= ~pst_can_pin->Tx_Pin_mask; /* Clear TX bit to general I/O */        
            *(pst_can_pin->p_CAN_Tx_Pin_PDR) &= ~pst_can_pin->Tx_Pin_mask; /* Set TX pin as input (for now):  0  */
                            
            /* Select CAN signal I/O pins in MPC register*/
            *(pst_can_pin->p_CAN_Tx_Pin_MPC) = PINFUNC_CAN;        
            *(pst_can_pin->p_CAN_Rx_Pin_MPC) = PINFUNC_CAN;
        
            /* Set RX and TX port pins as peripheral I/O (set bit to 1) */
            *(pst_can_pin->p_CAN_Rx_Pin_PMR) |= pst_can_pin->Rx_Pin_mask;        
            *(pst_can_pin->p_CAN_Tx_Pin_PMR) |= pst_can_pin->Tx_Pin_mask;
                
            /* set port directions */
            *(pst_can_pin->p_CAN_Rx_Pin_PDR) &= ~pst_can_pin->Rx_Pin_mask; /* CRX0 is input: clear the bit */        
            *(pst_can_pin->p_CAN_Tx_Pin_PDR) |= pst_can_pin->Tx_Pin_mask;  /* CTX0 is output: set the bit */

            #if defined(CAN0) && defined(CAN0_TRX_STB_PORT) && defined(CAN0_TRX_STB_PIN)        
            if (ch_nr == 0)
            {
                /* Configure CAN0 STBn pin. See config.h. */
                CAN_TRX_DDR( CAN0_TRX_STB_PORT, CAN0_TRX_STB_PIN ) = 1;
                CAN_TRX_DR(CAN0_TRX_STB_PORT, CAN0_TRX_STB_PIN) = CAN0_TRX_STB_LVL;
        
                /* Configure CAN0 EN pin. */
                CAN_TRX_DDR( CAN0_TRX_ENABLE_PORT, CAN0_TRX_ENABLE_PIN ) = 1;
                CAN_TRX_DR(CAN0_TRX_ENABLE_PORT, CAN0_TRX_ENABLE_PIN) = CAN0_TRX_ENABLE_LVL;        
            }
            #endif
            #if defined(CAN1) && defined(CAN1_TRX_STB_PORT) && defined(CAN1_TRX_STB_PIN)    
            if (ch_nr == 1)
            {
                /* Configure CAN1 STBn pin. See config.h. */
                CAN_TRX_DDR( CAN1_TRX_STB_PORT, CAN1_TRX_STB_PIN ) = 1;
                CAN_TRX_DR(CAN1_TRX_STB_PORT, CAN1_TRX_STB_PIN) = CAN1_TRX_STB_LVL;
        
                /* Configure CAN1 EN pin. */
                CAN_TRX_DDR( CAN1_TRX_ENABLE_PORT, CAN1_TRX_ENABLE_PIN ) = 1;
                CAN_TRX_DR(CAN1_TRX_ENABLE_PORT, CAN1_TRX_ENABLE_PIN) = CAN1_TRX_ENABLE_LVL;            
            }
            #endif
            #if defined(CAN2) && defined(CAN2_TRX_STB_PORT) && defined(CAN2_TRX_STB_PIN)
            if (ch_nr == 2)
            {
                /* Configure CAN2 STBn pin. See config.h. */
                CAN_TRX_DDR( CAN2_TRX_STB_PORT, CAN2_TRX_STB_PIN ) = 1;
                CAN_TRX_DR(CAN2_TRX_STB_PORT, CAN2_TRX_STB_PIN) = CAN2_TRX_STB_LVL;
        
                /* Configure CAN2 EN pin. */
                CAN_TRX_DDR( CAN2_TRX_ENABLE_PORT, CAN2_TRX_ENABLE_PIN ) = 1;
                CAN_TRX_DR(CAN2_TRX_ENABLE_PORT, CAN2_TRX_ENABLE_PIN) = CAN2_TRX_ENABLE_LVL;            
            }
            #endif
                
            MPC.PWPR.BYTE = 0x80;    /* PFS register write protect on */
        break;
    
        case DISABLE:
            /* Configure CAN1 TX and RX pins. */
        
            /* RX630 Port pin function select register setting */
            MPC.PWPR.BYTE = 0x00;    /* PWPR.PFSWE write protect off */
            MPC.PWPR.BYTE = 0x40;    /* PFS register write protect off */    

            *(pst_can_pin->p_CAN_Rx_Pin_PMR) &= ~pst_can_pin->Rx_Pin_mask; /* Clear RX bit to general I/O */
            *(pst_can_pin->p_CAN_Rx_Pin_PDR) &= ~pst_can_pin->Rx_Pin_mask; /* Set RX pin as input:  0  */
            *(pst_can_pin->p_CAN_Tx_Pin_PMR) &= ~pst_can_pin->Tx_Pin_mask; /* Clear TX bit to general I/O */        
            *(pst_can_pin->p_CAN_Tx_Pin_PDR) &= ~pst_can_pin->Tx_Pin_mask; /* Set TX pin as input: 0  */        
                    
            #if defined(CAN0) && defined(CAN0_TRX_STB_PORT) && defined(CAN0_TRX_STB_PIN)        
            if (ch_nr == 0)
            {
                /* Configure CAN0 STBn pin. See config.h. */
                CAN_TRX_DDR( CAN0_TRX_STB_PORT, CAN0_TRX_STB_PIN ) = 1;
                CAN_TRX_DR(CAN0_TRX_STB_PORT, CAN0_TRX_STB_PIN) = !CAN0_TRX_STB_LVL; /* Negated level. */
        
                /* Configure CAN0 EN pin. */
                CAN_TRX_DDR( CAN0_TRX_ENABLE_PORT, CAN0_TRX_ENABLE_PIN ) = 1;
                CAN_TRX_DR(CAN0_TRX_ENABLE_PORT, CAN0_TRX_ENABLE_PIN) = !CAN0_TRX_ENABLE_LVL; /* Negated level. */
            }
            #endif
            #if defined(CAN1) && defined(CAN1_TRX_STB_PORT) && defined(CAN1_TRX_STB_PIN)    
            if (ch_nr == 1)
            {
                /* Configure CAN1 STBn pin. See config.h. */
                CAN_TRX_DDR( CAN1_TRX_STB_PORT, CAN1_TRX_STB_PIN ) = 1;
                CAN_TRX_DR(CAN1_TRX_STB_PORT, CAN1_TRX_STB_PIN) = !CAN1_TRX_STB_LVL;
        
                /* Configure CAN1 EN pin. */
                CAN_TRX_DDR( CAN1_TRX_ENABLE_PORT, CAN1_TRX_ENABLE_PIN ) = 1;
                CAN_TRX_DR(CAN1_TRX_ENABLE_PORT, CAN1_TRX_ENABLE_PIN) = !CAN1_TRX_ENABLE_LVL;            
            }
            #endif
            #if defined(CAN2) && defined(CAN2_TRX_STB_PORT) && defined(CAN2_TRX_STB_PIN)
            if (ch_nr == 2)
            {
                /* Configure CAN2 STBn pin. See config.h. */
                CAN_TRX_DDR( CAN2_TRX_STB_PORT, CAN2_TRX_STB_PIN ) = 1;
                CAN_TRX_DR(CAN2_TRX_STB_PORT, CAN2_TRX_STB_PIN) = !CAN2_TRX_STB_LVL;
        
                /* Configure CAN2 EN pin. */
                CAN_TRX_DDR( CAN2_TRX_ENABLE_PORT, CAN2_TRX_ENABLE_PIN ) = 1;
                CAN_TRX_DR(CAN2_TRX_ENABLE_PORT, CAN2_TRX_ENABLE_PIN) = !CAN2_TRX_ENABLE_LVL;            
            }
            #endif
        
            MPC.PWPR.BYTE = 0x80;    /* PFS register write protect on */        
        break;
    
        /* Run in Listen Only test mode. */
        case CANPORT_TEST_LISTEN_ONLY:
            api_status = R_CAN_Control(ch_nr, HALT_CANMODE);
            can_block_p->TCR.BYTE = 0x03;
            api_status |= R_CAN_Control(ch_nr, OPERATE_CANMODE);
            api_status |= R_CAN_PortSet(ch_nr, ENABLE);
        break;
    
        /* Run in External Loopback test mode. */
        case CANPORT_TEST_0_EXT_LOOPBACK:
            api_status = R_CAN_Control(ch_nr, HALT_CANMODE);
            can_block_p->TCR.BYTE = 0x05;
            api_status |= R_CAN_Control(ch_nr, OPERATE_CANMODE);
            api_status |= R_CAN_PortSet(ch_nr, ENABLE);
        break;
    
        /* Run in Internal Loopback test mode. */
        case CANPORT_TEST_1_INT_LOOPBACK:
            api_status = R_CAN_Control(ch_nr, HALT_CANMODE);
            can_block_p->TCR.BYTE = 0x07;
            api_status |= R_CAN_Control(ch_nr, OPERATE_CANMODE);
        break;
    
        /* Return to default CAN bus mode. 
        This is the default setting at CAN reset. */
        case CANPORT_RETURN_TO_NORMAL:
            api_status = R_CAN_Control(ch_nr, HALT_CANMODE);
            can_block_p->TCR.BYTE = 0x00;
            api_status |= R_CAN_Control(ch_nr, OPERATE_CANMODE);
            api_status |= R_CAN_PortSet(ch_nr, ENABLE);
        break;
    
        default:
            /* Bad action type. */
            api_status = R_CAN_BAD_ACTION_TYPE;
        break;
    }
    return api_status;
}/* end R_CAN_PortSet() */

/*******************************************************************************
Function Name:  R_CAN_Control
Description:    Controls transition to CAN operating modes determined by the CAN 
                Control register. For example, the Halt mode should be used to 
                later configure a recieve mailbox. 
Arguments:      ch_nr
                action_type: EXITSLEEP_CANMODE, ENTERSLEEP_CANMODE,
                RESET_CANMODE, HALT_CANMODE, OPERATE_CANMODE.
Return value:   R_CAN_OK                Action completed successfully.
                R_CAN_SW_BAD_MBX        Bad mailbox number.
                R_CAN_BAD_CH_NR         The channel number does not exist.
                R_CAN_BAD_ACTION_TYPE   No such action type exists for this function.
                R_CAN_SW_WAKEUP_ERR     The CAN peripheral did not wake up from Sleep mode.
                R_CAN_SW_SLEEP_ERR      The CAN peripheral did enter Sleep mode.
                R_CAN_SW_RST_ERR        The CAN peripheral did not enter Halt mode.
                R_CAN_SW_HALT_ERR       The CAN peripheral did not enter Halt mode.
                R_CAN_SW_RST_ERR        The CAN peripheral did not enter Reset mode.
                See also R_CAN_PortSet return values.
*******************************************************************************/
uint32_t R_CAN_Control(const uint32_t ch_nr, const uint32_t action_type)
{
    uint32_t api_status = R_CAN_OK;
    uint32_t can_tmo_cnt = MAX_CAN_SW_DELAY;
    volatile struct st_can __evenaccess * can_block_p;
    
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }

    switch (action_type)
    {
        case EXITSLEEP_CANMODE:
            /* Set to Not Sleep, and ensure that RCAN exits in Operate mode.
            HW manual note says to write to the SLPM bit in CAN Halt mode,
            but if we currently are in Sleep mode, we should already also be
            in Halt mode. (See ENTERSLEEP_CANMODE below). */
            can_block_p->CTLR.BIT.SLPM = CAN_NOT_SLEEP;
            while ((can_block_p->STR.BIT.SLPST) && DEC_CHK_CAN_SW_TMR)
            {
				//                nop();
            }
            if (can_tmo_cnt == 0)
            { 
                api_status = R_CAN_SW_WAKEUP_ERR;
            }
            R_CAN_Control(0, OPERATE_CANMODE);
            break;

        case ENTERSLEEP_CANMODE:
            /* Set to, and ensure that RCAN returns in, the Sleep state.
            Write to the SLPM bit in CAN Reset or CAN Halt modes. */
            api_status = R_CAN_Control(0, HALT_CANMODE);

            can_block_p->CTLR.BIT.SLPM = CAN_SLEEP;
            while ((!can_block_p->STR.BIT.SLPST) && DEC_CHK_CAN_SW_TMR)
            {
				//                nop();
            }
            if (can_tmo_cnt == 0)
            { 
                api_status = R_CAN_SW_SLEEP_ERR;
            }
        break;

        case RESET_CANMODE:
            /* Set to, and ensure that RCAN returns in, the Reset state. */
            can_block_p->CTLR.BIT.CANM = CAN_RESET;
            while ((!can_block_p->STR.BIT.RSTST) && DEC_CHK_CAN_SW_TMR)
            {
                // Wait loop.
            }
            if (can_tmo_cnt == 0)
            {
                api_status = R_CAN_SW_RST_ERR;
            }
        break;

        case HALT_CANMODE:
            /* Set to, and ensure that RCAN returns in, the Halt state. */
            /* The CAN module enters CAN Halt mode after waiting for the end of 
            message reception or transmission. */
            can_block_p->CTLR.BIT.CANM = CAN_HALT;
            while ((!can_block_p->STR.BIT.HLTST) && DEC_CHK_CAN_SW_TMR)
            {
                // Wait loop.           
            }
            if (can_tmo_cnt == 0)
            {
                api_status = R_CAN_SW_HALT_ERR;
            }
        break;

        case OPERATE_CANMODE:  
     
            /* Take CAN out of Stop mode. */    
            R_CAN_ModuleStopStateCancel(ch_nr); /* exit module stop state */    

            /* Set to Operate mode. */
            can_block_p->CTLR.BIT.CANM = CAN_OPERATION;

            /* Ensure that RCAN is in Operation    mode. */
            while ((can_block_p->STR.BIT.RSTST) && DEC_CHK_CAN_SW_TMR)
            {
                // Wait loop.           
            }
            if (can_tmo_cnt == 0)
            {
                api_status = R_CAN_SW_RST_ERR;
            }
        break;
        default:
            api_status = R_CAN_BAD_ACTION_TYPE;
        break;
    }

    return api_status;
}/* end R_CAN_Control() */

/*******************************************************************************
Function Name:  R_CAN_TxSet
Description:    Set up a CAN mailbox to transmit.
                2014/02/26 設定のみで送信はしないようにした
Arguments:      Channel nr.
                Mailbox nr.
                frame_p - pointer to a data frame structure.
                remote - REMOTE_FRAME to send remote request, DATA_FRAME for 
                sending normal dataframe.
Return value:   R_CAN_OK                The mailbox was set up for transmission.
                R_CAN_SW_BAD_MBX        Bad mailbox number.
                R_CAN_BAD_CH_NR         The channel number does not exist.
                R_CAN_BAD_ACTION_TYPE   No such action type exists for this 
                                        function.
*******************************************************************************/
uint32_t R_CAN_TxSet(    const uint32_t            ch_nr,
                        const uint32_t             mbox_nr,
                        const can_frame_t*         frame_p,
                        const uint32_t             frame_type)
{
    uint32_t api_status = R_CAN_OK;
    uint32_t i;
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }

    /* Wait for any previous transmission to complete. */
	api_status = can_wait_tx_rx(ch_nr, mbox_nr);

    /* Interrupt disable the mailbox.in case it was a receive mailbox */
    can_block_p->MIER.LONG &= ~(bit_set[mbox_nr]);

    /* Clear message mailbox control register (trmreq to 0). */
    can_block_p->MCTL[mbox_nr].BYTE = 0;
        
    /*** Set Mailbox ID based on ID mode ***/
    if (frame_p->id & XID_MASK)    /* Check for XID flag bit set in ID field */
    {
        /* Set message mailbox buffer Extended ID, masking off temporary XID flag bit. */      
        can_block_p->MB[mbox_nr].ID.LONG = (frame_p->id & (~XID_MASK));
        can_block_p->MB[mbox_nr].ID.BIT.IDE = 1;            /* Frame select: Extended = 1 */
    }     
    else
    {   
        /* Set message mailbox buffer Standard ID. Put only the lower 11 bit in the SID. */ 
        can_block_p->MB[mbox_nr].ID.BIT.SID = (frame_p->id & SID_MASK);    
        /* Frame select: Standard = 0 */
        can_block_p->MB[mbox_nr].ID.BIT.IDE = 0;
    }

    /* Set the Data Length Code */
    can_block_p->MB[mbox_nr].DLC = (unsigned short)frame_p->dlc;
        
    /* Frame select: Data frame = 0, Remote = 1 */
    if (frame_type == REMOTE_FRAME)
    {
        can_block_p->MB[mbox_nr].ID.BIT.RTR = 1;
    }
    else 
    { 
        can_block_p->MB[mbox_nr].ID.BIT.RTR = 0;
    }
      
    /* Copy frame data into mailbox */
    for (i = 0; ((i < frame_p->dlc) && (i<8)); i++)
    {
        can_block_p->MB[mbox_nr].DATA[i] = frame_p->data[i];
    }
    /**********************/

    #if (USE_CAN_POLL == 0)
        /* Interrupt enable the mailbox */
        can_block_p->MIER.LONG |= (bit_set[mbox_nr]);        
    #endif

//    R_CAN_Tx(ch_nr, mbox_nr);			/* CANデータ送信 */
    
    return api_status;
} /* end R_CAN_TxSet() */


/*******************************************************************************
Function Name:  R_CAN_TxSetXid
Description:    Set up a CAN mailbox to transmit in extended ID mode.
                Uses temporary copy of the can_frame data structure to set the 
                MSB of the frame ID field to serve as a flag to indicate extended 
                ID mode, then calls the regular R_CAN_TxSet() function passing 
                along all the parameters.
Arguments:      Channel nr.
                Mailbox nr.
                frame_p - pointer to a data frame structure.
                remote - REMOTE_FRAME to send remote request, DATA_FRAME for 
                sending normal dataframe.
Return value:   R_CAN_OK                The mailbox was set up for transmission.
                R_CAN_SW_BAD_MBX        Bad mailbox number.
                R_CAN_BAD_CH_NR         The channel number does not exist.
                R_CAN_BAD_ACTION_TYPE   No such action type exists for this 
                                        function.
*******************************************************************************/
uint32_t R_CAN_TxSetXid(const uint32_t     ch_nr, 
                        const uint32_t     mbox_nr,
                        can_frame_t*       frame_p,
                        const uint32_t     frame_type)
{
    can_frame_t temp_frame;
    uint32_t    api_status;
    
    /* Copy the user frame to a temporary frame to which we add the Xid bit, 
    so that 29-bit ID will be used by R_CAN_TxSet(). The original frame is left 
    untouched (in case user later wants to send the same frame with standard ID). */
    memcpy(&temp_frame, frame_p, sizeof(can_frame_t));

    temp_frame.id |= XID_MASK;    /* Set XID flag bit set in ID field */
    api_status = R_CAN_TxSet(ch_nr, mbox_nr, (can_frame_t*)&temp_frame, frame_type);
           
    return api_status; 
}/* end R_CAN_TxSetXid() */

//  
//  機能   : R_CAN_Tx
//  説明   : Starts actual message transmission onto the CAN bus.
//  戻り値 : R_CAN_OK            The mailbox was set to transmit a previously 
//                                    configured mailbox.
//           R_CAN_SW_BAD_MBX    Bad mailbox number.
//           R_CAN_BAD_CH_NR     The channel number does not exist.
//           R_CAN_SW_SET_TX_TMO Waiting for previous transmission to finish 
//                               timed out.
//           R_CAN_SW_SET_RX_TMO Waiting for previous reception to complete 
//                               timed out.
//  備考   : 
//  
uint32_t R_CAN_Tx
(
	const uint32_t ch_nr, 					/// Channel nr.
	const uint32_t mbox_nr					/// Mailbox nr.
)
{
    uint32_t api_status = R_CAN_OK;
	volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR;
	if (ch_nr < MAX_CHANNELS)
	{
		can_block_p = CAN_CHANNELS[ch_nr];
	}
	else
	{
		return R_CAN_BAD_CH_NR;
	}

    /* Wait for any previous transmission to complete. */
    api_status = can_wait_tx_rx(ch_nr, mbox_nr);

    /* Clear SentData flag since we are about to send anew. */
    can_clear_sent_data(ch_nr, mbox_nr);
    
    /* Set TrmReq bit to "1" */
    can_block_p->MCTL[mbox_nr].BIT.TX.TRMREQ = 1;
    
    return api_status;
} /* end R_CAN_Tx() */

/*****************************************************************************
Name:           R_CAN_TxCheck
Arguments:      Channel nr.
                Mailbox nr.
Description:    Use to check a mailbox for a successful data frame transmission.
                Primarily used when polling to check that message was sent, so 
                that the next in series of messages can be sent. To do this when 
                using CAN interrupts, this function can be called to check which 
                mailbox caused the interrupt.
Return value:   R_CAN_OK            Transmission was completed successfully.
                R_CAN_SW_BAD_MBX    Bad mailbox number.
                R_CAN_BAD_CH_NR     The channel number does not exist.
                R_CAN_MSGLOST       Message was overwritten or lost.
                R_CAN_NO_SENTDATA   No message was sent.
*****************************************************************************/
uint32_t R_CAN_TxCheck(const uint32_t ch_nr, const uint32_t mbox_nr)
{
    uint32_t api_status = R_CAN_OK;
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }
    
    if (can_block_p->MCTL[mbox_nr].BIT.TX.SENTDATA == 0)
        api_status = R_CAN_NO_SENTDATA;
    else
    {
        /* Clear SentData flag. */
        can_clear_sent_data(ch_nr, mbox_nr);
    }
    return api_status;
}/* end R_CAN_TxCheck() */

/*****************************************************************************
Name:           R_CAN_TxStopMsg
Arguments:      Channel nr.
                Mailbox nr.
Description:    Stop a mailbox that has been asked to transmit a frame. If the 
                message was not stopped, R_CAN_SW_ABORT_ERR is returned. Note 
                that the cause of this could be that the message was already sent. 
Return value:   R_CAN_OK            Action completed successfully.
                R_CAN_SW_BAD_MBX    Bad mailbox number.
                R_CAN_BAD_CH_NR     The channel number does not exist.
                R_CAN_SW_ABORT_ERR  Waiting for an abort timed out.
*****************************************************************************/
uint32_t R_CAN_TxStopMsg(const uint32_t ch_nr, const uint32_t mbox_nr)
{
    uint32_t api_status = R_CAN_OK;
    uint32_t can_tmo_cnt = MAX_CAN_SW_DELAY;   
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }
    
    /* Clear message mailbox control register. Setting TRMREQ to 0 should abort. */
    can_block_p->MCTL[mbox_nr].BYTE = 0;

    /* Wait for abort. */
    while ((can_block_p->MCTL[mbox_nr].BIT.TX.TRMABT) && DEC_CHK_CAN_SW_TMR)
    {;}
    if (can_tmo_cnt == 0)
    {
        api_status = R_CAN_SW_ABORT_ERR;
    }
    /* Clear abort flag. */
    can_block_p->MCTL[mbox_nr].BIT.TX.TRMABT = 0;
    
    return api_status;
}/* end R_CAN_TxStopMsg() */

/*****************************************************************************
Name:           can_clear_sent_data
Arguments:      Channel nr.
                Mailbox nr.
Description:    Use in poll mode for checking successful data frame transmission.
Return value:   CAN API code (CAN_R_CAN_OK if mailbox has sent.)
*****************************************************************************/
//#pragma inline(can_clear_sent_data)
static void can_clear_sent_data(const uint32_t ch_nr, const uint32_t mbox_nr)
{
    volatile struct st_can __evenaccess * can_block_p;
    
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return;
    }

    /* Clear SentData to 0 *after* setting TrmReq to 0. */
    can_block_p->MCTL[mbox_nr].BIT.TX.TRMREQ = 0;
	//    nop();
    can_block_p->MCTL[mbox_nr].BIT.TX.SENTDATA = 0;
}/* end can_clear_sent_data() */   

/*******************************************************************************
Function Name:  R_CAN_RxSet
Description:    Set up a mailbox to receive. The API sets up a given mailbox to 
                receive dataframes with the given CAN ID. Incoming data frames 
                with the same ID will be stored in the mailbox. 
Arguments:      ch_nr
                Mailbox nr.
                Frame ID value
                remote - REMOTE_FRAME to listen for remote requests, DATA_FRAME
                for receiving normal dataframes.
Return value:   R_CAN_OK            Action completed successfully.
                R_CAN_SW_BAD_MBX    Bad mailbox number.
                R_CAN_BAD_CH_NR     The channel number does not exist.
                R_CAN_SW_SET_TX_TMO Waiting for previous transmission to finish 
                                    timed out.
                R_CAN_SW_SET_RX_TMO Waiting for previous reception to complete 
                                    timed out.
*******************************************************************************/
uint32_t R_CAN_RxSet(     const uint32_t     ch_nr, 
                        const uint32_t     mbox_nr, 
                        const uint32_t     sid,
                        const uint32_t     frame_type)
{
    uint32_t api_status = R_CAN_OK;
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }

    /* Wait for any previous transmission/reception to complete. */
    api_status = can_wait_tx_rx(ch_nr, mbox_nr);

    /* Interrupt disable the mailbox. */
    can_block_p->MIER.LONG &= ~(bit_set[mbox_nr]);
    
    /* Clear message mailbox control register. */
    can_block_p->MCTL[mbox_nr].BYTE = 0;

    /*** Set Mailbox ID based on ID mode ***/
    /* Check for XID flag bit set in ID argument */
    if (sid & XID_MASK)
    {
        /* Set message mailbox buffer Extended ID, masking off temporary XID flag bit. */
        can_block_p->MB[mbox_nr].ID.LONG = (sid & (~XID_MASK));                   
        can_block_p->MB[mbox_nr].ID.BIT.IDE = 1; /* Frame select: Extended = 1 */
    }     
    else    
    {
        /* Set message mailbox buffer Standard ID */ 
        can_block_p->MB[mbox_nr].ID.BIT.SID = (sid & SID_MASK); /* Now put the lower 11 bit in the SID. */  
        can_block_p->MB[mbox_nr].ID.BIT.IDE = 0;    /* Frame select: Standard = 0. */
    }

    /* Dataframe = 0, Remote frame = 1    */
    if (frame_type == REMOTE_FRAME)
    {
        can_block_p->MB[mbox_nr].ID.BIT.RTR = 1;
    }
    else 
    {
        can_block_p->MB[mbox_nr].ID.BIT.RTR = 0;
    }

    #if (USE_CAN_POLL == 0)
        /* Interrupt enable the mailbox */
        can_block_p->MIER.LONG |= (bit_set[mbox_nr]);
    #endif

    /* Request to receive the frame with RecReq bit. */
    can_block_p->MCTL[mbox_nr].BYTE = 0x40;

    return api_status;
} /* end R_CAN_RxSet() */

/*******************************************************************************
Function Name:  R_CAN_RxSetXid
Description:    Calls R_CAN_RxSet() after setting a bit in the sid parameter to
                serve as an extended ID mode flag.          
Arguments:      ch_nr
                Mailbox nr.
                Frame ID value
                remote - REMOTE_FRAME to listen for remote requests, DATA_FRAME
                for receiving normal dataframes.
Return value:   value returned by R_CAN_RxSet is passed on.
*******************************************************************************/
uint32_t R_CAN_RxSetXid(const uint32_t     ch_nr, 
                        const uint32_t     mbox_nr, 
                        uint32_t           xid,
                        const uint32_t     frame_type)
{   
    /* Add the Xid bit so that 29-bit ID will be used by R_CAN_RxSet(). */
    return R_CAN_RxSet(ch_nr, mbox_nr, (xid | XID_MASK) , frame_type);
} /* end R_CAN_RxSetXid() */

/*******************************************************************************
Function Name:  R_CAN_RxPoll
Description:    Checks for received message in mailbox.
Arguments:      Channel nr.
                Mailbox nr.
Return value:   R_CAN_OK            There is a message waiting.
                R_CAN_NOT_OK        No message waiting.
                R_CAN_RXPOLL_TMO    Message pending but timed out.
                R_CAN_SW_BAD_MBX    Bad mailbox number.
                R_CAN_BAD_CH_NR     The channel number does not exist.
*******************************************************************************/
uint32_t R_CAN_RxPoll(const uint32_t ch_nr, const uint32_t mbox_nr)
{
    uint32_t api_status = R_CAN_NOT_OK;
    uint32_t poll_delay = MAX_CANREG_POLLCYCLES;
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }
	
    /* Wait if new data is currently being received. */
    while ((can_block_p->MCTL[mbox_nr].BIT.RX.INVALDATA) && poll_delay)
    {
        poll_delay--;
    }
    if (poll_delay == 0)
    /* Still updating mailbox. Come back later. */
    {
        api_status = R_CAN_RXPOLL_TMO;
    }
    else /* Message received? */
    {
        /* If message received, tell user. */
        if (can_block_p->MCTL[mbox_nr].BIT.RX.NEWDATA == 1)
            api_status = R_CAN_OK;
    }
    return api_status;
}/* end R_CAN_RxPoll() */

/*******************************************************************************
Function Name:  R_CAN_RxRead
Arguments:      Mailbox nr.
                frame_p: Data frame structure
Description:    Call from CAN receive interrupt. Copies received data from 
                message mailbox to memory.
Return value:   R_CAN_OK            There is a message waiting.
                R_CAN_SW_BAD_MBX    Bad mailbox number.
                R_CAN_BAD_CH_NR     The channel number does not exist.
                R_CAN_MSGLOST       Message was overwritten or lost.
*******************************************************************************/
uint32_t R_CAN_RxRead( const uint32_t           ch_nr, 
                       const uint32_t           mbox_nr, 
                       can_frame_t* const       frame_p    )
{
    uint32_t i;
    uint32_t api_status = R_CAN_OK;
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }
    
    /* Copy received data from message mailbox to memory */
    if (can_block_p->MB[mbox_nr].ID.BIT.IDE == 1) /* Check for XID control bit set. */
    {
        /* Get mailbox Extended ID, keeping only lower 29 bits. */
        frame_p->id = (can_block_p->MB[mbox_nr].ID.LONG & (~XID_MASK)); 
    }     
    else
    {   
        /* Set message mailbox buffer Standard ID */ 
        frame_p->id = can_block_p->MB[mbox_nr].ID.BIT.SID; /* Get only the lower 11 bits from the SID. */
    }    
    
    frame_p->dlc = (uint8_t)can_block_p->MB[mbox_nr].DLC;
    frame_p->time_st = can_block_p->MB[mbox_nr].TS;			// タイムスタンプ格納　14/01/27
    
    for (i = 0; i < can_block_p->MB[mbox_nr].DLC; i++)
    {
        frame_p->data[i] = can_block_p->MB[mbox_nr].DATA[i];
    }

    /* Check if message was lost/overwritten. */
    if (can_block_p->MCTL[mbox_nr].BIT.RX.MSGLOST)
    {
        can_block_p->MCTL[mbox_nr].BIT.RX.MSGLOST = 0;
        api_status = R_CAN_MSGLOST;
    }

    /* Set NEWDATA bit to 0 since the mailbox was just emptied and start 
    over with new RxPolls. */
    can_block_p->MCTL[mbox_nr].BIT.RX.NEWDATA = 0;
    
    return api_status;
}/* end R_CAN_RxRead() */

/*******************************************************************************
Function Name:  R_CAN_RxSetMask
Description:    Set a CAN bus mask for specified mask register. Note that the 
                MKIVLR register is used to disable the acceptance filtering 
                function individually for each mailbox.
Arguments:      ch_nr
                mask value. For each bit that is 1; corresponding ID bit 
                is compared.
                mbox_nr     0-31. The mailbox nr translates to mask_reg_nr:
                                0 for mailboxes 0-3
                                1 for mailboxes 4-7
                                2 for mailboxes 8-11
                                3 for mailboxes 12-15
                                4 for mailboxes 16-19
                                5 for mailboxes 20-23
                                6 for mailboxes 24-27
                                7 for mailboxes 28-31
Return value:   -
*******************************************************************************/
void R_CAN_RxSetMask( const uint32_t ch_nr, 
                      const uint32_t mbox_nr,
                      const uint32_t mask_value)
{
    volatile struct st_can __evenaccess * can_block_p;
    
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return;
    }    

    /* Write to MKR0 to MKR7 in CAN reset mode or CAN halt mode. */
    R_CAN_Control(ch_nr, HALT_CANMODE);
    
    /* Set mask for the goup of mailboxes. */
    if (can_block_p->MB[mbox_nr].ID.BIT.IDE == 1) /* Check for XID control bit set. */
    {
        /* Set XID 29-bit mask value in mask register. */
        can_block_p->MKR[mbox_nr/4].LONG = (mask_value & (~XID_MASK));
    }     
    else
    {   
        /* Set SID 11-bit mask value in mask register. */
        can_block_p->MKR[mbox_nr/4].BIT.SID = mask_value;
    }      
    
    /* Set mailbox mask to be used. (0 = mask VALID.) */
    can_block_p->MKIVLR.LONG &= ~(bit_set[mbox_nr]);
    
    R_CAN_Control(ch_nr, OPERATE_CANMODE);
            
}/* end R_CAN_RxSetMask() */

/*****************************************************************************
Name:           can_wait_tx_rx
Arguments:      Channel nr.
                Mailbox nr.
Description:    Wait for communicating mailbox to complete action. This would 
                be apporopriate for example if a mailbox all of a sudden needs 
                to be reconfigured but the user wants any pending receive or 
                transmit to finish.
Return value:   R_CAN_OK            There is a message waiting.
                R_CAN_SW_BAD_MBX    Bad mailbox number.
                R_CAN_BAD_CH_NR     The channel number does not exist.
                R_CAN_SW_SET_TX_TMO Waiting for previous transmission to finish 
                                    timed out.
                R_CAN_SW_SET_RX_TMO Waiting for previous reception to complete 
                                    timed out.
*****************************************************************************/
//#pragma inline(can_wait_tx_rx)
static uint32_t can_wait_tx_rx(const uint32_t ch_nr, const uint32_t mbox_nr)
{
    uint32_t api_status = R_CAN_OK;
    uint32_t can_tmo_cnt = MAX_CAN_SW_DELAY;
    volatile struct st_can __evenaccess * can_block_p;
    
    CHECK_MBX_NR
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }

    /* Wait for any previous transmission to complete. */
    if (can_block_p->MCTL[mbox_nr].BIT.TX.TRMREQ)
    {
        while ((can_block_p->MCTL[mbox_nr].BIT.TX.SENTDATA == 0) && DEC_CHK_CAN_SW_TMR)
        {;}
        if (can_tmo_cnt == 0)
        {
            api_status = R_CAN_SW_SET_TX_TMO;
        }
    }
    /* Wait for any previous reception to complete. */
    else if (can_block_p->MCTL[mbox_nr].BIT.TX.RECREQ)    //Strange; but iodefine has TX here..
    {
        while ((can_block_p->MCTL[mbox_nr].BIT.RX.INVALDATA == 1) && DEC_CHK_CAN_SW_TMR) 
        {;}
        if (can_tmo_cnt == 0)
        {
            api_status = R_CAN_SW_SET_RX_TMO;
        }
    }
    return api_status;
}/* end can_wait_tx_rx() */

/*******************************************************************************
Function Name:  R_CAN_CheckErr
Description:    Checks CAN peripheraol error state.
Arguments:      -
Return value:   0 = No error
                1 = CAN is in error active state
                2 = CAN is in error passive state
                4 = CAN is in bus-off state
*******************************************************************************/
uint32_t R_CAN_CheckErr(const uint32_t    ch_nr)
{
    /* Store return value */
    uint32_t api_status = R_CAN_STATUS_ERROR_ACTIVE;
    volatile struct st_can __evenaccess * can_block_p;
    
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return R_CAN_BAD_CH_NR;
    }

    /* Check CAN error state */
    if (can_block_p->STR.BIT.EST)
    {
        /* Check error-passive state */
        if (can_block_p->STR.BIT.EPST)
        {
            api_status = R_CAN_STATUS_ERROR_PASSIVE;
        }

        /* Check bus-off state */
        else if (can_block_p->STR.BIT.BOST)
        {
            api_status = R_CAN_STATUS_BUSOFF;
        }
    }
    
    return api_status;
} /* end R_CAN_CheckErr() */

/*******************************************************************************
Function Name:  R_CAN_SetBitrate
Description:    Sets clock speed and bit rate for CAN as defined in 
                config.h.
Arguments:      -
Return value:   -
*******************************************************************************/
void R_CAN_SetBitrate(const uint32_t ch_nr)
{    
    volatile struct st_can __evenaccess * can_block_p;
    
    if (ch_nr < MAX_CHANNELS)
    {
        can_block_p = CAN_CHANNELS[ch_nr];
    }
    else
    {
        return;
    }

    /* Set TSEG1, TSEG2 and SJW. */
    can_block_p->BCR.BIT.BRP = CAN_BRP - 1;
    can_block_p->BCR.BIT.TSEG1 = CAN_TSEG1 - 1;
    can_block_p->BCR.BIT.TSEG2 = CAN_TSEG2 - 1;
    can_block_p->BCR.BIT.SJW = CAN_SJW - 1;
}/* end R_CAN_SetBitrate() */


/**********************************************************************************
Function Name:  config_can_interrupts
Description:    Configuration of CAN interrupts.    
Arguments:      const uint32_t ch_nr
Return value :  -
***********************************************************************************/
static void config_can_interrupts(const uint32_t ch_nr)
{
#ifdef CAN0
	if (ch_nr == 0)
	{
	#if (USE_CAN_POLL == 0)
		/* Configure CAN Tx interrupt. */
		ICU.IER[IER_CAN0_TXM0].BIT.IEN_CAN0_TXM0 = 1;		//  割り込み許可
		ICU.IPR[IPR_CAN0_TXM0].BIT.IPR = CAN0_INT_LVL;		//	割り込みレベル設定

#ifdef	CAN_RX_INT_ENB
		/* Configure CAN Rx interrupt. */
		ICU.IER[IER_CAN0_RXM0].BIT.IEN_CAN0_RXM0 = 1;		//  割り込み許可
		ICU.IPR[IPR_CAN0_RXM0].BIT.IPR = CAN0_INT_LVL;		//	割り込みレベル設定
#endif
		/* Configure CAN Error interrupt. Must enable group that it belongs to */
		/* in addition to individual source. */
	    ICU.IER[IER_ICU_GROUPE0].BIT.IEN_ICU_GROUPE0 = 1;	//  グループ0 割り込み許可
		ICU.GEN[GEN_CAN0_ERS0].BIT.EN0 = 1;					//  割り込み許可
		ICU.IPR[IPR_ICU_GROUPE0].BIT.IPR = 1;	//CAN0_INT_LVL;	//	グループ0 割り込みレベル設定

		CAN0.EIER.BYTE = 0x01;	//0xFF; /* enable all error interrupts within peripheral */
	#endif // #if (USE_CAN_POLL == 0)

		/* Mailbox interrupt enable registers. Disable interrupts for all slots. 
		They will be enabled individually by the API. */
		CAN0.MIER.LONG = 0x00000000;
	}
#endif
#ifdef CAN1    
	if (ch_nr == 1)
	{
	#if (USE_CAN_POLL == 0)
		ICU.IER[IER_CAN1_TXM1].BIT.IEN_CAN1_TXM1 = 1;		//  割り込み許可
		ICU.IPR[IPR_CAN1_TXM1].BIT.IPR = CAN1_INT_LVL;		//	割り込みレベル設定

#ifdef	CAN_RX_INT_ENB
		/* Configure CAN Rx interrupt. */
		ICU.IER[IER_CAN1_RXM1].BIT.IEN_CAN1_RXM1 = 1;		//  割り込み許可
		ICU.IPR[IPR_CAN1_RXM1].BIT.IPR = CAN1_INT_LVL;		//	割り込みレベル設定
#endif
		/* Configure CAN Error interrupt. Must enable group that it belongs to */
		/* in addition to individual source. */
	    ICU.IER[IER_ICU_GROUPE0].BIT.IEN_ICU_GROUPE0 = 1;	//  グループ0 割り込み許可
		ICU.GEN[GEN_CAN1_ERS1].BIT.EN1 = 1;					//  割り込み許可
		ICU.IPR[IPR_ICU_GROUPE0].BIT.IPR = 1;	//CAN1_INT_LVL;

		 /* Enable all error interrupts within peripheral */
		 CAN1.EIER.BYTE = 0x01;	//0xFF;

		/* Mailbox interrupt enable registers. Disable interrupts for all slots. 
		They will be enabled individually by the API. */
		CAN1.MIER.LONG = 0x00000000;
	#endif // #if (USE_CAN_POLL == 0)
	}
#endif
#ifdef CAN2    
	if (ch_nr == 2)
	{
	#if (USE_CAN_POLL == 0)        
		/* Configure CAN Tx interrupt. */
		ICU.IER[IER_CAN2_TXM2].BIT.IEN_CAN2_TXM2 = 1;		//  割り込み許可
		ICU.IPR[IPR_CAN2_TXM2].BIT.IPR = CAN2_INT_LVL;		//	割り込みレベル設定

#ifdef	CAN_RX_INT_ENB
		/* Configure CAN Rx interrupt. */
		ICU.IER[IER_CAN2_RXM2].BIT.IEN_CAN2_RXM2 = 1;		//  割り込み許可
		ICU.IPR[IPR_CAN2_RXM2].BIT.IPR = CAN2_INT_LVL;		//	割り込みレベル設定
#endif
		/* Configure CAN Error interrupt. Must enable group that it belongs to */
		/* in addition to individual source. */
	    ICU.IER[IER_ICU_GROUPE0].BIT.IEN_ICU_GROUPE0 = 1;	//  グループ0 割り込み許可
		ICU.GEN[GEN_CAN2_ERS2].BIT.EN2 = 1;					//  割り込み許可
		ICU.IPR[IPR_ICU_GROUPE0].BIT.IPR = 1;	//CAN2_INT_LVL;

		CAN2.EIER.BYTE = 0x01;	//0xFF; /* Enable all error interrupts within peripheral */
	#endif // #ifndef USE_CAN_POL

		/* Mailbox interrupt enable registers. Disable interrupts for all slots. 
		They will be enabled individually by the API. */
		CAN2.MIER.LONG = 0x00000000;
	}
#endif    
}/* end config_can_interrupts() */

/******************************************************************************
Function Name:  CAN_ModuleStopState_cancel()
Arguments:      -
Return value:   -
Description:    Release CAN peripherals from standby.
******************************************************************************/
void R_CAN_ModuleStopStateCancel(const uint32_t ch_nr)
{
    /* CAN: Module Stop Control Register B (MSTPCRB) bits 0-2 are for the CAN 
    Peripherals.
    0: The module stop state is canceled.
    1: Transition to the module stop state is made. */

    /* First unlock the protect register. */         
    /* Enable writing to PRCR bits while simultaneously enabling PRC1. */
    SYSTEM.PRCR.WORD = 0xA502;
    
    /* using the macros in iodefines.h*/
    
    if (ch_nr == 0)
    {    
        MSTP_CAN0 = 0;
    }
    else if (ch_nr == 1)
    {    
        MSTP_CAN1 = 0;
    }
    else if (ch_nr == 2)
    {    
        MSTP_CAN2 = 0;
    }
    
    /* Re-lock the protect register. */         
    /* Enable writing to PRCR bits while simultaneously disabling PRC1. */
    SYSTEM.PRCR.WORD = 0xA500;    
}/* end CAN_ModuleStopState_cancel() */


/*********************************************************************************
        Used CAN interrupts are normally in application. Below are templates.
        Duplicate for each CAN channel used except for the Error interrupt which 
        handles all channels in a group. 
        Set up the vector according to the channel.
**********************************************************************************/
#if (USE_CAN_POLL == 0)
void can_tx_mb(int ch, int mb);
int can_recv_frame(int ch, void *mbox);

/*****************************************************************************
Name:            CAN0_TXM0_ISR
Arguments:        -
Return Value:        -
Description:    CAN0 Transmit interrupt.
                Check which mailbox transmitted data and process it.
*****************************************************************************/
//#pragma interrupt CAN0_TXM0_ISR(vect=VECT_CAN1_TXM0, enable) 
void interrupt __vectno__{VECT_CAN0_TXM0} CAN0_TXM0_ISR(void)
{
	int			mb;
	int			id;
//	uint32_t	api_status = R_CAN_OK;

	CAN0.MSMR.BYTE = 1;	//	送信MBのSENTDATA検索
	while(CAN0.MSSR.BIT.SEST == 0)
	{	//	結果有り
		mb = (int)CAN0.MSSR.BIT.MBNST;
		if (!CAN0.MCTL[mb].BIT.TX.TRMACTIVE)
		{
			if (CAN0.MCTL[mb].BIT.TX.SENTDATA)
			{
				id = CAN0.MB[mb].ID.BIT.SID;
			//	if(id == led_monit_id && (led_monit_ch & 1) != 0) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x01) != 0)
				{
					led_monit_ch &= 0xF1;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
				can_tp_txecheck(0, id);	//	TP送信完了確認
				CAN0.MCTL[mb].BIT.TX.TRMREQ = 0;
			//	CAN0.MCTL[mb].BIT.TX.SENTDATA = 0;
				CAN0.MCTL[mb].BYTE = 0;	//	MB停止
			//	can_tx_mb(0, mb);		//	送信待ち検索
			}
		}
	}
	CAN0.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
}

/*****************************************************************************
Name:           CAN0_RXM0_ISR
Arguments:      -
Return Value:   -
Description:    CAN0 Receive interrupt.
                Check which mailbox received data and process it.
*****************************************************************************/
//#pragma interrupt CAN0_RXM0_ISR(vect=VECT_CAN0_RXM0, enable)
void interrupt __vectno__{VECT_CAN0_RXM0} CAN0_RXM0_ISR(void)
{
	/* Use CAN API. */
	int			mb, i;
	CAN_MBOX	*buf;
//	uint32_t api_status = R_CAN_OK;

	CAN0.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
	while(CAN0.MSSR.BIT.SEST == 0)
	{	//	結果有り
		mb = (int)CAN0.MSSR.BIT.MBNST;
	//	if(CAN0.MCTL[mb].BIT.RX.MSGLOST)
	//	{	//	ロスト又はオーバーラン
	//		can_recv_frame(0, (void *)&CAN0.MB[mb]);	//	受信データ取得
	//	}
		if(!CAN0.MCTL[mb].BIT.RX.INVALDATA)
		{
			if(CAN0.MCTL[mb].BIT.RX.NEWDATA)
			{
			//	if(CAN0.MB[mb].ID.BIT.SID == led_monit_id && (led_monit_ch & 1) != 0) PORTE.PODR.BIT.B0 = 0;
				if(CAN0.MB[mb].ID.BIT.SID == led_monit_id && (led_monit_ch & 0x10) != 0)
				{
					led_monit_ch &= 0x1F;
					PORTE.PODR.BIT.B0 = 0;
					cmt1_start(1000000, monit_timeover);
				}
				CAN0.MCTL[mb].BIT.RX.RECREQ = 0;
				CAN0.MCTL[mb].BYTE = 0;
				buf = &rxmb_buf[0].MB[rxmb_buf[0].WP++];
				rxmb_buf[0].WP &= (RX_MB_BUF_MAX-1);
				buf->ID.LONG = CAN0.MB[mb].ID.LONG;
				buf->DLC = CAN0.MB[mb].DLC;
				for(i = 0; i < 8; i ++) buf->DATA[i] = CAN0.MB[mb].DATA[i];
				CAN0.MCTL[mb].BYTE = 0x40;			//	受信再開
			//	can_recv_frame(0, (void *)&buf);	//	受信データ取得
			}
		}
	}
}/* end CAN0_RXM0_ISR() */

/*****************************************************************************
Name:           CAN0_RXF0_ISR
Arguments:      -
Return Value:   -
Description:    CAN0 Rx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt    CAN0_RXF0_ISR(vect=VECT_CAN0_RXF0, enable)
void interrupt __vectno__{VECT_CAN0_RXF0} CAN0_RXF0_ISR(void)
{
	logging("CAN0_RXF0_ISR\r");
//    nop();
}/* end CAN0_RXF0_ISR() */

/*****************************************************************************
Name:            CAN0_TXF0_ISR
Arguments:        -
Return Value:        -
Description:    CAN0 Tx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt    CAN0_TXF0_ISR(vect=VECT_CAN0_TXF0, enable)
void interrupt __vectno__{VECT_CAN0_TXF0} CAN0_TXF0_ISR(void)
{
	logging("CAN0_TXF0_ISR\r");
//    nop();
}/* end CAN0_TXF0_ISR() */

/*****************************************************************************
Name:            CAN1_TXM1_ISR
Arguments:        -
Return Value:        -
Description:    CAN1 Transmit interrupt.
                Check which mailbox transmitted data and process it.
*****************************************************************************/
//#pragma interrupt CAN1_TXM1_ISR(vect=VECT_CAN1_TXM0, enable) 
void interrupt __vectno__{VECT_CAN1_TXM1} CAN1_TXM1_ISR(void)
{
	int			mb;
	int			id;
//	uint32_t	api_status = R_CAN_OK;

	CAN1.MSMR.BYTE = 1;	//	送信MBのSENTDATA検索
	while(CAN1.MSSR.BIT.SEST == 0)
	{	//	結果有り
		mb = (int)CAN1.MSSR.BIT.MBNST;
		if (!CAN1.MCTL[mb].BIT.TX.TRMACTIVE)
		{
			if (CAN1.MCTL[mb].BIT.TX.SENTDATA)
			{
				id = CAN1.MB[mb].ID.BIT.SID;
			//	if(id == led_monit_id && (led_monit_ch & 2) != 0) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x02) != 0)
				{
					led_monit_ch &= 0xF2;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
				can_tp_txecheck(1, id);	//	TP送信完了確認
				CAN1.MCTL[mb].BIT.TX.TRMREQ = 0;
			//	CAN1.MCTL[mb].BIT.TX.SENTDATA = 0;
				CAN1.MCTL[mb].BYTE = 0;	//	MB停止
			//	can_tx_mb(1, mb);		//	送信待ち検索
			}
		}
	}
	CAN1.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
}

/*****************************************************************************
Name:           CAN1_RXM1_ISR
Arguments:      -
Return Value:   -
Description:    CAN1 Receive interrupt.
                Check which mailbox received data and process it.
*****************************************************************************/
//#pragma interrupt CAN1_RXM0_ISR(vect=VECT_CAN1_RXM0, enable)
void interrupt __vectno__{VECT_CAN1_RXM1} CAN1_RXM1_ISR(void)
{
	/* Use CAN API. */
	int	mb, i;
	CAN_MBOX	*buf;
//	uint32_t api_status = R_CAN_OK;

	CAN1.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
	while(CAN1.MSSR.BIT.SEST == 0)
	{	//	結果有り
		mb = (int)CAN1.MSSR.BIT.MBNST;
		if(!CAN1.MCTL[mb].BIT.RX.INVALDATA)
		{
			if(CAN1.MCTL[mb].BIT.RX.NEWDATA)
			{
			//	if(CAN1.MB[mb].ID.BIT.SID == led_monit_id && (led_monit_ch & 2) != 0) PORTE.PODR.BIT.B0 = 0;
				if(CAN1.MB[mb].ID.BIT.SID == led_monit_id && (led_monit_ch & 0x20) != 0)
				{
					led_monit_ch &= 0x2F;
					PORTE.PODR.BIT.B0 = 0;
					cmt1_start(1000000, monit_timeover);
				}
				CAN1.MCTL[mb].BIT.RX.RECREQ = 0;
				CAN1.MCTL[mb].BYTE = 0;
				buf = &rxmb_buf[1].MB[rxmb_buf[1].WP++];
				rxmb_buf[1].WP &= (RX_MB_BUF_MAX-1);
				buf->ID.LONG = CAN1.MB[mb].ID.LONG;
				buf->DLC = CAN1.MB[mb].DLC;
				for(i = 0; i < 8; i ++) buf->DATA[i] = CAN1.MB[mb].DATA[i];
				CAN1.MCTL[mb].BYTE = 0x40;			//	受信再開
			//	can_recv_frame(1, (void *)&buf);	//	受信データ取得
			}
		}
	}
}/* end CAN1_RXM0_ISR() */

/*****************************************************************************
Name:           CAN1_RXF1_ISR
Arguments:      -
Return Value:   -
Description:    CAN1 Rx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt    CAN1_RXF1_ISR(vect=VECT_CAN1_RXF1, enable)
void interrupt __vectno__{VECT_CAN1_RXF1} CAN1_RXF1_ISR(void)
{
	logging("CAN1_RXF1_ISR\r");
//    nop();
}/* end CAN1_RXF1_ISR() */

/*****************************************************************************
Name:            CAN1_TXF1_ISR
Arguments:        -
Return Value:        -
Description:    CAN1 Tx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt    CAN1_TXF1_ISR(vect=VECT_CAN1_TXF1, enable)
void interrupt __vectno__{VECT_CAN1_TXF1} CAN1_TXF1_ISR(void)
{
	logging("CAN1_TXF1_ISR\r");
//    nop();
}/* end CAN1_TXF1_ISR() */

/*****************************************************************************
Name:            CAN2_TXM2_ISR
Arguments:        -
Return Value:        -
Description:    CAN2 Transmit interrupt.
                Check which mailbox transmitted data and process it.
*****************************************************************************/
//#pragma interrupt CAN2_TXM2_ISR(vect=VECT_CAN2_TXM2, enable) 
void interrupt __vectno__{VECT_CAN2_TXM2} CAN2_TXM2_ISR(void)
{
	int			mb;
	int			id;
//	uint32_t	api_status = R_CAN_OK;

	CAN2.MSMR.BYTE = 1;	//	送信MBのSENTDATA検索
	while(CAN2.MSSR.BIT.SEST == 0)
	{	//	結果有り
		mb = (int)CAN2.MSSR.BIT.MBNST;
		if (!CAN2.MCTL[mb].BIT.TX.TRMACTIVE)
		{
			if (CAN2.MCTL[mb].BIT.TX.SENTDATA)
			{
				id = CAN2.MB[mb].ID.BIT.SID;
			//	if(id == led_monit_id && (led_monit_ch & 4) != 0) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x04) != 0)
				{
					led_monit_ch &= 0xF4;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
				can_tp_txecheck(2, id);	//	TP送信完了確認
				CAN2.MCTL[mb].BIT.TX.TRMREQ = 0;
			//	CAN2.MCTL[mb].BIT.TX.SENTDATA = 0;
				CAN2.MCTL[mb].BYTE = 0;	//	MB停止
			//	can_tx_mb(2, mb);		//	送信待ち検索
			}
		}
	}
	CAN2.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
}

/*****************************************************************************
Name:           CAN2_RXM2_ISR
Arguments:      -
Return Value:   -
Description:    CAN2 Receive interrupt.
                Check which mailbox received data and process it.
*****************************************************************************/
//#pragma interrupt CAN2_RXM2_ISR(vect=VECT_CAN2_RXM2, enable)
void interrupt __vectno__{VECT_CAN2_RXM2} CAN2_RXM2_ISR(void)
{
	/* Use CAN API. */
	int	mb, i;
	CAN_MBOX	*buf;
//	uint32_t api_status = R_CAN_OK;

	CAN2.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
	while(CAN2.MSSR.BIT.SEST == 0)
	{	//	結果有り
		mb = (int)CAN2.MSSR.BIT.MBNST;
		if(!CAN2.MCTL[mb].BIT.RX.INVALDATA)
		{
			if(CAN2.MCTL[mb].BIT.RX.NEWDATA)
			{
				if(CAN2.MB[mb].ID.BIT.SID == led_monit_id && (led_monit_ch & 0x40) != 0)
				{
					led_monit_ch &= 0x4F;
					PORTE.PODR.BIT.B0 = 0;
					cmt1_start(1000000, monit_timeover);
				}
				CAN2.MCTL[mb].BIT.RX.RECREQ = 0;
				CAN2.MCTL[mb].BYTE = 0;
				buf = &rxmb_buf[2].MB[rxmb_buf[2].WP++];
				rxmb_buf[2].WP &= (RX_MB_BUF_MAX-1);
				buf->ID.LONG = CAN2.MB[mb].ID.LONG;
				buf->DLC = CAN2.MB[mb].DLC;
				for(i = 0; i < 8; i ++) buf->DATA[i] = CAN2.MB[mb].DATA[i];
				CAN2.MCTL[mb].BYTE = 0x40;			//	受信再開
			//	can_recv_frame(2, (void *)&buf);	//	受信データ取得
			}
		}
	}
}/* end CAN2_RXM2_ISR() */

/*****************************************************************************
Name:           CAN2_RXF2_ISR
Arguments:      -
Return Value:   -
Description:    CAN2 Rx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt    CAN2_RXF1_ISR(vect=VECT_CAN2_RXF2, enable)
void interrupt __vectno__{VECT_CAN2_RXF2} CAN2_RXF2_ISR(void)
{
	logging("CAN2_RXF2_ISR\r");
//    nop();
}/* end CAN2_RXF2_ISR() */

/*****************************************************************************
Name:            CAN2_TXF2_ISR
Arguments:        -
Return Value:        -
Description:    CAN2 Tx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt    CAN2_TXF2_ISR(vect=VECT_CAN2_TXF2, enable)
void interrupt __vectno__{VECT_CAN2_TXF2} CAN2_TXF2_ISR(void)
{
	logging("CAN2_TXF2_ISR\r");
//    nop();
}/* end CAN2_TXF2_ISR() */

/*****************************************************************************
Name:           CAN0_ERS0_ISR
Arguments:      -
Return Value:   -
Description:    CAN0 Error interrupt.
                Check which CAN channel is source of interrupt
*****************************************************************************/
void interrupt __vectno__{VECT_ICU_GROUPE0} CANX_ERS_ISR(void)
{
	int	mb;
	unsigned char	ec;

	while( ICU.IR[ IR_ICU_GROUPE0 ].BIT.IR )
	{
		if(ICU.GRP[ GRP_CAN0_ERS0 ].BIT.IS_CAN0_ERS0)
		{
			ICU.GCR[ GCR_CAN0_ERS0 ].BIT.CLR_CAN0_ERS0 = 1;
			ec = CAN0.EIFR.BYTE;
			CAN0.EIFR.BYTE = 0;
			if(ec != 0)
			{
#if 1
			//	logging("CAN0 Error :");
				if((ec & 0x01) && (CAN0.STR.BIT.BOST))
				{	//	バスエラー
			//		logging(" BEIF");
					//	全送信メール取りやめ
					for(mb = 0; mb < 16; mb++)
					{
						if(CAN0.MCTL[mb].BIT.TX.TRMREQ)
						{
							CAN0.MCTL[mb].BIT.TX.TRMREQ = 0;
			//				logging(" %d", mb);
						}
					}
				}
#endif
#if 0
				if(ec & 0x02) logging(" EWIF");
				if(ec & 0x04) logging(" EPIF");
				if(ec & 0x08) logging(" BOEIF");
				if(ec & 0x10) logging(" BORIF");
				if(ec & 0x20) logging(" ORIF");
				if(ec & 0x40) logging(" OLIF");
				if(ec & 0x80) logging(" BLIF");
				logging("\r");
#endif
			}
			if(CAN0.STR.BIT.BOST)
			{
				logging("CAN0 BOST\r");
				if(CAN0.CTLR.BIT.BOM = 0)
				{	//	ノーマルモード
					CAN0.CTLR.BIT.RBOC = 1;	//	バスOFF強制復帰
				}
			}
		}
		if(ICU.GRP[ GRP_CAN1_ERS1 ].BIT.IS_CAN1_ERS1)
		{
			ICU.GCR[ GCR_CAN1_ERS1 ].BIT.CLR_CAN1_ERS1 = 1;
			ec = CAN1.EIFR.BYTE;
			CAN1.EIFR.BYTE = 0;
			if(ec != 0)
			{
#if 1
			//	logging("CAN1 Error :");
				if((ec & 0x01) && (CAN1.STR.BIT.BOST))
				{	//	バスエラー
			//		logging(" BEIF");
					//	全送信メール取りやめ
					for(mb = 0; mb < 16; mb++)
					{
						if(CAN1.MCTL[mb].BIT.TX.TRMREQ)
						{
							CAN1.MCTL[mb].BIT.TX.TRMREQ = 0;
			//				logging(" %d", mb);
						}
					}
				}
#endif
#if 0
				if(ec & 0x02) logging(" EWIF");
				if(ec & 0x04) logging(" EPIF");
				if(ec & 0x08) logging(" BOEIF");
				if(ec & 0x10) logging(" BORIF");
				if(ec & 0x20) logging(" ORIF");
				if(ec & 0x40) logging(" OLIF");
				if(ec & 0x80) logging(" BLIF");
				logging("\r");
#endif
			}
			if(CAN1.STR.BIT.BOST)
			{
				logging("CAN1 BOST\r");
				if(CAN1.CTLR.BIT.BOM = 0)
				{	//	ノーマルモード
					CAN1.CTLR.BIT.RBOC = 1;	//	バスOFF強制復帰
				}
			}
		}
		if(ICU.GRP[ GRP_CAN2_ERS2 ].BIT.IS_CAN2_ERS2)
		{
			ICU.GCR[ GCR_CAN2_ERS2 ].BIT.CLR_CAN2_ERS2 = 1;
			ec = CAN2.EIFR.BYTE;
			CAN2.EIFR.BYTE = 0;
			if(ec != 0)
			{
#if 1
			//	logging("CAN2 Error :");
				if((ec & 0x01) && (CAN2.STR.BIT.BOST))
				{	//	バスエラー
			//		logging(" BEIF");
					//	全送信メール取りやめ
					for(mb = 0; mb < 16; mb++)
					{
						if(CAN2.MCTL[mb].BIT.TX.TRMREQ)
						{
							CAN2.MCTL[mb].BIT.TX.TRMREQ = 0;
			//				logging(" %d", mb);
						}
					}
				}
#endif
#if 0
				if(ec & 0x02) logging(" EWIF");
				if(ec & 0x04) logging(" EPIF");
				if(ec & 0x08) logging(" BOEIF");
				if(ec & 0x10) logging(" BORIF");
				if(ec & 0x20) logging(" ORIF");
				if(ec & 0x40) logging(" OLIF");
				if(ec & 0x80) logging(" BLIF");
				logging("\r");
#endif
			}
			if(CAN2.STR.BIT.BOST)
			{
				logging("CAN2 BOST\r");
				if(CAN2.CTLR.BIT.BOM = 0)
				{	//	ノーマルモード
					CAN2.CTLR.BIT.RBOC = 1;	//	バスOFF強制復帰
				}
			}
		}
	}
}/* end CAN0_ERS0_ISR() */

#endif //USE_CAN_POLL
/* eof */

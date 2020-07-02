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
 * LFY-RX63N1 SCI-I/F communication
 *
 * ----------------------------------------------------------------------------------------
 * Develpment history
 *
 * 02/10/2016 Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ________________________________________________________________________________________
 */

#ifndef __CAN2ECU_CAN3RSPI2_IF__
#define __CAN2ECU_CAN3RSPI2_IF__

#include "ecu.h" // ECU common description 

/*
 *  Port setting
 *
 *          Port        SCI        I2C        SPI        Application
 *  ----------------------------------------------------------------------------
 *  RSPI2   PD2                               MISOC       <RSPI>   CAN3
 *          PD1                               MOSIC       <RSPI>   CAN3
 *          PD3                               RSPCKC      <RSPI>   CAN3
 *          PD4                               SSLC0       <RSPI>   CAN3
 *          PD0                               IRQ0      <- CINT    CAN3
 *          PD6                                         <- CRX0BF  CAN3
 *          PD7                                         <- CRX1BF  CAN3
 *          P90                                         -> CTX0RTS CAN3
 *          P91                                         -> CTX1RTS CAN3
 *          P92                                         -> CTX2RTS CAN3
 *          P93                                         -> CRSET   CAN3
 *          P07                                         <- CSOF    CAN3
 */

#define     RSPI2_ACTIVATE

//Indirect call prototype (1 argument) 
typedef void (*CAN3_PROC_CALL)(void *);

#ifdef  RSPI2_ACTIVATE

/* ________________________________________________________________________________________
 *
 * DTC description  (DTC is required to operate RSPI at 8MBPS)
 * ________________________________________________________________________________________
 */
#define     DTC_VECT_TOP    0x0003E000
#define     DTC_REQUEST_TOP 0x0003D000

extern unsigned long *dtc_table;

typedef union   __dtc_fulladdressmode__ {
    unsigned long   LONG[4];
    unsigned short  WORD[8];
    unsigned char BYTE[16];
    struct  {
        union   {
            unsigned long LONG;
            struct  {   // Mode register 
                // MRA 
                unsigned long MD :   2; /* DTC transfer mode select bit
                                         * b7 b6
                                         * 0 0:Normal transfer mode
                                         * 0 1:Repeat forward mode
                                         * 1 0:Block forward mode
                                         * 1 1:No not set*/
                unsigned long SZ :   2; /* DTC data transfer size bit
                                         * DTC data transfer size bit
                                         * b5 b4
                                         * 0 0:8 bit (byte) transfer
                                         * 0 1:16 bit (byte) transfer
                                         * 1 0:32 bit (byte) transfer
                                         * 1 1:Do not set*/
                unsigned long SM :   2; /* Source address addressing mode bit
                                         * b3 b2
                                         * 0 0:SAR register address fixed (SAR register write back is skipped)
                                         * 0 1:SAR register address fixed (SAR register write back is skipped)
                                         * 1 0:Increment the SAR register after transfer (+1 when SZ [1:0] bit is “00b”, +2 when “01b”, +4 when “10b”)
                                         * 1 1:Decrement the SAR register after transfer (-1 when SZ [1:0] bit is "00b", -2 when "01b", -4 when "10b")*/
                unsigned long : 2; // [0] 
                // MRB 
                unsigned long CHNE :   1;  /* DTC chain transfer permission bit
                                            * 0:Chain transfer disable
                                            * 1:Chain transfer enable*/
                unsigned long CHNS :   1;  /* DTC chain transfer select bit
                                            * 0:Perform chain transfer continuously
                                            * 1:Chain transfer is performed when the transfer counter changes from 1 to 0 or from 1 to CRAH*/
                unsigned long DISEL :   1; /* DTC interrupt select bit
                                            * 0:When the specified data transfer ends, an interrupt to the CPU occurs
                                            * 1:Interrupt to CPU occurs every time DTC data is transferred*/
                unsigned long DTS :   1;   /* DTC transfer mode select bit
                                            * 0:Transfer destination is repeat area or block area
                                            * 1:Transfer source is repeat area or block area*/
                unsigned long DM :   2;    /* Transfer destination address addressing mode bit
                                            * b3 b2
                                            * 0 0:DAR register address fixed (DAR register write-back is skipped)
                                            * 0 1:DAR register address fixed (DAR register write-back is skipped)
                                            * 1 0:Increment DAR register after transfer (+1 when MAR.SZ [1: 0] bit is “00b”, +2 when “01b”, +4 when “10b”)
                                            * 1 1:Decrement DAR register after transfer (-1 when MAR.SZ [1: 0] bit is "00b", -2 when "01b", -4 when "10b")*/
                unsigned long : 2; // [0] 
                // Reserve (0x0000 light) 
                unsigned long RES :   16;  // [0] 
            }   BIT;
        }   MR;
        unsigned long SAR; // Source address 
        unsigned long DAR; // Forwarding address 
        union   {
            unsigned long LONG;
            struct  {   // Normal trasfer mode 
                unsigned long A : 16; // DTC transfer count register A 
                unsigned long B : 16; // DTC transfer count register B (register that specifies the number of block transfers in block transfer mode) 
            }   NOR;
            struct  {   // Repeat transfer mode 
                unsigned long AH : 8;  // DTC retain transfer count 
                unsigned long AL : 8;  // DTC transfer count counter 
                unsigned long B  : 16; // DTC transfer count register B (register that specifies the number of block transfers in block transfer mode) 
            }   REP;
            struct  {   // Block transfer mode 
                unsigned long AH : 8;  // DTC retain block size 
                unsigned long AL : 8;  // DTC block size count counter 
                unsigned long B  : 16; // DTC transfer count register B (register that specifies the number of block transfers in block transfer mode) 
            }   BLK;
        }   CR;
    }   REG;
}   DTC_FAMD_STR;

#define     CAN3_REQUEST_DTC_MAX    64

// Transmission/receiving request structure definition 
typedef struct  __rspi_dtc_request__ { // 64byte*128=4096byte = 0x1000 (3D000 to 3DFFF) 
    unsigned short  DAT[8]; // 16 Transmission/receiving buffer 
    DTC_FAMD_STR    DTCTX;  // 16 DTC transmission request structure 
    DTC_FAMD_STR    DTCRX;  // 16 DTC receiving    request structure 
    int             TXL;    // 4  Number of transmission byte 
    int             RXL;    // 4  Number of receiving    byte 
    unsigned short *RXP;    // 4  Valid receiving data pointer 
    void *          CALL;   // 4  Call destination when transmission/receiving is completed 
}   RSPI_DTC_REQ;

// Transmittion/receiving request chain structure definition 
typedef struct  __rspi_dtc_request_list__ {
    RSPI_DTC_REQ REQ[CAN3_REQUEST_DTC_MAX]; // Request chain 
}   RSPI_REQUESTS;

/* ________________________________________________________________________________________
 *
 * rspi2_init
 * ----------------------------------------------------------------------------------------
 * Function description
 *  RSPI2 initialization
 *        Port        SCI        I2C        SPI        Application
 *   ----------------------------------------------------------------------------
 *   RSPI2 PD2                              MISOC        <RSPI>   CAN3
 *         PD1                              MOSIC        <RSPI>   CAN3
 *         PD3                              RSPCKC       <RSPI>   CAN3
 *         PD4                              SSLC0        <RSPI>   CAN3
 *         PD0                              IRQ0       <- CINT    CAN3
 *         PD6                              IRQ6       <- CRX0BF  CAN3
 *         PD7                              IRQ        <- CRX1BF  CAN3
 *         P90                                         -> CTX0RTS CAN3
 *         P91                                         -> CTX1RTS CAN3
 *         P92                                         -> CTX2RTS CAN3
 *         P93                                         -> CRSET   CAN3
 *         P07                                         <- CSOF    CAN3
 * Argument
 *  speed  Communication speed 100,000 to 10,000,000(Max. 10Mbps)
 * Return
 *  None
 * ________________________________________________________________________________________
 */
extern void rspi2_init(long bps);                   // RSPI2 initialization 

/* ________________________________________________________________________________________
 *
 * MCP2515 control definition
 * ________________________________________________________________________________________
 *
 * Port definition*/
#define CAN3_RESET_PORT  PORT9.PODR.BIT.B3 // Chip reset signal 
#define CAN3_SPI_CS_PORT PORTD.PODR.BIT.B4 // Chip select signal 
#define CAN3_MCPINT_PORT PORTD.PIDR.BIT.B0 // Interrupt        1->0=level 
#define CAN3_RX0BF_PORT  PORTD.PIDR.BIT.B6 // Message received 1->0=level 
#define CAN3_RX1BF_PORT  PORTD.PIDR.BIT.B7 // Message received 1->0=level 
#define CAN3_TX0RTS_PORT PORT9.PODR.BIT.B0 // Request to send  1->0=edge 
#define CAN3_TX1RTS_PORT PORT9.PODR.BIT.B1 // Request to send  1->0=edge 
#define CAN3_TX2RTS_PORT PORT9.PODR.BIT.B2 // Request to send  1->0=edge 

// MCP2515 code definition 
#define MCP2515CMD_READ   0x03 // Read registers in order from the selected address 
#define MCP2515CMD_WRITE  0x02 // Write to register sequentially from selected address 
#define MCP2515CMD_STATUS 0xA0 // Read status bit 
#define MCP2515CMD_BITX   0x05 // Bit change of specific register 

/* ----------------------------------------------------------------------------------------
 * MCP2515 internal address definition*/
enum    __mcp2515_ram_address__ {
    // Filter 
    MCP2515AD_RXF0SIDH  =0x00,MCP2515AD_RXF0SIDL,MCP2515AD_RXF0EID8,MCP2515AD_RXF0EID0,
    MCP2515AD_RXF1SIDH  =0x04,MCP2515AD_RXF1SIDL,MCP2515AD_RXF1EID8,MCP2515AD_RXF1EID0,
    MCP2515AD_RXF2SIDH  =0x08,MCP2515AD_RXF2SIDL,MCP2515AD_RXF2EID8,MCP2515AD_RXF2EID0,
    // Port control 
    MCP2515AD_BFPCTRL   =0x0C,
    MCP2515AD_TXRTSCTRL =0x0D,
    // Status read-only 
    MCP2515AD_CANSTAT   =0x0E,
    MCP2515AD_CANCTRL   =0x0F,
    // Filter 
    MCP2515AD_RXF3SIDH  =0x10,MCP2515AD_RXF3SIDL,MCP2515AD_RXF3EID8,MCP2515AD_RXF3EID0,
    MCP2515AD_RXF4SIDH  =0x14,MCP2515AD_RXF4SIDL,MCP2515AD_RXF4EID8,MCP2515AD_RXF4EID0,
    MCP2515AD_RXF5SIDH  =0x18,MCP2515AD_RXF5SIDL,MCP2515AD_RXF5EID8,MCP2515AD_RXF5EID0,
    // Error counter 
    MCP2515AD_TEC=0x1C,MCP2515AD_REC,MCP2515AD_CANSTAT1,MCP2515AD_CANCTRL1,
    // Mask 
    MCP2515AD_RXM0SIDH  =0x20,MCP2515AD_RXM0SIDL,MCP2515AD_RXM0EID8,MCP2515AD_RXM0EID0,
    MCP2515AD_RXM1SIDH  =0x24,MCP2515AD_RXM1SIDL,MCP2515AD_RXM1EID8,MCP2515AD_RXM1EID0,
    // Configuration 1 
    MCP2515AD_CNFIG3 =0x28,MCP2515AD_CNFIG2,MCP2515AD_CNFIG1,
    // Interrupt enable 
    MCP2515AD_CANINTE =0x2B,MCP2515AD_CANINTF,MCP2515AD_EFLG,MCP2515AD_CANSTAT2,MCP2515AD_CANCTRL2,
    // Transmission buffer 
    MCP2515AD_TXB0CTRL  =0x30,MCP2515AD_TXB0SIDH,MCP2515AD_TXB0SIDL,MCP2515AD_TXB0EID8,MCP2515AD_TXB0EID0,MCP2515AD_TXB0DLC,MCP2515AD_TXB0D0,MCP2515AD_TXB0D1,MCP2515AD_TXB0D2,MCP2515AD_TXB0D3,MCP2515AD_TXB0D4,MCP2515AD_TXB0D5,MCP2515AD_TXB0D6,MCP2515AD_TXB0D7,MCP2515AD_CANSTAT3,MCP2515AD_CANCTRL3,
    MCP2515AD_TXB1CTRL  =0x40,MCP2515AD_TXB1SIDH,MCP2515AD_TXB1SIDL,MCP2515AD_TXB1EID8,MCP2515AD_TXB1EID0,MCP2515AD_TXB1DLC,MCP2515AD_TXB1D0,MCP2515AD_TXB1D1,MCP2515AD_TXB1D2,MCP2515AD_TXB1D3,MCP2515AD_TXB1D4,MCP2515AD_TXB1D5,MCP2515AD_TXB1D6,MCP2515AD_TXB1D7,MCP2515AD_CANSTAT4,MCP2515AD_CANCTRL4,
    MCP2515AD_TXB2CTRL  =0x50,MCP2515AD_TXB2SIDH,MCP2515AD_TXB2SIDL,MCP2515AD_TXB2EID8,MCP2515AD_TXB2EID0,MCP2515AD_TXB2DLC,MCP2515AD_TXB2D0,MCP2515AD_TXB2D1,MCP2515AD_TXB2D2,MCP2515AD_TXB2D3,MCP2515AD_TXB2D4,MCP2515AD_TXB2D5,MCP2515AD_TXB2D6,MCP2515AD_TXB2D7,MCP2515AD_CANSTAT5,MCP2515AD_CANCTRL5,
    // Receiving buffer 
    MCP2515AD_RXB0CTRL  =0x60,MCP2515AD_RXB0SIDH,MCP2515AD_RXB0SIDL,MCP2515AD_RXB0EID8,MCP2515AD_RXB0EID0,MCP2515AD_RXB0DLC,MCP2515AD_RXB0D0,MCP2515AD_RXB0D1,MCP2515AD_RXB0D2,MCP2515AD_RXB0D3,MCP2515AD_RXB0D4,MCP2515AD_RXB0D5,MCP2515AD_RXB0D6,MCP2515AD_RXB0D7,MCP2515AD_CANSTAT6,MCP2515AD_CANCTRL6,
    MCP2515AD_RXB1CTRL  =0x70,MCP2515AD_RXB1SIDH,MCP2515AD_RXB1SIDL,MCP2515AD_RXB1EID8,MCP2515AD_RXB1EID0,MCP2515AD_RXB1DLC,MCP2515AD_RXB1D0,MCP2515AD_RXB1D1,MCP2515AD_RXB1D2,MCP2515AD_RXB1D3,MCP2515AD_RXB1D4,MCP2515AD_RXB1D5,MCP2515AD_RXB1D6,MCP2515AD_RXB1D7,MCP2515AD_CANSTAT7,MCP2515AD_CANCTRL7
};

/* ----------------------------------------------------------------------------------------
 * Response to the device status definition command (MCP2515CMD_STATUS)*/
typedef union   __mcp2515_status__ {
    unsigned char BYTE;
    struct  {
        unsigned char TX2IF : 1; // Transmit 2 interrupt flag 
        unsigned char TXB2R : 1; // Transmit 2 request 
        unsigned char TX1IF : 1; // Transmit 1 interrupt flag 
        unsigned char TXB1R : 1; // Transmit 1 request 
        unsigned char TX0IF : 1; // Transmit 0 interrupt flag 
        unsigned char TXB0R : 1; // Transmit 0 request 
        unsigned char RX1IF : 1; // Receive  1 interrupt flag 
        unsigned char RX0IF : 1; // Receive  0 interrupt flag 
    }   BIT;
}   MCP2515REG_STATUS;

/* ----------------------------------------------------------------------------------------
 * Response to receiving message status definition command (MCP2515CMD_RXSTS)*/
typedef union   __mcp2515_rx_status__ {
    unsigned char BYTE;
    struct  {
        unsigned char MSG  : 2; // Message 0=None / 1=With RXB0 / 2=With RXB1 / 3=With both 
        unsigned char      : 1; // 
        unsigned char TYPE : 2; // Type  0=Standard data / 1=Standard remote / 2=Extended data / 3=Extended remote 
        unsigned char FILT : 3; // Match filter 0 to 5=RXF0 to 5 / 6=RXF0(Forward to RXB1) / 7=RXF1(Forward to RXB1) 
    }   BIT;
}   MCP2515REG_RX_STS;

/* ----------------------------------------------------------------------------------------
 * RXnBF pin control / Status notification register definition  Address = 0x0C*/
typedef union   __mcp2515_bfp_ctrl__ {
    unsigned char BYTE;
    struct  {
        unsigned char       : 2; // [00] 
        unsigned char B1BFS : 1; // RX1BF pin status when output mode / [0] when receive interrupt mode 
        unsigned char B0BFS : 1; // RX0BF pin status when output mode / [0] when receive interrupt mode 
        unsigned char B1BFE : 1; // RX1BF pin function enable 1=enable / 0=disable(HiZ) 
        unsigned char B0BFE : 1; // RX0BF pin function enable 1=enable / 0=disable(HiZ) 
        unsigned char B1BFM : 1; // RX1BF pin mode setting 1=Interrupt output when receiving RXB1 / 0=Output B1BFS value 
        unsigned char B0BFM : 1; // RX0BF pin mode setting 1=Interrupt output when receiving RXB0 / 0=Output B1BFS value 
    }   BIT;
}   MCP2515REG_BFP_CTRL;

/* ----------------------------------------------------------------------------------------
 * TXnRTS pin control / Status notification register definition Address = 0x0D*/
typedef union   __mcp2515_txrts_ctrl__ {
    unsigned char BYTE;
    struct  {
        unsigned char        : 2; // [00] 
        unsigned char B2RTS  : 1; // TX2RTS pin status when input mode / [0] when transmit request mode 
        unsigned char B1RTS  : 1; // TX1RTS pin status when input mode / [0] when transmit request mode 
        unsigned char B0RTS  : 1; // TX0RTS pin status when input mode / [0] when transmit request mode 
        unsigned char B2RTSM : 1; // TX2RTS pin mode setting 1=Input TXB2 transmit request (↓Edge) / 0=Input 
        unsigned char B1RTSM : 1; // TX1RTS pin mode setting 1=Input TXB1 transmit request (↓Edge) / 0=Input 
        unsigned char B0RTSM : 1; // TX0RTS pin mode setting 1=Input TXB0 transmit request (↓Edge) / 0=Input 
    }   BIT;
}   MCP2515REG_TXRTS_CTRL;

/* ----------------------------------------------------------------------------------------
 * CAN status register       Address = 0xXE*/
typedef union   __mcp2515_stat__ {
    unsigned char BYTE;
    struct  {
        unsigned char OPMOD : 3; // Operation mode 0=Normal / 1=Sleep / 2=Loopback / 3=Listen / 4=Config 
        unsigned char       : 1; // [0] 
        unsigned char ICOD  : 3; // Interrupt clag code 0=None / 1=Error / 2=Wakeup / 3=TXB0 / 4=TXB1 / 5=TXB2 / 6=RXB0 / 7=RXB1 
        unsigned char       : 1; // [0] 
    }   BIT;
}   MCP2515REG_STAT;

/* ----------------------------------------------------------------------------------------
 * CAN control register       Address = 0xXF*/
typedef union   __mcp2515_ctrl__ {
    unsigned char BYTE;
    struct  {
        unsigned char REQOP  : 3; // Operation mode request bit  0=Normal / 1=Sleep / 2=Loopback / 3=Listen / 4=Config 
        unsigned char ABAT   : 1; // Stop all transmissions      1=Request to stop transmission of all transmission buffers / 0=Request to stop transmission 
        unsigned char OSM    : 1; // One-shot mode               0=Transmit every request / 1=Transmit only once 
        unsigned char CLKEN  : 1; // CLKOUT pin enable           0=Disable / 1=Enable 
        unsigned char CLKPRE : 2; // CLKOUT pin division setting 0=x/1 / 1=x/2 / 2=x/4 / 3=x/8 
    }   BIT;
}   MCP2515REG_CTRL;

/* ----------------------------------------------------------------------------------------
 * CNF3 configration 1    Address = 0x28*/
typedef union   __mcp2515_cnf3_config__ {
    unsigned char BYTE;
    struct  {
        unsigned char SOF    : 1; // Start of frame CLKOUT/SOF pin 0=Clock output / 1=SOF signal 
        unsigned char WAKFIL : 1; // Wakeup filter 0=Disable / 1=Enable 
        unsigned char        : 3; // [000] 
        unsigned char PHSEG2 : 3; // PS2 length 0 to 7 (PHSEG2+1)*TQ 
    }   BIT;
}   MCP2515REG_CNF3;

/* ----------------------------------------------------------------------------------------
 * CNF2 configration 1    Address = 0x29*/
typedef union   __mcp2515_cnf2_config__ {
    unsigned char BYTE;
    struct  {
        unsigned char BTLMODE : 1; // PS2 bit time length 0=PS2 length is greater than PS1 and IPT (2TQ) / 1=PS2 length is determined by PHSEG2 of CNF3 
        unsigned char SAM     : 1; // Sample point configuration 0=1 time / 1=3 times sample 
        unsigned char PHSEG1  : 3; // PS1 length     0 to 7 (PHSEG1+1)*TQ 
        unsigned char PHSEG   : 3; // Propagation segment length 0 to 7 (PHSEG+1)*TQ 
    }   BIT;
}   MCP2515REG_CNF2;

/* ----------------------------------------------------------------------------------------
 * CNF1 configration 1    Address = 0x2A*/
typedef union   __mcp2515_cnf1_config__ {
    unsigned char BYTE;
    struct  {
        unsigned char SJW : 2; // Resynchronization jump width length (SJW+1)*TQ 
        unsigned char BRP : 6; // Baud rate divider  TQ=2*(BRP+1)/Fosc 
    }   BIT;
}   MCP2515REG_CNF1;

/* ----------------------------------------------------------------------------------------
 * Interrupt enable       Address = 0x2B*/
typedef union   __mcp2515_intf__ {
    unsigned char BYTE;
    struct  {
        unsigned char MERRE : 1; // Message error interrupt           0=Disable / 1=Enable 
        unsigned char WAKIE : 1; // Wake-up interrupt                 0=Disable / 1=Enable 
        unsigned char ERRIE : 1; // Error interrupt                   0=Disable / 1=Enable 
        unsigned char TX2IE : 1; // Transmit buffer 2 empty interrupt 0=Disable / 1=Enable 
        unsigned char TX1IE : 1; // Transmit buffer 1 empty interrupt 0=Disable / 1=Enable 
        unsigned char TX0IE : 1; // Transmit buffer 0 empty interrupt 0=Disable / 1=Enable 
        unsigned char RX1IE : 1; // Receive buffer 1 full interrupt   0=Disable / 1=Enable 
        unsigned char RX0IE : 1; // Receive buffer 0 full interrupt   0=Disable / 1=Enable 
    }   BIT;
}   MCP2515REG_CANINTE;

/* ----------------------------------------------------------------------------------------
 * Interrupt flag       Address = 0x2C*/
typedef union   __mcp2515_intr__ {
    unsigned char BYTE;
    struct  {
        unsigned char MERRF : 1; // Message error interrupt           0 = None / 1 = Wait for interrupt 
        unsigned char WAKIF : 1; // Wake-up interrupt                 0 = None / 1 = Wait for interrupt 
        unsigned char ERRIF : 1; // Error interrupt                   0 = None / 1 = Wait for interrupt 
        unsigned char TX2IF : 1; // Transmit buffer 2 empty interrupt 0 = None / 1 = Wait for interrupt 
        unsigned char TX1IF : 1; // Transmit buffer 1 empty interrupt 0 = None / 1 = Wait for interrupt 
        unsigned char TX0IF : 1; // Transmit buffer 0 empty interrupt 0 = None / 1 = Wait for interrupt 
        unsigned char RX1IF : 1; // Receive buffer 1 full interrupt   0 = None / 1 = Wait for interrupt 
        unsigned char RX0IF : 1; // Receive buffer 0 full interrupt   0 = None / 1 = Wait for interrupt 
    }   BIT;
}   MCP2515REG_CANINTF;

/* ----------------------------------------------------------------------------------------
 * Error flag       Address = 0x2D*/
typedef union   __mcp2515_errfalg__ {
    unsigned char BYTE;
    struct  {
        unsigned char RX1OVR : 1; // Receive buffer 1 overflow          1=Occur 
        unsigned char RX0OVR : 1; // Receive buffer 1 overflow          1=Occur 
        unsigned char TXBO   : 1; // Bus off error flag                 1=Occur 
        unsigned char TXEP   : 1; // Transmission error / Passive error 1=Occur 
        unsigned char RXEP   : 1; //      Receive error / Passive error 1=Occur 
        unsigned char TXWAR  : 1; // Transmission error warning         1=Occur 
        unsigned char RXWAR  : 1; //      Receive error warning         1=Occur 
        unsigned char EWARN  : 1; // TXWAR or RXWAR is occurring        1=Occur 
    }   BIT;
}   MCP2515REG_EFLG;

/* ----------------------------------------------------------------------------------------
 * Transmit buffer control register definition   Address = 0x30 / 0x40 / 0x50*/
typedef union   __mcp2515_txb_ctrl__ {
    unsigned char BYTE;
    struct  {
        unsigned char       : 1; // [0] 
        unsigned char ABTF  : 1; // Message stop flag                1=Message has been stopped / 0=Message transmission completed successfully 
        unsigned char MLOA  : 1; // Arbitration disappeared          1=disappeared / 0=existed 
        unsigned char TXERR : 1; // Transmission error detection bit 1=Bus error occurred / 0=Normal 
        unsigned char TXREQ : 1; // Transmission request bit         1=Waiting for transmission  / 0=No waiting 
        unsigned char       : 1; // [0] 
        unsigned char TXP   : 2; // Transmission priority            0=Lowest to 3=Highest 
    }   BIT;
}   MCP2515REG_TXB_CTRL;

/* ----------------------------------------------------------------------------------------
 * TXBn Transmission data buffer structure definition  Address = 0x31 to 0x3D / 0x41 to 0x4D / 0x51 to 0x5D*/
typedef struct  __mcp2515_txb__ {
    // Transmit buffer control register definition   Address = 0x30 / 0x40 / 0x50 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char       : 1; // [0] 
            unsigned char ABTF  : 1; // Message stop flag                1=Message has been stopped / 0=Message transmission completed successfully 
            unsigned char MLOA  : 1; // Arbitration disappeared          1=disappeared / 0=existed 
            unsigned char TXERR : 1; // Transmission error detection bit 1=Bus error occurred / 0=Normal 
            unsigned char TXREQ : 1; // Transmission request bit         1=Waiting for transmission  / 0=No waiting 
            unsigned char       : 1; // [0] 
            unsigned char TXP   : 2; // Transmission priority            0=Lowest to 3=Highest 
        }   BIT;
    }   CTRL;
    // TXBnSIDH register definition Address = 0x31 / 0x41 / 0x51 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID10 : 1; 
            unsigned char SID9  : 1; 
            unsigned char SID8  : 1; 
            unsigned char SID7  : 1; 
            unsigned char SID6  : 1; 
            unsigned char SID5  : 1; 
            unsigned char SID4  : 1; 
            unsigned char SID3  : 1; 
        }   BIT;
    }   SIDH;
    // TXBnSIDL register definition Address = 0x32 / 0x42 / 0x52 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID2  : 1; // 
            unsigned char SID1  : 1; // 
            unsigned char SID0  : 1; // 
            unsigned char       : 1; // [0] 
            unsigned char EXIDE : 1; // Extended identifier enable bit 1=Extended / 0=Standard 
            unsigned char       : 1; // [0] 
            unsigned char EID17 : 1; // 
            unsigned char EID16 : 1; // 
        }   BIT;
    }   SIDL;
    // TXBnEID8 register definition Address = 0x33 / 0x43 / 0x53 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID15 : 1; 
            unsigned char EID14 : 1; 
            unsigned char EID13 : 1; 
            unsigned char EID12 : 1; 
            unsigned char EID11 : 1; 
            unsigned char EID10 : 1; 
            unsigned char EID9  : 1; 
            unsigned char EID8  : 1; 
        }   BIT;
    }   EID8;
    // TXBnEID0 register definition Address = 0x34 / 0x44 / 0x54 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID7 : 1; 
            unsigned char EID6 : 1; 
            unsigned char EID5 : 1; 
            unsigned char EID4 : 1; 
            unsigned char EID3 : 1; 
            unsigned char EID2 : 1; 
            unsigned char EID1 : 1; 
            unsigned char EID0 : 1; 
        }   BIT;
    }   EID0;
    // TXBnDLC register definition  Address = 0x35 / 0x45 / 0x55 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char     : 1; // [0] 
            unsigned char RTR : 1; // Remote transmission request 1=Remote transmission / 0=Data transmission 
            unsigned char     : 2; // [00] 
            unsigned char DLC : 4; // Data length   0 to 8 bytes 
        }   BIT;
    }   DLC;
    // TXBnDATA register definition  Address = 0x36 to 3D / 0x46 to 4D / 0x56 to 5D 
    unsigned char DATA[8];
}   MCP2515REG_TXB;

/* ----------------------------------------------------------------------------------------
 * RXBnCTRL register definition  Address = 0x60 / 0x70*/
typedef union   __mcp2515_rxb_ctrl__ {
    unsigned char BYTE;
    struct  {
        unsigned char         : 1; // [0] 
        unsigned char RXM     : 2; // Buffer operation mode 3=Receive all messages / 2=Extended match / 1=Standard match / 0=Match either standard or extended 
        unsigned char         : 1; // [0] 
        unsigned char RTR     : 1; // Remote transmission request 1=Remote request received / 0=No remote request 
        unsigned char BUKT    : 1; // Switching enabled 1=RXB1 received when RXB0 is full 
        unsigned char BUKT1   : 1; // [R] Same as above for internal use 
        unsigned char FILHIT0 : 1; // Filter match 1=RXF1 / 0=RXF0 
    }   BIT0;
    struct  {
        unsigned char        : 1; // [0] 
        unsigned char RXM    : 2; // Buffer operation mode 3=Receive all messages / 2=Extended match / 1=Standard match / 0=Match either standard or extended 
        unsigned char        : 1; // [0] 
        unsigned char RTR    : 1; // Remote transmission request 1=Remote request received / 0=No remote request 
        unsigned char FILHIT : 3; // Filter match  5 to 0=RXF5 to RXF0 
    }   BIT1;
}   MCP2515REG_RXB_CTRL;

/* ----------------------------------------------------------------------------------------
 * RXBn receiving data buffer structure definition  Address = 0x61 to 0x6D / 0x71 to 0x7D*/
typedef struct  __mcp2515_rxb__ {
    // RXBnCTRL register definition  Address = 0x60 / 0x70 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char         : 1; // [0] 
            unsigned char RXM     : 2; // Buffer operation mode 3=Receive all messages / 2=Extended match / 1=Standard match / 0=Match either standard or extended 
            unsigned char         : 1; // [0] 
            unsigned char RTR     : 1; // Remote transmission request 1=Remote request received / 0=No remote request 
            unsigned char BUKT    : 1; // Switching enabled 1=RXB1 received when RXB0 is full 
            unsigned char BUKT1   : 1; // [R] Same as above for internal use 
            unsigned char FILHIT0 : 1; // Filter match  1=RXF1 / 0=RXF0 
        }   BIT0;
        struct  {
            unsigned char         : 1; // [0] 
            unsigned char RXM     : 2; // Buffer operation mode 3=Receive all messages / 2=Extended match / 1=Standard match / 0=Match either standard or extended 
            unsigned char         : 1; // [0] 
            unsigned char RTR     : 1; // Remote send request 1=Remote request received / 0=No remote request 
            unsigned char FILHIT  : 3; // Filter match 5 to 0=RXF5 to RXF0 
        }   BIT1;
    }   CTRL;
    // RXBnSIDH register difinition Address = 0x61 / 0x71 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID10 : 1; 
            unsigned char SID9  : 1; 
            unsigned char SID8  : 1; 
            unsigned char SID7  : 1; 
            unsigned char SID6  : 1; 
            unsigned char SID5  : 1; 
            unsigned char SID4  : 1; 
            unsigned char SID3  : 1; 
        }   BIT;
    }   SIDH;
    // RXBnSIDL register difinition Address = 0x62 / 0x72 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID2  : 1; // 
            unsigned char SID1  : 1; // 
            unsigned char SID0  : 1; // 
            unsigned char RTR   : 1; // Standard frame remote request   1=Remote transmission request reception / 0=Data frame reception 
            unsigned char IDE   : 1; // Extended identifier enable bit  1=Extended / 0=Standard 
            unsigned char       : 1; // [0] 
            unsigned char EID17 : 1; // 
            unsigned char EID16 : 1; // 
        }   BIT;
    }   SIDL;
    // RXBnEID8 register difinition Address = 0x63 / 0x73 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID15 : 1; 
            unsigned char EID14 : 1; 
            unsigned char EID13 : 1; 
            unsigned char EID12 : 1; 
            unsigned char EID11 : 1; 
            unsigned char EID10 : 1; 
            unsigned char EID9  : 1; 
            unsigned char EID8  : 1; 
        }   BIT;
    }   EID8;
    // RXBnEID0 register difinition Address = 0x64 / 0x74 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID7 : 1; 
            unsigned char EID6 : 1; 
            unsigned char EID5 : 1; 
            unsigned char EID4 : 1; 
            unsigned char EID3 : 1; 
            unsigned char EID2 : 1; 
            unsigned char EID1 : 1; 
            unsigned char EID0 : 1; 
        }   BIT;
    }   EID0;
    // RXBnDLC register difinition  Address = 0x65 / 0x75 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char      : 1; // [0] 
            unsigned char ERTR : 1; // Extended frame remote transmission request 1=Send remotely / 0=Send data 
            unsigned char RB   : 2; // Reservation bit 
            unsigned char DLC  : 4; // Data length   0 to 8 bytes 
        }   BIT;
    }   DLC;
    // RXBnDATA register difinition Address = 0x66 to 6D / 0x76 to 7D 
    unsigned char DATA[8];
}   MCP2515REG_RXB;

/* ----------------------------------------------------------------------------------------
 * RXFn filter register difinition  Address = 0x00 to 0x03 / 0x04 to 0x07 / 0x08 to 0x0B / 0x10 to 0x13 / 0x14 to 0x17 / 0x18 to 0x1B*/
typedef struct  __mcp2515_rxfn__ {
    // RXFnSIDH register difinition Address = 0x00 / 0x04 / 0x08 / 0x10 / 0x14 / 0x18 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID10 : 1; 
            unsigned char SID9  : 1; 
            unsigned char SID8  : 1; 
            unsigned char SID7  : 1; 
            unsigned char SID6  : 1; 
            unsigned char SID5  : 1; 
            unsigned char SID4  : 1; 
            unsigned char SID3  : 1; 
        }   BIT;
    }   SIDH;
    // RXFnSIDL register difinition Address = 0x01 / 0x05 / 0x09 / 0x11 / 0x15 / 0x19 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID2  : 1; // 
            unsigned char SID1  : 1; // 
            unsigned char SID0  : 1; // 
            unsigned char       : 1; // [0] 
            unsigned char EXIDE : 1; // Extended identifier enable bit 1=Extension only / 0=Standard only 
            unsigned char       : 1; // [0] 
            unsigned char EID17 : 1; // 
            unsigned char EID16 : 1; // 
        }   BIT;
    }   SIDL;
    // RXFnEID8 register difinition Address = 0x02 / 0x06 / 0x0A / 0x12 / 0x16 / 0x1A 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID15 : 1; 
            unsigned char EID14 : 1; 
            unsigned char EID13 : 1; 
            unsigned char EID12 : 1; 
            unsigned char EID11 : 1; 
            unsigned char EID10 : 1; 
            unsigned char EID9  : 1; 
            unsigned char EID8  : 1; 
        }   BIT;
    }   EID8;
    // RXFnEID0 register difinition Address = 0x03 / 0x07 / 0x0B / 0x13 / 0x17 / 0x1B 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID7 : 1; 
            unsigned char EID6 : 1; 
            unsigned char EID5 : 1; 
            unsigned char EID4 : 1; 
            unsigned char EID3 : 1; 
            unsigned char EID2 : 1; 
            unsigned char EID1 : 1; 
            unsigned char EID0 : 1; 
        }   BIT;
    }   EID0;
}   MCP2515REG_RXF;

/* ----------------------------------------------------------------------------------------
 * RXMn Receive mask register definition Address = 0x20 to 0x23 / 0x24 to 0x27*/
typedef struct  __mcp2515_rxm__ {
    // RXMnSIDH register difinition Address = 0x20 / 0x24 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID10 : 1; 
            unsigned char SID9  : 1; 
            unsigned char SID8  : 1; 
            unsigned char SID7  : 1; 
            unsigned char SID6  : 1; 
            unsigned char SID5  : 1; 
            unsigned char SID4  : 1; 
            unsigned char SID3  : 1; 
        }   BIT;
    }   SIDH;
    // RXMnSIDL register difinition Address = 0x21 / 0x25 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char SID2  : 1; // 
            unsigned char SID1  : 1; // 
            unsigned char SID0  : 1; // 
            unsigned char       : 1; // [0] 
            unsigned char       : 1; // [0] 
            unsigned char       : 1; // [0] 
            unsigned char EID17 : 1; // 
            unsigned char EID16 : 1; // 
        }   BIT;
    }   SIDL;
    // RXMnEID8 register difinition Address = 0x22 / 0x26 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID15 : 1; 
            unsigned char EID14 : 1; 
            unsigned char EID13 : 1; 
            unsigned char EID12 : 1; 
            unsigned char EID11 : 1; 
            unsigned char EID10 : 1; 
            unsigned char EID9  : 1; 
            unsigned char EID8  : 1; 
        }   BIT;
    }   EID8;
    // RXMnEID0 register difinition Address = 0x23 / 0x27 
    union   {
        unsigned char BYTE;
        struct  {
            unsigned char EID7 : 1; 
            unsigned char EID6 : 1; 
            unsigned char EID5 : 1; 
            unsigned char EID4 : 1; 
            unsigned char EID3 : 1; 
            unsigned char EID2 : 1; 
            unsigned char EID1 : 1; 
            unsigned char EID0 : 1; 
        }   BIT;
    }   EID0;
}   MCP2515REG_RXM;

/* ----------------------------------------------------------------------------------------
 * MCP2515 internal register structure definition
 * ---------------------------------------------------------------------------------------- */
typedef union   __mcp2515_reg__ {
    unsigned long   LONG[64];
    unsigned short  WORD[128];
    unsigned char BYTE[256];
    struct  {
        // Filter 0 to 2                                                   Address = 0x00 to 0x0B 
        MCP2515REG_RXF RXF0[3];
        // RXnBF pin control / status notification register definition     Address = 0x0C 
        MCP2515REG_BFP_CTRL BFP_CTRL;
        // TXnRTS Pin control / status notification register definition    Address = 0x0D 
        MCP2515REG_TXRTS_CTRL TXRTS_CTRL;
        // CAN status register                                             Address = 0x0E 
        MCP2515REG_STAT CANSTAT0;
        // CAN control register                                            Address = 0x0F 
        MCP2515REG_CTRL CANCTRL0;
        // Filter 3 to 5                                                   Address = 0x10 to 0x1B 
        MCP2515REG_RXF RXF1[3];
        // Transmission error counter                                      Address = 0x1C 
        unsigned char TEC;
        // Receiving error counter                                         Address = 0x1D 
        unsigned char REC;
        // CAN status register                                             Address = 0x1E 
        MCP2515REG_STAT CANSTAT1;
        // CAN control register                                            Address = 0x1F 
        MCP2515REG_CTRL CANCTRL1;
        // Mask 0 to 1                                                     Address = 0x20 to 0x27 
        MCP2515REG_RXM RXM[2];
        // CNF3 configration 1                                             Address = 0x28 
        MCP2515REG_CNF3 CNF3;
        // CNF2 configration 1                                             Address = 0x29 
        MCP2515REG_CNF2 CNF2;
        // CNF1 configration 1                                             Address = 0x2A 
        MCP2515REG_CNF1 CNF1;
        // Interrupt enable                                                Address = 0x2B 
        MCP2515REG_CANINTE CANINTE;
        // Interrupt flag                                                  Address = 0x2C 
        MCP2515REG_CANINTF CANINTF;
        // Error flag                                                      Address = 0x2D 
        MCP2515REG_CANINTF EFLG;
        // CAN status register                                             Address = 0x2E 
        MCP2515REG_STAT CANSTAT2;
        // CAN control register                                            Address = 0x2F 
        MCP2515REG_CTRL CANCTRL2;
        // Transmission buffer 0 to 2                                      Address = 0x30 to 0x5F 
        struct  {
            // Transmission buffer control register                        Address = 0x30 / 0x40 / 0x50 
            MCP2515REG_TXB_CTRL TXBCTRL;
            // Transmission buffer frame                                   Address = 0x31 to 0x3D / 0x41 to 0x4D / 0x51 to 0x5D 
            MCP2515REG_TXB FRAME;
            // CAN status register                                         Address = 0xXE 
            MCP2515REG_STAT STAT;
            // CAN control register                                        Address = 0xXF 
            MCP2515REG_CTRL CTRL;
        }   TXB[3];
        // Receiving buffer 0,1                                            Address = 0x60 to 0x7F 
        struct  {
            // Receiving buffer control register                           Address = 0x60 / 0x70 
            MCP2515REG_RXB_CTRL RXBCTRL;
            // Receiving buffer frame                                      Address = 0x61 to 0x6D / 0x71 to 0x7D 
            MCP2515REG_RXB FRAME;
            // CAN status register                                         Address = 0xXE 
            MCP2515REG_STAT STAT;
            // CAN control register                                        Address = 0xXF 
            MCP2515REG_CTRL CTRL;
        }   RXB[2];
    }   REG;
}   MCP2515_REGMAP;

/* ----------------------------------------------------------------------------------------
 * TXnRTS/RXnBF pin control register definition  Address = 0x0C / 0x0D*/
typedef union   __mcp2515_bfp_rts_ctrl__ {
    unsigned short WORD;
    struct  {
        MCP2515REG_BFP_CTRL   BFP; // RXnBF  pin control / Status notification register definition Address = 0x0C 
        MCP2515REG_TXRTS_CTRL RTS; // TXnRTS pin control / Status notification register definition Address = 0x0D 
    }   BYTE;
}   MCP2515REG_BFPRTS_CTRL;

/* ----------------------------------------------------------------------------------------
 * CAN control register       Address = 0xXE / 0xXF*/
typedef union   __mcp2515_stat_ctrl__ {
    unsigned short WORD;
    struct  {
        MCP2515REG_STAT STAT; // Status 
        MCP2515REG_CTRL CTRL; // Control 
    }   BYTE;
}   MCP2515REG_STATCTRL;

/* ----------------------------------------------------------------------------------------
 * Communication error counter     Address = 0x1C / 0x1D*/
typedef union   __mcp2515_errcnt__ {
    unsigned short WORD;
    struct  {
        // Transmission error counter Address = 0x1C 
        unsigned char TEC;
        // Receiving error counter Address = 0x1D 
        unsigned char REC;
    }   BYTE;
}   MCP2515REG_ERRCNT;


/* ----------------------------------------------------------------------------------------
 * CAN configration 1    Address = 0x28 to 0x2B*/
typedef union   __mcp2515_config__ {
    unsigned short WORD[2];
    struct  {
        // CNF3 configration 1     Address = 0x28 
        MCP2515REG_CNF3 CNF3;
        // CNF2 configration 1     Address = 0x29 
        MCP2515REG_CNF2 CNF2;
        // CNF1 configration 1     Address = 0x2A 
        MCP2515REG_CNF1 CNF1;
        // Interrupt enable        Address = 0x2B 
        MCP2515REG_CANINTE CANINTE;
        // Interrupt flag          Address = 0x2C 
        MCP2515REG_CANINTF CANINTF;
        // Error flag              Address = 0x2D 
        MCP2515REG_EFLG EFLG;
    }   BYTE;
}   MCP2515REG_CONFIG;

/* ----------------------------------------------------------------------------------------
 * CAN configration 1    Address = 0x28 to 0x2B*/
typedef union   __mcp2515_intr_eflg__ {
    unsigned short WORD;
    struct  {
        // Interrupt flag          Address = 0x2C 
        MCP2515REG_CANINTF CANINTF;
        // Error flag              Address = 0x2D 
        MCP2515REG_EFLG EFLG;
    }   BYTE;
}   MCP2515REG_INTERR;

/* ----------------------------------------------------------------------------------------
 * TXBnCTRL register definition  Address = 0x30 / 0x40 / 0x50*/
typedef union   __mcp2515_txb_ctrl_buf__ {
    unsigned short  WORD[7];
    unsigned char BYTE[14];
    struct  {
        MCP2515REG_TXB TXB; // Transmission buffer 
    }   REG;
}   MCP2515REG_TXBUF;

/* ----------------------------------------------------------------------------------------
 * RXBnCTRL register definition  Address = 0x60 / 0x70*/
typedef union   __mcp2515_rxb_ctrl_buf__ {
    unsigned short  WORD[7];
    unsigned char BYTE[14];
    struct  {
        MCP2515REG_RXB RXB; // Transmission buffer 
    }   REG;
}   MCP2515REG_RXBUF;

/* ----------------------------------------------------------------------------------------
 * Bit set clear message definition    Command (MCP2515CMD_BITX) data*/
typedef union   __mcp2515_bit_set_clear__ {
    unsigned short WORD;
    struct  {
        union   {
            unsigned char         BYTE;    // Mask pattern (1=Change / 0=Keep) 
            MCP2515REG_TXB_CTRL   TXBCTRL; // Transmission buffer control 
            MCP2515REG_RXB_CTRL   RXBCTRL; //    Receiving buffer control 
            MCP2515REG_CNF3       CNF3;    // CNF3 configration 1  Address = 0x28 
            MCP2515REG_CNF2       CNF2;    // CNF2 configration 1  Address = 0x29 
            MCP2515REG_CNF1       CNF1;    // CNF1 configration 1  Address = 0x2A 
            MCP2515REG_CANINTE    INTE;    // Interrupt enable flag 
            MCP2515REG_CANINTF    INTF;    // Interrupt flag 
            MCP2515REG_EFLG       EFLG;    // Error flag 
            MCP2515REG_BFP_CTRL   BFP;     // RXnBF  pin control / Status notification register definition Address = 0x0C 
            MCP2515REG_TXRTS_CTRL RTS;     // TXnRTS pin control / Status notification register definition Address = 0x0D 
            MCP2515REG_CTRL       CTRL;    // CAN control register 
        }   MSK;
        union   {
            unsigned char         BYTE;    // Bit pattern 
            MCP2515REG_TXB_CTRL   TXBCTRL; // Transmission buffer control 
            MCP2515REG_RXB_CTRL   RXBCTRL; // Receiving buffer control 
            MCP2515REG_CNF3       CNF3;    // CNF3 configration 1  Address = 0x28 
            MCP2515REG_CNF2       CNF2;    // CNF2 configration 1  Address = 0x29 
            MCP2515REG_CNF1       CNF1;    // CNF1 configration 1  Address = 0x2A 
            MCP2515REG_CANINTE    INTE;    // Interrupt enable flag 
            MCP2515REG_CANINTF    INTF;    // Interrupt flag 
            MCP2515REG_EFLG       EFLG;    // Error flag 
            MCP2515REG_BFP_CTRL   BFP;     // RXnBF  pin control / Status notification register definition Address = 0x0C 
            MCP2515REG_TXRTS_CTRL RTS;     // TXnRTS pin control / Status notification register definition Address = 0x0D 
            MCP2515REG_CTRL       CTRL;    // CAN control register 
        }   PAT;
    }   BYTE;
}   MCP2515REG_BITX;

/* ----------------------------------------------------------------------------------------
 * MCP2515 operation job number definition
 * ---------------------------------------------------------------------------------------- */
enum    mcp2515_job {
    CAN3_JOB_INIT=0,    // Device initialization 
    CAN3_JOB_IW1,       // 
    CAN3_JOB_IW2,       // 
    CAN3_JOB_IW3,       // 
    CAN3_JOB_IW4,       // 
    CAN3_JOB_IW5,       // 
    CAN3_JOB_IW6,       // 
    CAN3_JOB_IW7,       // 
    CAN3_JOB_IW8,       // 
    CAN3_JOB_IW9,       // 

    CAN3_JOB_WAIT,      // Waiting for operation 
    CAN3_JOB_WW1,       // 
    CAN3_JOB_WW2,       // 
    CAN3_JOB_WW3,       // 
    CAN3_JOB_WW4,       // 
    CAN3_JOB_WW5,       // 

    CAN3_JOB_CHECK,     // Status check 


    CAN3_JOB_OVER
};

/* ----------------------------------------------------------------------------------------
 * can3_init
 * 
 *  Fnction description
 *      Initialization of MCP2515(via RSPI2)
 * 
 *  Argument
 *      None
 * 
 *  Return
 *      None
 * ---------------------------------------------------------------------------------------- */
extern RSPI_REQUESTS *  rspi_req;    // Transmit / receive request chain variable reference 
extern RSPI_DTC_REQ *   can3_now;    // Request during transmission / reception 
extern int              can3_job_id; // Processing number 

extern void can3_init(void); // CAN3 port initialization 
extern int  can3_job(void);  // Initialization JOB 

// Write outgoing mailbox 
extern int  CAN3_TxSet(int mb, SEND_WAIT_FLAME *act);
extern int  CAN3_GetTxMCTL(int mb); // Confirmation of transmission buffer free space 

/* ---------------------------------------------------------------------------------------
 *  Callback processing after interrupt status acquisition
 * --------------------------------------------------------------------------------------- */
extern void can3_sts_event(MCP2515REG_INTERR *rxd);

#endif //RSPI2_ACTIVATE
#endif //__CAN2ECU_CAN3RSPI2_IF__

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
 * MCP2515 CAN controller RSPI2-I/F communication
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 12/01/2016    Start coding (by Tachibana)
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
#include "timer.h"
#include "ecu.h"       // ECU common definition 
#include "can3_spi2.h" // CAN3 definition 
#include "cantp.h"     // CAN-TP definition 

// Enable if usage of mailbox is fixed 
#define MB_LOCKED_TYPE
/* Enable if only one mailbox is used
 *#define MB_USED_ONLYONE*/

/*
 *  Port settings
 *
 *          Port        SCI            I2C            SPI            Application
 *  ----------------------------------------------------------------------------
 *  RSPI2   PD2                                    MISOC        <RSPI>        CAN3
 *          PD1                                    MOSIC        <RSPI>        CAN3
 *          PD3                                    RSPCKC       <RSPI>        CAN3
 *          PD4                                    SSLC0        <RSPI>        CAN3
 *          PD0                                    IRQ0       <- CINT         CAN3
 *          PD6                                    IRQ6       <- CRX0BF       CAN3
 *          PD7                                    IRQ7       <- CRX1BF       CAN3
 *          P90                                               -> CTX0RTS      CAN3
 *          P91                                               -> CTX1RTS      CAN3
 *          P92                                               -> CTX2RTS      CAN3
 *          P93                                               -> CRSET        CAN3
 *          P07                                               <- CSOF         CAN3
 *
 *  DTC settings
 *
 *  DTC vector base    DTCVBR     $0003E000-$0003EFFF
 *
 *  Receiving DTC vector    DTCE_RSPI2_SPRI2    45        0003E0B4
 *  Sending   DTC vector    DTCE_RSPI2_SPTI2    46        0003E0B8
 */

// Log function 
void logging(char *fmt, ...);

#ifdef      RSPI2_ACTIVATE

/* ----------------------------------------------------------------------------------------
 * Variable definition for CAN3 port
 * ----------------------------------------------------------------------------------------
 */

// Refer to transmit/receive request chain variable 
RSPI_DTC_REQ *  can3_now;           // Request buffer pointer during transfer 
RSPI_REQUESTS * rspi_req;           // Request buffer 
int             rspi_req_WP = 0;    // Write position 
int             rspi_req_RP = 0;    // Read position 
int             rspi_req_PP = 0;    // Processing position 

// Sequence 
int can3_job_id     = CAN3_JOB_INIT;// Order processing 
int stat_event_flag = 0;            // Status reception event flag 

// Wait-to-transmit flag 
int tx_act[3]       = {0,0,0};
int tx_act_timer[3] = {0,0,0};

// Prototype 
int             can_recv_frame(int ch, void *mbox);
void            can_tx_mb(int ch, int mb);
RSPI_DTC_REQ *  can3_request(int cmd, int adr, int txlen, int rxlen, void *proc, void *data);

unsigned long *dtc_table = DTC_VECT_TOP;

void dtc_init(void)
{
    SYSTEM.PRCR.WORD    = 0xA503;   // Unlock
    MSTP_DTC            = 0;        // DTC module stop release 
    SYSTEM.PRCR.WORD    = 0xA500;   // Lock 

    DTC.DTCST.BIT.DTCST = 0;        /* DTC module start bit
                                     * 0:DTC module stop
                                     * 1:DTC module start*/
    memset(dtc_table, 0, 0x1000);   // Vector initialize 
    DTC.DTCVBR          = DTC_VECT_TOP; // DTC vector address setting    (0x0003E000) 
    DTC.DTCCR.BIT.RRS   = 0;        /* DTC forward information read/skip permission bit
                                     * 0:Do not allow forward information read/skip
                                     * 1:Allow forward information read/skip if vector numbers match*/
    DTC.DTCADMOD.BIT.SHORT = 0;     /* Short address mode setting bit
                                     * 0:Full address mode
                                     * 1:Short address mode*/
    DTC.DTCST.BIT.DTCST = 1;        /* DTC module start bit
                                     * 0:DTC module stop
                                     * 1:DTC module start*/
}

/* ---------------------------------------------------------------------------------------
 * rspi2_init
 *
 * Function description 
 *     RSPI2 Initialization
 *                 Port        SCI            I2C            SPI            Application
 *         ----------------------------------------------------------------------------
 *         RSPI2   PD2                                    MISOC        <RSPI>        CAN3
 *                 PD1                                    MOSIC        <RSPI>        CAN3
 *                 PD3                                    RSPCKC       <RSPI>        CAN3
 *                 PD4                                    SSLC0        <RSPI>        CAN3
 *                 PD0                                    IRQ0       <- CINT         CAN3
 *                 PD6                                               <- CRX0BF       CAN3
 *                 PD7                                               <- CRX1BF       CAN3
 *                 P90                                               -> CTX0RTS      CAN3
 *                 P91                                               -> CTX1RTS      CAN3
 *                 P92                                               -> CTX2RTS      CAN3
 *                 P93                                               -> CRSET        CAN3
 *                 P07                                               <- CSOF         CAN3
 * Argument
 *     speed        Communication speed    100,000 to 10,000,000(Max. 10Mbps)
 * Return
 *     None
 * ----------------------------------------------------------------------------------------
 */
void rspi2_init(long bps)
{
    SYSTEM.PRCR.WORD = 0xA503; // Unprotect 
    MSTP_RSPI2       = 0;      // RSPI2 module stop release 

    // RSPI2 interrupt request disable 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0; // Group12 Error interrupt disable 
    ICU.IER[IER_RSPI2_SPRI2].BIT.IEN_RSPI2_SPRI2 = 0; // Receive buffer full interrupt disable 
    ICU.IER[IER_RSPI2_SPTI2].BIT.IEN_RSPI2_SPTI2 = 0; // Send empty interrupt disable 
    ICU.IER[IER_RSPI2_SPII2].BIT.IEN_RSPI2_SPII2 = 0; // Send idle interrupt disable 
    ICU.IER[IER_ICU_IRQ0].BIT.IEN_ICU_IRQ0       = 0; // Module interrupt enable 
    ICU.IER[IER_ICU_IRQ6].BIT.IEN_ICU_IRQ6       = 0; // Module interrupt enable 
    ICU.IER[IER_ICU_IRQ7].BIT.IEN_ICU_IRQ7       = 0; // Module interrupt enable 

    RSPI2.SPCR.BYTE = 0;        // Module initialization 

    PORTD.PMR.BIT.B2 = 0; // Peripheral function MISOC 
    PORTD.PMR.BIT.B1 = 0; // Peripheral function MOSIC 
    PORTD.PMR.BIT.B3 = 0; // Peripheral function RSPCKC 
    PORTD.PMR.BIT.B4 = 0; // Peripheral function SSLC0 
    PORTD.PMR.BIT.B0 = 0; // Peripheral function CINT 
    PORTD.PMR.BIT.B6 = 0; // PD6 <- CRX0BF 
    PORTD.PMR.BIT.B7 = 0; // PD7 <- CRX1BF 
    PORT9.PMR.BIT.B0 = 0; // P90 -> CTX0RTS 
    PORT9.PMR.BIT.B1 = 0; // P91 -> CTX1RTS 
    PORT9.PMR.BIT.B2 = 0; // P92 -> CTX2RTS 
    PORT9.PMR.BIT.B3 = 0; // P93 -> CRSET 
    PORT0.PMR.BIT.B7 = 0; // P07 <- CSOF 

    // CAN3 pin setting (via RSPI2) 
    PORTD.PODR.BYTE   = 0x1F; // Port initial value 
    PORT9.PODR.BYTE   = 0x07; // Port initial value 
    PORTD.PDR.BIT.B2  = 0;    // PD2 <- MISOC 
    PORTD.PDR.BIT.B1  = 1;    // PD1 -> MOSIC 
    PORTD.PDR.BIT.B3  = 1;    // PD3 -> RSPCKC 
    PORTD.PDR.BIT.B4  = 1;    // PD4 -> SSLC0 
    PORTD.PDR.BIT.B0  = 0;    // PD0 <- CINT 
    PORTD.PDR.BIT.B6  = 0;    // PD6 <- CRX0BF 
    PORTD.PDR.BIT.B7  = 0;    // PD7 <- CRX1BF 
    PORT9.PDR.BIT.B0  = 1;    // P90 -> CTX0RTS 
    PORT9.PDR.BIT.B1  = 1;    // P91 -> CTX1RTS 
    PORT9.PDR.BIT.B2  = 1;    // P92 -> CTX2RTS 
    PORT9.PDR.BIT.B3  = 1;    // P93 -> CRSET 
    PORT0.PDR.BIT.B7  = 0;    // P07 <- CSOF 
    PORT9.PODR.BIT.B3 = 0;    // P93 -> CRSET 

    MPC.PWPR.BIT.B0WI   = 0;
    MPC.PWPR.BIT.PFSWE  = 1;

    MPC.PD2PFS.BYTE = 0x0D; // MISOC   SO 
    MPC.PD1PFS.BYTE = 0x0D; // MOSIC   SI 
    MPC.PD3PFS.BYTE = 0x0D; // RSPCKC  SCK 
    MPC.PD4PFS.BYTE = 0x0D; // SSLC0  /CS 
    MPC.PD0PFS.BYTE = 0x40; // IRQ0   /INT 
    MPC.PD6PFS.BYTE = 0x40; // IRQ6   /RX0BF 
    MPC.PD7PFS.BYTE = 0x40; // IRQ7   /RX1BF 
    MPC.P90PFS.BYTE = 0x00; // port   /TX0RTS 
    MPC.P91PFS.BYTE = 0x00; // port   /TX1RTS 
    MPC.P92PFS.BYTE = 0x00; // port   /TX2RTS 
    MPC.P93PFS.BYTE = 0x00; // port   /RES 
    MPC.P07PFS.BYTE = 0x00; // port   /CSOF 

    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI  = 1;

    PORTD.PMR.BIT.B2   = 1; // Peripheral function MISOC 
    PORTD.PMR.BIT.B1   = 1; // Peripheral function MOSIC 
    PORTD.PMR.BIT.B3   = 1; // Peripheral function RSPCKC 
    PORTD.PMR.BIT.B4   = 0; // Peripheral function SSLC0 
    PORTD.PMR.BIT.B0   = 1; // Peripheral function CINT 
    PORTD.PMR.BIT.B6   = 1; // PD6 <- CRX0BF 
    PORTD.PMR.BIT.B7   = 1; // PD7 <- CRX1BF 
    PORT9.PMR.BIT.B0   = 0; // P90 -> CTX0RTS 
    PORT9.PMR.BIT.B1   = 0; // P91 -> CTX1RTS 
    PORT9.PMR.BIT.B2   = 0; // P92 -> CTX2RTS 
    PORT9.PMR.BIT.B3   = 0; // P93 -> CRSET 
    PORT0.PMR.BIT.B7   = 0; // P07 <- CSOF 

    SYSTEM.PRCR.WORD = 0xA500;  // Port setting disable 

    // RSPI2 setting (Single master mode) 
    RSPI2.SSLP.BYTE  = 0;    // SSLnP is active Low 
    RSPI2.SPPCR.BYTE = 0x20; // MOSI iddle output is Low 
    RSPI2.SPSR.BYTE &= 0;    // Error flag cancel 
    RSPI2.SPSCR.BYTE = 0;    // Sequence initial value 

    /* Set baud rate to 1Mbps    N value (BRDV[1:0])=0 Fix    Min.=93,750bps
     * n = (PCLK Frequency) / (2 * 2^N * Bit Rate) - 1
     * n = (48,000,000) / (2 * 2^0 * 1,000,000) - 1
     * n = 24*/
    RSPI2.SPBR.BYTE  = 48000000 / (2 * bps) - 1;
    RSPI2.SPDCR.BYTE = 0x00; // SPDR is word access / Receive buffer read / 1 flame 
    RSPI2.SPCKD.BYTE = 1;    // Clock delay 1RSPCK 
    RSPI2.SSLND.BYTE = 0;    // SSL negate delay 1RSPCK 
    RSPI2.SPND.BYTE  = 0;    // Next access delay 1RSPCK + 2PCLK 
    RSPI2.SPCR2.BYTE = 0;    // Parity invalid / Idle interrupt disabled 

    /* Command register initialization
     * RSPCK phase setting bit*/
    RSPI2.SPCMD0.BIT.CPHA = 0; /* 0:Data sample at odd edges, data change at even edges
                                * 1:Data changes at odd edges, data samples at even edges*/
    // RSPCK polarity setting bit 
    RSPI2.SPCMD0.BIT.CPOL = 0; /* 0:RSPCK is Low when idle
                                * 1:RSPCK is High when idle*/
    // Bit rate division setting bit 
    RSPI2.SPCMD0.BIT.BRDV = 0; /* b3 b2
                                * 0 0:Select base bitrate
                                * 0 1:Select base bit rate divided by 2
                                * 1 0:Select base bit rate divided by 4
                                * 1 1:Select base bit rate divided by 8*/
    // SSL signal assertion setting bit 
    RSPI2.SPCMD0.BIT.SSLA = 0; /* b6 b4
                                * 0 0 0:FSSL0
                                * 0 0 1:FSSL1
                                * 0 1 0:FSSL2
                                * 0 1 1:FSSL3
                                * 1 x x:Do not set
                                * x:Don't care*/
    // SSL signal level hold bit 
    RSPI2.SPCMD0.BIT.SSLKP = 0; /* 0:Negate all SSL signals at the end of transfer
                                 * 1:SSL signal level is maintained from end of transfer until start of next access*/
    // RSPI data length setting bit 
    RSPI2.SPCMD0.BIT.SPB = 0xF; /* b11 b8
                                 * 0100 to 0111 :8bit
                                 * 1 0 0 0:9bit
                                 * 1 0 0 1:10bit
                                 * 1 0 1 0:11bit
                                 * 1 0 1 1:12bit
                                 * 1 1 0 0:13bit
                                 * 1 1 0 1:14bit
                                 * 1 1 1 0:15bit
                                 * 1 1 1 1:16bit
                                 * 0 0 0 0:20bit
                                 * 0 0 0 1:24bit
                                 * 0010, 0011 :32bit*/
    // RSPI LSB first bit 
    RSPI2.SPCMD0.BIT.LSBF = 0; /* 0:MSB first
                                * 1:LSB first*/
    // RSPI next access delay enable bit 
    RSPI2.SPCMD0.BIT.SPNDEN = 0; /* 0:Next access delay is 1RSPCK+2PCLK
                                  * 1:Next access delay is the set value of RSPI next access delay register (SPND)*/
    // SSL negate delay setting permission bit 
    RSPI2.SPCMD0.BIT.SLNDEN = 0; /* 0:SSL negate delay is 1RSPCK
                                  * 1:SSL negation delay is the set value of RSPI slave select negate delay register (SSLND)*/
    // RSPCK delay setting permission bit 
    RSPI2.SPCMD0.BIT.SCKDEN = 0; /* 0:RSPCK delay is 1RSPCK
                                  * 1:RSPCK delay is the set value of RSPI clock delay register (SPCKD)*/

    // Copy settings 
    RSPI2.SPCMD1.WORD   = RSPI2.SPCMD0.WORD;
    RSPI2.SPCMD2.WORD   = RSPI2.SPCMD0.WORD;
    RSPI2.SPCMD3.WORD   = RSPI2.SPCMD0.WORD;
    RSPI2.SPCMD4.WORD   = RSPI2.SPCMD0.WORD;
    RSPI2.SPCMD5.WORD   = RSPI2.SPCMD0.WORD;
    RSPI2.SPCMD6.WORD   = RSPI2.SPCMD0.WORD;
    RSPI2.SPCMD7.WORD   = RSPI2.SPCMD0.WORD;

    /* Operation permission
     * RSPI mode selection bit*/
    RSPI2.SPCR.BIT.SPMS = 1;        /* 0:SPI operation (4-wire type)
                                     * 1:Clock synchronous operation (3-wire type)*/
    // Communication operation mode selection bit 
    RSPI2.SPCR.BIT.TXMD = 0;        /* 0:Full-duplex synchronous serial communication
                                     * 1:Serial communication only for transmission operation*/
    // Mode fault error detection permission bit 
    RSPI2.SPCR.BIT.MODFEN = 0;      /* 0:Disable mode fault error detection
                                     * 1:Enable mode fault error detection*/
    // RSPI master/slave mode selection bit 
    RSPI2.SPCR.BIT.MSTR = 1;        /* 0:Slave mode
                                     * 1:Master mode*/
    // RSPI error interrupt permission bit 
    RSPI2.SPCR.BIT.SPEIE = 0;       /* 0:Disable generation of RSPI error interrupt requests
                                     * 1:Enable  generation of RSPI error interrupt requests*/
    // RSPI send interrupt permission bit 
    RSPI2.SPCR.BIT.SPTIE = 0;       /* 0:Disable generation of RSPI transmission interrupt request
                                     * 1:Enable  generation of RSPI transmission interrupt request*/
    // RSPI receive interrupt permission bit 
    RSPI2.SPCR.BIT.SPRIE = 0;       /* 0:Disable generation of RSPI reception interrupt request
                                     * 1:Enable  generation of RSPI reception interrupt request*/
    // RSPI idle interrupt permission bit 
    RSPI2.SPCR2.BIT.SPIIE = 0;      /* 0:Disable generation of idle interrupt requests
                                     * 1:Enable  generation of idle interrupt requests*/
    // RSPI function permission bit 
    RSPI2.SPCR.BIT.SPE = 0;         /* 0:Disable RSPI function
                                     * 1:Enable RSPI function*/

    // IRQ control 
    ICU.IRQCR[0].BIT.IRQMD = 1; // /INT signal 
    ICU.IRQCR[6].BIT.IRQMD = 1; // /RX0BF receive interrupt signal 
    ICU.IRQCR[7].BIT.IRQMD = 1; /* /RX1BF receive interrupt signal
                                 * b3 b2
                                 * 0 0:Low
                                 * 0 1:Falling edge
                                 * 1 0:Rising edge
                                 * 1 1:Both edges*/

    ICU.IPR[IPR_ICU_IRQ0].BIT.IPR = 10; // Interrupt level setting 
    ICU.IPR[IPR_ICU_IRQ6].BIT.IPR = 9;  // Interrupt level setting 
    ICU.IPR[IPR_ICU_IRQ7].BIT.IPR = 9;  // Interrupt level setting 
    ICU.IR[IR_ICU_IRQ0].BIT.IR    = 0;  // Interrupt flag clear 
    ICU.IR[IR_ICU_IRQ6].BIT.IR    = 0;  // Interrupt flag clear 
    ICU.IR[IR_ICU_IRQ7].BIT.IR    = 0;  // Interrupt flag clear 


    // Interrupt priority setting 
    ICU.IPR[IPR_RSPI2_].BIT.IPR = 8; // Interrupt level setting 
    // Interrupt flag clear 
    ICU.IR[IR_RSPI2_SPRI2].BIT.IR = 0;
    ICU.IR[IR_RSPI2_SPTI2].BIT.IR = 0;
    ICU.IR[IR_RSPI2_SPII2].BIT.IR = 0;
    // GROUP12 interrupt setting 
    ICU.GEN[GEN_RSPI2_SPEI2].BIT.EN_RSPI2_SPEI2 = 1;  // Group12 RSPI1 receive error interrupt enable 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR            = 2;  // Group12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR               = 0;  // Group12 interrupt flag clear 
    // Interrupt permission setting 
    ICU.IER[IER_RSPI2_SPRI2].BIT.IEN_RSPI2_SPRI2 = 1; // Enable receive buffer full interrupt 
    ICU.IER[IER_RSPI2_SPTI2].BIT.IEN_RSPI2_SPTI2 = 1; // Enable transmit empty interrupt 
    ICU.IER[IER_RSPI2_SPII2].BIT.IEN_RSPI2_SPII2 = 0; // Enable transmit idle interrupt 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Enable group12 interrupt 
    ICU.IER[IER_ICU_IRQ0].BIT.IEN_ICU_IRQ0       = 0; // Module interrupt enable 
    ICU.IER[IER_ICU_IRQ6].BIT.IEN_ICU_IRQ6       = 0; // Module interrupt enable 
    ICU.IER[IER_ICU_IRQ7].BIT.IEN_ICU_IRQ7       = 0; // Module interrupt enable 

    dtc_init(); // DTC initialization 
}

/* ----------------------------------------------------------------------------------------
 * can3_init
 * 
 * Function description
 *     MCP2515(via RSPI2) initialization
 *                 Port        SCI            I2C            SPI            Application
 *         ----------------------------------------------------------------------------
 *         RSPI2   PD2                                    MISOC        <RSPI>        CAN3
 *                 PD1                                    MOSIC        <RSPI>        CAN3
 *                 PD3                                    RSPCKC       <RSPI>        CAN3
 *                 PD4                                    SSLC0        <RSPI>        CAN3
 *                 PD0                                    IRQ0       <- CINT         CAN3
 *                 PD6                                    IRQ6       <- CRX0BF       CAN3
 *                 PD7                                    IRQ7       <- CRX1BF       CAN3
 *                 P90                                               -> CTX0RTS      CAN3
 *                 P91                                               -> CTX1RTS      CAN3
 *                 P92                                               -> CTX2RTS      CAN3
 *                 P93                                               -> CRSET        CAN3
 *                 P07                                               <- CSOF         CAN3
 * 
 * Argument
 *     None
 * 
 * Return
 *     None
 * ----------------------------------------------------------------------------------------
 */
void can3_init(void)
{
    rspi2_init(8000000);    // Initialize at 8Mbps 

    rspi_req = (RSPI_REQUESTS *)DTC_REQUEST_TOP;
    memset(rspi_req, 0, sizeof(RSPI_REQUESTS));

    can3_now = 0; // No current processing 

    CAN3_TX0RTS_PORT = 1; // TXB0 transmit request disable 
    CAN3_TX1RTS_PORT = 1; // TXB1 transmit request disable 
    CAN3_TX2RTS_PORT = 1; // TXB2 transmit request disable 

    // RSPI error interrupt permission bit 
    RSPI2.SPCR.BIT.SPEIE = 0; /* 0:Disable generation of RSPI error interrupt requests
                               * 1:Enable  generation of RSPI error interrupt requests*/
    // RSPI transmit interrupt permission bit 
    RSPI2.SPCR.BIT.SPTIE = 1; /* 0:Disable generation of RSPI transmission interrupt request
                               * 1:Enable  generation of RSPI transmission interrupt request*/
    // RSPI receive interrupt permission bit 
    RSPI2.SPCR.BIT.SPRIE = 1; /* 0:Disable generation of RSPI reception interrupt request
                               * 1:Enable  generation of RSPI reception interrupt request*/
    // RSPI idle interrupt permission bit 
    RSPI2.SPCR2.BIT.SPIIE = 0; /* 0:Disable generation of idle interrupt requests
                                * 1:Enable  generation of idle interrupt requests*/
    RSPI2.SPCR.BIT.SPE  = 0;   // 0:Disable RSPI function 
    DTC.DTCST.BIT.DTCST = 0;   // Stop module 

    ICU.IR[IR_ICU_IRQ0].BIT.IR             = 0; // Interrupt flag clear 
    ICU.IR[IR_ICU_IRQ6].BIT.IR             = 0; // Interrupt flag clear 
    ICU.IR[IR_ICU_IRQ7].BIT.IR             = 0; // Interrupt flag clear 
    ICU.IER[IER_ICU_IRQ0].BIT.IEN_ICU_IRQ0 = 0; // Module interrupt enable 
    ICU.IER[IER_ICU_IRQ6].BIT.IEN_ICU_IRQ6 = 0; // Module interrupt enable 
    ICU.IER[IER_ICU_IRQ7].BIT.IEN_ICU_IRQ7 = 0; // Module interrupt enable 
}
int  can3_txcheck(void);
void can3_send_nextbyte(void);
void can3_stat_event(MCP2515REG_STATUS *sts);

// Status control register status 
MCP2515REG_STATCTRL mcp_statctrl;
// Port setting 
MCP2515REG_BFPRTS_CTRL mcp_bfprts;
// Configuration 
MCP2515REG_CONFIG mcp_config;
// Transmit buffer 
MCP2515REG_TXBUF mcp_txb[3];
// Receive buffer 
MCP2515REG_RXBUF mcp_rxb[2];
// Change bit 
MCP2515REG_BITX mcp_bitx;

// Receive buffer 
CAN_MBOX mcp_mbx[16];
int      mcp_mbx_wp = 0;
int      mcp_mbx_rp = 0;

/* ----------------------------------------------------------------------------------------
 * MCP2515 status control read processing function
 * ---------------------------------------------------------------------------------------- */
void can3_getstat_callback(MCP2515REG_STATCTRL *rxd)
{
    mcp_statctrl = *rxd;
}

/* ---------------------------------------------------------------------------------------
 * Waiting message processing
 * --------------------------------------------------------------------------------------- */
void can3_recv_call(void)
{
    int i;
    if (mcp_mbx_wp != mcp_mbx_rp) {
        i = mcp_mbx_rp++;
        mcp_mbx_rp  &= 15;
        can_recv_frame(3, (void *)&mcp_mbx[i]); // Receive data processing 
    }
}
void can3_procwait(void)
{
    RSPI_DTC_REQ *w;
    int i;
    unsigned short sw;

    if (rspi_req_RP != rspi_req_PP) {
        w = &rspi_req->REQ[rspi_req_PP];
        if (w->TXL >= 0) {
            return; // Incomplete 
        }
        if (w->TXL == -2) {
            return; // Force sending 
        }
        rspi_req_PP++;
        if (rspi_req_PP >= CAN3_REQUEST_DTC_MAX) {
            rspi_req_PP = 0;
        }
        if (w->TXL == -1) { // Transmit completed 
            if (w->RXL > 0) { // Swap received data 
                for (i = 0; i < w->RXL; i++) {
                    sw = w->RXP[i];
                    w->RXP[i] = ((sw >> 8) & 0x00FF) | ((sw << 8) & 0xFF00);
                }
            }
            if (w->CALL) { // User processing function call 
                ((CAN3_PROC_CALL)w->CALL)((void *)w->RXP);
            }
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * can3_job
 * 
 * Function description
 *     MCP2515 startup initialization and wait processing
 * 
 * Argument
 *     None
 * 
 * Return
 *     None
 * ----------------------------------------------------------------------------------------
 */
int can3_job(void)
{
    can3_txcheck();
    can3_procwait();
    can3_recv_call();

    switch (can3_job_id) {
    case CAN3_JOB_INIT: // Device initialization 
        can3_job_id++;
        memset(&mcp_mbx, 0, sizeof(mcp_mbx));
        can3_request(
            MCP2515CMD_READ,
            MCP2515AD_CANSTAT, 0,
            1, can3_getstat_callback, 0
        ); // Get status control 
        break;
    case CAN3_JOB_IW1:
        if (can3_now == 0) {
            can3_job_id++;
            CAN3_RESET_PORT = 1; // Release reset
        }
        break;
    case CAN3_JOB_IW2:
        can3_job_id++;
        can3_request(
            MCP2515CMD_READ, 
            MCP2515AD_CANSTAT, 0,
            1, can3_getstat_callback, 0
        ); // Get status control 
        break;
    case CAN3_JOB_IW3:
        if (can3_now == 0) {
            if (
                mcp_statctrl.BYTE.STAT.BIT.OPMOD == 4  &&
               (mcp_statctrl.BYTE.CTRL.BIT.REQOP == 15 ||
                mcp_statctrl.BYTE.CTRL.BIT.REQOP == 4)
            ) { // Match the state immediately after reset 
                can3_job_id++;
            } else {
                can3_job_id = CAN3_JOB_IW2; // Re-acquire 
            }
        }
        break;
    case CAN3_JOB_IW4:
        can3_job_id++;
        mcp_bitx.BYTE.MSK.BYTE  = 0xE0;
        mcp_bitx.BYTE.PAT.BYTE  = 0x80;
        can3_request(
            MCP2515CMD_BITX,
            MCP2515AD_CANCTRL, 1,
            0, 0, &mcp_bitx
        ); // Change mode 
        can3_request(
            MCP2515CMD_READ,
            MCP2515AD_CANSTAT, 0,
            1, can3_getstat_callback, 0
        ); // Get status and control 
        break;
    case CAN3_JOB_IW5:
        if (can3_now == 0) {
            if (
                mcp_statctrl.BYTE.STAT.BIT.OPMOD == 4 &&
                mcp_statctrl.BYTE.CTRL.BIT.REQOP != 4
            ) { // Waiting for change to config 
                can3_job_id = CAN3_JOB_IW4;
            } else { // Start config mode 
                can3_job_id++;
            }
        }
        break;
    case CAN3_JOB_IW6:  // Parameter setting 
        can3_job_id++;
        memset(&mcp_config, 0, sizeof(mcp_config));
        memset(&mcp_bfprts, 0, sizeof(mcp_bfprts));
        // I/O port function setting 
        mcp_bfprts.BYTE.BFP.BYTE = 0x0F; // RXnBF = Receive buffer full interrupt output 
        mcp_bfprts.BYTE.RTS.BYTE = 0x07; // TXnRTS = TXBnREQ transmit request input 
        can3_request(MCP2515CMD_WRITE, MCP2515AD_BFPCTRL, 1, 0, 0, &mcp_bfprts); // Port setting 

        /* Baud rate setting (Fosc=20MHz : Tosc=50ns(1s/20M) : CAN=500Kbps)
         *            _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
         * tosc    |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|
         *            _0_     _1_     _2_     _3_     _4_     _5_     _6_     _7_     _8_     _9_     _0_     _1_     _
         * TBRPCLK __|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|
         *            _______ _______________________________________________________________________
         * tBIT    __|__Sync_|_____PropSeg___|__________PS1__________|______________PS2______________|_________________
         *            _______________________________________________|_________________________________________________
         * tTQ     __|_______|_______|_______|_______|_______|_______|_______|_______|_______|_______|_______|_______|_
         *           |                                               |                               |
         * CANBIT  ->|<----------------------------------------------A------------------------------>|<----------------
         *                                                        (Sample)
         * CAN1Bit: When 10TQ, TQ=1/500K/10=200ns : BRP=TQ/(2*Tosc)-1=200ns/100ns-1=2-1 therefore BRP=1 : CANBIT=10TQ
         * Condition : PropSeg + PS1 >= PS2 : PropSeg + PS1 >= TDELAY : PS2 > SJW
         * SyncSeg(1) + PropSeg(2) + PS1(3) + PS2(4) = 10TQ : SJW = PS2 - 2 = 2TQ : TDELAY = 2TQ*/
        mcp_config.BYTE.CNF1.BIT.BRP     = 2-1; // BRP = TQ / (2 * Tosc)    Tosc=50ns(20MHz)        TQ=    200ns 
        mcp_config.BYTE.CNF1.BIT.SJW     = 3-1; // SJW = (SJW+1)*TQ         Resynchronization jump width 
        mcp_config.BYTE.CNF2.BIT.PHSEG   = 2-1; // PropSeg = (PHSEG+1)*TQ   Propagation segment            400ns 
        mcp_config.BYTE.CNF2.BIT.PHSEG1  = 3-1; // PS1 = (PHSEG1+1)*TQ      Phase segment1                 600ns 
        mcp_config.BYTE.CNF2.BIT.SAM     = 0;   // 0=1 sample (1=3 samples) 
        mcp_config.BYTE.CNF2.BIT.BTLMODE = 1;   // Determined by PHSEG2 
        mcp_config.BYTE.CNF3.BIT.PHSEG2  = 4-1; // PS2 = (PHSEG2+1)*TQ      Phase segment2                 800ns 
        mcp_config.BYTE.CNF3.BIT.WAKFIL  = 0;   // Wake up filter invalid 
        mcp_config.BYTE.CNF3.BIT.SOF     = 0;   // SOF output 

        // (/INT Output)Interrupt enable setting 
        mcp_config.BYTE.CANINTE.BYTE = 0x1F; // MERRE / WAKIE disable : ERRIE / TX2IE / TX1IE / TX0IE / RX1IE / RX0IE enable 
        can3_request(
            MCP2515CMD_WRITE, 
            MCP2515AD_CNFIG3, 2,
            0, 0, &mcp_config
        ); // Baud rate setting + Interrupt enable 

        // Send buffer setting 
        memset(&mcp_txb[0], 0, 14);
        mcp_txb[0].REG.TXB.CTRL.BYTE = 0; // Priority "High" 
        can3_request(
            MCP2515CMD_WRITE, 
            MCP2515AD_TXB0CTRL, 7, 
            0, 0, &mcp_txb[0]
        ); // Save TXB0 setting 
        memset(&mcp_txb[1], 0, 14);
        mcp_txb[1].REG.TXB.CTRL.BYTE = 0; // Priority "Middle" 
        can3_request(
            MCP2515CMD_WRITE,
            MCP2515AD_TXB1CTRL, 7,
            0, 0, &mcp_txb[1]
        ); // Save TXB1 setting 
        memset(&mcp_txb[2], 0, 14);
        mcp_txb[2].REG.TXB.CTRL.BYTE = 0; // Priority "Low" 
        can3_request(
            MCP2515CMD_WRITE,
            MCP2515AD_TXB2CTRL, 7,
            0, 0, &mcp_txb[2]
        ); // Save TXB2 setting 

        // Receive buffer setting 
        memset(&mcp_rxb[0], 0, 14);
        mcp_rxb[0].REG.RXB.CTRL.BYTE = 0x64; // Receive all message & switch enable bit 
        can3_request(
            MCP2515CMD_WRITE,
            MCP2515AD_RXB0CTRL, 7,
            0, 0, &mcp_rxb[0]
        ); // RXB0 control 
        memset(&mcp_rxb[1], 0, 14);
        mcp_rxb[1].REG.RXB.CTRL.BYTE = 0x60; // Receive all message 
        can3_request(
            MCP2515CMD_WRITE,
            MCP2515AD_RXB1CTRL, 7,
            0, 0, &mcp_rxb[1]
        ); // RXB1 control 
        break;
    case CAN3_JOB_IW7:  // Wait for parameter setting completion 
        if (can3_now == 0) {
            can3_job_id++;
        }
        break;
    case CAN3_JOB_IW8:  // CAN start 
        can3_job_id++;
        mcp_bitx.BYTE.MSK.BYTE  = 0xE0; // Select mode 
        mcp_bitx.BYTE.PAT.BYTE  = 0x00; // Normal mode(0) 
        can3_request(
            MCP2515CMD_BITX,
            MCP2515AD_CANCTRL, 1,
            0, 0, &mcp_bitx
        ); // Change mode 
        can3_request(
            MCP2515CMD_READ,
            MCP2515AD_CANSTAT, 0,
            1, can3_getstat_callback, 0
        ); // Get status 
        break;
    case CAN3_JOB_IW9:  // Wait for parameter setting completion 
        if (can3_now == 0) {
            if (mcp_statctrl.BYTE.STAT.BIT.OPMOD != 0) { // Wait for change to config 
                can3_job_id = CAN3_JOB_IW8;     // Reset 
            } else { // Setting complete 
                can3_job_id++;
            }
        }
        break;

    case CAN3_JOB_WAIT: // Waiting for operation 
        if (rspi_req_RP == rspi_req_WP) {
            can3_job_id     = CAN3_JOB_WW1;
            stat_event_flag = 100;
            can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0);   // Get status 
        }
        break;
    case CAN3_JOB_WW1:  // Waiting for status acquisition 
        if (rspi_req_RP == rspi_req_WP) { // Transmitted 
            if (stat_event_flag == 0) {   // Receive complete 
                can3_job_id = CAN3_JOB_WAIT;
            } else { // Timeout detection 
                stat_event_flag--;
            }
        }
        break;
    }
    return ((can3_job_id >= CAN3_JOB_WAIT) ? 1 : 0); // Return 0 during initialization 
}

/* ----------------------------------------------------------------------------------------
 * Transmission processing
 * ----------------------------------------------------------------------------------------
 */
int can3_txcheck(void)
{
    for (; can3_now == 0 && rspi_req_RP != rspi_req_WP;) { // Waiting 
        can3_now = &rspi_req->REQ[rspi_req_RP++];
        if (rspi_req_RP >= CAN3_REQUEST_DTC_MAX) {
            rspi_req_RP = 0;
        }
        if (can3_now->TXL < 0) { // Transmitted, Pass 
            can3_now = 0;
            continue;
        }
        CAN3_SPI_CS_PORT                        = 1; // CS permission 
        DTC.DTCST.BIT.DTCST                     = 0; // Stop module 
        RSPI2.SPCR.BIT.SPE                      = 0; // 0:RSPI function invalid 
        RSPI2.SPCR.BIT.SPRIE                    = 0; // 1:Enable generation of RSPI reception interrupt request 
        RSPI2.SPCR.BIT.SPTIE                    = 0; // 1:Enable generation of RSPI transmission interrupt request 
        ICU.IR[IR_RSPI2_SPRI2].BIT.IR           = 0; // Interrupt flag area 
        ICU.IR[IR_RSPI2_SPTI2].BIT.IR           = 0; // Interrupt flag area 
        dtc_table[DTCE_RSPI2_SPTI2]             = (unsigned long)&can3_now->DTCTX.LONG[0];
        dtc_table[DTCE_RSPI2_SPRI2]             = (unsigned long)&can3_now->DTCRX.LONG[0];
        ICU.DTCER[ DTCE_RSPI2_SPRI2 ].BIT.DTCE  = 1; // Enable DTC activation by RSPI reception interrupt 
        ICU.DTCER[ DTCE_RSPI2_SPTI2 ].BIT.DTCE  = 1; // Enable DTC activation by RSPI transmission interrupt 
        RSPI2.SPCR.BIT.SPRIE                    = 1; // 1:Enable generation of RSPI reception interrupt request 
        RSPI2.SPCR.BIT.SPTIE                    = 1; // 1:Enable generation of RSPI transmission interrupt request 
        ICU.IR[IR_RSPI2_SPRI2].BIT.IR           = 0; // Interrupt flag area 
        ICU.IR[IR_RSPI2_SPTI2].BIT.IR           = 0; // Interrupt flag area 
        DTC.DTCST.BIT.DTCST                     = 1; // DTC permission 
        CAN3_SPI_CS_PORT                        = 0; // CS permission 
        RSPI2.SPCR.BIT.SPE                      = 1; // SPI permission 
        return 1;   // In process 
    }
    return 0;   // Not in process 
}

/* ---------------------------------------------------------------------------------------
 * SPRI2 receive buffer full       Data arrives at SPI receive buffer
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_RSPI2_SPRI2} RSPI2_SPRI2_ISR(void)
{
    // CS disable 
    CAN3_SPI_CS_PORT     = 1;
    RSPI2.SPCR.BIT.SPRIE = 0;  // 0:Disable generation of RSPI reception interrupt request 
    RSPI2.SPCR.BIT.SPE   = 0;  // RSPI2 disable 
    DTC.DTCST.BIT.DTCST  = 0;  // DTC disable 
    can3_now->TXL        = -1; // End mark 
    can3_now             = 0;
}

/* ---------------------------------------------------------------------------------------
 * SPTI2 transmit buffer empty     SPI transmit buffer free
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_RSPI2_SPTI2} RSPI2_SPTI2_ISR(void)
{
    RSPI2.SPCR.BIT.SPTIE = 0; // 0:Disable generation of RSPI transmission interrupt request 
}

/* ---------------------------------------------------------------------------------------
 * SPII2 Idle      Interrupt occurs when final writing of transmission data is completed (transmission completed)
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_RSPI2_SPII2} RSPI2_SPII2_ISR(void)
{
    RSPI2.SPCR2.BIT.SPIIE = 0; // 0:Disable generation of idle interrupt request 
    logging("IDLE\r");
}

/* ----------------------------------------------------------------------------------------
 * can3_request
 *
 * Function description
 *     Stack of SPI communication request
 * 
 * Argument
 *     int cmd        Command code
 *     int adr        Forwarding address
 *     int txlen      Number of transmitted bytes
 *     int rxlen      Number of received bytes
 *     void *proc     Callee function on completion
 *     void *data     Transmit data pointer
 * 
 * Return
 *     None
 * ----------------------------------------------------------------------------------------
 */
RSPI_DTC_REQ * can3_request(int cmd, int adr, int txlen, int rxlen, void *proc, void *data)
{
    int             i, j;
    RSPI_DTC_REQ *  act;
    unsigned short *dp, sw;

    // Get buffer 
    act = &rspi_req->REQ[rspi_req_WP++];
    if (rspi_req_WP >= CAN3_REQUEST_DTC_MAX) {
        rspi_req_WP = 0;
    }
    dp  = (unsigned short *)data;
    i   = 0; // Transmitted data pointer 
    // Command registration 
    act->TXL        = txlen; // Number of transmitted bytes  
    act->RXL        = rxlen; // Number of received bytes 
    act->DAT[i++]   = (unsigned short)((cmd << 8) | adr); // Command code 
    // Copy transmitted data 
    for (j = 0; j < txlen; j++) {
        sw              = dp[j];
        act->DAT[i++]   = ((sw >> 8) & 0x00FF) | ((sw << 8) & 0xFF00);
    }
    act->RXP = &act->DAT[i];
    // Received data dummy 
    for (; j < rxlen; j++) {
        act->DAT[i++] = 0;
    }
    // Register call destination upon completion 
    act->CALL = proc;
    /* DTC request information setting
     * Transmit*/
    act->DTCTX.REG.MR.LONG  = 0x18000000;
    act->DTCTX.REG.SAR      = (unsigned long)&act->DAT[0]; // Transfer source: Transmission data buffer 
    act->DTCTX.REG.DAR      = (unsigned long)&RSPI2.SPDR;  // Transfer destination: SPI data register 
    act->DTCTX.REG.CR.NOR.A = i;
    act->DTCTX.REG.CR.NOR.B = i;
    // Receive 
    act->DTCRX.REG.MR.LONG  = 0x10080000;
    act->DTCRX.REG.SAR      = (unsigned long)&RSPI2.SPDR;  // Transfer source: SPI data register 
    act->DTCRX.REG.DAR      = (unsigned long)&act->DAT[0]; // Transfer destination: Receive data buffer 
    act->DTCRX.REG.CR.NOR.A = i;
    act->DTCRX.REG.CR.NOR.B = i;
    return act;
}

void    CAN3_GetRx0(void);
void    CAN3_GetRx1(void);
/* ---------------------------------------------------------------------------------------
 * Received data callback
 * --------------------------------------------------------------------------------------- */
void CAN3_CallbackRx0(MCP2515REG_RXBUF *rxd)
{
    CAN_MBOX *mbx;
    mbx             = &mcp_mbx[mcp_mbx_wp++];
    mcp_mbx_wp     &= 15;
    mbx->ID.BIT.RTR = rxd->REG.RXB.SIDL.BIT.RTR;
    mbx->ID.BIT.SID = 
        ((((unsigned long)rxd->REG.RXB.SIDH.BYTE) << 3) & 0x7F8) |
        ((((unsigned long)rxd->REG.RXB.SIDL.BYTE) >> 5) & 0x007);
    mbx->DLC        = rxd->REG.RXB.DLC.BIT.DLC;
    memcpy(mbx->DATA, rxd->REG.RXB.DATA, 8);

    if (mbx->ID.BIT.SID == led_monit_id && (led_monit_ch & 0x80) != 0) {
        led_monit_ch &= 0x8F;
        PORTE.PODR.BIT.B0 = 0;
        cmt1_start(1000000, monit_timeover);
    }
}
void CAN3_CallbackRx1(MCP2515REG_RXBUF *rxd)
{
    CAN_MBOX *mbx;
    mbx         = &mcp_mbx[mcp_mbx_wp++];
    mcp_mbx_wp  &= 15;

    mbx->ID.BIT.RTR = rxd->REG.RXB.SIDL.BIT.RTR;
    mbx->ID.BIT.SID = 
        ((((unsigned long)rxd->REG.RXB.SIDH.BYTE) << 3) & 0x7F8) |
        ((((unsigned long)rxd->REG.RXB.SIDL.BYTE) >> 5) & 0x007);
    mbx->DLC        = rxd->REG.RXB.DLC.BIT.DLC;
    memcpy(mbx->DATA, rxd->REG.RXB.DATA, 8);

    if (mbx->ID.BIT.SID == led_monit_id && (led_monit_ch & 0x80) != 0) {
        led_monit_ch     &= 0x8F;
        PORTE.PODR.BIT.B0 = 0;
        cmt1_start(1000000, monit_timeover);
    }
}

/* ---------------------------------------------------------------------------------------
 * Send mailbox write callback (transmission start request)
 * --------------------------------------------------------------------------------------- */
void CAN3_CallbackTxSet0(MCP2515REG_TXBUF *rxd)  
{
    if (memcmp(&mcp_txb[0].BYTE[1], &rxd->BYTE[1], 13) == 0) {
        CAN3_TX0RTS_PORT = 0; // TX0 transmit request 
        tx_act[0]        = 2;
        CAN3_TX0RTS_PORT = 1; // TX0 transmit request 
    } else {
        logging("CAN3 TX0-Readback Error\r");
        can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB0CTRL, 7, 0, 0, &mcp_txb[0]);
        can3_request(MCP2515CMD_READ , MCP2515AD_TXB0CTRL, 0, 7, CAN3_CallbackTxSet0, 0);
    }
}
void CAN3_CallbackTxSet1(MCP2515REG_TXBUF *rxd)  
{
    if (memcmp(&mcp_txb[1].BYTE[1], &rxd->BYTE[1], 13) == 0) {
        CAN3_TX1RTS_PORT = 0; // TX1 transmit request 
        tx_act[1]        = 2;
        CAN3_TX1RTS_PORT = 1; // TX1 transmit request 
    } else {
        logging("CAN3 TX1-Readback Error\r");
        can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB1CTRL, 7, 0, 0, &mcp_txb[1]);
        can3_request(MCP2515CMD_READ , MCP2515AD_TXB1CTRL, 0, 7, CAN3_CallbackTxSet1, 0);
    }
}
void CAN3_CallbackTxSet2(MCP2515REG_TXBUF *rxd)  
{
    if (memcmp(&mcp_txb[2].BYTE[1], &rxd->BYTE[1], 13) == 0) {
        CAN3_TX2RTS_PORT = 0; // TX2 transmit request 
        tx_act[2]        = 2;
        CAN3_TX2RTS_PORT = 1; // TX2 transmit request 
    } else {
        logging("CAN3 TX2-Readback Error\r");
        can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB2CTRL, 7, 0, 0, &mcp_txb[2]);
        can3_request(MCP2515CMD_READ , MCP2515AD_TXB2CTRL, 0, 7, CAN3_CallbackTxSet2, 0);
    }
}

/* ---------------------------------------------------------------------------------------
 * Get empty mailbox status
 * --------------------------------------------------------------------------------------- */
int CAN3_GetTxMCTL(int mb)
{
#ifdef  MB_USED_ONLYONE
    if (tx_act[0] == 0) {
        return 0;
    }
    // Search for timeout MB 
    if (tx_act_timer[0] > 0) {
        tx_act_timer[0]--;
    }
    if (tx_act_timer[0] == 0) {
        tx_act[0] = 0;
        logging("CAN3 TXMB0:Timeout\r");
        return 0;   // Timeout free 
    }
#else // ifdef  MB_USED_ONLYONE
    int rc;
    rc = rspi_req_WP - rspi_req_RP;
    if (rc < 0) {
        rc += CAN3_REQUEST_DTC_MAX;
    }
    if (rc > (CAN3_REQUEST_DTC_MAX - 8)) { // Buffer danger 
        return 0xC0; // Insufficient buffer space 
    }

#ifdef  MB_LOCKED_TYPE
    if (tx_act[mb] == 0) {
        return 0;
    }
    // Search for timeout MB 
    if (tx_act_timer[mb] > 0) {
        tx_act_timer[mb]--;
    }
    if (tx_act_timer[mb] == 0) {
        tx_act[mb] = 0;
        return mb; // Timeout free 
    }
#else // ifdef  MB_LOCKED_TYPE
    if (tx_act[0] == 0) {
        return 0;
    }
    if (tx_act[1] == 0) {
        return 1;
    }
    if (tx_act[2] == 0) {
        return 2;
    }

    // Search for timeout MB 
    if (tx_act_timer[0] > 0) {
        tx_act_timer[0]--;
    }
    if (tx_act_timer[0] == 0) {
        tx_act[0] = 0;
        return 0; // Timeout free 
    }
    if (tx_act_timer[1] > 0) {
        tx_act_timer[1]--;
    }
    if (tx_act_timer[1] == 0) {
        tx_act[1] = 0;
        return 1; // Timeout free 
    }
    if (tx_act_timer[2] > 0) {
        tx_act_timer[2]--;
    }
    if (tx_act_timer[2] == 0) {
        tx_act[2] = 0;
        return 2; // Timeout free 
    }
#endif // ifdef  MB_LOCKED_TYPE
#endif // ifdef  MB_USED_ONLYONE
    return 0xC0; // No space 
}

/* ---------------------------------------------------------------------------------------
 * Write outgoing mailbox
 * --------------------------------------------------------------------------------------- */
int CAN3_TxSet(int mb, SEND_WAIT_FLAME *act)
{
    unsigned char ctrl = 3 - mb;
    // Check for request 
#ifdef  MB_LOCKED_TYPE
    if (tx_act[mb] != 0) {
        return -1; // No space 
    }
#else
    mb = 0;
    if (tx_act[0] != 0) {
        mb++;
        if (tx_act[1] != 0) {
            mb++;
            if (tx_act[2] != 0) {
                return -1;  // No space 
            }
        }
    }
#endif // ifdef  MB_LOCKED_TYPE
    tx_act[mb]          = 1;
    tx_act_timer[mb]    = 1000;
    // Buffer transfer 
    mcp_txb[mb].REG.TXB.CTRL.BYTE   = ctrl; // Priority 
    mcp_txb[mb].REG.TXB.SIDH.BYTE   = (unsigned char)(act->ID.BIT.SID >> 3);
    mcp_txb[mb].REG.TXB.SIDL.BYTE   = (act->ID.BIT.SID << 5) & 0xE0;
    mcp_txb[mb].REG.TXB.EID8.BYTE   = 0;
    mcp_txb[mb].REG.TXB.EID0.BYTE   = 0;
    mcp_txb[mb].REG.TXB.DLC.BIT.RTR = act->ID.BIT.RTR;
    mcp_txb[mb].REG.TXB.DLC.BIT.DLC = act->ID.BIT.DLC;
    memcpy(mcp_txb[mb].REG.TXB.DATA, act->FD.BYTE, 8);
    // SPI transmission processing 
    switch (mb) {
    case 0:
        can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB0CTRL, 7, 0, 0, &mcp_txb[mb]);
        can3_request(MCP2515CMD_READ , MCP2515AD_TXB0CTRL, 0, 7, CAN3_CallbackTxSet0, 0);
        break;
    case 1:
        can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB1CTRL, 7, 0, 0, &mcp_txb[mb]);
        can3_request(MCP2515CMD_READ , MCP2515AD_TXB1CTRL, 0, 7, CAN3_CallbackTxSet1, 0);
        break;
    case 2:
        can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB2CTRL, 7, 0, 0, &mcp_txb[mb]);
        can3_request(MCP2515CMD_READ , MCP2515AD_TXB2CTRL, 0, 7, CAN3_CallbackTxSet2, 0);
        break;
    }
    return 0;
}


/* ---------------------------------------------------------------------------------------
 * Callback processing after acquiring high-speed status
 * --------------------------------------------------------------------------------------- */
void can3_stat_event(MCP2515REG_STATUS *sts)
{
    int id;
    unsigned short buf = 0x0000;

    stat_event_flag = 0;

    if (sts->BIT.RX0IF != 0) { // Receive buffer 0 full 
        can3_request(MCP2515CMD_READ, MCP2515AD_RXB0CTRL, 0, 7, CAN3_CallbackRx0, 0);   // Read RXB0 
        buf |= 0x01;
    }
    if (sts->BIT.RX1IF != 0) { // Receive buffer 1 full 
        can3_request(MCP2515CMD_READ, MCP2515AD_RXB1CTRL, 0, 7, CAN3_CallbackRx1, 0);   // Read RXB1 
        buf |= 0x02;
    }

    // TXB0 
    if (sts->BIT.TXB0R != 0) {
        if (tx_act[0] == 2) {
            tx_act[0] = 3;
        }
    } else {
        if (tx_act[0] == 3) {
            tx_act[0] = 0;
            id = 
                ((((unsigned long)mcp_txb[0].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) |
                ((((unsigned long)mcp_txb[0].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
            if (led_monit_ch & 8) {
                if (id == led_monit_id && (led_monit_ch & 0x08) != 0) {
                    led_monit_ch &= 0xF8;
                    PORTE.PODR.BIT.B0 = 1;
                    monit_timeover();
                }
            }
            can_tp_txecheck(3, id); // Confirm TP transmission completion 
        }
    }
    if (sts->BIT.TX0IF != 0) { // Ending 
        if (sts->BIT.TXB0R == 0 && tx_act[0] == 2) {
            tx_act[0] = 0;
            id = 
                ((((unsigned long)mcp_txb[0].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) |
                ((((unsigned long)mcp_txb[0].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
            if (led_monit_ch & 8) {
                if (id == led_monit_id && (led_monit_ch & 0x08) != 0) {
                    led_monit_ch &= 0xF8;
                    PORTE.PODR.BIT.B0 = 1;
                    monit_timeover();
                }
            }
            can_tp_txecheck(3, id); // Confirm TP transmission completion 
        }
        buf |= 0x04;
    }

    // TXB1
    if (sts->BIT.TXB1R != 0) {
        if (tx_act[1] == 2)
            tx_act[1] = 3;
    } else {
        if (tx_act[1] == 3) {
            tx_act[1] = 0;
            id = ((((unsigned long)mcp_txb[1].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) |
                 ((((unsigned long)mcp_txb[1].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
            if (led_monit_ch & 8) {
                if (id == led_monit_id && (led_monit_ch & 0x08) != 0) {
                    led_monit_ch &= 0xF8;
                    PORTE.PODR.BIT.B0 = 1;
                    monit_timeover();
                }
            }
            can_tp_txecheck(3, id); // Confirm TP transmission completion
        }
    }
    if (sts->BIT.TX1IF != 0) { // Ending
        if (sts->BIT.TXB1R == 0 && tx_act[1] == 2) {
            tx_act[1] = 0;
            id = ((((unsigned long)mcp_txb[1].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) |
                 ((((unsigned long)mcp_txb[1].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
            if (led_monit_ch & 8) {
                if (id == led_monit_id && (led_monit_ch & 0x08) != 0) {
                    led_monit_ch &= 0xF8;
                    PORTE.PODR.BIT.B0 = 1;
                    monit_timeover();
                }
            }
            can_tp_txecheck(3, id); // Confirm TP transmission completion
        }
        buf |= 0x08;
    }

    // TXB2
    if (sts->BIT.TXB2R != 0) {
        if (tx_act[2] == 2)
            tx_act[2] = 3;
    } else {
        if (tx_act[2] == 3) {
            tx_act[2] = 0;
            id = ((((unsigned long)mcp_txb[2].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) |
                 ((((unsigned long)mcp_txb[2].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
            if (led_monit_ch & 8) {
                if (id == led_monit_id && (led_monit_ch & 0x08) != 0) {
                    led_monit_ch &= 0xF8;
                    PORTE.PODR.BIT.B0 = 1;
                    monit_timeover();
                }
            }
            can_tp_txecheck(3, id); // Confirm TP transmission completion
        }
    }
    if (sts->BIT.TX2IF != 0) { // Ending
        if (sts->BIT.TXB2R == 0 && tx_act[2] == 2) {
            tx_act[2] = 0;
            id = ((((unsigned long)mcp_txb[2].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) |
                 ((((unsigned long)mcp_txb[2].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
            if (led_monit_ch & 8) {
                // if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
                if (id == led_monit_id && (led_monit_ch & 0x08) != 0) {
                    led_monit_ch &= 0xF8;
                    PORTE.PODR.BIT.B0 = 1;
                    monit_timeover();
                }
            }
            can_tp_txecheck(3, id); // Confirm TP transmission completion
        }
        buf |= 0x10;
    }

    // Flag clear 
    if (buf != 0) {
        buf |= 0xE0;
        can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF, 1, 0, 0, &buf);                    // RX0IF clear 
    }
}

/* ---------------------------------------------------------------------------------------
 *  Callback processing after interrupt status acquisition
 * --------------------------------------------------------------------------------------- */
void can3_sts_event(MCP2515REG_INTERR *rxd)
{
    MCP2515REG_BITX bc, ec;

    bc.BYTE.MSK.BYTE    = rxd->BYTE.CANINTF.BYTE & 0xFC;
    bc.BYTE.PAT.BYTE    = 0;

    if (rxd->BYTE.CANINTF.BYTE == 0) {
        return;
    }
    if (rxd->BYTE.CANINTF.BIT.RX0IF) { // Receive buffer 0 full 
        can3_request(
            MCP2515CMD_READ, 
            MCP2515AD_RXB0CTRL, 0,
            7, CAN3_CallbackRx0, 0
        );   // Read RXB0 
    }
    if (rxd->BYTE.CANINTF.BIT.RX1IF) { // Receive buffer 1 full 
        can3_request(
            MCP2515CMD_READ,
            MCP2515AD_RXB1CTRL, 0,
            7, CAN3_CallbackRx1, 0
        );   // Read RXB1 
    }

    // Transmission completion check 
    if (rxd->BYTE.CANINTF.BIT.TX0IF) { // Transmit buffer 0 empty 
        tx_act[0] = 0;
    }
    if (rxd->BYTE.CANINTF.BIT.TX1IF) { // Transmit buffer 1 empty 
        tx_act[1] = 0;
    }
    if (rxd->BYTE.CANINTF.BIT.TX2IF) { // Transmit buffer 2 empty 
        tx_act[2] = 0;
    }

    if (rxd->BYTE.CANINTF.BIT.ERRIF) { // Error interrupt 
        logging("CAN3:Error %02X\r", (int)rxd->BYTE.EFLG.BYTE);
        ec.BYTE.MSK.BYTE = rxd->BYTE.EFLG.BYTE;
        ec.BYTE.PAT.BYTE = 0;
        can3_request(
            MCP2515CMD_BITX,
            MCP2515AD_CANINTF,
            1, 0, 0, &ec
        ); // Clear interrupt request flag 
    }

    if (rxd->BYTE.CANINTF.BIT.WAKIF) { // Wakeup interrupt 
        logging("CAN3:Wakeup\r");
    }
    if (rxd->BYTE.CANINTF.BIT.MERRF) { // Message error interrupt 
        logging("CAN3:MsgErr\r");
    }

    can3_request(
        MCP2515CMD_BITX,
        MCP2515AD_CANINTF,
        1, 0, 0, &bc
    ); // Clear interrupt request flag 
}

/* ---------------------------------------------------------------------------------------
 *  Function description   : External CAN module interrupt (MCP2515 - INT)
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_ICU_IRQ0} ICU_IRQ0_ISR(void)
{
    can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0); // Get status 
}

/* ---------------------------------------------------------------------------------------
 *  Function description   : External CAN module interrupt (MCP2515 - RX0BF)
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_ICU_IRQ6} ICU_IRQ6_ISR(void)
{
    const unsigned short buf = 0x0001;
    can3_request(MCP2515CMD_READ, MCP2515AD_RXB0CTRL, 0, 7, CAN3_CallbackRx0, 0); // Read RXB0 
    can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF , 1, 0, 0, &buf); // RX0IF clear 
}

/* ---------------------------------------------------------------------------------------
 *  Function description   : External CAN module interrupt (MCP2515 - RX1BF)
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_ICU_IRQ7} ICU_IRQ7_ISR(void)
{
    const unsigned short buf = 0x0002;
    can3_request(MCP2515CMD_READ, MCP2515AD_RXB1CTRL, 0, 7, CAN3_CallbackRx1, 0); // RXB1 read 
    can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF , 1, 0, 0, &buf); // RX0IF clear 
}

#endif // ifdef      RSPI2_ACTIVATE

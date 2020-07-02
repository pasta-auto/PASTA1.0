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
 * RX63N For uSD RSPI1-I/F Communication
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2016/02/10 Start coding (by Tachibana)
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
#include "ecu.h"       // ECU common definition 
#include "uSD_rspi1.h"

/*
 *  Port setting
 *
 *          Port        SCI        I2C        SPI        Apply
 *  ----------------------------------------------------------------------------
 *  RSPI1   PE7         MISOB                            <RSPI>  uSD
 *          PE6         MOSIB                            <RSPI>  uSD
 *          PE5         RSPCKB                           <RSPI>  uSD
 *          PE4         SSLB0                            <RSPI>  uSD
 */

#ifdef      RSPI1_ACTIVATE

/*
 * // RSPI management structure for uSD
 * typedef struct __spi_module__ {
 *  int    err;       // Error flag
 *  void   *rx_proc;  // Call function at reception completion interrupt
 *  void   *tx_proc;  // Call function at transmittion completion interrupt
 *  void   *ti_proc;  // Call function at idling interrupt
 *  void   *err_proc; // Call function at error interrupt
 * } SPI_MODULE;
 */
SPI_MODULE usd_spi_com;

// Log function 
void logging(char *fmt, ...);

/* ----------------------------------------------------------------------------------------
 * rspi1_init
 * 
 *  Function description
 *   RSPI1 initialization
 *           Port        SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   RSPI1   PE7         MISOB                            <RSPI>  uSD
 *           PE6         MOSIB                            <RSPI>  uSD
 *           PE5         RSPCKB                           <RSPI>  uSD
 *           PE4         SSLB0                            <RSPI>  uSD
 *  
 *  Argument
 *      speed  Communication speed 100,000 to 10,000,000
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void rspi1_init(long bps)
{
    memset(&usd_spi_com, 0, sizeof(SPI_MODULE));

    SYSTEM.PRCR.WORD    = 0xA502; // Unprotect 
    MSTP_RSPI1          = 0;      // RSPI1 module stop release 

    // RSPI1 Disable interrupt requests 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0; // Group 12 Disable error interrupt 
    ICU.IER[IER_RSPI1_SPRI1].BIT.IEN_RSPI1_SPRI1 = 0; // Disable receive buffer full interrupt 
    ICU.IER[IER_RSPI1_SPTI1].BIT.IEN_RSPI1_SPTI1 = 0; // Disable transmission empty interrupt 
    ICU.IER[IER_RSPI1_SPII1].BIT.IEN_RSPI1_SPII1 = 0; // Disable transmission idle interrupt 

    RSPI1.SPCR.BYTE = 0;        // Module initialization 

    // CAN3 pin setting (via RSPI2) 
    PORTE.PODR.BYTE  = 0x70; // Port initialization 
    PORTE.PDR.BIT.B7 = 0;    // PB7 <- MISOB 
    PORTE.PDR.BIT.B6 = 1;    // PB6 -> MOSIB 
    PORTE.PDR.BIT.B5 = 1;    // PB5 -> RSPCKB 
    PORTE.PDR.BIT.B4 = 1;    // PB4 -> SSLB0 

    PORTE.PMR.BIT.B7 = 1;    // Peripheral functions MISOB 
    PORTE.PMR.BIT.B6 = 1;    // Peripheral functions MOSIB 
    PORTE.PMR.BIT.B5 = 1;    // Peripheral functions RSPCKB 
    PORTE.PMR.BIT.B4 = 1;    // Peripheral functions SSLB0 

    MPC.PWPR.BIT.B0WI  = 0;
    MPC.PWPR.BIT.PFSWE = 1;

    MPC.PE7PFS.BYTE = 0x0D; // MISOB  SO 
    MPC.PE6PFS.BYTE = 0x0D; // MOSIB  SI 
    MPC.PE5PFS.BYTE = 0x0D; // RSPCKB  SCK 
    MPC.PE4PFS.BYTE = 0x0D; // SSLB0  /CS 

    MPC.PWPR.BIT.PFSWE = 0; 
    MPC.PWPR.BIT.B0WI  = 1; 

    SYSTEM.PRCR.WORD = 0xA500; // Port setting prohibited 

    // RSPI1 setting (single master mode) 
    RSPI1.SSLP.BYTE  = 0;    // SSLnP is active low 
    RSPI1.SPPCR.BYTE = 0x20; // MOSI idle output is low 
    RSPI1.SPSR.BYTE  &= 0;   // Error flag release 
    RSPI1.SPSCR.BYTE = 0;    // Sequence initial value 

    /* Set baud rate to 1Mbps N value (BRDV[1:0])=0 Fixed Min.=93,750bps
     * n = (PCLK Frequency) / (2 * 2^N * Bit Rate) - 1
     * n = (48,000,000) / (2 * 2^0 * 1,000,000) - 1
     * n = 24*/
    RSPI1.SPBR.BYTE  = 48000000 / (2 * bps) - 1;
    RSPI1.SPDCR.BYTE = 0x20; // SPDR is for longword access / read reception buffer / one frame 
    RSPI1.SPCKD.BYTE = 0;    // Clock delay 1RSPCK 
    RSPI1.SSLND.BYTE = 0;    // SSL negate delay 1RSPCK 
    RSPI1.SPND.BYTE  = 0;    // Next access delay 1RSPCK + 2PCLK 
    RSPI1.SPCR2.BYTE = 0;    // Parity invalid / Idle interrupt disable 

    /* Command register initialization
     * RSPCK phase setting bit*/
    RSPI1.SPCMD0.BIT.CPHA = 0; /* 0: Data sample at odd edge, data change at even edge
                                * 1: Data change at odd edge, data sample at even edge*/
    // RSPCK polarity setting bit 
    RSPI1.SPCMD0.BIT.CPOL = 0; /* 0: RSPCK at idle is low
                                * 1: RSPCK at idle is high*/
    // Bit rate division setting bit 
    RSPI1.SPCMD0.BIT.BRDV = 0; /* b3 b2
                                * 0 0: Base bit rate
                                * 0 1: Base bit rate divided by 2
                                * 1 0: Base bit rate divided by 4
                                * 1 1: Base bit rate divided by 8*/
    // SSL signal assertion setting bit 
    RSPI1.SPCMD0.BIT.SSLA = 0; /* b6 b4
                                * 0 0 0: SSL0
                                * 0 0 1: SSL1
                                * 0 1 0: SSL2
                                * 0 1 1: SSL3
                                * 1 x x: Do not set
                                * x: Don't care*/
    // SSL signal level hold bit 
    RSPI1.SPCMD0.BIT.SSLKP = 0; /* 0: Negate all SSL signals at the end of transfer
                                 * 1: SSL signal level is maintained from the end of transfer until the start of the next access*/
    // RSPI data length setting bit 
    RSPI1.SPCMD0.BIT.SPB = 4; /* b11 b8
                               * 0100 to 0111 : 8bit
                               * 1 0 0 0: 9bit
                               * 1 0 0 1: 10bit
                               * 1 0 1 0: 11bit
                               * 1 0 1 1: 12bit
                               * 1 1 0 0: 13bit
                               * 1 1 0 1: 14bit
                               * 1 1 1 0: 15bit
                               * 1 1 1 1: 16bit
                               * 0 0 0 0: 20bit
                               * 0 0 0 1: 24bit
                               * 0010,0011 : 32bit*/
    // RSPI LSB first bit 
    RSPI1.SPCMD0.BIT.LSBF = 0; /* 0: MSB first
                                * 1: LSB first*/
    // RSPI next access delay enable bit 
    RSPI1.SPCMD0.BIT.SPNDEN = 0; /* 0: Next access delay is 1RSPCK + 2PCLK
                                  * 1: Next access delay is the set value of the RSPI next access delay register (SPND)*/
    // SSL negate delay setting permission bit 
    RSPI1.SPCMD0.BIT.SLNDEN = 0; /* 0: SSL negate delay is 1RSPCK
                                  * 1: SSL negation delay is the set value of the RSPI slave select negate delay register (SSLND)*/
    // RSPCK delay setting enable bit 
    RSPI1.SPCMD0.BIT.SCKDEN = 0; /* 0: RSPCK delay is 1 RSPCK
                                  * 1: RSPCK delay is the value set in the RSPI clock delay register (SPCKD)*/

    // Copy settings 
    RSPI1.SPCMD1.WORD = RSPI1.SPCMD0.WORD;
    RSPI1.SPCMD2.WORD = RSPI1.SPCMD0.WORD;
    RSPI1.SPCMD3.WORD = RSPI1.SPCMD0.WORD;
    RSPI1.SPCMD4.WORD = RSPI1.SPCMD0.WORD;
    RSPI1.SPCMD5.WORD = RSPI1.SPCMD0.WORD;
    RSPI1.SPCMD6.WORD = RSPI1.SPCMD0.WORD;
    RSPI1.SPCMD7.WORD = RSPI1.SPCMD0.WORD;

    /* Operation permission
     * RSPI mode select bit*/
    RSPI1.SPCR.BIT.SPMS = 0; /* 0: SPI operation (4-wire)
                              * 1: Clock synchronous operation (3-wire type)*/
    // Communication operation mode selection bit 
    RSPI1.SPCR.BIT.TXMD = 0; /* 0: Full-duplex synchronous serial communication
                              * 1: Serial communication only for transmission operation*/
    // Mode fault error detection enable bit 
    RSPI1.SPCR.BIT.MODFEN = 0; /* 0: Disable mode fault error detection
                                * 1: Enable  mode fault error detection*/
    // RSPI master / slave mode select bit 
    RSPI1.SPCR.BIT.MSTR = 1; /* 0: Slave  mode
                              * 1: Master mode*/
    // RSPI error interrupt enable bit 
    RSPI1.SPCR.BIT.SPEIE = 0; /* 0: Disable generation of RSPI error interrupt request
                               * 1: Enable  generation of RSPI error interrupt request*/
    // RSPI transmission interrupt enable bit 
    RSPI1.SPCR.BIT.SPTIE = 0; /* 0: Disable generation of RSPI transmission interrupt request
                               * 1: Enable  generation of RSPI transmission interrupt request*/
    // RSPI reception interrupt enable bit 
    RSPI1.SPCR.BIT.SPRIE = 0; /* 0: Disable generation of RSPI reception interrupt request
                               * 1: Enable  generation of RSPI reception interrupt request*/
    // RSPI idle interrupt enable bit 
    RSPI1.SPCR2.BIT.SPIIE = 0; /* 0: Disable generation of idle interrupt request
                                * 1: Enable  generation of idle interrupt request*/
    // RSPI function enable bit 
    RSPI1.SPCR.BIT.SPE = 1; /* 0: RSPI function is disabled
                             * 1: RSPI function is enabled*/

    // Interrupt priority setting 
    ICU.IPR[IPR_RSPI1_].BIT.IPR = 2; // Interrupt level setting 
    // Clear interrupt flag 
    ICU.IR[IR_RSPI1_SPRI1].BIT.IR = 0;
    ICU.IR[IR_RSPI1_SPTI1].BIT.IR = 0;
    ICU.IR[IR_RSPI1_SPII1].BIT.IR = 0;
    // GROUP12 interrupt setting 
    ICU.GEN[GEN_RSPI1_SPEI1].BIT.EN_RSPI1_SPEI1 = 1;  // Group 12 Enable RSPI1 receive error interrupt 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR            = 1;  // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR               = 0;  // Group 12 interrupt flag clear 
    // Interrupt enable setting 
    ICU.IER[IER_RSPI1_SPRI1].BIT.IEN_RSPI1_SPRI1 = 1; // Enable receive buffer full interrupt 
    ICU.IER[IER_RSPI1_SPTI1].BIT.IEN_RSPI1_SPTI1 = 1; // Enable transmission empty interrupt 
    ICU.IER[IER_RSPI1_SPII1].BIT.IEN_RSPI1_SPII1 = 1; // Enable transmission idle interrupt 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 Enable interrupt 
}

/* ---------------------------------------------------------------------------------------
 * SPRI1 Receive buffer full Data arrives at SPI receive buffer
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_RSPI1_SPRI1} RSPI1_SPRI1_ISR(void)
{
    USD_PROC_CALL proc;
    // RSPI reception interrupt enable bit 
    RSPI2.SPCR.BIT.SPRIE = 0; /* 0: Disable generation of RSPI reception interrupt request
                               * 1: Enable  generation of RSPI reception interrupt request*/
    if (usd_spi_com.rx_proc) { // User processing function call 
        proc = (USD_PROC_CALL)usd_spi_com.rx_proc;
        proc(0);
    }
}

/* ---------------------------------------------------------------------------------------
 * SPTI1 transmit buffer empty SPI transmit buffer free
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_RSPI1_SPTI1} RSPI1_SPTI1_ISR(void)
{
    USD_PROC_CALL proc;
    // RSPI transmission interrupt enable bit 
    RSPI2.SPCR.BIT.SPTIE = 0; /* 0: Disable generation of RSPI transmission interrupt request
                               * 1: Enable  generation of RSPI transmission interrupt request*/
    if (usd_spi_com.tx_proc) { // User processing function call 
        proc = (USD_PROC_CALL)usd_spi_com.tx_proc;
        proc(0);
    }
}

/* ---------------------------------------------------------------------------------------
 * SPII1 Idle Interrupt occurs when the last write of transmission data is completed (transmission completed)
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_RSPI1_SPII1} RSPI1_SPII1_ISR(void)
{
    USD_PROC_CALL proc;
    // RSPI idle interrupt enable bit 
    RSPI1.SPCR2.BIT.SPIIE = 0; /* 0: Disable generation of idle interrupt request
                                * 1: Enable  generation of idle interrupt request*/
    if (usd_spi_com.ti_proc) { // User processing function call 
        proc = (USD_PROC_CALL)usd_spi_com.ti_proc;
        proc(0);
    } else { // Clear interrupt condition 
        if (usd_spi_com.tx_proc) { // User processing function call 
            proc = (USD_PROC_CALL)usd_spi_com.tx_proc;
            proc(0);
        }
    }
}
#endif //RSPI1_ACTIVATE

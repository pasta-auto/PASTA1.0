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
 * RX63N SCI-I/F communication
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
#include "ecu.h"            // ECU common definition 
#include "sci.h"

/*
 *  Port setting
 *
 *          Port        SCI        I2C        SPI        Apply
 *  ----------------------------------------------------------------------------
 *  SCI0    P20         TXD0       SDA0       SMOSI0  <RS-232C> COM0
 *          P21         RXD0       SCL0       SMISO0  <RS-232C> COM0
 *          P86         nRTS0                         <RS-232C> COM0
 *          P87         nCTS0                         <RS-232C> COM0
 *          P70         TX0SDN                        <RS-232C> Transmission permission
 *
 *  SCI1    P26         TXD1                          <REM-MON> COM1
 *          P30         RXD1                          <REM-MON> COM1
 *          PE1         nRTS1/(TXD12)                 <RS-232C> COM1/(COM12)
 *          PE2         nCTS1/(RXD12)                 <RS-232C> COM1/(COM12)
 *
 *  SCI2    P13         TXD2       SDA0               <RS-232C> COM2
 *          P12         RXD2       SCL0               <RS-232C> COM2
 *          P15         nRTS2                         <RS-232C> COM2
 *          P17         nCTS2                         <RS-232C> COM2
 *          P73         TX2SDN                        <RS-232C> Transmission permission
 *
 *  SCI3    P23         TXD3                          <RS-232C> COM3
 *          P25         RXD3                          <RS-232C> COM3
 *          P22         nRTS3                         <RS-232C> COM3
 *          P24         nCTS3                         <RS-232C> COM3
 *          P56         nEXRES                        </RESET> External module reset signal
 *
 *  SCI5    PC3         TXD5        SSDA5     SMOSI5  <SPI/I2C> External extension
 *          PC2         RXD5        SSCL5     SMISO5  <SPI/I2C> External extension
 *          PC4                     SCK5              <SPI>  External extension
 *          PC5                     SS0               <SPI>  External extension
 *          PC6                     SS1               <SPI>  External extension
 *
 *  SCI6    P00         TXD6       SSDA6      SMISO6  <TTL>  COM6
 *          P01         RXD6       SSCL6      SMOSI6  <TTL>  COM6
 *          P02         nRTS6                 SCK6    <TTL>  COM6
 *          PJ3         nCTS6                 SS6     <TTL>  COM6
 */

// SCI0,SCI1,SCI2,SCI3,SCI5,SCI6 SCI1 = Use with yellow scope 
SCI_MODULE sci_com[7];

// Log function 
void logging(char *fmt, ...);

/* ----------------------------------------------------------------------------------------
 * sci0_init
 *
 *  Function description
 *   SCI0 initialization
 *            Port       SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   SCI0     P20        TXD0       SDA0       SMOSI0     <RS-232C> COM0
 *            P21        RXD0       SCL0       SMISO0     <RS-232C> COM0
 *            P86        nRTS0                            <RS-232C> COM0
 *            P87        nCTS0                            <RS-232C> COM0
 *            P70        TX0SDN                           <RS-232C> Transmit permission
 * 
 *  Argument
 *      bps      Communication speed 300 to 115200
 *      datalen  Data length 7,8
 *      stoplen  Stop length 1,2
 *      parity   Parity  0 = none / 1 = odd number / 2 = even number
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void sci0_init(long bps, int datalen, int stoplen, int parity)
{
    SCI_MODULE *com = &sci_com[0];
    memset(com, 0, sizeof(SCI_MODULE));

#ifdef      SCI0_ACTIVATE

    SYSTEM.PRCR.WORD    = 0xA502;   // Unprotect 
    MSTP_SCI0           = 0;        // Release SCI module stop 

    // Disable SCI interrupt request 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0    = 0; // Group 12 interrupt disabled 
    ICU.IER[IER_SCI0_RXI0].BIT.IEN_SCI0_RXI0        = 0; // Disable receive interrupt 
    ICU.IER[IER_SCI0_TXI0].BIT.IEN_SCI0_TXI0        = 0; // Disable transmission end interrupt 
    ICU.IER[IER_SCI0_TEI0].BIT.IEN_SCI0_TEI0        = 0; // Disable transmission empty interrupt 

    // Enable write protection (Set protection) 
    SYSTEM.PRCR.WORD = 0xA500;      // Protect 

    /* SCR       - Serial Control Register
     *  b7  TIE  - Transmit Interrupt Enable     - A TXI interrupt request is disabled
     *  b6  RIE  - Receive Interrupt Enable      - RXI and ERI interrupt requests are disabled
     *  b5  TE   - Transmit Enable               - Serial transmission is disabled
     *  b4  RE   - Receive Enable                - Serial reception is disabled
     *  b2  TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled*/
    SCI0.SCR.BYTE = 0x00;

    while (0x00 != (SCI0.SCR.BYTE & 0xF0)) {
        // Confirm that bit is actually 0 
    }

    // Set the I/O port functions 

    // General-purpose port P20:TXD0, P21:RXD0 (P22:DE/RE) 
    PORT2.PODR.BIT.B0   = 1;    // TXD 
    PORT8.PODR.BIT.B6   = 1;    // RTS 
    // Set port direction - TXDn is output port, RXDn is input port(Port I/O settings) 
    PORT2.PDR.BIT.B0    = 1;    // Output TXD 
    PORT2.PDR.BIT.B1    = 0;    // Input RXD 
    PORT8.PDR.BIT.B6    = 1;    // Output RTS 
    PORT8.PDR.BIT.B7    = 0;    // Input CTS 

    // Set port mode - Use pin as general I/O port 
    PORT2.PMR.BIT.B0    = 0;    // General-purpose IO port setting 
    PORT2.PMR.BIT.B1    = 0;    // General-purpose IO port setting 
    PORT8.PMR.BIT.B6    = 0;    // General-purpose IO port setting 
    PORT8.PMR.BIT.B7    = 0;    // General-purpose IO port setting 

    PORT7.PDR.BIT.B0    = 1;    // Output 
    PORT7.PODR.BIT.B0   = 1;    // 0=SDN / 1=Normal 
    PORT7.PMR.BIT.B0    = 0;    // General-purpose IO port setting 

    /* PWPR - Write-Protect Register(Write protect register)
     *  b7  B0WI  - PFSWE Bit Write Disable - PFSWE disable
     *  b6  PFSWE - PFS Register Write Enable - PFS enable
     *  b5:b0 Reserved - These bits are read as 0. The write value should be 0.*/
    MPC.PWPR.BIT.B0WI   = 0;        // Set to 0 first 
    MPC.PWPR.BIT.PFSWE  = 1;        // Set to 1 later 

    /* PFS - Pin Function Control Register(Pin function register setting)
     *  b3:b0 PSEL - Pin Function Select - RXDn, TXDn*/
    MPC.P20PFS.BYTE = 0x0A;         // assign I/O pin to SCI0 TxD0 
    MPC.P21PFS.BYTE = 0x0A;         // assign I/O pin to SCI0 RxD0 
    MPC.P86PFS.BYTE = 0x00;         // assign I/O pin to Port 
    MPC.P87PFS.BYTE = 0x00;         // assign I/O pin to Port 
    MPC.P07PFS.BYTE = 0x00;         // assign I/O pin to Port 
    // Apply write protection 
    MPC.PWPR.BIT.PFSWE  = 0;
    MPC.PWPR.BIT.B0WI   = 1;

    // Use pin as I/O port for peripheral functions(IO pin function setting) 
    PORT2.PMR.BIT.B0    = 1;        // Peripheral function settings 
    PORT2.PMR.BIT.B1    = 1;        // Peripheral function settings 

    /* Initialization of SCI
     * Disable all module clock stop mode*/
    SYSTEM.MSTPCRA.BIT.ACSE = 0;
    // Release of SCI0 module stop state 
    MSTP_SCI0 = 0;


    // Select an On-chip baud rate generator to the clock source 
    SCI0.SCR.BIT.CKE = 0;

    /* SMR       - Serial Mode Register
     * b7  CM    - Communications Mode  - Asynchronous mode
     * b6  CHR   - Character Length     - Selects 8 bits as the data length
     * b5  PE    - Parity Enable        - When transmitting : Parity bit addition is not performed
     *                                    When receiving : Parity bit checking is not performed
     * b3  STOP  - Stop Bit Length      - 2 stop bits
     * b2  MP    - Multi-Processor Mode - Multi-processor communications function is disabled
     * b1:b0 CKS - Clock Select         - PCLK clock (n = 0)*/
    SCI0.SMR.BYTE = 0x08;

    /* SCMR           - Smart Card Mode Register
     * b6:b4 Reserved - The write value should be 1.
     * b3  SDIR       - Transmitted/Received Data Transfer Direction - Transfer with LSB-first
     * b2  SINV       - Transmitted/Received Data Invert             - TDR contents are transmitted as they are.
     *                                                                 Receive data is stored as it is in RDR.
     * b1  Reserved   - The write value should be 1.
     * b0  SMIF       - Smart Card Interface Mode Select - Serial communications interface mode*/
    SCI0.SCMR.BYTE = 0xF2;

    /* SEMR           - Serial Extended Mode Register
     * b7:b6 Reserved - The write value should be 0.
     * b5  NFEN       - Digital Noise Filter Function Enable - Noise cancellation function
     *                                                         for the RXDn input signal is disabled.
     * b4  ABCS       - Asynchronous Mode Base Clock Select  - Selects 16 base clock cycles for 1-bit period
     * b3:b1 Reserved - The write value should be 0.*/
    SCI0.SEMR.BYTE = 0x00;

    /* Set data transfer format in Serial Mode Register (SMR)* /
     * -Asynchronous Mode`
     * -8 bits
     * -no parity
     * -1 stop bit
     * -PCLK clock (n = 0)*/
    SCI0.SMR.BYTE = 0x00; // 0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64 

    /* BRR - Bit Rate Register
     * Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04*/
    if (stoplen == 1) {
        SCI0.SMR.BIT.STOP = 1;
    } else {
        SCI0.SMR.BIT.STOP = 0;
    }

    if (parity == 0) {
        SCI0.SMR.BIT.PE = 0;
    } else if (parity == 1) { //Odd parity 
        SCI0.SMR.BIT.PE = 1;
        SCI0.SMR.BIT.PM = 1;
    } else if (parity == 2) { // Even parity 
        SCI0.SMR.BIT.PE = 1;
        SCI0.SMR.BIT.PM = 0;
    }

    if (datalen == 7) { // 7bit length 
        SCI0.SMR.BIT.CHR = 1;
    } else if (datalen == 8) { // 8bit length 
        SCI0.SMR.BIT.CHR = 0;
    }

    /* Set baud rate to 115200
     * N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
     * N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
     * N = 12*/
    SCI0.BRR = 48000000 / ((64/2) * bps) - 1;

    // SCI interrupt priority setting 
    ICU.IPR[IPR_SCI0_].BIT.IPR = 1;                     // Interrupt level setting 

    // SCI0 interrupt setting 
    ICU.IER[IER_SCI0_RXI0].BIT.IEN_SCI0_RXI0    = 1;    // Receive interrupt 
    ICU.IER[IER_SCI0_TXI0].BIT.IEN_SCI0_TXI0    = 1;    // Transmission completion interrupt 
    ICU.IER[IER_SCI0_TEI0].BIT.IEN_SCI0_TEI0    = 1;    // Transmit empty interrupt 

    // Clear interrupt flag 
    ICU.IR[IR_SCI0_RXI0].BIT.IR = 0;
    ICU.IR[IR_SCI0_TXI0].BIT.IR = 0;
    ICU.IR[IR_SCI0_TEI0].BIT.IR = 0;

    // GROUP12 interrupt setting 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 interrupt enable 
    ICU.GEN[GEN_SCI0_ERI0].BIT.EN_SCI0_ERI0      = 1; // Group 12 SCI0 reception error interrupt enabled 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR             = 1; // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR                = 0; // Group 12 interrupt flag clear 

    SCI0.SCR.BIT.RIE    = 1;
    SCI0.SCR.BIT.RE     = 1;

#endif //SCI0_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * sci1_init
 * 
 *  Function description
 *   SCI1 initialization
 *           Port        SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   SCI1    P26         TXD1                             <REM-MON> COM1
 *           P30         RXD1                             <REM-MON> COM1
 *           PE1         TXD12/nRTS1                      <RS-232C> COM1/12
 *           PE2         RXD12/nCTS1                      <RS-232C> COM1/12
 * 
 *  Argument
 *      bps      Communication speed 300 to 115200
 *      datalen  Data length 7,8
 *      stoplen  Stop length 1,2
 *      parity   parity 0=none / 1=odd / 2=even
 * 
 *  Return
 *      None
 *  
 *  Note 
 *      SCI1 is used with Yellow Scope
 * ----------------------------------------------------------------------------------------*/
#ifdef      SCI1_ACTIVATE
#ifdef  __YIDE_REM_DEBUG__
// 217 
void interrupt sci1_rxi(void);
// 218 
void interrupt sci1_txi(void);
// 219 
void interrupt sci1_tei(void);
#endif // ifdef  __YIDE_REM_DEBUG__
#endif //SCI1_ACTIVATE
void sci1_init(long bps, int datalen, int stoplen, int parity)
{
#ifdef      SCI1_ACTIVATE
#ifdef  __YIDE_REM_DEBUG__
    static unsigned long *x_intb;
#endif
    unsigned short i;
#endif //SCI1_ACTIVATE
    SCI_MODULE *com = &sci_com[1];

    memset(com, 0, sizeof(SCI_MODULE));

#ifdef      SCI1_ACTIVATE

    SYSTEM.PRCR.WORD    = 0xA502;   // Unprotect 
    MSTP_SCI1           = 0;        // SCI module stop release 

    // Disable SCI interrupt request 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0    = 0; // Group L0 interrupt disabled 
    ICU.IER[IER_SCI1_RXI1].BIT.IEN_SCI1_RXI1        = 0; // Disable receive interrupt 
    ICU.IER[IER_SCI1_TXI1].BIT.IEN_SCI1_TXI1        = 0; // Disable transmission end interrupt 
    ICU.IER[IER_SCI1_TEI1].BIT.IEN_SCI1_TEI1        = 0; // Disable transmission empty interrupt 

#ifdef  __YIDE_REM_DEBUG__
    // Rewrite the contents of the interrupt vector and forcibly release the remote monitor 
    _asm extern _x_intb
    _asm MVFC INTB, r0
    _asm MOV.L   #_x_intb, r1
    _asm MOV.L r0, [r1]
    x_intb[VECT_SCI1_RXI1]  = (unsigned long)sci1_rxi;
    x_intb[VECT_SCI1_TXI1]  = (unsigned long)sci1_txi;
    x_intb[VECT_SCI1_TEI1]  = (unsigned long)sci1_tei;

#endif // ifdef  __YIDE_REM_DEBUG__

    // Enable write protection(Set protection) 
    SYSTEM.PRCR.WORD = 0xA500;      // Protect 

    /* SCR      - Serial Control Register
     * b7  TIE  - Transmit Interrupt Enable     - A TXI interrupt request is disabled
     * b6  RIE  - Receive Interrupt Enable      - RXI and ERI interrupt requests are disabled
     * b5  TE   - Transmit Enable               - Serial transmission is disabled
     * b4  RE   - Receive Enable                - Serial reception is disabled
     * b2  TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled*/
    SCI1.SCR.BYTE = 0x00;

    while (0x00 != (SCI1.SCR.BYTE & 0xF0)) {
        // Confirm that bit is actually 0 
    }

    // Set the I/O port functions 

    // General-purpose port P26:TXD1, P30:RXD1, (PE1:RTS1, PE2:CTS1) 
    PORT2.PODR.BIT.B6   = 1;    // TXD 
    PORTE.PODR.BIT.B1   = 1;    // RTS=Disable 
    // Set port direction - TXDn is output port, RXDn is input port(Port I/O settings) 
    PORT2.PDR.BIT.B6    = 1;    // Output TXD 
    PORT3.PDR.BIT.B0    = 0;    // Input RXD 
    PORTE.PDR.BIT.B1    = 1;    // Output RTS 
    PORTE.PDR.BIT.B2    = 0;    // Input CTS 

    // Set port mode - Use pin as general I/O port 
    PORT2.PMR.BIT.B6    = 0;    // General-purpose IO port setting 
    PORT3.PMR.BIT.B0    = 0;    // General-purpose IO port setting 
    PORTE.PMR.BIT.B1    = 0;    // General-purpose IO port setting 
    PORTE.PMR.BIT.B2    = 0;    // General-purpose IO port setting 

    /* PWPR           - Write-Protect Register(Write protect register)
     * b7  B0WI       - PFSWE Bit Write Disable   - PFSWE disable
     * b6  PFSWE      - PFS Register Write Enable - PFS enable
     * b5:b0 Reserved - These bits are read as 0. The write value should be 0.*/
    MPC.PWPR.BIT.B0WI   = 0;    // Set to 0 first 
    MPC.PWPR.BIT.PFSWE  = 1;    // Set to 1 later 

    MPC.P26PFS.BYTE = 0x0A;     // assign I/O pin to SCI1 TxD1 
    MPC.P30PFS.BYTE = 0x0A;     // assign I/O pin to SCI1 RxD1 
    MPC.PE1PFS.BYTE = 0x00;     // assign I/O pin to Port 
    MPC.PE2PFS.BYTE = 0x00;     // assign I/O pin to Port 

    // Apply write protection 
    MPC.PWPR.BIT.PFSWE  = 0;
    MPC.PWPR.BIT.B0WI   = 1;

    PORT2.PMR.BIT.B6    = 1;    // Used as peripheral functions 
    PORT3.PMR.BIT.B0    = 1;    // Used as peripheral functions 

    // Disable all module clock stop mode 
    SYSTEM.MSTPCRA.BIT.ACSE = 0;
    // Release of SCI1 module stop state 
    MSTP_SCI1 = 0;

    SCI1.SCR.BYTE = 0x00;       // Disable Tx/Rx and set clock to internal 
    /* Set data transfer format in Serial Mode Register (SMR)
     *  -Asynchronous Mode`
     *  -8 bits
     *  -no parity
     *  -1 stop bit
     *  -PCLK clock (n = 0)*/
    SCI1.SMR.BYTE = 0x00;       // 0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64 

    if (stoplen == 1) {
        SCI1.SMR.BIT.STOP = 1;
    } else {
        SCI1.SMR.BIT.STOP = 0;
    }

    if (parity == 0) {
        SCI1.SMR.BIT.PE = 0;
    } else if (parity == 1) { //Odd parity 
        SCI1.SMR.BIT.PE = 1;
        SCI1.SMR.BIT.PM = 1;
    } else if (parity == 2) { // Even parity 
        SCI1.SMR.BIT.PE = 1;
        SCI1.SMR.BIT.PM = 0;
    }

    if (datalen == 7) { // 7bit length 
        SCI1.SMR.BIT.CHR = 1;
    } else if (datalen == 8) { // 8bit length 
        SCI1.SMR.BIT.CHR = 0;
    }

    /* Set baud rate to 115200
     *  N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
     *  N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
     *  N = 12*/

    SCI1.BRR = 48000000 / ((64/2) * bps) - 1;
    // Change by clock division setting 

    // Wait at least one bit interval 
    for (i = 0; i < 5000; i++) {
        ; // assume minimum of 2 instructions at 98MHz ? 
    }
    // SCI interrupt priority setting 
    ICU.IPR[IPR_SCI1_].BIT.IPR = 1; // Interrupt level setting 

    // SCI1 interrupt setting 
    ICU.IER[IER_SCI1_RXI1].BIT.IEN_SCI1_RXI1 = 1; // Receive interrupt 
    ICU.IER[IER_SCI1_TXI1].BIT.IEN_SCI1_TXI1 = 1; // Transmission completion interrupt 
    ICU.IER[IER_SCI1_TEI1].BIT.IEN_SCI1_TEI1 = 1; // Transmit empty interrupt 

    // Clear interrupt flag 
    ICU.IR[IR_SCI1_RXI1].BIT.IR = 0;
    ICU.IR[IR_SCI1_TXI1].BIT.IR = 0;
    ICU.IR[IR_SCI1_TEI1].BIT.IR = 0;

    // GROUP12 interrupt setting 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 interrupt enable 
    ICU.GEN[GEN_SCI1_ERI1].BIT.EN_SCI1_ERI1      = 1; // Group 12 SCI1 receive error interrupt enabled 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR             = 1; // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR                = 0; // Group 12 interrupt flag clear 

    SCI1.SCR.BIT.RIE    = 1;
    SCI1.SCR.BIT.RE     = 1;

#ifdef  SCI1_FLOW
    PORTE.PODR.BIT.B1 = 0;      // RTS=Enable 
#endif

#endif //SCI1_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * sci2_init
 * 
 *  Function description
 *   SCI2 initialization
 *           Port        SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   SCI2    P13         TXD2                  SDA0       <RS-232C> COM2
 *           P12         RXD2                  SCL0       <RS-232C> COM2
 *           P15         nRTS2                            <RS-232C> COM2
 *           P17         nCTS2                            <RS-232C> COM2
 *           P73         TX2SDN                           <RS-232C> Transmit permission
 * 
 *  Argument
 *      bps     Communication speed 300 to 115200
 *      datalen  Data lentgh 7,8
 *      stoplen  Stop length 1,2
 *      parity   Parity 0=none / 1=odd / 2=even
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void sci2_init(long bps, int datalen, int stoplen, int parity)
{
    SCI_MODULE *com = &sci_com[2];
    memset(com, 0, sizeof(SCI_MODULE));

#ifdef      SCI2_ACTIVATE

    SYSTEM.PRCR.WORD    = 0xA502;   // Unprotect 
    MSTP_SCI2           = 0;        // SCI module stop release 

    // SCI Disable interrupt requests 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0; // Group 12 interrupt disabled 
    ICU.IER[IER_SCI2_RXI2].BIT.IEN_SCI2_RXI2     = 0; // Disable receive interrupt 
    ICU.IER[IER_SCI2_TXI2].BIT.IEN_SCI2_TXI2     = 0; // Disable transmission end interrupt 
    ICU.IER[IER_SCI2_TEI2].BIT.IEN_SCI2_TEI2     = 0; // Disable transmission empty interrupt 

    // Enable write protection(Set protection) 
    SYSTEM.PRCR.WORD = 0xA500;      // Protect 

    /* SCR       - Serial Control Register
     *  b7  TIE  - Transmit Interrupt Enable     - A TXI interrupt request is disabled
     *  b6  RIE  - Receive Interrupt Enable      - RXI and ERI interrupt requests are disabled
     *  b5  TE   - Transmit Enable               - Serial transmission is disabled
     *  b4  RE   - Receive Enable                - Serial reception is disabled
     *  b2  TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled*/
    SCI2.SCR.BYTE = 0x00;

    while (0x00 != (SCI2.SCR.BYTE & 0xF0)) {
        // Confirm that bit is actually 0 
    }

    // Set the I/O port functions 

    // General-purpose port P13:TXD2, P12:RXD2 
    PORT1.PODR.BIT.B3 = 1; // TXD 
    PORT1.PODR.BIT.B5 = 1; // RTS=Disable 
    // Set port direction - TXDn is output port, RXDn is input port(Port I/O settings) 
    PORT1.PDR.BIT.B3  = 1; // Output TXD 
    PORT1.PDR.BIT.B2  = 0; // Input RXD 
    PORT1.PDR.BIT.B5  = 1; // Output RTS 
    PORT1.PDR.BIT.B7  = 0; // Input CTS 

    // Set port mode - Use pin as general I/O port 
    PORT1.PMR.BIT.B3  = 0; // General-purpose IO port setting 
    PORT1.PMR.BIT.B2  = 0; // General-purpose IO port setting 
    PORT1.PMR.BIT.B5  = 0; // General-purpose IO port setting 
    PORT1.PMR.BIT.B7  = 0; // General-purpose IO port setting 

    /* PWPR            - Write-Protect Register(Write protect register)
     *  b7  B0WI       - PFSWE Bit Write Disable   - PFSWE disable
     *  b6  PFSWE      - PFS Register Write Enable - PFS enable
     *  b5:b0 Reserved - These bits are read as 0. The write value should be 0.*/
    MPC.PWPR.BIT.B0WI   = 0; // Set to 0 first 
    MPC.PWPR.BIT.PFSWE  = 1; // Set to 1 later 

    /* PFS - Pin Function Control Register(Pin function register setting)
     *  b3:b0 PSEL - Pin Function Select - RXDn, TXDn*/
    MPC.P13PFS.BYTE = 0x0A;  // assign I/O pin to SCI0 TxD3 
    MPC.P12PFS.BYTE = 0x0A;  // assign I/O pin to SCI0 RxD3 
    MPC.P15PFS.BYTE = 0x00;  // assign I/O pin to Port 
    MPC.P17PFS.BYTE = 0x00;  // assign I/O pin to Port 
    // Apply write protection 
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI  = 1;

    // Use pin as I/O port for peripheral functions(IO pin function setting) 
    PORT1.PMR.BIT.B3   = 1; // Peripheral function settings 
    PORT1.PMR.BIT.B2   = 1; // Peripheral function settings 

    /* Initialization of SCI
     * Disable all module clock stop mode*/
    SYSTEM.MSTPCRA.BIT.ACSE = 0;
    // Release of SCI2 module stop state 
    MSTP_SCI2 = 0;

    // Select an On-chip baud rate generator to the clock source 
    SCI2.SCR.BIT.CKE = 0;

    /* SMR        - Serial Mode Register
     *  b7  CM    - Communications Mode  - Asynchronous mode
     *  b6  CHR   - Character Length     - Selects 8 bits as the data length
     *  b5  PE    - Parity Enable        - When transmitting : Parity bit addition is not performed
     *                                        When receiving : Parity bit checking is not performed
     *  b3  STOP  - Stop Bit Length      - 2 stop bits
     *  b2  MP    - Multi-Processor Mode - Multi-processor communications function is disabled
     *  b1:b0 CKS - Clock Select         - PCLK clock (n = 0)*/
    SCI2.SMR.BYTE = 0x08;

    /* SCMR            - Smart Card Mode Register
     *  b6:b4 Reserved - The write value should be 1.
     *  b3  SDIR       - Transmitted/Received Data Transfer Direction - Transfer with LSB-first
     *  b2  SINV       - Transmitted/Received Data Invert             - TDR contents are transmitted as they are.
     *                                                                  Receive data is stored as it is in RDR.
     *  b1  Reserved   - The write value should be 1.
     *  b0  SMIF       - Smart Card Interface Mode Select             - Serial communications interface mode*/
    SCI2.SCMR.BYTE = 0xF2;

    /* SEMR            - Serial Extended Mode Register
     *  b7:b6 Reserved - The write value should be 0.
     *  b5  NFEN       - Digital Noise Filter Function Enable - Noise cancellation function
     *                                                          for the RXDn input signal is disabled.
     *  b4  ABCS       - Asynchronous Mode Base Clock Select  - Selects 16 base clock cycles for 1-bit period
     *  b3:b1 Reserved - The write value should be 0.*/
    SCI2.SEMR.BYTE = 0x00;

    /* Set data transfer format in Serial Mode Register (SMR)
     *  -Asynchronous Mode`
     *  -8 bits
     *  -no parity
     *  -1 stop bit
     *  -PCLK clock (n = 0)*/
    SCI2.SMR.BYTE = 0x00; // 0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64 

    /* BRR - Bit Rate Register
     *  Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04*/
    if (stoplen == 1) {
        SCI2.SMR.BIT.STOP = 1;
    } else {
        SCI2.SMR.BIT.STOP = 0;
    }

    if (parity == 0) {
        SCI2.SMR.BIT.PE = 0;
    } else if (parity == 1) { //Odd parity 
        SCI2.SMR.BIT.PE = 1;
        SCI2.SMR.BIT.PM = 1;
    } else if (parity == 2) { // Even parity 
        SCI2.SMR.BIT.PE = 1;
        SCI2.SMR.BIT.PM = 0;
    }

    if (datalen == 7) { // 7bit length 
        SCI2.SMR.BIT.CHR = 1;
    } else if (datalen == 8) { // 8bit length 
        SCI2.SMR.BIT.CHR = 0;
    }

    /* Set baud rate to 115200
     *  N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
     *  N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
     *  N = 12*/
    SCI2.BRR = 48000000 / ((64/2) * bps) - 1;

    // SCI interrupt priority setting 
    ICU.IPR[IPR_SCI2_].BIT.IPR = 1; // Interrupt level setting 

    // SCI2 interrupt setting 
    ICU.IER[IER_SCI2_RXI2].BIT.IEN_SCI2_RXI2 = 1; // Receive interrupt 
    ICU.IER[IER_SCI2_TXI2].BIT.IEN_SCI2_TXI2 = 1; // Transmission completion interrupt 
    ICU.IER[IER_SCI2_TEI2].BIT.IEN_SCI2_TEI2 = 1; // Transmit empty interrupt 

    // Interrupt flag clear 
    ICU.IR[IR_SCI2_RXI2].BIT.IR = 0;
    ICU.IR[IR_SCI2_TXI2].BIT.IR = 0;
    ICU.IR[IR_SCI2_TEI2].BIT.IR = 0;

    // GROUP12 interrupt setting 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 interrupt enable 
    ICU.GEN[GEN_SCI2_ERI2].BIT.EN_SCI2_ERI2      = 1; // Group 12 SCI2 receive error interrupt enabled 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR             = 1; // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR                = 0; // Group 12 interrupt flag clear 

    SCI2.SCR.BIT.RIE = 1;
    SCI2.SCR.BIT.RE  = 1;

    PORT1.PODR.BIT.B5 = 0;  // RTS=Enable 

#endif //SCI2_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * sci3_init
 * 
 *  Function description
 *   SCI3 initialization
 *           Port        SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   SCI3    P23         TXD3                             <RS-232C> COM3
 *           P25         RXD3                             <RS-232C> COM3
 *           P22         nRTS3                            <RS-232C> COM3
 *           P24         nCTS3                            <RS-232C> COM3
 *           P56         nEXRES                           </RESET> External module reset signal
 *  
 *  Argument
 *      bps      Communication speed 300 - 115200
 *      datalen  Data lentgh 7,8
 *      stoplen  Stop lentgh 1,2
 *      parity   Parity 0=none / 1=odd / 2=even
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void sci3_init(long bps, int datalen, int stoplen, int parity)
{
    SCI_MODULE *com = &sci_com[3];
    memset(com, 0, sizeof(SCI_MODULE));

#ifdef      SCI3_ACTIVATE

    SYSTEM.PRCR.WORD    = 0xA502;   // Unprotect 
    MSTP_SCI3           = 0;        // SCI module stop release 

    // Disable SCI interrupt request 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0; // Group 12 interrupt disabled 
    ICU.IER[IER_SCI3_RXI3].BIT.IEN_SCI3_RXI3     = 0; // Disable receive interrupt 
    ICU.IER[IER_SCI3_TXI3].BIT.IEN_SCI3_TXI3     = 0; // Disable transmission end interrupt 
    ICU.IER[IER_SCI3_TEI3].BIT.IEN_SCI3_TEI3     = 0; // Disable transmission empty interrupt 

    // Enable write protection(Set protection) 
    SYSTEM.PRCR.WORD = 0xA500;      // Protect 

    /* SCR      - Serial Control Register
     * b7  TIE  - Transmit Interrupt Enable     - A TXI interrupt request is disabled
     * b6  RIE  - Receive Interrupt Enable      - RXI and ERI interrupt requests are disabled
     * b5  TE   - Transmit Enable               - Serial transmission is disabled
     * b4  RE   - Receive Enable                - Serial reception is disabled
     * b2  TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled*/
    SCI3.SCR.BYTE = 0x00;

    while (0x00 != (SCI3.SCR.BYTE & 0xF0)) {
        // Confirm that bit is actually 0 
    }

    // Set the I/O port functions 

    // General-purpose port, P23:TXD3, P24:RXD3 
    PORT2.PODR.BIT.B3 = 1; // TXD 
    PORT2.PODR.BIT.B2 = 1; // RTS=Disable 
    // Set port direction - TXDn is output port, nCTS/RXDn is input port(Port I/O settings) 
    PORT2.PDR.BIT.B3  = 1; // Output TXD 
    PORT2.PDR.BIT.B5  = 0; // Input RXD 
    PORT2.PDR.BIT.B2  = 1; // Output RTS 
    PORT2.PDR.BIT.B4  = 0; // Input CTS 

    // Set port mode - Use pin as general I/O port 
    PORT2.PMR.BIT.B3  = 0; // General-purpose IO port setting 
    PORT2.PMR.BIT.B5  = 0; // General-purpose IO port setting 
    PORT2.PMR.BIT.B2  = 0; // General-purpose IO port setting 
    PORT2.PMR.BIT.B4  = 0; // General-purpose IO port setting 

    /* PWPR           - Write-Protect Register(Write protect register)
     * b7  B0WI       - PFSWE Bit Write Disable   - PFSWE disable
     * b6  PFSWE      - PFS Register Write Enable - PFS enable
     * b5:b0 Reserved - These bits are read as 0. The write value should be 0.*/
    MPC.PWPR.BIT.B0WI  = 0; // Set to 0 first 
    MPC.PWPR.BIT.PFSWE = 1; // Set to 1 later 

    /* PFS - Pin Function Control Register(Pin function register setting)
     * b3:b0 PSEL - Pin Function Select - RXDn, TXDn*/
    MPC.P23PFS.BYTE = 0x0A; // assign I/O pin to SCI3 TxD3 
    MPC.P25PFS.BYTE = 0x0A; // assign I/O pin to SCI3 RxD3 
    MPC.P22PFS.BYTE = 0x00; // assign I/O pin to Port 
    MPC.P24PFS.BYTE = 0x00; // assign I/O pin to Port 
    // Apply write protection 
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI  = 1;

    // Use pin as I/O port for peripheral functions(IO pin function setting) 
    PORT2.PMR.BIT.B3 = 1; // Peripheral function settings 
    PORT2.PMR.BIT.B5 = 1; // Peripheral function settings 

    // Initialization of SCI 

    // Select an On-chip baud rate generator to the clock source 
    SCI3.SCR.BIT.CKE = 0;

    /* SMR       - Serial Mode Register
     * b7  CM    - Communications Mode  - Asynchronous mode
     * b6  CHR   - Character Length     - Selects 8 bits as the data length
     * b5  PE    - Parity Enable        - When transmitting : Parity bit addition is not performed
     *                                       When receiving : Parity bit checking is not performed
     * b3  STOP  - Stop Bit Length      - 2 stop bits
     * b2  MP    - Multi-Processor Mode - Multi-processor communications function is disabled
     * b1:b0 CKS - Clock Select         - PCLK clock (n = 0)*/
    SCI3.SMR.BYTE = 0x08;

    /* SCMR           - Smart Card Mode Register
     * b6:b4 Reserved - The write value should be 1.
     * b3  SDIR       - Transmitted/Received Data Transfer Direction - Transfer with LSB-first
     * b2  SINV       - Transmitted/Received Data Invert - TDR contents are transmitted as they are.
     *                                                     Receive data is stored as it is in RDR.
     * b1  Reserved   - The write value should be 1.
     * b0  SMIF       - Smart Card Interface Mode Select - Serial communications interface mode*/
    SCI3.SCMR.BYTE = 0xF2;

    /* SEMR           - Serial Extended Mode Register
     * b7:b6 Reserved - The write value should be 0.
     * b5  NFEN       - Digital Noise Filter Function Enable - Noise cancellation function
     *                                                         for the RXDn input signal is disabled.
     * b4  ABCS       - Asynchronous Mode Base Clock Select - Selects 16 base clock cycles for 1-bit period
     * b3:b1 Reserved - The write value should be 0.*/
    SCI3.SEMR.BYTE = 0x00;

    /* Set data transfer format in Serial Mode Register (SMR)
     *  -Asynchronous Mode`
     *  -8 bits
     *  -no parity
     *  -1 stop bit
     *  -PCLK clock (n = 0)*/
    SCI3.SMR.BYTE = 0x00; // 0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64 

    /* BRR - Bit Rate Register
     * Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04*/
    if (stoplen == 1) {
        SCI3.SMR.BIT.STOP = 1;
    } else {
        SCI3.SMR.BIT.STOP = 0;
    }

    if (parity == 0) {
        SCI3.SMR.BIT.PE = 0;
    } else if (parity == 1) { //Odd parity 
        SCI3.SMR.BIT.PE = 1;
        SCI3.SMR.BIT.PM = 1;
    } else if (parity == 2) { // Even parity 
        SCI3.SMR.BIT.PE = 1;
        SCI3.SMR.BIT.PM = 0;
    }

    if (datalen == 7) { // 7bit lentgh 
        SCI3.SMR.BIT.CHR = 1;
    } else if (datalen == 8) { // 8bit lentgh 
        SCI3.SMR.BIT.CHR = 0;
    }

    /* Set baud rate to 115200
     * N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
     * N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
     * N = 12*/
    SCI3.BRR = 48000000 / ((64/2) * bps) - 1;

    // SCI interrupt priority setting 
    ICU.IPR[IPR_SCI3_].BIT.IPR = 1; // Interrupt level setting 

    // SCI3 interrupt setting 
    ICU.IER[IER_SCI3_RXI3].BIT.IEN_SCI3_RXI3 = 1; // Receive interrupt 
    ICU.IER[IER_SCI3_TXI3].BIT.IEN_SCI3_TXI3 = 1; // Transmission completion interrupt 
    ICU.IER[IER_SCI3_TEI3].BIT.IEN_SCI3_TEI3 = 1; // Transmit empty interrupt 

    // Clear interrupt flag 
    ICU.IR[IR_SCI3_RXI3].BIT.IR = 0;
    ICU.IR[IR_SCI3_TXI3].BIT.IR = 0;
    ICU.IR[IR_SCI3_TEI3].BIT.IR = 0;

    // GROUP12 interrupt setting 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 interrupt enable 
    ICU.GEN[GEN_SCI3_ERI3].BIT.EN_SCI3_ERI3      = 1; // Group 12 SCI3 receive error interrupt enabled 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR             = 1; // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR                = 0; // Group 12 interrupt flag clear 

    SCI3.SCR.BIT.RIE = 1;
    SCI3.SCR.BIT.RE  = 1;

    PORT2.PODR.BIT.B2 = 0; // RTS=Enable 

#endif //SCI3_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * spi5_init
 * 
 *  Function description
 *   Initialize SCI5 in simple SPI mode
 *           Port        SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   SCI5    PC3         TXD5                  SMOSI5     <SPI>  External extension
 *           PC2         RXD5                  SMISO5     <SPI>  External extension
 *           PC4                               SCK5       <SPI>  External extension
 *           PC5                               SS0        <SPI>  External extension
 *           PC6                               SS1        <SPI>  External extension
 * 
 *  Argument
 *      bps  Communication speed
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void spi5_init(long bps)
{
    SCI_MODULE *com = &sci_com[5];
    memset(com, 0, sizeof(SCI_MODULE));

#ifdef      SCI5_ACTIVATE

    SYSTEM.PRCR.WORD    = 0xA502;   // Unprotect 
    MSTP_SCI5           = 0;        // SCI module stop release 

    // Disable SCI interrupt request 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0; // Group 12 interrupt disabled 
    ICU.IER[IER_SCI5_RXI5].BIT.IEN_SCI5_RXI5     = 0; // Disable receive interrupt 
    ICU.IER[IER_SCI5_TXI5].BIT.IEN_SCI5_TXI5     = 0; // Disable transmission end interrupt 
    ICU.IER[IER_SCI5_TEI5].BIT.IEN_SCI5_TEI5     = 0; // Disable transmission empty interrupt 

    // Enable write protection(Set protection) 
    SYSTEM.PRCR.WORD = 0xA500;      // Protect 

    /* SCR      - Serial Control Register
     * b7  TIE  - Transmit Interrupt Enable     - A TXI interrupt request is disabled
     * b6  RIE  - Receive Interrupt Enable      - RXI and ERI interrupt requests are disabled
     * b5  TE   - Transmit Enable               - Serial transmission is disabled
     * b4  RE   - Receive Enable                - Serial reception is disabled
     * b2  TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled*/
    SCI5.SCR.BYTE = 0x00;

    while (0x00 != (SCI5.SCR.BYTE & 0xF0)) {
        // Confirm that bit is actually 0 
    }

    // Set the I/O port functions 

    // General-purpose port PC3:TXD5, PC2:RXD5 
    PORTC.PODR.BIT.B3 = 1; // SMOSI5 
    PORTC.PODR.BIT.B4 = 1; // SCK5 
    PORTC.PODR.BIT.B5 = 1; // SS0=Disable 
    PORTC.PODR.BIT.B6 = 1; // SS1=Disable 
    // Set port direction - TXDn is output port, RXDn is input port(Port I/O settings) 
    PORTC.PDR.BIT.B3  = 1; // Output MOSI 
    PORTC.PDR.BIT.B2  = 0; // Input MISO 
    PORTC.PDR.BIT.B4  = 1; // Output SCK5 
    PORTC.PDR.BIT.B5  = 1; // Output SS0 
    PORTC.PDR.BIT.B6  = 1; // Output SS1 

    // Set port mode - Use pin as general I/O port 
    PORTC.PMR.BIT.B3  = 0; // General-purpose IO port setting 
    PORTC.PMR.BIT.B2  = 0; // General-purpose IO port setting 
    PORTC.PMR.BIT.B4  = 0; // General-purpose IO port setting 
    PORTC.PMR.BIT.B5  = 0; // General-purpose IO port setting 
    PORTC.PMR.BIT.B6  = 0; // General-purpose IO port setting 

    /* PWPR           - Write-Protect Register(Write protect register)
     * b7  B0WI       - PFSWE Bit Write Disable   - PFSWE disable
     * b6  PFSWE      - PFS Register Write Enable - PFS enable
     * b5:b0 Reserved - These bits are read as 0. The write value should be 0.*/
    MPC.PWPR.BIT.B0WI  = 0; // Set to 0 first 
    MPC.PWPR.BIT.PFSWE = 1; // Set to 1 later 

    /* PFS - Pin Function Control Register(Pin function register setting)
     * b3:b0 PSEL - Pin Function Select - RXDn, TXDn*/
    MPC.PC3PFS.BYTE = 0x0A; // assign I/O pin to SCI5 TXD5 
    MPC.PC2PFS.BYTE = 0x0A; // assign I/O pin to SCI5 RXD5 
    MPC.PC4PFS.BYTE = 0x0A; // assign I/O pin to SCI5 SCK5 
    MPC.PC5PFS.BYTE = 0x00; // assign I/O pin to port 
    MPC.PC6PFS.BYTE = 0x00; // assign I/O pin to port 
    // Apply write protection 
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI  = 1;

    // Use pin as I/O port for peripheral functions(IO pin function setting) 
    PORTC.PMR.BIT.B3 = 1; // Peripheral function settings 
    PORTC.PMR.BIT.B2 = 1; // Peripheral function settings 
    PORTC.PMR.BIT.B4 = 1; // Peripheral function settings 

    // Initialization of SCI 

    // Select an On-chip baud rate generator to the clock source 
    SCI5.SCR.BIT.CKE = 0;

    /* SMR      - Serial Mode Register
     * b7  CM   - Communications Mode   - Asynchronous mode
     * b6  CHR  - Character Length      - Selects 8 bits as the data length
     * b5  PE   - Parity Enable         - When transmitting : Parity bit addition is not performed
     *                                       When receiving : Parity bit checking is not performed
     * b3  STOP  - Stop Bit Length      - 2 stop bits
     * b2  MP    - Multi-Processor Mode - Multi-processor communications function is disabled
     * b1:b0 CKS - Clock Select         - PCLK clock (n = 0)*/
    SCI5.SMR.BYTE = 0x88;

    /* SCMR           - Smart Card Mode Register
     * b6:b4 Reserved - The write value should be 1.
     * b3  SDIR       - Transmitted/Received Data Transfer Direction - Transfer with LSB-first
     * b2  SINV       - Transmitted/Received Data Invert             - TDR contents are transmitted as they are.
     *                                                                 Receive data is stored as it is in RDR.
     * b1  Reserved   - The write value should be 1.
     * b0  SMIF       - Smart Card Interface Mode Select             - Serial communications interface mode*/
    SCI5.SCMR.BYTE = 0xF2;

    /* SEMR           - Serial Extended Mode Register
     * b7:b6 Reserved - The write value should be 0.
     * b5  NFEN       - Digital Noise Filter Function Enable - Noise cancellation function
     *                                                         for the RXDn input signal is disabled.
     * b4  ABCS       - Asynchronous Mode Base Clock Select  - Selects 16 base clock cycles for 1-bit period
     * b3:b1 Reserved - The write value should be 0.*/
    SCI5.SEMR.BYTE = 0x00;

    /* Set data transfer format in Serial Mode Register (SMR)* /
     *  -Asynchronous Mode`
     *  -8 bits
     *  -no parity
     *  -1 stop bit
     *  -PCLK clock (n = 0)*/
    SCI5.SMR.BYTE = 0x00; // 0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64 

    /* BRR - Bit Rate Register
     * Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04
     * Set baud rate to 115200
     * N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
     * N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
     * N = 12*/
    SCI5.BRR = 48000000 / ((64/2) * bps) - 1;

    // SCI interrupt priority setting 
    ICU.IPR[IPR_SCI5_].BIT.IPR = 1; // Interrupt level setting 

    // SCI4 interrupt setting 
    ICU.IER[IER_SCI5_RXI5].BIT.IEN_SCI5_RXI5 = 1; // Receive interrupt 
    ICU.IER[IER_SCI5_TXI5].BIT.IEN_SCI5_TXI5 = 1; // Transmission completion interrupt 
    ICU.IER[IER_SCI5_TEI5].BIT.IEN_SCI5_TEI5 = 1; // Transmit empty interrupt 

    // Interrupt flag clear 
    ICU.IR[IR_SCI5_RXI5].BIT.IR = 0;
    ICU.IR[IR_SCI5_TXI5].BIT.IR = 0;
    ICU.IR[IR_SCI5_TEI5].BIT.IR = 0;

    // GROUP12 interrupt setting 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 interrupt enable 
    ICU.GEN[GEN_SCI5_ERI5].BIT.EN_SCI5_ERI5      = 1; // Group 12 SCI5 receive error interrupt enabled 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR             = 1; // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR                = 0; // Group 12 interrupt flag clear 

    // Force to simple SPI mode 
    SCI5.SCMR.BIT.SMIF  = 0; // 
    SCI5.SIMR1.BIT.IICM = 0; // 
    SCI5.SMR.BIT.CM     = 1; // Synchronous mode 
    SCI5.SPMR.BIT.SSe = 0; // Single master 
    SCI5.SCR.BIT.RIE    = 1;
    SCI5.SCR.BIT.RE     = 1;

#endif //SCI5_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * sci6_init
 * 
 *  Function description
 *   SCI6 initialization
 *           Port        SCI        I2C        SPI        Apply
 *   ----------------------------------------------------------------------------
 *   SCI6    P00         TXD6                            <TTL>  COM6
 *           P01         RXD6                            <TTL>  COM6
 *           P02         nRTS6                           <TTL>  COM6
 *           PJ3         nCTS6                           <TTL>  COM6
 * 
 *  Argument
 *      bps      Communication speed 300 to 115200
 *      datalen  Data lentgh 7,8
 *      stoplen  Stop lentgh 1,2
 *      parity   Parity 0=none / 1=odd / 2=even
 * 
 *  Return
 *     None
 * ----------------------------------------------------------------------------------------*/
void sci6_init(long bps, int datalen, int stoplen, int parity)
{
    SCI_MODULE *com = &sci_com[6];
    memset(com, 0, sizeof(SCI_MODULE));

#ifdef      SCI6_ACTIVATE

    SYSTEM.PRCR.WORD    = 0xA502;   // Unprotect 
    MSTP_SCI6           = 0;        // SCI module stop release 

    // Disable SCI interrupt request 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0; // Group 12 interrupt disabled 
    ICU.IER[IER_SCI6_RXI6].BIT.IEN_SCI6_RXI6     = 0; // Disable receive interrupt 
    ICU.IER[IER_SCI6_TXI6].BIT.IEN_SCI6_TXI6     = 0; // Disable transmission end interrupt 
    ICU.IER[IER_SCI6_TEI6].BIT.IEN_SCI6_TEI6     = 0; // Disable transmission empty interrupt 

    // Enable write protection(Set protection) 
    SYSTEM.PRCR.WORD = 0xA500;      // Protect 

    /* SCR      - Serial Control Register
     * b7  TIE  - Transmit Interrupt Enable     - A TXI interrupt request is disabled
     * b6  RIE  - Receive Interrupt Enable      - RXI and ERI interrupt requests are disabled
     * b5  TE   - Transmit Enable               - Serial transmission is Disable transmission empty interruptdisabled
     * b4  RE   - Receive Enable                - Serial reception is disabled
     * b2  TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled*/
    SCI6.SCR.BYTE = 0x00;

    while (0x00 != (SCI6.SCR.BYTE & 0xF0)) {
        // Confirm that bit is actually 0 
    }

    /* Set the I/O port functions
     * General-purpose port P00:TXD6, P01:RXD6*/
    PORT0.PODR.BIT.B0 = 1; // TXD 
    PORT0.PODR.BIT.B2 = 1; // RTS=Disable 
    // Set port direction - TXDn is output port, RXDn is input port (Port I/O settings) 
    PORT0.PDR.BIT.B0  = 1; // Output TXD 
    PORT0.PDR.BIT.B1  = 0; // Input RXD 
    PORT0.PDR.BIT.B2  = 1; // Output RTS 
    PORTJ.PDR.BIT.B3  = 0; // Onput CTS 

    // Set port mode - Use pin as general I/O port 
    PORT0.PMR.BIT.B0  = 0; // General-purpose IO port setting 
    PORT0.PMR.BIT.B1  = 0; // General-purpose IO port setting 
    PORT0.PMR.BIT.B2  = 0; // General-purpose IO port setting 
    PORTJ.PMR.BIT.B3  = 0; // General-purpose IO port setting 

    /* PWPR           - Write-Protect Register(Write protect register)
     * b7  B0WI       - PFSWE Bit Write Disable   - PFSWE disable
     * b6  PFSWE      - PFS Register Write Enable - PFS enable
     * b5:b0 Reserved - These bits are read as 0. The write value should be 0.*/
    MPC.PWPR.BIT.B0WI  = 0; // Set to 0 first 
    MPC.PWPR.BIT.PFSWE = 1; // Set to 1 later 

    /* PFS - Pin Function Control Register(Pin function register setting)
     * b3:b0 PSEL - Pin Function Select - RXDn, TXDn*/
    MPC.P00PFS.BYTE = 0x0A; // assign I/O pin to SCI6 TXD6 
    MPC.P01PFS.BYTE = 0x0A; // assign I/O pin to SCI6 RXD6 
    MPC.P02PFS.BYTE = 0x00; // assign I/O pin to port 
    MPC.PJ3PFS.BYTE = 0x00; // assign I/O pin to port 
    // Apply write protection 
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI  = 1;

    // Use pin as I/O port for peripheral functions(IO pin function setting) 
    PORT0.PMR.BIT.B0 = 1; // Peripheral function settings 
    PORT0.PMR.BIT.B1 = 1; // Peripheral function settings 
    // Initialization of SCI 

    // Select an On-chip baud rate generator to the clock source 
    SCI6.SCR.BIT.CKE = 0;

    /* SMR       - Serial Mode Register
     * b7  CM    - Communications Mode  - Asynchronous mode
     * b6  CHR   - Character Length     - Selects 8 bits as the data length
     * b5  PE    - Parity Enable        - When transmitting : Parity bit addition is not performed
     *                                       When receiving : Parity bit checking is not performed
     * b3  STOP  - Stop Bit Length      - 2 stop bits
     * b2  MP    - Multi-Processor Mode - Multi-processor communications function is disabled
     * b1:b0 CKS - Clock Select         - PCLK clock (n = 0)*/
    SCI6.SMR.BYTE = 0x08;

    /* SCMR           - Smart Card Mode Register
     * b6:b4 Reserved - The write value should be 1.
     * b3  SDIR       - Transmitted/Received Data Transfer Direction - Transfer with LSB-first
     * b2  SINV       - Transmitted/Received Data Invert - TDR contents are transmitted as they are.
     *                                                     Receive data is stored as it is in RDR.
     * b1  Reserved   - The write value should be 1.
     * b0  SMIF       - Smart Card Interface Mode Select - Serial communications interface mode*/
    SCI6.SCMR.BYTE = 0xF2;

    /* SEMR           - Serial Extended Mode Register
     * b7:b6 Reserved - The write value should be 0.
     * b5  NFEN       - Digital Noise Filter Function Enable - Noise cancellation function
     *                                                         for the RXDn input signal is disabled.
     * b4  ABCS       - Asynchronous Mode Base Clock Select  - Selects 16 base clock cycles for 1-bit period
     * b3:b1 Reserved - The write value should be 0.*/
    SCI6.SEMR.BYTE = 0x00;

    /* Set data transfer format in Serial Mode Register (SMR)
     *  -Asynchronous Mode`
     *  -8 bits
     *  -no parity
     *  -1 stop bit
     *  -PCLK clock (n = 0)*/
    SCI6.SMR.BYTE = 0x00; // 0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64 

    /* BRR - Bit Rate Register
     * Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04*/
    if (stoplen == 1) {
        SCI6.SMR.BIT.STOP = 1;
    } else {
        SCI6.SMR.BIT.STOP = 0;
    }

    if (parity == 0) {
        SCI6.SMR.BIT.PE = 0;
    } else if (parity == 1) { //Odd parity 
        SCI6.SMR.BIT.PE = 1;
        SCI6.SMR.BIT.PM = 1;
    } else if (parity == 2) { // Even parity 
        SCI6.SMR.BIT.PE = 1;
        SCI6.SMR.BIT.PM = 0;
    }

    if (datalen == 7) { // 7bit length 
        SCI6.SMR.BIT.CHR = 1;
    } else if (datalen == 8) { // 8bit length 
        SCI6.SMR.BIT.CHR = 0;
    }

    /* Set baud rate to 115200
     * N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
     * N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
     * N = 12*/
    SCI6.BRR = 48000000 / ((64/2) * bps) - 1;

    // SCI interrupt priority setting 
    ICU.IPR[IPR_SCI6_].BIT.IPR = 1; // Interrupt level setting 

    // SCI6 interrupt setting 
    ICU.IER[IER_SCI6_RXI6].BIT.IEN_SCI6_RXI6 = 1; // Receive interrupt 
    ICU.IER[IER_SCI6_TXI6].BIT.IEN_SCI6_TXI6 = 1; // Transmission completion interrupt 
    ICU.IER[IER_SCI6_TEI6].BIT.IEN_SCI6_TEI6 = 1; // Transmit empty interrupt 

    // Interrupt flag clear 
    ICU.IR[IR_SCI6_RXI6].BIT.IR = 0;
    ICU.IR[IR_SCI6_TXI6].BIT.IR = 0;
    ICU.IR[IR_SCI6_TEI6].BIT.IR = 0;

    // GROUP12 interrupt setting 
    ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1; // Group 12 interrupt enable 
    ICU.GEN[GEN_SCI6_ERI6].BIT.EN_SCI6_ERI6      = 1; // Group 12 SCI6 reception error interrupt enabled 
    ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR             = 1; // Group 12 interrupt level setting 
    ICU.IR[IR_ICU_GROUPL0].BIT.IR                = 0; // Group 12 interrupt flag clear 

    SCI6.SCR.BIT.RIE = 1;
    SCI6.SCR.BIT.RE  = 1;

    PORT0.PODR.BIT.B2 = 0; // RTS=Enable 

#endif //SCI6_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * sci_putcheck
 *
 *  Function description
 *      Find free space in the SCI transmission buffer
 * 
 *  Argument
 *     ch   SCI channel
 * 
 *  Return
 *     int   Number of free bytes
 * ----------------------------------------------------------------------------------------*/
int sci_putcheck(int ch)
{
    SCI_MODULE *com = &sci_com[ch];
    int sz = (com->txwp - com->txrp);
    if (sz < 0) {
        sz += BUFSIZE;
    }
    return (BUFSIZE - sz);
}

/* ----------------------------------------------------------------------------------------
 * sci_txbytes
 * 
 *  Function description
 *      Calculate the number of untransmitted bytes in the SCI transmit buffer
 * 
 *  Argument
 *      ch   SCI channel
 *  
 *  Return
 *      int   Number of unsent bytes
 * ----------------------------------------------------------------------------------------*/
int sci_txbytes(int ch)
{
    SCI_MODULE *com = &sci_com[ch];
    int sz = (com->txwp - com->txrp);
    if (sz < 0) {
        sz += BUFSIZE;
    }
    return sz;
}

/* ----------------------------------------------------------------------------------------
 * sci(n)_putb
 * 
 *  Function description
 *      Send a data string to SCI
 * 
 *  Argument
 *      *buf  Transmission bufffer
 *      size  Number of transmit bytes
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
 
/* ----------------------------------------------------------------------------------------
 * SCI0 SEND
 * ---------------------------------------------------------------------------------------- */
void sci0_putb(unsigned char *buf, int size)
{
    int i, ch = 0;
    SCI_MODULE *com = &sci_com[ch];
    while (size > 0) {
#ifdef      SCI0_ACTIVATE
#ifdef  SCI0_FLOW
        if (SCI0_CTS_PORT != 0) { // CTS=Disable 
            if (sci_putcheck(ch) < 2) {
                return;
            }
        }
#endif // ifdef  SCI0_FLOW
        while (sci_putcheck(ch) < 2) {  // Wait until buffer is free 
            if (SCI0.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI0_FLOW
                if (SCI0_CTS_PORT == 0) { // CTS=Enable 
#endif
                SCI0.SCR.BIT.TIE = 1; // Interrupt enable 
                SCI0.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI0_FLOW
            }
#endif
            }
        }
#endif // ifdef      SCI0_ACTIVATE
        com->txbuf[com->txwp++] = *buf++;
        size--;
        if (com->txwp >= BUFSIZE) {
            com->txwp = 0;
        }
#ifdef      SCI0_ACTIVATE
        if (SCI0.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI0_FLOW
            if (SCI0_CTS_PORT == 0) { // CTS=Enable 
#endif
            SCI0.SCR.BIT.TIE = 1; // Interrupt enable 
            SCI0.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI0_FLOW
        }
#endif
        }
#endif // ifdef      SCI0_ACTIVATE
    }
#ifdef      SCI0_ACTIVATE
    if (SCI0.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI0_FLOW
        if (SCI0_CTS_PORT == 0) { // CTS=Enable 
#endif
        SCI0.SCR.BIT.TIE = 1; // Interrupt enable 
        SCI0.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI0_FLOW
    }
#endif
    }
#endif // ifdef      SCI0_ACTIVATE
}
/* ----------------------------------------------------------------------------------------
 * SCI1 SEND
 * ---------------------------------------------------------------------------------------- */
void sci1_putb(unsigned char *buf, int size)
{
    int i, ch = 1;
    SCI_MODULE *com = &sci_com[ch];
    while (size > 0) {
#ifdef      SCI1_ACTIVATE
#ifdef  SCI1_FLOW
        if (SCI1_CTS_PORT != 0) { // CTS=Disable 
            if (sci_putcheck(ch) < 2) {
                return;
            }
        }
#endif // ifdef  SCI1_FLOW
        while (sci_putcheck(ch) < 2) {  // Wait until buffer is free 
            if (SCI1.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI1_FLOW
                if (SCI1_CTS_PORT == 0) { // CTS=Enable 
#endif
                SCI1.SCR.BIT.TIE = 1; // Interrupt enable 
                SCI1.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI1_FLOW
            }
#endif
            }
        }
#endif // ifdef      SCI1_ACTIVATE
        com->txbuf[com->txwp++] = *buf++;
        size--;
        if (com->txwp >= BUFSIZE) {
            com->txwp = 0;
        }
#ifdef      SCI1_ACTIVATE
        if (SCI1.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI1_FLOW
            if (SCI1_CTS_PORT == 0) { // CTS=Enable 
#endif
            SCI1.SCR.BIT.TIE = 1; // Interrupt enable 
            SCI1.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI1_FLOW
        }
#endif
        }
#endif // ifdef      SCI1_ACTIVATE
    }
#ifdef      SCI1_ACTIVATE
    if (SCI1.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI1_FLOW
        if (SCI1_CTS_PORT == 0) { // CTS=Enable 
#endif
        SCI1.SCR.BIT.TIE = 1; // Interrupt enable 
        SCI1.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI1_FLOW
    }
#endif
    }
#endif // ifdef      SCI1_ACTIVATE
}
/* ----------------------------------------------------------------------------------------
 * SCI2 SEND
 * ---------------------------------------------------------------------------------------- */
void sci2_putb(unsigned char *buf, int size)
{
    int i, ch = 2;
    SCI_MODULE *com = &sci_com[ch];
    while (size > 0) {
#ifdef      SCI2_ACTIVATE
#ifdef  SCI2_FLOW
        if (SCI2_CTS_PORT != 0) { // CTS=Disable 
            if (sci_putcheck(ch) < 2) {
                return;
            }
        }
#endif // ifdef  SCI2_FLOW
        while (sci_putcheck(ch) < 2) {  // Wait until buffer is free 
            if (SCI2.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI2_FLOW
                if (SCI2_CTS_PORT == 0) { // CTS=Enable 
#endif
                SCI2.SCR.BIT.TIE = 1; // Interrupt enable 
                SCI2.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI2_FLOW
            }
#endif
            }
        }
#endif // ifdef      SCI2_ACTIVATE
        com->txbuf[com->txwp++] = *buf++;
        size--;
        if (com->txwp >= BUFSIZE) {
            com->txwp = 0;
        }
#ifdef      SCI2_ACTIVATE
        if (SCI2.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI2_FLOW
            if (SCI2_CTS_PORT == 0) { // CTS=Enable 
#endif
            SCI2.SCR.BIT.TIE = 1; // Interrupt enable 
            SCI2.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI2_FLOW
        }
#endif
        }
#endif // ifdef      SCI2_ACTIVATE
    }
#ifdef      SCI2_ACTIVATE
    if (SCI2.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI2_FLOW
        if (SCI2_CTS_PORT == 0) { // CTS=Enable 
#endif
        SCI2.SCR.BIT.TIE = 1; // Interrupt enable 
        SCI2.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI2_FLOW
    }
#endif
    }
#endif // ifdef      SCI2_ACTIVATE
}
/* ----------------------------------------------------------------------------------------
 * SCI3 SEND
 * ---------------------------------------------------------------------------------------- */
void sci3_putb(unsigned char *buf, int size)
{
    int i, ch = 3;
    SCI_MODULE *com = &sci_com[ch];
    while (size > 0) {
#ifdef      SCI3_ACTIVATE
#ifdef  SCI3_FLOW
        if (SCI3_CTS_PORT != 0) { // CTS=Disable 
            if (sci_putcheck(ch) < 2) {
                return;
            }
        }
#endif // ifdef  SCI3_FLOW
        while (sci_putcheck(ch) < 2) {    // Wait until buffer is free 
            if (SCI3.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI3_FLOW
                if (SCI3_CTS_PORT == 0) { // CTS=Enable 
#endif
                SCI3.SCR.BIT.TIE = 1; // Interrupt enable 
                SCI3.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI3_FLOW
            }
#endif
            }
        }
#endif // ifdef      SCI3_ACTIVATE
        com->txbuf[com->txwp++] = *buf++;
        size--;
        if (com->txwp >= BUFSIZE) {
            com->txwp = 0;
        }
#ifdef      SCI3_ACTIVATE
        if (SCI3.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI3_FLOW
            if (SCI3_CTS_PORT == 0) { // CTS=Enable 
#endif
            SCI3.SCR.BIT.TIE = 1; // Interrupt enable 
            SCI3.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI3_FLOW
        }
#endif
        }
#endif // ifdef      SCI3_ACTIVATE
    }
#ifdef      SCI3_ACTIVATE
    if (SCI3.SCR.BIT.TE == 0) {   // Transmission start processing 
#ifdef  SCI3_FLOW
        if (SCI3_CTS_PORT == 0) { // CTS=Enable 
#endif
        SCI3.SCR.BIT.TIE = 1; // Interrupt enable 
        SCI3.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI3_FLOW
    }
#endif
    }
#endif // ifdef      SCI3_ACTIVATE
}
/* ----------------------------------------------------------------------------------------
 * SCI5 SEND
 * ---------------------------------------------------------------------------------------- */
void sci5_putb(unsigned char *buf, int size)
{
    int i, ch = 5;
    SCI_MODULE *com = &sci_com[ch];
    while (size > 0) {
#ifdef SCI5_ACTIVATE
        while (sci_putcheck(ch) < 2) {  // Wait until buffer is free 
            if (SCI5.SCR.BIT.TE == 0) { // Transmission start processing 
                SCI5.SCR.BIT.TIE = 1;   // Interrupt enable 
                SCI5.SCR.BIT.TE  = 1;   // Transmission enable 
            }
        }
#endif // ifdef      SCI5_ACTIVATE
        com->txbuf[com->txwp++] = *buf++;
        size--;
        if (com->txwp >= BUFSIZE) {
            com->txwp = 0;
        }
#ifdef      SCI5_ACTIVATE
        if (SCI5.SCR.BIT.TE == 0) { // Transmission start processing 
            SCI5.SCR.BIT.TIE = 1;   // Interrupt enable 
            SCI5.SCR.BIT.TE  = 1;   // Transmission enable 
        }
#endif
    }
#ifdef      SCI5_ACTIVATE
    if (SCI5.SCR.BIT.TE == 0) { // Transmission start processing 
        SCI5.SCR.BIT.TIE = 1;   // Interrupt enable 
        SCI5.SCR.BIT.TE  = 1;   // Transmission enable 
    }
#endif
}
/* ----------------------------------------------------------------------------------------
 * SCI6 SEND
 * ---------------------------------------------------------------------------------------- */
void sci6_putb(unsigned char *buf, int size)
{
    int i, ch = 6;
    SCI_MODULE *com = &sci_com[ch];
    while (size > 0) {
#ifdef      SCI6_ACTIVATE
#ifdef  SCI6_FLOW
        if (SCI6_CTS_PORT != 0) { // CTS=Disable 
            if (sci_putcheck(ch) < 2) {
                return;
            }
        }
#endif // ifdef  SCI6_FLOW
        while (sci_putcheck(ch) < 2) {  // Wait until buffer is free 
            if (SCI6.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI6_FLOW
                if (SCI6_CTS_PORT == 0) { // CTS=Enable 
#endif
                SCI6.SCR.BIT.TIE = 1; // Interrupt enable 
                SCI6.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI6_FLOW
            }
#endif
            }
        }
#endif // ifdef      SCI6_ACTIVATE
        com->txbuf[com->txwp++] = *buf++;
        size--;
        if (com->txwp >= BUFSIZE) {
            com->txwp = 0;
        }
#ifdef      SCI6_ACTIVATE
        if (SCI6.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI6_FLOW
            if (SCI6_CTS_PORT == 0) { // CTS=Enable 
#endif
            SCI6.SCR.BIT.TIE = 1; // Interrupt enable 
            SCI6.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI6_FLOW
        }
#endif
        }
#endif // ifdef      SCI6_ACTIVATE
    }
#ifdef      SCI6_ACTIVATE
    if (SCI6.SCR.BIT.TE == 0) { // Transmission start processing 
#ifdef  SCI6_FLOW
        if (SCI6_CTS_PORT == 0) { // CTS=Enable 
#endif
        SCI6.SCR.BIT.TIE = 1; // Interrupt enable 
        SCI6.SCR.BIT.TE  = 1; // Transmission enable 
#ifdef  SCI6_FLOW
    }
#endif
    }
#endif // ifdef      SCI6_ACTIVATE
}

/* ----------------------------------------------------------------------------------------
 * sci_puts
 * 
 *  Function description
 *      Transmit string to SCI
 * 
 *  Argument
 *      ch    SCI channel
 *      *str  Sent string
 * 
 *  Return
 *      int   Number of free bytes
 * ----------------------------------------------------------------------------------------*/
void sci_puts(int ch, char *str)
{
    int len = 0;
    for (len = 0; str[len] != 0 && len < 256; len++) {
        ;
    }
    switch (ch) {
    case 0:
        sci0_putb((unsigned char *)str, len);
        break;
    case 1:
        sci1_putb((unsigned char *)str, len);
        break;
    case 2:
        sci2_putb((unsigned char *)str, len);
        break;
    case 3:
        sci3_putb((unsigned char *)str, len);
        break;
    case 5:
        sci5_putb((unsigned char *)str, len);
        break;
    case 6:
        sci6_putb((unsigned char *)str, len);
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * sci_putb
 *
 *  Function description
 *      Transmit byte sequence to SCI
 * 
 *  Argument
 *      ch    SCI channel
 *      *buf  Transmit byte sequence
 *      len   Byte length
 * 
 *  Return
 *      int   Number of free bytes
 * ----------------------------------------------------------------------------------------*/
void sci_putb(int ch, unsigned char *buf, int len)
{
    switch (ch) {
    case 0:
        sci0_putb(buf, len);
        break;
    case 1:
        sci1_putb(buf, len);
        break;
    case 2:
        sci2_putb(buf, len);
        break;
    case 3:
        sci3_putb(buf, len);
        break;
    case 5:
        sci5_putb(buf, len);
        break;
    case 6:
        sci6_putb(buf, len);
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * sci_putc
 * 
 *  Function description
 *      Send one character to SCI
 * 
 *  Argument
 *      ch    SCI channel
 *      data  Transmitted character
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void sci_putc(int ch, char data)
{
    char str[2];
    str[0]  = data;
    str[1]  = 0;
    switch (ch) {
    case 0:
        sci0_putb((unsigned char *)str, 1);
        break;
    case 1:
        sci1_putb((unsigned char *)str, 1);
        break;
    case 2:
        sci2_putb((unsigned char *)str, 1);
        break;
    case 3:
        sci3_putb((unsigned char *)str, 1);
        break;
    case 5:
        sci5_putb((unsigned char *)str, 1);
        break;
    case 6:
        sci6_putb((unsigned char *)str, 1);
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * sci(n)_txi
 * 
 *  Function description
 *      SCI0,1,2,3,5,6 Interrupt function generated when transmission buffer becomes empty
 * 
 *  Argument
 *      None
 * 
 *  Return
 *      None
 * 
 *  Note
 *     Interrupt vector (VECT_SCI0_TXI0 to VECT_SCI6_TXI6) 215,218,221,224,230,233
 * ----------------------------------------------------------------------------------------*/
 
#ifdef      SCI0_ACTIVATE
// 215
void interrupt __vectno__ {VECT_SCI0_TXI0} sci0_txi(void)
{
    int i;
    SCI_MODULE *com = &sci_com[0];
    if (com->txrp != com->txwp) { // Data remains in buffer 
#ifdef  SCI0_TXOSDN
        SCI0_TXOSDN_PORT = 1; // 0=Receive only / 1=Transmit possible 
#endif
#ifdef  SCI0_FLOW
        if (SCI0_CTS_PORT == 0) { // CTS=Enable 
#endif
        i = com->txrp++;
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI0.TDR = com->txbuf[i];
#ifdef  SCI0_FLOW
    } else { // Stop transmission by CTS 
        SCI0.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI0.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
#endif
    } else { // Transmit buffer is empty 
        SCI0.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI0.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
}
#endif // ifdef      SCI0_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI1_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 218*/
#ifndef __YIDE_REM_DEBUG__
void interrupt __vectno__ {VECT_SCI1_TXI1} sci1_txi(void)
#else
void interrupt sci1_txi(void)
#endif
{
    int i;
    SCI_MODULE *com = &sci_com[1];
    if (com->txrp != com->txwp) { // Data remains in buffer 
#ifdef  SCI1_FLOW
        if (SCI1_CTS_PORT == 0) { // CTS=Enable 
#endif
        i = com->txrp++
            if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI1.TDR = com->txbuf[i];
#ifdef  SCI1_FLOW
    } else { // Stop transmission by CTS 
        SCI1.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI1.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
#endif
    } else { // Transmit buffer is empty 
        SCI1.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI1.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
}
#endif // ifdef      SCI1_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI2_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 221*/
void interrupt __vectno__ {VECT_SCI2_TXI2} sci2_txi(void)
{
    int i;
    SCI_MODULE *com = &sci_com[2];
    if (com->txrp != com->txwp) { // Data remains in buffer 
#ifdef  SCI2_TXOSDN
        SCI2_TXOSDN_PORT = 1;     // 0=Receive only / 1=Transmit enable 
#endif
#ifdef  SCI2_FLOW
        if (SCI2_CTS_PORT == 0) { // CTS=Enable 
#endif
        i = com->txrp++;
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI2.TDR = com->txbuf[i];
#ifdef  SCI2_FLOW
    } else { // Stop transmission by CTS 
        SCI2.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI2.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
#endif
    } else { // Transmit buffer is empty 
        SCI2.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI2.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
}
#endif // ifdef      SCI2_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI3_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 224*/
void interrupt __vectno__ {VECT_SCI3_TXI3} sci3_txi(void)
{
    int i;
    SCI_MODULE *com = &sci_com[3];
    if (com->txrp != com->txwp) { // Data remains in buffer 
#ifdef  SCI3_FLOW
        if (SCI3_CTS_PORT == 0) { // CTS=Enable 
#endif
        i = com->txrp++
            if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI3.TDR = com->txbuf[i];
#ifdef  SCI3_FLOW
    } else { // Stop transmission by CTS 
        SCI3.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI3.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
#endif
    } else { // Transmit buffer is empty 
        SCI3.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI3.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
}
#endif // ifdef      SCI3_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI5_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 230*/
void interrupt __vectno__ {VECT_SCI5_TXI5} sci5_txi(void)
{
    int i;
    SCI_MODULE *com = &sci_com[5];
    if (com->txrp != com->txwp) { // Data remains in buffer 
#ifdef  SCI5_FLOW
        if (SCI5_CTS_PORT == 0) { // CTS=Enable 
#endif
        i = com->txrp++
            if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI5.TDR = com->txbuf[i];
#ifdef  SCI5_FLOW
    } else { // Stop transmission by CTS 
        SCI5.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI5.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
#endif
    } else { // Transmit buffer is empty 
        SCI5.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI5.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
}
#endif // ifdef      SCI5_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI6_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 233*/
void interrupt __vectno__ {VECT_SCI6_TXI6} sci6_txi(void)
{
    int i;
    SCI_MODULE *com = &sci_com[6];
    if (com->txrp != com->txwp) { // Data remains in buffer 
#ifdef  SCI6_FLOW
        if (SCI6_CTS_PORT == 0) { // CTS=Enable 
#endif
        i = com->txrp++
            if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI6.TDR = com->txbuf[i];
#ifdef  SCI61_FLOW
    } else { // Stop transmission by CTS 
        SCI6.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI6.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
#endif
    } else { // Transmit buffer is empty 
        SCI6.SCR.BIT.TIE  = 0; // TXI interrupt disable 
        SCI6.SCR.BIT.TEIE = 1; // TEI interrupt enable 
    }
}
#endif // ifdef      SCI6_ACTIVATE

/* ----------------------------------------------------------------------------------------
 * sci(n)_tei
 *
 *  Function description
 *      SCI0,1,2,3,5,6 Interrupt function that occurs when the shift register completes transmission
 * 
 *  Argument
 *      None
 * 
 *  Return
 *      None
 * 
 *  Note
 *     Interrupt vector (VECT_SCI0_TEI0 to VECT_SCI6_TEI6) 216,219,222,225,231,234
 * ----------------------------------------------------------------------------------------*/

#ifdef      SCI0_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 216*/
void interrupt __vectno__ {VECT_SCI0_TEI0} sci0_tei(void)
{
    int i;
    SCI_MODULE *com = &sci_com[0];
    SCI0.SCR.BIT.TEIE = 0; // TEI interrupt disable 
#ifdef  SCI0_TXOSDN
    SCI0_TXOSDN_PORT = 0; // 0=nRE(Receiving) / 1=DE(Transmission) 
#endif
    if (com->txrp != com->txwp) { // Data remains in buffer 
        SCI0.SCR.BIT.TIE = 1; // Transmit operation enable 
        i = com->txrp++;
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI0.TDR = com->txbuf[i];
    } else {
        SCI0.SCR.BIT.TE = 0; // Transmit operation disable 
    }
}
#endif // ifdef      SCI0_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI1_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 219*/
#ifndef __YIDE_REM_DEBUG__
void interrupt __vectno__ {VECT_SCI1_TEI1} sci1_tei(void)
#else
void interrupt sci1_tei(void)
#endif
{
    int i;
    SCI_MODULE *com = &sci_com[1];
    SCI1.SCR.BIT.TEIE = 0; // TEI interrupt disable 
    if (com->txrp != com->txwp) { // Data remains in buffer 
        SCI1.SCR.BIT.TIE = 1; // Transmit operation enable 
        i = com->txrp++
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI1.TDR = com->txbuf[i];
    } else {
        SCI1.SCR.BIT.TE = 0; // Transmit operation disable 
    }
}
#endif // ifdef      SCI1_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI2_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 222*/
void interrupt __vectno__ {VECT_SCI2_TEI2} sci2_tei(void)
{
    int i;
    SCI_MODULE *com = &sci_com[2];
    SCI2.SCR.BIT.TEIE = 0; // TEI interrupt disable 
#ifdef  SCI2_TXOSDN
    SCI2_TXOSDN_PORT = 0;  // 0=nRE(Receiving) / 1=DE(Transmission) 
#endif
    if (com->txrp != com->txwp) {   // Data remains in buffer 
        SCI2.SCR.BIT.TIE = 1; // Transmit operation enable 
        i = com->txrp++;
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI2.TDR = com->txbuf[i];
    } else {
        SCI2.SCR.BIT.TE = 0; // Transmit operation disable 
    }
}
#endif // ifdef      SCI2_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI3_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 225*/
void interrupt __vectno__ {VECT_SCI3_TEI3} sci3_tei(void)
{
    int i;
    SCI_MODULE *com = &sci_com[3];
    SCI3.SCR.BIT.TEIE = 0;  // TEI interrupt disable 
    if (com->txrp != com->txwp) { // Data remains in buffer 
        SCI3.SCR.BIT.TIE = 1; // Transmit operation enable 
        i = com->txrp++
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI3.TDR = com->txbuf[i];
    } else {
        SCI3.SCR.BIT.TE = 0; // Transmit operation edisble 
    }
}
#endif // ifdef      SCI3_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI5_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 231*/
void interrupt __vectno__ {VECT_SCI5_TEI5} sci5_tei(void)
{
    int i;
    SCI_MODULE *com = &sci_com[5];
    SCI5.SCR.BIT.TEIE = 0; // TEI interrupt disable 
    if (com->txrp != com->txwp) { // Data remains in buffer 
        SCI5.SCR.BIT.TIE = 1; // Transmit operation enable 
        i = com->txrp++
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI5.TDR = com->txbuf[i];
    } else {
        SCI5.SCR.BIT.TE = 0; // Transmit operation disable 
    }
}
#endif // ifdef      SCI5_ACTIVATE
//---------------------------------------------------------------------------------------- 
#ifdef      SCI6_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * 234*/
void interrupt __vectno__ {VECT_SCI6_TEI6} sci6_tei(void)
{
    int i;
    SCI_MODULE *com = &sci_com[6];
    SCI6.SCR.BIT.TEIE = 0; // TEI interrupt disable 
    if (com->txrp != com->txwp) { // Data remains in buffer 
        SCI6.SCR.BIT.TIE = 1; // Transmit operation enable 
        i = com->txrp++
        if (com->txrp >= BUFSIZE) {
            com->txrp = 0;
        }
        SCI6.TDR = com->txbuf[i];
    } else {
        SCI6.SCR.BIT.TE = 0; // Transmit operation disable 
    }
}
#endif // ifdef      SCI6_ACTIVATE

/* ----------------------------------------------------------------------------------------
 * sci_load
 * 
 *  Function description
 *      SCI0,1,2,3,5,6 Stack received data in buffer
 * 
 *  Argument
 *      ch    SCI channel
 *      err   Error code
 *      data  Data + Error flag
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void sci_load(int ch, unsigned char err, unsigned char data)
{
    int sz;
    SCI_MODULE *com = &sci_com[ch];
    if (err != 0) {
        com->err++;
        if ((err & 0x08) != 0) { // Parity error 
            com->perr++;
        }
        if ((err & 0x10) != 0) { // Framing error 
            com->ferr++;
        }
        if ((err & 0x20) != 0) { // Overrun error 
            com->oerr++;
        }
    }
    // Save data 
    com->rxbuf[com->rxwp++] = data;
    if (com->rxwp >= BUFSIZE) {
        com->rxwp = 0; // Reset pointer to 0 
    }
    // RTS flow control 
    switch (ch) {
#ifdef      SCI0_ACTIVATE
#ifdef  SCI0_FLOW
    case 0: // COM0 dedicated flow control 
        sz = com->rxrp - com->rxwp;
        if (sz < 0) {
            sz += BUFSIZE;
        }
        if (sz < (BUFSIZE * 3 / 4)) { // RTS=Disable 
            SCI0_RTS_PORT = 1;
        }
        break;
#endif // ifdef  SCI0_FLOW
#endif // ifdef      SCI0_ACTIVATE
#ifdef      SCI1_ACTIVATE
#ifdef  SCI1_FLOW
    case 1: // COM1 dedicated flow control 
        sz = com->rxrp - com->rxwp;
        if (sz < 0) {
            sz += BUFSIZE;
        }
        if (sz < (BUFSIZE * 3 / 4)) { // RTS=Disable 
            SCI1_RTS_PORT = 1;
        }
        break;
#endif // ifdef  SCI1_FLOW
#endif // ifdef      SCI1_ACTIVATE
#ifdef      SCI2_ACTIVATE
#ifdef  SCI2_FLOW
    case 2: // COM2 dedicated flow control 
        sz = com->rxrp - com->rxwp;
        if (sz < 0) {
            sz += BUFSIZE;
        }
        if (sz < (BUFSIZE * 3 / 4)) { // RTS=Disable 
            SCI2_RTS_PORT = 1;
        }
        break;
#endif // ifdef  SCI2_FLOW
#endif // ifdef      SCI2_ACTIVATE
#ifdef      SCI3_ACTIVATE
#ifdef  SCI3_FLOW
    case 3: // COM3 dedicated flow control 
        sz = com->rxrp - com->rxwp;
        if (sz < 0) {
            sz += BUFSIZE;
        }
        if (sz < (BUFSIZE * 3 / 4)) { // RTS=Disable 
            SCI3_RTS_PORT = 1;
        }
        break;
#endif // ifdef  SCI3_FLOW
#endif // ifdef      SCI3_ACTIVATE
#ifdef      SCI6_ACTIVATE
#ifdef  SCI6_FLOW
    case 6: // COM6 dedicated flow control 
        sz = com->rxrp - com->rxwp;
        if (sz < 0) {
            sz += BUFSIZE;
        }
        if (sz < (BUFSIZE * 3 / 4)) { // RTS=Disable 
            SCI6_RTS_PORT = 1;
        }
        break;
#endif // ifdef  SCI6_FLOW
#endif // ifdef      SCI6_ACTIVATE
    default:
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * sci_err
 *
 *  Function description
 *      SCI error occurrence interrupt (GROUP12) Acquires received data when an error occurs
 * 
 *  Argument
 *      None
 * 
 *  Return
 *      None
 * 
 *  Note
 *      Interrupt vector (VECT_ICU_GROUPL0) 114
 * ----------------------------------------------------------------------------------------*/
void interrupt __vectno__ {VECT_ICU_GROUPL0} sci_err(void)
{
    unsigned char d, e;
    PROC_CALL p;

#ifdef      SCI0_ACTIVATE
    // SCI0 
    if (ICU.GRP[GRP_SCI0_ERI0].BIT.IS_SCI0_ERI0) { // SCI0 With error 
        e = SCI0.SSR.BYTE;
        d = SCI0.RDR & 0x00FF;
        sci_load(0, e, d);
        SCI0.SSR.BYTE = 0;
    }
#endif // ifdef      SCI0_ACTIVATE
#ifdef      SCI1_ACTIVATE
    // SCI1 
    if (ICU.GRP[GRP_SCI1_ERI1].BIT.IS_SCI1_ERI1) { // SCI1 With error 
        e = SCI1.SSR.BYTE;
        d = SCI1.RDR & 0x00FF;
        sci_load(1, e, d);
        SCI1.SSR.BYTE = 0;
    }
#endif // ifdef      SCI1_ACTIVATE
#ifdef      SCI2_ACTIVATE
    // SCI2 
    if (ICU.GRP[GRP_SCI2_ERI2].BIT.IS_SCI2_ERI2) { // SCI2 With error 
        e = SCI2.SSR.BYTE;
        d = SCI2.RDR & 0x00FF;
        sci_load(2, e, d);
        SCI2.SSR.BYTE = 0;
    }
#endif // ifdef      SCI2_ACTIVATE
#ifdef      SCI3_ACTIVATE
    // SCI3 
    if (ICU.GRP[GRP_SCI3_ERI3].BIT.IS_SCI3_ERI3) { // SCI3 With error 
        e = SCI3.SSR.BYTE;
        d = SCI3.RDR & 0x00FF;
        sci_load(3, e, d);
        SCI3.SSR.BYTE = 0;
    }
#endif // ifdef      SCI3_ACTIVATE
#ifdef      SCI5_ACTIVATE
    // SCI5 
    if (ICU.GRP[GRP_SCI5_ERI5].BIT.IS_SCI5_ERI5) { // SCI5 With error 
        e = SCI5.SSR.BYTE;
        d = SCI5.RDR & 0x00FF;
        sci_load(5, e, d);
        SCI5.SSR.BYTE = 0;
    }
#endif // ifdef      SCI5_ACTIVATE
#ifdef      SCI6_ACTIVATE
    // SCI6 
    if (ICU.GRP[GRP_SCI6_ERI6].BIT.IS_SCI6_ERI6) { // SCI6 With error 
        e = SCI6.SSR.BYTE;
        d = SCI6.RDR & 0x00FF;
        sci_load(6, e, d);
        SCI6.SSR.BYTE = 0;
    }
#endif // ifdef      SCI6_ACTIVATE
#ifdef      RSPI1_ACTIVATE
    // RSPI1 
    if (ICU.GRP[GRP_RSPI1_SPEI1].BIT.IS_RSPI1_SPEI1) { // RSPI1 With error 
        e = (unsigned short)RSPI1.SPSR.BYTE;
        RSPI1.SPSR.BYTE = 0;
        logging("SPI1 Error %02X\r", (int)e);
    }
#endif // ifdef      RSPI1_ACTIVATE
    // RSPI2 
    if (ICU.GRP[GRP_RSPI2_SPEI2].BIT.IS_RSPI2_SPEI2) { // RSPI2 With error 
        e = (unsigned short)RSPI2.SPSR.BYTE;
        RSPI2.SPSR.BYTE = 0;
        logging("SPI2 Error %02X\r", (int)e);
    }
}

/* ----------------------------------------------------------------------------------------
 * sci(n)_rxi
 * 
 *  Function description
 *      SCI receive interrupt processing
 * 
 *  Argument
 *      None
 * 
 *  Return
 *      None
 * 
 *  Note
 *      Interrupt vector (VECT_SCI0_RXI0 to VECT_SCI6_RXI6) 214,217,220,223,229,232
 * ----------------------------------------------------------------------------------------*/
#ifdef      SCI0_ACTIVATE
/* ---------------------------------------------------------------------------------------
 * 214
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_SCI0_RXI0} sci0_rxi(void)
{
    sci_load(0, 0, SCI0.RDR);
}
#endif // ifdef      SCI0_ACTIVATE
#ifdef      SCI1_ACTIVATE
/* ---------------------------------------------------------------------------------------
 * 217
 * --------------------------------------------------------------------------------------- */
#ifndef __YIDE_REM_DEBUG__
void interrupt __vectno__ {VECT_SCI1_RXI1} sci1_rxi(void)
#else
void interrupt sci1_rxi(void)
#endif
{
    sci_load(1, 0, SCI1.RDR);
}
#endif // ifdef      SCI1_ACTIVATE
#ifdef      SCI2_ACTIVATE
/* ---------------------------------------------------------------------------------------
 * 220
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_SCI2_RXI2} sci2_rxi(void)
{
    sci_load(2, 0, SCI2.RDR);
}
#endif // ifdef      SCI2_ACTIVATE
#ifdef      SCI3_ACTIVATE
/* ---------------------------------------------------------------------------------------
 * 223
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_SCI3_RXI3} sci3_rxi(void)
{
    sci_load(3, 0, SCI3.RDR);
}
#endif // ifdef      SCI3_ACTIVATE
#ifdef      SCI5_ACTIVATE
/* ---------------------------------------------------------------------------------------
 * 229
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_SCI5_RXI5} sci5_rxi(void)
{
    sci_load(5, 0, SCI5.RDR);
}
#endif // ifdef      SCI5_ACTIVATE
#ifdef      SCI6_ACTIVATE
/* ---------------------------------------------------------------------------------------
 * 232
 * --------------------------------------------------------------------------------------- */
void interrupt __vectno__ {VECT_SCI6_RXI6} sci6_rxi(void)
{
    sci_load(6, 0, SCI6.RDR);
}
#endif // ifdef      SCI6_ACTIVATE
/* ----------------------------------------------------------------------------------------
 * sci_get_check
 * 
 *  Function description
 *      SCI Get number of received bytes
 * 
 *  Argument
 *      ch    SCI channel number
 * 
 *  Return
 *      int   Receive buffer unprocessed bytes
 * ----------------------------------------------------------------------------------------*/
int sci_get_check(int ch)
{
    SCI_MODULE *com = &sci_com[ch];
    int sz = (com->rxwp - com->rxrp);
    if (sz < 0) {
        sz += BUFSIZE;
    }
    return sz;
}

/* ----------------------------------------------------------------------------------------
 * sci_get_check
 *
 *  Function description
 *      SCI Get number of received bytes
 * 
 *  Argument
 *      ch    SCI channel number
 * 
 *  Return
 *      int   Receive buffer unprocessed bytes
 * ----------------------------------------------------------------------------------------*/
void sci_rts_control(int ch)
{
    // RTS flow control 
    if (sci_get_check(ch) > (BUFSIZE / 4)) {
        return;
    }
    switch (ch) {
#ifdef      SCI0_ACTIVATE
#ifdef  SCI0_FLOW
    case 0: // COM0 dedicated flow control 
        SCI0_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI0_ACTIVATE
#ifdef      SCI1_ACTIVATE
#ifdef  SCI1_FLOW
    case 1: // COM1 dedicated flow control 
        SCI1_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI1_ACTIVATE
#ifdef      SCI2_ACTIVATE
#ifdef  SCI2_FLOW
    case 2: // COM2 dedicated flow control 
        SCI2_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI2_ACTIVATE
#ifdef      SCI3_ACTIVATE
#ifdef  SCI3_FLOW
    case 3: // COM3 dedicated flow control 
        SCI3_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI3_ACTIVATE
#ifdef      SCI6_ACTIVATE
#ifdef  SCI6_FLOW
    case 6: // COM6 dedicated flow control 
        SCI6_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI6_ACTIVATE
    default:
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * sci_get_char
 * 
 *  Function description
 *      SCI Get number of received bytes
 * 
 *  Argument
 *      ch    SCI channel number
 * 
 *  Return
 *      int   Data 0x00 to 0xFF:Yes -1:No
 * ----------------------------------------------------------------------------------------*/
int sci_get_char(int ch)
{
    int data;
    SCI_MODULE *com = &sci_com[ch];
    if (com->rxrp != com->rxwp) {
        data = (int)com->rxbuf[com->rxrp++] & 0x00FF;
        if (com->rxrp >= BUFSIZE) {
            com->rxrp = 0;
        }
        sci_rts_control(ch);
    } else {
        data = -1;
    }
    return data;
}

/* ----------------------------------------------------------------------------------------
 * sci_get_buf
 * 
 *  Function description
 *      SCI Get received data
 * 
 *  Argument
 *      ch    SCI channel number
 *      *buf  Copy destination buffer
 *      size  Transfer bytes
 * 
 *  Return
 *      int   Actual transfer bytes
 * ----------------------------------------------------------------------------------------*/
int sci_get_buf(int ch, unsigned char *buf, int size)
{
    int i;
    SCI_MODULE *com = &sci_com[ch];
    for (i = 0; i < size; i++) {
        if (com->rxrp != com->rxwp) {
            *buf++ = com->rxbuf[com->rxrp++];
            if (com->rxrp >= BUFSIZE) {
                com->rxrp = 0;
            }
        } else {
            break;
        }
    }
    sci_rts_control(ch);
    return i;
}

/* ----------------------------------------------------------------------------------------
 * sci_get_string
 * 
 *  Function description
 *      SCI Get received string
 * 
 *  Argument
 *      ch    SCI channel number
 *      *str  Copy destination buffer
 *      size  Transfer character limit
 * 
 *  Return
 *      int   Actual transfer characters
 * ----------------------------------------------------------------------------------------*/
int sci_get_string(int ch, char *str, int size)
{
    int i;
    char c;
    SCI_MODULE *com = &sci_com[ch];
    for (i = 0; i < size; i++) {
        if (com->rxrp != com->rxwp) {
            c = (char)com->rxbuf[com->rxrp++];
            if (com->rxrp >= BUFSIZE) {
                com->rxrp = 0;
            }
            *str++ = c;
            if (c == 0) {
                return i; // End of string 
            }
        } else {
            break;
        }
    }
    *str = 0; // Add NUL 
    sci_rts_control(ch);
    return i;
}

/* ----------------------------------------------------------------------------------------
 * sci_get_line
 * 
 *  Function description
 *      SCI Get one line of received character string
 * 
 *  Argument
 *      ch     SCI channel number
 *      *str   Copy destination buffer
 *      size   Transfer character limit
 *      echar  End of line character
 * 
 *  Return
 *      int    Actual transfer characters / negative numbers indicate buffer shortage or end of line
 * ----------------------------------------------------------------------------------------*/
int sci_get_line(int ch, char *str, int size, char echar)
{
    int i, e;
    char c;
    SCI_MODULE *com = &sci_com[ch];

    for (e = 0, i = 0; i < size;) {
        if (com->rxrp != com->rxwp) {
            c = (char)com->rxbuf[com->rxrp++];
            if (com->rxrp >= BUFSIZE) {
                com->rxrp = 0;
            }
            if (c == echar) { // End with end-of-line code 
                e++;
                break;
            } else {
                str[i++] = c;
            }
        } else {
            break;
        }
    }
    str[i] = 0; // Add NUL 
    sci_rts_control(ch);
    if (e == 0) {
        return -i;
    }
    return i;
}

/* ----------------------------------------------------------------------------------------
 * sci_clear
 *
 *  Function description
 *      SCI Clear control variables
 * 
 *  Argument
 *      ch    SCI channel number
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void sci_clear(int ch)
{
    SCI_MODULE *com = &sci_com[ch];
    memset(com, 0, sizeof(SCI_MODULE));
    switch (ch) {
#ifdef      SCI0_ACTIVATE
#ifdef  SCI0_FLOW
    case 0: // COM0 dedicated flow control 
        SCI0_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI0_ACTIVATE
#ifdef      SCI1_ACTIVATE
#ifdef  SCI1_FLOW
    case 1: // COM1 dedicated flow control 
        SCI1_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI1_ACTIVATE
#ifdef      SCI2_ACTIVATE
#ifdef  SCI2_FLOW
    case 2: // COM2 dedicated flow control 
        SCI2_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI2_ACTIVATE
#ifdef      SCI3_ACTIVATE
#ifdef  SCI3_FLOW
    case 3: // COM3 dedicated flow control 
        SCI3_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI3_ACTIVATE
#ifdef      SCI6_ACTIVATE
#ifdef  SCI6_FLOW
    case 6: // COM6 dedicated flow control 
        SCI6_RTS_PORT = 0;
        break;
#endif
#endif // ifdef      SCI6_ACTIVATE
    default:
        break;
    }
}

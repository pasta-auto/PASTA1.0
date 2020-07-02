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
#ifndef __CAN2ECU_SCI_IF__
#define __CAN2ECU_SCI_IF__

#include    "ecu.h"         // ECU common definition 

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

// Select SCI port to use 

#define     SCI0_ACTIVATE
//#define  SCI1_ACTIVATE 
#define     SCI2_ACTIVATE
/* #define  SCI3_ACTIVATE
 * #define  SCI4_ACTIVATE
 * #define  SCI5_ACTIVATE
 *#define  SCI6_ACTIVATE*/


// SCI buffer size 
#define     BUFSIZE     1024
/* Use SCI0 as RS-485 half duplex
 * #define  SCI0_RS485
 * Use nRTS and nCTS of SCI1
 * #define  SCI1_FLOW
 * Assign nCTS of SCI3 to P24 port
 * #define     SCI3_nCTS
 * Use nRTS / nCTS of SCI1 as SCI6 port
 * #define  SCI6_ACTIVE
 * SCI management structure*/
typedef struct  __sci_module__ {
    unsigned char   txbuf[BUFSIZE];     // Transmission buffer 
    unsigned char   rxbuf[BUFSIZE];     // Receiving buffer 
    int             txwp;               // Transmit write pointer 
    int             txrp;               // Transmit  read pointer 
    int             rxwp;               // Receive  write pointer 
    int             rxrp;               // Receive   read pointer 
    int             err;                // Total    error counter 
    int             perr;               // Parity   error counter 
    int             ferr;               // Framing  error counter 
    int             oerr;               // Overrun  error counter 
}   SCI_MODULE;

//Indirect call prototype (1 argument) 
typedef void (*PROC_CALL)(void *);

/* SCI0,SCI1,SCI2,SCI3,SCI5,SCI6 SCI1=Use with Yellow Scope
 * extern CONSOLE_CTRL  sci_com;
 *extern CONSOLE_CTRL  usb_com;*/

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
extern void sci0_init(long bps, int datalen, int stoplen, int parity);
#define     SCI0_RTS_PORT           PORT8.PODR.BIT.B6   // out 0=Enable / 1=Disable 
#define     SCI0_CTS_PORT           PORT8.PIDR.BIT.B7   // in 0=Enable / 1=Disable 
#define     SCI0_TXOSDN_PORT        PORT7.PODR.BIT.B0   // 0=RX / 1=TX 

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
extern void sci1_init(long bps, int datalen, int stoplen, int parity);
#define     SCI1_RTS_PORT           PORTE.PODR.BIT.B1   // out 0=Enable / 1=Disable 
#define     SCI1_CTS_PORT           PORTE.PIDR.BIT.B2   // in 0=Enable / 1=Disable 

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
 *      bps      Communication speed 300 to 115200
 *      datalen  Data lentgh 7,8
 *      stoplen  Stop length 1,2
 *      parity   Parity 0=none / 1=odd / 2=even
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
extern void sci2_init(long bps, int datalen, int stoplen, int parity);
#define     SCI2_RTS_PORT           PORT1.PODR.BIT.B5   // out 0=Enable / 1=Disable 
#define     SCI2_CTS_PORT           PORT1.PIDR.BIT.B7   //  in 0=Enable / 1=Disable 
#define     SCI2_TXOSDN_PORT        PORT7.PODR.BIT.B3   //         0=RX / 1=TX 

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
 *      bps      Communication speed 300ï¿½`115200
 *      datalen  Data lentgh 7,8
 *      stoplen  Stop lentgh 1,2
 *      parity   Parity 0=none / 1=odd / 2=even
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
extern void sci3_init(long bps, int datalen, int stoplen, int parity);
#define     SCI3_RTS_PORT           PORT2.PODR.BIT.B2   // out 0=Enable / 1=Disable 
#define     SCI3_CTS_PORT           PORT2.PIDR.BIT.B4   //  in 0=Enable / 1=Disable 
#define     SCI3_EXRES_PORT         PORT5.PODR.BIT.B6   // External reset output 0=RESET 

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
 *      speed  Communication speed
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
extern void sci5_init(long bps, int datalen, int stoplen, int parity);
#define     SCI5_SS0_PORT           PORTC.PODR.BIT.B5   // External selection line output 0=Select 
#define     SCI5_SS1_PORT           PORTC.PODR.BIT.B6   // External selection line output 0=Select 

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
extern void sci6_init(long bps, int datalen, int stoplen, int parity);
#define     SCI6_RTS_PORT           PORT0.PODR.BIT.B2   // out 0=Enable / 1=Disable 
#define     SCI6_CTS_PORT           PORTJ.PIDR.BIT.B3   //  in 0=Enable / 1=Disable 

/* ----------------------------------------------------------------------------------------
 * sci_putcheck
 *
 *  Function description
 *      Find the free space in the SCI transmission buffer
 * 
 *  Argument
 *     ch   SCI channel
 * 
 *  Return
 *     int   Number of free bytes
 * ----------------------------------------------------------------------------------------*/
extern int sci_putcheck(int ch);

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
extern int sci_txbytes(int ch);

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
extern void sci0_putb(unsigned char *buf, int size);
extern void sci1_putb(unsigned char *buf, int size);
extern void sci2_putb(unsigned char *buf, int size);
extern void sci3_putb(unsigned char *buf, int size);
extern void sci5_putb(unsigned char *buf, int size);
extern void sci6_putb(unsigned char *buf, int size);

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
extern void sci_puts(int ch, char *str);

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
extern void sci_putb(int ch, unsigned char *buf, int len);

/* ----------------------------------------------------------------------------------------
 *
 * sci_putc
 * ----------------------------------------------------------------------------------------
 * Function description
 *     Send one character to SCI
 * Argument
 *     ch    SCI channel
 *     data  Transmitted character
 * Return
 *     None
 * ----------------------------------------------------------------------------------------
 */
extern void sci_putc(int ch, char data);

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
extern void sci_txint(int ch);

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
extern void sci_load(int ch, unsigned char err, unsigned char data);

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
extern int sci_get_check(int ch);

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
extern int sci_get_char(int ch);

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
extern int sci_get_buf(int ch, unsigned char *buf, int size);

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
extern int sci_get_string(int ch, char *str, int size);

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
extern int sci_get_line(int ch, char *str, int size, char echar);

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
extern void sci_clear(int ch);

#endif //__CAN2ECU_SCI_IF__

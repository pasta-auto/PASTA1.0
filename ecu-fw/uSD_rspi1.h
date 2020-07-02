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
 * ________________________________________________________________________________________
 */
#ifndef __CAN2ECU_uSD_IF__
#define __CAN2ECU_uSD_IF__

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

#define     RSPI1_ACTIVATE

#ifdef      RSPI1_ACTIVATE

// RSPI management structure for uSD 
typedef struct  __spi_module__ {
    int   err;      // Error flag 
    void *rx_proc;  // Call function at reception completion interrupt 
    void *tx_proc;  // Call function at transmission completion interrupt 
    void *ti_proc;  // Call function at idling interrupt 
    void *err_proc; // Call function at error interrupt 
}   SPI_MODULE;

extern SPI_MODULE usd_spi_com;

//Indirect call prototype (1 argument) 
typedef void (*USD_PROC_CALL)(void *);

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
extern void rspi1_init(long bps);

#endif // RSPI1_ACTIVATE
#endif //__CAN2ECU_uSD_IF__

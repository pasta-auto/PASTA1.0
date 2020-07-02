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
* File Name    : mcu_info.h
* Version      : 1.0
* Device(s)    : RX
* H/W Platform : RSK+RX63N
* Description  : Information about the MCU on this board.
**********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 28.11.2011 1.00     First Release
**********************************************************************************************************************/

#ifndef _MCU_INFO
#define _MCU_INFO

// MCU that is used. 
#define MCU_RX63N           (1)

// Package. 
#define PACKAGE_LQFP176     (1)

// Memory size of your MCU. 
#define ROM_SIZE_BYTES      (1048576)
#define RAM_SIZE_BYTES      (131072)
#define DF_SIZE_BYTES       (32768)

// System clock speed in Hz. 
#define ICLK_HZ             (96000000)
// Peripheral clock speed in Hz. 
#define PCLK_HZ             (48000000)
// External bus clock speed in Hz. 
#define BCLK_HZ             (24000000)
// FlashIF clock speed in Hz. 
#define FCLK_HZ             (48000000)

#endif // _MCU_INFO

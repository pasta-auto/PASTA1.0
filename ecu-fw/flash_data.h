/* ---------------------------------------------------------------------------------------
*  Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.
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
/* ---------------------------------------------------------------------------------------
 * File Name     : flash_data.h
 * Version       : 1.00
 * Device        : R5F563NB
 * Tool-Chain    : Renesas RX Standard 1.2.0.0
 * H/W Platform  : RSK+RX63N
 * Description   : Provides declarations for functions defined in flash_data.c
* ---------------------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------------------
* History       : 13 Aug. 2012  Ver. 1.00 First Release
* ---------------------------------------------------------------------------------------*/

// Multiple inclusion prevention macro 
#ifndef FLASH_DATA_H
#define FLASH_DATA_H

/* Define PROG_SIZE (see Flash API Application note for further details)
 * #define PROG_SIZE   256
 * Define BUFF_SIZE (see Flash API Application note for further details)
 *#define BUFF_SIZE   256*/

// Initialise flash sample function prototype declaration 
void Init_FlashData(void);
// Flash write function prototype declaration 
int Write_FlashData(void);
// Flash erase function prototype declaration 
void Erase_FlashData(void);


// End of multiple inclusion prevention macro 
#endif // ifndef FLASH_DATA_H

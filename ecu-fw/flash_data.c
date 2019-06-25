/*******************************************************************************
Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.    

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
/*******************************************************************************
* File Name     : flash_data.c
* Version       : 1.00
* Device        : R5F563NB
* Tool-Chain    : Renesas RX Standard 1.2.0.0
* H/W Platform  : RSK+RX63N
* Description   : Defines flash data functions used in this sample.
*******************************************************************************/
/*******************************************************************************
* History       : 13 Aug. 2012  Ver. 1.00 First Release
*******************************************************************************/
/******************************************************************************
* System Includes
******************************************************************************/
/* Provides standard string function definitions used in this file */
#include <string.h>

/******************************************************************************
* Project Includes
******************************************************************************/
#include "altypes.h"
#include "iodefine.h"

/* Defines RX63N port registers */
//#include "iodefine_RX63N.h"
/* Defines macros relating to the RX63N user LEDs and switches */
//#include "rskrx63ndef.h"
/* Defines switch functions and variables used in this file */
//#include "switch.h"
/* Defines LCD functions used in this file */
//#include "lcd.h"
/* Declares prototypes for functions defined in this file */
#include "flash_data.h"
/* Defines flash API functions used in this file */
#include "r_Flash_API_RX600.h"

/******************************************************************************
* Global Variables
******************************************************************************/
/* Flash write buffer array */
//uint8_t gFlashWriteBuffer[16];
uint8_t gFlashWriteBuffer[6];
/* Flash write address location global variable */
uint32_t gFlashWriteAddr;
/* Total number of data flash blocks */
uint16_t gNumFlashBlocks = sizeof(g_flash_BlockAddresses)/4u;

/*******************************************************************************
* Local Function Prototypes
*******************************************************************************/
/* ADC initialisation function prototype declaration */
//void InitADC_FlashData(void);
/* Switch callback function prototype delcaration */
void CB_Switch(void);

void Exit_FlashData(void)
{
    /* Enable MCU access to the data flash area */
    R_FlashDataAreaAccess(0x0, 0x0);
}

/*******************************************************************************
* Outline      : Init_FlashData
* Description  : This function initialises the MCU flash area, allowing it to be
*                 read and written to by user code. The function then calls the
*                 InitADC_FlashData function to configure the ADC unit used in
*                 the project. Finally the function erases the contents of the
*                 data flash.
* Argument     : none
* Return value : none
*******************************************************************************/
void Init_FlashData(void)
{
    /* Enable MCU access to the data flash area */
    R_FlashDataAreaAccess(0xFFFF, 0xFFFF);
    
    /* Erase flash memory before writing to it */
	//    Erase_FlashData();
}
/******************************************************************************
* End of function Init_FlashData
******************************************************************************/

/*******************************************************************************
* Outline      : Erase_FlashData
* Description  : This function enters a for loop, and erases a block of data
*                 flash memory each iteration until all blocks have been erased.
* Argument     : none
* Return value : none
*******************************************************************************/
void Erase_FlashData(void)
{
    /* Declare flash API error flag */
    uint8_t ret = 0u; 
        
    /* Declare current data flash block variable */
    uint8_t current_block;
    
    /* Initialise a for loop to erase each of the data flash blocks */
    for(current_block = BLOCK_DB0; current_block < gNumFlashBlocks;
        current_block++)
    {
        /* Fetch beginning address of DF block */  
        uint32_t address = g_flash_BlockAddresses[current_block];
    
        /* Erase data flash block */
        ret |=     R_FlashErase
                (
                    current_block
                 );        
        
        /* Halt here if erase was unsuccessful */
        while(R_FlashGetStatus() != FLASH_SUCCESS);
                        
         /* Check Blank Checking */
         ret |= R_FlashDataAreaBlankCheck
                (
                    address,
                    BLANK_CHECK_ENTIRE_BLOCK
                );
    
        /* Halt here if check was unsuccessful */        
        while(R_FlashGetStatus() != FLASH_SUCCESS);
    }
          
    /* Halt in while loop when flash API errors detected */
	//    while(ret);
	return;
}    
/*******************************************************************************
* End of function Erase_FlashData
*******************************************************************************/

/*******************************************************************************
* Outline      : Write_FlashData
* Description  : This function writes the contents of gFlashWriteBuffer to the
*                 dataflash, at the location pointed by gFlashWriteAddr. If the
*                 number of bytes to write to flash is not a multiple of 8, the
*                 data is padded with null bytes (0x00) to make up to the 
*                 nearest multiple of 8.
* Argument     : none
* Return value : none
*******************************************************************************/
int Write_FlashData(void)
{
    /* Declare flash API error flag */
    uint8_t ret = 0;
    
    /* Declare data padding array and loop counter */
    uint8_t pad_buffer[256];
    
    /* Declare pointer to flash write location */
    uint32_t * flash_ptr = (uint32_t *)gFlashWriteAddr;
    
    /* Declare the number of bytes variable, and initialise with the number of
       bytes the variable gFlashWriteBuffer contains */
    uint8_t num_bytes = sizeof(gFlashWriteBuffer);
        
    /* Clear the contents of the flash write buffer array */
    memset(pad_buffer, 0x00, 256);
    
    /* Copy contents of the write buffer to the padding buffer */
    memcpy((char*)pad_buffer, (char*)gFlashWriteBuffer, num_bytes);

#if 0    
    /* Check if number of bytes is greater than 256 */
    if(num_bytes > 256)
    {    
        /* Number of bytes to write too high, set error flag to 1 */
        ret |= 1;
    }
    /* Check if number of bytes to write is a multiple of 8 */
    else if(num_bytes % 8u)
    {
        /* Pad the data to write so it makes up to the nearest multiple of 8 */
        num_bytes += 8u - (num_bytes % 8u);
    }
    
    /* Halt in while loop when flash API errors detected */
    while(ret);
#endif    

    /* Write contents of write buffer to data flash */
    ret |=     R_FlashWrite
            (
                gFlashWriteAddr,
                (uint32_t)pad_buffer,
                (uint16_t)num_bytes
            );

    /* Compare memory locations to verify written data */    
    ret |= memcmp(gFlashWriteBuffer, flash_ptr, (size_t)num_bytes);

    /* Halt in while loop when flash API errors detected */
	//    while(ret);
	return ret;
} 
/*******************************************************************************
* End of function Write_FlashData
*******************************************************************************/

/*******************************************************************************
* Outline      : CB_Switch 
* Description  : Switch interrupt callback function. This function first checks
*                 which switch has been pressed. If SW1 was pressed, the function
*                 triggers an ADC conversion and fetches the result, and writes
*                 the value into an incrementing data flash address. The function
*                 then displays both the ADC value and the address location it
*                 was written to on the debug LCD. If SW3 was pressed, the
*                 function erases the flash contents, and resets the write flash
*                 write address.
* Argument     : none
* Return value : none
*******************************************************************************/
#if 0
void CB_Switch(void)
{    
    /* Declare temporary character string */
    uint8_t lcd_buffer[] = "=H\'WXYZ \0";

    /* Declare ADC result container variable */
    uint16_t adc_result = 0x0000;
    
    /* Check if switch pressed was SW3 */
    if(gSwitchFlag & SWITCHPRESS_1)
    {
        /* Trigger AD conversion */
       S12AD.ADCSR.BIT.ADST = 1;        
        
        /* Poll ADC unit until conversion is complete */        
        while(S12AD.ADCSR.BIT.ADST == 1);
        
        /* Store result in the global variable adc_result */
        adc_result = S12AD.ADDR0;    
        
        /* Convert ADC result into a character string, and store in the local
           string lcd_buffer */              
        u16ToString(lcd_buffer, 3u, adc_result);
        
        /* Clear the contents of the flash write buffer array */
        memset(gFlashWriteBuffer, 0x00, 6);
        
        /* Copy the contents of the lcd buffer into the flash write buffer
           array */
		//        strncpy ((char*)gFlashWriteBuffer, (char*)lcd_buffer, 8);
        
        /* Write the contents of gFlashDataBuffer to flash memory, at the
           location specified in the address variable gFlashWriteAddr */
        Write_FlashData();
                
        /* Display the contents of the local string lcd_buffer */
        Display_LCD(LCD_LINE2, lcd_buffer);    
    
        /* Convert the 32 bit address location into a 8bit character string */
        u32ToString(lcd_buffer, 0u, gFlashWriteAddr);
        
        /* Display the flash write address on the LCD */
        Display_LCD(LCD_LINE1, lcd_buffer);

        /* Increment the flash write address by 16 bytes */
        gFlashWriteAddr += 0x10;
    }    
    /* Check if switch SW3 was pressed */
    else if(gSwitchFlag & SWITCHPRESS_3)
    {
        /* Erase the data flash memory */
        Erase_FlashData();
        
        /* Reset the flash write address to the start of block DB0 */
        gFlashWriteAddr = g_flash_BlockAddresses[BLOCK_DB0];
        
        /* Display 'Flash Erased' message on the debug LCD */
        Display_LCD(LCD_LINE1, "Flash   ");
        Display_LCD(LCD_LINE2, "Erased  ");
    }
    
    /* Clear switch press flags */
    gSwitchFlag &= 0x0F;
}
/******************************************************************************
* End of function CB_Switch
******************************************************************************/
#endif

#if 0
/*******************************************************************************
* Outline      : InitADC_FlashData
* Description  : This function configures the ADC unit for single shot operation
*                 and sets the switch callback function to CB_Switch.
* Argument     : none
* Return value : none
*******************************************************************************/
void InitADC_FlashData(void)
{
    /* Protection off */
    SYSTEM.PRCR.WORD = 0xA503;
    
    /* Cancel the S12AD module clock stop mode */
    MSTP_S12AD = 0;
    
    /* Protection on */
    SYSTEM.PRCR.WORD = 0xA500;
    
    /* Use the AN000 (Potentiometer) pin 
       as an I/O for peripheral functions */
    PORT4.PMR.BYTE = 0x01;
    
    /* ADC clock = PCLK/8, single scan mode */
    S12AD.ADCSR.BYTE = 0x00;
    
    /* Selects AN000 */
    S12AD.ADANS0.WORD = 0x0001;    
    
    /* Set the initial flash write address to the start of block DB00 */
    gFlashWriteAddr = g_flash_BlockAddresses[BLOCK_DB0];
    
    /* Configure switch callback function */
    SetSwitchReleaseCallback(CB_Switch);    
}
/******************************************************************************
* End of function InitADC_FlashData
******************************************************************************/
#endif




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
/******************************************************************************
* File Name	   : r_flash_api_rx600_config.c
* Version	   : 2.20
* Device 	   : RX600 Series
* Tool-Chain   : RX Family C Compiler
* H/W Platform : RSKRX62N, RSKRX610, YRDKRX62N, RSKRX630, RSKRX63N, RSKRX62T,
*                YRDKRX63N
* Description  : Flash programming for the RX600 Group. This file has options
*                to let the user customize parts of the Flash API.
*******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.12.2009 1.00    First Release
*         : 13.01.2010 1.10    Made function names and variables RAPI compliant
*         : 11.02.2010 1.20    Fixed other RAPI issues and fixed I flag issue
*         : 29.04.2010 1.30    Added support for RX621/N Group. Moved most
*                              device specific data to header file.
*         : 26.05.2010 1.40    Added support for RX62T Group
*         : 28.07.2010 1.41    Fixed bug when performing a blank check on an
*                              entire data flash block.  Also declared 
*                              functions not in the API as 'static'.
*		  : 23.08.2010 1.42    Updated source to raise the coding standard, to
*							   meet GSCE & RSK standards.
*         : 15.02.2011 1.43    Fixed bug in blank check routine when handling
*                              input arguments and moved _Flash_Init() to
*                              _Enter_PE_Mode() function.
*         : 21.04.2011 2.00    Added BGO capabilities for data flash. Made 
*                              some more changes to fit coding standard. Added
*                              ability to do ROM to ROM or DF to DF transfers.
*                              Added the ability to use the lock bit feature
*                              on ROM blocks.  Added BGO capabilities for
*                              ROM operations.
*         : 06.07.2011 2.10    Added support for RX630, RX631, and RX63N.
*                              Also added R_FlashEraseRange() for devices like
*                              RX63x that have finer granularity data flash.
*                              Various bug fixes as well. Example bug fix was
*                              removing DATA_FLASH_OPERATION_PIPL and 
*                              ROM_OPERATION_PIPL #defines since the IPL was
*                              not restored when leaving flash ready interrupt.
*         : 29.11.2011 2.20    Renamed private functions according to new 
*                              Coding Standard. Removed unused 'bytes' argument 
*                              from enter_pe_mode() function. Removed 'far' 
*                              keyword since it is not needed. Fixed where some 
*                              functions were being placed in RAM when this was
*                              not needed. Uses platform.h now instead of 
*                              having multiple iodefine_rxXXX.h's. Added 
*                              __evenaccess directive to FCU accesses. This 
*                              ensures proper bus width accesses. Added
*                              R_FlashCodeCopy() function. When clearing the
*                              FENTRYR register, the register is read back to
*                              ensure its value is 0x0000. Added call to 
*                              exit_pe_mode() when enter_pe_mode() function 
*                              fails to protect against reading ROM in P/E
*                              mode. Added option to use r_bsp package.
******************************************************************************/

#ifndef _FLASH_API_CONFIG_H
#define _FLASH_API_CONFIG_H

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
 ENABLE ROM PROGRAMMING              
******************************************************************************/
/* When uncommented, code is included to program the User ROM program area of
   Flash. Since this code must be executed from within RAM, the sections
   'PFRAM'(ROM) and 'RPFRAM'(RAM) must be added to the linker settings. Also
   the linker option '-rom=PFRAM=RPFRAM' must be added. Finally, the 
   initialization of the 'RPFRAM' must be added to the 'dbsct.c' file as such:
   { __sectop("PFRAM"), __secend("PFRAM"), __sectop("RPFRAM") } */
#define ENABLE_ROM_PROGRAMMING

/******************************************************************************
 ENABLE ROM to ROM or DF to DF programs              
******************************************************************************/
/* If you are doing ROM to ROM writes or DF to DF writes then this DEFINE
   needs to be uncommented. This is necessary because when programming a ROM
   area the MCU cannot read from ROM and when programming a DF area the MCU
   cannot read from the DF. To get by this, if this DEFINE is uncommented 
   then data that is moving from either ROM to ROM or DF to DF is first 
   buffered in a RAM array, then written. If you are never doing one of these
   operations then you can comment out this DEFINE and save yourself 
   the RAM needed to buffer one ROM write (RX62x = 256 bytes, 
   RX630 = 128 bytes). */
//#define FLASH_TO_FLASH   

/******************************************************************************
 ENABLE BGO & NON-BLOCKING DATA FLASH OPERATIONS
******************************************************************************/
/* If this is defined then the flash ready interrupt will be used and 
   FlashAPI routines that deal with the data flash will exit after the
   operation has been started instead of polling for it to finish. */
//#define DATA_FLASH_BGO

/******************************************************************************
 ENABLE BGO & NON-BLOCKING ROM OPERATIONS
 EXTRA CARE SHOULD BE TAKEN IF YOU ENABLE ROM_BGO. SINCE PROGRAM/ERASE 
 FUNCTIONS WILL BE NON-BLOCKING THE API WILL RETURN BEFORE THE ROM OPERATION
 HAS FINISHED. THIS MEANS THAT THE USER CODE THAT CALLS THE FLASH API 
 FUNCTION MUST BE IN RAM OR EXTERNAL MEMORY. THIS WOULD ALSO MEAN THAT THE
 USER HAS TO RELOCATE THE INTERRUPT VECTOR TABLE TO SOMEWHERE OTHER THAN
 ROM OR DATA FLASH. IF THE USER ATTEMPTS TO ACCESS ROM DURING A ROM PROGRAM/
 ERASE OPERATION THE FLASH CONTROL UNIT WILL THROW AN ERROR.
******************************************************************************/
/* If this is defined then the flash ready interrupt will be used and 
   FlashAPI routines that deal with the on-chip ROM will exit after the
   operation has been started instead of polling for it to finish. */
//#define ROM_BGO

/******************************************************************************
 SET IPL OF FLASH READY INTERRUPT
******************************************************************************/
/* If using BGO then the Flash Ready Interrupt will need to have an IPL. */
#define FLASH_READY_IPL     5

/******************************************************************************
 ENABLE OR DISABLE LOCK BIT PROTECTION              
******************************************************************************/
/* Each erasure block has a corresponding lock bit that can be used to 
   protect that block from being programmed/erased after the lock bit is
   set. The use of lock bits can be used or ignored. If the #define below
   is uncommented then lock bits will be ignored and programs/erases to a
   block will not be limited. This only applies to ROM since the DF does
   not have lock bits. */
//#define IGNORE_LOCK_BITS

/******************************************************************************
 COPY API CODE TO RAM USING API FUNCTION OR DBSCT.C
******************************************************************************/
/* After a reset parts of the Flash API must be copied to RAM before the API
   can be used. This originally was done by editing the dbsct.c file to
   copy the code over when other RAM sections are initialized. There is now
   the R_FlashCodeCopy() function which does the same thing. Uncomment this 
   macro if you will be using the R_FlashCodeCopy() function. Comment out this
   macro if you are using the original dbsct.c method. */
#define COPY_CODE_BY_API

/******************************************************************************
 WHETHER TO USE R_BSP PACKAGE OR NOT
******************************************************************************/
/* Starting with v2.20 of the Simple Flash API for RX600 this middleware makes
   use of the r_bsp package. This package contains iodefine's and startup code
   for RX MCUs and boards. The demo that comes with this package makes use of
   the r_bsp package. If the user does not wish to use this package then
   comment out the macro below. If this macro is commented out then the user
   will need to do 2 things:
   1) Make sure that an iodefine.h file is available to the Flash API code. 
      This means that the file should be in the same folder or the project
      should have an include path setup.
   2) Copy the appropriate mcu_info.h file from the r_bsp/board/xxx folder
      that applies to your board. If one of these folders does not apply
      then you may copy from any of them or from the r_bsp/board/user folder.
      This file contains information about the MCU that will be used for 
      configuring the Flash API code (e.g. Flash clock speed). */
//#define FLASH_API_USE_R_BSP

#endif /* _FLASH_API_CONFIG_H */


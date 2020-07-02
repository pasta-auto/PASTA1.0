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
 *  RX63N   Rewrite program ROM area
 *
 * ----------------------------------------------------------------------------------------
 *  Development history
 *
 *  2017/02/10  Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 *  T.Tachibana
 *  L&F
 * ----------------------------------------------------------------------------------------
 */

#ifndef __RX631_FLASH_ROM__
#define __RX631_FLASH_ROM__

/*
 *  ROM block structure (2MByte)
 *
 *  Block
 *  69      FFE0 0000   64K     Area 3  Big data area 1
 *  68      FFE1 0000   64K     Area 3  512K
 *  67      FFE2 0000   64K     Area 3
 *  66      FFE3 0000   64K     Area 3
 *  65      FFE4 0000   64K     Area 3
 *  64      FFE5 0000   64K     Area 3
 *  63      FFE6 0000   64K     Area 3
 *  62      FFE7 0000   64K     Area 3
 *
 *  61      FFE8 0000   64K     Area 2  Big data area 2
 *  60      FFE9 0000   64K     Area 2  512K
 *  59      FFEA 0000   64K     Area 2
 *  58      FFEB 0000   64K     Area 2
 *  57      FFEC 0000   64K     Area 2
 *  56      FFED 0000   64K     Area 2
 *  55      FFEE 0000   64K     Area 2
 *  54      FFEF 0000   64K     Area 2
 *
 *  53      FFF0 0000   32K     Area 1  Application area 1
 *  52      FFF0 8000   32K     Area 1  256K
 *  51      FFF1 0000   32K     Area 1
 *  50      FFF1 8000   32K     Area 1
 *  49      FFF2 0000   32K     Area 1
 *  48      FFF2 8000   32K     Area 1
 *  47      FFF3 0000   32K     Area 1
 *  46      FFF3 8000   32K     Area 1
 *
 *  45      FFF4 0000   32K     Area 1  Application area 2
 *  44      FFF4 8000   32K     Area 1  256K
 *  43      FFF5 0000   32K     Area 1
 *  42      FFF5 8000   32K     Area 1
 *  41      FFF6 0000   32K     Area 1
 *  40      FFF6 8000   32K     Area 1
 *  39      FFF7 0000   32K     Area 1
 *  38      FFF7 8000   32K     Area 1
 *
 *  37..8   FFF8 0000   16K*30  Area 0  Remote monitor + boot loader
 *  7..0    FFFF 8000   4K*8    Area 0  512K
 *
 *  Access procedure
 *      reset_fcu();            FCU reset
 *      flash_init();           FCU initialize
 *      flash_get_block_info(); Obtain ROM block information to be written
 *      flash_erase_rom();      Block erase (block size)
 *      flash_write_rom();      Write (in units of 128 bytes)
 *      If continuing
 *      End
 */

//  Pointer conversion 
typedef union __pointer_union__ {
    void *              vp;
    unsigned long *     lp;
    unsigned short *    wp;
    unsigned char *     bp;
    unsigned long       val;
} MIX_POINTER;

/* ----------------------------------------------------------------------------------------
 * flash_get_block_info
 * 
 * Function description
 *     Get ROM block information 
 * 
 * Argument
 *     addr        Target address
 *     *start      Start address storage location
 *     *end        End address storage location
 *     *area_no    Area number storage location
 *     *block_no   Block number storage location
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int flash_get_block_info(unsigned long addr, unsigned long *start,
                         unsigned long *end, int *area_no, int *block_no);

/* ----------------------------------------------------------------------------------------
 * reset_fcu
 * 
 * Function description
 *     FCU module re-initialization
 * 
 * Argument
 *     None
 * 
 * Return
 *     None
 * ----------------------------------------------------------------------------------------*/
void reset_fcu(void);

/* ----------------------------------------------------------------------------------------
 * check_fstatr
 * 
 * Function description
 *     Check FCU module status check
 * 
 * Argument
 *     *peaddr     P/E address
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int check_fstatr(unsigned long peaddr);

/* ----------------------------------------------------------------------------------------
 * wait_frdy
 * 
 * Outline
 *     Wait for FCU module permission to begin processing
 * 
 * Argument
 *     timeout     Longest wait time
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int wait_frdy(unsigned long timeout);

/* ----------------------------------------------------------------------------------------
 * switch_to_read
 * 
 * Outline
 *     Change the FCU module to read mode
 * 
 * Argument
 *     *peaddr     P/E address
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int switch_to_read(unsigned long peaddr);

/* ----------------------------------------------------------------------------------------
 * notify_fclk
 * 
 * Outline 
 *     Clock setting of FCU module
 * 
 * Argument
 *     *peaddr     P/E address
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int notify_fclk(unsigned long peaddr);

/* ----------------------------------------------------------------------------------------
 * switch_to_pe
 * 
 * Outline
 *     Move the FCU module to P/E mode
 * 
 * Argument
 *     *peaddr     P/E address
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int switch_to_pe(unsigned long peaddr);

/* ----------------------------------------------------------------------------------------
 * flash_init
 * 
 * Argument
 *     None
 * 
 * Return
 *     None
 * ----------------------------------------------------------------------------------------*/
void flash_init(void);

/* ----------------------------------------------------------------------------------------
 * flash_write_rom
 * 
 * Argument
 *     dst         Write destination ROM address
 *     src         Read source RAM address
 *     len         Number of bytes written
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int flash_write_rom(unsigned long dst, const unsigned long src, int len);

/* ----------------------------------------------------------------------------------------
 * Block erase
 * 
 * Argument
 *     dst         Erase address
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int flash_erase_rom(unsigned long dst);

#endif //__RX631_FLASH_ROM__

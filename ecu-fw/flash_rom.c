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
 * LFY-RX63N1  Rewrite program ROM area
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2016/02/10  Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ----------------------------------------------------------------------------------------
 */

#include <sysio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "iodefine.h"
#include "timer.h"
#include "flash_rom.h"

/*
 *  ROM block structure (2MByte)
 *
 *  Block
 *  69      FFE0 0000   64K     Area 3   Big data area 1
 *  68      FFE1 0000   64K     Area 3   512K
 *  67      FFE2 0000   64K     Area 3
 *  66      FFE3 0000   64K     Area 3
 *  65      FFE4 0000   64K     Area 3
 *  64      FFE5 0000   64K     Area 3
 *  63      FFE6 0000   64K     Area 3
 *  62      FFE7 0000   64K     Area 3
 *
 *  61      FFE8 0000   64K     Area 2   Big data area 2
 *  60      FFE9 0000   64K     Area 2   512K
 *  59      FFEA 0000   64K     Area 2
 *  58      FFEB 0000   64K     Area 2
 *  57      FFEC 0000   64K     Area 2
 *  56      FFED 0000   64K     Area 2
 *  55      FFEE 0000   64K     Area 2
 *  54      FFEF 0000   64K     Area 2
 *
 *  53      FFF0 0000   32K     Area 1   Application area 1
 *  52      FFF0 8000   32K     Area 1   256K
 *  51      FFF1 0000   32K     Area 1
 *  50      FFF1 8000   32K     Area 1
 *  49      FFF2 0000   32K     Area 1
 *  48      FFF2 8000   32K     Area 1
 *  47      FFF3 0000   32K     Area 1
 *  46      FFF3 8000   32K     Area 1
 *
 *  45      FFF4 0000   32K     Area 1   Application area 2
 *  44      FFF4 8000   32K     Area 1   256K
 *  43      FFF5 0000   32K     Area 1
 *  42      FFF5 8000   32K     Area 1
 *  41      FFF6 0000   32K     Area 1
 *  40      FFF6 8000   32K     Area 1
 *  39      FFF7 0000   32K     Area 1
 *  38      FFF7 8000   32K     Area 1
 *
 *  37..8   FFF8 0000   16K*30  Area 0   Remote monitor + boot loader
 *   7..0   FFFF 8000    4K*8   Area 0   512K
 */

#define ROM_WRITE_TOP 0x0E000000 //  ROM writing address 

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define T_FCUR (35)
#define T_PCKA (120)
#define T_P128 (33600)
#define T_E16K (576000)

#define FCU_RAM_ADDR 0x007F8000 //  FCU dedicated RAM area 
#define FCU_FRM_ADDR 0xFEFFE000 //  FCU firmware 
#define FCU_FRM_SIZE 0x00002000 //  FW size 

static const struct {
    unsigned long   start_addr;
    unsigned long   end_addr;
    int             area_no;
} block_info[] = {
    // 4 KiB * 8 blocks 
    { 0xfffff000, 0xffffffff, 0 },
    { 0xffffe000, 0xffffefff, 0 },
    { 0xffffd000, 0xffffdfff, 0 },
    { 0xffffc000, 0xffffcfff, 0 },
    { 0xffffb000, 0xffffbfff, 0 },
    { 0xffffa000, 0xffffafff, 0 },
    { 0xffff9000, 0xffff9fff, 0 },
    { 0xffff8000, 0xffff8fff, 0 },

    // 16 KiB * 22 blocks 
    { 0xffff4000, 0xffff7fff, 0 },
    { 0xffff0000, 0xffff3fff, 0 },
    { 0xfffec000, 0xfffeffff, 0 },
    { 0xfffe8000, 0xfffebfff, 0 },
    { 0xfffe4000, 0xfffe7fff, 0 },
    { 0xfffe0000, 0xfffe3fff, 0 },
    { 0xfffdc000, 0xfffdffff, 0 },
    { 0xfffd8000, 0xfffdbfff, 0 },
    { 0xfffd4000, 0xfffd7fff, 0 },
    { 0xfffd0000, 0xfffd3fff, 0 },
    { 0xfffcc000, 0xfffcffff, 0 },
    { 0xfffc8000, 0xfffcbfff, 0 },
    { 0xfffc4000, 0xfffc7fff, 0 },
    { 0xfffc0000, 0xfffc3fff, 0 },
    { 0xfffbc000, 0xfffbffff, 0 },
    { 0xfffb8000, 0xfffbbfff, 0 },
    { 0xfffb4000, 0xfffb7fff, 0 },
    { 0xfffb0000, 0xfffb3fff, 0 },
    { 0xfffac000, 0xfffaffff, 0 },
    { 0xfffa8000, 0xfffabfff, 0 },
    { 0xfffa4000, 0xfffa7fff, 0 },
    { 0xfffa0000, 0xfffa3fff, 0 },

    // 16 KiB * 8 blocks 
    { 0xfff9c000, 0xfff9ffff, 0 },
    { 0xfff98000, 0xfff9bfff, 0 },
    { 0xfff94000, 0xfff97fff, 0 },
    { 0xfff90000, 0xfff93fff, 0 },
    { 0xfff8c000, 0xfff8ffff, 0 },
    { 0xfff88000, 0xfff8bfff, 0 },
    { 0xfff84000, 0xfff87fff, 0 },
    { 0xfff80000, 0xfff83fff, 0 },

    // 32 KiB * 8 blocks 
    { 0xfff78000, 0xfff7ffff, 1 },
    { 0xfff70000, 0xfff77fff, 1 },
    { 0xfff68000, 0xfff6ffff, 1 },
    { 0xfff60000, 0xfff67fff, 1 },
    { 0xfff58000, 0xfff5ffff, 1 },
    { 0xfff50000, 0xfff57fff, 1 },
    { 0xfff48000, 0xfff4ffff, 1 },
    { 0xfff40000, 0xfff47fff, 1 },

    // 32 KiB * 8 blocks 
    { 0xfff38000, 0xfff3ffff, 1 },
    { 0xfff30000, 0xfff37fff, 1 },
    { 0xfff28000, 0xfff2ffff, 1 },
    { 0xfff20000, 0xfff27fff, 1 },
    { 0xfff18000, 0xfff1ffff, 1 },
    { 0xfff10000, 0xfff17fff, 1 },
    { 0xfff08000, 0xfff0ffff, 1 },
    { 0xfff00000, 0xfff07fff, 1 },

    // 64 KiB * 8 blocks 
    { 0xffef0000, 0xffefffff, 2 },
    { 0xffee0000, 0xffeeffff, 2 },
    { 0xffed0000, 0xffedffff, 2 },
    { 0xffec0000, 0xffecffff, 2 },
    { 0xffeb0000, 0xffebffff, 2 },
    { 0xffea0000, 0xffeaffff, 2 },
    { 0xffe90000, 0xffe9ffff, 2 },
    { 0xffe80000, 0xffe8ffff, 2 },

    // 64 KiB * 8 blocks 
    { 0xffe70000, 0xffe7ffff, 3 },
    { 0xffe60000, 0xffe6ffff, 3 },
    { 0xffe50000, 0xffe5ffff, 3 },
    { 0xffe40000, 0xffe4ffff, 3 },
    { 0xffe30000, 0xffe3ffff, 3 },
    { 0xffe20000, 0xffe2ffff, 3 },
    { 0xffe10000, 0xffe1ffff, 3 },
    { 0xffe00000, 0xffe0ffff, 3 }
};
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
                         unsigned long *end, int *area_no, int *block_no)
{
    int i;

    for (i = 0; i < sizeof(block_info) / sizeof(block_info[0]); i++) {
        if (addr >= block_info[i].start_addr && addr <= block_info[i].end_addr) {
            if (start != NULL) {
                *start = block_info[i].start_addr;
            }
            if (end != NULL) {
                *end = block_info[i].end_addr;
            }
            if (area_no != NULL) {
                *area_no = block_info[i].area_no;
            }
            if (block_no != NULL) {
                *block_no = i;
            }
            return 0;
        }
    }
    // Outside ROM area 
    return -1;
}

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
void reset_fcu(void)
{
    // forcibly terminate programming/erasure operations 
    FLASH.FRESETR.WORD = 0xCC01;

    // Wait 35usec 
    cmt1_start(T_FCUR * CMT1_1US, 0); // * 1usec 
    while (cmt1_ni_check() != 0) {
        ;
    }

    // release reset state 
    FLASH.FRESETR.WORD = 0xCC00;
}

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
int check_fstatr(unsigned long peaddr)
{
    MIX_POINTER p;
    p.val = peaddr;

    if (FLASH.FSTATR1.BIT.FCUERR) { // FCU error 
        reset_fcu();                // Reinitialization 
        return -1;
    }

    if (FLASH.FSTATR0.BIT.ILGLERR) {  // Command error 
        if (FLASH.FASTAT.BIT.CMDLK) { /* Command lock release FASTAT.ROMAE / DLFAE /
                                       * DFLRPE  / DFLWPE*/
            FLASH.FASTAT.BYTE = 0x10;
        }
        *p.lp = 0x50; // Status clear command 
        return -1;
    }

    if (FLASH.FSTATR0.BIT.ERSERR || FLASH.FSTATR0.BIT.PRGERR) {
        // Erase or write error 
        *p.lp = 0x50; // Status clear command 
        return -1;
    }
    // Normal 
    return 0;
}

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
int wait_frdy(unsigned long timeout)
{
    int frdy = 0;

    timeout = (timeout + 99) / 100; // Round to 100usec 
    while (timeout-- > 0 && frdy == 0) {
        cmt1_start(100 * CMT1_1US, 0); // 100*1usec 
        while (cmt1_ni_check() != 0) {
            if (FLASH.FSTATR0.BIT.FRDY) {
                frdy = 1;
                break;
            }
        }
    }
    return frdy ? 0 : -1;
}

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
int switch_to_read(unsigned long peaddr)
{
    int ret;

    // Wait for permission 
    if (wait_frdy((unsigned long)(T_E16K)) != 0) { // Timeout 
        reset_fcu();
        return -1;
    }

    // Error confirmation 
    ret = check_fstatr(peaddr);

    // FENTRYR.FENTRY0..3 Clear * / 
    FLASH.FENTRYR.WORD = 0xAA00;
    while (FLASH.FENTRYR.WORD != 0x0000) {
        ;
    }

    // Prohibit 
    FLASH.FWEPROR.BIT.FLWE = 2; /* 0,2,3 = Disable (DEF) / 1 = P/E, lock bit P/E,
                                 * lock bit read, blank check enabled*/
    return ret;
}

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
int notify_fclk(unsigned long peaddr)
{
    MIX_POINTER p;
    p.val = peaddr;
    // 96MHz/4 : FCLK = 24MHz 
    FLASH.PCKAR.BIT.PCKA = 24;

    /* write 0xE9 to the destination address
     * for peripheral clock notification */
    *p.bp = 0xE9;

    /* write 0x03 to the destination address
     * for peripheral clock notification */
    *p.bp = 0x03;

    /* write 0x0f0f to the destination address
     * for peripheral clock notification three times(as a word) */
    *p.wp   = 0x0F0F;
    *p.wp   = 0x0F0F;
    *p.wp   = 0x0F0F;

    /* write 0xd0 to the destination address
     * for peripheral clock notification */
    *p.bp = 0xD0;

    // FRDY bit check(tWAIT = tPCKA) 
    if (wait_frdy((unsigned long)T_PCKA) != 0) { // Timeout 
        reset_fcu();
        return -1;
    }
    // Error check 
    return check_fstatr(peaddr);
}

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
int switch_to_pe(unsigned long peaddr)
{
    static int  fclk_notified = 0;
    int         area_no;

    // Area information acquisition 
    if (flash_get_block_info(peaddr | 0xff000000, NULL, NULL, &area_no, NULL) != 0) {
        return -1;
    }

    // P/E permission 
    FLASH.FWEPROR.BIT.FLWE = 1; /* 0,2,3 = Disable (DEF) / 1 = P/E, lock bit P/E,
                                 * lock bit read, blank check enabled*/

    // Area bit clear 
    FLASH.FENTRYR.WORD = 0xAA00;
    while (FLASH.FENTRYR.WORD != 0x0000) {
        ;
    }

    // Area permission setting 
    FLASH.FENTRYR.WORD = 0xAA00 | (0x0001 << area_no);

    // Error check 
    if (check_fstatr(peaddr)) {
        return -1;
    }

    // Check clock non-setting 
    if (!fclk_notified) { // FCLK setting 
        if (notify_fclk(peaddr)) {
            return -1;
        }
        fclk_notified = 1;
    }
    return 0;
}

/* ----------------------------------------------------------------------------------------
 * flash_init
 * 
 * Argument
 *     None
 * 
 * Return
 *     None
 * ----------------------------------------------------------------------------------------*/
void flash_init(void)
{
    // Lock bit read by memory area read method (1) 
    FLASH.FMODR.BIT.FRDMD = 0;

    // Disable ready interrupt 
    FLASH.FRDYIE.BIT.FRDYIE = 0;

    // Disable error interrupt 
    FLASH.FAEINT.BYTE = 0x00;

    // Cancel area selection 
    FLASH.FENTRYR.WORD = 0xAA00;

    // Write protect 
    FLASH.FWEPROR.BIT.FLWE = 0;

    // E2 access setting 
    FLASH.DFLRE0.WORD   = 0x2DFF; // E2 read  enable / disable 
    FLASH.DFLRE1.WORD   = 0xD2FF; // E2 read  enable / disable 
    FLASH.DFLWE0.WORD   = 0x1EFF; // E2 write enable / disable 
    FLASH.DFLWE1.WORD   = 0xE1FF; // E2 write enable / disable 

    // Permission to use FCU-RAM 
    FLASH.FCURAME.WORD = 0xC401;

    // Copy FCU-FW from ROM to RAM 
    memcpy((void *)FCU_RAM_ADDR, (void *)FCU_FRM_ADDR, FCU_FRM_SIZE);
}

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
int flash_write_rom(unsigned long dst, const unsigned long src, int len)
{
    MIX_POINTER     ra, wa;
    unsigned long   npad_lead, npad_trail;
    unsigned long   l, m;
    unsigned short *p   = (unsigned short *)src;
    int             odd = 0;

    if (((unsigned long)dst & 0xFF000000) != 0xFF000000) {
        return -1; // Out of ROM range 

    }
    // Write every 128 bytes 
    while (len > 0) {
        // Write address 
        ra.val  = dst & 0x00FFFFFF;     // Write address 
        wa.val  = ra.val & ~0x0000007F; // Mask lower 128 bytes 

        // Number of first FF bytes 
        m = MIN(
                    128 - (ra.val - wa.val),
                    len // Lesser    m = 1..128  Number of bytes not written 
        );
        l           = m;
        npad_lead   = ra.val - wa.val;       // Fraction of first write byte 0..127 
        npad_trail  = 128 - (npad_lead + m); // Number of trailing FF bytes  0..127 

        // Start P/E mode 
        if (switch_to_pe(wa.val) != 0) {
            return -1;
        }

        // Lock bit release 
        FLASH.FPROTR.WORD = 0x5501;
        // Command 
        *wa.bp  = 0xE8; // 1st 
        *wa.bp  = 0x40; // 2nd 

        // Top FF 
        while (npad_lead > 1) {
            *wa.wp      = 0xFFFF;
            npad_lead   -= 2;
        }
        if (npad_lead == 1 && m >= 1) { // Fractional bytes 
            *wa.wp = (*p << 8) | 0x00FF;
            m--;
            odd = 1;
        }

        if (odd) { // Odd word 
            while (m > 1) {
                *wa.wp = (*p >> 8) | (*(p + 1) << 8);
                p++;
                m -= 2;
            }
        } else { // Word transfer 
            while (m > 1) {
                *wa.wp  = *p++;
                m       -= 2;
            }
        }

        // Last odd byte 
        if (m == 1) {
            *wa.wp = *p++ | 0xFF00;
            npad_trail--;
        }
        // Rear FF 
        while (npad_trail > 0) {
            *wa.wp      = 0xFFFF;
            npad_trail  -= 2;
        }
        // End 
        *wa.bp = 0xD0; // 67th 

        // Wait for processing completion 
        if (wait_frdy((unsigned long)(T_P128 * 11 / 10)) != 0) { // Timeout 
            reset_fcu();
            return -1;
        }

        // Read mode 
        if (switch_to_read(wa.val) != 0) {
            return -1;
        }

        dst += l;
        len -= l;
    }
    return 0;
}

/* ----------------------------------------------------------------------------------------
 * Block erase
 * 
 * Argument
 *     dst         Erase address
 * 
 * Return
 *     int         0: Normal / Other: Error
 * ----------------------------------------------------------------------------------------*/
int flash_erase_rom(unsigned long dst)
{
    MIX_POINTER ba;

    if ((dst & 0xFF000000) != 0xFF000000) {
        return -1; // Out of ROM range 

    }
    // Erase address 
    ba.val = dst & 0x00FFFFFF;

    // P/E mode 
    if (switch_to_pe(ba.val) == -1) {
        return -1;
    }

    // Lock bit release 
    FLASH.FPROTR.WORD = 0x5501;
    // Command 
    *ba.bp  = 0x20; // 1st 
    *ba.bp  = 0xD0; // 2nd     Start erasing 

    // Waiting for completion 
    if (wait_frdy((unsigned long)(T_E16K * 11 / 10)) != 0) { // Timeout 
        reset_fcu();
        return -1;
    }

    // Read mode 
    return switch_to_read(ba.val);
}

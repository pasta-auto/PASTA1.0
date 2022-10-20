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

/* -*-c++-*-
 * $RCSfile: rtc.c,v $
 * $Revision: 1.2 $
 * $Date: 2016/03/07 09:08:29 $
 *
 * Copyright (c) 2015 LandF Corporation.
 *
 * History:
 */

/*
 * System definition
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sysio.h>
/*
 * User definition
 *
 * CPU register definition*/
#include "iodefine.h"
#include "macros.h"
#include "altypes.h"
#include "libs.h"
#include "cmnsys.h"
#include "timer.h"
#include "rtc.h"

#define INT_REQ_BASE 0x87000 // Interrupt request register address 
#define INT_REQ(a)   *((volatile unsigned char *)(INT_REQ_BASE+(a)))
#define RTC_CUP      62
#define RTC_ALM      92
#define RTC_PRD      93

/* ----------------------------------------------------------------------------------------
 * no_use_subclk_as_sysclk
 *   
 *  Outline
 *      Processing when not using the sub-clock as the system clock.
 * 
 *  Description  : Set the SOSTP bit to 1 (sub-clock stops)
 *                 when the sub-clock is used only as the RTC count source.
 *  
 *  Argument
 *      None 
 *  
 *  Return
 *      None 
* ---------------------------------------------------------------------------------------- */
static void no_use_subclk_as_sysclk(void)
{
    // Disable protection 
    SYSTEM.PRCR.WORD = 0xA503;

    // ---- Stop the Sub-clock oscillator ---- 
    /* SOSCCR - Sub-Clock Oscillator Control Register
     * b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is stopped. */
    SYSTEM.SOSCCR.BYTE = 0x01;
    while (0x01 != SYSTEM.SOSCCR.BYTE) {
        // Confirm that the written value can be read correctly. 
    }

    // Enable protection 
    SYSTEM.PRCR.WORD = 0xA500;
}

/* ----------------------------------------------------------------------------------------
 * enable_RTC
 *  
 *  Outline
 *      Initialization when using the RTC
 *  
 *  Argument
 *      None 
 *  
 *  Return
 *      None 
 * ---------------------------------------------------------------------------------------- */
static void enable_RTC(void)
{
    uint8_t             i;
    volatile uint8_t    dummy;

    // ---- Set RCR3 register ---- 
    /* RCR3 - RTC Control Register 3
     * b0       RTCEN    - Sub-clock oscillator is running. */
    RTC.RCR3.BIT.RTCEN = 1;

    // dummy read three times 
    for (i = 0; i < 3; i++) {
        dummy = RTC.RCR3.BYTE;
    }

    while (!RTC.RCR3.BIT.RTCEN) {
        // Confirm that the written value can be read correctly. 
    }

    // ---- Stop prescaler and counter ---- 
    /* RCR2 - RTC Control Register 2
     * b7       Reserved - The write value should be 0.
     * b0       START    - start - Prescaler is stopped. */
    RTC.RCR2.BYTE &= 0x7E;
    while (0 != (RTC.RCR2.BYTE & 0x01)) {
        // Confirm that the written value can be read correctly. 
    }

    // ---- RTC Software Reset ---- 
    /* RCR2 - RTC Control Register 2
     * b1    RESET     - RTC Software Reset
     *                 - The prescaler and target registers are reset by RTC software reset.*/
    RTC.RCR2.BYTE |= 0x02;
    while (0 != (RTC.RCR2.BYTE & 0x02)) {
        // Confirm that the written value can be read correctly. 
    }

    // ---- Set RCR3 register ---- 
    /* RCR3 - RTC Control Register 3
     * b7:b4    Reserved - The write value should be 0.
     * b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL.
     * b0       RTCEN    - Sub-clock oscillator is running. */
    RTC.RCR3.BYTE = 0x0D;

    for (i = 0; i < 3; i++) {
        dummy = RTC.RCR3.BYTE;
    }

    while (0x0D != RTC.RCR3.BYTE) {
        // Confirm that the written value can be read correctly. 
    }
}

/* ----------------------------------------------------------------------------------------
 * oscillation_subclk
 *  
 *  Outline
 *      Configure sub-clock oscillation
 *  
 *  Argument
 *      None 
 *  
 *  Return
 *      None 
 * ---------------------------------------------------------------------------------------- */
static void oscillation_subclk(void)
{
    uint8_t             i;
    volatile uint8_t    dummy;
    int                 wait;

    // Disable protection 
    SYSTEM.PRCR.WORD = 0xA503;

    // ---- Stop the Sub-clock oscillator ---- 
    /* SOSCCR - Sub-Clock Oscillator Control Register
     * b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is stopped. */
    SYSTEM.SOSCCR.BYTE = 0x01;
    while (0x01 != SYSTEM.SOSCCR.BYTE) {
        // Confirm that the written value can be read correctly. 
    }
    /* RCR3 - RTC Control Register 3
     * b0        RTCEN    - Sub-clock oscillator is stopped. */
    RTC.RCR3.BIT.RTCEN = 0;

    // dummy read three times 
    for (i = 0; i < 3; i++) {
        dummy = RTC.RCR3.BYTE;
    }

    while (0 != RTC.RCR3.BIT.RTCEN) {
        // Confirm that the written 
    }

#if defined(_use_cmt_)
    // ---- Wait for five sub-clock cycles ---- 
    wait_timer(1, 2); // 200us 
    do {} while(check_timer(1) == 0);
#else
    wait = 2088; // 152(us)/(0.0104(us)*7) 
    do {} while(--wait);
#endif // if defined(_use_cmt_)
    // ---- Setting of the sub-clock drive strength ---- 
    /* RCR3 - RTC Control Register 3
     * b3:b1     RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL.
     * b0        RTCEN    - Sub-clock oscillator is stopped. */
    RTC.RCR3.BYTE = 0x0C;

    // dummy read three times 
    for (i = 0; i < 3; i++) {
        dummy = RTC.RCR3.BYTE;
    }

    while (0x0C != RTC.RCR3.BYTE) {
        // Confirm that the written 
    }

    // ---- Set wait time until the sub-clock oscillator stabilizes ---- 
    /* SOSCWTCR          - Sub-Clock Oscillator Wait Control Register
     * b7:b5    Reserved - The write value should be 0.
     * b4:b0    SSTS     - Sub-Clock Oscillator Waiting Time - Waiting time is 2 cycles. */
    // Investigate the problem that RTC RESET is not released in LFY # 2 
    SYSTEM.SOSCWTCR.BYTE = 0x0a; // Wait time is 16384 sub-clock cycles (approx. 500 ms). 

    // ---- Operate the Sub-clock oscillator ---- 
    /* SOSCCR - Sub-Clock Oscillator Control Register
     * b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is running. */
    SYSTEM.SOSCCR.BYTE = 0x00;
    while (0x00 != SYSTEM.SOSCCR.BYTE) {
        // Confirm that the written 
    }

#if defined(_use_cmt_)
    // ---- Wait processing for the clock oscillation stabilization ---- 
    wait_timer(1, 26000); // 26000 * 100us = 2.6s 
    do {} while(check_timer(1) == 0);
    /*
     * When the oscillation stabilization time of the crystal oscillator is 2s, the subclock start time is the sum of +500 ms (SOSCWTCR.SSTS setting value 16384 / 32.768 = 500) and the following value
     * wait_timer(1, 31000);     // (26000+500) * 100us = 3.1s
     * Since there is enough time until the sub clock is used,
     * the sub clock stabilization wait time is delayed by 2.6s.
     */
#else // if defined(_use_cmt_)
    wait = 35714286; // 2.6(s)=2600000(us)/0.0104*7 
    do {} while(--wait);
#endif // if defined(_use_cmt_)

    // Enable protection 
    SYSTEM.PRCR.WORD = 0xA500;
}

/* ----------------------------------------------------------------------------------------
 * CGC_subclk_as_RTC
 *  
 *  Outline
 *      Sub-clock pattern D
 * 
 *  Header
 *      r_init_clock.h
 * 
 *  Description
 *      Configure setting when the sub-clock is used
 *      as the RTC count source and not used as the system clock.
 *  
 *  Argument
 *      None 
 *  
 *  Return
 *      None 
 *  
 *  Ref.
 *      r01an1245jj0110_rx63n.pdf RX63N    Initial setting example
 *                                         When using sub clock only for RTC
 * ---------------------------------------------------------------------------------------- */
void subclk_as_RTC(void)
{
    uint8_t             i;
    volatile uint8_t    dummy;

    // ---- setting of RTC count source ---- 
    /* RCR4            - RTC Control Register 4
     * b7:b1  Reserved - The write value should be 0.
     * b0     RCKSEL   - Count Source Select - RTC count source use the sub-clock */
    RTC.RCR4.BIT.RCKSEL = 0;

    // dummy read three times 
    for (i = 0; i < 3; i++) {
        dummy = RTC.RCR4.BYTE;
    }

    // ---- Setting of the sub-clock oscillation ---- 
    oscillation_subclk();

    // ---- When using the RTC ---- 
    enable_RTC();

    // ---- Setting of the sub-clock do not use the system clock--- 
    no_use_subclk_as_sysclk();
}
/* ----------------------------------------------------------------------------------------
 * rtc_init
 *  
 *  Outline
 *      RTC initial setting
 * 
 *  Arguments
 *    time_bcd_t *tm
 *  
 *  Return
 *      None 
 * ---------------------------------------------------------------------------------------- */
void rtc_init(time_bcd_t *tm)
{
    uint8_t             i;
    volatile uint8_t    dummy;

    // ==== Counters and prescaler are stopped ==== 
    RTC.RCR2.BYTE &= 0x7E;
    while (0 != (RTC.RCR2.BYTE & 0x01)) {
        // Confirm that the written value can be read correctly. 
    }

    // ==== Set the time ==== 
    RTC.RSECCNT.BYTE    = tm->second;   // Set the BCD-coded value: 00 second 
    RTC.RMINCNT.BYTE    = tm->minute;   // Set the BCD-coded value: 00 minute 
    RTC.RHRCNT.BYTE     = tm->hour;     // Set the BCD-coded value: 00 hour 
    RTC.RWKCNT.BYTE     = tm->dayweek;  // Set the day of the week: Tuesday 
    RTC.RDAYCNT.BYTE    = tm->day;      // Set the BCD-coded value: 01 day 
    RTC.RMONCNT.BYTE    = tm->month;    // Set the BCD-coded value: 01 month 
    RTC.RYRCNT.WORD     = tm->year;     // Set the BCD-coded value: 13 year 

    RTC.RCR2.BYTE |= 0x40;              // Hours Mode: 24-hour mode 
    for (i = 0; i < 3; i++) {           // Dummy read three times 
        dummy = RTC.RCR2.BYTE;
    }

    // ==== Set the interrupt ==== 
    RTC.RCR1.BYTE = 0x00;               // Disable RTC interrupt requests 
    while (0x00 != RTC.RCR1.BYTE) {
        // Confirm that the written value can be read correctly. 
    }

    INT_REQ(RTC_CUP)    = 0;            // Clear RTC.CUP interrupt request 
    INT_REQ(RTC_ALM)    = 0;            // Clear RTC.ALM interrupt request 
    INT_REQ(RTC_PRD)    = 0;            // Clear RTC.PRD interrupt request 

    RTC.RCR1.BYTE = 0xD2;               /* A carry interrupt request is enabled
                                         * Periodic interrupt is generated every 1/2 second*/
    while (0xD2 != RTC.RCR1.BYTE) {
        // Confirm that the written value can be read correctly. 
    }

    // ==== Counters and prescaler operate normally ==== 
    RTC.RCR2.BYTE |= 0x01;
    while (1 != (RTC.RCR2.BYTE & 0x01)) {
        // Confirm that the written value can be read correctly. 
    }
}

/* ----------------------------------------------------------------------------------------
 * rtc_time_read
 *  
 *  Outline
 *      RTC time data reading
 *  
 *  Description
 *      Reads the time information into to global variables 
 *  
 *  Argument
 *      None 
 *   
 *  Return
 *      None 
 * ---------------------------------------------------------------------------------------- */
void rtc_time_read(time_bcd_t *tm)
{
    do{
        INT_REQ(RTC_CUP) = 0;                   // Clear COUNTUP interrupt request 
        tm->second  = RTC.RSECCNT.BYTE;         // Read the BCD-code second 
        tm->minute  = RTC.RMINCNT.BYTE;         // Read the BCD-code minute 
        tm->hour    = RTC.RHRCNT.BYTE;          // Read the BCD-coded hour 
        tm->dayweek = RTC.RWKCNT.BYTE;          // Read the day of the week 
        tm->day     = RTC.RDAYCNT.BYTE;         // Read the BCD-coded day 
        tm->month   = RTC.RMONCNT.BYTE;         // Read the BCD-coded month 
        tm->year    = 0x2000 | RTC.RYRCNT.WORD; // Read the BCD-coded year 
    }while (0 != INT_REQ(RTC_CUP));
}

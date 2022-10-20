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
 *  LFY-RX63N1  CMT Timer processing
 *
 * ----------------------------------------------------------------------------------------
 *  Development history
 *
 *  2016/02/10  Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 *  T.Tachibana
 *  L&F
 * ----------------------------------------------------------------------------------------
 */

#include    <string.h>
#include    "iodefine.h"
#include    "timer.h"

//  1msec unit general-purpose timer 
int             cmt0_time[TIMER_AN]; // Down counter 
int             cmt0_tupf[TIMER_AN]; // Time up flag 
TIMER_CALL      cmt0_call[TIMER_AN]; // Post-up call function 
unsigned int    freerun_timer;       // Free run timer 

//  1usec unit short wait timer 
int         cmt1_tupf;              // Time up flag 
int         cmt1_msec;              // msec order remaining counter 
int         cmt1_last;              // Final count value 
int         cmt1_time;              // Integration time 
TIMER_CALL  cmt1_call;              // Post-up call function 

/* ----------------------------------------------------------------------------------------
 *  cmt0_init
 * 
 *   Function description
 *       CMT0 initialization
 *  
 *   Argument
 *       None
 *  
 *   Return
 *       None
 * ----------------------------------------------------------------------------------------*/
void cmt0_init(void)
{
    freerun_timer = 0;
    memset(cmt0_time, 0, sizeof(cmt0_time));
    memset(cmt0_tupf, 0, sizeof(cmt0_tupf));
    memset(cmt0_call, 0, sizeof(cmt0_call));
    SYSTEM.PRCR.WORD        = 0xA502; // Release of CLK related register protection 
    ICU.IER[0x03].BIT.IEN4  = 1;      // 0=Disable 1=Interrupt enable 
    ICU.IPR[004].BIT.IPR    = 7;      // Interrupt level (15 is the highest) 
    MSTP_CMT0               = 0;      // CMT unit 0 Module power ON 
    CMT0.CMCR.BIT.CKS       = 0;      // 0=PCLK/8 1=PCLK/32 2=PCLK/128 3=PCLK/512 
    CMT0.CMCR.BIT.CMIE      = 1;      // Enable interrupt 
    CMT0.CMCOR              = CMT0_COUNT_VAL; // Interrupt every 1msec 
    CMT.CMSTR0.BIT.STR0     = 1;      // 0=Stop, 1=Start 
}

/* ----------------------------------------------------------------------------------------
 * start_timer
 * 
 *  Function description
 *      Start timer count
 *  
 *  Argument
 *      tch   Channel number
 *      interval Wait time setting in 1msec units
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void start_timer(int tch, int interval )
{
    cmt0_time[tch]  = interval;
    cmt0_tupf[tch]  = 0;
    cmt0_call[tch]  = 0;
}

/* ----------------------------------------------------------------------------------------
 * after_call
 * 
 *  Function description
 *      Start timer count
 *  
 *  Argument
 *      tch      Channel number
 *      interval Wait time setting in 1msec units
 *      *proc    Callee after up
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void after_call(int tch, int interval, void *proc )
{
    if (interval < 0) { // High speed call 
        cmt0_time[tch]  = -interval;
        cmt0_tupf[tch]  = -1;
        cmt0_call[tch]  = (TIMER_CALL)proc;
    } else { // Low speed call 
        cmt0_time[tch]  = interval;
        cmt0_tupf[tch]  = 0;
        cmt0_call[tch]  = (TIMER_CALL)proc;
    }
}

/* ----------------------------------------------------------------------------------------
 * check_timer
 * 
 *  Function description
 *      Confirmation of designated timer up
 *  
 *  Argument
 *      tch   Channel number
 *  
 *  Return
 *      int   Remaining time (msec)
 * ----------------------------------------------------------------------------------------*/
int check_timer(int tch)
{
    return (cmt0_time[tch] > 0) ? 0 : 1;
}

/* ----------------------------------------------------------------------------------------
 * stop_timer
 * 
 *  Function description
 *      Stop specified timer
 *  
 *  Argument
 *      tch   Channel number
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void stop_timer(int tch)
{
    cmt0_time[tch] = 0;
}

/* ----------------------------------------------------------------------------------------
 * wait
 * 
 *  Function description
 *      Wait for specified msec
 *  
 *  Argument
 *      msec     Waiting time
 *      *loop    Function call during wait time
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void wait(int msec, void *loop)
{
    TIMER_CALL p = (TIMER_CALL)loop;
    cmt0_time[(TIMER_AN-1)] = msec;
    while (cmt0_time[(TIMER_AN-1)] > 0) {
        if (p != 0) {
            p(); // Execute during waiting 
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * cmt0_job
 *
 *  Function description
 *      Function call of time-up timer
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void cmt0_job(void)
{
    int         c;
    TIMER_CALL  p;

    for (c = 0; c < TIMER_AN; c++) {
        if (cmt0_tupf[c] == 1) { // Timeup 
            cmt0_tupf[c]    = 0;
            p               = cmt0_call[c];
            if (p != 0) { // Function call 
                p();
            }
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * cmt0_int
 * 
 *  Function description
 *      CMT0 interrupt (1msec)
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void interrupt __vectno__ {VECT_CMT0_CMI0} cmt0_int(void)
{
    int         c;
    TIMER_CALL  p;

    freerun_timer++;    // 1msec free run counter 

    for (c = 0; c < TIMER_AN; c++) {
        if (cmt0_time[c] > 0) { // With remaining count 
            cmt0_time[c]--;
            if (cmt0_time[c] == 0) { // Timeup 
                p = cmt0_call[c];
                if (p != 0 && cmt0_tupf[c] < 0) { // Function call 
                    p();
                } else {
                    cmt0_tupf[c] = 1;
                }
            }
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * cmt1_init
 * 
 *  Function description
 *      CMT1 initialization
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void cmt1_init(void)
{
    cmt1_tupf                                   = 0;
    cmt1_last                                   = 0;
    cmt1_msec                                   = 0;
    cmt1_call                                   = 0;
    cmt1_time                                   = 0;
    SYSTEM.PRCR.WORD                            = 0xA502; // Release of CLK related register protection 
    ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1    = 0;   // 0=Disabled 1=Interrupt enabled 
    ICU.IPR[IPR_CMT1_CMI1].BIT.IPR              = 0;   // Interrupt level 0 =Disable / 1 to 15 =Enable 
    CMT.CMSTR0.BIT.STR1                         = 0;   // 0=Stop, 1=Start 
    MSTP_CMT1                                   = 0;   // CMT unit 1 Module power ON 
    CMT1.CMCR.BIT.CKS                           = 0;   // PCLK=48MHz : 0=PCLK/8 1=PCLK/32 2=PCLK/128 3=PCLK/512 
    CMT1.CMCR.BIT.CMIE                          = 0;   // Interrupt 1: Enable / 0: Disable 
    CMT1.CMCOR                                  = 0xFFFF; // 6MHz / 65536 
    CMT1.CMCNT                                  = 0;   // Counter 
}

/* ----------------------------------------------------------------------------------------
 * cmt1_start
 * 
 *  Function description
 *      CMT1 timing started (1usec)
 *  
 *  Argument
 *      count    Waiting time 0 to 10000 : 10.000ms : 0.01sec
 *      *proc    Callee after up
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void cmt1_start(int count, void *proc)
{
    cmt1_tupf   = 0;
    cmt1_time   = 0;
    cmt1_msec   = count / 10000;
    cmt1_last   = count - cmt1_msec * 10000;
    if (cmt1_msec > 0) {
        count = 10000;
    }
    cmt1_call                                   = (TIMER_CALL)proc;
    CMT.CMSTR0.BIT.STR1                         = 0; // 0=Stop, 1=Start 
    ICU.IR[IR_CMT1_CMI1].BIT.IR                 = 0; // Clear interrupt flag 
    CMT1.CMCR.BIT.CKS                           = 0; // PCLK=48MHz : 0=PCLK/8 1=PCLK/32 2=PCLK/128 3=PCLK/512 
    CMT1.CMCR.BIT.CMIE                          = 1; // Interrupt 1:Enable / 0:Disable 
    CMT1.CMCOR                                  = count * CMT1_1US - 1; // 6MHz/count 
    CMT1.CMCNT                                  = 0; // Counter 
    ICU.IPR[IPR_CMT1_CMI1].BIT.IPR              = 1; // Interrupt level   0=Disable / 1 to 15=Enable 
    ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1    = 1; // 0=Disable 1=Interrupt enable 
    CMT.CMSTR0.BIT.STR1                         = 1; // 0=Stop, 1=Start 
}

/* ----------------------------------------------------------------------------------------
 * cmt1_check
 * 
 *  Function description
 *      CMT1 time up confirmation
 *  
 *  Argument
 *      None
 * 
 *  Return
 *      int  0=Stopping / 1=Timekeeping
 * ----------------------------------------------------------------------------------------*/
int cmt1_ni_check(void)
{
    int count;
    if (CMT.CMSTR0.BIT.STR1 != 0) {
        if (ICU.IR[IR_CMT1_CMI1].BIT.IR != 0) { // Count up 
            CMT.CMSTR0.BIT.STR1         = 0;    // 0=Stop, 1=Start 
            ICU.IR[IR_CMT1_CMI1].BIT.IR = 0;    // Interrupt flag clear 
            if (cmt1_msec > 0) {
                cmt1_msec--;
                if (cmt1_msec > 0) {
                    count = 10000;
                } else {
                    count = cmt1_last;
                }
                CMT1.CMCR.BIT.CKS   = 0;         // PCLK=48MHz : 0=PCLK/8 1=PCLK/32 2=PCLK/128 3=PCLK/512 
                CMT1.CMCR.BIT.CMIE  = 1;         // Interrupt 1:Enable / 0:Disable 
                CMT1.CMCOR          = count * CMT1_1US - 1; // 6MHz/count 
                CMT1.CMCNT          = 0;         // Counter 
                CMT.CMSTR0.BIT.STR1 = 1;         // 0=Stop, 1=Start 
            } else {
                cmt1_tupf++;
                if (cmt1_call != 0) {
                    cmt1_call();
                }
                ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1    = 0; // 0=Disable 1=Interrupt enable 
                ICU.IPR[IPR_CMT1_CMI1].BIT.IPR              = 0; // Interrupt level   0=Disable / 1 to 15=Enable 
            }
        }
    }
    return (CMT.CMSTR0.BIT.STR1); // 0=Stop / 1=Time 
}
int cmt1_check(void)
{
    return (CMT.CMSTR0.BIT.STR1); // 0=Stop / 1=Time 
}

/* ----------------------------------------------------------------------------------------
 * cmt1_stop
 * 
 *  Function description
 *      Stop CMT1 and get elapsed time
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      Elapsed time (1us unit)
 * ----------------------------------------------------------------------------------------*/
int cmt1_stop(void)
{
    if (CMT.CMSTR0.BIT.STR1) {
        CMT.CMSTR0.BIT.STR1                         = 0; // 0=Stop, 1=Start 
        ICU.IR[IR_CMT1_CMI1].BIT.IR                 = 0; // Interrupt flag clear 
        ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1    = 0; // 0=Disable 1=Interrupt enable 
        ICU.IPR[IPR_CMT1_CMI1].BIT.IPR              = 0; // Interrupt level   0=Disable / 1 to 15=Enable 
        cmt1_time                                   += CMT1.CMCNT / CMT1_1US;
    }
    return cmt1_time;
}

/* ----------------------------------------------------------------------------------------
 *  cmt1_int
 * 
 *  Function description
 *      CMT1 interrupt(1usec)
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void interrupt __vectno__ {VECT_CMT1_CMI1} cmt1_int(void)
{
    int count;
    CMT.CMSTR0.BIT.STR1         = 0; // 0=Stop, 1=Start 
    ICU.IR[IR_CMT1_CMI1].BIT.IR = 0; // Interrupt flag clear 
    cmt1_time                   += ((int)CMT1.CMCOR + 1) / CMT1_1US;
    if (cmt1_msec > 0) {
        cmt1_msec--;
        if (cmt1_msec > 0) {
            count = 10000;
        } else {
            count = cmt1_last;
        }
        CMT1.CMCR.BIT.CKS   = 0;     // PCLK=48MHz : 0=PCLK/8 1=PCLK/32 2=PCLK/128 3=PCLK/512 
        CMT1.CMCR.BIT.CMIE  = 1;     // Interrupt 1:Enable / 0:Disable 
        CMT1.CMCOR          = count * CMT1_1US - 1; // 6MHz/count 
        CMT1.CMCNT          = 0;     // Counter 
        CMT.CMSTR0.BIT.STR1 = 1;     // 0=Stop, 1=Start 
    } else {
        ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1    = 0; // 0=Disable 1=Interrupt enable 
        ICU.IPR[IPR_CMT1_CMI1].BIT.IPR              = 0; // Interrupt level   0=Disable / 1 to 15=Enable 
        cmt1_tupf++;
        if (cmt1_call != 0) {
            cmt1_call();
        }
    }
}

/* ----------------------------------------------------------------------------------------
 * swait
 * 
 *  Function description
 *      Short wait wait for usec
 *  
 *  Argument
 *      usec     Waiting time
 *      *loop    Function call during wait time
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void swait(int usec, void *loop)
{
    int         ms  = usec / 10000;
    TIMER_CALL  p   = (TIMER_CALL)loop;
    for (; ms > 0 && usec > 65535; ms--) {
        usec -= 10000;
        cmt1_start(10000, 0);
        while (cmt1_tupf == 0) {
            if (p != 0) {
                p(); // Execute during waiting 
            }
        }
    }
    cmt1_start(usec, 0);
    while (cmt1_tupf == 0) {
        if (p != 0) {
            p();    // Execute during waiting 
        }
    }
}

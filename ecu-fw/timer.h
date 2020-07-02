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
 * ________________________________________________________________________________________
 */

//#include "timer.h" 

#ifndef __LFY_RX63N1_CMT_TIMER__
#define __LFY_RX63N1_CMT_TIMER__

#define     CMT1_1US    6                       // 1usec = CMT1.CLK = PCLK / 8 = 6MHz = 6count = 1usec 

//  Indirect call prototype (1 argument) 
typedef void (*TIMER_CALL)(void);

#define     CMT0_COUNT_NUM  6000                // 1ms timer setting 
#define     CMT0_COUNT_VAL  CMT0_COUNT_NUM-1    // 1ms timer setting value 
#define     TIMER_AN    16                      // Timer count 

#define     CMT1_COUNT_NUM  6                   // 1us timer setting 
#define     CMT1_COUNT_VAL  CMT1_COUNT_NUM-1    // 1us timer setting value 

#define     ECU_TIMER_ID    0                   // ECU timer (1ms) 
#define     TP_TIMER_ID     1                   // Separation timer 
#define     DTC_TIMER_ID    2                   // DTC continuation timer 

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
void cmt0_init(void);

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
void start_timer(int tch, int interval );

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
void after_call(int tch, int interval, void *proc );

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
int check_timer(int tch);

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
void stop_timer(int tch);

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
void wait(int msec, void *loop);

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
void cmt0_job(void);

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
void cmt1_init(void);

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
void cmt1_start(int count, void *proc);

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
int cmt1_check(void);
int cmt1_ni_check(void);

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
int cmt1_stop(void);

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
void swait(int usec, void *loop);

#endif //__LFY_RX63N1_CMT_TIMER__

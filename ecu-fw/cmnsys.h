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

// -*-c++-*-
// $RCSfile$
// $Revision$
// $Date$
// 
// Copyright (c) 2015 LandF Corporation.
//
// History:
//

#if !defined(_cmnsys_h_)
#define _cmnsys_h_

//#include "ether.h"
#include "rtc.h"

// 定数定義
#define SIZE_OF_WORKBUF		512
#define SIZE_OF_LINE		128
#define BUFSIZE				256

/* Declare structure to hold received ethernet frames */
typedef struct
{
    uint8_t frame[BUFSIZE];
    int32_t len;
    uint8_t wk[12];
} USER_BUFFER;
// 共用制御ブロック構造体定義
struct _common_cb {
	uint  LogLevelValue;
	uchar wb[SIZE_OF_WORKBUF];
	uchar lines[SIZE_OF_LINE];
	USER_BUFFER recv[10];
	int rtcInitDone;
	time_bcd_t tm;
} ;

typedef struct _common_cb		SYSTEMCB;

#endif	/* !defined(_cmnsys_h_) */

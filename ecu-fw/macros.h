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
// $RCSfile: macros.h,v $
// $Revision: 1.1 $
// $Date: 2016/03/23 07:32:57 $
// 
// Copyright (c) 2015 LandF Corporation.
//
// History:
//

#if !defined(_macros_h_)
#define _macros_h_

#define	_B1(a)	(1<<(a))
#define	_B0(a)	(0)
#define	_B(a,b)	((a)<<(b))

#if defined(false)
#undef false
#endif
#if defined(true)
#undef true
#endif

#define false			0
#define	true			1

#define __evenaccess

#define	BIT0		_B(1,0)				// bit0
#define	BIT1		_B(1,1)				// bit1
#define	BIT2		_B(1,2)				// bit2
#define	BIT3		_B(1,3)				// bit3
#define	BIT4		_B(1,4)				// bit4
#define	BIT5		_B(1,5)				// bit5
#define	BIT6		_B(1,6)				// bit6
#define	BIT7		_B(1,7)				// bit7

#endif	/* !defined(_macros_h_) */

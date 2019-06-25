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

#if !defined(_altypes_h_)
#define _altypes_h_

/* bsd */
typedef unsigned char		u_char;
typedef unsigned short		u_short;
typedef unsigned int		u_int;
typedef unsigned long		u_long;

/* sysv */
typedef unsigned char		unchar;
typedef unsigned short		ushort;
typedef unsigned int		uint;
typedef unsigned long		ulong;

/* linux kernel's */
typedef	char				int8_t;
typedef unsigned char		uint8_t;
typedef unsigned short		u16;
typedef unsigned short		uint16_t;
typedef unsigned int 		__u32;
typedef	__u32				u_int32_t;
typedef u_int32_t           uint32_t;
typedef signed long			int32_t;

/* Renesas */
typedef signed char			_SBYTE;
typedef unsigned char		_UBYTE;
typedef signed short		_SWORD;
typedef unsigned short		_UWORD;
typedef signed int			_SINT;
typedef unsigned int		_UINT;
typedef signed long			_SDWORD;
typedef unsigned long		_UDWORD;

/* Renesas for RX */
typedef uint32_t			natural_uint_t;
typedef int32_t				natural_int_t;

/* misc */
typedef unsigned char		uchar;
typedef unsigned int		uint32;
typedef int					bool;
typedef short				int16_t;

#endif	/* !defined(_altypes_h_) */

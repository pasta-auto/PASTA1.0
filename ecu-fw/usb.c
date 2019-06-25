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

// 
// システム定義
// 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sysio.h>
#include <ctype.h>

//
// ユーザ定義
// 

// CPUレジスタ定義
//#include "iodefine.h"
//#include "cpu_rx63n.h"

// アプリケーション定義

//
// マクロ宣言 =====
// 

//
// 外部参照宣言 =====
// 
extern void __mm_usb_init(void);
extern int __mm_sio_ind0(void);
extern int __mm_sio_out0(unsigned char data);
extern void __mm_sio_flush0(void);
//
// 内部関数宣言 =====
// 
void usb_init(void);
int usb_getch(void);
int usb_putch(unsigned char c);
int usb_puts(char *str);

//
// 型宣言 =====
// 

//
// 変数宣言 =====
//

void usb_init(void)
{
	__mm_usb_init();
}

int usb_getch(void)
{
	int c;
	c = __mm_sio_ind0();
//	if (c < 0) c = 0;
	return c;
}

int usb_putch(unsigned char c)
{
	return __mm_sio_out0(c);
}

void usb_flush(void)
{
	__mm_sio_flush0();
}

int usb_puts(char *str)
{
	while(*str) {
		usb_putch(*str);
		str++;
	}
	usb_flush();
}



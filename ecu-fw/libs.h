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

#if !defined(_libs_h_)
#define _libs_h_

extern void chomp( char s[] );

enum _loglvl {LOG_EMERG=0
			  ,LOG_ALERT
			  ,LOG_CRIT
			  ,LOG_ERR
			  ,LOG_WARNING
			  ,LOG_NOTICE
			  ,LOG_INFO
			  ,LOG_DEBUG };
extern void	logging(char *fmt, ...);
//extern void	logging(int loglvl, char *bp, char *fmt, ...);
extern int  getLogLevel(void);
extern void setLogLevel(int level);
extern int get_token_buf(const char *ptr,
						 char *cp_sept,   /* セパレータ列 ex:" ,\t" */
						 char  c_cmnt,	   /* コメント識別キャラクタ ex:'#' */
						 char *cp_token[],	/* tokenへのポインター格納領域 */
						 int   token_max,
						 int  *ip_tokcnt); 	/* 切り出した文字列のカウンタ値 */

extern int a2bcd(char *ascii, char *bcd);

#endif	/* !defined(_libs_h_) */

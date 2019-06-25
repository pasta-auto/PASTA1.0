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

//________________________________________________________________________________________
//
//  LFY-RX63N1  CMT タイマー処理
//
//----------------------------------------------------------------------------------------
//  開発履歴
//
//  2016/02/10  コーディング開始（橘）
//
//----------------------------------------------------------------------------------------
//  T.Tachibana
//  ㈱L&F
//________________________________________________________________________________________
//

#include	<string.h>
#include	"iodefine.h"
#include	"timer.h"

//  間接呼び出しのプロトタイプ(引数１個)
//typedef   void	(*TIMER_CALL)(void);
//
//#define		CMT0_COUNT_NUM	6000				// 1msタイマー設定
//#define		CMT0_COUNT_VAL	CMT0_COUNT_NUM-1	// 1msタイマー設定値
//#define		TIMER_AN		4					//　タイマー数
//  1msec単位汎用タイマー
int				cmt0_time[TIMER_AN];				//  ダウンカウンター
int				cmt0_tupf[TIMER_AN];				//  タイムアップフラグ
TIMER_CALL		cmt0_call[TIMER_AN];				//  アップ後呼び出し関数
unsigned int	freerun_timer;						//	フリーランタイマー

//#define	   CMT1_COUNT_NUM	6					// 1usタイマー設定
//#define	   CMT1_COUNT_VAL	CMT1_COUNT_NUM-1	// 1usタイマー設定値
//  1usec単位短期待ちタイマー
int				cmt1_tupf;							//	タイムアップフラグ
int				cmt1_msec;							//	msecオーダー残カウンタ
int				cmt1_last;							//	最終カウント値
int				cmt1_time;							//	積算時間
TIMER_CALL		cmt1_call;							//	アップ後呼び出し関数

//________________________________________________________________________________________
//
//  cmt0_init
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT0初期化
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void cmt0_init(void)
{
	freerun_timer = 0;
	memset(cmt0_time, 0, sizeof(cmt0_time));
	memset(cmt0_tupf, 0, sizeof(cmt0_tupf));
	memset(cmt0_call, 0, sizeof(cmt0_call));
	SYSTEM.PRCR.WORD = 0xA502;		// CLK関連レジスタプロテクト解除
	ICU.IER[0x03].BIT.IEN4 = 1;		// 0=禁止　1=割り込み許可
	ICU.IPR[004].BIT.IPR = 7;		// 割り込みレベル（15が最高）
	MSTP_CMT0 = 0;					// CMTユニット0 モジュール電源ON
	CMT0.CMCR.BIT.CKS = 0;			// 0=PCLK/8　1=PCLK/32 2=PCLK/128 3=PCLK/512
	CMT0.CMCR.BIT.CMIE = 1;			// 割り込み許可
	CMT0.CMCOR = CMT0_COUNT_VAL;	// 1msec毎割り込み
	CMT.CMSTR0.BIT.STR0 = 1;		// 0=停止、1=開始
}

//________________________________________________________________________________________
//
//  start_timer
//----------------------------------------------------------------------------------------
//  機能説明
//	  指定タイマー計時開始
//  引数
//	  tch		 チャンネル番号
//	  interval	1msec単位の待ち時間設定
//  戻り
//	  無し
//________________________________________________________________________________________
//
void start_timer(int tch, int interval )
{
	cmt0_time[tch] = interval;
	cmt0_tupf[tch] = 0;
	cmt0_call[tch] = 0;
}

//________________________________________________________________________________________
//
//  after_call
//----------------------------------------------------------------------------------------
//  機能説明
//	  指定タイマー計時開始
//  引数
//	  tch		 チャンネル番号
//	  interval	1msec単位の待ち時間設定
//	  *proc	   アップ後呼び出し先
//  戻り
//	  無し
//________________________________________________________________________________________
//
void after_call(int tch, int interval, void *proc )
{
	if(interval < 0)
	{	//	高速コール
		cmt0_time[tch] = -interval;
		cmt0_tupf[tch] = -1;
		cmt0_call[tch] = (TIMER_CALL)proc;
	}
	else
	{	//	低速コール
		cmt0_time[tch] = interval;
		cmt0_tupf[tch] = 0;
		cmt0_call[tch] = (TIMER_CALL)proc;
	}
}

//________________________________________________________________________________________
//
//  check_timer
//----------------------------------------------------------------------------------------
//  機能説明
//	  指定タイマーアップ確認
//  引数
//	  tch		 チャンネル番号
//  戻り
//	  int		 残時間(msec)
//________________________________________________________________________________________
//
int check_timer(int tch)
{
	return (cmt0_time[tch] > 0) ? 0: 1;
}

//________________________________________________________________________________________
//
//  stop_timer
//----------------------------------------------------------------------------------------
//  機能説明
//	  指定タイマー停止
//  引数
//	  tch		 チャンネル番号
//  戻り
//	  無し
//________________________________________________________________________________________
//
void stop_timer(int tch)
{
	cmt0_time[tch] = 0;
}

//________________________________________________________________________________________
//
//  wait
//----------------------------------------------------------------------------------------
//  機能説明
//	  指定msec間待ち
//  引数
//	  msec		待ち時間
//	  *loop	   待ち時間中の関数呼び出し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void wait(int msec, void *loop)
{
	TIMER_CALL  p = (TIMER_CALL)loop;
	cmt0_time[(TIMER_AN-1)] = msec;
	while(cmt0_time[(TIMER_AN-1)] > 0)
	{
		if(p != 0) p();	//	待ち中実行
	}
}

//________________________________________________________________________________________
//
//  cmt0_job
//----------------------------------------------------------------------------------------
//  機能説明
//	  タイムアップタイマーの関数呼び出し
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void cmt0_job(void)
{
	int			c;
	TIMER_CALL	p;

	for( c = 0; c < TIMER_AN; c++)
	{
		if(cmt0_tupf[c] == 1)
		{  	//	タイムアップ
			cmt0_tupf[c] = 0;
			p = cmt0_call[c]; 
			if(p != 0)
			{  	//	関数呼び出し
				p();
			}
		}
	}
}

//________________________________________________________________________________________
//
//  cmt0_int
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT0割り込み(1msec)
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void interrupt __vectno__{VECT_CMT0_CMI0} cmt0_int(void)
{
	int			c;
	TIMER_CALL	p;

	freerun_timer++;	//  1msecフリーランカウンタ

	for( c = 0; c < TIMER_AN; c++)
	{
		if(cmt0_time[c] > 0)
		{  	//	残カウントあり
			cmt0_time[c]--;
			if(cmt0_time[c] == 0)
			{  	//	タイムアップ
				p = cmt0_call[c]; 
				if(p != 0 && cmt0_tupf[c] < 0)
				{  	//	関数呼び出し
					p();
				}
				else
				{
					cmt0_tupf[c] = 1;
				}
			}
		}
	}
}

//________________________________________________________________________________________
//
//  cmt1_init
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT1初期化
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void cmt1_init(void)
{
	cmt1_tupf = 0;
	cmt1_last = 0;
	cmt1_msec = 0;
	cmt1_call = 0;
	cmt1_time = 0;
	SYSTEM.PRCR.WORD = 0xA502;						// CLK関連レジスタプロテクト解除
	ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1 = 0;	// 0=禁止　1=割り込み許可
	ICU.IPR[IPR_CMT1_CMI1].BIT.IPR = 0;				// 割り込みレベル   0=禁止 / 1～15=許可
	CMT.CMSTR0.BIT.STR1 = 0;						// 0=停止、1=開始
	MSTP_CMT1 = 0;									// CMTユニット1 モジュール電源ON
	CMT1.CMCR.BIT.CKS = 0;							// PCLK=48MHz : 0=PCLK/8　1=PCLK/32 2=PCLK/128 3=PCLK/512
	CMT1.CMCR.BIT.CMIE = 0;							// 割り込み 1:許可 / 0:禁止
	CMT1.CMCOR = 0xFFFF;							// 6MHz / 65536
	CMT1.CMCNT = 0;									// カウンタ
}

//________________________________________________________________________________________
//
//  cmt1_start
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT1計時開始(1usec)
//  引数
//	  count	   待ち時間 0～10000 : 10.000ms : 0.01sec
//	  *proc	   アップ後呼び出し先
//  戻り
//	  無し
//________________________________________________________________________________________
//
void cmt1_start(int count, void *proc)
{
	cmt1_tupf = 0;
	cmt1_time = 0;
	cmt1_msec = count / 10000;
	cmt1_last = count - cmt1_msec * 10000;
	if(cmt1_msec > 0)
	{
		count = 10000;
	}
	cmt1_call = (TIMER_CALL)proc;
	CMT.CMSTR0.BIT.STR1 = 0;						//  0=停止、1=開始
	ICU.IR[IR_CMT1_CMI1].BIT.IR = 0;				//  割り込みフラグクリア
	CMT1.CMCR.BIT.CKS = 0;							//	PCLK=48MHz : 0=PCLK/8　1=PCLK/32 2=PCLK/128 3=PCLK/512
	CMT1.CMCR.BIT.CMIE = 1;							//	割り込み 1:許可 / 0:禁止
	CMT1.CMCOR = count * CMT1_1US - 1; 				//	6MHz/count
	CMT1.CMCNT = 0;									//	カウンタ
	ICU.IPR[IPR_CMT1_CMI1].BIT.IPR = 1;				// 割り込みレベル   0=禁止 / 1～15=許可
	ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1 = 1;	// 0=禁止　1=割り込み許可
	CMT.CMSTR0.BIT.STR1 = 1;						//  0=停止、1=開始
}

//________________________________________________________________________________________
//
//  cmt1_check
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT1タイムアップ確認
//  引数
//	  無し
//  戻り
//	  int	 0=停止中 / 1=計時中
//________________________________________________________________________________________
//
int cmt1_ni_check(void)
{
	int count;
	if(CMT.CMSTR0.BIT.STR1 != 0)
	{
		if(ICU.IR[IR_CMT1_CMI1].BIT.IR != 0)
		{  	//	カウントアップ
			CMT.CMSTR0.BIT.STR1 = 0;			// 0=停止、1=開始
			ICU.IR[IR_CMT1_CMI1].BIT.IR = 0;	//  割り込みフラグクリア
			if(cmt1_msec > 0)
			{
				cmt1_msec--;
				if(cmt1_msec > 0)
				{
					count = 10000;
				}
				else
				{
					count = cmt1_last;
				}
				CMT1.CMCR.BIT.CKS = 0;			 	//	PCLK=48MHz : 0=PCLK/8　1=PCLK/32 2=PCLK/128 3=PCLK/512
				CMT1.CMCR.BIT.CMIE = 1;				//	割り込み 1:許可 / 0:禁止
				CMT1.CMCOR = count * CMT1_1US - 1; 	//	6MHz/count
				CMT1.CMCNT = 0;						//	カウンタ
				CMT.CMSTR0.BIT.STR1 = 1;			//  0=停止、1=開始
			}
			else
			{
				cmt1_tupf++;
				if(cmt1_call != 0) cmt1_call();
				ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1 = 0;	// 0=禁止　1=割り込み許可
				ICU.IPR[IPR_CMT1_CMI1].BIT.IPR = 0;				// 割り込みレベル   0=禁止 / 1～15=許可
			}
		}
	}
	return (CMT.CMSTR0.BIT.STR1);	  	//	0=停止 / 1=計時
}
int cmt1_check(void)
{
	return (CMT.CMSTR0.BIT.STR1);	  	//	0=停止 / 1=計時
}

//________________________________________________________________________________________
//
//  cmt1_stop
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT1停止と経過時間取得
//  引数
//	  無し
//  戻り
//	  経過時間(1us単位)
//________________________________________________________________________________________
//
int	cmt1_stop(void)
{
	if(CMT.CMSTR0.BIT.STR1)
	{
		CMT.CMSTR0.BIT.STR1 = 0;						//	0=停止、1=開始
		ICU.IR[IR_CMT1_CMI1].BIT.IR = 0;				//  割り込みフラグクリア
		ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1 = 0;	//	0=禁止　1=割り込み許可
		ICU.IPR[IPR_CMT1_CMI1].BIT.IPR = 0;				//	割り込みレベル   0=禁止 / 1～15=許可
		cmt1_time += CMT1.CMCNT / CMT1_1US;
	}
	return cmt1_time;
}

//________________________________________________________________________________________
//
//  cmt1_int
//----------------------------------------------------------------------------------------
//  機能説明
//	  CMT1割り込み(1usec)
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void interrupt __vectno__{VECT_CMT1_CMI1} cmt1_int(void)
{
	int count;
	CMT.CMSTR0.BIT.STR1 = 0;			// 0=停止、1=開始
	ICU.IR[IR_CMT1_CMI1].BIT.IR = 0;	//  割り込みフラグクリア
	cmt1_time += ((int)CMT1.CMCOR + 1) / CMT1_1US;
	if(cmt1_msec > 0)
	{
		cmt1_msec--;
		if(cmt1_msec > 0)
		{
			count = 10000;
		}
		else
		{
			count = cmt1_last;
		}
		CMT1.CMCR.BIT.CKS = 0;			 	//	PCLK=48MHz : 0=PCLK/8　1=PCLK/32 2=PCLK/128 3=PCLK/512
		CMT1.CMCR.BIT.CMIE = 1;				//	割り込み 1:許可 / 0:禁止
		CMT1.CMCOR = count * CMT1_1US - 1; 	//	6MHz/count
		CMT1.CMCNT = 0;						//	カウンタ
		CMT.CMSTR0.BIT.STR1 = 1;			//  0=停止、1=開始
	}
	else
	{
		ICU.IER[IER_CMT1_CMI1].BIT.IEN_CMT1_CMI1 = 0;	// 0=禁止　1=割り込み許可
		ICU.IPR[IPR_CMT1_CMI1].BIT.IPR = 0;				// 割り込みレベル   0=禁止 / 1～15=許可
		cmt1_tupf++;
		if(cmt1_call != 0) cmt1_call();
	}
}

//________________________________________________________________________________________
//
//  swait
//----------------------------------------------------------------------------------------
//  機能説明
//	  ショートウェイト	usec間待ち
//  引数
//	  usec		待ち時間
//	  *loop	   待ち時間中の関数呼び出し
//  戻り
//	  無し
//________________________________________________________________________________________
//
void swait(int usec, void *loop)
{
	int ms = usec / 10000;
	TIMER_CALL  p = (TIMER_CALL)loop;
	for(; ms > 0 && usec > 65535; ms--)
	{
		usec -= 10000;
		cmt1_start(10000, 0);
		while(cmt1_tupf == 0)
		{
			if(p != 0) p();	//	待ち中実行
		}
	}
	cmt1_start(usec, 0);
	while(cmt1_tupf == 0)
	{
		if(p != 0) p();	//	待ち中実行
	}
}



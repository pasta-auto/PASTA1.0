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
//	RX63N	uSD用	RSPI1-I/F 通信
//
//----------------------------------------------------------------------------------------
//	開発履歴
//
//	2016/02/10	コーディング開始（橘）
//
//----------------------------------------------------------------------------------------
//	T.Tachibana
//	㈱L&F
//________________________________________________________________________________________
//

//#include "uSD_rspi1.h"

#ifndef __CAN2ECU_uSD_IF__
#define __CAN2ECU_uSD_IF__

/*
	ポート設定

			Port		SCI			I2C			SPI			適用
	----------------------------------------------------------------------------
	RSPI1	PE7									MISOB		<RSPI>		uSD
			PE6									MOSIB		<RSPI>		uSD
			PE5									RSPCKB		<RSPI>		uSD
			PE4									SSLB0		<RSPI>		uSD
*/

#define		RSPI1_ACTIVATE

#ifdef		RSPI1_ACTIVATE

//	uSD用 RSPI管理構造体
typedef struct	__spi_module__ {
	int				err;				//	エラーフラグ
	void			*rx_proc;			//	受信完了割り込み時呼び出し関数
	void			*tx_proc;			//	送信完了割り込み時呼び出し関数
	void			*ti_proc;			//	アイドリング割り込み時呼び出し関数
	void			*err_proc;			//	エラー発生割り込み時呼び出し関数
}	SPI_MODULE;

extern	SPI_MODULE		usd_spi_com;

//間接呼び出しのプロトタイプ(引数１個)
typedef	void 			(*USD_PROC_CALL)(void *);

//	ログ機能
void	logging(char *fmt, ...);

//________________________________________________________________________________________
//
//	rspi1_init
//----------------------------------------------------------------------------------------
//	機能説明
//		RSPI1初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			RSPI1	PE7									MISOB		<RSPI>		uSD
//					PE6									MOSIB		<RSPI>		uSD
//					PE5									RSPCKB		<RSPI>		uSD
//					PE4									SSLB0		<RSPI>		uSD
//	引数
//		speed		通信速度	100,000～10,000,000
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void rspi1_init(long bps);

#endif		/*RSPI1_ACTIVATE*/
#endif		/*__CAN2ECU_uSD_IF__*/

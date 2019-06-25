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

#include <sysio.h>
#include <string.h>
#include <stdio.h>
#include "iodefine.h"
#include "ecu.h"			/*	ECU 共通定義			*/
#include "uSD_rspi1.h"

/*
	ポート設定

			Port		SCI			I2C			SPI			適用
	----------------------------------------------------------------------------
	RSPI1	PE7									MISOB		<RSPI>		uSD
			PE6									MOSIB		<RSPI>		uSD
			PE5									RSPCKB		<RSPI>		uSD
			PE4									SSLB0		<RSPI>		uSD
*/

#ifdef		RSPI1_ACTIVATE

/*
//	uSD用 RSPI管理構造体
typedef struct	__spi_module__ {
	int				err;				//	エラーフラグ
	void			*rx_proc;			//	受信完了割り込み時呼び出し関数
	void			*tx_proc;			//	送信完了割り込み時呼び出し関数
	void			*ti_proc;			//	アイドリング割り込み時呼び出し関数
	void			*err_proc;			//	エラー発生割り込み時呼び出し関数
}	SPI_MODULE;

*/
SPI_MODULE		usd_spi_com;

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
void rspi1_init(long bps)
{
	memset(&usd_spi_com, 0, sizeof(SPI_MODULE));

	SYSTEM.PRCR.WORD = 0xA502;	//	プロテクト解除
	MSTP_RSPI1 = 0;				//	RSPI1モジュールストップ解除

	//	RSPI1 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 エラー割り込み禁止
	ICU.IER[IER_RSPI1_SPRI1].BIT.IEN_RSPI1_SPRI1 = 0;	//	受信バッファフル割り込み禁止
	ICU.IER[IER_RSPI1_SPTI1].BIT.IEN_RSPI1_SPTI1 = 0;	//	送信エンプティ割り込み禁止
	ICU.IER[IER_RSPI1_SPII1].BIT.IEN_RSPI1_SPII1 = 0;	//	送信アイドル割り込み禁止

	RSPI1.SPCR.BYTE = 0;		//	モジュール初期化

	// CAN3の端子設定(RSPI2経由)
	PORTE.PODR.BYTE = 0x70;		//	ポート初期化
	PORTE.PDR.BIT.B7 = 0;		//	PB7 <- MISOB
	PORTE.PDR.BIT.B6 = 1;		//	PB6 -> MOSIB
	PORTE.PDR.BIT.B5 = 1;		//	PB5 -> RSPCKB
	PORTE.PDR.BIT.B4 = 1;		//	PB4 -> SSLB0

	PORTE.PMR.BIT.B7 = 1;		//	周辺機能	MISOB
	PORTE.PMR.BIT.B6 = 1;		//	周辺機能	MOSIB
	PORTE.PMR.BIT.B5 = 1;		//	周辺機能	RSPCKB
	PORTE.PMR.BIT.B4 = 1;		//	周辺機能	SSLB0

	MPC.PWPR.BIT.B0WI = 0;		//	
	MPC.PWPR.BIT.PFSWE = 1;		//	

	MPC.PE7PFS.BYTE = 0x0D;		//	MISOB		SO
	MPC.PE6PFS.BYTE = 0x0D;		//	MOSIB		SI
	MPC.PE5PFS.BYTE = 0x0D;		//	RSPCKB		SCK
	MPC.PE4PFS.BYTE = 0x0D;		//	SSLB0		/CS

	MPC.PWPR.BIT.PFSWE = 0;		//	
	MPC.PWPR.BIT.B0WI = 1;		//	

	SYSTEM.PRCR.WORD = 0xA500;	//	ポート設定禁止
	
	//	RSPI1の設定(シングルマスタモード)
	RSPI1.SSLP.BYTE = 0;		//	SSLnP はアクティブLow
	RSPI1.SPPCR.BYTE = 0x20;	//	MOSIのアイドル出力はLow
	RSPI1.SPSR.BYTE &= 0;		//	エラーフラグ解除
	RSPI1.SPSCR.BYTE = 0;		//	シーケンス初期値

	//	Set baud rate to 1Mbps	N値(BRDV[1:0])=0 固定	最小=93,750bps
	//	n = (PCLK Frequency) / (2 * 2^N * Bit Rate) - 1
	//	n = (48,000,000) / (2 * 2^0 * 1,000,000) - 1
	//	n = 24
	RSPI1.SPBR.BYTE = 48000000 / (2 * bps) - 1;
	RSPI1.SPDCR.BYTE = 0x20;	//	SPDRはロングワードアクセス / 受信バッファ読み出し / 1フレーム
	RSPI1.SPCKD.BYTE = 0;		//	クロック遅延 1RSPCK
	RSPI1.SSLND.BYTE = 0;		//	SSLネゲート遅延 1RSPCK
	RSPI1.SPND.BYTE = 0;		//	次アクセス遅延 1RSPCK + 2PCLK
	RSPI1.SPCR2.BYTE = 0;		//	パリティ無効 / アイドル割り込み禁止
	
	//	コマンドレジスタ初期化
	//	RSPCK位相設定ビット
	RSPI1.SPCMD0.BIT.CPHA = 0;		//	0 : 奇数エッジでデータサンプル、偶数エッジでデータ変化
									//	1 : 奇数エッジでデータ変化、偶数エッジでデータサンプル
	//	RSPCK極性設定ビット
	RSPI1.SPCMD0.BIT.CPOL = 0;		//	0：アイドル時のRSPCKがLow
									//	1：アイドル時のRSPCKがHigh
	//	ビットレート分周設定ビット
	RSPI1.SPCMD0.BIT.BRDV = 0;		//	b3 b2
									//	0 0：ベースのビットレートを選択
									//	0 1：ベースのビットレートの2分周を選択
									//	1 0：ベースのビットレートの4分周を選択
									//	1 1：ベースのビットレートの8分周を選択
	//	SSL信号アサート設定ビット
	RSPI1.SPCMD0.BIT.SSLA = 0;		//	b6 b4
									//	0 0 0：SSL0
									//	0 0 1：SSL1
									//	0 1 0：SSL2
									//	0 1 1：SSL3
									//	1 x x：設定しないでください
									//	x：Don’t care
	//	SSL信号レベル保持ビット
	RSPI1.SPCMD0.BIT.SSLKP = 0;		//	0：転送終了時に全SSL信号をネゲート
									//	1：転送終了後から次アクセス開始までSSL信号レベルを保持
	//	RSPIデータ長設定ビット
	RSPI1.SPCMD0.BIT.SPB = 4;		//	b11 b8
									//	0100～0111 ：8ビット
									//	1 0 0 0：9ビット
									//	1 0 0 1：10ビット
									//	1 0 1 0：11ビット
									//	1 0 1 1：12ビット
									//	1 1 0 0：13ビット
									//	1 1 0 1：14ビット
									//	1 1 1 0：15ビット
									//	1 1 1 1：16ビット
									//	0 0 0 0：20ビット
									//	0 0 0 1：24ビット
									//	0010、0011 ：32ビット
	//	RSPI LSBファーストビット
	RSPI1.SPCMD0.BIT.LSBF = 0;		//	0：MSBファースト
									//	1：LSBファースト
	//	RSPI次アクセス遅延許可ビット
	RSPI1.SPCMD0.BIT.SPNDEN = 0;	//	0：次アクセス遅延は1RSPCK＋2PCLK
									//	1：次アクセス遅延はRSPI次アクセス遅延レジスタ（SPND）の設定値
	//	SSLネゲート遅延設定許可ビット
	RSPI1.SPCMD0.BIT.SLNDEN = 0;	//	0：SSLネゲート遅延は1RSPCK
									//	1：SSLネゲート遅延はRSPIスレーブセレクトネゲート遅延レジスタ（SSLND）の設定値
	//	RSPCK遅延設定許可ビット
	RSPI1.SPCMD0.BIT.SCKDEN = 0;	//	0：RSPCK遅延は1RSPCK
									//	1：RSPCK遅延はRSPIクロック遅延レジスタ（SPCKD）の設定値

	//	設定コピー
	RSPI1.SPCMD1.WORD = RSPI1.SPCMD0.WORD;
	RSPI1.SPCMD2.WORD = RSPI1.SPCMD0.WORD;
	RSPI1.SPCMD3.WORD = RSPI1.SPCMD0.WORD;
	RSPI1.SPCMD4.WORD = RSPI1.SPCMD0.WORD;
	RSPI1.SPCMD5.WORD = RSPI1.SPCMD0.WORD;
	RSPI1.SPCMD6.WORD = RSPI1.SPCMD0.WORD;
	RSPI1.SPCMD7.WORD = RSPI1.SPCMD0.WORD;
	
	//	動作許可
	//	RSPIモード選択ビット
	RSPI1.SPCR.BIT.SPMS = 0;		//	0：SPI動作（4線式）
									//	1：クロック同期式動作（3線式）
	//	通信動作モード選択ビット
	RSPI1.SPCR.BIT.TXMD = 0;		//	0：全二重同期式シリアル通信
									//	1：送信動作のみのシリアル通信
	//	モードフォルトエラー検出許可ビット
	RSPI1.SPCR.BIT.MODFEN = 0;		//	0：モードフォルトエラー検出を禁止
									//	1：モードフォルトエラー検出を許可
	//	RSPIマスタ/スレーブモード選択ビット
	RSPI1.SPCR.BIT.MSTR = 1;		//	0：スレーブモード
									//	1：マスタモード
	//	RSPIエラー割り込み許可ビット
	RSPI1.SPCR.BIT.SPEIE = 0;		//	0：RSPIエラー割り込み要求の発生を禁止
									//	1：RSPIエラー割り込み要求の発生を許可
	//	RSPI送信割り込み許可ビット
	RSPI1.SPCR.BIT.SPTIE = 0;		//	0：RSPI送信割り込み要求の発生を禁止
									//	1：RSPI送信割り込み要求の発生を許可
	//	RSPI受信割り込み許可ビット
	RSPI1.SPCR.BIT.SPRIE = 0;		//	0：RSPI受信割り込み要求の発生を禁止
									//	1：RSPI受信割り込み要求の発生を許可
	//	RSPIアイドル割り込み許可ビット
	RSPI1.SPCR2.BIT.SPIIE = 0;		//	0：アイドル割り込み要求の発生を禁止
									//	1：アイドル割り込み要求の発生を許可
	//	RSPI機能許可ビット
	RSPI1.SPCR.BIT.SPE = 1;			//	0：RSPI機能は無効
									//	1：RSPI機能が有効

	//	割り込み優先順位設定
	ICU.IPR[IPR_RSPI1_].BIT.IPR = 2;					//	割り込みレベル設定
	//	割り込みフラグクリア
	ICU.IR[IR_RSPI1_SPRI1].BIT.IR = 0;
	ICU.IR[IR_RSPI1_SPTI1].BIT.IR = 0;
	ICU.IR[IR_RSPI1_SPII1].BIT.IR = 0;
	//	GROUP12割り込み設定
	ICU.GEN[GEN_RSPI1_SPEI1].BIT.EN_RSPI1_SPEI1 = 1;	//	グループ12 RSPI1受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア
	//	割り込み許可設定
	ICU.IER[IER_RSPI1_SPRI1].BIT.IEN_RSPI1_SPRI1 = 1;	//	受信バッファフル割り込み許可
	ICU.IER[IER_RSPI1_SPTI1].BIT.IEN_RSPI1_SPTI1 = 1;	//	送信エンプティ割り込み許可
	ICU.IER[IER_RSPI1_SPII1].BIT.IEN_RSPI1_SPII1 = 1;	//	送信アイドル割り込み許可
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
}

//---------------------------------------------------------------------------------------
//	SPRI1 受信バッファフル	SPI受信バッファにデータ到着
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_RSPI1_SPRI1} RSPI1_SPRI1_ISR(void)
{
	USD_PROC_CALL	proc;
	//	RSPI受信割り込み許可ビット
	RSPI2.SPCR.BIT.SPRIE = 0;		//	0：RSPI受信割り込み要求の発生を禁止
									//	1：RSPI受信割り込み要求の発生を許可
	if(usd_spi_com.rx_proc)
	{	//	ユーザー処理関数呼び出し
		proc = (USD_PROC_CALL)usd_spi_com.rx_proc;
		proc(0);
	}
}

//---------------------------------------------------------------------------------------
//	SPTI1 送信バッファエンプティ	SPI送信バッファが空いた
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_RSPI1_SPTI1} RSPI1_SPTI1_ISR(void)
{
	USD_PROC_CALL	proc;
	//	RSPI送信割り込み許可ビット
	RSPI2.SPCR.BIT.SPTIE = 0;		//	0：RSPI送信割り込み要求の発生を禁止
									//	1：RSPI送信割り込み要求の発生を許可
	if(usd_spi_com.tx_proc)
	{	//	ユーザー処理関数呼び出し
		proc = (USD_PROC_CALL)usd_spi_com.tx_proc;
		proc(0);
	}
}

//---------------------------------------------------------------------------------------
//	SPII1 アイドル	送信データの最終書き込み完了時に割り込み発生（送信完了）
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_RSPI1_SPII1} RSPI1_SPII1_ISR(void)
{
	USD_PROC_CALL	proc;
	//	RSPIアイドル割り込み許可ビット
	RSPI1.SPCR2.BIT.SPIIE = 0;		//	0：アイドル割り込み要求の発生を禁止
									//	1：アイドル割り込み要求の発生を許可
	if(usd_spi_com.ti_proc)
	{	//	ユーザー処理関数呼び出し
		proc = (USD_PROC_CALL)usd_spi_com.ti_proc;
		proc(0);
	}
	else
	{	//	割り込み条件クリア
		if(usd_spi_com.tx_proc)
		{	//	ユーザー処理関数呼び出し
			proc = (USD_PROC_CALL)usd_spi_com.tx_proc;
			proc(0);
		}
	}
}

#endif		/*RSPI1_ACTIVATE*/

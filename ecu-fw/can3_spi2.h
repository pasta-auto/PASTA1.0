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
//	LFY-RX63N1	SCI-I/F 通信
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

//#include	"can3_spi2.h"

#ifndef __CAN2ECU_CAN3RSPI2_IF__
#define __CAN2ECU_CAN3RSPI2_IF__

#include	"ecu.h"			/*	ECU 共通定義			*/

/*
	ポート設定

			Port		SCI			I2C			SPI			適用
	----------------------------------------------------------------------------
	RSPI2	PD2									MISOC		<RSPI>		CAN3
			PD1									MOSIC		<RSPI>		CAN3
			PD3									RSPCKC		<RSPI>		CAN3
			PD4									SSLC0		<RSPI>		CAN3
			PD0									IRQ0		<- CINT		CAN3
			PD6												<- CRX0BF	CAN3
			PD7												<- CRX1BF	CAN3
			P90												-> CTX0RTS	CAN3
			P91												-> CTX1RTS	CAN3
			P92												-> CTX2RTS	CAN3
			P93												-> CRSET	CAN3
			P07												<- CSOF		CAN3


*/

#define		RSPI2_ACTIVATE

//間接呼び出しのプロトタイプ(引数１個)
typedef	void 			(*CAN3_PROC_CALL)(void *);

#ifdef	RSPI2_ACTIVATE

//________________________________________________________________________________________
//
//	DTC定義		(RSPIを8MBPSで運用するためにDTC化が必須)
//________________________________________________________________________________________
//

#define		DTC_VECT_TOP	0x0003E000
#define		DTC_REQUEST_TOP	0x0003D000

extern	unsigned long	*dtc_table;

typedef	union	__dtc_fulladdressmode__	{
	unsigned long	LONG[4];
	unsigned short	WORD[8];
	unsigned char	BYTE[16];
	struct	{
		union	{
			unsigned long	LONG;
			struct	{	//	モードレジスタ
				//	MRA
				unsigned long	MD		:	2;		//	DTC転送モード選択ビット
													//	b7 b6
													//	0 0：ノーマル転送モード
													//	0 1：リピート転送モード
													//	1 0：ブロック転送モード
													//	1 1：設定しないでください
				unsigned long	SZ		:	2;		//	DTCデータトランスファサイズビット
													//	DTCデータトランスファサイズビット
													//	b5 b4
													//	0 0：8ビット（バイト）転送
													//	0 1：16ビット（ワード）転送
													//	1 0：32ビット（ロングワード）転送
													//	1 1：設定しないでください
				unsigned long	SM		:	2;		//	転送元アドレスアドレッシングモードビット
													//	b3 b2
													//	0 0：SARレジスタはアドレス固定（SARレジスタのライトバックはスキップされます）
													//	0 1：SARレジスタはアドレス固定（SARレジスタのライトバックはスキップされます）
													//	1 0：転送後SARレジスタをインクリメント（SZ[1:0]ビットが“00b”のとき+1 、“01b”のとき+2、“10b”のとき+4）
													//	1 1：転送後SARレジスタをデクリメント（SZ[1:0]ビットが“00b”のとき-1 、“01b”のとき-2、“10b”のとき-4）
				unsigned long			:	2;		//	[0]
				//	MRB
				unsigned long	CHNE	:	1;		//	DTCチェーン転送許可ビット
													//	0：チェーン転送禁止
													//	1：チェーン転送許可
				unsigned long	CHNS	:	1;		//	DTCチェーン転送選択ビット
													//	0：連続してチェーン転送を行う
													//	1：転送カウンタが1→0、または1→CRAHとなったとき、チェーン転送を行う
				unsigned long	DISEL	:	1;		//	DTC割り込み選択ビット
													//	0：指定されたデータ転送終了時、CPUへの割り込みが発生
													//	1：DTCデータ転送のたびに、CPUへの割り込みが発生
				unsigned long	DTS		:	1;		//	DTC転送モード選択ビット
													//	0：転送先がリピート領域またはブロック領域
													//	1：転送元がリピート領域またはブロック領域
				unsigned long	DM		:	2;		//	転送先アドレスアドレッシングモードビット
													//	b3 b2
													//	0 0：DARレジスタはアドレス固定（DARレジスタのライトバックはスキップされます）
													//	0 1：DARレジスタはアドレス固定（DARレジスタのライトバックはスキップされます）
													//	1 0：転送後、DARレジスタをインクリメント（MAR.SZ[1:0]ビットが“00b”のとき+1、“01b”のとき+2、“10b”のとき+4）
													//	1 1：転送後DARレジスタをデクリメント（MAR.SZ[1:0]ビットが“00b”のとき-1、“01b”のとき-2、“10b”のとき-4）
				unsigned long			:	2;		//	[0]
				//	予約(0x0000ライト)
				unsigned long	RES		:	16;		//	[0]
			}	BIT;
		}	MR;
		unsigned long	SAR;	//	転送元アドレス
		unsigned long	DAR;	//	転送先アドレス
		union	{
			unsigned long	LONG;
			struct	{	//	ノーマル転送モード
				unsigned long	A		:	16;		//	DTC 転送カウントレジスタA
				unsigned long	B		:	16;		//	DTC 転送カウントレジスタB（ブロック転送モード時のブロック転送回数を指定するレジスタ）
			}	NOR;
			struct	{	//	リピート転送モード
				unsigned long	AH		:	8;		//	DTC 転送回数保持
				unsigned long	AL		:	8;		//	DTC 転送回数カウンタ
				unsigned long	B		:	16;		//	DTC 転送カウントレジスタB（ブロック転送モード時のブロック転送回数を指定するレジスタ）
			}	REP;
			struct	{	//	ブロック転送モード
				unsigned long	AH		:	8;		//	DTC ブロックサイズ保持
				unsigned long	AL		:	8;		//	DTC ブロックサイズ回数カウンタ
				unsigned long	B		:	16;		//	DTC 転送カウントレジスタB（ブロック転送モード時のブロック転送回数を指定するレジスタ）
			}	BLK;
		}	CR;
	}	REG;
}	DTC_FAMD_STR;

#define		CAN3_REQUEST_DTC_MAX	64

//	送受信リクエスト構造体定義
typedef	struct	__rspi_dtc_request__	{	//	64byte*128=4096byte = 0x1000 (3D000～3DFFF)
	unsigned short	DAT[8];		//	16	送信受信バッファ
	DTC_FAMD_STR	DTCTX;		//	16	DTC送信要求構造体
	DTC_FAMD_STR	DTCRX;		//	16	DTC受信要求構造体
	int				TXL;		//	4	送信バイト数
	int				RXL;		//	4	受信バイト数
	unsigned short	*RXP;		//	4	有効受信データポインタ
	void			*CALL;		//	4	送受信完了時呼び出し先
}	RSPI_DTC_REQ;

//	送受信リクエストチェーン構造体定義
typedef	struct	__rspi_dtc_request_list__	{
	RSPI_DTC_REQ	REQ[CAN3_REQUEST_DTC_MAX];	//	リクエストチェーン
}	RSPI_REQUESTS;

//________________________________________________________________________________________
//
//	rspi2_init
//----------------------------------------------------------------------------------------
//	機能説明
//		RSPI2初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			RSPI2	PD2									MISOC		<RSPI>		CAN3
//					PD1									MOSIC		<RSPI>		CAN3
//					PD3									RSPCKC		<RSPI>		CAN3
//					PD4									SSLC0		<RSPI>		CAN3
//					PD0									IRQ0		<- CINT		CAN3
//					PD6									IRQ6		<- CRX0BF	CAN3
//					PD7									IRQ7		<- CRX1BF	CAN3
//					P90												-> CTX0RTS	CAN3
//					P91												-> CTX1RTS	CAN3
//					P92												-> CTX2RTS	CAN3
//					P93												-> CRSET	CAN3
//					P07												<- CSOF		CAN3
//	引数
//		speed		通信速度	100,000～10,000,000(最速10Mbps)
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void			rspi2_init(long bps);		//	RSPI2初期化

//________________________________________________________________________________________
//
//	MCP2515制御定義
//________________________________________________________________________________________
//
//	ポート定義
#define		CAN3_RESET_PORT		PORT9.PODR.BIT.B3	/*	チップリセット信号			*/
#define		CAN3_SPI_CS_PORT	PORTD.PODR.BIT.B4	/*	チップセレクト信号			*/
#define		CAN3_MCPINT_PORT	PORTD.PIDR.BIT.B0	/*	割り込み 1→0=レベル		*/
#define		CAN3_RX0BF_PORT		PORTD.PIDR.BIT.B6	/*	受信完了 1→0=レベル		*/
#define		CAN3_RX1BF_PORT		PORTD.PIDR.BIT.B7	/*	受信完了 1→0=レベル		*/
#define		CAN3_TX0RTS_PORT	PORT9.PODR.BIT.B0	/*	送信要求 1→0=エッジ		*/
#define		CAN3_TX1RTS_PORT	PORT9.PODR.BIT.B1	/*	送信要求 1→0=エッジ		*/
#define		CAN3_TX2RTS_PORT	PORT9.PODR.BIT.B2	/*	送信要求 1→0=エッジ		*/

//	MCP2515コード定義
//#define		MCP2515CMD_RESET	0xC0		/*	MCP2515 内部リセット					*/
#define		MCP2515CMD_READ		0x03		/*	選択アドレスから順にレジスタを読み出す	*/
//#define		MCP2515CMD_RXB0_RH	0x90		/*	受信バッファ0SIDH読み出し	0x61		*/
//#define		MCP2515CMD_RXB0_RD	0x92		/*	受信バッファ0D0読み出し		0x66		*/
//#define		MCP2515CMD_RXB1_RH	0x94		/*	受信バッファ1SIDH読み出し	0x71		*/
//#define		MCP2515CMD_RXB1_RD	0x96		/*	受信バッファ1D0読み出し		0x76		*/
#define		MCP2515CMD_WRITE	0x02		/*	選択アドレスから順にレジスタへ書き込む	*/
//#define		MCP2515CMD_TXB0_WH	0x40		/*	送信バッファ0SIDH書き込み	0x31		*/
//#define		MCP2515CMD_TXB0_WD	0x41		/*	送信バッファ0D0書き込み		0x36		*/
//#define		MCP2515CMD_TXB1_WH	0x42		/*	送信バッファ1SIDH書き込み	0x41		*/
//#define		MCP2515CMD_TXB1_WD	0x43		/*	送信バッファ1D0書き込み		0x46		*/
//#define		MCP2515CMD_TXB2_WH	0x44		/*	送信バッファ2SIDH書き込み	0x51		*/
//#define		MCP2515CMD_TXB2_WD	0x45		/*	送信バッファ2D0書き込み		0x56		*/
//#define		MCP2515CMD_RTS		0x80		/*	送信バッファ0の送信要求		0x81～0x87	*/
//#define		MCP2515CMD_RTS0		0x81		/*	送信バッファ0の送信要求					*/
//#define		MCP2515CMD_RTS1		0x82		/*	送信バッファ1の送信要求					*/
//#define		MCP2515CMD_RTS2		0x84		/*	送信バッファ2の送信要求					*/
#define		MCP2515CMD_STATUS	0xA0		/*	状態ビット読み出し						*/
//#define		MCP2515CMD_RXSTS	0xB0		/*	受信メッセージフィルタ一致・タイプ取得	*/
#define		MCP2515CMD_BITX		0x05		/*	特定レジスタのビット変更				*/

//----------------------------------------------------------------------------------------
//	MCP2515内部アドレス定義
enum	__mcp2515_ram_address__	{
	//	フィルタ
	MCP2515AD_RXF0SIDH=0x00,MCP2515AD_RXF0SIDL,MCP2515AD_RXF0EID8,MCP2515AD_RXF0EID0,
	MCP2515AD_RXF1SIDH=0x04,MCP2515AD_RXF1SIDL,MCP2515AD_RXF1EID8,MCP2515AD_RXF1EID0,
	MCP2515AD_RXF2SIDH=0x08,MCP2515AD_RXF2SIDL,MCP2515AD_RXF2EID8,MCP2515AD_RXF2EID0,
	//	ポート制御
	MCP2515AD_BFPCTRL=0x0C,
	MCP2515AD_TXRTSCTRL=0x0D,
	//	ステータス読み取り専用
	MCP2515AD_CANSTAT = 0x0E,
	MCP2515AD_CANCTRL = 0x0F,
	//	フィルタ
	MCP2515AD_RXF3SIDH=0x10,MCP2515AD_RXF3SIDL,MCP2515AD_RXF3EID8,MCP2515AD_RXF3EID0,
	MCP2515AD_RXF4SIDH=0x14,MCP2515AD_RXF4SIDL,MCP2515AD_RXF4EID8,MCP2515AD_RXF4EID0,
	MCP2515AD_RXF5SIDH=0x18,MCP2515AD_RXF5SIDL,MCP2515AD_RXF5EID8,MCP2515AD_RXF5EID0,
	//	エラーカウンタ
	MCP2515AD_TEC=0x1C,MCP2515AD_REC,MCP2515AD_CANSTAT1,MCP2515AD_CANCTRL1,
	//	マスク
	MCP2515AD_RXM0SIDH=0x20,MCP2515AD_RXM0SIDL,MCP2515AD_RXM0EID8,MCP2515AD_RXM0EID0,
	MCP2515AD_RXM1SIDH=0x24,MCP2515AD_RXM1SIDL,MCP2515AD_RXM1EID8,MCP2515AD_RXM1EID0,
	//	コンフィギュレーション1
	MCP2515AD_CNFIG3=0x28,MCP2515AD_CNFIG2,MCP2515AD_CNFIG1,
	//	割り込み許可
	MCP2515AD_CANINTE=0x2B,MCP2515AD_CANINTF,MCP2515AD_EFLG,MCP2515AD_CANSTAT2,MCP2515AD_CANCTRL2,
	//	送信バッファ
	MCP2515AD_TXB0CTRL=0x30,MCP2515AD_TXB0SIDH,MCP2515AD_TXB0SIDL,MCP2515AD_TXB0EID8,MCP2515AD_TXB0EID0,MCP2515AD_TXB0DLC,MCP2515AD_TXB0D0,MCP2515AD_TXB0D1,MCP2515AD_TXB0D2,MCP2515AD_TXB0D3,MCP2515AD_TXB0D4,MCP2515AD_TXB0D5,MCP2515AD_TXB0D6,MCP2515AD_TXB0D7,MCP2515AD_CANSTAT3,MCP2515AD_CANCTRL3,
	MCP2515AD_TXB1CTRL=0x40,MCP2515AD_TXB1SIDH,MCP2515AD_TXB1SIDL,MCP2515AD_TXB1EID8,MCP2515AD_TXB1EID0,MCP2515AD_TXB1DLC,MCP2515AD_TXB1D0,MCP2515AD_TXB1D1,MCP2515AD_TXB1D2,MCP2515AD_TXB1D3,MCP2515AD_TXB1D4,MCP2515AD_TXB1D5,MCP2515AD_TXB1D6,MCP2515AD_TXB1D7,MCP2515AD_CANSTAT4,MCP2515AD_CANCTRL4,
	MCP2515AD_TXB2CTRL=0x50,MCP2515AD_TXB2SIDH,MCP2515AD_TXB2SIDL,MCP2515AD_TXB2EID8,MCP2515AD_TXB2EID0,MCP2515AD_TXB2DLC,MCP2515AD_TXB2D0,MCP2515AD_TXB2D1,MCP2515AD_TXB2D2,MCP2515AD_TXB2D3,MCP2515AD_TXB2D4,MCP2515AD_TXB2D5,MCP2515AD_TXB2D6,MCP2515AD_TXB2D7,MCP2515AD_CANSTAT5,MCP2515AD_CANCTRL5,
	//	受信バッファ
	MCP2515AD_RXB0CTRL=0x60,MCP2515AD_RXB0SIDH,MCP2515AD_RXB0SIDL,MCP2515AD_RXB0EID8,MCP2515AD_RXB0EID0,MCP2515AD_RXB0DLC,MCP2515AD_RXB0D0,MCP2515AD_RXB0D1,MCP2515AD_RXB0D2,MCP2515AD_RXB0D3,MCP2515AD_RXB0D4,MCP2515AD_RXB0D5,MCP2515AD_RXB0D6,MCP2515AD_RXB0D7,MCP2515AD_CANSTAT6,MCP2515AD_CANCTRL6,
	MCP2515AD_RXB1CTRL=0x70,MCP2515AD_RXB1SIDH,MCP2515AD_RXB1SIDL,MCP2515AD_RXB1EID8,MCP2515AD_RXB1EID0,MCP2515AD_RXB1DLC,MCP2515AD_RXB1D0,MCP2515AD_RXB1D1,MCP2515AD_RXB1D2,MCP2515AD_RXB1D3,MCP2515AD_RXB1D4,MCP2515AD_RXB1D5,MCP2515AD_RXB1D6,MCP2515AD_RXB1D7,MCP2515AD_CANSTAT7,MCP2515AD_CANCTRL7
};

//----------------------------------------------------------------------------------------
//	デバイス状態定義	コマンド(MCP2515CMD_STATUS)への応答
typedef	union	__mcp2515_status__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	TX2IF	:	1;		//	送信2割り込みフラグ
		unsigned char	TXB2R	:	1;		//	送信2リクエスト
		unsigned char	TX1IF	:	1;		//	送信1割り込みフラグ
		unsigned char	TXB1R	:	1;		//	送信1リクエスト
		unsigned char	TX0IF	:	1;		//	送信0割り込みフラグ
		unsigned char	TXB0R	:	1;		//	送信0リクエスト
		unsigned char	RX1IF	:	1;		//	受信1割り込みフラグ
		unsigned char	RX0IF	:	1;		//	受信0割り込みフラグ
	}	BIT;
}	MCP2515REG_STATUS;

//----------------------------------------------------------------------------------------
//	受信メッセージ状態定義	コマンド(MCP2515CMD_RXSTS)への応答
typedef	union	__mcp2515_rx_status__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	MSG		:	2;		//	メッセージ	0=無し / 1=RXB0有り / 2=RXB1有り / 3=両方に有り
		unsigned char			:	1;		//	
		unsigned char	TYPE	:	2;		//	タイプ		0=標準データ / 1=標準リモート / 2=拡張データ / 3=拡張リモート
		unsigned char	FILT	:	3;		//	一致フィルタ0～5=RXF0～5 / 6=RXF0(RXB1に転送) / 7=RXF1(RXB1に転送)
	}	BIT;
}	MCP2515REG_RX_STS;

//----------------------------------------------------------------------------------------
//	RXnBFピン制御・状態通知レジスタ定義		Address = 0x0C
typedef	union	__mcp2515_bfp_ctrl__
{
	unsigned char	BYTE;
	struct	{
		unsigned char			:	2;		//	[00]
		unsigned char	B1BFS	:	1;		//	出力モード時/RX1BFピンの状態 / 受信割り込みモード時は[0]
		unsigned char	B0BFS	:	1;		//	出力モード時/RX0BFピンの状態 / 受信割り込みモード時は[0]
		unsigned char	B1BFE	:	1;		//	/RX1BFピン機能有効化	1=有効 / 0=無効(HiZ)
		unsigned char	B0BFE	:	1;		//	/RX0BFピン機能有効化	1=有効 / 0=無効(HiZ)
		unsigned char	B1BFM	:	1;		//	/RX1BFピンモード設定	1=RXB1受信時割り込み出力 / 0=B1BFS値出力
		unsigned char	B0BFM	:	1;		//	/RX0BFピンモード設定	1=RXB0受信時割り込み出力 / 0=B1BFS値出力
	}	BIT;
}	MCP2515REG_BFP_CTRL;

//----------------------------------------------------------------------------------------
//	TXnRTSピン制御・状態通知レジスタ定義	Address = 0x0D
typedef	union	__mcp2515_txrts_ctrl__
{
	unsigned char	BYTE;
	struct	{
		unsigned char			:	2;		//	[00]
		unsigned char	B2RTS	:	1;		//	入力モード時/TX2RTSピンの状態 / 送信要求モード時は[0]
		unsigned char	B1RTS	:	1;		//	入力モード時/TX1RTSピンの状態 / 送信要求モード時は[0]
		unsigned char	B0RTS	:	1;		//	入力モード時/TX0RTSピンの状態 / 送信要求モード時は[0]
		unsigned char	B2RTSM	:	1;		//	/TX2RTSピンモード設定	1=TXB2送信要求入力(↓エッジ) / 0=入力
		unsigned char	B1RTSM	:	1;		//	/TX1RTSピンモード設定	1=TXB1送信要求入力(↓エッジ) / 0=入力
		unsigned char	B0RTSM	:	1;		//	/TX0RTSピンモード設定	1=TXB0送信要求入力(↓エッジ) / 0=入力
	}	BIT;
}	MCP2515REG_TXRTS_CTRL;

//----------------------------------------------------------------------------------------
//	CAN状態レジスタ							Address = 0xXE
typedef	union	__mcp2515_stat__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	OPMOD	:	3;		//	動作モード	0=通常 / 1=スリープ / 2=ループバック / 3=リッスン / 4=コンフィグ
		unsigned char			:	1;		//	[0]
		unsigned char	ICOD	:	3;		//	割り込みフラグコード	0=無し / 1=エラー / 2=ウェイクアップ / 3=TXB0 / 4=TXB1 / 5=TXB2 / 6=RXB0 / 7=RXB1
		unsigned char			:	1;		//	[0]
	}	BIT;
}	MCP2515REG_STAT;

//----------------------------------------------------------------------------------------
//	CAN制御レジスタ							Address = 0xXF
typedef	union	__mcp2515_ctrl__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	REQOP	:	3;		//	動作モード要求ビット	0=通常 / 1=スリープ / 2=ループバック / 3=リッスン / 4=コンフィグ
		unsigned char	ABAT	:	1;		//	全ての送信の停止		1=全ての送信バッファの送信停止要求 / 0=停止の終了要求
		unsigned char	OSM		:	1;		//	ワンショットモード		0=要求の都度送信 / 1=1回だけ送信
		unsigned char	CLKEN	:	1;		//	CLKOUTピン有効化		0=無効 / 1=有効
		unsigned char	CLKPRE	:	2;		//	CLKOUTピン分周設定		0=x/1 / 1=x/2 / 2=x/4 / 3=x/8
	}	BIT;
}	MCP2515REG_CTRL;

//----------------------------------------------------------------------------------------
//	CNF3コンフィギュレーション1				Address = 0x28
typedef	union	__mcp2515_cnf3_config__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	SOF		:	1;		//	スタートオブフレームCLKOUT/SOFピン	0=クロック出力 / 1=SOF信号
		unsigned char	WAKFIL	:	1;		//	ウェイクアップフィルタ				0=無効 / 1=有効
		unsigned char			:	3;		//	[000]
		unsigned char	PHSEG2	:	3;		//	PS2長さ				0～7	(PHSEG2+1)*TQ
	}	BIT;
}	MCP2515REG_CNF3;

//----------------------------------------------------------------------------------------
//	CNF2コンフィギュレーション1				Address = 0x29
typedef	union	__mcp2515_cnf2_config__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	BTLMODE	:	1;		//	PS2ビットタイム長	0=PS2の長さはPS1及びIPT(2TQ)より大きくする / 1=PS2の長さはCNF3のPHSEG2で決定
		unsigned char	SAM		:	1;		//	サンプル点コンフィグ	0=1回 / 1=3回　サンプル
		unsigned char	PHSEG1	:	3;		//	PS1長さ				0～7	(PHSEG1+1)*TQ
		unsigned char	PHSEG	:	3;		//	伝播セグメント長さ	0～7	(PHSEG+1)*TQ
	}	BIT;
}	MCP2515REG_CNF2;

//----------------------------------------------------------------------------------------
//	CNF1コンフィギュレーション1				Address = 0x2A
typedef	union	__mcp2515_cnf1_config__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	SJW		:	2;		//	再同期ジャンプ幅長さ	(SJW+1)*TQ
		unsigned char	BRP		:	6;		//	ボーレート分周器		TQ=2*(BRP+1)/Fosc
	}	BIT;
}	MCP2515REG_CNF1;

//----------------------------------------------------------------------------------------
//	割り込み許可							Address = 0x2B
typedef	union	__mcp2515_intf__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	MERRE	:	1;		//	メッセージエラー割り込み		0=禁止 / 1=許可
		unsigned char	WAKIE	:	1;		//	ウェイクアップ割り込み			0=禁止 / 1=許可
		unsigned char	ERRIE	:	1;		//	エラー割り込み					0=禁止 / 1=許可
		unsigned char	TX2IE	:	1;		//	送信バッファ2エンプティ割り込み	0=禁止 / 1=許可
		unsigned char	TX1IE	:	1;		//	送信バッファ1エンプティ割り込み	0=禁止 / 1=許可
		unsigned char	TX0IE	:	1;		//	送信バッファ0エンプティ割り込み	0=禁止 / 1=許可
		unsigned char	RX1IE	:	1;		//	受信バッファ1フル割り込み		0=禁止 / 1=許可
		unsigned char	RX0IE	:	1;		//	受信バッファ0フル割り込み		0=禁止 / 1=許可
	}	BIT;
}	MCP2515REG_CANINTE;

//----------------------------------------------------------------------------------------
//	割り込みフラグ							Address = 0x2C
typedef	union	__mcp2515_intr__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	MERRF	:	1;		//	メッセージエラー割り込み		0=無し / 1=割り込み待ち
		unsigned char	WAKIF	:	1;		//	ウェイクアップ割り込み			0=無し / 1=割り込み待ち
		unsigned char	ERRIF	:	1;		//	エラー割り込み					0=無し / 1=割り込み待ち
		unsigned char	TX2IF	:	1;		//	送信バッファ2エンプティ割り込み	0=無し / 1=割り込み待ち
		unsigned char	TX1IF	:	1;		//	送信バッファ1エンプティ割り込み	0=無し / 1=割り込み待ち
		unsigned char	TX0IF	:	1;		//	送信バッファ0エンプティ割り込み	0=無し / 1=割り込み待ち
		unsigned char	RX1IF	:	1;		//	受信バッファ1フル割り込み		0=無し / 1=割り込み待ち
		unsigned char	RX0IF	:	1;		//	受信バッファ0フル割り込み		0=無し / 1=割り込み待ち
	}	BIT;
}	MCP2515REG_CANINTF;

//----------------------------------------------------------------------------------------
//	エラーフラグ							Address = 0x2D
typedef	union	__mcp2515_errfalg__
{
	unsigned char	BYTE;
	struct	{
		unsigned char	RX1OVR	:	1;		//	受信バッファ1オーバーフロー		1=発生
		unsigned char	RX0OVR	:	1;		//	受信バッファ1オーバーフロー		1=発生
		unsigned char	TXBO	:	1;		//	バスオフエラーフラグ			1=発生
		unsigned char	TXEP	:	1;		//	送信エラー・パッシブエラー		1=発生
		unsigned char	RXEP	:	1;		//	受信エラー・パッシブエラー		1=発生
		unsigned char	TXWAR	:	1;		//	送信エラー警告					1=発生
		unsigned char	RXWAR	:	1;		//	受信エラー警告					1=発生
		unsigned char	EWARN	:	1;		//	TXWAR又はRXWAR発生中			1=発生
	}	BIT;
}	MCP2515REG_EFLG;

//----------------------------------------------------------------------------------------
//	送信バッファ制御レジスタ定義			Address = 0x30 / 0x40 / 0x50
typedef	union	__mcp2515_txb_ctrl__
{
	unsigned char	BYTE;
	struct	{
		unsigned char			:	1;		//	[0]
		unsigned char	ABTF	:	1;		//	メッセージ停止フラグ	1=メッセージは停止された / 0=メッセージ送信正常完了
		unsigned char	MLOA	:	1;		//	アービトレーション消失	1=消失 / 0=現存
		unsigned char	TXERR	:	1;		//	送信エラー検出ビット	1=バスエラー発生 / 0=正常
		unsigned char	TXREQ	:	1;		//	送信要求ビット			1=送信待ち / 0=待ち無し
		unsigned char			:	1;		//	[0]
		unsigned char	TXP		:	2;		//	送信優先順位			0=最低 ～ 3=最高
	}	BIT;
}	MCP2515REG_TXB_CTRL;

//----------------------------------------------------------------------------------------
//	TXBn 送信データバッファ構造体定義		Address = 0x31～0x3D / 0x41～0x4D / 0x51～0x5D
typedef	struct	__mcp2515_txb__
{
	//	送信バッファ制御レジスタ定義			Address = 0x30 / 0x40 / 0x50
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char			:	1;		//	[0]
			unsigned char	ABTF	:	1;		//	メッセージ停止フラグ	1=メッセージは停止された / 0=メッセージ送信正常完了
			unsigned char	MLOA	:	1;		//	アービトレーション消失	1=消失 / 0=現存
			unsigned char	TXERR	:	1;		//	送信エラー検出ビット	1=バスエラー発生 / 0=正常
			unsigned char	TXREQ	:	1;		//	送信要求ビット			1=送信待ち / 0=待ち無し
			unsigned char			:	1;		//	[0]
			unsigned char	TXP		:	2;		//	送信優先順位			0=最低 ～ 3=最高
		}	BIT;
	}	CTRL;
	//	TXBnSIDHレジスタ定義	Address = 0x31 / 0x41 / 0x51
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID10	:	1;		//	
			unsigned char	SID9	:	1;		//	
			unsigned char	SID8	:	1;		//	
			unsigned char	SID7	:	1;		//	
			unsigned char	SID6	:	1;		//	
			unsigned char	SID5	:	1;		//	
			unsigned char	SID4	:	1;		//	
			unsigned char	SID3	:	1;		//	
		}	BIT;
	}	SIDH;
	//	TXBnSIDLレジスタ定義	Address = 0x32 / 0x42 / 0x52
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID2	:	1;		//	
			unsigned char	SID1	:	1;		//	
			unsigned char	SID0	:	1;		//	
			unsigned char			:	1;		//	[0]
			unsigned char	EXIDE	:	1;		//	拡張識別子イネーブルビット	1=拡張 / 0=標準
			unsigned char			:	1;		//	[0]
			unsigned char	EID17	:	1;		//	
			unsigned char	EID16	:	1;		//	
		}	BIT;
	}	SIDL;
	//	TXBnEID8レジスタ定義	Address = 0x33 / 0x43 / 0x53
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID15	:	1;		//	
			unsigned char	EID14	:	1;		//	
			unsigned char	EID13	:	1;		//	
			unsigned char	EID12	:	1;		//	
			unsigned char	EID11	:	1;		//	
			unsigned char	EID10	:	1;		//	
			unsigned char	EID9	:	1;		//	
			unsigned char	EID8	:	1;		//	
		}	BIT;
	}	EID8;
	//	TXBnEID0レジスタ定義	Address = 0x34 / 0x44 / 0x54
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID7	:	1;		//	
			unsigned char	EID6	:	1;		//	
			unsigned char	EID5	:	1;		//	
			unsigned char	EID4	:	1;		//	
			unsigned char	EID3	:	1;		//	
			unsigned char	EID2	:	1;		//	
			unsigned char	EID1	:	1;		//	
			unsigned char	EID0	:	1;		//	
		}	BIT;
	}	EID0;
	//	TXBnDLCレジスタ定義		Address = 0x35 / 0x45 / 0x55
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char			:	1;		//	[0]
			unsigned char	RTR		:	1;		//	リモート送信要求	1=リモートで送信 / 0=データで送信
			unsigned char			:	2;		//	[00]
			unsigned char	DLC		:	4;		//	データ長			0～8バイト
		}	BIT;
	}	DLC;
	//	TXBnDATAレジスタ定義		Address = 0x36～3D / 0x46～4D / 0x56～5D
	unsigned char	DATA[8];
}	MCP2515REG_TXB;

//----------------------------------------------------------------------------------------
//	RXBnCTRLレジスタ定義		Address = 0x60 / 0x70
typedef	union	__mcp2515_rxb_ctrl__
{
	unsigned char	BYTE;
	struct	{
		unsigned char			:	1;		//	[0]
		unsigned char	RXM		:	2;		//	バッファ動作モード	3=全メッセージ受信 / 2=拡張一致 / 1=標準一致 / 0=標準・拡張どちらかに一致
		unsigned char			:	1;		//	[0]
		unsigned char	RTR		:	1;		//	リモート送信要求	1=リモート要求受信 / 0=リモート要求無し
		unsigned char	BUKT	:	1;		//	切り替え許可		1=RXB0がフルならRXB1に受信
		unsigned char	BUKT1	:	1;		//	[R] 同上の内部使用
		unsigned char	FILHIT0	:	1;		//	フィルタ一致		1=RXF1 / 0=RXF0
	}	BIT0;
	struct	{
		unsigned char			:	1;		//	[0]
		unsigned char	RXM		:	2;		//	バッファ動作モード	3=全メッセージ受信 / 2=拡張一致 / 1=標準一致 / 0=標準・拡張どちらかに一致
		unsigned char			:	1;		//	[0]
		unsigned char	RTR		:	1;		//	リモート送信要求	1=リモート要求受信 / 0=リモート要求無し
		unsigned char	FILHIT	:	3;		//	フィルタ一致		5～0=RXF5～RXF0
	}	BIT1;
}	MCP2515REG_RXB_CTRL;

//----------------------------------------------------------------------------------------
//	RXBn 受信データバッファ構造体定義	Address = 0x61～0x6D / 0x71～0x7D
typedef	struct	__mcp2515_rxb__
{
	//	RXBnCTRLレジスタ定義		Address = 0x60 / 0x70
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char			:	1;		//	[0]
			unsigned char	RXM		:	2;		//	バッファ動作モード	3=全メッセージ受信 / 2=拡張一致 / 1=標準一致 / 0=標準・拡張どちらかに一致
			unsigned char			:	1;		//	[0]
			unsigned char	RTR		:	1;		//	リモート送信要求	1=リモート要求受信 / 0=リモート要求無し
			unsigned char	BUKT	:	1;		//	切り替え許可		1=RXB0がフルならRXB1に受信
			unsigned char	BUKT1	:	1;		//	[R] 同上の内部使用
			unsigned char	FILHIT0	:	1;		//	フィルタ一致		1=RXF1 / 0=RXF0
		}	BIT0;
		struct	{
			unsigned char			:	1;		//	[0]
			unsigned char	RXM		:	2;		//	バッファ動作モード	3=全メッセージ受信 / 2=拡張一致 / 1=標準一致 / 0=標準・拡張どちらかに一致
			unsigned char			:	1;		//	[0]
			unsigned char	RTR		:	1;		//	リモート送信要求	1=リモート要求受信 / 0=リモート要求無し
			unsigned char	FILHIT	:	3;		//	フィルタ一致		5～0=RXF5～RXF0
		}	BIT1;
	}	CTRL;
	//	RXBnSIDHレジスタ定義	Address = 0x61 / 0x71
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID10	:	1;		//	
			unsigned char	SID9	:	1;		//	
			unsigned char	SID8	:	1;		//	
			unsigned char	SID7	:	1;		//	
			unsigned char	SID6	:	1;		//	
			unsigned char	SID5	:	1;		//	
			unsigned char	SID4	:	1;		//	
			unsigned char	SID3	:	1;		//	
		}	BIT;
	}	SIDH;
	//	RXBnSIDLレジスタ定義	Address = 0x62 / 0x72
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID2	:	1;		//	
			unsigned char	SID1	:	1;		//	
			unsigned char	SID0	:	1;		//	
			unsigned char	RTR		:	1;		//	標準フレームのリモート要求	1=リモート送信要求受信 / 0=データフレーム受信
			unsigned char	IDE		:	1;		//	拡張識別子イネーブルビット	1=拡張 / 0=標準
			unsigned char			:	1;		//	[0]
			unsigned char	EID17	:	1;		//	
			unsigned char	EID16	:	1;		//	
		}	BIT;
	}	SIDL;
	//	RXBnEID8レジスタ定義	Address = 0x63 / 0x73
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID15	:	1;		//	
			unsigned char	EID14	:	1;		//	
			unsigned char	EID13	:	1;		//	
			unsigned char	EID12	:	1;		//	
			unsigned char	EID11	:	1;		//	
			unsigned char	EID10	:	1;		//	
			unsigned char	EID9	:	1;		//	
			unsigned char	EID8	:	1;		//	
		}	BIT;
	}	EID8;
	//	RXBnEID0レジスタ定義	Address = 0x64 / 0x74
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID7	:	1;		//	
			unsigned char	EID6	:	1;		//	
			unsigned char	EID5	:	1;		//	
			unsigned char	EID4	:	1;		//	
			unsigned char	EID3	:	1;		//	
			unsigned char	EID2	:	1;		//	
			unsigned char	EID1	:	1;		//	
			unsigned char	EID0	:	1;		//	
		}	BIT;
	}	EID0;
	//	RXBnDLCレジスタ定義		Address = 0x65 / 0x75
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char			:	1;		//	[0]
			unsigned char	ERTR	:	1;		//	拡張フレームリモート送信要求	1=リモートで送信 / 0=データで送信
			unsigned char	RB		:	2;		//	予約ビット
			unsigned char	DLC		:	4;		//	データ長			0～8バイト
		}	BIT;
	}	DLC;
	//	RXBnDATAレジスタ定義		Address = 0x66～6D / 0x76～7D
	unsigned char	DATA[8];
}	MCP2515REG_RXB;

//----------------------------------------------------------------------------------------
//	RXFn フィルタレジスタ定義	Address = 0x00～0x03 / 0x04～0x07 / 0x08～0x0B / 0x10～0x13 / 0x14～0x17 / 0x18～0x1B
typedef	struct	__mcp2515_rxfn__
{
	//	RXFnSIDHレジスタ定義	Address = 0x00 / 0x04 / 0x08 / 0x10 / 0x14 / 0x18
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID10	:	1;		//	
			unsigned char	SID9	:	1;		//	
			unsigned char	SID8	:	1;		//	
			unsigned char	SID7	:	1;		//	
			unsigned char	SID6	:	1;		//	
			unsigned char	SID5	:	1;		//	
			unsigned char	SID4	:	1;		//	
			unsigned char	SID3	:	1;		//	
		}	BIT;
	}	SIDH;
	//	RXFnSIDLレジスタ定義	Address = 0x01 / 0x05 / 0x09 / 0x11 / 0x15 / 0x19
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID2	:	1;		//	
			unsigned char	SID1	:	1;		//	
			unsigned char	SID0	:	1;		//	
			unsigned char			:	1;		//	[0]
			unsigned char	EXIDE	:	1;		//	拡張識別子イネーブルビット	1=拡張のみ / 0=標準のみ
			unsigned char			:	1;		//	[0]
			unsigned char	EID17	:	1;		//	
			unsigned char	EID16	:	1;		//	
		}	BIT;
	}	SIDL;
	//	RXFnEID8レジスタ定義	Address = 0x02 / 0x06 / 0x0A / 0x12 / 0x16 / 0x1A
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID15	:	1;		//	
			unsigned char	EID14	:	1;		//	
			unsigned char	EID13	:	1;		//	
			unsigned char	EID12	:	1;		//	
			unsigned char	EID11	:	1;		//	
			unsigned char	EID10	:	1;		//	
			unsigned char	EID9	:	1;		//	
			unsigned char	EID8	:	1;		//	
		}	BIT;
	}	EID8;
	//	RXFnEID0レジスタ定義	Address = 0x03 / 0x07 / 0x0B / 0x13 / 0x17 / 0x1B
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID7	:	1;		//	
			unsigned char	EID6	:	1;		//	
			unsigned char	EID5	:	1;		//	
			unsigned char	EID4	:	1;		//	
			unsigned char	EID3	:	1;		//	
			unsigned char	EID2	:	1;		//	
			unsigned char	EID1	:	1;		//	
			unsigned char	EID0	:	1;		//	
		}	BIT;
	}	EID0;
}	MCP2515REG_RXF;

//----------------------------------------------------------------------------------------
//	RXMn 受信マスクレジスタ定義	Address = 0x20～0x23 / 0x24～0x27
typedef	struct	__mcp2515_rxm__
{
	//	RXMnSIDHレジスタ定義	Address = 0x20 / 0x24
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID10	:	1;		//	
			unsigned char	SID9	:	1;		//	
			unsigned char	SID8	:	1;		//	
			unsigned char	SID7	:	1;		//	
			unsigned char	SID6	:	1;		//	
			unsigned char	SID5	:	1;		//	
			unsigned char	SID4	:	1;		//	
			unsigned char	SID3	:	1;		//	
		}	BIT;
	}	SIDH;
	//	RXMnSIDLレジスタ定義	Address = 0x21 / 0x25
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SID2	:	1;		//	
			unsigned char	SID1	:	1;		//	
			unsigned char	SID0	:	1;		//	
			unsigned char			:	1;		//	[0]
			unsigned char			:	1;		//	[0]
			unsigned char			:	1;		//	[0]
			unsigned char	EID17	:	1;		//	
			unsigned char	EID16	:	1;		//	
		}	BIT;
	}	SIDL;
	//	RXMnEID8レジスタ定義	Address = 0x22 / 0x26
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID15	:	1;		//	
			unsigned char	EID14	:	1;		//	
			unsigned char	EID13	:	1;		//	
			unsigned char	EID12	:	1;		//	
			unsigned char	EID11	:	1;		//	
			unsigned char	EID10	:	1;		//	
			unsigned char	EID9	:	1;		//	
			unsigned char	EID8	:	1;		//	
		}	BIT;
	}	EID8;
	//	RXMnEID0レジスタ定義	Address = 0x23 / 0x27
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	EID7	:	1;		//	
			unsigned char	EID6	:	1;		//	
			unsigned char	EID5	:	1;		//	
			unsigned char	EID4	:	1;		//	
			unsigned char	EID3	:	1;		//	
			unsigned char	EID2	:	1;		//	
			unsigned char	EID1	:	1;		//	
			unsigned char	EID0	:	1;		//	
		}	BIT;
	}	EID0;
}	MCP2515REG_RXM;

//----------------------------------------------------------------------------------------
//	MCP2515内部レジスタ構造体定義
//----------------------------------------------------------------------------------------
typedef	union	__mcp2515_reg__
{
	unsigned long			LONG[64];
	unsigned short			WORD[128];
	unsigned char			BYTE[256];
	struct	{
		//	フィルタ0～2							Address = 0x00～0x0B
		MCP2515REG_RXF			RXF0[3];
		//	RXnBFピン制御・状態通知レジスタ定義		Address = 0x0C
		MCP2515REG_BFP_CTRL		BFP_CTRL;
		//	TXnRTSピン制御・状態通知レジスタ定義	Address = 0x0D
		MCP2515REG_TXRTS_CTRL	TXRTS_CTRL;
		//	CAN状態レジスタ							Address = 0x0E
		MCP2515REG_STAT			CANSTAT0;
		//	CAN制御レジスタ							Address = 0x0F
		MCP2515REG_CTRL			CANCTRL0;
		//	フィルタ3～5							Address = 0x10～0x1B
		MCP2515REG_RXF			RXF1[3];
		//	送信エラーカウンタ						Address = 0x1C
		unsigned char			TEC;
		//	受信エラーカウンタ						Address = 0x1D
		unsigned char			REC;
		//	CAN状態レジスタ							Address = 0x1E
		MCP2515REG_STAT			CANSTAT1;
		//	CAN制御レジスタ							Address = 0x1F
		MCP2515REG_CTRL			CANCTRL1;
		//	マスク0～1								Address = 0x20～0x27
		MCP2515REG_RXM			RXM[2];
		//	CNF3コンフィギュレーション1				Address = 0x28
		MCP2515REG_CNF3			CNF3;
		//	CNF2コンフィギュレーション1				Address = 0x29
		MCP2515REG_CNF2			CNF2;
		//	CNF1コンフィギュレーション1				Address = 0x2A
		MCP2515REG_CNF1			CNF1;
		//	割り込み許可							Address = 0x2B
		MCP2515REG_CANINTE		CANINTE;
		//	割り込みフラグ							Address = 0x2C
		MCP2515REG_CANINTF		CANINTF;
		//	エラーフラグ							Address = 0x2D
		MCP2515REG_CANINTF		EFLG;
		//	CAN状態レジスタ							Address = 0x2E
		MCP2515REG_STAT			CANSTAT2;
		//	CAN制御レジスタ							Address = 0x2F
		MCP2515REG_CTRL			CANCTRL2;
		//	送信バッファ0～2						Address = 0x30～0x5F
		struct	{
			//	送信バッファ制御レジスタ			Address = 0x30 / 0x40 / 0x50
			MCP2515REG_TXB_CTRL		TXBCTRL;
			//	送信バッファフレーム				Address = 0x31～0x3D / 0x41～0x4D / 0x51～0x5D
			MCP2515REG_TXB			FRAME;
			//	CAN状態レジスタ						Address = 0xXE
			MCP2515REG_STAT			STAT;
			//	CAN制御レジスタ						Address = 0xXF
			MCP2515REG_CTRL			CTRL;
		}	TXB[3];
		//	受信バッファ0,1		Address = 0x60～0x7F
		struct	{
			//	受信バッファ制御レジスタ			Address = 0x60 / 0x70
			MCP2515REG_RXB_CTRL		RXBCTRL;
			//	受信バッファフレーム				Address = 0x61～0x6D / 0x71～0x7D
			MCP2515REG_RXB			FRAME;
			//	CAN状態レジスタ						Address = 0xXE
			MCP2515REG_STAT			STAT;
			//	CAN制御レジスタ						Address = 0xXF
			MCP2515REG_CTRL			CTRL;
		}	RXB[2];
	}	REG;
}	MCP2515_REGMAP;

//----------------------------------------------------------------------------------------
//	TXnRTS/RXnBFピン制御レジスタ定義		Address = 0x0C / 0x0D
typedef	union	__mcp2515_bfp_rts_ctrl__
{
	unsigned short	WORD;
	struct	{
		MCP2515REG_BFP_CTRL		BFP;			//	RXnBFピン制御・状態通知レジスタ定義		Address = 0x0C
		MCP2515REG_TXRTS_CTRL	RTS;			//	TXnRTSピン制御・状態通知レジスタ定義	Address = 0x0D
	}	BYTE;
}	MCP2515REG_BFPRTS_CTRL;

//----------------------------------------------------------------------------------------
//	CAN制御レジスタ							Address = 0xXE / 0xXF
typedef	union	__mcp2515_stat_ctrl__
{
	unsigned short	WORD;
	struct	{
		MCP2515REG_STAT		STAT;	//	ステータス
		MCP2515REG_CTRL		CTRL;	//	コントロール
	}	BYTE;
}	MCP2515REG_STATCTRL;

//----------------------------------------------------------------------------------------
//	通信エラーカウンタ						Address = 0x1C / 0x1D
typedef	union	__mcp2515_errcnt__
{
	unsigned short	WORD;
	struct	{
		//	送信エラーカウンタ	Address = 0x1C
		unsigned char	TEC;
		//	受信エラーカウンタ	Address = 0x1D
		unsigned char	REC;
	}	BYTE;
}	MCP2515REG_ERRCNT;


//----------------------------------------------------------------------------------------
//	CAN コンフィギュレーション1				Address = 0x28～0x2B
typedef	union	__mcp2515_config__
{
	unsigned short	WORD[2];
	struct	{
		//	CNF3コンフィギュレーション1			Address = 0x28
		MCP2515REG_CNF3		CNF3;
		//	CNF2コンフィギュレーション1			Address = 0x29
		MCP2515REG_CNF2		CNF2;
		//	CNF1コンフィギュレーション1			Address = 0x2A
		MCP2515REG_CNF1		CNF1;
		//	割り込み許可						Address = 0x2B
		MCP2515REG_CANINTE	CANINTE;
		//	割り込みフラグ						Address = 0x2C
		MCP2515REG_CANINTF	CANINTF;
		//	エラーフラグ						Address = 0x2D
		MCP2515REG_EFLG		EFLG;
	}	BYTE;
}	MCP2515REG_CONFIG;

//----------------------------------------------------------------------------------------
//	CAN コンフィギュレーション1				Address = 0x28～0x2B
typedef	union	__mcp2515_intr_eflg__
{
	unsigned short	WORD;
	struct	{
		//	割り込みフラグ						Address = 0x2C
		MCP2515REG_CANINTF	CANINTF;
		//	エラーフラグ						Address = 0x2D
		MCP2515REG_EFLG		EFLG;
	}	BYTE;
}	MCP2515REG_INTERR;

//----------------------------------------------------------------------------------------
//	TXBnCTRLレジスタ定義		Address = 0x30 / 0x40 / 0x50
typedef	union	__mcp2515_txb_ctrl_buf__
{
	unsigned short	WORD[7];
	unsigned char	BYTE[14];
	struct	{
	//	MCP2515REG_TXB_CTRL		CTRL;		//	送信バッファ制御
		MCP2515REG_TXB			TXB;		//	送信バッファ
	}	REG;
}	MCP2515REG_TXBUF;

//----------------------------------------------------------------------------------------
//	RXBnCTRLレジスタ定義		Address = 0x60 / 0x70
typedef	union	__mcp2515_rxb_ctrl_buf__
{
	unsigned short	WORD[7];
	unsigned char	BYTE[14];
	struct	{
	//	MCP2515REG_RXB_CTRL		CTRL;		//	送信バッファ制御
		MCP2515REG_RXB			RXB;		//	送信バッファ
	}	REG;
}	MCP2515REG_RXBUF;

//----------------------------------------------------------------------------------------
//	ビットセットクリアメッセージ定義	コマンド(MCP2515CMD_BITX)データ
typedef	union	__mcp2515_bit_set_clear__
{
	unsigned short	WORD;
	struct	{
		union	{
			unsigned char			BYTE;		//	マスクパターン(1=変更 / 0=保持)
			MCP2515REG_TXB_CTRL		TXBCTRL;	//	送信バッファ制御
			MCP2515REG_RXB_CTRL		RXBCTRL;	//	受信バッファ制御
			MCP2515REG_CNF3			CNF3;		//	CNF3コンフィギュレーション1		Address = 0x28
			MCP2515REG_CNF2			CNF2;		//	CNF2コンフィギュレーション1		Address = 0x29
			MCP2515REG_CNF1			CNF1;		//	CNF1コンフィギュレーション1		Address = 0x2A
			MCP2515REG_CANINTE		INTE;		//	割り込み許可フラグ
			MCP2515REG_CANINTF		INTF;		//	割り込みフラグ
			MCP2515REG_EFLG			EFLG;		//	エラーフラグ
			MCP2515REG_BFP_CTRL		BFP;		//	RXnBFピン制御・状態通知レジスタ定義	Address = 0x0C
			MCP2515REG_TXRTS_CTRL	RTS;		//	TXnRTSピン制御・状態通知レジスタ定義	Address = 0x0D
			MCP2515REG_CTRL			CTRL;		//	CAN制御レジスタ
		}	MSK;
		union	{
			unsigned char			BYTE;		//	ビットパターン
			MCP2515REG_TXB_CTRL		TXBCTRL;	//	送信バッファ制御
			MCP2515REG_RXB_CTRL		RXBCTRL;	//	受信バッファ制御
			MCP2515REG_CNF3			CNF3;		//	CNF3コンフィギュレーション1		Address = 0x28
			MCP2515REG_CNF2			CNF2;		//	CNF2コンフィギュレーション1		Address = 0x29
			MCP2515REG_CNF1			CNF1;		//	CNF1コンフィギュレーション1		Address = 0x2A
			MCP2515REG_CANINTE		INTE;		//	割り込み許可フラグ
			MCP2515REG_CANINTF		INTF;		//	割り込みフラグ
			MCP2515REG_EFLG			EFLG;		//	エラーフラグ
			MCP2515REG_BFP_CTRL		BFP;		//	RXnBFピン制御・状態通知レジスタ定義	Address = 0x0C
			MCP2515REG_TXRTS_CTRL	RTS;		//	TXnRTSピン制御・状態通知レジスタ定義	Address = 0x0D
			MCP2515REG_CTRL			CTRL;		//	CAN制御レジスタ
		}	PAT;
	}	BYTE;
}	MCP2515REG_BITX;

//----------------------------------------------------------------------------------------
//	MCP2515操作JOB番号定義
//----------------------------------------------------------------------------------------
enum	mcp2515_job	{
	CAN3_JOB_INIT=0,	//	デバイス初期化
	CAN3_JOB_IW1,		//	
	CAN3_JOB_IW2,		//	
	CAN3_JOB_IW3,		//	
	CAN3_JOB_IW4,		//	
	CAN3_JOB_IW5,		//	
	CAN3_JOB_IW6,		//	
	CAN3_JOB_IW7,		//	
	CAN3_JOB_IW8,		//	
	CAN3_JOB_IW9,		//	
	
	CAN3_JOB_WAIT,		//	操作待ち
	CAN3_JOB_WW1,		//	
	CAN3_JOB_WW2,		//	
	CAN3_JOB_WW3,		//	
	CAN3_JOB_WW4,		//	
	CAN3_JOB_WW5,		//	
	
	CAN3_JOB_CHECK,		//	ステータスチェック
	
	
	CAN3_JOB_OVER
};

//________________________________________________________________________________________
//
//	can3_init
//----------------------------------------------------------------------------------------
//	機能説明
//		MCP2515(RSPI2経由)の初期化
//	引数
//		無し
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	RSPI_REQUESTS	*rspi_req;					//	送受信リクエストチェーン変数参照
extern	RSPI_DTC_REQ	*can3_now;					//	送受信中リクエスト
extern	int				can3_job_id;				//	処理番号

extern	void			can3_init(void);			//	CAN3ポート初期化
extern	int				can3_job(void);				//	初期化JOB

//	送信メールボックス書き込み
extern	int				CAN3_TxSet(int mb, SEND_WAIT_FLAME *act);
extern	int				CAN3_GetTxMCTL(int mb);		//	送信バッファ空き確認

//---------------------------------------------------------------------------------------
//  割り込み後ステータス取得後のCallback処理
//---------------------------------------------------------------------------------------
extern	void			can3_sts_event(MCP2515REG_INTERR *rxd);

#endif	/*RSPI2_ACTIVATE*/

#endif	/*__CAN2ECU_CAN3RSPI2_IF__*/


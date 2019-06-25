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
//	MCP2515 CANコントローラ RSPI2-I/F 通信
//
//----------------------------------------------------------------------------------------
//	開発履歴
//
//	2016/12/01	コーディング開始（橘）
//
//----------------------------------------------------------------------------------------
//	T.Tachibana
//	㈱L&F
//________________________________________________________________________________________
//

#include	<sysio.h>
#include	<string.h>
#include	<stdio.h>
#include	"iodefine.h"
#include	"timer.h"
#include	"ecu.h"				/*	ECU 共通定義			*/
#include	"can3_spi2.h"		/*	CAN3 定義				*/
#include	"cantp.h"			/*	CAN-TP 定義				*/

//	メールボックスの使用範囲を固定した用法の場合有効にする
#define		MB_LOCKED_TYPE
//	メールボックスを１つだけ使用する場合有効にする
//#define		MB_USED_ONLYONE

/*
	ポート設定

			Port		SCI			I2C			SPI			適用
	----------------------------------------------------------------------------
	RSPI2	PD2									MISOC		<RSPI>		CAN3
			PD1									MOSIC		<RSPI>		CAN3
			PD3									RSPCKC		<RSPI>		CAN3
			PD4									SSLC0		<RSPI>		CAN3
			PD0									IRQ0		<- CINT		CAN3
			PD6									IRQ6		<- CRX0BF	CAN3
			PD7									IRQ7		<- CRX1BF	CAN3
			P90												-> CTX0RTS	CAN3
			P91												-> CTX1RTS	CAN3
			P92												-> CTX2RTS	CAN3
			P93												-> CRSET	CAN3
			P07												<- CSOF		CAN3

	DTC設定
	
	DTCベクタベース	DTCVBR     $0003E000-$0003EFFF
	
	受信DTCベクタ	DTCE_RSPI2_SPRI2	45		0003E0B4
	送信DTCベクタ	DTCE_RSPI2_SPTI2	46		0003E0B8


*/

//	ログ機能
void	logging(char *fmt, ...);

#ifdef		RSPI2_ACTIVATE

//________________________________________________________________________________________
//
//	CAN3ポート専用変数定義
//________________________________________________________________________________________
//

//	送受信リクエストチェーン変数参照
RSPI_DTC_REQ	*can3_now;			//	転送中の要求バッファポインタ
RSPI_REQUESTS	*rspi_req;			//	要求バッファ
int				rspi_req_WP = 0;	//	書き込み位置
int				rspi_req_RP = 0;	//	読み出し位置
int				rspi_req_PP = 0;	//	処理位置


//	シーケンス
int				can3_job_id = CAN3_JOB_INIT;	//	順序処理
int				stat_event_flag = 0;			//	ステータス受信イベントフラグ

//	送信待ちフラグ
int				tx_act[3] = {0,0,0};
int				tx_act_timer[3] = {0,0,0};

//	プロトタイプ
int can_recv_frame(int ch, void *mbox);
void can_tx_mb(int ch, int mb);
RSPI_DTC_REQ *can3_request(int cmd, int adr, int txlen, int rxlen, void *proc, void *data);

unsigned long	*dtc_table = DTC_VECT_TOP;

void	dtc_init(void)
{
	SYSTEM.PRCR.WORD = 0xA503;		//	プロテクト解除
	MSTP_DTC = 0;					//	DTCモジュールストップ解除
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	DTC.DTCST.BIT.DTCST = 0;		//	DTCモジュール起動ビット
									//	0：DTCモジュール停止
									//	1：DTCモジュール動作
	memset(dtc_table, 0, 0x1000);	//	ベクタ初期化
	DTC.DTCVBR = DTC_VECT_TOP;		//	DTCベクタベースアドレス設定	(0x0003E000)
	DTC.DTCCR.BIT.RRS = 0;			//	DTC転送情報リードスキップ許可ビット
									//	0：転送情報リードスキップを行わない
									//	1：ベクタ番号の値が一致したとき、転送情報リードスキップを行う
	DTC.DTCADMOD.BIT.SHORT = 0;		//	ショートアドレスモード設定ビット
									//	0：フルアドレスモード
									//	1：ショートアドレスモード
	DTC.DTCST.BIT.DTCST = 1;		//	DTCモジュール起動ビット
									//	0：DTCモジュール停止
									//	1：DTCモジュール動作
//	a = DTC.DTCSTS.BIT.ACT;			//	[R]アクティブベクタ番号モニタ(1:動作中)
//	v = DTC.DTCSTS.BIT.VECN;		//	[R]アクティブフラグ(転送中のベクタ番号)
}

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
//					PD6												<- CRX0BF	CAN3
//					PD7												<- CRX1BF	CAN3
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
void rspi2_init(long bps)
{
	SYSTEM.PRCR.WORD = 0xA503;	//	プロテクト解除
	MSTP_RSPI2 = 0;				//	RSPI2モジュールストップ解除

	//	RSPI2 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 エラー割り込み禁止
	ICU.IER[IER_RSPI2_SPRI2].BIT.IEN_RSPI2_SPRI2 = 0;	//	受信バッファフル割り込み禁止
	ICU.IER[IER_RSPI2_SPTI2].BIT.IEN_RSPI2_SPTI2 = 0;	//	送信エンプティ割り込み禁止
	ICU.IER[IER_RSPI2_SPII2].BIT.IEN_RSPI2_SPII2 = 0;	//	送信アイドル割り込み禁止
	ICU.IER[IER_ICU_IRQ0].BIT.IEN_ICU_IRQ0 = 0;			//	モジュール割り込み許可
	ICU.IER[IER_ICU_IRQ6].BIT.IEN_ICU_IRQ6 = 0;			//	モジュール割り込み許可
	ICU.IER[IER_ICU_IRQ7].BIT.IEN_ICU_IRQ7 = 0;			//	モジュール割り込み許可

	RSPI2.SPCR.BYTE = 0;		//	モジュール初期化

	PORTD.PMR.BIT.B2 = 0;		//	周辺機能	MISOC
	PORTD.PMR.BIT.B1 = 0;		//	周辺機能	MOSIC
	PORTD.PMR.BIT.B3 = 0;		//	周辺機能	RSPCKC
	PORTD.PMR.BIT.B4 = 0;		//	周辺機能	SSLC0
	PORTD.PMR.BIT.B0 = 0;		//	周辺機能	CINT
	PORTD.PMR.BIT.B6 = 0;		//	PD6	<- CRX0BF
	PORTD.PMR.BIT.B7 = 0;		//	PD7	<- CRX1BF
	PORT9.PMR.BIT.B0 = 0;		//	P90 -> CTX0RTS
	PORT9.PMR.BIT.B1 = 0;		//	P91 -> CTX1RTS
	PORT9.PMR.BIT.B2 = 0;		//	P92 -> CTX2RTS
	PORT9.PMR.BIT.B3 = 0;		//	P93 -> CRSET
	PORT0.PMR.BIT.B7 = 0;		//	P07 <- CSOF

	// CAN3の端子設定(RSPI2経由)
	PORTD.PODR.BYTE = 0x1F;		//	ポート初期値
	PORT9.PODR.BYTE = 0x07;		//	ポート初期値
	PORTD.PDR.BIT.B2 = 0;		//	PD2 <- MISOC
	PORTD.PDR.BIT.B1 = 1;		//	PD1 -> MOSIC
	PORTD.PDR.BIT.B3 = 1;		//	PD3 -> RSPCKC
	PORTD.PDR.BIT.B4 = 1;		//	PD4 -> SSLC0
	PORTD.PDR.BIT.B0 = 0;		//	PD0 <- CINT
	PORTD.PDR.BIT.B6 = 0;		//	PD6 <- CRX0BF
	PORTD.PDR.BIT.B7 = 0;		//	PD7 <- CRX1BF
	PORT9.PDR.BIT.B0 = 1;		//	P90 -> CTX0RTS
	PORT9.PDR.BIT.B1 = 1;		//	P91 -> CTX1RTS
	PORT9.PDR.BIT.B2 = 1;		//	P92 -> CTX2RTS
	PORT9.PDR.BIT.B3 = 1;		//	P93 -> CRSET
	PORT0.PDR.BIT.B7 = 0;		//	P07 <- CSOF
	PORT9.PODR.BIT.B3 = 0;		//	P93 -> CRSET

	MPC.PWPR.BIT.B0WI = 0;		//	
	MPC.PWPR.BIT.PFSWE = 1;		//	

	MPC.PD2PFS.BYTE = 0x0D;		//	MISOC		SO
	MPC.PD1PFS.BYTE = 0x0D;		//	MOSIC		SI
	MPC.PD3PFS.BYTE = 0x0D;		//	RSPCKC		SCK
	MPC.PD4PFS.BYTE = 0x0D;		//	SSLC0		/CS
	MPC.PD0PFS.BYTE = 0x40;		//	IRQ0		/INT
	MPC.PD6PFS.BYTE = 0x40;		//	IRQ6		/RX0BF
	MPC.PD7PFS.BYTE = 0x40;		//	IRQ7		/RX1BF
	MPC.P90PFS.BYTE = 0x00;		//	port		/TX0RTS
	MPC.P91PFS.BYTE = 0x00;		//	port		/TX1RTS
	MPC.P92PFS.BYTE = 0x00;		//	port		/TX2RTS
	MPC.P93PFS.BYTE = 0x00;		//	port		/RES
	MPC.P07PFS.BYTE = 0x00;		//	port		/CSOF
	
	MPC.PWPR.BIT.PFSWE = 0;		//	
	MPC.PWPR.BIT.B0WI = 1;		//	

	PORTD.PMR.BIT.B2 = 1;		//	周辺機能	MISOC
	PORTD.PMR.BIT.B1 = 1;		//	周辺機能	MOSIC
	PORTD.PMR.BIT.B3 = 1;		//	周辺機能	RSPCKC
	PORTD.PMR.BIT.B4 = 0;		//	周辺機能	SSLC0
	PORTD.PMR.BIT.B0 = 1;		//	周辺機能	CINT
	PORTD.PMR.BIT.B6 = 1;		//	PD6	<- CRX0BF
	PORTD.PMR.BIT.B7 = 1;		//	PD7	<- CRX1BF
	PORT9.PMR.BIT.B0 = 0;		//	P90 -> CTX0RTS
	PORT9.PMR.BIT.B1 = 0;		//	P91 -> CTX1RTS
	PORT9.PMR.BIT.B2 = 0;		//	P92 -> CTX2RTS
	PORT9.PMR.BIT.B3 = 0;		//	P93 -> CRSET
	PORT0.PMR.BIT.B7 = 0;		//	P07 <- CSOF

//	//	CANドライバS端子制御ポート
//	PORT6.PDR.BIT.B0 = 1;		//	P60 -- Port-out CAN0S
//	PORT6.PDR.BIT.B1 = 1;		//	P61 -- Port-out CAN1S
//	PORT6.PDR.BIT.B2 = 1;		//	P62 -- Port-out CAN2S
//	PORT6.PDR.BIT.B3 = 1;		//	P63 -- Port-out CAN3S
//	PORT6.PODR.BYTE = 0x0F;		//	ポート初期化

	SYSTEM.PRCR.WORD = 0xA500;	//	ポート設定禁止
	
	//	RSPI2の設定(シングルマスタモード)
	RSPI2.SSLP.BYTE = 0;		//	SSLnP はアクティブLow
	RSPI2.SPPCR.BYTE = 0x20;	//	MOSIのアイドル出力はLow
	RSPI2.SPSR.BYTE &= 0;		//	エラーフラグ解除
	RSPI2.SPSCR.BYTE = 0;		//	シーケンス初期値

	//	Set baud rate to 1Mbps	N値(BRDV[1:0])=0 固定	最小=93,750bps
	//	n = (PCLK Frequency) / (2 * 2^N * Bit Rate) - 1
	//	n = (48,000,000) / (2 * 2^0 * 1,000,000) - 1
	//	n = 24
	RSPI2.SPBR.BYTE = 48000000 / (2 * bps) - 1;
	RSPI2.SPDCR.BYTE = 0x00;	//	SPDRはワードアクセス / 受信バッファ読み出し / 1フレーム
	RSPI2.SPCKD.BYTE = 1;		//	クロック遅延 1RSPCK
	RSPI2.SSLND.BYTE = 0;		//	SSLネゲート遅延 1RSPCK
	RSPI2.SPND.BYTE = 0;		//	次アクセス遅延 1RSPCK + 2PCLK
	RSPI2.SPCR2.BYTE = 0;		//	パリティ無効 / アイドル割り込み禁止
	
	//	コマンドレジスタ初期化
	//	RSPCK位相設定ビット
	RSPI2.SPCMD0.BIT.CPHA = 0;		//	0 : 奇数エッジでデータサンプル、偶数エッジでデータ変化
									//	1 : 奇数エッジでデータ変化、偶数エッジでデータサンプル
	//	RSPCK極性設定ビット
	RSPI2.SPCMD0.BIT.CPOL = 0;		//	0：アイドル時のRSPCKがLow
									//	1：アイドル時のRSPCKがHigh
	//	ビットレート分周設定ビット
	RSPI2.SPCMD0.BIT.BRDV = 0;		//	b3 b2
									//	0 0：ベースのビットレートを選択
									//	0 1：ベースのビットレートの2分周を選択
									//	1 0：ベースのビットレートの4分周を選択
									//	1 1：ベースのビットレートの8分周を選択
	//	SSL信号アサート設定ビット
	RSPI2.SPCMD0.BIT.SSLA = 0;		//	b6 b4
									//	0 0 0：SSL0
									//	0 0 1：SSL1
									//	0 1 0：SSL2
									//	0 1 1：SSL3
									//	1 x x：設定しないでください
									//	x：Don’t care
	//	SSL信号レベル保持ビット
	RSPI2.SPCMD0.BIT.SSLKP = 0;		//	0：転送終了時に全SSL信号をネゲート
									//	1：転送終了後から次アクセス開始までSSL信号レベルを保持
	//	RSPIデータ長設定ビット
	RSPI2.SPCMD0.BIT.SPB = 0xF;		//	b11 b8
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
	RSPI2.SPCMD0.BIT.LSBF = 0;		//	0：MSBファースト
									//	1：LSBファースト
	//	RSPI次アクセス遅延許可ビット
	RSPI2.SPCMD0.BIT.SPNDEN = 0;	//	0：次アクセス遅延は1RSPCK＋2PCLK
									//	1：次アクセス遅延はRSPI次アクセス遅延レジスタ（SPND）の設定値
	//	SSLネゲート遅延設定許可ビット
	RSPI2.SPCMD0.BIT.SLNDEN = 0;	//	0：SSLネゲート遅延は1RSPCK
									//	1：SSLネゲート遅延はRSPIスレーブセレクトネゲート遅延レジスタ（SSLND）の設定値
	//	RSPCK遅延設定許可ビット
	RSPI2.SPCMD0.BIT.SCKDEN = 0;	//	0：RSPCK遅延は1RSPCK
									//	1：RSPCK遅延はRSPIクロック遅延レジスタ（SPCKD）の設定値

	//	設定コピー
	RSPI2.SPCMD1.WORD = RSPI2.SPCMD0.WORD;
	RSPI2.SPCMD2.WORD = RSPI2.SPCMD0.WORD;
	RSPI2.SPCMD3.WORD = RSPI2.SPCMD0.WORD;
	RSPI2.SPCMD4.WORD = RSPI2.SPCMD0.WORD;
	RSPI2.SPCMD5.WORD = RSPI2.SPCMD0.WORD;
	RSPI2.SPCMD6.WORD = RSPI2.SPCMD0.WORD;
	RSPI2.SPCMD7.WORD = RSPI2.SPCMD0.WORD;
	
	//	動作許可
	//	RSPIモード選択ビット
	RSPI2.SPCR.BIT.SPMS = 1;		//	0：SPI動作（4線式）
									//	1：クロック同期式動作（3線式）
	//	通信動作モード選択ビット
	RSPI2.SPCR.BIT.TXMD = 0;		//	0：全二重同期式シリアル通信
									//	1：送信動作のみのシリアル通信
	//	モードフォルトエラー検出許可ビット
	RSPI2.SPCR.BIT.MODFEN = 0;		//	0：モードフォルトエラー検出を禁止
									//	1：モードフォルトエラー検出を許可
	//	RSPIマスタ/スレーブモード選択ビット
	RSPI2.SPCR.BIT.MSTR = 1;		//	0：スレーブモード
									//	1：マスタモード
	//	RSPIエラー割り込み許可ビット
	RSPI2.SPCR.BIT.SPEIE = 0;		//	0：RSPIエラー割り込み要求の発生を禁止
									//	1：RSPIエラー割り込み要求の発生を許可
	//	RSPI送信割り込み許可ビット
	RSPI2.SPCR.BIT.SPTIE = 0;		//	0：RSPI送信割り込み要求の発生を禁止
									//	1：RSPI送信割り込み要求の発生を許可
	//	RSPI受信割り込み許可ビット
	RSPI2.SPCR.BIT.SPRIE = 0;		//	0：RSPI受信割り込み要求の発生を禁止
									//	1：RSPI受信割り込み要求の発生を許可
	//	RSPIアイドル割り込み許可ビット
	RSPI2.SPCR2.BIT.SPIIE = 0;		//	0：アイドル割り込み要求の発生を禁止
									//	1：アイドル割り込み要求の発生を許可
	//	RSPI機能許可ビット
	RSPI2.SPCR.BIT.SPE = 0;			//	0：RSPI機能は無効
									//	1：RSPI機能が有効

	//	IRQコントロール
	ICU.IRQCR[0].BIT.IRQMD = 1;		//	/INT 信号
	ICU.IRQCR[6].BIT.IRQMD = 1;		//	/RX0BF 受信割り込み信号
	ICU.IRQCR[7].BIT.IRQMD = 1;		//	/RX1BF 受信割り込み信号
									//	b3 b2
									//	0 0：Low
									//	0 1：立ち下がりエッジ
									//	1 0：立ち上がりエッジ
									//	1 1：両エッジ

	ICU.IPR[IPR_ICU_IRQ0].BIT.IPR = 10;					//	割り込みレベル設定
	ICU.IPR[IPR_ICU_IRQ6].BIT.IPR = 9;					//	割り込みレベル設定
	ICU.IPR[IPR_ICU_IRQ7].BIT.IPR = 9;					//	割り込みレベル設定
	ICU.IR[IR_ICU_IRQ0].BIT.IR = 0;						//	割り込みフラグクリア
	ICU.IR[IR_ICU_IRQ6].BIT.IR = 0;						//	割り込みフラグクリア
	ICU.IR[IR_ICU_IRQ7].BIT.IR = 0;						//	割り込みフラグクリア
										

	//	割り込み優先順位設定
	ICU.IPR[IPR_RSPI2_].BIT.IPR = 8;					//	割り込みレベル設定
	//	割り込みフラグクリア
	ICU.IR[IR_RSPI2_SPRI2].BIT.IR = 0;
	ICU.IR[IR_RSPI2_SPTI2].BIT.IR = 0;
	ICU.IR[IR_RSPI2_SPII2].BIT.IR = 0;
	//	GROUP12割り込み設定
	ICU.GEN[GEN_RSPI2_SPEI2].BIT.EN_RSPI2_SPEI2 = 1;	//	グループ12 RSPI1受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 2;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア
	//	割り込み許可設定
	ICU.IER[IER_RSPI2_SPRI2].BIT.IEN_RSPI2_SPRI2 = 1;	//	受信バッファフル割り込み許可
	ICU.IER[IER_RSPI2_SPTI2].BIT.IEN_RSPI2_SPTI2 = 1;	//	送信エンプティ割り込み許可
	ICU.IER[IER_RSPI2_SPII2].BIT.IEN_RSPI2_SPII2 = 0;	//	送信アイドル割り込み許可
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.IER[IER_ICU_IRQ0].BIT.IEN_ICU_IRQ0 = 0;			//	モジュール割り込み許可
	ICU.IER[IER_ICU_IRQ6].BIT.IEN_ICU_IRQ6 = 0;			//	モジュール割り込み許可
	ICU.IER[IER_ICU_IRQ7].BIT.IEN_ICU_IRQ7 = 0;			//	モジュール割り込み許可
	
	//	MCP2515-RESET 解除
//	PORT9.PODR.BIT.B3 = 1;

	dtc_init();	//	DTC初期化
}

//________________________________________________________________________________________
//
//	can3_init
//----------------------------------------------------------------------------------------
//	機能説明
//		MCP2515(RSPI2経由)の初期化
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
//		無し
//	戻り
//		無し
//________________________________________________________________________________________
//
void can3_init(void)
{
	rspi2_init(8000000);	//	8Mbpsで初期化

	rspi_req = (RSPI_REQUESTS *)DTC_REQUEST_TOP;
	memset(rspi_req, 0, sizeof(RSPI_REQUESTS));

	can3_now = 0;			//	現在の処理無し

	CAN3_TX0RTS_PORT = 1;	//	TXB0送信要求解除
	CAN3_TX1RTS_PORT = 1;	//	TXB1送信要求解除
	CAN3_TX2RTS_PORT = 1;	//	TXB2送信要求解除
	
	//	RSPIエラー割り込み許可ビット
	RSPI2.SPCR.BIT.SPEIE = 0;		//	0：RSPIエラー割り込み要求の発生を禁止
									//	1：RSPIエラー割り込み要求の発生を許可
	//	RSPI送信割り込み許可ビット
	RSPI2.SPCR.BIT.SPTIE = 1;		//	0：RSPI送信割り込み要求の発生を禁止
									//	1：RSPI送信割り込み要求の発生を許可
	//	RSPI受信割り込み許可ビット
	RSPI2.SPCR.BIT.SPRIE = 1;		//	0：RSPI受信割り込み要求の発生を禁止
									//	1：RSPI受信割り込み要求の発生を許可
	//	RSPIアイドル割り込み許可ビット
	RSPI2.SPCR2.BIT.SPIIE = 0;		//	0：アイドル割り込み要求の発生を禁止
									//	1：アイドル割り込み要求の発生を許可
	RSPI2.SPCR.BIT.SPE = 0;			//	0：RSPI機能は無効
	DTC.DTCST.BIT.DTCST = 0;		//	モジュール停止

	ICU.IR[IR_ICU_IRQ0].BIT.IR = 0;						//	割り込みフラグクリア
	ICU.IR[IR_ICU_IRQ6].BIT.IR = 0;						//	割り込みフラグクリア
	ICU.IR[IR_ICU_IRQ7].BIT.IR = 0;						//	割り込みフラグクリア
	ICU.IER[IER_ICU_IRQ0].BIT.IEN_ICU_IRQ0 = 0;			//	モジュール割り込み許可
	ICU.IER[IER_ICU_IRQ6].BIT.IEN_ICU_IRQ6 = 0;			//	モジュール割り込み許可
	ICU.IER[IER_ICU_IRQ7].BIT.IEN_ICU_IRQ7 = 0;			//	モジュール割り込み許可
}
int can3_txcheck(void);
void can3_send_nextbyte(void);
void can3_stat_event(MCP2515REG_STATUS *sts);

//	ステータス制御レジスタ状態
MCP2515REG_STATCTRL		mcp_statctrl;
//	ポート設定
MCP2515REG_BFPRTS_CTRL	mcp_bfprts;
//	コンフィギュレーション
MCP2515REG_CONFIG		mcp_config;
//	送信バッファ
MCP2515REG_TXBUF		mcp_txb[3];
//	受信バッファ
MCP2515REG_RXBUF		mcp_rxb[2];
//	ビット変更
MCP2515REG_BITX			mcp_bitx;

//	受信バッファ
CAN_MBOX				mcp_mbx[16];
int						mcp_mbx_wp = 0;
int						mcp_mbx_rp = 0;

//----------------------------------------------------------------------------------------
//	MCP2515ステータス・コントロール読み出し処理関数
//----------------------------------------------------------------------------------------
void can3_getstat_callback(MCP2515REG_STATCTRL *rxd)
{
	mcp_statctrl = *rxd;
}

//---------------------------------------------------------------------------------------
//	処理待ち電文処理
//---------------------------------------------------------------------------------------
void can3_recv_call(void)
{
	int	i;
	if(mcp_mbx_wp != mcp_mbx_rp)
	{
		i = mcp_mbx_rp++;
		mcp_mbx_rp &= 15;
		can_recv_frame(3, (void *)&mcp_mbx[i]);	//	受信データ処理
	}
}
void can3_procwait(void)
{
	RSPI_DTC_REQ	*w;
	int				i;
	unsigned short	sw;

	if(rspi_req_RP != rspi_req_PP)
	{
		w = &rspi_req->REQ[rspi_req_PP];
		if(w->TXL >= 0) return;		//	未完了
		if(w->TXL == -2) return;	//	強制送信中
		rspi_req_PP++;
		if(rspi_req_PP >= CAN3_REQUEST_DTC_MAX) rspi_req_PP = 0;
		if(w->TXL == -1)
		{	//	送信完了
			if(w->RXL > 0)
			{	//	受信データのスワップ実施
				for(i = 0; i < w->RXL; i++)
				{
					sw = w->RXP[i];
					w->RXP[i] = ((sw >> 8) & 0x00FF) | ((sw << 8) & 0xFF00);
				}
			}
			if(w->CALL)
			{	//	ユーザー処理関数呼び出し
				((CAN3_PROC_CALL)w->CALL)((void *)w->RXP);
			}
		}
	}
}

//________________________________________________________________________________________
//
//	can3_job
//----------------------------------------------------------------------------------------
//	機能説明
//		MCP2515起動初期化処理と待ち処理
//	引数
//		無し
//	戻り
//		無し
//________________________________________________________________________________________
//
int can3_job(void)
{
	can3_txcheck();
	can3_procwait();
	can3_recv_call();
	
	switch(can3_job_id)
	{
	case CAN3_JOB_INIT:	//	デバイス初期化
		can3_job_id++;
		memset(&mcp_mbx, 0, sizeof(mcp_mbx));
		can3_request(MCP2515CMD_READ, MCP2515AD_CANSTAT, 0, 1, can3_getstat_callback, 0);		//	ステータス・コントロール取得
		break;
	case CAN3_JOB_IW1:
		if(can3_now == 0)
		{
			can3_job_id++;
			CAN3_RESET_PORT = 1;	//	リセット解除
		}
		break;
	case CAN3_JOB_IW2:
		can3_job_id++;
		can3_request(MCP2515CMD_READ, MCP2515AD_CANSTAT, 0, 1, can3_getstat_callback, 0);		//	ステータス・コントロール取得
		break;
	case CAN3_JOB_IW3:
		if(can3_now == 0)
		{
			if(mcp_statctrl.BYTE.STAT.BIT.OPMOD == 4 && (mcp_statctrl.BYTE.CTRL.BIT.REQOP == 15 || mcp_statctrl.BYTE.CTRL.BIT.REQOP == 4))
			{	//	リセット直後の状態と一致
				can3_job_id++;
			}
			else
			{
				can3_job_id = CAN3_JOB_IW2;	//	再取得
			}
		}
		break;
	case CAN3_JOB_IW4:
		can3_job_id++;
		mcp_bitx.BYTE.MSK.BYTE = 0xE0;
		mcp_bitx.BYTE.PAT.BYTE = 0x80;
		can3_request(MCP2515CMD_BITX, MCP2515AD_CANCTRL, 1, 0, 0, &mcp_bitx);				//	モード切り替え
		can3_request(MCP2515CMD_READ, MCP2515AD_CANSTAT, 0, 1, can3_getstat_callback, 0);	//	ステータス・コントロール取得
		break;
	case CAN3_JOB_IW5:
		if(can3_now == 0)
		{
			if(mcp_statctrl.BYTE.STAT.BIT.OPMOD == 4 && mcp_statctrl.BYTE.CTRL.BIT.REQOP != 4)
			{	//	コンフィグへの変化待ち
				can3_job_id = CAN3_JOB_IW4;
			}
			else
			{	//	コンフィグモード開始
				can3_job_id++;
			}
		}
		break;
	case CAN3_JOB_IW6:	//	パラメータ設定
		can3_job_id++;
		memset(&mcp_config, 0, sizeof(mcp_config));
		memset(&mcp_bfprts, 0, sizeof(mcp_bfprts));
		//	入出力ポート機能設定
		mcp_bfprts.BYTE.BFP.BYTE = 0x0F;	//	RXnBF = 受信バッファフル割り込み出力
		mcp_bfprts.BYTE.RTS.BYTE = 0x07;	//	TXnRTS = TXBnREQ送信要求入力
		can3_request(MCP2515CMD_WRITE, MCP2515AD_BFPCTRL, 1, 0, 0, &mcp_bfprts);	//	ポート設定
		
		//	ボーレート設定(Fosc=20MHz : Tosc=50ns(1s/20M) : CAN=500Kbps)
		//			   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
		//	tosc	|_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| 
		//			   _0_     _1_     _2_     _3_     _4_     _5_     _6_     _7_     _8_     _9_     _0_     _1_     _
		//	TBRPCLK	__|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___| 
		//			   _______ _______________________________________________________________________                  
		//	tBIT	__|__Sync_|_____PropSeg___|__________PS1__________|______________PS2______________|_________________
		//			   _______________________________________________|_________________________________________________
		//	tTQ		__|_______|_______|_______|_______|_______|_______|_______|_______|_______|_______|_______|_______|_
		//			  |                                               |                               |                 
		//	CANBIT	->|<----------------------------------------------A------------------------------>|<----------------
		//			                                               (Sample)                                             
		//	CAN1Bit:10TQ として TQ=1/500K/10=200ns : BRP=TQ/(2*Tosc)-1=200ns/100ns-1=2-1 従いBRP=1 : CANBIT=10TQ
		//	条件 : PropSeg + PS1 >= PS2 : PropSeg + PS1 >= TDELAY : PS2 > SJW
		//	SyncSeg(1) + PropSeg(2) + PS1(3) + PS2(4) = 10TQ : SJW = PS2 - 2 = 2TQ : TDELAY = 2TQ
		mcp_config.BYTE.CNF1.BIT.BRP = 2-1;			//	BRP = TQ / (2 * Tosc)	Tosc=50ns(20MHz)	TQ=	200ns
		mcp_config.BYTE.CNF1.BIT.SJW = 3-1;			//	SJW = (SJW+1)*TQ		再同期ジャンプ幅
		mcp_config.BYTE.CNF2.BIT.PHSEG = 2-1;		//	PropSeg = (PHSEG+1)*TQ	伝搬セグメント			400ns
		mcp_config.BYTE.CNF2.BIT.PHSEG1 = 3-1;		//	PS1 = (PHSEG1+1)*TQ		フェーズセグメント１	600ns
		mcp_config.BYTE.CNF2.BIT.SAM = 0;			//	0=1回サンプル(1=3回サンプル)
		mcp_config.BYTE.CNF2.BIT.BTLMODE = 1;		//	PHSEG2で決定
		mcp_config.BYTE.CNF3.BIT.PHSEG2 = 4-1;		//	PS2 = (PHSEG2+1)*TQ		フェーズセグメント２	800ns
		mcp_config.BYTE.CNF3.BIT.WAKFIL = 0;		//	ウェイクアップフィルタ無効
		mcp_config.BYTE.CNF3.BIT.SOF = 0;			//	SOF出力
		
		//	(/INT 出力)割り込み許可設定
		mcp_config.BYTE.CANINTE.BYTE = 0x1F;		//	MERRE / WAKIE 禁止 : ERRIE / TX2IE / TX1IE / TX0IE / RX1IE / RX0IE 許可
		can3_request(MCP2515CMD_WRITE, MCP2515AD_CNFIG3, 2, 0, 0, &mcp_config);	//	ボーレート設定＋割り込み許可
		
		//	送信バッファの設定
		memset(&mcp_txb[0], 0, 14);
		mcp_txb[0].REG.TXB.CTRL.BYTE = 0;	//	優先度「高」
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB0CTRL, 7, 0, 0, &mcp_txb[0]);	//	TXB0設定保存
		memset(&mcp_txb[1], 0, 14);
		mcp_txb[1].REG.TXB.CTRL.BYTE = 0;	//	優先度「中」
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB1CTRL, 7, 0, 0, &mcp_txb[1]);	//	TXB1設定保存
		memset(&mcp_txb[2], 0, 14);
		mcp_txb[2].REG.TXB.CTRL.BYTE = 0;	//	優先度「低」
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB2CTRL, 7, 0, 0, &mcp_txb[2]);	//	TXB2設定保存
		
		//	受信バッファの設定
		memset(&mcp_rxb[0], 0, 14);
		mcp_rxb[0].REG.RXB.CTRL.BYTE = 0x64;	//	全メッセージ受信 & 切り替え許可ビット
		can3_request(MCP2515CMD_WRITE, MCP2515AD_RXB0CTRL, 7, 0, 0, &mcp_rxb[0]);	//	RXB0コントロール
		memset(&mcp_rxb[1], 0, 14);
		mcp_rxb[1].REG.RXB.CTRL.BYTE = 0x60;	//	全メッセージ受信
		can3_request(MCP2515CMD_WRITE, MCP2515AD_RXB1CTRL, 7, 0, 0, &mcp_rxb[1]);	//	RXB1コントロール
		break;
	case CAN3_JOB_IW7:	//	パラメータ設定完了待ち
		if(can3_now == 0)
		{
			can3_job_id++;
		}
		break;
	case CAN3_JOB_IW8:	//	CANスタート
		can3_job_id++;
		mcp_bitx.BYTE.MSK.BYTE = 0xE0;	//	モード選択
		mcp_bitx.BYTE.PAT.BYTE = 0x00;	//	通常モード(0)
		can3_request(MCP2515CMD_BITX, MCP2515AD_CANCTRL, 1, 0, 0, &mcp_bitx);				//	モード切り替え
		can3_request(MCP2515CMD_READ, MCP2515AD_CANSTAT, 0, 1, can3_getstat_callback, 0);	//	ステータス取得
		break;
	case CAN3_JOB_IW9:	//	パラメータ設定完了待ち
		if(can3_now == 0)
		{
			if(mcp_statctrl.BYTE.STAT.BIT.OPMOD != 0)
			{	//	コンフィグへの変化待ち
				can3_job_id = CAN3_JOB_IW8;		//	再設定
			}
			else
			{	//	設定完了
				can3_job_id++;
			}
		}
		break;

	case CAN3_JOB_WAIT:	//	操作待ち
		if(rspi_req_RP == rspi_req_WP)
		{
			can3_job_id = CAN3_JOB_WW1;
			stat_event_flag = 100;
			can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0);	//	ステータス取得
		}
		break;
	case CAN3_JOB_WW1:	//	ステータス取得待ち
		if(rspi_req_RP == rspi_req_WP)
		{	//	送信済み
			if(stat_event_flag == 0)
			{	//	受信完了
				can3_job_id = CAN3_JOB_WAIT;
			}
			else
			{	//	タイムアウト検出
				stat_event_flag--;
			}
		}
		break;
	}
	return ((can3_job_id >= CAN3_JOB_WAIT) ? 1 : 0);	//	初期化中は0を返す
}

//________________________________________________________________________________________
//
//	送信処理
//________________________________________________________________________________________
//
int can3_txcheck(void)
{
	for(;can3_now == 0 && rspi_req_RP != rspi_req_WP;)
	{	//	待ち有り
		can3_now = &rspi_req->REQ[rspi_req_RP++];
		if(rspi_req_RP >= CAN3_REQUEST_DTC_MAX) rspi_req_RP = 0;
		if(can3_now->TXL < 0)
		{	//	送信済み、パスする
			can3_now = 0;
			continue;
		}
		CAN3_SPI_CS_PORT = 1;				//	CS許可
		DTC.DTCST.BIT.DTCST = 0;			//	モジュール停止
		RSPI2.SPCR.BIT.SPE = 0;				//	0：RSPI機能は無効
		RSPI2.SPCR.BIT.SPRIE = 0;			//	1：RSPI受信割り込み要求の発生を許可
		RSPI2.SPCR.BIT.SPTIE = 0;			//	1：RSPI送信割り込み要求の発生を許可
		ICU.IR[IR_RSPI2_SPRI2].BIT.IR = 0;	//	割り込みフラグクリア
		ICU.IR[IR_RSPI2_SPTI2].BIT.IR = 0;	//	割り込みフラグクリア
		dtc_table[DTCE_RSPI2_SPTI2] = (unsigned long)&can3_now->DTCTX.LONG[0];
		dtc_table[DTCE_RSPI2_SPRI2] = (unsigned long)&can3_now->DTCRX.LONG[0];
		ICU.DTCER[ DTCE_RSPI2_SPRI2 ].BIT.DTCE = 1;			//	RSPI受信割り込みによるDTC起動許可
		ICU.DTCER[ DTCE_RSPI2_SPTI2 ].BIT.DTCE = 1;			//	RSPI送信割り込みによるDTC起動許可
		RSPI2.SPCR.BIT.SPRIE = 1;			//	1：RSPI受信割り込み要求の発生を許可
		RSPI2.SPCR.BIT.SPTIE = 1;			//	1：RSPI送信割り込み要求の発生を許可
		ICU.IR[IR_RSPI2_SPRI2].BIT.IR = 0;	//	割り込みフラグクリア
		ICU.IR[IR_RSPI2_SPTI2].BIT.IR = 0;	//	割り込みフラグクリア
		DTC.DTCST.BIT.DTCST = 1;			//	DTC許可
		CAN3_SPI_CS_PORT = 0;				//	CS許可
		RSPI2.SPCR.BIT.SPE = 1;				//	SPI許可
		return 1;	//	仕掛有り
	}
	return 0;	//	仕掛無し
}

//---------------------------------------------------------------------------------------
//	SPRI2 受信バッファフル	SPI受信バッファにデータ到着
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_RSPI2_SPRI2} RSPI2_SPRI2_ISR(void)
{
//	int	i;
//	unsigned short sw;
	//	CS解除
	CAN3_SPI_CS_PORT = 1;
	RSPI2.SPCR.BIT.SPRIE = 0;		//	0：RSPI受信割り込み要求の発生を禁止
	RSPI2.SPCR.BIT.SPE = 0;			//	RSPI2停止
	DTC.DTCST.BIT.DTCST = 0;		//	DTC停止
//	if(can3_now->RXL == 0 && can3_now->CALL != 0)
//	{	//	送信のみのコールバック有りは処理する
//		((CAN3_PROC_CALL)can3_now->CALL)(0);
//	}
	can3_now->TXL = -1;				//	終了マーク
	can3_now = 0;
}

//---------------------------------------------------------------------------------------
//	SPTI2 送信バッファエンプティ	SPI送信バッファが空いた
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_RSPI2_SPTI2} RSPI2_SPTI2_ISR(void)
{
	RSPI2.SPCR.BIT.SPTIE = 0;		//	0：RSPI送信割り込み要求の発生を禁止
}

//---------------------------------------------------------------------------------------
//	SPII2 アイドル	送信データの最終書き込み完了時に割り込み発生（送信完了）
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_RSPI2_SPII2} RSPI2_SPII2_ISR(void)
{
	RSPI2.SPCR2.BIT.SPIIE = 0;		//	0：アイドル割り込み要求の発生を禁止
	logging("IDLE\r");
}

//________________________________________________________________________________________
//
//	can3_request
//----------------------------------------------------------------------------------------
//	機能説明
//		SPI通信要求の積み上げ
//	引数
//		int cmd		コマンドコード
//		int adr		転送先アドレス
//		int txlen	送信バイト数
//		int rxlen	受信バイト数
//		void *proc	完了時呼び出し先関数
//		void *data	送信データポインタ
//	戻り
//		無し
//________________________________________________________________________________________
//
RSPI_DTC_REQ *can3_request(int cmd, int adr, int txlen, int rxlen, void *proc, void *data)
{
	int				i, j;
	RSPI_DTC_REQ	*act;
	unsigned short	*dp, sw;
	
	//	バッファ取得
	act = &rspi_req->REQ[rspi_req_WP++];
	if(rspi_req_WP >= CAN3_REQUEST_DTC_MAX) rspi_req_WP = 0;
	dp = (unsigned short *)data;
	i = 0;	//	送信データポインタ
	//	コマンド登録
	act->TXL = txlen;		//	送信データ数
	act->RXL = rxlen;		//	受信データ数
	act->DAT[i++] = (unsigned short)((cmd << 8) | adr);	//	コマンドコード
	//	送信データコピー
	for(j = 0; j < txlen; j++)
	{
		sw = dp[j];
		act->DAT[i++] = ((sw >> 8) & 0x00FF) | ((sw << 8) & 0xFF00);
	}
	act->RXP = &act->DAT[i];
	//	受信データダミー
	for(; j < rxlen; j++) act->DAT[i++] = 0;
	//	完了時呼び出し先登録
	act->CALL = proc;
	//	DTC要求情報設定
	//	送信
	act->DTCTX.REG.MR.LONG = 0x18000000;
	act->DTCTX.REG.SAR = (unsigned long)&act->DAT[0];	//	転送元：送信データバッファ
	act->DTCTX.REG.DAR = (unsigned long)&RSPI2.SPDR;	//	転送先：SPIデータレジスタ
	act->DTCTX.REG.CR.NOR.A = i;
	act->DTCTX.REG.CR.NOR.B = i;
	//	受信
	act->DTCRX.REG.MR.LONG = 0x10080000;
	act->DTCRX.REG.SAR = (unsigned long)&RSPI2.SPDR;	//	転送元：SPIデータレジスタ
	act->DTCRX.REG.DAR = (unsigned long)&act->DAT[0];	//	転送先：受信データバッファ
	act->DTCRX.REG.CR.NOR.A = i;
	act->DTCRX.REG.CR.NOR.B = i;
	return act;
}

void CAN3_GetRx0(void);
void CAN3_GetRx1(void);
//---------------------------------------------------------------------------------------
//	受信データ取得コールバック
//---------------------------------------------------------------------------------------
void CAN3_CallbackRx0(MCP2515REG_RXBUF *rxd)
{
	CAN_MBOX	*mbx;
	mbx = &mcp_mbx[mcp_mbx_wp++];
	mcp_mbx_wp &= 15;
//	mcp_mbx[0].ID.LONG = 0;
//	mcp_mbx[0].ID.BIT.IDE = rxd->BYTE.RXB.SIDL.BIT.IDE;
	mbx->ID.BIT.RTR = rxd->REG.RXB.SIDL.BIT.RTR;
	mbx->ID.BIT.SID = ((((unsigned long)rxd->REG.RXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)rxd->REG.RXB.SIDL.BYTE) >> 5) & 0x007);
	mbx->DLC = rxd->REG.RXB.DLC.BIT.DLC;
	memcpy(mbx->DATA, rxd->REG.RXB.DATA, 8);
//	mcp_mbx[mcp_mbx_wp].TS++;
//	can_recv_frame(3, (void *)&mbx);	//	ECU受信データ取得
//	can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0);	//	ステータス取得
//	if(mbx->ID.BIT.SID == led_monit_id && (led_monit_ch & 8) != 0) PORTE.PODR.BIT.B0 = 0;
	if(mbx->ID.BIT.SID == led_monit_id && (led_monit_ch & 0x80) != 0)
	{
		led_monit_ch &= 0x8F;
		PORTE.PODR.BIT.B0 = 0;
		cmt1_start(1000000, monit_timeover);
	}
}
void CAN3_CallbackRx1(MCP2515REG_RXBUF *rxd)
{
	CAN_MBOX	*mbx;
	mbx = &mcp_mbx[mcp_mbx_wp++];
	mcp_mbx_wp &= 15;
//	mbx.ID.LONG = 0;
//	mbx.ID.BIT.IDE = rxd->REG.RXB.SIDL.BIT.IDE;
	mbx->ID.BIT.RTR = rxd->REG.RXB.SIDL.BIT.RTR;
	mbx->ID.BIT.SID = ((((unsigned long)rxd->REG.RXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)rxd->REG.RXB.SIDL.BYTE) >> 5) & 0x007);
	mbx->DLC = rxd->REG.RXB.DLC.BIT.DLC;
	memcpy(mbx->DATA, rxd->REG.RXB.DATA, 8);
//	mcp_mbx[mcp_mbx_wp].TS++;
//	can_recv_frame(3, (void *)&mbx);	//	ECU受信データ取得
//	can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0);	//	ステータス取得
//	if(mbx->ID.BIT.SID == led_monit_id && (led_monit_ch & 8) != 0) PORTE.PODR.BIT.B0 = 0;
	if(mbx->ID.BIT.SID == led_monit_id && (led_monit_ch & 0x80) != 0)
	{
		led_monit_ch &= 0x8F;
		PORTE.PODR.BIT.B0 = 0;
		cmt1_start(1000000, monit_timeover);
	}
}

//---------------------------------------------------------------------------------------
//	送信メールボックス書き込みコールバック（送信開始要求）
//---------------------------------------------------------------------------------------
void CAN3_CallbackTxSet0(MCP2515REG_TXBUF *rxd)	//MCP2515REG_TXBUF *txr)
{
	if(memcmp(&mcp_txb[0].BYTE[1], &rxd->BYTE[1], 13) == 0)
	{
		CAN3_TX0RTS_PORT = 0;	//	TX0送信要求
		tx_act[0] = 2;
		CAN3_TX0RTS_PORT = 1;	//	TX0送信要求
	}
	else
	{
		logging("CAN3 TX0-Readback Error\r");
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB0CTRL, 7, 0, 0, &mcp_txb[0]);
		can3_request(MCP2515CMD_READ, MCP2515AD_TXB0CTRL, 0, 7, CAN3_CallbackTxSet0, 0);
	}
}
void CAN3_CallbackTxSet1(MCP2515REG_TXBUF *rxd)	//MCP2515REG_TXBUF *txr)
{
	if(memcmp(&mcp_txb[1].BYTE[1], &rxd->BYTE[1], 13) == 0)
	{
		CAN3_TX1RTS_PORT = 0;	//	TX1送信要求
		tx_act[1] = 2;
		CAN3_TX1RTS_PORT = 1;	//	TX1送信要求
	}
	else
	{
		logging("CAN3 TX1-Readback Error\r");
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB1CTRL, 7, 0, 0, &mcp_txb[1]);
		can3_request(MCP2515CMD_READ, MCP2515AD_TXB1CTRL, 0, 7, CAN3_CallbackTxSet1, 0);
	}
}
void CAN3_CallbackTxSet2(MCP2515REG_TXBUF *rxd)	//MCP2515REG_TXBUF *txr)
{
	if(memcmp(&mcp_txb[2].BYTE[1], &rxd->BYTE[1], 13) == 0)
	{
		CAN3_TX2RTS_PORT = 0;	//	TX2送信要求
		tx_act[2] = 2;
		CAN3_TX2RTS_PORT = 1;	//	TX2送信要求
	}
	else
	{
		logging("CAN3 TX2-Readback Error\r");
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB2CTRL, 7, 0, 0, &mcp_txb[2]);
		can3_request(MCP2515CMD_READ, MCP2515AD_TXB2CTRL, 0, 7, CAN3_CallbackTxSet2, 0);
	}
}

//---------------------------------------------------------------------------------------
//	送信メールボックス空き状態取得
//---------------------------------------------------------------------------------------
int CAN3_GetTxMCTL(int mb)
{
#ifdef	MB_USED_ONLYONE
	if(tx_act[0] == 0) return 0;
	//	タイムアウトMBの検索
	if(tx_act_timer[0] > 0) tx_act_timer[0]--;
	if(tx_act_timer[0] == 0)
	{
		tx_act[0] = 0;
		logging("CAN3 TXMB0:Timeout\r");
		return 0;	//	タイムアウト空き
	}
#else
	int	rc;
	rc = rspi_req_WP - rspi_req_RP;
	if(rc < 0) rc += CAN3_REQUEST_DTC_MAX;
	if(rc > (CAN3_REQUEST_DTC_MAX - 8))
	{	//	バッファ危険
		return 0xC0;	//	バッファ空き不足
	}

#ifdef	MB_LOCKED_TYPE
	if(tx_act[mb] == 0) return 0;
	//	タイムアウトMBの検索
	if(tx_act_timer[mb] > 0) tx_act_timer[mb]--;
	if(tx_act_timer[mb] == 0)
	{
		tx_act[mb] = 0;
	//	logging("CAN3 TXMB%d:Timeout\r", mb);
		return mb;	//	タイムアウト空き
	}
#else
	if(tx_act[0] == 0) return 0;
	if(tx_act[1] == 0) return 1;
	if(tx_act[2] == 0) return 2;
	
	//	タイムアウトMBの検索
	if(tx_act_timer[0] > 0) tx_act_timer[0]--;
	if(tx_act_timer[0] == 0)
	{
		tx_act[0] = 0;
	//	logging("CAN3 TXMB0:Timeout\r");
		return 0;	//	タイムアウト空き
	}
	if(tx_act_timer[1] > 0) tx_act_timer[1]--;
	if(tx_act_timer[1] == 0)
	{
		tx_act[1] = 0;
	//	logging("CAN3 TXMB1:Timeout\r");
		return 1;	//	タイムアウト空き
	}
	if(tx_act_timer[2] > 0) tx_act_timer[2]--;
	if(tx_act_timer[2] == 0)
	{
		tx_act[2] = 0;
	//	logging("CAN3 TXMB2:Timeout\r");
		return 2;	//	タイムアウト空き
	}
#endif
#endif
	return 0xC0;	//	空き無し
}

//---------------------------------------------------------------------------------------
//	送信メールボックス書き込み
//---------------------------------------------------------------------------------------
int CAN3_TxSet(int mb, SEND_WAIT_FLAME *act)
{
	unsigned char		ctrl = 3 - mb;
	//	リクエストの有無チェック
#ifdef	MB_LOCKED_TYPE
	if(tx_act[mb] != 0) return -1;	//	空き無し
#else
	mb = 0;
	if(tx_act[0] != 0)
	{
		mb++;
		if(tx_act[1] != 0)
		{
			mb++;
			if(tx_act[2] != 0)
			{
				return -1;	//	空き無し
			}
		}
	}
#endif
	tx_act[mb] = 1;
	tx_act_timer[mb] = 1000;
	//	バッファ転送
	mcp_txb[mb].REG.TXB.CTRL.BYTE = ctrl;	//	優先度
	mcp_txb[mb].REG.TXB.SIDH.BYTE = (unsigned char)(act->ID.BIT.SID >> 3);
	mcp_txb[mb].REG.TXB.SIDL.BYTE = (act->ID.BIT.SID << 5) & 0xE0;
	mcp_txb[mb].REG.TXB.EID8.BYTE = 0;
	mcp_txb[mb].REG.TXB.EID0.BYTE = 0;
	mcp_txb[mb].REG.TXB.DLC.BIT.RTR = act->ID.BIT.RTR;
	mcp_txb[mb].REG.TXB.DLC.BIT.DLC = act->ID.BIT.DLC;
	memcpy(mcp_txb[mb].REG.TXB.DATA, act->FD.BYTE, 8);
	//	SPI送信処理
	switch(mb)
	{
	case 0:
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB0CTRL, 7, 0, 0, &mcp_txb[mb]);
		can3_request(MCP2515CMD_READ, MCP2515AD_TXB0CTRL, 0, 7, CAN3_CallbackTxSet0, 0);
		break;
	case 1:
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB1CTRL, 7, 0, 0, &mcp_txb[mb]);
		can3_request(MCP2515CMD_READ, MCP2515AD_TXB1CTRL, 0, 7, CAN3_CallbackTxSet1, 0);
		break;
	case 2:
		can3_request(MCP2515CMD_WRITE, MCP2515AD_TXB2CTRL, 7, 0, 0, &mcp_txb[mb]);
		can3_request(MCP2515CMD_READ, MCP2515AD_TXB2CTRL, 0, 7, CAN3_CallbackTxSet2, 0);
		break;
	}
//	can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0);	//	ステータス取得
	return 0;
}


//---------------------------------------------------------------------------------------
// 高速ステータス取得後のCallback処理
//---------------------------------------------------------------------------------------
void can3_stat_event(MCP2515REG_STATUS *sts)
{
	int	id;
	unsigned short	buf = 0x0000;
	
	stat_event_flag = 0;
	
#if 1
	if(sts->BIT.RX0IF != 0)
	{	//	受信バッファ0フル
		can3_request(MCP2515CMD_READ, MCP2515AD_RXB0CTRL, 0, 7, CAN3_CallbackRx0, 0);	//	RXB0読み出し
		buf |= 0x01;
	}
	if(sts->BIT.RX1IF != 0)
	{	//	受信バッファ1フル
		can3_request(MCP2515CMD_READ, MCP2515AD_RXB1CTRL, 0, 7, CAN3_CallbackRx1, 0);	//	RXB1読み出し
		buf |= 0x02;
	}
#endif
	//	TXB0
	if(sts->BIT.TXB0R != 0)
	{
		if(tx_act[0] == 2) tx_act[0] = 3;
	}
	else
	{
		if(tx_act[0] == 3)
		{
			tx_act[0] = 0;
			id = ((((unsigned long)mcp_txb[0].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)mcp_txb[0].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
			if(led_monit_ch & 8)
			{
			//	if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x08) != 0)
				{
					led_monit_ch &= 0xF8;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
			}
			can_tp_txecheck(3, id);	//	TP送信完了確認
		}
	}
	if(sts->BIT.TX0IF != 0)
	{	//	終了している
		if(sts->BIT.TXB0R == 0 && tx_act[0] == 2)
		{
			tx_act[0] = 0;
			id = ((((unsigned long)mcp_txb[0].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)mcp_txb[0].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
			if(led_monit_ch & 8)
			{
			//	if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x08) != 0)
				{
					led_monit_ch &= 0xF8;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
			}
			can_tp_txecheck(3, id);	//	TP送信完了確認
		}
		buf |= 0x04;
	}
	
	//	TXB1
	if(sts->BIT.TXB1R != 0)
	{
		if(tx_act[1] == 2) tx_act[1] = 3;
	}
	else
	{
		if(tx_act[1] == 3)
		{
			tx_act[1] = 0;
			id = ((((unsigned long)mcp_txb[1].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)mcp_txb[1].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
			if(led_monit_ch & 8)
			{
			//	if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x08) != 0)
				{
					led_monit_ch &= 0xF8;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
			}
			can_tp_txecheck(3, id);	//	TP送信完了確認
		}
	}
	if(sts->BIT.TX1IF != 0)
	{	//	終了している
		if(sts->BIT.TXB1R == 0 && tx_act[1] == 2)
		{
			tx_act[1] = 0;
			id = ((((unsigned long)mcp_txb[1].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)mcp_txb[1].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
			if(led_monit_ch & 8)
			{
			//	if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x08) != 0)
				{
					led_monit_ch &= 0xF8;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
			}
			can_tp_txecheck(3, id);	//	TP送信完了確認
		}
		buf |= 0x08;
	}
	
	//	TXB2
	if(sts->BIT.TXB2R != 0)
	{
		if(tx_act[2] == 2) tx_act[2] = 3;
	}
	else
	{
		if(tx_act[2] == 3)
		{
			tx_act[2] = 0;
			id = ((((unsigned long)mcp_txb[2].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)mcp_txb[2].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
			if(led_monit_ch & 8)
			{
			//	if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x08) != 0)
				{
					led_monit_ch &= 0xF8;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
			}
			can_tp_txecheck(3, id);	//	TP送信完了確認
		}
	}
	if(sts->BIT.TX2IF != 0)
	{	//	終了している
		if(sts->BIT.TXB2R == 0 && tx_act[2] == 2)
		{
			tx_act[2] = 0;
			id = ((((unsigned long)mcp_txb[2].REG.TXB.SIDH.BYTE) << 3) & 0x7F8) | ((((unsigned long)mcp_txb[2].REG.TXB.SIDL.BYTE) >> 5) & 0x007);
			if(led_monit_ch & 8)
			{
			//	if(id == led_monit_id) PORTE.PODR.BIT.B0 = 1;
				if(id == led_monit_id && (led_monit_ch & 0x08) != 0)
				{
					led_monit_ch &= 0xF8;
					PORTE.PODR.BIT.B0 = 1;
					monit_timeover();
				}
			}
			can_tp_txecheck(3, id);	//	TP送信完了確認
		}
		buf |= 0x10;
	}

	//	フラグクリア
	if(buf != 0)
	{
		buf |= 0xE0;
		can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF, 1, 0, 0, &buf);					//	RX0IFクリア
	}
}

//---------------------------------------------------------------------------------------
//  割り込み後ステータス取得後のCallback処理
//---------------------------------------------------------------------------------------
void can3_sts_event(MCP2515REG_INTERR *rxd)
{
	MCP2515REG_BITX	bc, ec;

	bc.BYTE.MSK.BYTE = rxd->BYTE.CANINTF.BYTE & 0xFC;
	bc.BYTE.PAT.BYTE = 0;

#if	1
	if(rxd->BYTE.CANINTF.BYTE == 0) return;
	if(rxd->BYTE.CANINTF.BIT.RX0IF)
	{	//	受信バッファ0フル
		can3_request(MCP2515CMD_READ, MCP2515AD_RXB0CTRL, 0, 7, CAN3_CallbackRx0, 0);	//	RXB0読み出し
	}
	if(rxd->BYTE.CANINTF.BIT.RX1IF)
	{	//	受信バッファ1フル
		can3_request(MCP2515CMD_READ, MCP2515AD_RXB1CTRL, 0, 7, CAN3_CallbackRx1, 0);	//	RXB1読み出し
	}
#else
	if(bc.BYTE.MSK.BYTE == 0) return;
#endif
	//	送信完了チェック
	if(rxd->BYTE.CANINTF.BIT.TX0IF)
	{	//	送信バッファ0エンプティ
		tx_act[0] = 0;
	}
	if(rxd->BYTE.CANINTF.BIT.TX1IF)
	{	//	送信バッファ1エンプティ
		tx_act[1] = 0;
	}
	if(rxd->BYTE.CANINTF.BIT.TX2IF)
	{	//	送信バッファ2エンプティ
		tx_act[2] = 0;
	}
#if	1
	if(rxd->BYTE.CANINTF.BIT.ERRIF)
	{	//	エラー割り込み
	//	tx_act[0] = 0;
	//	tx_act[1] = 0;
	//	tx_act[2] = 0;
		logging("CAN3:Error %02X\r", (int)rxd->BYTE.EFLG.BYTE);
		ec.BYTE.MSK.BYTE = rxd->BYTE.EFLG.BYTE;
		ec.BYTE.PAT.BYTE = 0;
		can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF, 1, 0, 0, &ec);	//	割り込み要求フラグクリア
	}
#endif
#if	1
	if(rxd->BYTE.CANINTF.BIT.WAKIF)
	{	//	ウェイクアップ割り込み
		logging("CAN3:Wakeup\r");
	}
	if(rxd->BYTE.CANINTF.BIT.MERRF)
	{	//	メッセージエラー割り込み
		logging("CAN3:MsgErr\r");
	}
#endif
	can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF, 1, 0, 0, &bc);	//	割り込み要求フラグクリア
}

//---------------------------------------------------------------------------------------
//  機能   : 外部CANモジュール割り込み(MCP2515 - INT)
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_ICU_IRQ0} ICU_IRQ0_ISR(void)
{
	can3_request(MCP2515CMD_STATUS, 0, 0, 1, can3_stat_event, 0);	//	ステータス取得
//	can3_request(MCP2515CMD_READ, MCP2515AD_CANINTF, 0, 1, can3_sts_event, 0);		//	ステータス強制取得
}

//---------------------------------------------------------------------------------------
//  機能   : 外部CANモジュール割り込み(MCP2515 - RX0BF)
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_ICU_IRQ6} ICU_IRQ6_ISR(void)
{
	const unsigned short	buf = 0x0001;
	can3_request(MCP2515CMD_READ, MCP2515AD_RXB0CTRL, 0, 7, CAN3_CallbackRx0, 0);	//	RXB0読み出し
	can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF, 1, 0, 0, &buf);				//	RX0IFクリア
}

//---------------------------------------------------------------------------------------
//  機能   : 外部CANモジュール割り込み(MCP2515 - RX1BF)
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_ICU_IRQ7} ICU_IRQ7_ISR(void)
{
	const unsigned short	buf = 0x0002;
	can3_request(MCP2515CMD_READ, MCP2515AD_RXB1CTRL, 0, 7, CAN3_CallbackRx1, 0);	//	RXB1読み出し
	can3_request(MCP2515CMD_BITX, MCP2515AD_CANINTF, 1, 0, 0, &buf);				//	RX0IFクリア
}

#endif


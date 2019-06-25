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
//	RX63N	SCI-I/F 通信
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

//#include	"sci.h"

#ifndef __CAN2ECU_SCI_IF__
#define __CAN2ECU_SCI_IF__

#include	"ecu.h"			/*	ECU 共通定義			*/

/*
	ポート設定

			Port		SCI			I2C			SPI			適用
	----------------------------------------------------------------------------
	SCI0	P20			TXD0		SDA0		SMOSI0		<RS-232C>	COM0
			P21			RXD0		SCL0		SMISO0		<RS-232C>	COM0
			P86			nRTS0								<RS-232C>	COM0
			P87			nCTS0								<RS-232C>	COM0
			P70			TX0SDN								<RS-232C>	送信許可
															
	SCI1	P26			TXD1								<REM-MON>	COM1
			P30			RXD1								<REM-MON>	COM1
			PE1			nRTS1/(TXD12)						<RS-232C>	COM1/(COM12)
			PE2			nCTS1/(RXD12)						<RS-232C>	COM1/(COM12)
															
	SCI2	P13			TXD2		SDA0					<RS-232C>	COM2
			P12			RXD2		SCL0					<RS-232C>	COM2
			P15			nRTS2								<RS-232C>	COM2
			P17			nCTS2								<RS-232C>	COM2
			P73			TX2SDN								<RS-232C>	送信許可
															
	SCI3	P23			TXD3								<RS-232C>	COM3
			P25			RXD3								<RS-232C>	COM3
			P22			nRTS3								<RS-232C>	COM3
			P24			nCTS3								<RS-232C>	COM3
			P56			nEXRES								</RESET>	外部モジュールリセット信号
															
	SCI5	PC3			TXD5		SSDA5		SMOSI5		<SPI/I2C>	外部拡張
			PC2			RXD5		SSCL5		SMISO5		<SPI/I2C>	外部拡張
			PC4									SCK5		<SPI>		外部拡張
			PC5									SS0			<SPI>		外部拡張
			PC6									SS1			<SPI>		外部拡張
															
	SCI6	P00			TXD6		SSDA6		SMISO6		<TTL>		COM6
			P01			RXD6		SSCL6		SMOSI6		<TTL>		COM6
			P02			nRTS6					SCK6		<TTL>		COM6
			PJ3			nCTS6					SS6			<TTL>		COM6
*/

//	使用するSCIポートの選択

#define		SCI0_ACTIVATE
//#define		SCI1_ACTIVATE
#define		SCI2_ACTIVATE
//#define		SCI3_ACTIVATE
//#define		SCI4_ACTIVATE
//#define		SCI5_ACTIVATE
//#define		SCI6_ACTIVATE


//	SCIバッファサイズ
#define		BUFSIZE		1024
//	SCI0をRS-485半二重として使用
//#define		SCI0_RS485
//	SCI1のnRTS,nCTSを使用
//#define		SCI1_FLOW
//  SCI3のnCTSをP24ポートに割り当て
//#define     SCI3_nCTS
//	SCI1のnRTS/nCTSをSCI6ポートとして使用
//#define		SCI6_ACTIVE
//	SCI管理構造体
typedef struct	__sci_module__ {
	unsigned char	txbuf[BUFSIZE];		//	送信バッファ
	unsigned char	rxbuf[BUFSIZE];		//	受信バッファ
	int				txwp;				//	送信書き込みポインタ
	int				txrp;				//	送信読み出しポインタ
	int				rxwp;				//	受信書き込みポインタ
	int				rxrp;				//	受信読み出しポインタ
	int				err;				//	総エラーカウンタ
	int				perr;				//	パリティーエラーカウンタ
	int				ferr;				//	フレーミングエラーカウンタ
	int				oerr;				//	オーバーランエラーカウンタ
}	SCI_MODULE;

//間接呼び出しのプロトタイプ(引数１個)
typedef	void 			(*PROC_CALL)(void *);

//	SCI0,SCI1,SCI2,SCI3,SCI5,SCI6	SCI1=イエロースコープで使用
//extern	CONSOLE_CTRL		sci_com;
//extern	CONSOLE_CTRL		usb_com;

//________________________________________________________________________________________
//
//	sci0_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI0初期化
//							Port		SCI			I2C			SPI			
//				--------------------------------------------------------
//				SCI0	P20			TXD0(*)		SDA0		SMOSI0
//							P21			RXD0(*)		SCL0		SMISO0
//							P22			DE/nRE(*)<RS-485>
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci0_init(long bps, int datalen, int stoplen, int parity);
#define		SCI0_RTS_PORT			PORT8.PODR.BIT.B6	/*	out 0=Enable / 1=Disable	*/
#define		SCI0_CTS_PORT			PORT8.PIDR.BIT.B7	/*	in	0=Enable / 1=Disable	*/
#define		SCI0_TXOSDN_PORT		PORT7.PODR.BIT.B0	/*	0=RX / 1=TX					*/

//________________________________________________________________________________________
//
//	sci1_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI1初期化
//							Port		SCI			I2C			SPI			
//				--------------------------------------------------------
//				SCI1	P26			TXD1(*) <REM-MON>	
//							P30			RXD1(*) <REM-MON>	
//							P00			nRTS1(*)<REM-MON>	
//							P01			nCTS1(*)<REM-MON>	
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//	備考
//		SCI1はイエロースコープで使用
//________________________________________________________________________________________
//
extern	void sci1_init(long bps, int datalen, int stoplen, int parity);
#define		SCI1_RTS_PORT			PORTE.PODR.BIT.B1	/*	out 0=Enable / 1=Disable	*/
#define		SCI1_CTS_PORT			PORTE.PIDR.BIT.B2	/*	in	0=Enable / 1=Disable	*/

//________________________________________________________________________________________
//
//	sci2_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI2初期化
//							Port		SCI			RIIC		SPI			
//				--------------------------------------------------------
//				SCI2	P13			TXD2		SDA0
//							P12			RXD2		SCL0
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci2_init(long bps, int datalen, int stoplen, int parity);
#define		SCI2_RTS_PORT			PORT1.PODR.BIT.B5	/*	out 0=Enable / 1=Disable	*/
#define		SCI2_CTS_PORT			PORT1.PIDR.BIT.B7	/*	in	0=Enable / 1=Disable	*/
#define		SCI2_TXOSDN_PORT		PORT7.PODR.BIT.B3	/*	0=RX / 1=TX					*/

//________________________________________________________________________________________
//
//	sci3_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI3初期化
//							Port		SCI			I2C			SPI			
//				--------------------------------------------------------
//				SCI3	P23			TXD3(*)		
//							P24			RXD3(*)		
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci3_init(long bps, int datalen, int stoplen, int parity);
#define		SCI3_RTS_PORT			PORT2.PODR.BIT.B2	/*	out 0=Enable / 1=Disable	*/
#define		SCI3_CTS_PORT			PORT2.PIDR.BIT.B4	/*	in	0=Enable / 1=Disable	*/
#define		SCI3_EXRES_PORT			PORT5.PODR.BIT.B6	/*	外部リセット出力 0=RESET	*/

//________________________________________________________________________________________
//
//	sci5_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI5初期化
//							Port		SCI			I2C			SPI			
//				--------------------------------------------------------
//				SCI5	PC3			TXD5(*)		
//							PC2			RXD5(*)		
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci5_init(long bps, int datalen, int stoplen, int parity);
#define		SCI5_SS0_PORT			PORTC.PODR.BIT.B5	/*	外部選択線出力 0=Select		*/
#define		SCI5_SS1_PORT			PORTC.PODR.BIT.B6	/*	外部選択線出力 0=Select		*/

//________________________________________________________________________________________
//
//	sci6_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI6初期化
//						Port		SCI			I2C			SPI			
//				--------------------------------------------------------
//				SCI6	P00			TXD6        						<TTL>		COM6
//						P01			RXD6        						<TTL>		COM6
//						P02			nRTS6								<TTL>		COM6
//						PJ3			nCTS6								<TTL>		COM6
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci6_init(long bps, int datalen, int stoplen, int parity);
#define		SCI6_RTS_PORT			PORT0.PODR.BIT.B2	/*	out 0=Enable / 1=Disable	*/
#define		SCI6_CTS_PORT			PORTJ.PIDR.BIT.B3	/*	in	0=Enable / 1=Disable	*/

//________________________________________________________________________________________
//
//	sci_putcheck
//----------------------------------------------------------------------------------------
//	機能説明
//		SCIの送信バッファの空き容量を求める
//	引数
//		ch			SCIチャンネル
//	戻り
//		int			空きバイト数
//________________________________________________________________________________________
//
extern	int sci_putcheck(int ch);

//________________________________________________________________________________________
//
//	sci_txbytes
//----------------------------------------------------------------------------------------
//	機能説明
//		SCIの送信バッファの未送信バイト数を求める
//	引数
//		ch			SCIチャンネル
//	戻り
//		int			未送信バイト数
//________________________________________________________________________________________
//
extern	int sci_txbytes(int ch);

//________________________________________________________________________________________
//
//	sci(n)_putb
//----------------------------------------------------------------------------------------
//	機能説明
//		SCIへデータ列を送信する
//	引数
//		*buf		送信バッファ
//		size		送信バイト数
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void	sci0_putb(unsigned char *buf, int size);
extern	void	sci1_putb(unsigned char *buf, int size);
extern	void	sci2_putb(unsigned char *buf, int size);
extern	void	sci3_putb(unsigned char *buf, int size);
extern	void	sci5_putb(unsigned char *buf, int size);
extern	void	sci6_putb(unsigned char *buf, int size);

//________________________________________________________________________________________
//
//	sci_puts
//----------------------------------------------------------------------------------------
//	機能説明
//		SCIへ文字列を送信する
//	引数
//		ch			SCIチャンネル
//		*str		送信文字列
//	戻り
//		int			空きバイト数
//________________________________________________________________________________________
//
extern	void sci_puts(int ch, char *str);

//________________________________________________________________________________________
//
//	sci_putb
//----------------------------------------------------------------------------------------
//	機能説明
//		SCIへバイト列を送信する
//	引数
//		ch			SCIチャンネル
//		*buf		送信バイト列
//		len			バイト長
//	戻り
//		int			空きバイト数
//________________________________________________________________________________________
//
extern	void sci_putb(int ch, unsigned char *buf, int len);

//________________________________________________________________________________________
//
//	sci_putc
//----------------------------------------------------------------------------------------
//	機能説明
//		SCIへ1文字送信する
//	引数
//		ch			SCIチャンネル
//		data		送信文字
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci_putc(int ch, char data);

//________________________________________________________________________________________
//
//	sci_txint
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI送信割り込み処理
//	引数
//		ch			SCIチャンネル
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void sci_txint(int ch);

//________________________________________________________________________________________
//
//	sci(n)_txi
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI0,1,2,3,5,6 送信バッファが空になると発生する割り込み関数
//	引数
//		無し
//	戻り
//		無し
//	備考
//		割り込みベクタ(VECT_SCI0_TXI0～VECT_SCI6_TXI6) 215,218,221,224,230,233
//________________________________________________________________________________________
//
/*
interrupt void sci0_txi(void);
interrupt void sci1_txi(void);
interrupt void sci2_txi(void);
interrupt void sci3_txi(void);
interrupt void sci5_txi(void);
interrupt void sci6_txi(void);
*/
//________________________________________________________________________________________
//
//	sci(n)_tei
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI0,1,2,3,5,6 シフトレジスタが送信完了すると発生する割り込み関数
//	引数
//		無し
//	戻り
//		無し
//	備考
//		割り込みベクタ(VECT_SCI0_TEI0～VECT_SCI6_TEI6) 216,219,222,225,231,234
//________________________________________________________________________________________
//
/*
interrupt void sci0_tei(void);
interrupt void sci1_tei(void);
interrupt void sci2_tei(void);
interrupt void sci3_tei(void);
interrupt void sci5_tei(void);
interrupt void sci6_tei(void);
*/
//________________________________________________________________________________________
//
//	sci_load
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI0,1,2,3,5,6 受信データをバッファに積み上げる
//	引数
//		ch			SCIチャンネル
//		err			エラーコード
//		data		データ＋エラーフラグ
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void	sci_load(int ch, unsigned char err, unsigned char data);

//________________________________________________________________________________________
//
//	sci_err
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI エラー発生割り込み(GROUP12) エラー時は受信データを取得する
//	引数
//		無し
//	戻り
//		無し
//	備考
//		割り込みベクタ(VECT_ICU_GROUP12) 114
//________________________________________________________________________________________
//
//interrupt void sci_err(void);

//________________________________________________________________________________________
//
//	sci(n)_rxi
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 受信割り込み処理
//	引数
//		無し
//	戻り
//		無し
//	備考
//		割り込みベクタ(VECT_SCI0_RXI0～VECT_SCI6_RXI6) 214,217,220,223,229,232
//________________________________________________________________________________________
//
/*
interrupt void sci0_rxi(void);
interrupt void sci1_rxi(void);
interrupt void sci2_rxi(void);
interrupt void sci3_rxi(void);
interrupt void sci5_rxi(void);
interrupt void sci6_rxi(void);
*/
//________________________________________________________________________________________
//
//	sci_get_check
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 受信バイト数取得
//	引数
//		ch			SCIチャンネル番号
//	戻り
//		int			受信バッファ未処理バイト数
//________________________________________________________________________________________
//
extern	int sci_get_check(int ch);

//________________________________________________________________________________________
//
//	sci_get_char
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 受信バイト数取得
//	引数
//		ch			SCIチャンネル番号
//	戻り
//		int			データ	0x00～0xFF:有り -1:無し
//________________________________________________________________________________________
//
extern	int sci_get_char(int ch);

//________________________________________________________________________________________
//
//	sci_get_buf
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 受信データ取得
//	引数
//		ch			SCIチャンネル番号
//		*buf		コピー先バッファ
//		size		転送バイト数
//	戻り
//		int			実転送バイト数
//________________________________________________________________________________________
//
extern	int sci_get_buf(int ch, unsigned char *buf, int size);

//________________________________________________________________________________________
//
//	sci_get_string
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 受信文字列取得
//	引数
//		ch			SCIチャンネル番号
//		*str		コピー先バッファ
//		size		転送文字数上限
//	戻り
//		int			実転送文字数
//________________________________________________________________________________________
//
extern	int sci_get_string(int ch, char *str, int size);

//________________________________________________________________________________________
//
//	sci_get_line
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 受信文字列１行取得
//	引数
//		ch			SCIチャンネル番号
//		*str		コピー先バッファ
//		size		転送文字数上限
//		echar		行末キャラクタ
//	戻り
//		int			実転送文字数 / 負の数はバッファ不足量又は行末無し
//________________________________________________________________________________________
//
extern	int sci_get_line(int ch, char *str, int size, char echar);

//________________________________________________________________________________________
//
//	sci_clear
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI 制御変数をクリアする
//	引数
//		ch			SCIチャンネル番号
//	戻り
//		無し
//________________________________________________________________________________________
//
extern	void	sci_clear(int ch);

#endif	/*__CAN2ECU_SCI_IF__*/


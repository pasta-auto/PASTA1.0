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
#include "sci.h"

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

/*
//	SCIバッファサイズ
#define		BUFSIZE		1024
//	SCI0をRS-485半二重として使用
#define		SCI0_RS485
//	SCI1のnRTS,nCTSを使用
#define		SCI1_FLOW
//	SCI3のnCTSをP24ポートに割り当て
#define		SCI3_nCTS
//	SCI1のnRTS/nCTSをSCI6ポートとして使用
#define		SCI6_ACTIVE
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
*/
//	SCI0,SCI1,SCI2,SCI3,SCI5,SCI6	SCI1=イエロースコープで使用
SCI_MODULE		sci_com[7];

//	ログ機能
void	logging(char *fmt, ...);

//________________________________________________________________________________________
//
//	sci0_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI0初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			SCI0	P20			TXD0		SDA0		SMOSI0		<RS-232C>	COM0
//					P21			RXD0		SCL0		SMISO0		<RS-232C>	COM0
//					P86			nRTS0								<RS-232C>	COM0
//					P87			nCTS0								<RS-232C>	COM0
//					P70			TX0SDN								<RS-232C>	送信許可
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
void sci0_init(long bps, int datalen, int stoplen, int parity)
{
	SCI_MODULE	*com = &sci_com[0];
	memset(com, 0, sizeof(SCI_MODULE));

#ifdef		SCI0_ACTIVATE

	SYSTEM.PRCR.WORD = 0xA502;		//	プロテクト解除
	MSTP_SCI0 = 0;					//	SCIモジュールストップ解除

	//	SCI 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 割り込み禁止
	ICU.IER[IER_SCI0_RXI0].BIT.IEN_SCI0_RXI0 = 0;		//	受信割り込み禁止
	ICU.IER[IER_SCI0_TXI0].BIT.IEN_SCI0_TXI0 = 0;		//	送信完了割り込み禁止
	ICU.IER[IER_SCI0_TEI0].BIT.IEN_SCI0_TEI0 = 0;		//	送信エンプティ割り込み禁止

	//	Enable write protection(プロテクト掛ける)
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	//	SCR - Serial Control Register
	//		b7		TIE	- Transmit Interrupt Enable		- A TXI interrupt request is disabled
	//		b6		RIE	- Receive Interrupt Enable		- RXI and ERI interrupt requests are disabled
	//		b5		TE	- Transmit Enable				- Serial transmission is disabled
	//		b4		RE	- Receive Enable				- Serial reception is disabled
	//		b2		TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled
	SCI0.SCR.BYTE = 0x00;

	while (0x00 != (SCI0.SCR.BYTE & 0xF0))
	{
		//	Confirm that bit is actually 0
	}

	//	Set the I/O port functions

	//	汎用ポートP20:TXD0、P21:RXD0、(P22:DE/RE)
	PORT2.PODR.BIT.B0 = 1;		//	TXD
	PORT8.PODR.BIT.B6 = 1;		//	RTS
	//	Set port direction - TXDn is output port, RXDn is input port(ポート入出力設定)
	PORT2.PDR.BIT.B0 = 1;		//	出力	TXD
	PORT2.PDR.BIT.B1 = 0;		//	入力	RXD
	PORT8.PDR.BIT.B6 = 1;		//	出力	RTS
	PORT8.PDR.BIT.B7 = 0;		//	入力	CTS
	
	//	Set port mode - Use pin as general I/O port
	PORT2.PMR.BIT.B0 = 0;		//	汎用IOポート設定
	PORT2.PMR.BIT.B1 = 0;		//	汎用IOポート設定
	PORT8.PMR.BIT.B6 = 0;		//	汎用IOポート設定
	PORT8.PMR.BIT.B7 = 0;		//	汎用IOポート設定

	PORT7.PDR.BIT.B0 = 1;		//	出力
	PORT7.PODR.BIT.B0 = 1;		//	0=SDN / 1=通常
	PORT7.PMR.BIT.B0 = 0;		//	汎用IOポート設定

	//	PWPR - Write-Protect Register(書き込みプロテクトレジスタ)
	//		b7		B0WI		- PFSWE Bit Write Disable	- PFSWE禁止
	//		b6		PFSWE	- PFS Register Write Enable - PFS許可
	//		b5:b0	Reserved - These bits are read as 0. The write value should be 0.
	MPC.PWPR.BIT.B0WI = 0;			//	先に0にする
	MPC.PWPR.BIT.PFSWE = 1;			//	後に1にする

	//	PFS - Pin Function Control Register(ピンファンクションレジスタ設定)
	//		b3:b0	PSEL - Pin Function Select - RXDn, TXDn
	MPC.P20PFS.BYTE = 0x0A;			//	assign I/O pin to SCI0 TxD0
	MPC.P21PFS.BYTE = 0x0A;			//	assign I/O pin to SCI0 RxD0
	MPC.P86PFS.BYTE = 0x00;			//	assign I/O pin to Port
	MPC.P87PFS.BYTE = 0x00;			//	assign I/O pin to Port
	MPC.P07PFS.BYTE = 0x00;			//	assign I/O pin to Port
	//	書き込みプロテクトをかける
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;

	//	Use pin as I/O port for peripheral functions(IOピン機能設定)
	PORT2.PMR.BIT.B0 = 1;			//	周辺機能設定
	PORT2.PMR.BIT.B1 = 1;			//	周辺機能設定

	//	Initialization of SCI
	//	全モジュールクロックストップモード禁止
	SYSTEM.MSTPCRA.BIT.ACSE = 0;
	//	SCI0モジュールストップ状態の解除
	MSTP_SCI0 = 0;


	//	Select an On-chip baud rate generator to the clock source
	SCI0.SCR.BIT.CKE = 0;

	//	SMR - Serial Mode Register
	//	b7		CM	- Communications Mode	- Asynchronous mode
	//	b6		CHR	- Character Length		- Selects 8 bits as the data length
	//	b5		PE	- Parity Enable			- When transmitting : Parity bit addition is not performed
	//								When receiving	: Parity bit checking is not performed
	//	b3		STOP - Stop Bit Length		- 2 stop bits
	//	b2		MP	- Multi-Processor Mode	- Multi-processor communications function is disabled
	//	b1:b0	CKS	- Clock Select			- PCLK clock (n = 0)
	SCI0.SMR.BYTE = 0x08;

	//	SCMR - Smart Card Mode Register
	//	b6:b4	Reserved - The write value should be 1.
	//	b3		SDIR		- Transmitted/Received Data Transfer Direction - Transfer with LSB-first
	//	b2		SINV		- Transmitted/Received Data Invert	- TDR contents are transmitted as they are. 
	//												Receive data is stored as it is in RDR.
	//	b1		Reserved - The write value should be 1.
	//	b0		SMIF		- Smart Card Interface Mode Select	- Serial communications interface mode
	SCI0.SCMR.BYTE = 0xF2;

	//	SEMR - Serial Extended Mode Register
	//	b7:b6	Reserved - The write value should be 0.
	//	b5		NFEN		- Digital Noise Filter Function Enable	- Noise cancellation function 
	//													for the RXDn input signal is disabled.
	//	b4		ABCS		- Asynchronous Mode Base Clock Select	- Selects 16 base clock cycles for 1-bit period
	//	b3:b1	Reserved - The write value should be 0.
	SCI0.SEMR.BYTE = 0x00;

	//	Set data transfer format in Serial Mode Register (SMR)*/ 
	//	-Asynchronous Mode`
	//	-8 bits
	//	-no parity
	//	-1 stop bit
	//	-PCLK clock (n = 0)
	SCI0.SMR.BYTE = 0x00;		//	0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64

	//	BRR - Bit Rate Register
	//	Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04
	if(stoplen == 1)
	{
		SCI0.SMR.BIT.STOP = 1;
	}
	else
	{
		SCI0.SMR.BIT.STOP = 0;
	}

	if(parity == 0)
	{
		SCI0.SMR.BIT.PE = 0;
	}
	else
	if(parity == 1)
	{		//　奇数パリティ
		SCI0.SMR.BIT.PE = 1;
		SCI0.SMR.BIT.PM = 1;
	}
	else
	if(parity == 2)
	{	//	偶数パリティ
		SCI0.SMR.BIT.PE = 1;
		SCI0.SMR.BIT.PM = 0;
	}

	if(datalen == 7)
	{		//	7bit長
		SCI0.SMR.BIT.CHR = 1;
	}
	else
	if(datalen == 8)
	{	//	8bit長
		SCI0.SMR.BIT.CHR = 0;
	}

	//	Set baud rate to 115200
	//	N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
	//	N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
	//	N = 12
	SCI0.BRR = 48000000 / ((64/2) * bps) - 1;

	//	SCI割り込み優先順位設定
	ICU.IPR[IPR_SCI0_].BIT.IPR = 1;						//	割り込みレベル設定

	//	SCI0 割り込み設定
	ICU.IER[IER_SCI0_RXI0].BIT.IEN_SCI0_RXI0 = 1;		//	受信割り込み
	ICU.IER[IER_SCI0_TXI0].BIT.IEN_SCI0_TXI0 = 1;		//	送信完了割り込み
	ICU.IER[IER_SCI0_TEI0].BIT.IEN_SCI0_TEI0 = 1;		//	送信エンプティ割り込み

	//	割り込みフラグクリア
	ICU.IR[IR_SCI0_RXI0].BIT.IR = 0;
	ICU.IR[IR_SCI0_TXI0].BIT.IR = 0;
	ICU.IR[IR_SCI0_TEI0].BIT.IR = 0;

	//	GROUP12割り込み設定
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.GEN[GEN_SCI0_ERI0].BIT.EN_SCI0_ERI0 = 1;		//	グループ12 SCI0受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア

	SCI0.SCR.BIT.RIE = 1;
	SCI0.SCR.BIT.RE = 1;

#endif		/*SCI0_ACTIVATE*/
}

//________________________________________________________________________________________
//
//	sci1_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI1初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			SCI1	P26			TXD1								<REM-MON>	COM1
//					P30			RXD1								<REM-MON>	COM1
//					PE1			TXD12/nRTS1							<RS-232C>	COM1/12
//					PE2			RXD12/nCTS1							<RS-232C>	COM1/12
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
#ifdef		SCI1_ACTIVATE
#ifdef	__YIDE_REM_DEBUG__
//	217
void interrupt sci1_rxi(void);
//	218
void interrupt sci1_txi(void);
//	219
void interrupt sci1_tei(void);
#endif
#endif		/*SCI1_ACTIVATE*/
void sci1_init(long bps, int datalen, int stoplen, int parity)
{
#ifdef		SCI1_ACTIVATE
#ifdef	__YIDE_REM_DEBUG__
	static unsigned long	*x_intb;
#endif
	unsigned short	i;
#endif		/*SCI1_ACTIVATE*/
	SCI_MODULE		*com = &sci_com[1];

	memset(com, 0, sizeof(SCI_MODULE));

#ifdef		SCI1_ACTIVATE

	SYSTEM.PRCR.WORD = 0xA502;		//	プロテクト解除
	MSTP_SCI1 = 0;					//	SCIモジュールストップ解除

	//	SCI 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループL0 割り込み禁止
	ICU.IER[IER_SCI1_RXI1].BIT.IEN_SCI1_RXI1 = 0;		//	受信割り込み禁止
	ICU.IER[IER_SCI1_TXI1].BIT.IEN_SCI1_TXI1 = 0;		//	送信完了割り込み禁止
	ICU.IER[IER_SCI1_TEI1].BIT.IEN_SCI1_TEI1 = 0;		//	送信エンプティ割り込み禁止

#ifdef	__YIDE_REM_DEBUG__
	//	割り込みベクタの内容を書き換え、リモートモニタを強制解除する
	_asm	extern	_x_intb
	_asm	MVFC	INTB, r0
	_asm	MOV.L	#_x_intb, r1
	_asm	MOV.L	r0, [r1]
	x_intb[VECT_SCI1_RXI1] = (unsigned long)sci1_rxi;
	x_intb[VECT_SCI1_TXI1] = (unsigned long)sci1_txi;
	x_intb[VECT_SCI1_TEI1] = (unsigned long)sci1_tei;
	
#endif

	//	Enable write protection(プロテクト掛ける)
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	//	SCR - Serial Control Register
	//	b7		TIE	- Transmit Interrupt Enable		- A TXI interrupt request is disabled
	//	b6		RIE	- Receive Interrupt Enable		- RXI and ERI interrupt requests are disabled
	//	b5		TE	- Transmit Enable				- Serial transmission is disabled
	//	b4		RE	- Receive Enable				- Serial reception is disabled
	//	b2		TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled
	SCI1.SCR.BYTE = 0x00;

	while (0x00 != (SCI1.SCR.BYTE & 0xF0))
	{
		//	Confirm that bit is actually 0
	}

	//	Set the I/O port functions

	//	汎用ポートP26:TXD1、P30:RXD1、(PE1:RTS1、PE2:CTS1)
	PORT2.PODR.BIT.B6 = 1;		//	TXD
	PORTE.PODR.BIT.B1 = 1;		//	RTS=Disable
	//	Set port direction - TXDn is output port, RXDn is input port(ポート入出力設定)
	PORT2.PDR.BIT.B6 = 1;		//	出力	TXD
	PORT3.PDR.BIT.B0 = 0;		//	入力	RXD
	PORTE.PDR.BIT.B1 = 1;		//	出力	RTS
	PORTE.PDR.BIT.B2 = 0;		//	入力	CTS

	//	Set port mode - Use pin as general I/O port
	PORT2.PMR.BIT.B6 = 0;		//	汎用IOポート設定
	PORT3.PMR.BIT.B0 = 0;		//	汎用IOポート設定
	PORTE.PMR.BIT.B1 = 0;		//	汎用IOポート設定
	PORTE.PMR.BIT.B2 = 0;		//	汎用IOポート設定

	//	PWPR - Write-Protect Register(書き込みプロテクトレジスタ)
	//	b7		B0WI		- PFSWE Bit Write Disable	- PFSWE禁止
	//	b6		PFSWE	- PFS Register Write Enable - PFS許可
	//	b5:b0	Reserved - These bits are read as 0. The write value should be 0.
	MPC.PWPR.BIT.B0WI = 0;			//	先に0にする
	MPC.PWPR.BIT.PFSWE = 1;			//	後に1にする
	
	MPC.P26PFS.BYTE = 0x0A;		//	assign I/O pin to SCI1 TxD1
	MPC.P30PFS.BYTE = 0x0A;		//	assign I/O pin to SCI1 RxD1
	MPC.PE1PFS.BYTE = 0x00;		//	assign I/O pin to Port
	MPC.PE2PFS.BYTE = 0x00;		//	assign I/O pin to Port

	//	書き込みプロテクトをかける
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;

	PORT2.PMR.BIT.B6 = 1;		//	周辺機能として使用
	PORT3.PMR.BIT.B0 = 1;		//	周辺機能として使用

	//	全モジュールクロックストップモード禁止
	SYSTEM.MSTPCRA.BIT.ACSE = 0;
	//	SCI1モジュールストップ状態の解除
	MSTP_SCI1 = 0;
	
	SCI1.SCR.BYTE = 0x00;		//	Disable Tx/Rx and set clock to internal
	//	Set data transfer format in Serial Mode Register (SMR)
	//		-Asynchronous Mode`
	//		-8 bits
	//		-no parity
	//		-1 stop bit
	//		-PCLK clock (n = 0)
	SCI1.SMR.BYTE = 0x00;		//	0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64

	if(stoplen == 1)
	{
		SCI1.SMR.BIT.STOP = 1;
	}
	else
	{
		SCI1.SMR.BIT.STOP = 0;
	}

	if(parity == 0)
	{
		SCI1.SMR.BIT.PE = 0;
	}
	else
	if(parity == 1)
	{		//　奇数パリティ
		SCI1.SMR.BIT.PE = 1;
		SCI1.SMR.BIT.PM = 1;
	}
	else
	if(parity == 2)
	{	//	偶数パリティ
		SCI1.SMR.BIT.PE = 1;
		SCI1.SMR.BIT.PM = 0;
	}

	if(datalen == 7)
	{		//	7bit長
		SCI1.SMR.BIT.CHR = 1;
	}
	else
	if(datalen == 8)
	{	//	8bit長
		SCI1.SMR.BIT.CHR = 0;
	}

	//	Set baud rate to 115200
	//		N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
	//		N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
	//		N = 12

	SCI1.BRR = 48000000 / ((64/2) * bps) - 1;
	//			↑クロック分周設定によって変更する

	//	Wait at least one bit interval
	for ( i = 0; i < 5000; i++ );	//	assume minimum of 2 instructions at 98MHz ?

	//	SCI割り込み優先順位設定
	ICU.IPR[IPR_SCI1_].BIT.IPR = 1;		//	割り込みレベル設定

	//	SCI1 割り込み設定
	ICU.IER[IER_SCI1_RXI1].BIT.IEN_SCI1_RXI1 = 1;		//	受信割り込み
	ICU.IER[IER_SCI1_TXI1].BIT.IEN_SCI1_TXI1 = 1;		//	送信完了割り込み
	ICU.IER[IER_SCI1_TEI1].BIT.IEN_SCI1_TEI1 = 1;		//	送信エンプティ割り込み

	//	割り込みフラグクリア
	ICU.IR[IR_SCI1_RXI1].BIT.IR = 0;
	ICU.IR[IR_SCI1_TXI1].BIT.IR = 0;
	ICU.IR[IR_SCI1_TEI1].BIT.IR = 0;

	//	GROUP12割り込み設定
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.GEN[GEN_SCI1_ERI1].BIT.EN_SCI1_ERI1 = 1;		//	グループ12 SCI1受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア

	SCI1.SCR.BIT.RIE = 1;
	SCI1.SCR.BIT.RE = 1;

#ifdef	SCI1_FLOW
	PORTE.PODR.BIT.B1 = 0;		//	RTS=Enable
#endif

#endif		/*SCI1_ACTIVATE*/

}

//________________________________________________________________________________________
//
//	sci2_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI2初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			SCI2	P13			TXD2		SDA0					<RS-232C>	COM2
//					P12			RXD2		SCL0					<RS-232C>	COM2
//					P15			nRTS2								<RS-232C>	COM2
//					P17			nCTS2								<RS-232C>	COM2
//					P73			TX2SDN								<RS-232C>	送信許可
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
void sci2_init(long bps, int datalen, int stoplen, int parity)
{
	SCI_MODULE	*com = &sci_com[2];
	memset(com, 0, sizeof(SCI_MODULE));

#ifdef		SCI2_ACTIVATE

	SYSTEM.PRCR.WORD = 0xA502;		//	プロテクト解除
	MSTP_SCI2 = 0;					//	SCIモジュールストップ解除

	//	SCI 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 割り込み禁止
	ICU.IER[IER_SCI2_RXI2].BIT.IEN_SCI2_RXI2 = 0;		//	受信割り込み禁止
	ICU.IER[IER_SCI2_TXI2].BIT.IEN_SCI2_TXI2 = 0;		//	送信完了割り込み禁止
	ICU.IER[IER_SCI2_TEI2].BIT.IEN_SCI2_TEI2 = 0;		//	送信エンプティ割り込み禁止

	//	Enable write protection(プロテクト掛ける)
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	//	SCR - Serial Control Register
	//		b7		TIE	- Transmit Interrupt Enable		- A TXI interrupt request is disabled
	//		b6		RIE	- Receive Interrupt Enable		- RXI and ERI interrupt requests are disabled
	//		b5		TE	- Transmit Enable				- Serial transmission is disabled
	//		b4		RE	- Receive Enable				- Serial reception is disabled
	//		b2		TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled
	SCI2.SCR.BYTE = 0x00;

	while (0x00 != (SCI2.SCR.BYTE & 0xF0))
	{
		//	Confirm that bit is actually 0
	}

	//	Set the I/O port functions

	//	汎用ポートP13:TXD2、P12:RXD2
	PORT1.PODR.BIT.B3 = 1;	//	TXD
	PORT1.PODR.BIT.B5 = 1;	//	RTS=Disable
	//	Set port direction - TXDn is output port, RXDn is input port(ポート入出力設定)
	PORT1.PDR.BIT.B3 = 1;		//	出力	TXD
	PORT1.PDR.BIT.B2 = 0;		//	入力	RXD
	PORT1.PDR.BIT.B5 = 1;		//	出力	RTS
	PORT1.PDR.BIT.B7 = 0;		//	入力	CTS

	//	Set port mode - Use pin as general I/O port
	PORT1.PMR.BIT.B3 = 0;		//	汎用IOポート設定
	PORT1.PMR.BIT.B2 = 0;		//	汎用IOポート設定
	PORT1.PMR.BIT.B5 = 0;		//	汎用IOポート設定
	PORT1.PMR.BIT.B7 = 0;		//	汎用IOポート設定

	//	PWPR - Write-Protect Register(書き込みプロテクトレジスタ)
	//		b7		B0WI		- PFSWE Bit Write Disable	- PFSWE禁止
	//		b6		PFSWE	- PFS Register Write Enable - PFS許可
	//		b5:b0	Reserved - These bits are read as 0. The write value should be 0.
	MPC.PWPR.BIT.B0WI = 0;			//	先に0にする
	MPC.PWPR.BIT.PFSWE = 1;			//	後に1にする

	//	PFS - Pin Function Control Register(ピンファンクションレジスタ設定)
	//		b3:b0	PSEL - Pin Function Select - RXDn, TXDn
	MPC.P13PFS.BYTE = 0x0A;		//	assign I/O pin to SCI0 TxD3
	MPC.P12PFS.BYTE = 0x0A;		//	assign I/O pin to SCI0 RxD3
	MPC.P15PFS.BYTE = 0x00;		//	assign I/O pin to Port
	MPC.P17PFS.BYTE = 0x00;		//	assign I/O pin to Port
	//	書き込みプロテクトをかける
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;

	//	Use pin as I/O port for peripheral functions(IOピン機能設定)
	PORT1.PMR.BIT.B3 = 1;			//	周辺機能設定
	PORT1.PMR.BIT.B2 = 1;			//	周辺機能設定

	//	Initialization of SCI
	//	全モジュールクロックストップモード禁止
	SYSTEM.MSTPCRA.BIT.ACSE = 0;
	//	SCI2モジュールストップ状態の解除
	MSTP_SCI2 = 0;


	//	Select an On-chip baud rate generator to the clock source
	SCI2.SCR.BIT.CKE = 0;

	//	SMR - Serial Mode Register
	//		b7		CM	- Communications Mode	- Asynchronous mode
	//		b6		CHR	- Character Length		- Selects 8 bits as the data length
	//		b5		PE	- Parity Enable			- When transmitting : Parity bit addition is not performed
	//									When receiving	: Parity bit checking is not performed
	//		b3		STOP - Stop Bit Length		- 2 stop bits
	//		b2		MP	- Multi-Processor Mode	- Multi-processor communications function is disabled
	//		b1:b0	CKS	- Clock Select			- PCLK clock (n = 0)
	SCI2.SMR.BYTE = 0x08;

	//	SCMR - Smart Card Mode Register
	//		b6:b4	Reserved - The write value should be 1.
	//		b3		SDIR		- Transmitted/Received Data Transfer Direction - Transfer with LSB-first
	//		b2		SINV		- Transmitted/Received Data Invert	- TDR contents are transmitted as they are. 
	//													Receive data is stored as it is in RDR.
	//		b1		Reserved - The write value should be 1.
	//		b0		SMIF		- Smart Card Interface Mode Select	- Serial communications interface mode
	SCI2.SCMR.BYTE = 0xF2;

	//	SEMR - Serial Extended Mode Register
	//		b7:b6	Reserved - The write value should be 0.
	//		b5		NFEN		- Digital Noise Filter Function Enable	- Noise cancellation function 
	//														for the RXDn input signal is disabled.
	//		b4		ABCS		- Asynchronous Mode Base Clock Select	- Selects 16 base clock cycles for 1-bit period
	//		b3:b1	Reserved - The write value should be 0.
	SCI2.SEMR.BYTE = 0x00;

	//	Set data transfer format in Serial Mode Register (SMR)
	//		-Asynchronous Mode`
	//		-8 bits
	//		-no parity
	//		-1 stop bit
	//		-PCLK clock (n = 0)
	SCI2.SMR.BYTE = 0x00;		//	0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64

	//	BRR - Bit Rate Register
	//		Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04
	if(stoplen == 1)
	{
		SCI2.SMR.BIT.STOP = 1;
	}
	else
	{
		SCI2.SMR.BIT.STOP = 0;
	}

	if(parity == 0)
	{
		SCI2.SMR.BIT.PE = 0;
	}
	else
	if(parity == 1)
	{		//　奇数パリティ
		SCI2.SMR.BIT.PE = 1;
		SCI2.SMR.BIT.PM = 1;
	}
	else
	if(parity == 2)
	{	//	偶数パリティ
		SCI2.SMR.BIT.PE = 1;
		SCI2.SMR.BIT.PM = 0;
	}

	if(datalen == 7)
	{		//	7bit長
		SCI2.SMR.BIT.CHR = 1;
	}
	else
	if(datalen == 8)
	{	//	8bit長
		SCI2.SMR.BIT.CHR = 0;
	}

	//	Set baud rate to 115200
	//		N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
	//		N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
	//		N = 12
	SCI2.BRR = 48000000 / ((64/2) * bps) - 1;

	//	SCI割り込み優先順位設定
	ICU.IPR[IPR_SCI2_].BIT.IPR = 1;						//	割り込みレベル設定

	//	SCI2 割り込み設定
	ICU.IER[IER_SCI2_RXI2].BIT.IEN_SCI2_RXI2 = 1;		//	受信割り込み
	ICU.IER[IER_SCI2_TXI2].BIT.IEN_SCI2_TXI2 = 1;		//	送信完了割り込み
	ICU.IER[IER_SCI2_TEI2].BIT.IEN_SCI2_TEI2 = 1;		//	送信エンプティ割り込み

	//	割り込みフラグクリア
	ICU.IR[IR_SCI2_RXI2].BIT.IR = 0;
	ICU.IR[IR_SCI2_TXI2].BIT.IR = 0;
	ICU.IR[IR_SCI2_TEI2].BIT.IR = 0;

	//	GROUP12割り込み設定
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.GEN[GEN_SCI2_ERI2].BIT.EN_SCI2_ERI2 = 1;		//	グループ12 SCI2受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア

	SCI2.SCR.BIT.RIE = 1;
	SCI2.SCR.BIT.RE = 1;

	PORT1.PODR.BIT.B5 = 0;	//	RTS=Enable

#endif		/*SCI2_ACTIVATE*/
}

//________________________________________________________________________________________
//
//	sci3_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI3初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			SCI3	P23			TXD3								<RS-232C>	COM3
//					P25			RXD3								<RS-232C>	COM3
//					P22			nRTS3								<RS-232C>	COM3
//					P24			nCTS3								<RS-232C>	COM3
//					P56			nEXRES								</RESET>	外部モジュールリセット信号
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
void sci3_init(long bps, int datalen, int stoplen, int parity)
{
	SCI_MODULE	*com = &sci_com[3];
	memset(com, 0, sizeof(SCI_MODULE));

#ifdef		SCI3_ACTIVATE

	SYSTEM.PRCR.WORD = 0xA502;		//	プロテクト解除
	MSTP_SCI3 = 0;					//	SCIモジュールストップ解除

	//	SCI 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 割り込み禁止
	ICU.IER[IER_SCI3_RXI3].BIT.IEN_SCI3_RXI3 = 0;		//	受信割り込み禁止
	ICU.IER[IER_SCI3_TXI3].BIT.IEN_SCI3_TXI3 = 0;		//	送信完了割り込み禁止
	ICU.IER[IER_SCI3_TEI3].BIT.IEN_SCI3_TEI3 = 0;		//	送信エンプティ割り込み禁止

	//	Enable write protection(プロテクト掛ける)
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	//	SCR - Serial Control Register
	//	b7		TIE	- Transmit Interrupt Enable		- A TXI interrupt request is disabled
	//	b6		RIE	- Receive Interrupt Enable		- RXI and ERI interrupt requests are disabled
	//	b5		TE	- Transmit Enable				- Serial transmission is disabled
	//	b4		RE	- Receive Enable				- Serial reception is disabled
	//	b2		TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled
	SCI3.SCR.BYTE = 0x00;

	while (0x00 != (SCI3.SCR.BYTE & 0xF0))
	{
		//	Confirm that bit is actually 0
	}

	//	Set the I/O port functions

	//	汎用ポートP23:TXD3、P24:RXD3
	PORT2.PODR.BIT.B3 = 1;		//	TXD
	PORT2.PODR.BIT.B2 = 1;		//	RTS=Disable
	//	Set port direction - TXDn is output port, nCTS/RXDn is input port(ポート入出力設定)
	PORT2.PDR.BIT.B3 = 1;		//	出力	TXD
	PORT2.PDR.BIT.B5 = 0;		//	入力	RXD
	PORT2.PDR.BIT.B2 = 1;		//	出力	RTS
	PORT2.PDR.BIT.B4 = 0;		//	入力	CTS

	//	Set port mode - Use pin as general I/O port
	PORT2.PMR.BIT.B3 = 0;		//	汎用IOポート設定
	PORT2.PMR.BIT.B5 = 0;		//	汎用IOポート設定
	PORT2.PMR.BIT.B2 = 0;		//	汎用IOポート設定
	PORT2.PMR.BIT.B4 = 0;		//	汎用IOポート設定

	//	PWPR - Write-Protect Register(書き込みプロテクトレジスタ)
	//	b7		B0WI		- PFSWE Bit Write Disable	- PFSWE禁止
	//	b6		PFSWE	- PFS Register Write Enable - PFS許可
	//	b5:b0	Reserved - These bits are read as 0. The write value should be 0.
	MPC.PWPR.BIT.B0WI = 0;			//	先に0にする
	MPC.PWPR.BIT.PFSWE = 1;			//	後に1にする

	//	PFS - Pin Function Control Register(ピンファンクションレジスタ設定)
	//	b3:b0	PSEL - Pin Function Select - RXDn, TXDn
	MPC.P23PFS.BYTE = 0x0A;		//	assign I/O pin to SCI3 TxD3
	MPC.P25PFS.BYTE = 0x0A;		//	assign I/O pin to SCI3 RxD3
	MPC.P22PFS.BYTE = 0x00;		//	assign I/O pin to Port
	MPC.P24PFS.BYTE = 0x00;		//	assign I/O pin to Port
	//	書き込みプロテクトをかける
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;

	//	Use pin as I/O port for peripheral functions(IOピン機能設定)
	PORT2.PMR.BIT.B3 = 1;			//	周辺機能設定
	PORT2.PMR.BIT.B5 = 1;			//	周辺機能設定

	//	Initialization of SCI


	//	Select an On-chip baud rate generator to the clock source
	SCI3.SCR.BIT.CKE = 0;

	//	SMR - Serial Mode Register
	//	b7		CM	- Communications Mode	- Asynchronous mode
	//	b6		CHR	- Character Length		- Selects 8 bits as the data length
	//	b5		PE	- Parity Enable			- When transmitting : Parity bit addition is not performed
	//								When receiving	: Parity bit checking is not performed
	//	b3		STOP - Stop Bit Length		- 2 stop bits
	//	b2		MP	- Multi-Processor Mode	- Multi-processor communications function is disabled
	//	b1:b0	CKS	- Clock Select			- PCLK clock (n = 0)
	SCI3.SMR.BYTE = 0x08;

	//	SCMR - Smart Card Mode Register
	//	b6:b4	Reserved - The write value should be 1.
	//	b3		SDIR		- Transmitted/Received Data Transfer Direction - Transfer with LSB-first
	//	b2		SINV		- Transmitted/Received Data Invert	- TDR contents are transmitted as they are. 
	//												Receive data is stored as it is in RDR.
	//	b1		Reserved - The write value should be 1.
	//	b0		SMIF		- Smart Card Interface Mode Select	- Serial communications interface mode
	SCI3.SCMR.BYTE = 0xF2;

	//	SEMR - Serial Extended Mode Register
	//	b7:b6	Reserved - The write value should be 0.
	//	b5		NFEN		- Digital Noise Filter Function Enable	- Noise cancellation function 
	//													for the RXDn input signal is disabled.
	//	b4		ABCS		- Asynchronous Mode Base Clock Select	- Selects 16 base clock cycles for 1-bit period
	//	b3:b1	Reserved - The write value should be 0.
	SCI3.SEMR.BYTE = 0x00;

	//	Set data transfer format in Serial Mode Register (SMR)
	//		-Asynchronous Mode`
	//		-8 bits
	//		-no parity
	//		-1 stop bit
	//		-PCLK clock (n = 0)
	SCI3.SMR.BYTE = 0x00;		//	0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64

	//	BRR - Bit Rate Register
	//	Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04
	if(stoplen == 1)
	{
		SCI3.SMR.BIT.STOP = 1;
	}
	else
	{
		SCI3.SMR.BIT.STOP = 0;
	}

	if(parity == 0)
	{
		SCI3.SMR.BIT.PE = 0;
	}
	else
	if(parity == 1)
	{		//　奇数パリティ
		SCI3.SMR.BIT.PE = 1;
		SCI3.SMR.BIT.PM = 1;
	}
	else
	if(parity == 2)
	{	//	偶数パリティ
		SCI3.SMR.BIT.PE = 1;
		SCI3.SMR.BIT.PM = 0;
	}

	if(datalen == 7)
	{		//	7bit長
		SCI3.SMR.BIT.CHR = 1;
	}
	else
	if(datalen == 8)
	{	//	8bit長
		SCI3.SMR.BIT.CHR = 0;
	}

	//	Set baud rate to 115200
	//	N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
	//	N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
	//	N = 12
	SCI3.BRR = 48000000 / ((64/2) * bps) - 1;

	//	SCI割り込み優先順位設定
	ICU.IPR[IPR_SCI3_].BIT.IPR = 1;						//	割り込みレベル設定

	//	SCI3 割り込み設定
	ICU.IER[IER_SCI3_RXI3].BIT.IEN_SCI3_RXI3 = 1;		//	受信割り込み
	ICU.IER[IER_SCI3_TXI3].BIT.IEN_SCI3_TXI3 = 1;		//	送信完了割り込み
	ICU.IER[IER_SCI3_TEI3].BIT.IEN_SCI3_TEI3 = 1;		//	送信エンプティ割り込み

	//	割り込みフラグクリア
	ICU.IR[IR_SCI3_RXI3].BIT.IR = 0;
	ICU.IR[IR_SCI3_TXI3].BIT.IR = 0;
	ICU.IR[IR_SCI3_TEI3].BIT.IR = 0;

	//	GROUP12割り込み設定
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.GEN[GEN_SCI3_ERI3].BIT.EN_SCI3_ERI3 = 1;		//	グループ12 SCI3受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア

	SCI3.SCR.BIT.RIE = 1;
	SCI3.SCR.BIT.RE = 1;

	PORT2.PODR.BIT.B2 = 0;		//	RTS=Enable

#endif		/*SCI3_ACTIVATE*/
}

//________________________________________________________________________________________
//
//	spi5_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI5を簡易SPIモードで初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			SCI5	PC3			TXD5					SMOSI5		<SPI>		外部拡張
//					PC2			RXD5					SMISO5		<SPI>		外部拡張
//					PC4									SCK5		<SPI>		外部拡張
//					PC5									SS0			<SPI>		外部拡張
//					PC6									SS1			<SPI>		外部拡張
//	引数
//		speed		通信速度
//	戻り
//		無し
//________________________________________________________________________________________
//
void spi5_init(long bps)
{
	SCI_MODULE	*com = &sci_com[5];
	memset(com, 0, sizeof(SCI_MODULE));

#ifdef		SCI5_ACTIVATE

	SYSTEM.PRCR.WORD = 0xA502;		//	プロテクト解除
	MSTP_SCI5 = 0;					//	SCIモジュールストップ解除

	//	SCI 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 割り込み禁止
	ICU.IER[IER_SCI5_RXI5].BIT.IEN_SCI5_RXI5 = 0;		//	受信割り込み禁止
	ICU.IER[IER_SCI5_TXI5].BIT.IEN_SCI5_TXI5 = 0;		//	送信完了割り込み禁止
	ICU.IER[IER_SCI5_TEI5].BIT.IEN_SCI5_TEI5 = 0;		//	送信エンプティ割り込み禁止

	//	Enable write protection(プロテクト掛ける)
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	//	SCR - Serial Control Register
	//	b7		TIE	- Transmit Interrupt Enable		- A TXI interrupt request is disabled
	//	b6		RIE	- Receive Interrupt Enable		- RXI and ERI interrupt requests are disabled
	//	b5		TE	- Transmit Enable				- Serial transmission is disabled
	//	b4		RE	- Receive Enable				- Serial reception is disabled
	//	b2		TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled
	SCI5.SCR.BYTE = 0x00;

	while (0x00 != (SCI5.SCR.BYTE & 0xF0))
	{
		//	Confirm that bit is actually 0
	}

	//	Set the I/O port functions

	//	汎用ポートPC3:TXD5、PC2:RXD5
	PORTC.PODR.BIT.B3 = 1;		//	SMOSI5
	PORTC.PODR.BIT.B4 = 1;		//	SCK5
	PORTC.PODR.BIT.B5 = 1;		//	SS0=Disable
	PORTC.PODR.BIT.B6 = 1;		//	SS1=Disable
	//	Set port direction - TXDn is output port, RXDn is input port(ポート入出力設定)
	PORTC.PDR.BIT.B3 = 1;		//	出力	MOSI
	PORTC.PDR.BIT.B2 = 0;		//	入力	MISO
	PORTC.PDR.BIT.B4 = 1;		//	出力	SCK5
	PORTC.PDR.BIT.B5 = 1;		//	出力	SS0
	PORTC.PDR.BIT.B6 = 1;		//	出力	SS1

	//	Set port mode - Use pin as general I/O port
	PORTC.PMR.BIT.B3 = 0;		//	汎用IOポート設定
	PORTC.PMR.BIT.B2 = 0;		//	汎用IOポート設定
	PORTC.PMR.BIT.B4 = 0;		//	汎用IOポート設定
	PORTC.PMR.BIT.B5 = 0;		//	汎用IOポート設定
	PORTC.PMR.BIT.B6 = 0;		//	汎用IOポート設定

	//	PWPR - Write-Protect Register(書き込みプロテクトレジスタ)
	//	b7		B0WI		- PFSWE Bit Write Disable	- PFSWE禁止
	//	b6		PFSWE	- PFS Register Write Enable - PFS許可
	//	b5:b0	Reserved - These bits are read as 0. The write value should be 0.
	MPC.PWPR.BIT.B0WI = 0;			//	先に0にする
	MPC.PWPR.BIT.PFSWE = 1;			//	後に1にする

	//	PFS - Pin Function Control Register(ピンファンクションレジスタ設定)
	//	b3:b0	PSEL - Pin Function Select - RXDn, TXDn
	MPC.PC3PFS.BYTE = 0x0A;		//	assign I/O pin to SCI5 TXD5
	MPC.PC2PFS.BYTE = 0x0A;		//	assign I/O pin to SCI5 RXD5
	MPC.PC4PFS.BYTE = 0x0A;		//	assign I/O pin to SCI5 SCK5
	MPC.PC5PFS.BYTE = 0x00;		//	assign I/O pin to port
	MPC.PC6PFS.BYTE = 0x00;		//	assign I/O pin to port
	//	書き込みプロテクトをかける
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;

	//	Use pin as I/O port for peripheral functions(IOピン機能設定)
	PORTC.PMR.BIT.B3 = 1;			//	周辺機能設定
	PORTC.PMR.BIT.B2 = 1;			//	周辺機能設定
	PORTC.PMR.BIT.B4 = 1;			//	周辺機能設定

	//	Initialization of SCI


	//	Select an On-chip baud rate generator to the clock source
	SCI5.SCR.BIT.CKE = 0;

	//	SMR - Serial Mode Register
	//	b7		CM	- Communications Mode	- Asynchronous mode
	//	b6		CHR	- Character Length		- Selects 8 bits as the data length
	//	b5		PE	- Parity Enable			- When transmitting : Parity bit addition is not performed
	//								When receiving	: Parity bit checking is not performed
	//	b3		STOP - Stop Bit Length		- 2 stop bits
	//	b2		MP	- Multi-Processor Mode	- Multi-processor communications function is disabled
	//	b1:b0	CKS	- Clock Select			- PCLK clock (n = 0)
	SCI5.SMR.BYTE = 0x88;

	//	SCMR - Smart Card Mode Register
	//	b6:b4	Reserved - The write value should be 1.
	//	b3		SDIR		- Transmitted/Received Data Transfer Direction - Transfer with LSB-first
	//	b2		SINV		- Transmitted/Received Data Invert	- TDR contents are transmitted as they are. 
	//												Receive data is stored as it is in RDR.
	//	b1		Reserved - The write value should be 1.
	//	b0		SMIF		- Smart Card Interface Mode Select	- Serial communications interface mode
	SCI5.SCMR.BYTE = 0xF2;

	//	SEMR - Serial Extended Mode Register
	//	b7:b6	Reserved - The write value should be 0.
	//	b5		NFEN		- Digital Noise Filter Function Enable	- Noise cancellation function 
	//													for the RXDn input signal is disabled.
	//	b4		ABCS		- Asynchronous Mode Base Clock Select	- Selects 16 base clock cycles for 1-bit period
	//	b3:b1	Reserved - The write value should be 0.
	SCI5.SEMR.BYTE = 0x00;

	//	Set data transfer format in Serial Mode Register (SMR)*/ 
	//		-Asynchronous Mode`
	//		-8 bits
	//		-no parity
	//		-1 stop bit
	//		-PCLK clock (n = 0)
	SCI5.SMR.BYTE = 0x00;		//	0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64

	//	BRR - Bit Rate Register
	//	Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04
	//	Set baud rate to 115200
	//	N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
	//	N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
	//	N = 12
	SCI5.BRR = 48000000 / ((64/2) * bps) - 1;

	//	SCI割り込み優先順位設定
	ICU.IPR[IPR_SCI5_].BIT.IPR = 1;						//	割り込みレベル設定

	//	SCI4 割り込み設定
	ICU.IER[IER_SCI5_RXI5].BIT.IEN_SCI5_RXI5 = 1;		//	受信割り込み
	ICU.IER[IER_SCI5_TXI5].BIT.IEN_SCI5_TXI5 = 1;		//	送信完了割り込み
	ICU.IER[IER_SCI5_TEI5].BIT.IEN_SCI5_TEI5 = 1;		//	送信エンプティ割り込み

	//	割り込みフラグクリア
	ICU.IR[IR_SCI5_RXI5].BIT.IR = 0;
	ICU.IR[IR_SCI5_TXI5].BIT.IR = 0;
	ICU.IR[IR_SCI5_TEI5].BIT.IR = 0;

	//	GROUP12割り込み設定
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.GEN[GEN_SCI5_ERI5].BIT.EN_SCI5_ERI5 = 1;		//	グループ12 SCI5受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア

	//	簡易SPIモードに強制
	SCI5.SCMR.BIT.SMIF = 0;		//	
	SCI5.SIMR1.BIT.IICM = 0;	//	
	SCI5.SMR.BIT.CM = 1;		//	同期モード
	SCI5.SPMR.BIT.SSE = 0;		//	シングルマスタ

	SCI5.SCR.BIT.RIE = 1;
	SCI5.SCR.BIT.RE = 1;

#endif		/*SCI5_ACTIVATE*/
}

//________________________________________________________________________________________
//
//	sci6_init
//----------------------------------------------------------------------------------------
//	機能説明
//		SCI6初期化
//					Port		SCI			I2C			SPI			適用
//			----------------------------------------------------------------------------
//			SCI6	P00			TXD6								<TTL>		COM6
//					P01			RXD6								<TTL>		COM6
//					P02			nRTS6								<TTL>		COM6
//					PJ3			nCTS6								<TTL>		COM6
//	引数
//		speed		通信速度	300～115200
//		datalen		データ長	7,8
//		stoplen		ストップ長	1,2
//		parity		パリティー	0=無し / 1=奇数 / 2=偶数
//	戻り
//		無し
//________________________________________________________________________________________
//
void sci6_init(long bps, int datalen, int stoplen, int parity)
{
	SCI_MODULE	*com = &sci_com[6];
	memset(com, 0, sizeof(SCI_MODULE));

#ifdef		SCI6_ACTIVATE

	SYSTEM.PRCR.WORD = 0xA502;		//	プロテクト解除
	MSTP_SCI6 = 0;					//	SCIモジュールストップ解除

	//	SCI 割り込み要求禁止
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 0;	//	グループ12 割り込み禁止
	ICU.IER[IER_SCI6_RXI6].BIT.IEN_SCI6_RXI6 = 0;		//	受信割り込み禁止
	ICU.IER[IER_SCI6_TXI6].BIT.IEN_SCI6_TXI6 = 0;		//	送信完了割り込み禁止
	ICU.IER[IER_SCI6_TEI6].BIT.IEN_SCI6_TEI6 = 0;		//	送信エンプティ割り込み禁止

	//	Enable write protection(プロテクト掛ける)
	SYSTEM.PRCR.WORD = 0xA500;		//	プロテクト

	//	SCR - Serial Control Register
	//	b7		TIE	- Transmit Interrupt Enable		- A TXI interrupt request is disabled
	//	b6		RIE	- Receive Interrupt Enable		- RXI and ERI interrupt requests are disabled
	//	b5		TE	- Transmit Enable				- Serial transmission is disabled
	//	b4		RE	- Receive Enable				- Serial reception is disabled
	//	b2		TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled
	SCI6.SCR.BYTE = 0x00;

	while (0x00 != (SCI6.SCR.BYTE & 0xF0))
	{
		//	Confirm that bit is actually 0
	}

	//	Set the I/O port functions
	//	汎用ポートP00:TXD6、P01:RXD6
	PORT0.PODR.BIT.B0 = 1;		//	TXD
	PORT0.PODR.BIT.B2 = 1;		//	RTS=Disable
	//	Set port direction - TXDn is output port, RXDn is input port(ポート入出力設定)
	PORT0.PDR.BIT.B0 = 1;		//	出力	TXD
	PORT0.PDR.BIT.B1 = 0;		//	入力	RXD
	PORT0.PDR.BIT.B2 = 1;		//	出力	RTS
	PORTJ.PDR.BIT.B3 = 0;		//	入力	CTS

	//	Set port mode - Use pin as general I/O port
	PORT0.PMR.BIT.B0 = 0;		//	汎用IOポート設定
	PORT0.PMR.BIT.B1 = 0;		//	汎用IOポート設定
	PORT0.PMR.BIT.B2 = 0;		//	汎用IOポート設定
	PORTJ.PMR.BIT.B3 = 0;		//	汎用IOポート設定

	//	PWPR - Write-Protect Register(書き込みプロテクトレジスタ)
	//	b7		B0WI		- PFSWE Bit Write Disable	- PFSWE禁止
	//	b6		PFSWE	- PFS Register Write Enable - PFS許可
	//	b5:b0	Reserved - These bits are read as 0. The write value should be 0.
	MPC.PWPR.BIT.B0WI = 0;			//	先に0にする
	MPC.PWPR.BIT.PFSWE = 1;			//	後に1にする

	//	PFS - Pin Function Control Register(ピンファンクションレジスタ設定)
	//	b3:b0	PSEL - Pin Function Select - RXDn, TXDn
	MPC.P00PFS.BYTE = 0x0A;		//	assign I/O pin to SCI6 TXD6
	MPC.P01PFS.BYTE = 0x0A;		//	assign I/O pin to SCI6 RXD6
	MPC.P02PFS.BYTE = 0x00;		//	assign I/O pin to port
	MPC.PJ3PFS.BYTE = 0x00;		//	assign I/O pin to port
	//	書き込みプロテクトをかける
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;

	//	Use pin as I/O port for peripheral functions(IOピン機能設定)
	PORT0.PMR.BIT.B0 = 1;			//	周辺機能設定
	PORT0.PMR.BIT.B1 = 1;			//	周辺機能設定
	//	Initialization of SCI


	//	Select an On-chip baud rate generator to the clock source
	SCI6.SCR.BIT.CKE = 0;

	//	SMR - Serial Mode Register
	//	b7		CM	- Communications Mode	- Asynchronous mode
	//	b6		CHR	- Character Length		- Selects 8 bits as the data length
	//	b5		PE	- Parity Enable			- When transmitting : Parity bit addition is not performed
	//								When receiving	: Parity bit checking is not performed
	//	b3		STOP - Stop Bit Length		- 2 stop bits
	//	b2		MP	- Multi-Processor Mode	- Multi-processor communications function is disabled
	//	b1:b0	CKS	- Clock Select			- PCLK clock (n = 0)
	SCI6.SMR.BYTE = 0x08;

	//	SCMR - Smart Card Mode Register
	//	b6:b4	Reserved - The write value should be 1.
	//	b3		SDIR		- Transmitted/Received Data Transfer Direction - Transfer with LSB-first
	//	b2		SINV		- Transmitted/Received Data Invert	- TDR contents are transmitted as they are. 
	//												Receive data is stored as it is in RDR.
	//	b1		Reserved - The write value should be 1.
	//	b0		SMIF		- Smart Card Interface Mode Select	- Serial communications interface mode
	SCI6.SCMR.BYTE = 0xF2;

	//	SEMR - Serial Extended Mode Register
	//	b7:b6	Reserved - The write value should be 0.
	//	b5		NFEN		- Digital Noise Filter Function Enable	- Noise cancellation function 
	//													for the RXDn input signal is disabled.
	//	b4		ABCS		- Asynchronous Mode Base Clock Select	- Selects 16 base clock cycles for 1-bit period
	//	b3:b1	Reserved - The write value should be 0.
	SCI6.SEMR.BYTE = 0x00;

	//	Set data transfer format in Serial Mode Register (SMR)
	//		-Asynchronous Mode`
	//		-8 bits
	//		-no parity
	//		-1 stop bit
	//		-PCLK clock (n = 0)
	SCI6.SMR.BYTE = 0x00;		//	0=PCLK, 1=PCLK/4, 2=PCLK/16, 3=PCLK/64

	//	BRR - Bit Rate Register
	//	Bit Rate: (48MHz/(64*2^(-1)*57600bps))-1=25.04
	if(stoplen == 1)
	{
		SCI6.SMR.BIT.STOP = 1;
	}
	else
	{
		SCI6.SMR.BIT.STOP = 0;
	}

	if(parity == 0)
	{
		SCI6.SMR.BIT.PE = 0;
	}
	else
	if(parity == 1)
	{		//　奇数パリティ
		SCI6.SMR.BIT.PE = 1;
		SCI6.SMR.BIT.PM = 1;
	}
	else
	if(parity == 2)
	{	//	偶数パリティ
		SCI6.SMR.BIT.PE = 1;
		SCI6.SMR.BIT.PM = 0;
	}

	if(datalen == 7)
	{		//	7bit長
		SCI6.SMR.BIT.CHR = 1;
	}
	else
	if(datalen == 8)
	{	//	8bit長
		SCI6.SMR.BIT.CHR = 0;
	}

	//	Set baud rate to 115200
	//	N = (PCLK Frequency) / (64 * 2^(2*n - 1) * Bit Rate) - 1
	//	N = (48,000,000) / (64 * 2^(2*0 - 1) * 115200) - 1
	//	N = 12
	SCI6.BRR = 48000000 / ((64/2) * bps) - 1;

	//	SCI割り込み優先順位設定
	ICU.IPR[IPR_SCI6_].BIT.IPR = 1;						//	割り込みレベル設定

	//	SCI6 割り込み設定
	ICU.IER[IER_SCI6_RXI6].BIT.IEN_SCI6_RXI6 = 1;		//	受信割り込み
	ICU.IER[IER_SCI6_TXI6].BIT.IEN_SCI6_TXI6 = 1;		//	送信完了割り込み
	ICU.IER[IER_SCI6_TEI6].BIT.IEN_SCI6_TEI6 = 1;		//	送信エンプティ割り込み

	//	割り込みフラグクリア
	ICU.IR[IR_SCI6_RXI6].BIT.IR = 0;
	ICU.IR[IR_SCI6_TXI6].BIT.IR = 0;
	ICU.IR[IR_SCI6_TEI6].BIT.IR = 0;

	//	GROUP12割り込み設定
	ICU.IER[IER_ICU_GROUPL0].BIT.IEN_ICU_GROUPL0 = 1;	//	グループ12 割り込み許可
	ICU.GEN[GEN_SCI6_ERI6].BIT.EN_SCI6_ERI6 = 1;		//	グループ12 SCI6受信エラー割り込み許可
	ICU.IPR[IPR_ICU_GROUPL0].BIT.IPR = 1;				//	グル―プ12 割り込みレベル設定
	ICU.IR[IR_ICU_GROUPL0].BIT.IR = 0;					//	グループ12 割り込みフラグクリア

	SCI6.SCR.BIT.RIE = 1;
	SCI6.SCR.BIT.RE = 1;

	PORT0.PODR.BIT.B2 = 0;		//	RTS=Enable

#endif		/*SCI6_ACTIVATE*/
}

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
int sci_putcheck(int ch)
{
	SCI_MODULE	*com = &sci_com[ch];
	int sz = (com->txwp - com->txrp);
	if(sz < 0) sz += BUFSIZE;
	return (BUFSIZE - sz);
}

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
int sci_txbytes(int ch)
{
	SCI_MODULE	*com = &sci_com[ch];
	int sz = (com->txwp - com->txrp);
	if(sz < 0) sz += BUFSIZE;
	return sz;
}

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
//----------------------------------------------------------------------------------------
//	SCI0	SEND
//----------------------------------------------------------------------------------------
void	sci0_putb(unsigned char *buf, int size)
{
	int			i, ch = 0;
	SCI_MODULE	*com = &sci_com[ch];
	while(size > 0)
	{
#ifdef		SCI0_ACTIVATE
#ifdef	SCI0_FLOW
		if(SCI0_CTS_PORT != 0)
		{	//	CTS=Disable
			if(sci_putcheck(ch) < 2) return;
		}
#endif
		while( sci_putcheck(ch) < 2)
		{	//	バッファが空くまで待つ
			if(SCI0.SCR.BIT.TE == 0)
			{	//	送信開始処理
#ifdef	SCI0_FLOW
				if(SCI0_CTS_PORT == 0)
				{	//	CTS=Enable
#endif
					SCI0.SCR.BIT.TIE = 1;				//	割り込みイネーブル
					SCI0.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI0_FLOW
				}
#endif
			}
		}
#endif
		com->txbuf[com->txwp++] = *buf++;
		size--;
		if(com->txwp >= BUFSIZE) com->txwp = 0;
#ifdef		SCI0_ACTIVATE
		if(SCI0.SCR.BIT.TE == 0)
		{	//	送信開始処理
#ifdef	SCI0_FLOW
			if(SCI0_CTS_PORT == 0)
			{	//	CTS=Enable
#endif
				SCI0.SCR.BIT.TIE = 1;				//	割り込みイネーブル
				SCI0.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI0_FLOW
			}
#endif
		}
#endif
	}
#ifdef		SCI0_ACTIVATE
	if(SCI0.SCR.BIT.TE == 0)
	{	//	送信開始処理
#ifdef	SCI0_FLOW
		if(SCI0_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			SCI0.SCR.BIT.TIE = 1;				//	割り込みイネーブル
			SCI0.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI0_FLOW
		}
#endif
	}
#endif
}
//----------------------------------------------------------------------------------------
//	SCI1	SEND
//----------------------------------------------------------------------------------------
void	sci1_putb(unsigned char *buf, int size)
{
	int			i, ch = 1;
	SCI_MODULE	*com = &sci_com[ch];
	while(size > 0)
	{
#ifdef		SCI1_ACTIVATE
#ifdef	SCI1_FLOW
		if(SCI1_CTS_PORT != 0)
		{	//	CTS=Disable
			if(sci_putcheck(ch) < 2) return;
		}
#endif
		while( sci_putcheck(ch) < 2)
		{	//	バッファが空くまで待つ
			if(SCI1.SCR.BIT.TE == 0)
			{	//	送信開始処理
#ifdef	SCI1_FLOW
				if(SCI1_CTS_PORT == 0)
				{	//	CTS=Enable
#endif
					SCI1.SCR.BIT.TIE = 1;				//	割り込みイネーブル
					SCI1.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI1_FLOW
				}
#endif
			}
		}
#endif
		com->txbuf[com->txwp++] = *buf++;
		size--;
		if(com->txwp >= BUFSIZE) com->txwp = 0;
	#ifdef		SCI1_ACTIVATE
		if(SCI1.SCR.BIT.TE == 0)
		{	//	送信開始処理
	#ifdef	SCI1_FLOW
			if(SCI1_CTS_PORT == 0)
			{	//	CTS=Enable
	#endif
				SCI1.SCR.BIT.TIE = 1;				//	割り込みイネーブル
				SCI1.SCR.BIT.TE = 1;				//	送信イネブール
	#ifdef	SCI1_FLOW
			}
	#endif
		}
	#endif
	}
#ifdef		SCI1_ACTIVATE
	if(SCI1.SCR.BIT.TE == 0)
	{	//	送信開始処理
#ifdef	SCI1_FLOW
		if(SCI1_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			SCI1.SCR.BIT.TIE = 1;				//	割り込みイネーブル
			SCI1.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI1_FLOW
		}
#endif
	}
#endif
}
//----------------------------------------------------------------------------------------
//	SCI2	SEND
//----------------------------------------------------------------------------------------
void	sci2_putb(unsigned char *buf, int size)
{
	int			i, ch = 2;
	SCI_MODULE	*com = &sci_com[ch];
	while(size > 0)
	{
#ifdef		SCI2_ACTIVATE
#ifdef	SCI2_FLOW
		if(SCI2_CTS_PORT != 0)
		{	//	CTS=Disable
			if(sci_putcheck(ch) < 2) return;
		}
#endif
		while( sci_putcheck(ch) < 2)
		{	//	バッファが空くまで待つ
			if(SCI2.SCR.BIT.TE == 0)
			{	//	送信開始処理
#ifdef	SCI2_FLOW
				if(SCI2_CTS_PORT == 0)
				{	//	CTS=Enable
#endif
					SCI2.SCR.BIT.TIE = 1;				//	割り込みイネーブル
					SCI2.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI2_FLOW
				}
#endif
			}
		}
#endif
		com->txbuf[com->txwp++] = *buf++;
		size--;
		if(com->txwp >= BUFSIZE) com->txwp = 0;
	#ifdef		SCI2_ACTIVATE
		if(SCI2.SCR.BIT.TE == 0)
		{	//	送信開始処理
	#ifdef	SCI2_FLOW
			if(SCI2_CTS_PORT == 0)
			{	//	CTS=Enable
	#endif
				SCI2.SCR.BIT.TIE = 1;				//	割り込みイネーブル
				SCI2.SCR.BIT.TE = 1;				//	送信イネブール
	#ifdef	SCI2_FLOW
			}
	#endif
		}
	#endif
	}
#ifdef		SCI2_ACTIVATE
	if(SCI2.SCR.BIT.TE == 0)
	{	//	送信開始処理
#ifdef	SCI2_FLOW
		if(SCI2_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			SCI2.SCR.BIT.TIE = 1;				//	割り込みイネーブル
			SCI2.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI2_FLOW
		}
#endif
	}
#endif
}
//----------------------------------------------------------------------------------------
//	SCI3	SEND
//----------------------------------------------------------------------------------------
void	sci3_putb(unsigned char *buf, int size)
{
	int			i, ch = 3;
	SCI_MODULE	*com = &sci_com[ch];
	while(size > 0)
	{
#ifdef		SCI3_ACTIVATE
#ifdef	SCI3_FLOW
		if(SCI3_CTS_PORT != 0)
		{	//	CTS=Disable
			if(sci_putcheck(ch) < 2) return;
		}
#endif
		while( sci_putcheck(ch) < 2)
		{	//	バッファが空くまで待つ
			if(SCI3.SCR.BIT.TE == 0)
			{	//	送信開始処理
#ifdef	SCI3_FLOW
				if(SCI3_CTS_PORT == 0)
				{	//	CTS=Enable
#endif
					SCI3.SCR.BIT.TIE = 1;				//	割り込みイネーブル
					SCI3.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI3_FLOW
				}
#endif
			}
		}
#endif
		com->txbuf[com->txwp++] = *buf++;
		size--;
		if(com->txwp >= BUFSIZE) com->txwp = 0;
	#ifdef		SCI3_ACTIVATE
		if(SCI3.SCR.BIT.TE == 0)
		{	//	送信開始処理
	#ifdef	SCI3_FLOW
			if(SCI3_CTS_PORT == 0)
			{	//	CTS=Enable
	#endif
				SCI3.SCR.BIT.TIE = 1;				//	割り込みイネーブル
				SCI3.SCR.BIT.TE = 1;				//	送信イネブール
	#ifdef	SCI3_FLOW
			}
	#endif
		}
	#endif
	}
#ifdef		SCI3_ACTIVATE
	if(SCI3.SCR.BIT.TE == 0)
	{	//	送信開始処理
#ifdef	SCI3_FLOW
		if(SCI3_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			SCI3.SCR.BIT.TIE = 1;				//	割り込みイネーブル
			SCI3.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI3_FLOW
		}
#endif
	}
#endif
}
//----------------------------------------------------------------------------------------
//	SCI5	SEND
//----------------------------------------------------------------------------------------
void	sci5_putb(unsigned char *buf, int size)
{
	int			i, ch = 5;
	SCI_MODULE	*com = &sci_com[ch];
	while(size > 0)
	{
#ifdef		SCI5_ACTIVATE
		while( sci_putcheck(ch) < 2)
		{	//	バッファが空くまで待つ
			if(SCI5.SCR.BIT.TE == 0)
			{	//	送信開始処理
				SCI5.SCR.BIT.TIE = 1;				//	割り込みイネーブル
				SCI5.SCR.BIT.TE = 1;				//	送信イネブール
			}
		}
#endif
		com->txbuf[com->txwp++] = *buf++;
		size--;
		if(com->txwp >= BUFSIZE) com->txwp = 0;
	#ifdef		SCI5_ACTIVATE
		if(SCI5.SCR.BIT.TE == 0)
		{	//	送信開始処理
			SCI5.SCR.BIT.TIE = 1;				//	割り込みイネーブル
			SCI5.SCR.BIT.TE = 1;				//	送信イネブール
		}
	#endif
	}
#ifdef		SCI5_ACTIVATE
	if(SCI5.SCR.BIT.TE == 0)
	{	//	送信開始処理
		SCI5.SCR.BIT.TIE = 1;				//	割り込みイネーブル
		SCI5.SCR.BIT.TE = 1;				//	送信イネブール
	}
#endif
}
//----------------------------------------------------------------------------------------
//	SCI6	SEND
//----------------------------------------------------------------------------------------
void	sci6_putb(unsigned char *buf, int size)
{
	int			i, ch = 6;
	SCI_MODULE	*com = &sci_com[ch];
	while(size > 0)
	{
#ifdef		SCI6_ACTIVATE
#ifdef	SCI6_FLOW
		if(SCI6_CTS_PORT != 0)
		{	//	CTS=Disable
			if(sci_putcheck(ch) < 2) return;
		}
#endif
		while( sci_putcheck(ch) < 2)
		{	//	バッファが空くまで待つ
			if(SCI6.SCR.BIT.TE == 0)
			{	//	送信開始処理
#ifdef	SCI6_FLOW
				if(SCI6_CTS_PORT == 0)
				{	//	CTS=Enable
#endif
					SCI6.SCR.BIT.TIE = 1;				//	割り込みイネーブル
					SCI6.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI6_FLOW
				}
#endif
			}
		}
#endif
		com->txbuf[com->txwp++] = *buf++;
		size--;
		if(com->txwp >= BUFSIZE) com->txwp = 0;
	#ifdef		SCI6_ACTIVATE
		if(SCI6.SCR.BIT.TE == 0)
		{	//	送信開始処理
	#ifdef	SCI6_FLOW
			if(SCI6_CTS_PORT == 0)
			{	//	CTS=Enable
	#endif
				SCI6.SCR.BIT.TIE = 1;				//	割り込みイネーブル
				SCI6.SCR.BIT.TE = 1;				//	送信イネブール
	#ifdef	SCI6_FLOW
			}
	#endif
		}
	#endif
	}
#ifdef		SCI6_ACTIVATE
	if(SCI6.SCR.BIT.TE == 0)
	{	//	送信開始処理
#ifdef	SCI6_FLOW
		if(SCI6_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			SCI6.SCR.BIT.TIE = 1;				//	割り込みイネーブル
			SCI6.SCR.BIT.TE = 1;				//	送信イネブール
#ifdef	SCI6_FLOW
		}
#endif
	}
#endif
}
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
void sci_puts(int ch, char *str)
{
	int len = 0;
	for(len = 0; str[len] != 0 && len < 256; len++);
	switch(ch)
	{
	case 0:
		sci0_putb((unsigned char *)str, len);
		break;
	case 1:
		sci1_putb((unsigned char *)str, len);
		break;
	case 2:
		sci2_putb((unsigned char *)str, len);
		break;
	case 3:
		sci3_putb((unsigned char *)str, len);
		break;
	case 5:
		sci5_putb((unsigned char *)str, len);
		break;
	case 6:
		sci6_putb((unsigned char *)str, len);
		break;
	}
}

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
void sci_putb(int ch, unsigned char *buf, int len)
{
	switch(ch)
	{
	case 0:
		sci0_putb(buf, len);
		break;
	case 1:
		sci1_putb(buf, len);
		break;
	case 2:
		sci2_putb(buf, len);
		break;
	case 3:
		sci3_putb(buf, len);
		break;
	case 5:
		sci5_putb(buf, len);
		break;
	case 6:
		sci6_putb(buf, len);
		break;
	}
}

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
void sci_putc(int ch, char data)
{
	char	str[2];
	str[0] = data;
	str[1] = 0;
	switch(ch)
	{
	case 0:
		sci0_putb((unsigned char *)str, 1);
		break;
	case 1:
		sci1_putb((unsigned char *)str, 1);
		break;
	case 2:
		sci2_putb((unsigned char *)str, 1);
		break;
	case 3:
		sci3_putb((unsigned char *)str, 1);
		break;
	case 5:
		sci5_putb((unsigned char *)str, 1);
		break;
	case 6:
		sci6_putb((unsigned char *)str, 1);
		break;
	}
}

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
//----------------------------------------------------------------------------------------
#ifdef		SCI0_ACTIVATE
//----------------------------------------------------------------------------------------
//	215
void interrupt __vectno__{VECT_SCI0_TXI0} sci0_txi(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[0];
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
#ifdef	SCI0_TXOSDN
		SCI0_TXOSDN_PORT = 1;		//	0=受信のみ / 1=送信可能
#endif
#ifdef	SCI0_FLOW
		if(SCI0_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			i = com->txrp++;
			if(com->txrp >= BUFSIZE) com->txrp = 0;
			SCI0.TDR = com->txbuf[i];
#ifdef	SCI0_FLOW
		}
		else
		{	//	CTSにより送信停止
			SCI0.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
			SCI0.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
		}
#endif
	}
	else
	{	//	送信バッファが空
		SCI0.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
		SCI0.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI1_ACTIVATE
//----------------------------------------------------------------------------------------
//	218
#ifndef __YIDE_REM_DEBUG__
void interrupt __vectno__{VECT_SCI1_TXI1} sci1_txi(void)
#else
void interrupt sci1_txi(void)
#endif
{
	int	i;
	SCI_MODULE	*com = &sci_com[1];
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
#ifdef	SCI1_FLOW
		if(SCI1_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			i = com->txrp++
			if(com->txrp >= BUFSIZE) com->txrp = 0;
			SCI1.TDR = com->txbuf[i];
#ifdef	SCI1_FLOW
		}
		else
		{	//	CTSにより送信停止
			SCI1.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
			SCI1.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
		}
#endif
	}
	else
	{	//	送信バッファが空
		SCI1.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
		SCI1.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI2_ACTIVATE
//----------------------------------------------------------------------------------------
//	221
void interrupt __vectno__{VECT_SCI2_TXI2} sci2_txi(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[2];
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
#ifdef	SCI2_TXOSDN
		SCI2_TXOSDN_PORT = 1;		//	0=受信のみ / 1=送信可能
#endif
#ifdef	SCI2_FLOW
		if(SCI2_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			i = com->txrp++;
			if(com->txrp >= BUFSIZE) com->txrp = 0;
			SCI2.TDR = com->txbuf[i];
#ifdef	SCI2_FLOW
		}
		else
		{	//	CTSにより送信停止
			SCI2.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
			SCI2.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
		}
#endif
	}
	else
	{	//	送信バッファが空
		SCI2.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
		SCI2.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI3_ACTIVATE
//----------------------------------------------------------------------------------------
//	224
void interrupt __vectno__{VECT_SCI3_TXI3} sci3_txi(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[3];
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
#ifdef	SCI3_FLOW
		if(SCI3_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			i = com->txrp++
			if(com->txrp >= BUFSIZE) com->txrp = 0;
			SCI3.TDR = com->txbuf[i];
#ifdef	SCI3_FLOW
		}
		else
		{	//	CTSにより送信停止
			SCI3.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
			SCI3.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
		}
#endif
	}
	else
	{	//	送信バッファが空
		SCI3.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
		SCI3.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI5_ACTIVATE
//----------------------------------------------------------------------------------------
//	230
void interrupt __vectno__{VECT_SCI5_TXI5} sci5_txi(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[5];
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
#ifdef	SCI5_FLOW
		if(SCI5_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			i = com->txrp++
			if(com->txrp >= BUFSIZE) com->txrp = 0;
			SCI5.TDR = com->txbuf[i];
#ifdef	SCI5_FLOW
		}
		else
		{	//	CTSにより送信停止
			SCI5.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
			SCI5.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
		}
#endif
	}
	else
	{	//	送信バッファが空
		SCI5.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
		SCI5.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI6_ACTIVATE
//----------------------------------------------------------------------------------------
//	233
void interrupt __vectno__{VECT_SCI6_TXI6} sci6_txi(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[6];
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
#ifdef	SCI6_FLOW
		if(SCI6_CTS_PORT == 0)
		{	//	CTS=Enable
#endif
			i = com->txrp++
			if(com->txrp >= BUFSIZE) com->txrp = 0;
			SCI6.TDR = com->txbuf[i];
#ifdef	SCI61_FLOW
		}
		else
		{	//	CTSにより送信停止
			SCI6.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
			SCI6.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
		}
#endif
	}
	else
	{	//	送信バッファが空
		SCI6.SCR.BIT.TIE = 0;		//	TXI割り込み禁止
		SCI6.SCR.BIT.TEIE = 1;		//	TEI割り込み許可
	}
}
#endif

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
//----------------------------------------------------------------------------------------
#ifdef		SCI0_ACTIVATE
//----------------------------------------------------------------------------------------
//	216
void interrupt __vectno__{VECT_SCI0_TEI0} sci0_tei(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[0];
	SCI0.SCR.BIT.TEIE = 0;		//	TEI割り込みディスエイブル
#ifdef	SCI0_TXOSDN
	SCI0_TXOSDN_PORT = 0;		//	0=nRE(受信) / 1=DE(送信)
#endif
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
		SCI0.SCR.BIT.TIE = 1;		//	送信動作イネーブル
		i = com->txrp++;
		if(com->txrp >= BUFSIZE) com->txrp = 0;
		SCI0.TDR = com->txbuf[i];
	}
	else
	{
		SCI0.SCR.BIT.TE = 0;		//	送信動作ディスエイブル
	}
//	logging("sci0_tei\r");
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI1_ACTIVATE
//----------------------------------------------------------------------------------------
//	219
#ifndef __YIDE_REM_DEBUG__
void interrupt __vectno__{VECT_SCI1_TEI1} sci1_tei(void)
#else
void interrupt sci1_tei(void)
#endif
{
	int	i;
	SCI_MODULE	*com = &sci_com[1];
	SCI1.SCR.BIT.TEIE = 0;		//	TEI割り込みディスエイブル
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
		SCI1.SCR.BIT.TIE = 1;		//	送信動作イネーブル
		i = com->txrp++
		if(com->txrp >= BUFSIZE) com->txrp = 0;
		SCI1.TDR = com->txbuf[i];
	}
	else
	{
		SCI1.SCR.BIT.TE = 0;		//	送信動作ディスエイブル
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI2_ACTIVATE
//----------------------------------------------------------------------------------------
//	222
void interrupt __vectno__{VECT_SCI2_TEI2} sci2_tei(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[2];
	SCI2.SCR.BIT.TEIE = 0;		//	TEI割り込みディスエイブル
#ifdef	SCI2_TXOSDN
	SCI2_TXOSDN_PORT = 0;		//	0=nRE(受信) / 1=DE(送信)
#endif
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
		SCI2.SCR.BIT.TIE = 1;		//	送信動作イネーブル
		i = com->txrp++;
		if(com->txrp >= BUFSIZE) com->txrp = 0;
		SCI2.TDR = com->txbuf[i];
	}
	else
	{
		SCI2.SCR.BIT.TE = 0;		//	送信動作ディスエイブル
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI3_ACTIVATE
//----------------------------------------------------------------------------------------
//	225
void interrupt __vectno__{VECT_SCI3_TEI3} sci3_tei(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[3];
	SCI3.SCR.BIT.TEIE = 0;		//	TEI割り込みディスエイブル
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
		SCI3.SCR.BIT.TIE = 1;		//	送信動作イネーブル
		i = com->txrp++
		if(com->txrp >= BUFSIZE) com->txrp = 0;
		SCI3.TDR = com->txbuf[i];
	}
	else
	{
		SCI3.SCR.BIT.TE = 0;		//	送信動作ディスエイブル
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI5_ACTIVATE
//----------------------------------------------------------------------------------------
//	231
void interrupt __vectno__{VECT_SCI5_TEI5} sci5_tei(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[5];
	SCI5.SCR.BIT.TEIE = 0;		//	TEI割り込みディスエイブル
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
		SCI5.SCR.BIT.TIE = 1;		//	送信動作イネーブル
		i = com->txrp++
		if(com->txrp >= BUFSIZE) com->txrp = 0;
		SCI5.TDR = com->txbuf[i];
	}
	else
	{
		SCI5.SCR.BIT.TE = 0;		//	送信動作ディスエイブル
	}
}
#endif
//----------------------------------------------------------------------------------------
#ifdef		SCI6_ACTIVATE
//----------------------------------------------------------------------------------------
//	234
void interrupt __vectno__{VECT_SCI6_TEI6} sci6_tei(void)
{
	int	i;
	SCI_MODULE	*com = &sci_com[6];
	SCI6.SCR.BIT.TEIE = 0;		//	TEI割り込みディスエイブル
	if( com->txrp != com->txwp )
	{	//	バッファにデータが残っている
		SCI6.SCR.BIT.TIE = 1;		//	送信動作イネーブル
		i = com->txrp++
		if(com->txrp >= BUFSIZE) com->txrp = 0;
		SCI6.TDR = com->txbuf[i];
	}
	else
	{
		SCI6.SCR.BIT.TE = 0;		//	送信動作ディスエイブル
	}
}
#endif

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
void	sci_load(int ch, unsigned char err, unsigned char data)
{
	int	sz;
	SCI_MODULE	*com = &sci_com[ch];
	if(err != 0)
	{
		com->err++;
		if((err & 0x08) != 0)
		{	//	パリティエラー
			com->perr++;
		}
		if((err & 0x10) != 0)
		{	//	フレーミングエラー
			com->ferr++;
		}
		if((err & 0x20) != 0)
		{	//	オーバーランエラー
			com->oerr++;
		}
	}
	//	データ保存
	com->rxbuf[com->rxwp++] = data;
	if(com->rxwp >= BUFSIZE) com->rxwp = 0; //	ポインタを0に戻す
	//	RTSフロー制御
	switch(ch)
	{
#ifdef		SCI0_ACTIVATE
#ifdef	SCI0_FLOW
	case 0:	//	COM0専用フロー制御
		sz = com->rxrp - com->rxwp;
		if(sz < 0) sz += BUFSIZE;
		if(sz < (BUFSIZE * 3 / 4))
		{	//	RTS=Disable
			SCI0_RTS_PORT = 1;
		}
		break;
#endif
#endif
#ifdef		SCI1_ACTIVATE
#ifdef	SCI1_FLOW
	case 1:	//	COM1専用フロー制御
		sz = com->rxrp - com->rxwp;
		if(sz < 0) sz += BUFSIZE;
		if(sz < (BUFSIZE * 3 / 4))
		{	//	RTS=Disable
			SCI1_RTS_PORT = 1;
		}
		break;
#endif
#endif
#ifdef		SCI2_ACTIVATE
#ifdef	SCI2_FLOW
	case 2:	//	COM2専用フロー制御
		sz = com->rxrp - com->rxwp;
		if(sz < 0) sz += BUFSIZE;
		if(sz < (BUFSIZE * 3 / 4))
		{	//	RTS=Disable
			SCI2_RTS_PORT = 1;
		}
		break;
#endif
#endif
#ifdef		SCI3_ACTIVATE
#ifdef	SCI3_FLOW
	case 3:	//	COM3専用フロー制御
		sz = com->rxrp - com->rxwp;
		if(sz < 0) sz += BUFSIZE;
		if(sz < (BUFSIZE * 3 / 4))
		{	//	RTS=Disable
			SCI3_RTS_PORT = 1;
		}
		break;
#endif
#endif
#ifdef		SCI6_ACTIVATE
#ifdef	SCI6_FLOW
	case 6:	//	COM6専用フロー制御
		sz = com->rxrp - com->rxwp;
		if(sz < 0) sz += BUFSIZE;
		if(sz < (BUFSIZE * 3 / 4))
		{	//	RTS=Disable
			SCI6_RTS_PORT = 1;
		}
		break;
#endif
#endif
	default:
		break;
	}
}

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
//		割り込みベクタ(VECT_ICU_GROUPL0) 114
//________________________________________________________________________________________
//
void interrupt __vectno__{VECT_ICU_GROUPL0} sci_err(void)
{
	unsigned char	d, e;
	PROC_CALL		p;
	
#ifdef		SCI0_ACTIVATE
	//	SCI0
	if(ICU.GRP[GRP_SCI0_ERI0].BIT.IS_SCI0_ERI0)
	{	//	SCI0 Error有り
		e = SCI0.SSR.BYTE;
		d = SCI0.RDR & 0x00FF;
		sci_load(0, e, d);
		SCI0.SSR.BYTE = 0;
	}
#endif
#ifdef		SCI1_ACTIVATE
	//	SCI1
	if(ICU.GRP[GRP_SCI1_ERI1].BIT.IS_SCI1_ERI1)
	{	//	SCI1 Error有り
		e = SCI1.SSR.BYTE;
		d = SCI1.RDR & 0x00FF;
		sci_load(1, e, d);
		SCI1.SSR.BYTE = 0;
	}
#endif
#ifdef		SCI2_ACTIVATE
	//	SCI2
	if(ICU.GRP[GRP_SCI2_ERI2].BIT.IS_SCI2_ERI2)
	{	//	SCI2 Error有り
		e = SCI2.SSR.BYTE;
		d = SCI2.RDR & 0x00FF;
		sci_load(2, e, d);
		SCI2.SSR.BYTE = 0;
	}
#endif
#ifdef		SCI3_ACTIVATE
	//	SCI3
	if(ICU.GRP[GRP_SCI3_ERI3].BIT.IS_SCI3_ERI3)
	{	//	SCI3 Error有り
		e = SCI3.SSR.BYTE;
		d = SCI3.RDR & 0x00FF;
		sci_load(3, e, d);
		SCI3.SSR.BYTE = 0;
	}
#endif
#ifdef		SCI5_ACTIVATE
	//	SCI5
	if(ICU.GRP[GRP_SCI5_ERI5].BIT.IS_SCI5_ERI5)
	{	//	SCI5 Error有り
		e = SCI5.SSR.BYTE;
		d = SCI5.RDR & 0x00FF;
		sci_load(5, e, d);
		SCI5.SSR.BYTE = 0;
	}
#endif
#ifdef		SCI6_ACTIVATE
	//	SCI6
	if(ICU.GRP[GRP_SCI6_ERI6].BIT.IS_SCI6_ERI6)
	{	//	SCI6 Error有り
		e = SCI6.SSR.BYTE;
		d = SCI6.RDR & 0x00FF;
		sci_load(6, e, d);
		SCI6.SSR.BYTE = 0;
	}
#endif
#ifdef		RSPI1_ACTIVATE
	//	RSPI1
	if(ICU.GRP[GRP_RSPI1_SPEI1].BIT.IS_RSPI1_SPEI1)
	{	//	RSPI1 Error有り
		e = (unsigned short)RSPI1.SPSR.BYTE;
		RSPI1.SPSR.BYTE = 0;
		logging("SPI1 Error %02X\r", (int)e);
	}
#endif
#if	1	/*#ifdef	RSPI2_ACTIVATE*/
	//	RSPI2
	if(ICU.GRP[GRP_RSPI2_SPEI2].BIT.IS_RSPI2_SPEI2)
	{	//	RSPI2 Error有り
		e = (unsigned short)RSPI2.SPSR.BYTE;
		RSPI2.SPSR.BYTE = 0;
		logging("SPI2 Error %02X\r", (int)e);
	}
#endif
}

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
#ifdef		SCI0_ACTIVATE
//---------------------------------------------------------------------------------------
//	214
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_SCI0_RXI0} sci0_rxi(void)
{
	sci_load(0, 0, SCI0.RDR);
}
#endif
#ifdef		SCI1_ACTIVATE
//---------------------------------------------------------------------------------------
//	217
//---------------------------------------------------------------------------------------
#ifndef __YIDE_REM_DEBUG__
void interrupt __vectno__{VECT_SCI1_RXI1} sci1_rxi(void)
#else
void interrupt sci1_rxi(void)
#endif
{
	sci_load(1, 0, SCI1.RDR);
}
#endif
#ifdef		SCI2_ACTIVATE
//---------------------------------------------------------------------------------------
//	220
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_SCI2_RXI2} sci2_rxi(void)
{
	sci_load(2, 0, SCI2.RDR);
}
#endif
#ifdef		SCI3_ACTIVATE
//---------------------------------------------------------------------------------------
//	223
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_SCI3_RXI3} sci3_rxi(void)
{
	sci_load(3, 0, SCI3.RDR);
}
#endif
#ifdef		SCI5_ACTIVATE
//---------------------------------------------------------------------------------------
//	229
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_SCI5_RXI5} sci5_rxi(void)
{
	sci_load(5, 0, SCI5.RDR);
}
#endif
#ifdef		SCI6_ACTIVATE
//---------------------------------------------------------------------------------------
//	232
//---------------------------------------------------------------------------------------
void interrupt __vectno__{VECT_SCI6_RXI6} sci6_rxi(void)
{
	sci_load(6, 0, SCI6.RDR);
}
#endif
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
int sci_get_check(int ch)
{
	SCI_MODULE	*com = &sci_com[ch];
	int sz = (com->rxwp - com->rxrp);
	if(sz < 0) sz += BUFSIZE;
	return sz;
}

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
void sci_rts_control(int ch)
{
	//	RTSフロー制御
	if(sci_get_check(ch) > (BUFSIZE / 4)) return;
	switch(ch)
	{
#ifdef		SCI0_ACTIVATE
#ifdef	SCI0_FLOW
	case 0:	//	COM0専用フロー制御
		SCI0_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI1_ACTIVATE
#ifdef	SCI1_FLOW
	case 1:	//	COM1専用フロー制御
		SCI1_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI2_ACTIVATE
#ifdef	SCI2_FLOW
	case 2:	//	COM2専用フロー制御
		SCI2_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI3_ACTIVATE
#ifdef	SCI3_FLOW
	case 3:	//	COM3専用フロー制御
		SCI3_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI6_ACTIVATE
#ifdef	SCI6_FLOW
	case 6:	//	COM6専用フロー制御
		SCI6_RTS_PORT = 0;
		break;
#endif
#endif
	default:
		break;
	}
}

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
int sci_get_char(int ch)
{
	int data;
	SCI_MODULE	*com = &sci_com[ch];
	if(com->rxrp != com->rxwp)
	{
		data = (int)com->rxbuf[com->rxrp++] & 0x00FF;
		if(com->rxrp >= BUFSIZE) com->rxrp = 0;
		sci_rts_control(ch);
	}
	else
	{
		data = -1;
	}
	return data;
}

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
int sci_get_buf(int ch, unsigned char *buf, int size)
{
	int i;
	SCI_MODULE	*com = &sci_com[ch];
	for(i = 0; i < size; i++)
	{
		if(com->rxrp != com->rxwp)
		{
			*buf++ = com->rxbuf[com->rxrp++];
			if(com->rxrp >= BUFSIZE) com->rxrp = 0;
		}
		else break;
	}
	sci_rts_control(ch);
	return i;
}

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
int sci_get_string(int ch, char *str, int size)
{
	int		i;
	char	c;
	SCI_MODULE	*com = &sci_com[ch];
	for(i = 0; i < size; i++)
	{
		if(com->rxrp != com->rxwp)
		{
			c = (char)com->rxbuf[com->rxrp++];
			if(com->rxrp >= BUFSIZE) com->rxrp = 0;
			*str++ = c;
			if(c == 0) return i;	//	文字列の終端
		}
		else break;
	}
	*str = 0;	//	NUL付加
	sci_rts_control(ch);
	return i;
}

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
int sci_get_line(int ch, char *str, int size, char echar)
{
	int		i, e;
	char	c;
	SCI_MODULE	*com = &sci_com[ch];
	
	for(e = 0, i = 0; i < size; )
	{
		if(com->rxrp != com->rxwp)
		{
			c = (char)com->rxbuf[com->rxrp++];
			if(com->rxrp >= BUFSIZE) com->rxrp = 0;
			if(c == echar)
			{	//	行末コードで終了
				e++;
				break;
			}
			else
			{
				str[i++] = c;
			}
		}
		else break;
	}
	str[i] = 0; //	NUL付加
	sci_rts_control(ch);
	if(e == 0) return -i;
	return i;
}

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
void	sci_clear(int ch)
{
	SCI_MODULE	*com = &sci_com[ch];
	memset(com, 0, sizeof(SCI_MODULE));
	switch(ch)
	{
#ifdef		SCI0_ACTIVATE
#ifdef	SCI0_FLOW
	case 0:	//	COM0専用フロー制御
		SCI0_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI1_ACTIVATE
#ifdef	SCI1_FLOW
	case 1:	//	COM1専用フロー制御
		SCI1_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI2_ACTIVATE
#ifdef	SCI2_FLOW
	case 2:	//	COM2専用フロー制御
		SCI2_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI3_ACTIVATE
#ifdef	SCI3_FLOW
	case 3:	//	COM3専用フロー制御
		SCI3_RTS_PORT = 0;
		break;
#endif
#endif
#ifdef		SCI6_ACTIVATE
#ifdef	SCI6_FLOW
	case 6:	//	COM6専用フロー制御
		SCI6_RTS_PORT = 0;
		break;
#endif
#endif
	default:
		break;
	}
}


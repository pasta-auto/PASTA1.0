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
// $RCSfile: StartupLFY63N.c,v $
// $Revision: 1.3 $
// $Date: 2016/03/14 01:17:29 $
// 
//	YellowIDE7対応
//	スタートアップルーチン CSC63N.C
//
//	LFY-RX63N1用
//
// Copyright (c) 2015 LandF Corporation.
//
//	ユーザが変更する所は全部で3か所あります。
//	それぞれ<<変更①>>～<<変更③>>で示してあります。
//
//--------------------------------------------------------------------
//=============================ヘッダ部==============================
#include "YIDESYM.H"		// STACK_SIZE HEAP_SIZE STRUCT_SIZEの読み込

//変更①
#define	RAM_BASE		0x0000000		//RAMの先頭番地を記述してください。
#define	CLOCK			96000000L		//CPUクロック Hz単位	*/
#define	PCLOCK			48000000L		//内臓周辺クロック Hz単位*/
#define	MAX_VECTNO		256				//最大割込みベクタ番号+1を記述してください。ハードウェアマニュアル参照
										//分からない場合は256
#define	DBG_PORT		SCI1			//デバッグに使うシリアルチャンネルの設定
//#define	DBG_PORT		USB_R			//デバッグに使うシリアルチャンネルの設定
#define	DBG_PORT_BPS	38400L			//デバックポートでのボーレート
#define	PIN_SIZE		144				//パッケージのピン数144(145ピンを含む),100,64,48 あるいは 177

//	SDRAM未使用
#define	_SDRAM_NOUSE_

/*
	ＵＳＢ　シリアル番号定義
	
	１つのＰＣに複数のボードを接続する場合、同じシリアル番号では
	同じポートに割り当てられるため、正常に動作しません。従って、下記のシリアル番号
	の値を変更してください。
*/

#define	USB_SERIAL_NO	0

//変更①終わり

#include <csheader.h>

/*
	簡易メモリーマップ	CPU R5F563NEDDFB
	(内蔵ROM有効拡張モード)
		0x00000000	- 0x0001FFFF	内蔵SRAM(128KB)
		0x00080000  - 0x000FFFFF	I/O
		0x08000000  - 0x09FFFFFF	SDRAM領域
		0xFFE00000  - 0xFFFFFFFF	内蔵ROM(2048KB)
*/

void					 __CSTARTUP_INIT2__(void);
extern long				 _STACK_TOP[];
extern long				 _USTACK_TOP[];
static unsigned long	 temp;
static unsigned char	*p1, *p2;

/* モニターをＲＡＭ上で動作させるマクロ（修正しない）	*/
#ifdef	__TEXT_RAM__
extern unsigned char TEXT_RAM_rom, TEXT_RAM_start, DATA_CONST_RAM_rom, DATA_CONST_RAM_start ;
extern unsigned long TEXT_RAM_size, DATA_CONST_RAM_size ;
#endif	/* __TEXT_RAM__ */

#define	_B1(a)	(1<<(a))
#define	_B0(a)	(0)
#define	_B(a,b)	((a)<<(b))

#define	BIT_4	_B1(4)
#define	BIT_6	_B1(6)
#define	BIT_7	_B1(7)
//===================================================================
//	オプション設定メモリ
//===================================================================
/*
	オプション設定メモリは、リセット後のマイコンの状態を選択するレジスタです。
	レジスタはＲＯＭ上にあり、フラッシュＲＯＭ書き込み時に決定されます。
	下記のマクロでOFS0,OFS1,MDESを設定してください。ただし、MDESは自動で設定されます。
*/
/*****************************************
   オプション機能選択レジスタ0（OFS0）
*****************************************/

// リセット後、IWDT動作
#define	OFS0_IWDTSTART_AUTO		_B0(1)		// IWDTはオートスタートモードにて自動的に起動
#define	OFS0_IWDTSTART_STOP		_B1(1)		// IWDTは停止状態
// IWDTタイムアウト期間選択ビット
#define	OFS0_IWDTTOPS_1024		_B(0,2)		// 1024サイクル
#define	OFS0_IWDTTOPS_4096		_B(1,2)		// 4096サイクル
#define	OFS0_IWDTTOPS_8192		_B(2,2)		// 8192サイクル
#define	OFS0_IWDTTOPS_16384		_B(3,2)		// 16384サイクル
// IWDTクロック分周比選択ビット
#define	OFS0_IWDTCKS_1			_B(0,4)		// 1分周（周期 131ms）
#define	OFS0_IWDTCKS_16			_B(2,4)		// 16分周（周期 2.10s）
#define	OFS0_IWDTCKS_32			_B(3,4)		// 32分周（周期 4.19s）
#define	OFS0_IWDTCKS_64			_B(4,4)		// 64分周（周期 8.39s）
#define	OFS0_IWDTCKS_128		_B(15,4)	// 128分周（周期 16.8s）
#define	OFS0_IWDTCKS_256		_B(5,4)		// 256分周（周期 33.6s）
// IWDTウィンドウ終了位置選択ビット
#define	OFS0_IWDTRPES_75		_B(0,8)		// 75%
#define	OFS0_IWDTRPES_50		_B(1,8)		// 50%
#define	OFS0_IWDTRPES_25		_B(2,8)		// 25%
#define	OFS0_IWDTRPES_0			_B(3,8)		// 0%（ウィンドウの終了位置設定なし）
// IWDTウィンドウ開始位置選択ビット
#define	OFS0_IWDTRPSS_25		_B(0,10)	// 25%
#define	OFS0_IWDTRPSS_50		_B(1,10)	// 50%
#define	OFS0_IWDTRPSS_75		_B(2,10)	// 75%
#define	OFS0_IWDTRPSS_100		_B(3,10)	// 100%（ウィンドウの開始位置設定なし）
// IWDTリセット割り込み要求選択
#define	OFS0_IWDTRSTIRQS_NMI	_B0(12)		// ノンマスカブル割り込み要求を許可
#define	OFS0_IWDTRSTIRQS_RESET	_B1(12)		// リセットを許可
// IWDTスリープモードカウント停止制御ビット
#define	OFS0_IWDTSLCSTP_DIS		_B0(14)		// カウント停止無効
#define	OFS0_IWDTSLCSTP_SLEEP	_B1(14)		// スリープモード、ソフトウェアスタンバイモード、
											// ディープソフトウェアスタンバイモード、
											// および全モジュールクロックストップモード移行時のカウント停止有効
// WDTスタートモード選択ビット
#define OFS0_WDTSTRT_AUTO		_B0(17)		// リセット後、WDTはオートスタートモードにて自動的に起動
#define OFS0_WDTSTRT_STOP		_B1(17)		// リセット後、WDTは停止状態

// WDTタイムアウト期間選択ビット
#define	OFS0_WDTTOPS_1024		_B(0,18)	// 1024サイクル
#define	OFS0_WDTTOPS_4096		_B(1,18)	// 4096サイクル
#define	OFS0_WDTTOPS_8192		_B(2,18)	// 8192サイクル
#define	OFS0_WDTTOPS_16384		_B(3,18)	// 16384サイクル
// WDTクロック分周比選択ビット
#define	OFS0_WDTCKS_4			_B(1,20)	// 4分周
#define	OFS0_WDTCKS_64			_B(4,20)	// 64分周
#define	OFS0_WDTCKS_128			_B(15,20)	// 128分周
#define	OFS0_WDTCKS_512			_B(6,20)	// 512分周
#define	OFS0_WDTCKS_2048		_B(7,20)	// 2048分周
#define	OFS0_WDTCKS_8192		_B(8,20)	// 8192分周
// WDTウィンドウ終了位置選択ビット
#define	OFS0_WDTRPES_75			_B(0,24)	// 75%
#define	OFS0_WDTRPES_50			_B(1,24)	// 50%
#define	OFS0_WDTRPES_25			_B(2,24)	// 25%
#define	OFS0_WDTRPES_0			_B(3,24)	// 0%（ウィンドウの終了位置設定なし）
// IWDTウィンドウ開始位置選択ビット
#define	OFS0_WDTRPSS_25			_B(0,26)	// 25%
#define	OFS0_WDTRPSS_50			_B(1,26)	// 50%
#define	OFS0_WDTRPSS_75			_B(2,26)	// 75%
#define	OFS0_WDTRPSS_100		_B(3,26)	// 100%（ウィンドウの開始位置設定なし）
// IWDTリセット割り込み要求選択
#define	OFS0_WDTRSTIRQS_NMI		_B0(28)		// ノンマスカブル割り込み要求を許可
#define	OFS0_WDTRSTIRQS_RESET	_B1(28)		// リセットを許可

#define	OFS0_DEF				0xe001a001

/* OFS0 設定値	*/
#define	OFS0					( OFS0_DEF 					| \
								  OFS0_IWDTSTART_STOP 		| \
								  OFS0_IWDTTOPS_16384 		| \
							      OFS0_IWDTCKS_128			| \
								  OFS0_IWDTRPES_0			| \
								  OFS0_IWDTRPSS_100			| \
								  OFS0_IWDTRSTIRQS_RESET	| \
								  OFS0_IWDTSLCSTP_SLEEP		| \
								  OFS0_WDTSTRT_STOP			| \
								  OFS0_WDTTOPS_16384 		| \
							      OFS0_WDTCKS_128			| \
								  OFS0_WDTRPES_0			| \
								  OFS0_WDTRPSS_100			| \
								  OFS0_WDTRSTIRQS_RESET	)

/*****************************************
   オプション機能選択レジスタ1（OFS1）
*****************************************/

// 電圧検出0回路起動ビット
#define	OFS1_LVDAS_VALID		_B0(2)		// リセット後、電圧監視0リセット有効
#define	OFS1_LVDAS_INVALID		_B1(2)		// リセット後、電圧監視0リセット無効
// HOCO発振有効ビット
#define	OFS1_HOCOEN_VALID		_B0(8)		// リセット後、HOCO発振が有効
#define	OFS1_HOCOEN_INVALID		_B1(8)		// リセット後、HOCO発振が無効

#define	OFS1_DEF				0xfffffefb

/* OFS1 設定値	*/
#define	OFS1					( OFS1_DEF | OFS1_LVDAS_INVALID | OFS1_HOCOEN_INVALID )

/*****************************************
  エンディアン選択レジスタS（MDES）
*****************************************/
/*　コンパイラー定義マクロにより自動設定	*/
#define	MDES_BIG				0xfffffff8
#define	MDES_LTTTLE				0xffffffff
/* MDES 設定値	*/
#ifdef	__LITTLE_ENDIAN__
#define	MDES					MDES_LTTTLE
#else
#define	MDES					MDES_BIG
#endif	/* __LITTLE_ENDIAN__ */

//===================================================================
//	Ｉ／Ｏ定義
//===================================================================
//システムクロックコントロールレジスタ
#define	SCKCR				(*((volatile unsigned long *)0x80020))	
#define	SCKCR_MASK			0x00000011
#define	SCKCR_PCKB1			_B(0,8)		// 周辺モジュールクロックB  1分周
#define	SCKCR_PCKB2			_B(1,8)		//                          2分周
#define	SCKCR_PCKB4			_B(2,8)		//                          4分周
#define	SCKCR_PCKB8			_B(3,8)		//                          8分周
#define	SCKCR_PCKB16		_B(4,8)		//                         16分周
#define	SCKCR_PCKB32		_B(5,8)		//                         32分周
#define	SCKCR_PCKB64		_B(6,8)		//                         64分周
#define	SCKCR_PCKA1			_B(0,12)	// 周辺モジュールクロックA  1分周
#define	SCKCR_PCKA2			_B(1,12)	//                          2分周
#define	SCKCR_PCKA4			_B(2,12)	//                          4分周
#define	SCKCR_PCKA8			_B(3,12)	//                          8分周
#define	SCKCR_PCKA16		_B(4,12)	//                         16分周
#define	SCKCR_PCKA32		_B(5,12)	//                         32分周
#define	SCKCR_PCKA64		_B(6,12)	//                         64分周
#define	SCKCR_BCK1			_B(0,16)	// 外部バスクロック         1分周
#define	SCKCR_BCK2			_B(1,16)	//                          2分周
#define	SCKCR_BCK4			_B(2,16)	//                          4分周
#define	SCKCR_BCK8			_B(3,16)	//                          8分周
#define	SCKCR_BCK16			_B(4,16)	//                         16分周
#define	SCKCR_BCK32			_B(5,16)	//                         32分周
#define	SCKCR_BCK64			_B(6,16)	//                         64分周

#define	PSTOP0_BIT			_B1(22)		// SDCLK端子出力制御ビット 0:動作 1:停止
#define	PSTOP1_BIT			_B1(23)		// BCLK端子出力制御ビット 0:動作 1:停止
#define	SCKCR_ICK1			_B(0,24)	// システムクロック         1分周
#define	SCKCR_ICK2			_B(1,24)	//                          2分周
#define	SCKCR_ICK4			_B(2,24)	//                          4分周
#define	SCKCR_ICK8			_B(3,24)	//                          8分周
#define	SCKCR_ICK16			_B(4,24)	//                         16分周
#define	SCKCR_ICK32			_B(5,24)	//                         32分周
#define	SCKCR_ICK64			_B(6,24)	//                         64分周
#define	SCKCR_FCK1			_B(0,28)	// FlashIFクロック          1分周
#define	SCKCR_FCK2			_B(1,28)	//                          2分周
#define	SCKCR_FCK4			_B(2,28)	//                          4分周
#define	SCKCR_FCK8			_B(3,28)	//                          8分周
#define	SCKCR_FCK16			_B(4,28)	//                         16分周
#define	SCKCR_FCK32			_B(5,28)	//                         32分周
#define	SCKCR_FCK64			_B(6,28)	//                         64分周

// 内部クロック分周比を設定
// b31:b28      FCK         FlashIFクロック          - 4分周
// b27:b24      ICK         システムクロック         - 2分周
// b23          PSTOP1      BCLK出力制御             - 停止
// b22          PSTOP0      SDCLK出力制御            - 停止
// b21:b20      Reserved    予約
// b19:b16      BCK         外部バスクロック         - 4分周
// b15:b12      PCLKA       周辺モジュールクロックA  - 2分周
// b11:b8       PCLKB       周辺モジュールクロックB  - 4分周
#define SCKCR_VAL			( SCKCR_MASK | SCKCR_FCK4 | SCKCR_ICK2 | SCKCR_PCKB4 | SCKCR_PCKA2 | SCKCR_BCK4 | PSTOP1_BIT | PSTOP0_BIT )

//システムクロックコントロールレジスタ2
#define	SCKCR2				(*((volatile unsigned short *)0x80024))

//システムクロックコントロールレジスタ3
#define	SCKCR3				(*((volatile unsigned short *)0x80026))
#define	SCKCR3_LOCO			_B(0,8)		// クロックソース選択ビット　LOCO選択
#define	SCKCR3_HOCO			_B(1,8)		//                           HOCO選択
#define	SCKCR3_MAIN			_B(2,8)		//                           メインクロック選択
#define	SCKCR3_SUB			_B(3,8)		//                           サブクロック選択
#define	SCKCR3_PLL			_B(4,8)		//                           PLL回路選択

// 外部バスコントロールレジスタ
#define	BCKCR				(*((volatile unsigned char *)0x80030))

// 高速オンチップオシレータコントロールレジスタ
#define	HOCOCR				(*((volatile unsigned char *)0x80036))
#define	HCSTP_BIT			_B1(0)

// 高速オンチップオシレータ電源コントロールレジスタ
#define	HOCOPCR				(*((volatile unsigned char *)0x8C294))

// システムコントロールレジスタ０
#define	SYSCR0				(*((volatile unsigned short *)0x80006))
#define	SYSCR0_ROME			_B1(0)		// 内蔵ROM許可ビット
#define	SYSCR0_EXBE			_B1(1)		// 外部バス許可ビット
#define	SYSCR0_KEY			_B(0x5A,8)	// SYSCR0キーコード
#define	SYSCR1_RAME			_B1(0)		// 内蔵RAM許可ビット

// メインクロック発振器ウェイトコントロール レジスタ 
#define	MOSCWTCR			(*((volatile unsigned char *)0x800A2))

// メインクロック発振器コントロール レジスタ 
#define	MOSCCR				(*((volatile unsigned char *)0x80032))

//	PLL コントロールレジスタ（PLLCR）
#define	PLLCR				(*((volatile unsigned short *)0x80028))
// PLL入力分周比選択ビット
#define	PLLCR_PLIDIV_1		_B(0,0)		// 1分周
#define	PLLCR_PLIDIV_2		_B(1,0)		// 2分周
#define	PLLCR_PLIDIV_4		_B(2,0)		// 4分周
// 周波数逓倍率設定ビット
#define	PLLCR_STC_8			_B(7,8)		// ×8
#define	PLLCR_STC_10		_B(9,8)		// ×10
#define	PLLCR_STC_12		_B(11,8)	// ×12
#define	PLLCR_STC_16		_B(15,8)	// ×16
#define	PLLCR_STC_20		_B(19,8)	// ×20
#define	PLLCR_STC_24		_B(23,8)	// ×24
#define	PLLCR_STC_25		_B(24,8)	// ×25
#define	PLLCR_STC_50		_B(49,8)	// ×50

//	PLL コントロールレジスタ2（PLLCR2）
#define	PLLCR2				(*((volatile unsigned char *)0x8002a))		

//	PLL ウェイトコントロールレジスタ (PLLWTCR)
#define	PLLWTCR				(*((volatile unsigned char *)0x800A6))
#define	PLLWTCR_16_CYL		_B(0,0)		// 16サイクル
#define	PLLWTCR_32_CYL		_B(1,0)		// 32サイクル
#define	PLLWTCR_64_CYL		_B(2,0)		// 64サイクル
#define	PLLWTCR_512_CYL		_B(3,0)		// 512サイクル
#define	PLLWTCR_1024_CYL	_B(4,0)		// 1024サイクル
#define	PLLWTCR_2048_CYL	_B(5,0)		// 2048サイクル
#define	PLLWTCR_4096_CYL	_B(6,0)		// 4096サイクル
#define	PLLWTCR_16384_CYL	_B(7,0)		// 16384サイクル
#define	PLLWTCR_32768_CYL	_B(8,0)		// 32768サイクル
#define	PLLWTCR_65536_CYL	_B(9,0)		// 65536サイクル
#define	PLLWTCR_131072_CYL	_B(10,0)	// 131072サイクル
#define	PLLWTCR_262144_CYL	_B(11,0)	// 262144サイクル
#define	PLLWTCR_524288_CYL	_B(12,0)	// 524288サイクル
#define	PLLWTCR_1048576_CYL	_B(13,0)	// 1048576サイクル
#define	PLLWTCR_2097152_CYL	_B(14,0)	// 2097152サイクル
#define	PLLWTCR_4194304_CYL	_B(15,0)	// 4194304サイクル

// ポート方向レジスタ(PDR)
#define	P0DR				(*((volatile unsigned char *)0x8c000))			// PORT0 ポート方向レジスタ
#define	P1DR				(*((volatile unsigned char *)0x8c001))			// PORT1 ポート方向レジスタ
#define	P2DR				(*((volatile unsigned char *)0x8c002))			// PORT2 ポート方向レジスタ
#define	P3DR				(*((volatile unsigned char *)0x8c003))			// PORT3 ポート方向レジスタ
#define	P4DR				(*((volatile unsigned char *)0x8c004))			// PORT4 ポート方向レジスタ
#define	P5DR				(*((volatile unsigned char *)0x8c005))			// PORT5 ポート方向レジスタ
#define	P6DR				(*((volatile unsigned char *)0x8c006))			// PORT6 ポート方向レジスタ
#define	P7DR				(*((volatile unsigned char *)0x8c007))			// PORT7 ポート方向レジスタ
#define	P8DR				(*((volatile unsigned char *)0x8c008))			// PORT8 ポート方向レジスタ
#define	P9DR				(*((volatile unsigned char *)0x8c009))			// PORT9 ポート方向レジスタ
#define	PADR				(*((volatile unsigned char *)0x8c00A))			// PORTA ポート方向レジスタ
#define	PBDR				(*((volatile unsigned char *)0x8c00B))			// PORTB ポート方向レジスタ
#define	PCDR				(*((volatile unsigned char *)0x8c00C))			// PORTC ポート方向レジスタ
#define	PDDR				(*((volatile unsigned char *)0x8c00D))			// PORTD ポート方向レジスタ
#define	PEDR				(*((volatile unsigned char *)0x8c00E))			// PORTE ポート方向レジスタ
#define	PFDR				(*((volatile unsigned char *)0x8c00F))			// PORTF ポート方向レジスタ
#define	PGDR				(*((volatile unsigned char *)0x8c010))			// PORTG ポート方向レジスタ
#define	PJDR				(*((volatile unsigned char *)0x8c012))			// PORTJ ポート方向レジスタ

// ポート出力データレジスタ(PODR)
#define P0ODR				(*((volatile unsigned char *)0x8c020))
#define P1ODR				(*((volatile unsigned char *)0x8c021))
#define P2ODR				(*((volatile unsigned char *)0x8c022))

// ポート入力レジスタ(PIDR)
#define	P0IDR				(*((volatile unsigned char *)0x8c040))
#define	PFIDR				(*((volatile unsigned char *)0x8c04f))
#define	PCIDR				(*((volatile unsigned char *)0x8c04c))
#define	PJIDR				(*((volatile unsigned char *)0x8c052))

// ポートモードレジスタ (PMR)
#define P0MR				(*((volatile unsigned char *)0x8c060))
#define P1MR				(*((volatile unsigned char *)0x8c061))
#define P2MR				(*((volatile unsigned char *)0x8c062))
#define P3MR				(*((volatile unsigned char *)0x8c063))

#if PIN_SIZE == 144
#define NE_P1PDR			0x03		// 存在しないピン: P10, P11
#define NE_P5PDR			0x80		// 存在しないピン: P57
#define	NE_P8PDR			0x30		// 存在しないピン: P84,P85
#define	NE_P9PDR			0xF0		// 存在しないピン: P94 to P97
#define NE_PFPDR			0x1F		// 存在しないピン: PF0 to PF4
#define NE_PGPDR			0xFF		// 存在しないピン: PG0 to PG7
#elif PIN_SIZE == 100
#define	NE_P0PDR			0x0F		// 存在しないピン: P00 to P03
#define	NE_P1PDR			0x03		// 存在しないピン: P10,P11
#define	NE_P5PDR			0xC0		// 存在しないピン: P56,P57
#define	NE_P6PDR			0xFF		// 存在しないピン: P60 to P67
#define	NE_P7PDR			0xFF		// 存在しないピン: P70 to P77
#define	NE_P8PDR			0xFF		// 存在しないピン: P80 to P87
#define	NE_P9PDR			0xFF		// 存在しないピン: P90 to P97
#define NE_PFPDR			0x3F		// 存在しないピン: PF0 to PF5
#define NE_PGPDR			0xFF		// 存在しないピン: PG0 to PG7
#define NE_PJPDR			0x20		// 存在しないピン: PJ5
#endif

// モジュールストップコントロールレジスタ
#define	MSTPCRA				(*((volatile unsigned long *)0x80010))	//モジュールストップコントロールレジスタA
#define	MSTPCRB				(*((volatile unsigned long *)0x80014))	//モジュールストップコントロールレジスタB
#define	MSTPCRC				(*((volatile unsigned long *)0x80018))	//モジュールストップコントロールレジスタC

#define	MSTPCRA_DMAC_BIT	28				// DMAC
#define MSTPCRA_EXDMAC_BIT	29				// EXDMAC
#define	MSTPCRB_USB1_BIT	18				// ユニバーサルシリアルバスインタフェース1(BGAパッケージのみ）
#define	MSTPCRB_USB0_BIT	19				// ユニバーサルシリアルバスインタフェース0
#define	MSTPCRB_SCI6_BIT	25				// シリアルコミュニケーションインタフェース6
#define	MSTPCRB_SCI5_BIT	26				// シリアルコミュニケーションインタフェース5
#define	MSTPCRB_SCI3_BIT	28				// シリアルコミュニケーションインタフェース3
#define	MSTPCRB_SCI2_BIT	29				// シリアルコミュニケーションインタフェース2
#define	MSTPCRB_SCI1_BIT	30				// シリアルコミュニケーションインタフェース1
#define	MSTPCRB_SCI0_BIT	31				// シリアルコミュニケーションインタフェース0

// プロテクトレジスタ（PRCR）
#define	PRCR				(*((volatile unsigned short*)0x803fe))
#define	PRCR_KEY			0xA500
#define	PRCR_PRC0			_B1(0)			// クロック発生回路関連
#define	PRCR_PRC1			_B1(1)			// 動作モード、消費電力低減、ソフトリセット
#define	PRCR_PRC3			_B1(3)			// LVD関連

// システムコンフィギュレーションコントロールレジスタ(SYSCFG) USB0使用
#define	SYSCFG0				(*((volatile unsigned short*)0xa0000))
#define	SYSCFG0_USBE_ENA	_B1(0)			// USB動作許可
#define	SYSCFG0_DPRPU_ENA	_B1(4)			// プルアップ許可
#define	SYSCFG0_DRPD_ENA	_B1(5)			// プルダウン許可
#define	SYSCFG0_DCFM_F		_B0(6)			// ファンクション機能
#define	SYSCFG0_DCFM_H		_B1(6)			// ホスト機能
#define	SYSCFG0_USBCLK_DIS	_B0(10)			// USBへのクロック供給停止
#define	SYSCFG0_USBCLK_ENA	_B1(10)			// USBへのクロック供給許可

// バスエラー監視許可レジスタ(BEREN)
#define	BEREN	(*((volatile unsigned char *)0x81304))
#define	BEREN_IGAEN_DIS		_B0(0)						   // 不正アドレスアクセス検出許可ビット
#define	BEREN_TOEN_DIS		_B0(1)						   // バスタイムアウト検出許可ビット

#define	BUSPRI	(*((volatile unsigned short *)0x81310)) // バスプライオリティ制御レジスタ(BUSPRI)
#define	BUSPRI_BPRA_FIX		_B(0,0)			// メモリバス１(RAM)  優先順位固定
#define	BUSPRI_BPRA_TOG		_B(1,0)			//                    優先順位トグル
#define	BUSPRI_BPRO_FIX		_B(0,2)			// メモリバス２(ROM)  優先順位固定
#define	BUSPRI_BPRO_TOG		_B(1,2)			//                    優先順位トグル
#define	BUSPRI_BPIB_FIX		_B(0,4)			// 内部周辺バス１     優先順位固定
#define	BUSPRI_BPIB_TOG		_B(1,4)			//                    優先順位トグル
#define	BUSPRI_BPGB_FIX		_B(0,6)			// 内部周辺バス２、３ 優先順位固定
#define	BUSPRI_BPGB_TOG		_B(1,6)			//                    優先順位トグル
#define	BUSPRI_BPHB_FIX		_B(0,8)			// 内部周辺バス４，５ 優先順位固定
#define	BUSPRI_BPHB_TOG		_B(1,8)			//                    優先順位トグル
#define	BUSPRI_BPFB_FIX		_B(0,10)		// 内部周辺バス６     優先順位固定
#define	BUSPRI_BPFB_TOG		_B(1,10)		//                    優先順位トグル
#define	BUSPRI_BPEB_FIX		_B(0,12)		// 外部バス           優先順位固定
#define	BUSPRI_BPEB_TOG		_B(1,12)		//                    優先順位トグル

// マルチファンクションピンコントローラ (MPC)
#define	PFAOE0				(*((volatile unsigned char *)0x8C104)) // アドレス出力許可レジスタ１
#define	PFAOE1				(*((volatile unsigned char *)0x8C105)) // アドレス出力許可レジスタ２
#define	PFBCR0				(*((volatile unsigned char *)0x8C106)) // 外部バス制御レジスタ１
#define	PFBCR1				(*((volatile unsigned char *)0x8C107)) // 外部バス制御レジスタ１
#define	PWPR				(*((volatile unsigned char *)0x8C11F)) // 書き込みプロテクトレジスタ
#define	B0WI_BIT			_B1(7)
#define	PFSWE_BIT			_B1(6)

#define	P00FS				(*((volatile unsigned char *)0x8C140)) // P00端子機能制御レジスタ
#define	P01FS				(*((volatile unsigned char *)0x8C141)) // P01端子機能制御レジスタ
#define	P14FS				(*((volatile unsigned char *)0x8C14C)) // P14端子機能制御レジスタ
#define	P16FS				(*((volatile unsigned char *)0x8C14E)) // P16端子機能制御レジスタ
#define	P26FS				(*((volatile unsigned char *)0x8C156)) // P26端子機能制御レジスタ
#define	P30FS				(*((volatile unsigned char *)0x8C158)) // P30端子機能制御レジスタ
#define	PFUSB0				(*((volatile unsigned char *)0x8C114)) // USB0制御レジスタ
#define	PUPHZS_BIT			_B1(2)								   // PUPHZ選択ビット
#define	PDHZS_BIT			_B1(3)								   // PDHZ選択ビット

#define	SDCLKE_DIS			_B0(7)		// SDCLK出力無効
#define	SDCLKE_ENA			_B1(7)		// SDCLK出力有効

#define	SDRFCR				(*((volatile unsigned short *)0x83C14)) // SDRAMリフレッシュ制御レジスタ (SDRFCR)
#define	SDCMOD				(*((volatile unsigned char *)0x83C01)) // SDCモードレジスタ (SDCMOD)
#define	SDAMOD				(*((volatile unsigned char *)0x83C02)) // SDRAMアクセスモードレジスタ (SDAMOD)
#define	SDRFEN				(*((volatile unsigned char *)0x83C16)) // SDRAMオートリフレッシュ制御レジスタ (SDRFEN)
#define	SDIR				(*((volatile unsigned short *)0x83C24)) // SDRAM初期化レジスタ (SDIR)
#define	SDADR				(*((volatile unsigned char *)0x83C40)) // SDRAMアドレスレジスタ (SDADR)
#define	SDTR				(*((volatile unsigned long *)0x83C44)) // SDRAMタイミングレジスタ (SDTR)
#define	SDMOD				(*((volatile unsigned short *)0x83C48)) // SDRAMモードレジスタ(SDMOD)
#define	SDSR				(*((volatile unsigned char *)0x83C50)) // SDRAMステータスレジスタ (SDSR)
#define	SDICR				(*((volatile unsigned char *)0x83C20)) // SDRAM初期化シーケンス制御レジスタ(SDICR)
#define	SDCCR				(*((volatile unsigned char *)0x83C00)) // SDC制御レジスタ(SDCCR)
#define	SDC_EXENB_DIS		_B0(0)						   // 動作許可ビット 動作禁止
#define	SDC_EXENB_ENA		_B1(0)						   // 動作許可ビット 動作許可
#define	SDC_BSIZE_16		_B(0,4)						   // SDRAMバス幅 16ビット
#define	SDC_BSIZE_32		_B(1,4)						   // SDRAMバス幅 32ビット
#define	SDC_BSIZE_8			_B(2,4)						   // SDRAMバス幅  8ビット

#define RCR1				(*((volatile unsigned char *)0x8C422)) // RTCコントロールレジスタ1
#define RCR2				(*((volatile unsigned char *)0x8C424)) // RTCコントロールレジスタ2
#define RCR3				(*((volatile unsigned char *)0x8C426)) // RTCコントロールレジスタ3
#define RCR4				(*((volatile unsigned char *)0x8C428)) // RTCコントロールレジスタ4
#define SOSCCR              (*((volatile unsigned char *)0x80033)) // サブクロック発振器コントロールレジスタ


//=================プログラムはここから始まる========================
void __startup_func__ START(void);
void interrupt __vectno__{24} _restart_pos(void)
//void	_restart_pos(void)
{
	START();
}
void __startup_func__ START(void)
{
	int wait;
	int i;
//---------------------------------------------------------------------
//	スタックポインタ、PSW初期化	(変更しないで下さい)
//---------------------------------------------------------------------

#ifndef __YIDE_MON_MAKE__
#ifdef __SUB_PROJECT__
_asm	MVTC	#_SYSTEM_STACK_TOP+4,ISP
_asm	MVTC	#_SYSTEM_USTACK_TOP+4,USP	//モニタ、ダウンローダのスタック
#else
//_asm	MVTC	#_STACK_TOP+4,ISP			//割り込みスタック
//_asm	MVTC	#_USTACK_TOP+4,USP			//ユーザスタック
_asm	MVTC	#0x3CFFC,ISP			//割り込みスタック
_asm	MVTC	#0x39FFC,USP			//ユーザスタック
#endif
#else
_asm	extern	___mm_monitor_stack ___mm_monitor_ustack
_asm	MVTC	#___mm_monitor_stack,ISP	//モニタースタック
_asm	MVTC	#___mm_monitor_ustack,USP	//モニタースタック
#endif
_asm	MVTC	#0, PSW						// ＰＳＷ初期化

_asm	extern	_main_start
_asm	MVTC	#_main_start, INTB			// INTB

//---------------------------------------------------------------------
//	クロックの設定
//---------------------------------------------------------------------
//
//		IΦ=96MHz, PΦ=48MHz, BΦ=48MHz,SDCLK有効
//
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__)) //<- ROM化とROMデバッグ時のみ必要

	// レジスタプロテクト解除
	PRCR	 = (PRCR_KEY | PRCR_PRC1 | PRCR_PRC0) ;

	// 周辺機能の停止 (DMAC/DTC,EXDMAC)
	MSTPCRA	|= (_B1(MSTPCRA_DMAC_BIT) | _B1(MSTPCRA_EXDMAC_BIT));

	// 存在しないポートは"1" 出力設定
#if PIN_SIZE == 144
	P1DR = (P1DR | NE_P1PDR);
	P5DR = (P5DR | NE_P5PDR);
	P8DR = (P8DR | NE_P8PDR);
	P9DR = (P9DR | NE_P9PDR);
	PFDR = (PFDR | NE_PFPDR);
	PGDR = (PGDR | NE_PGPDR);
#endif

	// 
	// メインクロックの発振設定
	// 

	P3MR |= (BIT_6 | BIT_7);

	// メインクロック発振器ウェイトコントロール
	MOSCWTCR = 0x0d;					// wait 131072 cycles ( 10ms )

	// メインクロック発振器
	// b7:b1  Reserved
	// b0     メインクロック発振停止    - メインクロック発振器動作 = 0
	MOSCCR = 0x00;
    do {} while (0x00 != MOSCCR);

	wait = 575; // 23000/(8us x 5(cycle)
	do {} while(--wait) ;

	//
	// PLLの発振設定
	// 

	// PLLコントロールレジスタ PLL入力分周比選択ビット：1分周、周波数逓倍：16倍
	PLLCR = (PLLCR_STC_16 | PLLCR_PLIDIV_1);
	//	PLLCR = 0x0f00;

	// PLL：192MHz PLLクロック安定時間500usec
	// 500 × 192 = 96000 => 待機時間 131072
	PLLWTCR = PLLWTCR_131072_CYL;
	//	PLLWTCR = 0x0a;

	// PLL動作
    PLLCR2 = 0x00;

	wait = 300;
	do {} while(--wait);

	//
	// クロック分周比設定
	// 

	// 内部クロック分周比を設定
	// b31:b28      FCK       FlashIFクロック          - 4分周
	// b27:b24      ICK       システムクロック         - 2分周
	// b23          PSTOP1    BCLK出力制御             - 停止
	// b22          PSTOP0    SDCLK出力制御            - 停止
	// b21:b20      Reserved  予約
	// b19:b16      BCK       外部バスクロック         - 4分周
	// b15:b12      PCLKA     周辺モジュールクロックA  - 2分周
	// b11:b8       PCLKB     周辺モジュールクロックB  - 4分周
	SCKCR = SCKCR_VAL;
	do {} while (SCKCR_VAL != SCKCR);

	SCKCR2 = 0x0032;
	do {} while(0x0032 != SCKCR2);

	// 外部バスクロックコントロール
	// 分周無し
	BCKCR = 0x00;
	do {} while(0x00 != BCKCR) ;

	// クロックソース選択
	SCKCR3 = SCKCR3_PLL;
	do {} while(SCKCR3_PLL != SCKCR3) ;

	// HOCO停止
	HOCOCR = HCSTP_BIT;
	// HOCO電源OFF
	HOCOPCR = 0x01;

	// レジスタプロテクト
	PRCR	 = PRCR_KEY;

#endif	// #if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__))

//<<変更②>>
//======================================================================
//	外部RAMの初期化
//	------ユーザのCPUボードに合わせて変更して下さい-----
//	外部RAMを使用する場合はバス設定など外部RAMの初期化が必要です。
//　内臓RAMだけの場合は不要ですから削除（コメントにする）してください。
//======================================================================
//====================================================================
// I/Oアドレスの定義
// LFY-63N1以外のCPUの場合はIOアドレスを確認してください。
//====================================================================

#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__)) //<- ROM化とROMデバッグ時のみ必要
	
	/* 外部バスなどを設定する場合ここに定義		*/

#ifndef	_SDRAM_NOUSE_
	// SDRAM機能初期化
	// レジスタプロテクト解除
	PRCR	 = PRCR_KEY | PRCR_PRC1 | PRCR_PRC0 ;

	// SDCLK,BCLK出力停止
	SCKCR |= (PSTOP1_BIT | PSTOP0_BIT);

	// バスエラー監視機能の停止
	BEREN = ( BEREN_IGAEN_DIS | BEREN_TOEN_DIS );

	// バスプライオリティの設定
	BUSPRI = (BUSPRI_BPRA_FIX | BUSPRI_BPRO_FIX | BUSPRI_BPIB_FIX | BUSPRI_BPGB_FIX |
			  BUSPRI_BPHB_FIX | BUSPRI_BPFB_FIX | BUSPRI_BPEB_FIX);

	// SDRAM端子機能

	// ADRLE  = 1 : PA0-PA7を外部アドレスバスA0-A7に設定
	// ADRHMS = 0 : PC0-PC7を外部アドレスバスA16-A23に設定
	// DHE    = 1 : PE0-PE7を外部データバスD8-D15に設定
	// DH32E  = 0 : PG0-PG7、P90-P97をI/Oポートに設定
	PFBCR0 = 0x11;

	// MDSDE  = 1 : CKE,SDCS#,RAS#,CAS#,WE#,DQM0 出力有効
	// DQM1E  = 1 : DQM1出力有効
	// SDCLKE = 1 : SDCLK出力有効
	PFBCR1 = 0xD0;

	// A15E=1 : A15出力禁止
	// A14E=1 : A14出力許可
	// A13E=1 : A13出力許可
	// A12E=1 : A12出力許可
	// A11E=1 : A11出力許可
	// A10E=1 : A10出力許可
	//  A9E=1 : A9出力許可
	//  A8E=1 : A8出力許可
	PFAOE0 = 0xFF;

	// A23E=0 : A23出力禁止
	// A22E=0 : A22出力禁止
	// A21E=0 : A21出力禁止
	// A20E=0 : A20出力禁止
	// A19E=0 : A19出力禁止
	// A18E=0 : A18出力禁止
	// A17E=0 : A17出力禁止
	// A16E=0 : A16出力禁止
	PFAOE1 = 0x00;

	// 入力ポート設定
	PADR = 0x00;						// A0-A7で使用
	PBDR = 0x00;						// A8-A14で使用
	PDDR = 0x00;						// D0-D7で使用
	PEDR = 0x00;						// D8-D15で使用
	P6DR = 0x00;						// CKE,SDCS#,RAS#,CAS#,W#,DQM0,DQM1で使用
	P7DR &= ~0x01; 						// P70 = SDCLK端子 入力ポート

	// 内蔵ROM有効・外部バス有効 設定
	SYSCR0 = ( SYSCR0_KEY | SYSCR0_EXBE | SYSCR0_ROME ); // 0x5A03

	while(!(SYSCR0 & SYSCR0_EXBE))
		;

	// SDCLK端子出力許可
	SCKCR  &= ~PSTOP0_BIT;

	// レジスタプロテクト
	PRCR	 = PRCR_KEY;

	// 100u以上待つ必要あり
	wait = 686;			// 100usec
	do {} while( --wait ) ;		// １ループ７クロック

	// SDRAM初期化レジスタ
	// ARFI=1:初期化オートリフレッシュ間隔 4サイクル
	// ARFC=2:初期化オートリフレッシュ回数 2回
	//  PRC=0:初期化プリチャージサイクル数 3サイクル
	SDIR =0x0021;

	// SDRAM初期化シーケンス制御レジスタ(SDICR)
	// INIRQ=1:初期化シーケンス開始
	SDICR = 0x01;

	// SDRAMステータス確認
	while(SDSR)
		;

	// SDRAMバス幅選択
	SDCCR = SDC_BSIZE_16;				// 16ビット、SDRAM動作禁止

	// SDRAMモード選択
	// A12-A10 Reserved
	// A9      WriteBurstMode - Single Bit (1)
	// A8-A7   TestMode       - Normal (00)
	// A6-A4   CAS Latency    - 2 (010)
	// A3      BurstType      - Sequential (0)
	// A2-A0   BurstLength    - 1 (000)
	SDMOD = 0x0220;

	// SDRAMタイミングレジスタ (SDTR)
	// CL=2 :SDRAMCカラムレイテンシ設定 2サイクル
	// WR=2 :ライトリカバリ 2サイクル 42ns(48MHz)
	// RP=1 :ロウプリチャージ 2サイクル 21ns(48MHz)
	// RCD=1:ロウカラムレイテンシ 2サイクル 21ns(48MHz)
	// RAS=3:ロウアクティブ期間 3サイクル 62ns(48MHz)
	SDTR = 0x00020102;

	// アドレスマルチプレクス設定 
	// ロウアドレス13 ビット、カラムアドレス9 ビット、256M ビット品、16 ビットバスのSDRAMを1 個接続 -> シフト量9
	SDADR = 0x01;

	// エンディアン設定
	// 動作モードと同じエンディアン
	// EMODE(b0) = 0:SDRAMアドレス空間のエンディアンは動作モードのエンディアンと同じ
	SDCMOD = 0x00;

	// アクセスモードの設定
	//  BE=0: 連続アクセス禁止
	SDAMOD = 0x00;

	// オートリフレッシュタイミング設定
	// RFC=369サイクル tREF/ロウアドレス数=63ns/8192=7.690us 7.690/(1/48Mhz)=369Cycle(0x171)
	// REF=4サイクル
	SDRFCR = 0x3171;

	// オートリフレッシュ動作設定
	// RFEN=1:オートリフレッシュ有効
	SDRFEN = 0x01;

	// SDRAMアドレス空間の動作許可
	SDCCR |= SDC_EXENB_ENA;				// SDRAM動作許可

#endif	// defined(_SDRAM_USE_)

#endif	// #if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__))

	// 内蔵ROM有効・外部バス無効 設定
	SYSCR0 = ( SYSCR0_KEY | SYSCR0_ROME ); // 0x5A01

	/* ＲＡＭ上実行の場合はセグメントコピー（変更しない)	*/
#ifdef	__TEXT_RAM__
	/* テキストセグメントのコピー	*/
	p1 = (unsigned char *)&TEXT_RAM_rom;
	p2 = (unsigned char *)&TEXT_RAM_start;
	temp = (unsigned long)&TEXT_RAM_size ;
	do {
		*p2++ = *p1++;
	} while(--temp) ;
	/* 定数セグメントのコピー	*/
	p1 = (unsigned char *)&DATA_CONST_RAM_rom;
	p2 = (unsigned char *)&DATA_CONST_RAM_start;
	temp = (unsigned long)&DATA_CONST_RAM_size ;
	do {
		*p2++ = *p1++;
	} while(--temp) ;
#endif /* __TEXT_RAM__ */

	__CSTARTUP_INIT2__();
}

//<<以下変更③>>
//======================================================================
//	シリアル通信の初期化
//　デバッグ用、およびC言語のprintf等関数に使うシリアルチャンネルの
//　初期化を行います。
//	アプリケーションで独自に使うシリアルチャンネルはここで初期化
//　する必要はありません。main関数で行ってください。
//======================================================================
//====================================================================
// I/Oアドレスの定義
// RX63N以外のCPUの場合はIOアドレスを確認してください。
//====================================================================
#define	SMR1	(*((volatile unsigned char *)0x8A020))		//シリアルモードレジスタ
#define	BRR1	(*((volatile unsigned char *)0x8A021))		//ビットレートレジスタ
#define	SCR1	(*((volatile unsigned char *)0x8A022))		//シリアルコントロールレジスタ
#define	TDR1	(*((volatile unsigned char *)0x8A023))		//トランスミットデータレジスタ
#define	SSR1	(*((volatile unsigned char *)0x8A024))		//シリアルステータスレジスタ
#define	RDR1	(*((volatile unsigned char *)0x8A025))		//レシーブデータレジスタ
#define	SCMR1	(*((volatile unsigned char *)0x8A026))		//スマートカードモードレジスタ
#define	SEMR1	(*((volatile unsigned char *)0x8A027))		//シリアル拡張モードレジスタ

#define	SMR6	(*((volatile unsigned char *)0x8A0C0))
#define	BRR6	(*((volatile unsigned char *)0x8A0C1))
#define	SCR6	(*((volatile unsigned char *)0x8A0C2))
#define	TDR6	(*((volatile unsigned char *)0x8A0C3))
#define	SSR6	(*((volatile unsigned char *)0x8A0C4))
#define	RDR6	(*((volatile unsigned char *)0x8A0C5))
#define	SCMR6	(*((volatile unsigned char *)0x8A0C6))
#define	SEMR6	(*((volatile unsigned char *)0x8A0C7))

// シリアルステータスレジスタ　ビットシンボル
#define	SSR_ORER	_B1(5)				// オーバランエラーフラグ
#define	SSR_FER		_B1(4)				// フレーミングエラーフラグ
#define	SSR_PER		_B1(3)				// パリティエラーフラグ
#define	SSR_TEND	_B1(2)				// トランスミットエンドフラグ

// シリアルコントロールレジスタ　ビットシンボル
#define	CKE_BIT		_B(3,0)			   // クロックイネーブルビット
#define	TEIE_BIT	_B1(2)			   // トランスミットエンド・インタラプトイネーブル
#define	MPIE_BIT	_B1(3)			   // マルチプロセッサ・インタラプトイネーブル
#define	RE_BIT		_B1(4)			   // レシーブイネーブルビット
#define	TE_BIT		_B1(5)			   // トランスミットイネーブル
#define	RIE_BIT		_B1(6)			   // レシーブインタラプトイネーブル
#define	TIE_BIT		_B1(7)			   // トランスミットインタラプトイネーブル

#define	INT_REQ_BASE	0x87000		// 割り込み要求レジスタアドレス
#define	INT_REQ(a)		*((volatile unsigned char *)(INT_REQ_BASE+(a)))

#define	INT_IPR_BASE	0x87300		// 割り込み優先順位レジスタアドレス
#define	INT_IPR(n,a)	*((volatile unsigned char *)(INT_IPR_BASE+(n)))=(a)

#define	IER04			(*((volatile unsigned char *)0x87204))
#define	IER1B			(*((volatile unsigned char *)0x8721B))
#define	IER1D			(*((volatile unsigned char *)0x8721D))

//====================================================================
// シリアルポート各種マクロ定義
// RX62N以外のCPUの場合は変更必要
//====================================================================

// RXD:P30, TXD:P26
#define	SCI1_RXINT			217									//シリアル受信割込みベクタ番号
#define	SCI1_TXINT			218									//シリアル送信割込みベクタ番号
#define	SCI1_MSTCR			MSTPCRB								//モジュールストップレジスタ
#define	SCI1_MSTCR_BIT		_B1(MSTPCRB_SCI1_BIT)				//モジュールストップレジスタビット一
#define	SCI1_IPR(a)			INT_IPR(0xD9,a)						//割込みレベル設定マクロ 0xd9=217
#define	SCI1_PORT_TX		(P26FS |= _B(10,0))					//TXD端子を有効にするマクロ
#define	SCI1_PORT_RX		(P30FS |= _B(10,0))					//RXD端子を有効にするマクロ
#define	SCI1_IER_RX			IER1B
#define	SCI1_IER_RX_BIT		_B1(1)
#define	SCI1_IER_TX			IER1B
#define	SCI1_IER_TX_BIT		_B1(2)

#define	SCI1_PODR_TX		P2ODR								//TXD端子ポート出力データ(HIGH)
#define SCI1_PDR_TX			P2DR
#define SCI1_PDR_RX			P3DR
#define SCI1_PMR_TX			P2MR
#define SCI1_PMR_RX			P3MR
#define PORT_TXD_BIT		_B1(6)		// P26
#define PORT_RXD_BIT		_B1(0)		// P30

#if (DBG_PORT==SCI1)
#define DBG_PODR_TX			SCI1_PODR_TX
#define DBG_PDR_TX			SCI1_PDR_TX
#define DBG_PDR_RX			SCI1_PDR_RX
#define DBG_PMR_TX			SCI1_PMR_TX
#define DBG_PMR_RX			SCI1_PMR_RX
#define DBG_SEMR			SEMR1
#define	DBG_IER_TX			SCI1_IER_TX
#define	DBG_IER_TX_BIT		SCI1_IER_TX_BIT
#endif

// RXD:P01, TXD:P00
#define	SCI6_RXINT			232									//シリアル受信割込みベクタ番号
#define	SCI6_TXINT			233									//シリアル送信割込みベクタ番号
#define	SCI6_MSTCR			MSTPCRB								//モジュールストップレジスタ
#define	SCI6_MSTCR_BIT		_B1(MSTPCRB_SCI6_BIT)				//モジュールストップレジスタビット一
#define	SCI6_IPR(a)			INT_IPR(0xE8,a)						//割込みレベル設定マクロ
#define	SCI6_PODR_TX		P0PODR |= _B1(0)					//TXD端子ポート出力データ(HIGH)
#define	SCI6_PORT_TX		P00FS |= _B(10,0)					//TXD端子を有効にするマクロ
#define	SCI6_PORT_RX		P01FS |= _B(10,0)					//RXD端子を有効にするマクロ
#define	SCI6_IER_RX			IER1D
#define	SCI6_IER_RX_BIT		_B1(0)
#define	SCI6_IER_TX			IER1D
#define	SCI6_IER_TX_BIT		_B1(1)

#if (DBG_PORT==SCI6)
#define DBG_PODR_TX			SCI6_PODR_TX
#define DBG_PDR_TX			SCI6_PDR_TX
#define DBG_PDR_RX			SCI6_PDR_RX
#define DBG_PMR_TXGEN		SCI6_PMR_TXGEN
#define DBG_PMR_RXGEN		SCI6_PMR_RXGEN
#define DBG_PMR_TXPER		SCI6_PMR_TXPER
#define DBG_PMR_RXPER		SCI6_PMR_RXPER
#define DBG_SEMR			SEMR1
#endif

//====================================================================
// ＵＳＢポートの各種定義
//====================================================================

#define	USB_MSTCR			MSTPCRB								//ＵＳＢモジュールストップレジスタを記述してください
#define	USB_MSTCR_BIT		_B1(MSTPCRB_USB0_BIT)				//モジュールストップレジスタビット位置
#define	IPR_USB(a)			INT_IPR(0x23,a)
#define	USB_IER				IER04
#define	USB_IER_BIT			_B1(3)

#ifndef __YIDE_MON_MAKE__
#define	USB_INT_UEVEL		15		// モニター以外の設定
#else
#define	USB_INT_UEVEL		14
#endif

int  _YDrvUsbInit( char* pSerialNo, void* pRxIntFunc  ) ;
void _YDrvUsbSoftInit( char* pSerialNo, void* pRxIntFunc ) ;
int  _YDrvUsbStart();
int  _YDrvUsbEnd() ;

//====================================================================
//　エコーバックの設定
//この一行をコメントにするとfgetc関数等の入力をエコーバックしません。
//====================================================================
#define	ECHO_BACK_ON	//stdinからの入力をエコーバックする

//====================================================================
//	C言語の入出力デバイス数を設定します。
//	たとえば入出力デバイスを2にしSCI0とSCI1を入出力デバイスにすると
//	fputc('A', stdout) -> SCI0に出力
//	fputc('B', std_sio1) -> SCI1に出力
//	のように出力先を切り替えることができます
//====================================================================
#define	DEVICE_COUNT	2			//入出力デバイス数　最低1  最大6


/*
	入出力デバイス0の初期設定
		入出力デバイス0は、プログラムをダウンロードしたりイエロースコープでデバッグする
		ときのシリアルチャンネルを割り当てます。
		また、printfやfgetc関数のstdin、stdoutの割り当て先でもあります。
		入出力デバイス0は、通常はROM書き込み時のシリアルチャンネルを割り当てます。
*/

/* モニターをＲＡＭ上で動作させるマクロ	*/
#ifdef	__TEXT_RAM__
#pragma	seg_text	TEXT_RAM
#pragma	seg_const	DATA_CONST_RAM
#define	TEXT		TEXT_RAM
#endif	/* __TEXT_RAM__ */

/********************************************************************
*****	初期化関数												*****
*****		シリアル通信の初期化を書く							*****
*****		引数	なし										*****
*****		戻り値	なし										*****
********************************************************************/

// ボーレート誤差判定
#if( BRR_CHK(DBG_PORT_BPS,2) )
#error "デバッグシリアルポートのボーレート誤差が２％以上です。２％未満のボーレートを設定してください。" 
#endif

void ___mm_usb_init(void);

void ___mm_sio_init0(void)
{
/* リモートデバッグの場合、既にモニターでシリアルを初期化しているため、何もしない	*/
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__)) //<- ROM化とROMデバッグ時のみ必要
#if	( DBG_PORT == USB_R )

	//////////////////////////////////////
	//	ＣＰＵ内蔵ＵＳＢポート

	/* ハードリセット処理	*/
	___mm_usb_init() ;

#else

	//////////////////////////////////////
	//	シリアルポート
	
	int	wait ;

	// レジスタプロテクト解除
	PRCR	 = PRCR_KEY | PRCR_PRC1 ;
	// モジュールストップ解除
	DBG_MSTCR &= ~DBG_MSTCR_BIT ;
	// レジスタプロテクト
	PRCR	 = PRCR_KEY ;

	// RXI 割り込みレベル設定、イエロースコープの場合は１４
	DBG_IPR(14) ;

	// 送受信停止＆クロック分周率設定
	DBG_SCR = 0 ;			//SCRレジスタを0にする	
	do {} while( 0x00 != (DBG_SCR & 0xf0)) ;

	// ポート出力データの設定　TXD:HIGH
	DBG_PODR_TX |= PORT_TXD_BIT;

	// ポート方向を設定
	DBG_PDR_TX |= PORT_TXD_BIT;
	DBG_PDR_RX &= ~(PORT_RXD_BIT);

	// ポートモードを汎用入出力ポートに設定
	// 　　PmnPFSレジスタ設定時は、当該PMRレジスタを"0"(汎用入出力ポート)に設定が必要
	DBG_PMR_TX &= ~(PORT_TXD_BIT);
	DBG_PMR_RX &= ~(PORT_RXD_BIT);

	// MPC書き込みプロテクトレジスタ操作
	PWPR &= ~B0WI_BIT;
	PWPR |= PFSWE_BIT;

	// ピンファンクションコントローラの設定
	DBG_PORT_TX ;
	DBG_PORT_RX ;

	// MPC書き込みプロテクトレジスタ操作
	PWPR &= ~PFSWE_BIT;
	PWPR |= B0WI_BIT;

	// ポートモードを周辺機能に設定
	DBG_PMR_RX |= PORT_RXD_BIT;
	DBG_PMR_TX |= PORT_TXD_BIT;

	// SCI初期化
	// CKE=00 内部ボーレートジェネレータ
	DBG_SCR &= ~(CKE_BIT);

	// シリアルモードレジスタ
	// b7    CM:コミュニケーションモード  - 調歩同期モード(0)
	// b6    CHR   キャラクタレングス     - 8bits(0)
	// b5    PE    パリティイネーブル     - パリティ無し(0)
	// b4    PM    パリティモード         - 無視(0)
	// b3    STOP  ストップビット         - 1 ストップビット(0)
	// b2    MP    マルチプロセッサモード - 通信機能禁止(0)
	// b1:b0 CKS   クロックセレクト       - PCLKクロック(n=0) (0)
	DBG_SMR = 0;			//8bit,parity-non,stop 1  

	DBG_SCMR = 0xF2;		// シリアルコミュニケーションI/Fモード
							// TDRレジスタの内容をそのまま送信
							// LSBファースト
	DBG_SEMR = 0x00;		// RXDノイズ除去無効
							// 16基本クロックサイクル

	//BRRレジスタでボーレートを設定
	DBG_BRR = BRR_CAL(DBG_PORT_BPS);
	/*　1ビット送信する時間ウエイトを入れる。１ループ７クロック	*/
	wait = BRR_WAIT(DBG_PORT_BPS);
	do {} while( --wait ) ;		// １ループ７クロック

	// 割り込み要求レジスタのリセット
	INT_REQ(DBG_SCI_TXINT) = 0;
	INT_REQ(DBG_SCI_RXINT) = 0;

	//送受信許可、割り込み許可。実際の割り込みの許可禁止はINTCで行う
	DBG_SCR = 0xf0;
	do {} while(0xf0 != (DBG_SCR & 0xf0)) ;
#endif
#endif	// #if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__))
}

/********************************************************************
*****	ＵＳＢ初期化関数										*****
*****		ＵＳＢの初期化を行う								*****
*****		引数	なし										*****
*****		戻り値	なし										*****
********************************************************************/
/*
	シリアル番号定義
	
	ＵＮＩコード１０進数文字列１０桁で表現する。
*/

/*　シリアル番号を指定された桁のＵＮＩコード数字文字に変換するマクロ	*/
#define	_UD(a)	((USB_SERIAL_NO/(a))%10)+'0',0

/*　シリアル番号文字列													*/
static const char USB_SerialNo[] = {22,3,
_UD(1000000000),_UD(100000000),_UD(10000000),_UD(1000000),_UD(100000),_UD(10000),_UD(1000),_UD(100),_UD(10),_UD(1) };

extern int	__ei();

/*
	モジュールストップが既に解除されている場合、初期化処理が終わっていると
	解釈し、初期化は行わない。
*/
void ___mm_usb_init(void)
{
	// レジスタプロテクト解除
	PRCR	 = PRCR_KEY | PRCR_PRC1 ;

	/* ＵＳＢモジュールストップ解除	*/
	USB_MSTCR &= ~USB_MSTCR_BIT ;

	// レジスタプロテクト
	PRCR	 = PRCR_KEY ;

	// MPC書き込みプロテクトレジスタ操作
	PWPR &= ~B0WI_BIT;
	PWPR |= PFSWE_BIT;

	// P14,P16 ポートモードを汎用入出力に設定
	P1MR &= ~(BIT_4|BIT_6);

	// ピンファンクション設定
	P14FS = 0x11;						// USB0_DPUPE
	P16FS = 0x11;						// USB0_VBUS

	// P14,P16 ポートモードを周辺機能に設定
	P1MR |= (BIT_4|BIT_6);

	PFUSB0 |= PUPHZS_BIT;

	// MPC書き込みプロテクトレジスタ操作
	PWPR &= ~PFSWE_BIT;
	PWPR |= B0WI_BIT;
		
	/* 割り込み優先順位初期化		*/
	IPR_USB(USB_INT_UEVEL) ;

	/* USB割り込み許可				*/
	USB_IER |= USB_IER_BIT ;

	/* ＵＳＢドライバー初期化		*/
	_YDrvUsbInit( USB_SerialNo, 0 ) ;
	_YDrvUsbStart() ;
	__ei();
}

/********************************************************************
*****	1バイト入力関数	(ポーリングあり)						*****
*****		引数	なし										*****
*****		戻り値	入力したデータ								*****
********************************************************************/

#if( DBG_PORT == USB_R )
int ___mm_sio_in0(void);
#ifndef	__YIDE_MON_MAKE__
#asmb {
	extern	_YDrvUsbRcvBytePoll _YDrvUsbSndBytePoll _YDrvUsbSndFlushPoll
	segment	TEXT ATR_CODE
	public	___mm_sio_in0
___mm_sio_in0:
	PUSH	R15
	MOV		#0, R15							// タイムアウト無効
	BSR		_YDrvUsbSndFlushPoll			// 入力を実行する場合はここでフラッシュを実行
											// 出力関数において改行が無い場合、このフラッシュで表示される。
	MOV		#0, R15							// タイムアウト無効
	BSR		_YDrvUsbRcvBytePoll
	RTSD	#1*4,R15-R15
	
	segment	TEXT ATR_CODE
	public	___mm_sio_in0_nf
___mm_sio_in0_nf:
	PUSH	R15
	MOV		#0, R15						// タイムアウト無効
	BSR		_YDrvUsbRcvBytePoll
	RTSD	#1*4,R15-R15
}
#else
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_in0
___mm_sio_in0:
	RTS
}
#endif	/* __YIDE_MON_MAKE__ */
#else
int ___mm_sio_in0(void)
{
	unsigned char c;

	// インタラプト要求発生待ち
	do {} while( (INT_REQ(DBG_SCI_RXINT) == 0) );
#if 1
	if (!(DBG_IER_RX & DBG_IER_RX_BIT)) {
		// 
		// 割り込み要求禁止状態(ポーリング受信時)は割り込み要求フラグをオフする
		// 
		_asm	pushc	psw;
		_asm	clrpsw	I;					// スーパバイザモード前提

		INT_REQ(DBG_SCI_RXINT) = 0;

		_asm	popc	psw;
	}
#endif
	c = DBG_RDR;
	return c;
}
#endif	/* ( DBG_PORT == USB_R ) */

/********************************************************************
*****	1バイト入力関数(ポーリングなし）						*****
*****		引数	なし										*****
*****		戻り値	入力したデータ								*****
********************************************************************/
/*
	シリアルの場合、割り込み要求レジスタはエッジ検出のため、自動的にクリアされる
*/
#if( DBG_PORT == USB_R )
int ___mm_sio_ind0(void);
#ifndef	__YIDE_MON_MAKE__
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_ind0
___mm_sio_ind0:
	PUSH	R15
	//	MOV		#0, R15					// タイムアウト無効
	MOV		#1, R15					// タイムアウト無効
	BSR		_YDrvUsbRcvBytePoll
	RTSD	#1*4,R15-R15
}
#else
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_ind0
___mm_sio_ind0:
	RTS
}
#endif	/* __YIDE_MON_MAKE__ */
#else
int ___mm_sio_ind0(void)
{
	return DBG_RDR;
}
#endif	/* ( DBG_PORT == USB_R ) */

/********************************************************************
*****	1バイト出力関数											*****
*****		引数	出力するデータ								*****
*****		戻り値	出力したデータ								*****
********************************************************************/

#if( DBG_PORT == USB_R )
int ___mm_sio_out0(unsigned char data);

#define	INTSTS0_ADR		0xA0040
#define	INTSTS0_VBSTS	7

#ifndef	__YIDE_MON_MAKE__
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_out0
___mm_sio_out0:
	PUSHM	R14-R15
	MOV		#INTSTS0_ADR, R1
	MOV.W	[R1], R1
	BTST	#INTSTS0_VBSTS, R1			// USBが接続されているか判定
	BEQ		___mm_sio_out0_L00
		MOV		#0, R14					// タイムアウト無効
		BSR		_YDrvUsbSndBytePoll
		CMP		#'\n', R3				// 送信データが"\n"の場合フラッシュを実行
		BNE		___mm_sio_out0_L00
		MOV		#0, R15					// タイムアウト無効
		BSR		_YDrvUsbSndFlushPoll
___mm_sio_out0_L00:
	MOVU.B	R3, R3
	RTSD	#2*4,R14-R15
}
#else
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_out0
___mm_sio_out0:
	RTS
}
#endif	/* __YIDE_MON_MAKE__ */
#else
int ___mm_sio_out0(unsigned char data)
{
	do {} while( (INT_REQ(DBG_SCI_TXINT) == 0) );
#if 0
	// 割り込み未使用時は、割り込み要求フラグをオフする
	if (!(DBG_IER_TX & DBG_IER_TX_BIT)) {
		_asm	pushc	psw;
		_asm	clrpsw	I;					// スーパバイザモード前提

		// 送信割り込み要求クリア

		INT_REQ(DBG_SCI_TXINT) = 0;

		_asm	popc	psw;
	}
#endif
	// TDRレジスタへデータ書込み
	DBG_TDR = data;
	return data;
}
#endif	/* ( DBG_PORT == USB_R ) */

/********************************************************************
*****	フラッシュ関数											*****
********************************************************************/
/*
	送信完了まで待ちます。
*/
#if( DBG_PORT == USB_R )
void ___mm_sio_flush0(void);
#ifndef	__YIDE_MON_MAKE__
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_flush0
___mm_sio_flush0:
	PUSH	R15
	MOV		#0, R15					// タイムアウト無効
	BSR		_YDrvUsbSndFlushPoll
	RTSD	#1*4,R15-R15
}
#else
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_flush0
___mm_sio_flush0:
	RTS
}
#endif	/* __YIDE_MON_MAKE__ */
#else
void ___mm_sio_flush0(void)
{
	do {} while ((DBG_SSR & SSR_TEND) == 0) ;
}
#endif	/* ( DBG_PORT == USB_R ) */

/********************************************************************
*****	受信割り込み許可										*****
*****		引数	flag(割り込みフラグをクリアする場合１）		*****
*****		戻り値	なし										*****
********************************************************************/

#if( DBG_PORT == USB_R )
void ___mm_sio_ei0(int flag);
#ifndef	__YIDE_MON_MAKE__
#asmb {
	extern	_YDrvUsbEnaRcvInt
	segment	TEXT ATR_CODE
	public	___mm_sio_ei0
___mm_sio_ei0:
	BRA		_YDrvUsbEnaRcvInt
}
#else
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_ei0
___mm_sio_ei0:
	RTS
}
#endif	/* __YIDE_MON_MAKE__ */
#else
void ___mm_sio_ei0(int flag)
{
	if(flag) INT_REQ(DBG_SCI_RXINT) = 0 ;
	DBG_IER_RX |= DBG_IER_RX_BIT ;
}
#endif	/* ( DBG_PORT == USB_R ) */

/********************************************************************
*****	受信割り込み不許可										*****
*****		引数	なし										*****
*****		戻り値	なし										*****
********************************************************************/

#if( DBG_PORT == USB_R )
void ___mm_sio_di0(void);
#ifndef	__YIDE_MON_MAKE__
#asmb {
	extern	_YDrvUsbDisRcvInt
	segment	TEXT ATR_CODE
	public	___mm_sio_di0
___mm_sio_di0:
	BRA		_YDrvUsbDisRcvInt
}
#else
#asmb {
	segment	TEXT ATR_CODE
	public	___mm_sio_di0
___mm_sio_di0:
	RTS
}
#endif	/* __YIDE_MON_MAKE__ */
#else
void ___mm_sio_di0(void)
{
	DBG_IER_RX &= ~DBG_IER_RX_BIT ;
}
#endif	/* ( DBG_PORT == USB_R ) */

//------------------------------通常の使用では変更はここまでです。---------------------------------
//---------------------入出力デバイスが2以上の場合は以下も記述してください-------------------------
/*
	入出力デバイス1の設定
		入出力デバイス1は、printfやfgetc関数のstd_sio1の割り当て先になります。
		この例ではSCI2に割り当てています。
		したがって、以下のような関数の入出力先はSCI2になります。
		
		printf(std_sio1, ....);
		fgetc(std_sio1);
		
		標準入出力(stdin、stdout、stderr)以外使わない場合はここの設定は
		不要です。
*/
/********************************************************************
*****	初期化関数												*****
*****		シリアル通信の初期化を書く							*****
*****		引数	なし										*****
*****		戻り値	なし										*****
********************************************************************/

// ＳＣＩ６でのボーレート
#define	SCI6_BPS	38400L

// ボーレート誤差判定
#if( BRR_CHK(SCI6_BPS,2) )
#error "シリアルポート２のボーレート誤差が２％以上です。２％未満のボーレートを設定してください。" 
#endif

void ___mm_sio_init1(void)
{
	int	wait ;

	// モジュールストップ解除
	SCI6_MSTCR &= ~SCI6_MSTCR_BIT ;
	// 割り込みレベル設定、イエロースコープの場合は１４
	SCI6_IPR(14) ;
	// 送受信停止＆クロック分周率設定
	SCR6 = 0 ;			//SMRレジスタを0にする	
	//ピンファンクションコントローラの設定
	SCI6_PORT_TX ;
	SCI6_PORT_RX ;
	SMR6 = 0;			//8bit,parity-non,stop 1  
	//BRRレジスタでボーレートを設定
	BRR6 = BRR_CAL(SCI6_BPS);
	/*　1ビット送信する時間ウエイトを入れる。１ループ７クロック	*/
	wait = BRR_WAIT(SCI6_BPS);
	do {} while( --wait ) ;		// １ループ７クロック
	//送受信許可、割り込み許可。実際の割り込みの許可禁止はINTCで行う
	SCR6 = 0xf0;
}

/********************************************************************
*****	1バイト入力関数	(ポーリングあり)						*****
*****		引数	なし										*****
*****		戻り値	入力したデータ								*****
********************************************************************/
int ___mm_sio_in1(void)
{
//	do {} while( ( SSR6 & SSR_RDRF ) == 0 );
	return RDR6;
}

/********************************************************************
*****	1バイト入力関数(ポーリングなし）						*****
*****		引数	なし										*****
*****		戻り値	入力したデータ								*****
********************************************************************/
int ___mm_sio_ind1(void)
{
	return RDR6;
}

/********************************************************************
*****	1バイト出力関数											*****
*****		引数	出力するデータ								*****
*****		戻り値	出力したデータ								*****
********************************************************************/
int ___mm_sio_out1(int data)
{
//	do {} while( ( SSR6 & SSR_TRDE ) == 0 );
	TDR6 = data;
	return data ;
}

/********************************************************************
*****	フラッシュ関数											*****
********************************************************************/
/*
	送信完了まで待ちます。
*/
void ___mm_sio_flush1(void)
{
	do {} while ((SSR6 & SSR_TEND) == 0) ;
}

/********************************************************************
*****	受信割り込み許可										*****
*****		引数	flag(割り込みフラグをクリアする場合１）		*****
*****		戻り値	なし										*****
********************************************************************/
void ___mm_sio_ei1(int flag)
{
	if(flag) INT_REQ(SCI6_RXINT) = 0 ;
	SCI6_IER_RX |= SCI6_IER_RX_BIT ;
}

/********************************************************************
*****	受信割り込み不許可										*****
*****		引数	なし										*****
*****		戻り値	なし										*****
********************************************************************/
void ___mm_sio_di1(void)
{
	SCI6_IER_RX &= ~SCI6_IER_RX_BIT ;
}

/********************************************************************
*****	DTC関連の設定											*****
*****		DTCを使う場合はヘルプを参照してください				*****
********************************************************************/

#pragma dtc_info 	4,256,0x000,0x0,0x17fff			// DTC情報、詳しくはヘルプを参照してください
#define DTCVBR		0x82404							// DTCVBRレジスタアドレス

/********************************************************************
*****	DTCVBR設定												*****
*****		引数	設定アドレス								*****
*****		戻り値	なし										*****
********************************************************************/

void ___mm_set_dtcvbr( unsigned long* vbr )
{
	*((volatile unsigned long*)DTCVBR) = (unsigned long)vbr ;
}

//----------	ユーザが変更するのはここまでです ---------------------
//----------	以下のコードは変更の必要はありません	--------------















#if defined(__YIDE_REM_DEBUG__) || defined(__YIDE_MON_MAKE__) || defined(__YIDE_LOAD_MAKE__)
#asmb {
INCLUDE (YIDESYM.DEF)
__MAX_VECTNO__	MAX_VECTNO+32		/*	割り込みベクタ数（固定割り込みベクターは続けて配置）	*/
}
#else
#asmb {
INCLUDE (YIDESYM.DEF)
__MAX_VECTNO__	MAX_VECTNO			/*	割り込みベクタ数										*/
}
#endif

// 無条件に呼び出す。
#deflib "USB\YDRVUSB_RX_01.OBJ"					/* ルネサス内蔵ＵＳＢ用ライブラリの指定		*/

#if ( DBG_PORT == USB_R )
#if !( defined(__YIDE_MON_MAKE__) || defined(__YIDE_ROM_DEBUG__) )
#deflib "USB\YDRVUSB_RX_01.OBJ"					/* ルネサス内蔵ＵＳＢ用ライブラリの指定		*/
#endif
#endif

#ifdef __YIDE_ROM_DEBUG__
#error "ROMデバッグはできません"
#endif

#ifdef __YIDE_MON_MAKE__
#ifndef	__TEXT_RAM__
#if( DBG_PORT == USB_R )
#deflib "REM-MON\RX600_02\REMRX.LIB"		/*  内蔵ＵＳＢ用リモートデバッグ用モニタライブラリの指定	*/
#else
#deflib "REM-MON\RX600_00\REMRX.LIB"		/* リモートデバッグ用モニタライブラリの指定					*/
#endif
#else
#if( DBG_PORT == USB_R )
#error "内蔵ＳＲＡＭ動作するＵＳＢモニターはありません"
#else
#deflib "REM-MON\RX600_80\REMRX.LIB"		/* リモートデバッグ用モニタライブラリの指定					*/
#endif
#endif	/*　__TEXT_RAM__　		*/
#endif	/* __YIDE_MON_MAKE__	*/

#ifdef __YIDE_LOAD_MAKE__
#if( DBG_PORT == USB_R )
#deflib "LOADER\RX600_01\LOADRX.LIB"		/* 内蔵ＵＳＢ用ローダー用モニタライブラリの指定				*/
#else
#deflib "LOADER\RX600_00\LOADRX.LIB"		/* ローダー用モニタライブラリの指定							*/
#endif
#endif	/* __YIDE_LOAD_MAKE__	*/

#define	NULL	((void *)0)

#ifdef	__INT_RAM__
#undef	RAM_BASE
#define	RAM_BASE		0x0000000		//内蔵SRAMの先頭番地を記述してください。下位１２ビットは常に０になるアドレスにしてください。
#endif

#ifdef __SUB_PROJECT__
#define	_USTACK_TOP	_SYSTEM_USTACK_TOP
#define	_USTACK_BTM	_SYSTEM_USTACK_BTM
#define	_STACK_TOP	_SYSTEM_STACK_TOP
#define	_STACK_BTM	_SYSTEM_STACK_BTM
extern long _SYSTEM_USTACK_TOP[];
extern long _SYSTEM_STACK_TOP[];
#asmb {
DEFINE vector_table_top system_vector_table_top
}
#endif

extern	void	___mm_monitor_stack ;
extern	void	___mm_monitor_istack ;

#if defined(__YIDE_MON_MAKE__) || defined(__YIDE_LOAD_MAKE__)
#rambase	RAM_BASE
#endif

__vect_table__ {

//--------------------------------------------------------------------
//　割込みベクタテーブル
//　変更しないで下さい YellowIDEが設定します。
//--------------------------------------------------------------------

#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__) || (defined __YIDE_SIM_DEBUG__))
#endif

#ifdef __YIDE_ROM_DEBUG__				// ROMデバッグ時
#if( DBG_PORT != USB_R )
	DBG_SCI_RXINT , ___mm_sci_int       // SCI受信割り込み　変更しないで下さい
#endif
#endif
#if( DBG_PORT != USB_R )
#ifdef __YIDE_MON_MAKE__	 			//モニター構築時
	DBG_SCI_RXINT , ___mm_sci_int       // SCI受信割り込み　変更しないで下さい
#endif
#endif
}

#ifdef __YIDE_REM_DEBUG__	// リモートデバッグ時
#if(DBG_PORT==USB_R)
#asmb {
	__EXC_VECTNO__	13		// 未定義固定割り込みベクター検出用
	__EXC_VECTNO__	14		// 未定義割り込みベクター検出用
	__EXC_VECTNO__	15		// システムコール用
	__DBG_VECTTBL__
}
#else
#asmb {
	__EXC_VECTNO__	13		// 未定義固定割り込みベクター検出用
	__EXC_VECTNO__	14		// 未定義割り込みベクター検出用
	__EXC_VECTNO__	15		// システムコール用
	__EXC_VECTNO__	DBG_SCI_RXINT
	__DBG_VECTTBL__
}
#endif
#endif	/* __YIDE_REM_DEBUG__ */

#ifdef	__YIDE_RAM__		// ダウンローダー
#asmb {
	__DBG_VECTTBL__
}
#endif

//-----------------ベクタテーブル終わり-------------------------------

static void (*dp)(void);
extern int  _main(int, int);
extern void __exit(void);
extern void __exit2(int);
extern void monitor(void);
extern unsigned char DATA_rom, DATA_start;
extern unsigned long DATA_size;
extern unsigned int  __mm_dtc_vect_addr ;
extern unsigned char __Heapbase[];
extern void _WriteSP(long *);
extern void __init_intb__(void);
extern unsigned char * const __smr_adr[];
extern void ___mm_putc_yscope(unsigned char);
extern int  ___mm_getc_yscope(void);

void __CSTARTUP_INIT2__(void)
{
//----------	以下のコードは変更の必要はありません	--------------
//=====================================================================
//	シリアルの初期設定
//=====================================================================
	for (temp = 1; temp < 8; temp++) {
		dp = (void (*)(void))__smr_adr[temp];
		if (dp != NULL) {
			dp();
		}
	}
	dp = (void (*)(void))__smr_adr[0];
	if (dp != NULL) dp();
//---------------------------------------------------------------------
//	INTBレジスタの設定（割り込みベクタのベースアドレス
//---------------------------------------------------------------------

#ifndef __SUB_PROJECT__
#ifndef	__YIDE_MON_MAKE__
	__init_intb__();
#endif	/* __YIDE_MON_MAKE__ 	*/
#endif	/* __SUB_PROJECT__		*/

//---------------------------------------------------------------------
//	データセグメントのコピー
//---------------------------------------------------------------------
#if ((defined __YIDE_ROM__) || (defined __YIDE_ROM_DEBUG__) || (defined __YIDE_SIM_DEBUG__) || (defined __YIDE_REM_DEBUG__))
	p1 = (unsigned char *)&DATA_rom;
	p2 = (unsigned char *)&DATA_start;
	for (temp = 0; temp < (unsigned long)&DATA_size; temp++) {
		*p2++ = *p1++;
	}
#endif

//--------------------------------------------------------------------
//	ヒープ領域の初期化
//--------------------------------------------------------------------
#if HEAP_SIZE
	for (temp = 0; temp < HEAP_SIZE+8; temp++) {
		__Heapbase[temp++] = 0;
	}
#endif

//---------------------------------------------------------------------
//	DTCVBRレジスタの設定（DTCベクタのベースアドレス)
//---------------------------------------------------------------------

	___mm_set_dtcvbr( (unsigned long*)&__mm_dtc_vect_addr ) ;

//---------------------------------------------------------------------
//	モニタをコール
//---------------------------------------------------------------------

#ifdef __YIDE_ROM_DEBUG__
	monitor();
#endif

//---------------------------------------------------------------------
//	main関数を呼ぶ
//---------------------------------------------------------------------
	temp = _main(0,0);

//--------------------------------------------------------------------
//		mainから戻った後の処理
//	ラベル__exitは、exit、raise関数で必要
//--------------------------------------------------------------------
	__exit2(temp);
}

//====================================================================
//		TEXTセグメント　（関数）
// 終了処理関数
//====================================================================

void __exit(void)
{
#if	defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
	_asm	MOV		#0, R15
	_asm	INT		#15
	_asm	DC.B	0xC8
#else
	while (1);
#endif
}

void __exit2(int ret)
{
#if	defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
	_asm	INT		#15
	_asm	DC.B	0xC8
#else
	while (1);
#endif
}

//====================================================================
//		TEXTセグメント　（関数）
// スタックがオーバーフローした時の処理を書く
//====================================================================

void _stack_over_task(void)
{
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
	_asm	MOV		SP, R1		// R1 = ErrorFuncSP
	_asm	INT		#15			// R2 = ErrorFuncPtr
	_asm	DC.B	0xCC
#else
	extern void _puts(const char *);
	extern void _TransitSuperMode(void);
//------------------------------------------------------------------------
	_TransitSuperMode();
_asm	MVTC	#_STACK_TOP+4,ISP			//割り込みスタック
_asm	MVTC	#_USTACK_TOP+4,USP			//ユーザスタック
	_puts("スタックオーバーフロー");
//------------------------------------------------------------------------
	__exit();			//終了

	//注意　この関数はリターンできません。
#endif
}

//--------------------------------------------------------------------
//	デバイスドライバ切り分け
//--------------------------------------------------------------------
#if (DEVICE_COUNT < 2)
#define	DEV_INIT1	NULL
#define	DEV_IN1		NULL
#define	DEV_IND1	NULL
#define	DEV_OUT1	NULL
#define	DEV_FLS1	NULL
#define	DEV_EI1		NULL
#define	DEV_DI1		NULL
#else
#define DEV_INIT1	___mm_sio_init1
#define	DEV_IN1		___mm_sio_in1
#define	DEV_IND1	___mm_sio_ind1
#define	DEV_OUT1	___mm_sio_out1
#define	DEV_FLS1	___mm_sio_flush1
#define	DEV_EI1		___mm_sio_ei1
#define	DEV_DI1		___mm_sio_di1
#endif
#if (DEVICE_COUNT < 3)
#define	DEV_INIT2	NULL
#define	DEV_IN2		NULL
#define	DEV_IND2	NULL
#define	DEV_OUT2	NULL
#define	DEV_FLS2	NULL
#define	DEV_EI2		NULL
#define	DEV_DI2		NULL
#else
#define DEV_INIT2	___mm_sio_init2
#define	DEV_IN2		___mm_sio_in2
#define	DEV_IND2	___mm_sio_ind2
#define	DEV_OUT2	___mm_sio_out2
#define	DEV_FLS2	___mm_sio_flush2
#define	DEV_EI2		___mm_sio_ei2
#define	DEV_DI2		___mm_sio_di2
#endif
#if (DEVICE_COUNT < 4)
#define	DEV_INIT3	NULL
#define	DEV_IN3		NULL
#define	DEV_IND3	NULL
#define	DEV_OUT3	NULL
#define	DEV_FLS3	NULL
#define	DEV_EI3		NULL
#define	DEV_DI3		NULL
#else
#define DEV_INIT3	___mm_sio_init3
#define	DEV_IN3		___mm_sio_in3
#define	DEV_IND3	___mm_sio_ind3
#define	DEV_OUT3	___mm_sio_out3
#define	DEV_FLS3	___mm_sio_flush3
#define	DEV_EI3		___mm_sio_ei3
#define	DEV_DI3		___mm_sio_di3
#endif
#if (DEVICE_COUNT < 5)
#define	DEV_INIT4	NULL
#define	DEV_IN4		NULL
#define	DEV_IND4	NULL
#define	DEV_OUT4	NULL
#define	DEV_FLS4	NULL
#define	DEV_EI4		NULL
#define	DEV_DI4		NULL
#else
#define DEV_INIT4	___mm_sio_init4
#define	DEV_IN4		___mm_sio_in4
#define	DEV_IND4	___mm_sio_ind4
#define	DEV_OUT4	___mm_sio_out4
#define	DEV_FLS4	___mm_sio_flush4
#define	DEV_EI4		___mm_sio_ei4
#define	DEV_DI4		___mm_sio_di4
#endif
#if (DEVICE_COUNT < 6)
#define	DEV_INIT5	NULL
#define	DEV_IN5		NULL
#define	DEV_IND5	NULL
#define	DEV_OUT5	NULL
#define	DEV_FLS5	NULL
#define	DEV_EI5		NULL
#define	DEV_DI5		NULL
#else
#define DEV_INIT5	___mm_sio_init5
#define	DEV_IN5		___mm_sio_in5
#define	DEV_IN5		___mm_sio_ind5
#define	DEV_OUT5	___mm_sio_out5
#define	DEV_FLS5	___mm_sio_flush5
#define	DEV_EI5		___mm_sio_ei5
#define	DEV_DI5		___mm_sio_di5
#endif

//--------------------------------------------------------------------
//	標準入出力関数の入出力先の割り当て
//--------------------------------------------------------------------

unsigned char * const __smr_adr[] = {
//初期化
//	___mm_sio_init0,				//stdin
	NULL,							//stdin
	NULL,							//stdout
	NULL,							//stderr
	DEV_INIT1,						//std_sio1
	DEV_INIT2,						//std_sio2
	DEV_INIT3,						//std_sio3
	DEV_INIT4,						//std_sio4
	DEV_INIT5,						//std_sio5
//入力
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
	___mm_getc_yscope,				// stdin
#else
//	___mm_sio_in0,					// stdin
	NULL,							// stdin
#endif
	NULL,							//stdout
	NULL,							//stderr
	DEV_IN1,						//std_sio1
	DEV_IN2,						//std_sio2
	DEV_IN3,						//std_sio3
	DEV_IN4,						//std_sio4
	DEV_IN5,						//std_sio5
//ポーリングなし入力
//	___mm_sio_ind0,					//stdin
	NULL,							//stdin
	NULL,							//stdout
	NULL,							//stderr
	DEV_IND1,						//std_sio1
	DEV_IND2,						//std_sio2
	DEV_IND3,						//std_sio3
	DEV_IND4,						//std_sio4
	DEV_IND5,						//std_sio5
//出力
	NULL,							//stdin
#if defined(__YIDE_YSCOPE__) || defined(__YIDE_MON_MAKE__)
	___mm_putc_yscope,				//stdout
	___mm_putc_yscope,				//stderr
#else
		//	___mm_sio_out0,					//stdout
		//	___mm_sio_out0,					//stderr
	NULL,							//stderr
	NULL,							//stderr
#endif
	DEV_OUT1,						//std_sio1
	DEV_OUT2,						//std_sio2
	DEV_OUT3,						//std_sio3
	DEV_OUT4,						//std_sio4
	DEV_OUT5,						//std_sio5
//フラッシュ
	___mm_sio_flush0,				//stdin
	___mm_sio_flush0,				//stdout
	___mm_sio_flush0,				//stderr
	DEV_FLS1,						//std_sio1
	DEV_FLS2,						//std_sio2
	DEV_FLS3,						//std_sio3
	DEV_FLS4,						//std_sio4
	DEV_FLS5,						//std_sio5
//受信割り込み許可
	___mm_sio_ei0,					//stdin
	___mm_sio_ei0,					//stdout
	___mm_sio_ei0,					//stderr
	DEV_EI1,						//std_sio1
	DEV_EI2,						//std_sio2
	DEV_EI3,						//std_sio3
	DEV_EI4,						//std_sio4
	DEV_EI5,						//std_sio5
//受信割り込み不許可
	___mm_sio_di0,					//stdin
	___mm_sio_di0,					//stdout
	___mm_sio_di0,					//stderr
	DEV_DI1,						//std_sio1
	DEV_DI2,						//std_sio2
	DEV_DI3,						//std_sio3
	DEV_DI4,						//std_sio4
	DEV_DI5,						//std_sio5
//予約1
	NULL,							//stdin
	NULL,							//stdout
	NULL,							//stderr
	NULL,							//std_sio1
	NULL,							//std_sio2
	NULL,							//std_sio3
	NULL,							//std_sio4
	NULL,							//std_sio5
//予約2
	NULL,							//stdin
	NULL,							//stdout
	NULL,							//stderr
	NULL,							//std_sio1
	NULL,							//std_sio2
	NULL,							//std_sio3
	NULL,							//std_sio4
	NULL,							//std_sio5
};

//--------------------------------------------------------------------
//	DATA_CONSTセグメント　（constがつくデータ）
//--------------------------------------------------------------------
#if HEAP_SIZE
const unsigned short __HeapSize = HEAP_SIZE;
#endif
#ifdef ECHO_BACK_ON
const char  __dbg_flag = 0x80;
#else
const char  __dbg_flag = 0xC0;
#endif

//--------------------------------------------------------------------
//		BSSセグメント　（constがつかないデータ）
//--------------------------------------------------------------------
#if RET_STRUCT_SIZE
unsigned char __struct_ret[RET_STRUCT_SIZE];
#endif

//--------------------------------------------------------------------
//		HEAPセグメント
//	malloc等の関数を使用しないのなら削除してください。
//--------------------------------------------------------------------
#if HEAP_SIZE
#pragma seg_bss HEAP
unsigned char __Heapbase[HEAP_SIZE + 8];	//8 = sizeof(HEADER)
#pragma seg_bss_end
#endif

//--------------------------------------------------------------------
//		スタックセグメント
//(変更しないで下さい）
//--------------------------------------------------------------------

#pragma seg_bss STACK				// 通常時
static char ___stack_free[64];		//余裕分
char _STACK_BTM[STACK_SIZE-4];
long _STACK_TOP[1];
#pragma seg_bss_end

#pragma seg_bss USTACK				// 通常時
static char ___ustack_free[64];		//余裕分
char _USTACK_BTM[USTACK_SIZE-4];
long _USTACK_TOP[1];
#pragma seg_bss_end

/*********************************************
 **											**
 **		固定割り込みベクター例外処理		**
 **											**
 *********************************************/

#if defined(__YIDE_MON_MAKE__) || defined(__YIDE_LOAD_MAKE__)
#asmb {
MACRO	EXC_EXE(no)
	//
	//	固定例外処理３２個はは割り込みベクターの２５６番から２８７番に再ジャンプするように設定されている
	//	そのベクターにあるアドレスへジャンプするマクロ
	//
	SUB		#4, SP						// ＲＴＳ用トラップ先アドレス確保
	PUSH	R7							// アドレス計算用一時レジスタースタック退避
	MVFC	INTB, R7					// 可変割り込みベクター先頭アドレス取得
	MOV.L	((no)+256)*4[R7],4[SP]		// 指定固定ベクター番号に２５６を足した値の４倍のアドレスを取得
										// 取得したジャンプ先を、最初にスタック上に確保したＲＴＳトラップアドレスに保存
	RTSD	#4, R7-R7					// ＲＴＳＤ命令でジャンプ
ENDM
}
#else
#asmb {
MACRO	EXC_EXE(no) LOCAL(L0)
L0:	BRA		L0
ENDM
}
#endif

#asmb {
	segment	TEXT
___mm_fi_nmi_def:
	EXC_EXE(30)

	segment	TEXT
___mm_fi_fpuexc_def:
	EXC_EXE(25)

	segment	TEXT
___mm_fi_undefinst_def:
	EXC_EXE(23)

	segment	TEXT
___mm_fi_accexc_def:
	EXC_EXE(21)

	segment	TEXT
___mm_fi_superexc_def:
	EXC_EXE(20)

	segment	TEXT
___mm_fi_undefexc_def:
	EXC_EXE( 8)
}

/*************************************
 **									**
 **		固定割り込みベクター		**
 **									**
 *************************************
 
 	ディフォルト定義
 	
*/

#asmb {
extern	___mm_fi_undefexc ___mm_fi_superexc ___mm_fi_accexc ___mm_fi_undefinst ___mm_fi_fpuexc ___mm_fi_nmi
}

#ifdef	__YIDE_ROM__
#asmb {
externdef	___mm_fi_undefexc 	___mm_fi_undefexc_def
externdef	___mm_fi_superexc 	___mm_fi_superexc_def
externdef	___mm_fi_accexc 	___mm_fi_accexc_def
externdef	___mm_fi_undefinst 	___mm_fi_undefinst_def
externdef	___mm_fi_fpuexc 	___mm_fi_fpuexc_def
externdef	___mm_fi_nmi		___mm_fi_nmi_def
}
#else
#asmb {
extern	___mm_fi_undefexc_yscope8 ___mm_fi_undefexc_yscope20 ___mm_fi_undefexc_yscope21 ___mm_fi_undefexc_yscope23 ___mm_fi_undefexc_yscope25 ___mm_fi_undefexc_yscope30

externdef	___mm_fi_undefexc 	___mm_fi_undefexc_yscope8	___mm_fi_undefexc_def
externdef	___mm_fi_superexc 	___mm_fi_undefexc_yscope20	___mm_fi_superexc_def
externdef	___mm_fi_accexc 	___mm_fi_undefexc_yscope21	___mm_fi_accexc_def
externdef	___mm_fi_undefinst 	___mm_fi_undefexc_yscope23	___mm_fi_undefinst_def
externdef	___mm_fi_fpuexc 	___mm_fi_undefexc_yscope25	___mm_fi_fpuexc_def
externdef	___mm_fi_nmi		___mm_fi_undefexc_yscope30	___mm_fi_nmi_def
}
#endif

#asmb {
	segment	FIX_INTV			// ATR_VECTは追加してはならない
	DC.L	MDES				// エンディアン選択レジスタS
	DC.L	___mm_fi_undefexc	// 1
	DC.L	OFS1				// オプション機能選択レジスタ1
	DC.L	OFS0				// オプション機能選択レジスタ0
	DC.L	___mm_fi_undefexc	// 4
	DC.L	___mm_fi_undefexc	// 5
	DC.L	___mm_fi_undefexc	// 6
	DC.L	___mm_fi_undefexc	// 7
	DC.L	___mm_fi_undefexc	// 8
	DC.L	___mm_fi_undefexc	// 9
	DC.L	___mm_fi_undefexc	// 10
	DC.L	___mm_fi_undefexc	// 11
	DC.L	___mm_fi_undefexc	// 12
	DC.L	___mm_fi_undefexc	// 13
	DC.L	___mm_fi_undefexc	// 14
	DC.L	___mm_fi_undefexc	// 15
	DC.L	___mm_fi_undefexc	// 16
	DC.L	___mm_fi_undefexc	// 17
	DC.L	___mm_fi_undefexc	// 18
	DC.L	___mm_fi_undefexc	// 19
	DC.L	___mm_fi_superexc	// 20	特権命令例外
	DC.L	___mm_fi_accexc		// 21	アクセス例外(RX610は無し）
	DC.L	___mm_fi_undefexc	// 22
	DC.L	___mm_fi_undefinst	// 23	未定義命令例外
	DC.L	___mm_fi_undefexc	// 24
	DC.L	___mm_fi_fpuexc		// 25	浮動小数点例外
	DC.L	___mm_fi_undefexc	// 26
	DC.L	___mm_fi_undefexc	// 27
	DC.L	___mm_fi_undefexc	// 28
	DC.L	___mm_fi_undefexc	// 29
	DC.L	___mm_fi_nmi		// 30	ＮＭＩ
	DC.L	START				// 31	リセット
}

//----------	以下イエロースコープ関係のルーチン	----------

//--------------------------------------------------------------------
//		デバッグポート識別子
//--------------------------------------------------------------------

const unsigned long     ___mm_RM_INTB 			= RAM_BASE ;
const unsigned char 	___mm_RMDebugPort 		= DBG_PORT | 0x80 ;
#if(DBG_PORT != USB_R )
const unsigned char  	___mm_RM_PortIntOff 	= DBG_SCI_RXINT ;
#endif

#ifdef	__LITTLE_ENDIAN__
#asmb {
__CPUOPT__ 0,2		// RX600系設定命令、リトルエンディアン
}
#else
#asmb {
__CPUOPT__ 0,3		// RX600系設定命令、ビックエンディアン
}
#endif

#ifdef	__TEXT_RAM__
#pragma	seg_text_end
#pragma	seg_const_end
#undef	TEXT
#endif	/* __TEXT_RAM__ */

#ifndef	__CPU_RX600__
#error "CPUの種類がRX600ではありません"
#endif

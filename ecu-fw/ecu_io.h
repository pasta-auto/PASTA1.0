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
// $RCSfile: ecu_io.h,v $
// $Revision: 1.00 $
// $Date: 2016/12/15 14:14:48 $
// 
// Copyright (c) 2016 LandF Corporation.
//
// History:
//


#ifndef		__CAN_ECU_IO_MAP__
#define		__CAN_ECU_IO_MAP__

//#define		EX_IO_MAX	64

typedef	union	__ext_io_memory__
{
	int				INTE;
	unsigned long	LONG;
	unsigned short	WORD[2];
	unsigned char	BYTE[4];
	struct	{
		unsigned long	B31	:	1;
		unsigned long	B30	:	1;
		unsigned long	B29	:	1;
		unsigned long	B28	:	1;
		unsigned long	B27	:	1;
		unsigned long	B26	:	1;
		unsigned long	B25	:	1;
		unsigned long	B24	:	1;
		unsigned long	B23	:	1;
		unsigned long	B22	:	1;
		unsigned long	B21	:	1;
		unsigned long	B20	:	1;
		unsigned long	B19	:	1;
		unsigned long	B18	:	1;
		unsigned long	B17	:	1;
		unsigned long	B16	:	1;
		unsigned long	B15	:	1;
		unsigned long	B14	:	1;
		unsigned long	B13	:	1;
		unsigned long	B12	:	1;
		unsigned long	B11	:	1;
		unsigned long	B10	:	1;
		unsigned long	B9	:	1;
		unsigned long	B8	:	1;
		unsigned long	B7	:	1;
		unsigned long	B6	:	1;
		unsigned long	B5	:	1;
		unsigned long	B4	:	1;
		unsigned long	B3	:	1;
		unsigned long	B2	:	1;
		unsigned long	B1	:	1;
		unsigned long	B0	:	1;
	}	BIT;
}	EX_IO_MEM;

typedef	union	__ext_io_status__
{
	EX_IO_MEM	DATA[EX_IO_MAX];
	struct	{
		//	0	インパネ：PCSマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	PCS;
		//	1	インパネ：エンジンマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_ENG;
		//	2	インパネ：黄（！）マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_Y;
		//	3	インパネ：ヘッドライト上下調整マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_HLUDC;
		//	4	インパネ：(ABS)マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_ABS;
		//	5	インパネ：リアフォグマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_RFOG;
		//	6	インパネ：バッテリーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_BATT;
		//	7	インパネ：赤（！）マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_R;
		//	8	インパネ：ドア開マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_DOP;
		//	9	インパネ：フロントフォグマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_FFOG;
		//	10	インパネ：エアバッグマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_ABG;
		//	11	インパネ：温度計マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_ETMP;
		//	12	インパネ：ハンドル！マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_RUD;
		//	13	インパネ：ポジション（尾灯）マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_POSL;
		//	14	インパネ：セキュリティーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_SCQ;
		//	15	インパネ：ハイビームマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_HIB;
		//	16	インパネ：自動速度マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_ATS;
		//	17	インパネ：SETマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_SET;
		//	18	インパネ：車＋メーターマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_ATC;
		//	19	インパネ：シートベルトマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_SB;
		//	20	インパネ：スリップマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_SLP;
		//	21	インパネ：故障！マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	EMG_R;
		//	22	インパネ：READYマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_READY;
		//	23	インパネ：左ウィンカーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_LBRINKER;
		//	24	インパネ：PWR MODEマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_PWRMODE;
		//	25	インパネ：ECO MODEマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_ECOMODE;
		//	26	インパネ：EV MODEマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_EVMODE;
		//	27	インパネ：右ウィンカーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_RBRINKER;
		//	28	インパネ：燃費(km/L)マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	DTP_KMPL;
		//	29	インパネ：時速(km/h)マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	DTP_KMPH;
		//	30	インパネ：サイドブレーキマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_SIDEBK;
		//	31	インパネ：アクセルマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_ACC;
		//	32	インパネ：シフトポジションマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_SHIFTPOS;
		//	33	インパネ：燃費ゲージ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_KMPL;
		//	34	インパネ：燃料ゲージ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_FUIEL;
		//	35	インパネ：速度表示
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_SPEED;
		//	36	インパネ：パッシングスイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_PASSING;
		//	37	インパネ：ポジション（尾灯）スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_POSITION;
		//	38	インパネ：ロービームスイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_LOWBEAM;
		//	39	インパネ：ハイビームスイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_HIGHBEAM;
		//	40	インパネ：アクセル踏み込み量[%]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_ACC;
		//	41	インパネ：ブレーキ踏み込み量[%]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_BRK;
		//	42	インパネ：ハンドル位置[±%]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_RUD;
		//	43	インパネ：エンジン温度[℃]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_ETMP;
		//	44	インパネ：右ドアウィンドウ位置[%]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_RDWP;
		//	45	インパネ：左ドアウィンドウ位置[%]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_LDWP;
		//	46	インパネ：右ドアロック・アンロック[ON,OFF]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_RLOCK;
		//	47	インパネ：左ドアロック・アンロック[ON,OFF]
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_LLOCK;
		//	48	インパネ：フロントウォッシャー液吐出スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_FWASH;
		//	49	インパネ：フロントワイパー駆動スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_FWIPER;
		//	50	インパネ：リアウォッシャー液吐出スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_RWASH;
		//	51	インパネ：リアワイパー駆動スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_RWIPER;
		//	52	インパネ：右ドアウィンドウ上下スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_RWUD;
		//	53	インパネ：左ドアウィンドウ上下スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_LWUD;
		//	54	インパネ：エンジン回転数
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_ERPM;
		//	55	インパネ：チャイルドロックマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_CHALOCK;
		//	56	インパネ：ハンドル目標位置（パワステ）
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SYS_RUDTG;
		//	57	インパネ：ハンドルトルク（パワステ）
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SYS_RUDTQ;
		//	58	インパネ：エンジンスタートスイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_ESTART;
		//	59	インパネ：燃費自動算出フラグ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	FLG_ATKMPL;
		//	60	インパネ：燃料自動消費フラグ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	FLG_ATFL;
		//	61	インパネ：PWR・ECO・EVモード自動表示フラグ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	FLG_ATRMODE;
		//	62	インパネ：デモンストレーション走行許可フラグ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	FLG_ATDEMO;
		//	63	インパネ：表示モード番号
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SYS_DISPMODE;
		//	64	インパネ：制御切り替え入力スイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SYS_CNTSEL;
		//	65	インパネ：シフトレバーUP/DOWNスイッチ
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	SW_SHIFTUD;
		//	66	未使用
		unsigned long	RES_66;
		//	67	未使用
		unsigned long	RES_67;
		//	68	未使用
		unsigned long	RES_68;
		//	69	未使用
		unsigned long	RES_69;
		//	70	未使用
		unsigned long	RES_70;
		//	71	未使用
		unsigned long	RES_71;
		//	72	未使用
		unsigned long	RES_72;
		//	73	未使用
		unsigned long	RES_73;
		//	74	未使用
		unsigned long	RES_74;
		//	75	未使用
		unsigned long	RES_75;
		//	76	未使用
		unsigned long	RES_76;
		//	77	未使用
		unsigned long	RES_77;
		//	78	未使用
		unsigned long	RES_78;
		//	79	インパネ：間欠ワイパー間隔ボリューム
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	VAL_FWINT;
		//	80	インパネ：フロントワイパーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_FW;
		//	81	インパネ：フロントワイパーAUTOマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_FWAT;
		//	82	インパネ：フロントワイパーウォッシュマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_FWWASH;
		//	83	インパネ：フロントウォッシャーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_FWASH;
		//	84	インパネ：リアワイパーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_RW;
		//	85	インパネ：リアワイパーAUTOマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_RWAT;
		//	86	インパネ：リアワイパーウォッシュマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_RWWASH;
		//	87	インパネ：リアウォッシャーマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_RWASH;
		//	88	未使用
		unsigned long	RES_88;
		//	89	未使用
		unsigned long	RES_89;
		//	90	インパネ：ボンネット開マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_BCOP;
		//	91	インパネ：トランク開マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_TROP;
		//	92	未使用
		unsigned long	RES_92;
		//	93	未使用
		unsigned long	RES_93;
		//	94	インパネ：ホーンマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_HORN;
		//	95	インパネ：電球（〇）マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_LMP;
		//	96	インパネ：電球（！）マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	WAR_LMP;
		//	97	インパネ：電球（×）マーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	EMG_LMP;
		//	98	インパネ：Bluetoothマーク
		union	{
			unsigned long	L;
			struct	{
				unsigned long	VAL		:	16;		//	
				unsigned long	OUT		:	8;		//	
				unsigned long	IN		:	8;		//	
			}	BIT;
		}	INF_BTT;
		//	99	未使用
		unsigned long	RES_99;
		//	以下、未使用
		unsigned long	RES_XX[28];
	}	UNIT;
}	EXT_IO_STATUS;

//	外部入出力機器状態
extern	EXT_IO_STATUS	exiosts;

//	デジタルビット入力
#define		X_DB_0		PORTC.PIDR.BIT.B0	/*	CN6-19(IRQ14)	PC0	*/
#define		X_DB_1		PORTC.PIDR.BIT.B1	/*	CN6-20(IRQ12)	PC1	*/
#define		X_DB_2		PORTC.PIDR.BIT.B2	/*	CN6-21(SMISO5)	PC2	*/
#define		X_DB_3		PORTC.PIDR.BIT.B3	/*	CN6-22(SMOSI5)	PC3	*/
#define		X_DB_4		PORTC.PIDR.BIT.B4	/*	CN6-23(SCK5)	PC4	*/
#define		X_DB_5		PORTC.PIDR.BIT.B5	/*	CN6-24(SS0)		PC5	*/
#define		X_DB_6		PORTC.PIDR.BIT.B6	/*	CN6-25(SS1)		PC6	*/
#define		X_DB_7		PORT6.PIDR.BIT.B4	/*	CN6-26(EXP)		P64	*/

#define		X_DB_8		PORTB.PIDR.BIT.B2	/*	P1-1 (D8)		PB2	*/
#define		X_DB_9		PORTB.PIDR.BIT.B4	/*	P1-2 (D9)		PB4	*/
#define		X_DB_10		PORTB.PIDR.BIT.B5	/*	P1-3 (D10)		PB5	*/
#define		X_DB_11		PORTB.PIDR.BIT.B1	/*	P1-4 (D11)		PB1	*/
#define		X_DB_12		PORTB.PIDR.BIT.B0	/*	P1-5 (D12)		PB0	*/
#define		X_DB_13		PORTB.PIDR.BIT.B3	/*	P1-6 (D13)		PB3	*/
#define		X_DB_14		PORTB.PIDR.BIT.B7	/*	P1-9 (SDA)		PB7	*/
#define		X_DB_15		PORTB.PIDR.BIT.B6	/*	P1-10(SCL)		PB6	*/

#define		X_DB_16		PORTA.PIDR.BIT.B2	/*	P3-1 (D0)		PA2	*/
#define		X_DB_17		PORTA.PIDR.BIT.B4	/*	P3-2 (D1)		PA4	*/
#define		X_DB_18		PORTA.PIDR.BIT.B0	/*	P3-3 (D2)		PA0	*/
#define		X_DB_19		PORTA.PIDR.BIT.B1	/*	P3-4 (D3)		PA1	*/
#define		X_DB_20		PORTA.PIDR.BIT.B5	/*	P3-5 (D4)		PA5	*/
#define		X_DB_21		PORTA.PIDR.BIT.B3	/*	P3-6 (D5)		PA3	*/
#define		X_DB_22		PORTA.PIDR.BIT.B6	/*	P3-7 (D6)		PA6	*/
#define		X_DB_23		PORTA.PIDR.BIT.B7	/*	P3-8 (D7)		PA7	*/

//	デジタルビット出力
#define		Y_DB_0		PORTC.PODR.BIT.B0	/*	CN6-19(IRQ14)	PC0	*/
#define		Y_DB_1		PORTC.PODR.BIT.B1	/*	CN6-20(IRQ12)	PC1	*/
#define		Y_DB_2		PORTC.PODR.BIT.B2	/*	CN6-21(SMISO5)	PC2	*/
#define		Y_DB_3		PORTC.PODR.BIT.B3	/*	CN6-22(SMOSI5)	PC3	*/
#define		Y_DB_4		PORTC.PODR.BIT.B4	/*	CN6-23(SCK5)	PC4	*/
#define		Y_DB_5		PORTC.PODR.BIT.B5	/*	CN6-24(SS0)		PC5	*/
#define		Y_DB_6		PORTC.PODR.BIT.B6	/*	CN6-25(SS1)		PC6	*/
#define		Y_DB_7		PORT6.PODR.BIT.B4	/*	CN6-26(EXP)		P64	*/

#define		Y_DB_8		PORTB.PODR.BIT.B2	/*	P1-1 (D8)		PB2	*/
#define		Y_DB_9		PORTB.PODR.BIT.B4	/*	P1-2 (D9)		PB4	*/
#define		Y_DB_10		PORTB.PODR.BIT.B5	/*	P1-3 (D10)		PB5	*/
#define		Y_DB_11		PORTB.PODR.BIT.B1	/*	P1-4 (D11)		PB1	*/
#define		Y_DB_12		PORTB.PODR.BIT.B0	/*	P1-5 (D12)		PB0	*/
#define		Y_DB_13		PORTB.PODR.BIT.B3	/*	P1-6 (D13)		PB3	*/
#define		Y_DB_14		PORTB.PODR.BIT.B7	/*	P1-9 (SDA)		PB7	*/
#define		Y_DB_15		PORTB.PODR.BIT.B6	/*	P1-10(SCL)		PB6	*/

#define		Y_DB_16		PORTA.PODR.BIT.B2	/*	P3-1 (D0)		PA2	*/
#define		Y_DB_17		PORTA.PODR.BIT.B4	/*	P3-2 (D1)		PA4	*/
#define		Y_DB_18		PORTA.PODR.BIT.B0	/*	P3-3 (D2)		PA0	*/
#define		Y_DB_19		PORTA.PODR.BIT.B1	/*	P3-4 (D3)		PA1	*/
#define		Y_DB_20		PORTA.PODR.BIT.B5	/*	P3-5 (D4)		PA5	*/
#define		Y_DB_21		PORTA.PODR.BIT.B3	/*	P3-6 (D5)		PA3	*/
#define		Y_DB_22		PORTA.PODR.BIT.B6	/*	P3-7 (D6)		PA6	*/
#define		Y_DB_23		PORTA.PODR.BIT.B7	/*	P3-8 (D7)		PA7	*/

//	アナログ入力
#define		X_AN_0		S12AD.ADDR0			/*	CN6-15(AN000)	P40	*/
#define		X_AN_1		S12AD.ADDR1			/*	CN6-16(AN001)	P41	*/
#define		X_AN_2		S12AD.ADDR2			/*	P2-1  (AN002)	P42	*/
#define		X_AN_3		S12AD.ADDR3			/*	P2-2  (AN003)	P43	*/
#define		X_AN_4		S12AD.ADDR4			/*	P2-3  (AN004)	P44	*/
#define		X_AN_5		S12AD.ADDR5			/*	P2-4  (AN005)	P45	*/
#define		X_AN_6		S12AD.ADDR6			/*	P2-5  (AN006)	P46	*/
#define		X_AN_7		S12AD.ADDR7			/*	P2-6  (AN007)	P47	*/

//	アナログ出力
#define		Y_AN_0		DA.DADR0			/*	CN6-17(DA0)		P03	*/
#define		Y_AN_1		DA.DADR1			/*	CN6-18(DA1)		P05	*/

#endif		/*__CAN_ECU_IO_MAP__*/

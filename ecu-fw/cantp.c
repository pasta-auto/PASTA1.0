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
//	CAN-TP トランスポート層プロトコル処理
//
//----------------------------------------------------------------------------------------
//	開発履歴
//
//	2017/08/01	コーディング開始（橘）
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
#include "timer.h"
#include "ecu.h"			/*	ECU 共通定義			*/
#include "can3_spi2.h"		/*	CAN3 定義				*/
#include "cantp.h"			/*	CAN-TP 定義				*/
#include "obd2.h"			/*	CAN-OBDII 定義			*/
#include "uds.h"			/*	CAN-UDS 定義			*/

/*
	CAN-TP の概要

	TPはシングル又は複数パケットを連結→上位層に渡す→上位層から受け取り→シングル又は複数パケットに分解して転送する。

	CAN-TPフレーム定義
	
	適用
	ブロードキャスト	CAN-ID	0x7DF
	指定ECU向け			CAN-ID	0x7E0～0x7E7
	指定ECU応答			CAN-ID	0x7E8～0x7EF
	
	要求フレーム
		
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Byte			|			0			|			1			|			2			|			3			|			4			|			5			|			6			|			7			|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Bit(LE)			|	7..4	|	3..0	|		  15..8			|		  23..16		|		  31..24		|						|						|						|						|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Single			|	Type 0	| Size 0..7	|		 Data A			|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|		  Data G		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		First			|	Type 1	| Size 8..4095						|		  Data A		|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Consecutive		|	Type 2	|Index 0..15|		 Data A			|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|		  Data G		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Flow			|	Type 3	|FCflag0,1,2|		Block Size		|			ST			|						|						|						|						|						|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		予備			|	4..15	|	0..15	|		 Data A			|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|		  Data G		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+

	応答(要求CAN-ID + 0x008)
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Byte			|			0			|			1			|			2			|			3			|			4			|			5			|			6			|			7			|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Bit(LE)			|	7..4	|	3..0	|		  15..8			|		  23..16		|		  31..24		|						|						|						|						|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Single			|	Type 0	| Size 0..7	|		 Data A			|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|		  Data G		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		First			|	Type 1	| Size 8..4095						|		  Data A		|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Consecutive		|	Type 2	|Index 0..15|		 Data A			|		  Data B		|		  Data C		|		  Data D		|		  Data E		|		  Data F		|		  Data G		|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Flow			|	Type 3	|FCflag0,1,2|		Block Size		|			ST			|						|						|						|						|						|
						+-----------+-----------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
	

*/
#if	0
//----------------------------------------------------------------------------------------
//	TYPEコード定義
//----------------------------------------------------------------------------------------
#define		CAN_TP_SINGLE			0x00		/*	シングルフレーム								*/
#define		CAN_TP_FIRST			0x01		/*	マルチフレームの先頭							*/
#define		CAN_TP_CONT				0x02		/*	マルチフレームの継続							*/
#define		CAN_TP_FLOW				0x03		/*	フロー制御										*/

//----------------------------------------------------------------------------------------
//	タイマーID設定
//----------------------------------------------------------------------------------------
#define		TP_TIMER_ID				1			/*	分離時間タイマーID								*/
#define		DTC_TIMER_ID			2			/*	DTC継続時間タイマーID							*/

//----------------------------------------------------------------------------------------
//	モードフラグビット合成
//----------------------------------------------------------------------------------------
#define		CANTP_MODE_RECV			1			/*	TP動作モード(受信中)							*/
#define		CANTP_MODE_SEND			2			/*	TP動作モード(送信中)							*/
#define		CANTP_MODE_WFL			4			/*	TP動作モード(フロー応答待ち)					*/
#define		CANTP_MODE_WTE			8			/*	TP動作モード(送信完了待ち)						*/
#define		CANTP_MODE_WTU			16			/*	TP動作モード(タイムアップ待ち)					*/

//----------------------------------------------------------------------------------------
//	バッファサイズ
//----------------------------------------------------------------------------------------
#define		CAN_TP_BUF_SIZE		256

//	マルチフレームバッファ型定義
typedef	struct	__can_tp_buffer__
{
	int				RPOS;	//	読み出し位置
	int				WPOS;	//	書き込み位置
	unsigned char	BUF[CAN_TP_BUF_SIZE];	//	バッファ
}	CAN_TP_BUF;

//	シングルフレーム型定義
typedef	union	__can_tp_single__
{
	unsigned long	L[2];
	unsigned short	W[4];
	unsigned char	B[8];
	struct	{
		union	{
			unsigned char	BYTE;
			struct	{
				unsigned char	CODE	:	4;	//	フレームタイプ(0)
				unsigned char	SIZE	:	4;	//	データバイト数(0～7)
			}	HEAD;
		}	PCI;
		unsigned char	DATA[7];	//	データ
	}	FRAME;
}	TP_FRM_SINGL;

//	マルチフレーム先頭型定義
typedef	union	__can_tp_first__
{
	unsigned long	L[2];
	unsigned short	W[4];
	unsigned char	B[8];
	struct	{
		union	{
			unsigned char	BYTE;
			struct	{
				unsigned char	CODE	:	4;	//	フレームタイプ(1)
				unsigned char	SIZE	:	4;	//	データバイト数上位4ビット(0～F)
			}	HEAD;
		}	PCI;
		unsigned char	SIZEL;		//	データバイト数下位8ビット(00～FF)
		unsigned char	DATA[6];	//	データ
	}	FRAME;
}	TP_FRM_FIRST;

//	マルチフレーム後続型定義
typedef	union	__can_tp_consec__
{
	unsigned long	L[2];
	unsigned short	W[4];
	unsigned char	B[8];
	struct	{
		union	{
			unsigned char	BYTE;
			struct	{
				unsigned char	CODE	:	4;	//	フレームタイプ(2)
				unsigned char	INDEX	:	4;	//	フレームインデックス番号(0～15)
			}	HEAD;
		}	PCI;
		unsigned char	DATA[7];	//	データ
	}	FRAME;
}	TP_FRM_CONSEC;

//	マルチフレームフロー制御型定義
typedef	union	__can_tp_flow__
{
	unsigned long	L[2];
	unsigned short	W[4];
	unsigned char	B[8];
	struct	{
		union	{
			unsigned char	BYTE;
			struct	{
				unsigned char	CODE	:	4;	//	フレームタイプ(3)
				unsigned char	FC		:	4;	//	フロー制御(0=送信許可,1=WAIT,2=オーバーフロー)
			}	HEAD;
		}	PCI;
		unsigned char	BS;			//	ブロックサイズ(0=受信遅延無し,1～受信可能フレーム数)
		unsigned char	ST;			//	フレーム分割時間(0～127msec,0xF1～0xF9=100～900msec)
		unsigned char	DATA[5];	//	無効
	}	FRAME;
}	TP_FRM_FLOW;

//	フレーム共用体定義
typedef	union	__can_tp_frame__
{
	unsigned long	L[2];
	unsigned short	W[4];
	unsigned char	B[8];
	TP_FRM_SINGL	SINGLE;		//	シングルフレーム
	TP_FRM_FIRST	FIRST;		//	マルチフレーム先頭
	TP_FRM_CONSEC	CONSEC;		//	マルチフレーム後続
	TP_FRM_FLOW		FLOW;		//	マルチフレームフロー制御
}	CAN_TP_FRAME;

//	TP制御バッファ型定義
typedef	struct	__can_tp_packet__
{
	int				MODE;	//	TP動作モード(0:要求待ち,ビット合成(1:受信中,2:送信中,4:フロー応答待ち,8:送信完了待ち,16:タイムアップ待ち))
	int				TIME;	//	DTC継続時間(0=解除,0<継続中)※定期的に継続要求を受け更新する
	int				CH;		//	CANポート番号
	int				ID;		//	CAN Frame ID
	int				INDEX;	//	マルチフレームインデックス番号
	int				SIZE;	//	マルチフレームデータサイズ
	int				BC;		//	ブロックカウンタ
	int				FC;		//	フロー制御(0=送信許可,1=WAIT,2=オーバーフロー)
	int				BS;		//	ブロックサイズ(0=受信遅延無し,1～受信可能フレーム数)
	int				ST;		//	フレーム分割時間(0～127msec,0xF1～0xF9=100～900msec)
	int				TXIF;	//	送信完了処理要求フラグ
	int				TXID;	//	送信CAN-ID保持
	CAN_TP_FRAME	RXF;	//	受信フレーム
	CAN_TP_FRAME	TXF;	//	送信フレーム
	CAN_TP_BUF		RXD;	//	受信バッファ
	CAN_TP_BUF		TXD;	//	送信バッファ
}	CAN_TP_PACK;
#endif
CAN_TP_PACK		tp_pack;	//	TP制御変数

//----------------------------------------------------------------------------------------
//	CAN-TP変数初期化
//----------------------------------------------------------------------------------------
void	can_tp_init(void)
{
	memset(&tp_pack, 0, sizeof(CAN_TP_PACK));
}

//----------------------------------------------------------------------------------------
//	CAN-TP データ積み上げ処理
//----------------------------------------------------------------------------------------
int	can_tp_build(unsigned char *dp, int sz)
{
	int	i;
	int	f = 0;
	//	データ積み上げ
	if(sz > 0)
	{	//	データ有り
		for(i = 0; i < sz && tp_pack.RXD.WPOS < tp_pack.SIZE; i++)
		{
			tp_pack.RXD.BUF[tp_pack.RXD.WPOS++] = *dp++;
		}
		if(tp_pack.RXD.WPOS == tp_pack.SIZE)
		{	//	全データ受信完了
			if(tp_pack.RXD.BUF[0] < 0x10)
			{	//	OBD2プロトコル
				f = obd2_job(tp_pack.RXD.BUF, tp_pack.RXD.WPOS, tp_pack.TXD.BUF);
			}
			else
			{	//	UDSプロトコル
				f = uds_job(tp_pack.RXD.BUF, tp_pack.RXD.WPOS, tp_pack.TXD.BUF);
			}
			if(f > 0)
			{
				tp_pack.TXD.RPOS = 0;
				tp_pack.TXD.WPOS = f;
			}
		}
	}
	return f;
}

//----------------------------------------------------------------------------------------
//	CAN-TP データ送信処理
//----------------------------------------------------------------------------------------
int	can_tp_send(void)
{
	int				i, sz;
	unsigned char	*dp;
	//	データ積み上げ
	if(tp_pack.TXD.RPOS < tp_pack.TXD.WPOS)
	{	//	データ有り
		memset(&tp_pack.TXF, 0, sizeof(CAN_TP_FRAME));
		if(tp_pack.TXD.WPOS < 8)
		{	//	シングルフレームで送信
			tp_pack.TXF.SINGLE.FRAME.PCI.HEAD.CODE = CAN_TP_SINGLE;
			tp_pack.TXF.SINGLE.FRAME.PCI.HEAD.SIZE = tp_pack.TXD.WPOS;
			tp_pack.SIZE = tp_pack.TXD.WPOS;
			tp_pack.INDEX = 1;
			tp_pack.BC = 0;
			tp_pack.MODE = CANTP_MODE_SEND;
			dp = tp_pack.TXF.SINGLE.FRAME.DATA;
			sz = 7;
		}
		else
		if(tp_pack.TXD.RPOS == 0)
		{	//	マルチフレームで送信、先頭
			tp_pack.TXF.FIRST.FRAME.PCI.HEAD.CODE = CAN_TP_FIRST;
			tp_pack.TXF.FIRST.FRAME.PCI.HEAD.SIZE = (tp_pack.TXD.WPOS >> 8) & 0x0F;
			tp_pack.TXF.FIRST.FRAME.SIZEL = tp_pack.TXD.WPOS & 0xFF;
			tp_pack.SIZE = tp_pack.TXD.WPOS;
			tp_pack.BC = 0;
			tp_pack.INDEX = 1;
			tp_pack.MODE = CANTP_MODE_SEND | CANTP_MODE_WFL;	//	フロー待ち
			dp = tp_pack.TXF.FIRST.FRAME.DATA;
			sz = 6;
		}
		else
		{	//	マルチフレームで送信、継続
			tp_pack.TXF.CONSEC.FRAME.PCI.HEAD.CODE = CAN_TP_CONT;
			tp_pack.TXF.CONSEC.FRAME.PCI.HEAD.INDEX = tp_pack.INDEX++;
			tp_pack.INDEX &= 15;
			tp_pack.BC++;
			tp_pack.MODE = CANTP_MODE_SEND | CANTP_MODE_WTE;	//	送信完了待ち
			if(tp_pack.BC >= tp_pack.BS && tp_pack.BS > 0)
			{	//	連続ブロック数到達
				tp_pack.BC = 0;
				tp_pack.FC = CANTP_FC_WAIT;
				tp_pack.MODE |= CANTP_MODE_WFL;	//	フロー待ち
			}
			dp = tp_pack.TXF.CONSEC.FRAME.DATA;
			sz = tp_pack.TXD.WPOS - tp_pack.TXD.RPOS;
			if(sz > 7) sz = 7;
		}
		//	データコピー
		for(i = 0; i < sz && tp_pack.TXD.RPOS < tp_pack.TXD.WPOS; i++)
		{
			*dp++ = tp_pack.TXD.BUF[tp_pack.TXD.RPOS++];
		}
		if(tp_pack.TXD.RPOS == tp_pack.SIZE)
		{	//	全データ送信完了
			tp_pack.MODE = 0;
		}
		return 1;	//	送信有り
	}
	return 0;	//	送信無し
}

//----------------------------------------------------------------------------------------
//	CAN-TP フロー送信処理
//----------------------------------------------------------------------------------------
int	can_tp_flow(void)
{
	//	データ積み上げ
	memset(&tp_pack.TXF, 0, sizeof(CAN_TP_FRAME));
	tp_pack.TXF.FLOW.FRAME.PCI.HEAD.CODE = CAN_TP_FLOW;	//	フローコントロール
	tp_pack.TXF.FLOW.FRAME.PCI.HEAD.FC = CANTP_FC_CTS;	//	送信許可
	tp_pack.TXF.FLOW.FRAME.BS = 1;						//	ブロックサイズ(0=連続)
	tp_pack.TXF.FLOW.FRAME.ST = 0;						//	フレーム分割時間(1ms)
	tp_pack.BS = tp_pack.TXF.FLOW.FRAME.BS;
	tp_pack.BC = 0;
	tp_pack.MODE = 0;
	return 1;
}

//----------------------------------------------------------------------------------------
//	継続送信処理
//----------------------------------------------------------------------------------------
void	can_tp_consecutive(void)
{
	int				id = SELECT_ECU_UNIT + 0x7E8;
	int				f = 0;
	
	switch(tp_pack.FC)
	{
	case CANTP_FC_CTS:	//	送信許可
		if(tp_pack.MODE & CANTP_MODE_SEND)
		{	//	送信中
			if(tp_pack.MODE & CANTP_MODE_WTE)
			{	//	送信完了待ち
				return;
			}
			if(tp_pack.MODE & CANTP_MODE_WTU)
			{
				if(tp_pack.ST > 0)
				{	//	タイマー有効
					if(check_timer(TP_TIMER_ID) == 0)
					{	//	タイムアップ待ち
						return;
					}
				}
				tp_pack.MODE ^= CANTP_MODE_WTU;
			}
			//	継続送信フレーム生成
			f = can_tp_send();
		}
		break;
	case CANTP_FC_WAIT:	//	許可待ち
		if(tp_pack.ST > 0)
		{	//	タイマー有効
			if(check_timer(TP_TIMER_ID))
			{	//	タイムオーバー
				tp_pack.MODE = 0;
				return;
			}
		}
		break;
	case CANTP_FC_ABORT:	//	中止
		tp_pack.MODE = 0;	//	待機
		tp_pack.CH = -1;
		tp_pack.ID = -1;
		break;
	}
	if(f != 0)
	{	//	返信
		memcpy(&can_buf.ID[id], tp_pack.TXF.B, 8);
		add_mbox_frame(tp_pack.CH, 8, CAN_DATA_FRAME, id);	//	送信待ちバッファ積み上げ
	}
}

//----------------------------------------------------------------------------------------
//	CAN-TP送信完了処理(mainから呼び出す)
//----------------------------------------------------------------------------------------
void	can_tp_txendreq(void)
{
	if(tp_pack.ST > 0)
	{	//	次の送信までの時間設定
		if(tp_pack.FC == 0 && (tp_pack.BS > tp_pack.BC || tp_pack.BS == 0))
		{	//	送信許可
			tp_pack.MODE |= CANTP_MODE_WTU;	//	タイムアップ待ち
			after_call(TP_TIMER_ID, tp_pack.ST, can_tp_consecutive);	//	タイマーセット
		}
	}
	else
	{	//	分離時間無し
		if(tp_pack.FC == 0 && (tp_pack.BS > tp_pack.BC || tp_pack.BS == 0))
		{	//	送信許可
			can_tp_consecutive();
		}
	}
}

//----------------------------------------------------------------------------------------
//	CAN-TP送信完了待ち解除チェック
//----------------------------------------------------------------------------------------
void	can_tp_txecheck(int ch, int id)
{
	if(tp_pack.CH == ch && tp_pack.TXID == id)
	{
		if(tp_pack.MODE & CANTP_MODE_WTE)
		{	//	送信完了待ち
			tp_pack.MODE ^= CANTP_MODE_WTE;	//	待ち解除
			tp_pack.TXIF = 1;				//	送信完了処理要求フラグ
		}
	}
}

//----------------------------------------------------------------------------------------
//	CAN-TP処理
//----------------------------------------------------------------------------------------
int can_tp_job(int ch, int id, void *frame)
{
	int				sw = SELECT_ECU_UNIT + 0x7E0;
	int				f = 0;
	int				sz, i;
	unsigned char	*dp;
	
	if(id != 0x7DF && id != sw) return 0;
	
	memcpy(&tp_pack.RXF, frame, sizeof(CAN_TP_FRAME));
	memset(&tp_pack.TXF, 0x00, sizeof(CAN_TP_FRAME));
	
	switch(tp_pack.RXF.SINGLE.FRAME.PCI.HEAD.CODE)
	{
	case CAN_TP_SINGLE:	//	シングルフレーム
		if(tp_pack.MODE == 0)
		{	//	待機中
			sz = tp_pack.RXF.SINGLE.FRAME.PCI.HEAD.SIZE;
			tp_pack.CH = ch;
			tp_pack.ID = id;
			tp_pack.INDEX = 0;
			tp_pack.SIZE = sz;
			memset(&tp_pack.RXD, 0, sizeof(CAN_TP_BUF));
			dp = tp_pack.RXF.SINGLE.FRAME.DATA;
			f = can_tp_build(dp, sz);
			if(f > 0)
			{	//	送信処理
				if(can_tp_send() == 0) f = 0;
			}
		}
		break;
	case CAN_TP_FIRST:	//	マルチ先頭フレーム
		if(tp_pack.MODE == 0)
		{	//	待機中
			sz = (tp_pack.RXF.FIRST.FRAME.PCI.HEAD.SIZE) << 8;
			sz |= (tp_pack.RXF.FIRST.FRAME.SIZEL) & 0xFF;
			tp_pack.CH = ch;
			tp_pack.ID = id;
			tp_pack.INDEX = 1;
			tp_pack.SIZE = sz;
			memset(&tp_pack.RXD, 0, sizeof(CAN_TP_BUF));
			dp = tp_pack.RXF.FIRST.FRAME.DATA;
			sz = 6;
			f = can_tp_build(dp, sz);
			if(f > 0)
			{	//	送信処理
				if(can_tp_send() == 0) f = 0;
			}
			else
			{	//	応答送信
				f = can_tp_flow();
			}
		}
		break;
	case CAN_TP_CONT:	//	マルチ継続フレーム
		if(tp_pack.MODE == 0)
		{	//	待機中
			if(tp_pack.CH == ch && tp_pack.ID == id)
			{	//	ポート、ID一致
				i = tp_pack.RXF.CONSEC.FRAME.PCI.HEAD.INDEX;
				if(i != tp_pack.INDEX)
				{	//	インデックスが一致しない
					tp_pack.TXF.FLOW.FRAME.PCI.HEAD.CODE = CAN_TP_CONT;			//	フロー制御
					tp_pack.TXF.FLOW.FRAME.PCI.HEAD.FC = CANTP_FC_ABORT;		//	中止
					tp_pack.TXF.FLOW.FRAME.BS = 0;								//	
					tp_pack.TXF.FLOW.FRAME.ST = 0;								//	
					tp_pack.TXF.FLOW.FRAME.DATA[0] = tp_pack.INDEX;				//	現在のインデックス
					f = 1;
				}
				else
				{	//	インデックス一致
					tp_pack.BC++;
					tp_pack.INDEX++;
					tp_pack.INDEX &= 15;
					dp = tp_pack.RXF.CONSEC.FRAME.DATA;
					sz = 7;
					f = can_tp_build(dp, sz);
					if(f > 0)
					{	//	送信処理
						if(can_tp_send() == 0) f = 0;
					}
					else
					if(tp_pack.BC >= tp_pack.BS && tp_pack.BS > 0)
					{	//	ブロック数到達　フロー制御送信
						tp_pack.TXF.FLOW.FRAME.PCI.HEAD.CODE = CAN_TP_FLOW;	//	フローコントロール
						tp_pack.TXF.FLOW.FRAME.PCI.HEAD.FC = CANTP_FC_CTS;	//	送信許可
						tp_pack.TXF.FLOW.FRAME.BS = tp_pack.BS;				//	ブロックサイズ(0=連続)
						tp_pack.TXF.FLOW.FRAME.ST = 0;						//	フレーム分割時間(1ms)
						tp_pack.BC = 0;
						f = 1;
					}
				}
			}
		}
		break;
	case CAN_TP_FLOW:	//	フロー制御フレーム受信
		if(tp_pack.MODE & CANTP_MODE_WFL)
		{	//	フロー待機中
			tp_pack.MODE ^= CANTP_MODE_WFL;	//	待機解除
			if(tp_pack.CH == ch && tp_pack.ID == id)
			{	//	ポート、ID一致
				//	フロー制御
				tp_pack.BC = 0;
				tp_pack.BS = 0;
				tp_pack.ST = 0;
				tp_pack.FC = tp_pack.RXF.FLOW.FRAME.PCI.HEAD.FC;
				switch(tp_pack.FC)
				{
				case CANTP_FC_CTS:		//	送信継続
					tp_pack.BS = tp_pack.RXF.FLOW.FRAME.BS;		//	ブロックサイズ
					if(tp_pack.BS > 0)
					{	//	指定ブロック数連続＋遅延有効
						tp_pack.ST = tp_pack.RXF.FLOW.FRAME.ST;		//	分離時間(ms)
						if(tp_pack.ST > 0xF0) tp_pack.ST = (tp_pack.ST - 0xF0) * 100;	//	100～900ms
					}
					f = can_tp_send();
					break;
				case CANTP_FC_WAIT:		//	送信待ち
					tp_pack.MODE |= CANTP_MODE_WFL | CANTP_MODE_WTU;
					tp_pack.BS = -1;	//	送信禁止
					tp_pack.ST = 10000;	//	タイムアウト規定(10sec)
					after_call(TP_TIMER_ID, tp_pack.ST, can_tp_consecutive);
					break;
				default:				//	エラー中止(オーバーフロー、アボート)
					tp_pack.MODE = 0;	//	待機
					tp_pack.CH = -1;
					tp_pack.ID = -1;
					break;
				}
			}
		}
		break;
	default:	//	未対応モード
		return 0;
	}
	if(tp_pack.TXF.B[0] != 0 && f != 0)
	{	//	応答返信
		sw += 8;
		tp_pack.TXID = sw;
		memcpy(&can_buf.ID[sw], tp_pack.TXF.B, 8);
		if(ch >= 0) add_mbox_frame(ch, 8, CAN_DATA_FRAME, sw);	//	送信待ちバッファ積み上げ
		return sw;
	}
	return 0;
}




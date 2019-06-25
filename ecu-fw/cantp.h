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

//#include "cantp.h"			/*	CAN-TP 定義				*/

#ifndef		__CAN_TRANSE_PORT_PROTOCOL__
#define		__CAN_TRANSE_PORT_PROTOCOL__

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
//#define		TP_TIMER_ID				1			/*	分離時間タイマーID								*/
//#define		DTC_TIMER_ID			2			/*	DTC継続時間タイマーID							*/

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
#define		CANTP_FC_CTS		0	/*	送信許可				*/
#define		CANTP_FC_WAIT		1	/*	送信待ち				*/
#define		CANTP_FC_OVER		2	/*	バッファオーバーフロー	*/
#define		CANTP_FC_ABORT		2	/*	中止					*/
#define		CANTP_FC_INDEX		3	/*	インデックス不一致		*/

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

extern	CAN_TP_PACK		tp_pack;	//	TP制御変数

//----------------------------------------------------------------------------------------
//	CAN-TP変数初期化
//----------------------------------------------------------------------------------------
extern	void	can_tp_init(void);
//----------------------------------------------------------------------------------------
//	CAN-TP データ積み上げ処理
//----------------------------------------------------------------------------------------
extern	int	can_tp_build(unsigned char *dp, int sz);
//----------------------------------------------------------------------------------------
//	CAN-TP データ送信処理
//----------------------------------------------------------------------------------------
extern	int	can_tp_send(void);
//----------------------------------------------------------------------------------------
//	継続送信処理
//----------------------------------------------------------------------------------------
extern	void	can_tp_consecutive(void);
//----------------------------------------------------------------------------------------
//	CAN-TP送信完了処理
//----------------------------------------------------------------------------------------
extern	void	can_tp_txendreq(void);
//----------------------------------------------------------------------------------------
//	CAN-TP送信完了待ち解除チェック
//----------------------------------------------------------------------------------------
extern	void	can_tp_txecheck(int ch, int id);
//----------------------------------------------------------------------------------------
//	CAN-TP処理
//----------------------------------------------------------------------------------------
extern	int can_tp_job(int ch, int id, void *frame);

#endif		/*__CAN_TRANSE_PORT_PROTOCOL__*/


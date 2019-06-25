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
//	OBD2-CAN プロトコル処理
//
//----------------------------------------------------------------------------------------
//	開発履歴
//
//	2017/02/13	コーディング開始（橘）
//
//----------------------------------------------------------------------------------------
//	T.Tachibana
//	㈱L&F
//________________________________________________________________________________________
//

//	#include "obd2.h"			/*	CAN-OBDII 定義			*/

#ifndef		__CAN_OBD2_PROTOCOL__
#define		__CAN_OBD2_PROTOCOL__

/*
	OBD2 処理の概要

	OBD2は

	CANフレーム定義
	
	ブロードキャスト
	CAN-ID	0x7DF
	
					ECUクエリ受信(0x7DF)※すべてのECUが処理対象となる
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		PID-type / Byte	|			0			|			1			|			2			|			3			|			4			|			5			|			6			|			7			|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		SAE standard	|Number of additional	|Mode					|PID code				|not used																												|
						| data bytes:2			|01=show current data	|(e.g.:05=Engine coolant|(may be 55h)																											|
						|						|02=freeze frame		| temperature)			|																														|
						|						|etc.					|						|																														|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Vehicle speific	|Number of additional	|Custom mode:			|PID code										|not used																						|
						| data bytes:3			|(e.g.:22=enhanced data	|(e.g.:4980h)									|(may be 00h or 55h)																			|
						|						|						|												|																								|
						|						|						|												|																								|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
	
	マルチフレーム
	CAN-ID	0x7E0	ECU0(パワトレ系)への要求
			0x7E1	ECU1(ボディー系)への要求
			0x7E2	ECU2(シャシー系)への要求
			0x7E3	ECU3(CGW-DTC)への要求
			
					ECUクエリ受信(0x7E0～7E7)※個別ECUに対するOBD2要求
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		PID-type / Byte	|			0			|			1			|			2			|			3			|			4			|			5			|			6			|			7			|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		SAE standard	|Number of additional	|Mode					|PID code				|not used																												|
						| data bytes:2			|01=show current data	|(e.g.:05=Engine coolant|(may be 55h)																											|
						|						|02=freeze frame		| temperature)			|																														|
						|						|etc.					|						|																														|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Vehicle speific	|Number of additional	|Custom mode:			|PID code										|not used																						|
						| data bytes:3			|(e.g.:22=enhanced data	|(e.g.:4980h)									|(may be 00h or 55h)																			|
						|						|						|												|																								|
						|						|						|												|																								|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
	
	CAN-ID	0x7E8	ECU0(パワトレ系)からの応答
			0x7E9	ECU1(ボディー系)からの応答
			0x7EA	ECU2(シャシー系)からの応答
			0x7EB	ECU3(CGW-DTC)からの応答
	
					ECUクエリ送信(0x7E8～7EF)
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		PID-type / Byte	|			0			|			1			|			2			|			3			|			4			|			5			|			6			|			7			|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		SAE standard	|Number of additional	|Custom mode (+40h)		|PID code				|value of the			|value.					|value.					|value.					|not used				|
						| data bytes:3 to 6		|41=show current data	|(e.g.:05=Engine coolant|specified parameter.	|byte 1					|byte 2					|byte 3					|(may be 00h or 55h)	|
						|						|42=freeze frame		| temperature)			|byte 0					|(optional)				|(optional)				|(optional)				|						|
						|						|etc.					|						|						|						|						|						|						|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Vehicle speific	|Number of additional	|Custom mode (+40h)		|PID code										|value of the			|value.					|value.					|value.					|
						| data bytes:4 to 7		|(e.g.:62h=response to	|(e.g.:4980h)									|specified parameter.	|byte 1					|byte 2					|byte 3					|
						|						| mode 22h request)		|												|byte 0					|(optional)				|(optional)				|(optional)				|
						|						|						|												|						|						|						|						|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
		Vehicle speific	|Number of additional	|7Fh this a general		|Custom mode:			|31h					|not used																						|
						| data bytes:3			|response usually		|(e.g.:22h=enhanced 	|						|(may be 00h)																					|
						|						|indicating the module	|diagnostic data by PID,|						|																								|
						|						| doesn't recognize the	|21h=enhanced data by	|						|																								|
						|						| request.				|offset					|						|																								|
						+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
	

*/

//----------------------------------------------------------------------------------------
//	OBD2パケット共用体定義
//----------------------------------------------------------------------------------------
typedef	union	__obd2_query_frame__	{
	unsigned char	BYTE[8];
	struct	{
		unsigned char	MODE;		//	モード
		unsigned char	PID;		//	パラメータ番号
		unsigned char	NU[6];		//	未使用[55h]
	}	SAE_OBD;
	struct	{
		unsigned char	MODE;		//	モード（0～A）
		unsigned char	PIDH;		//	パラメータ番号上位8ビット
		unsigned char	PIDL;		//	パラメータ番号下位8ビット
		unsigned char	NU[5];		//	未使用[55h]
	}	VS_OBD;
	struct	{
		unsigned char	MODE;		//	モード(+40h)
		unsigned char	PID;		//	パラメータ番号
		unsigned char	VAL[5];		//	パラメータ値
		unsigned char	NU;			//	未使用[00h or 55h]
	}	SAE_ECU;
	struct	{
		unsigned char	MODE;		//	モード(+40h)
		unsigned char	PIDH;		//	パラメータ番号上位8ビット
		unsigned char	PIDL;		//	パラメータ番号下位8ビット
		unsigned char	VAL[5];		//	パラメータ値
	}	VS_ECU;
	struct	{
		unsigned char	MODE;		//	モード(+40h)
		unsigned char	VAL[7];		//	パラメータ値
	}	MD3_ECU;
	struct	{
		unsigned char	MODE;		//	モード(+40h)
		unsigned char	NU[7];		//	パラメータ値
	}	MD4_ECU;
	struct	{
		unsigned char	X7F;		//	未対応要求コード：固定[7Fh]
		unsigned char	MODE;		//	受信モード
		unsigned char	X31;		//	固定[31h]
		unsigned char	NU[5];		//	未使用[00h]
	}	NOT_ECU;
}	OBD2_QUERY_FRAME;

//----------------------------------------------------------------------------------------
//	MODEコード定義
//----------------------------------------------------------------------------------------
#define		SHOW_CURRENT_DATA		0x01		/*	現在の値取得									*/
#define		SHOW_FREEZE_FDATA		0x02		/*	一時停止フレームの取得							*/
#define		SHOW_STORED_DTC			0x03		/*	DTC(Diagnostic Trouble Codes)の取得(診断ログ)	*/
#define		CLEAR_STORED_DTC		0x04		/*	明確なDTC情報									*/
#define		TEST_RESULT_NON_CAN		0x05		/*	試験結果（CANでは機能しない）					*/
#define		TEST_RESULT_ONLY_CAN	0x06		/*	試験結果（CANのみ酸素センサー監視）				*/
#define		SHOW_PENDING_DTC		0x07		/*	未知の診断トラブルコード情報取得				*/
#define		CTRL_OPERATION_SYS		0x08		/*	オンボードシステム支援							*/
#define		REQUEST_VEHICLE_INFO	0x09		/*	車両情報の取得									*/
#define		PERMANENT_DTC			0x0A		/*	永続DTC情報										*/

//----------------------------------------------------------------------------------------
//	診断トラブルコード(DTC)定義
//----------------------------------------------------------------------------------------
typedef	union	__obd2_dtc_str__	{
	unsigned char	BYTE[2];
	struct	{
		struct	{
			unsigned char	ECU		:	2;		//	1st DTC character	0:[P] Powertrain / 1:[C] Chassis / 2:[B] Body / 3:[U] Network
			unsigned char	CH		:	2;		//	2nd DTC character	0～3
			unsigned char	CL		:	4;		//	3rd DTC character	0～F
		}	A;
		struct	{
			unsigned char	CH		:	4;		//	4th DTC character	00～FF
			unsigned char	CL		:	4;		//	5th DTC character	00～FF
		}	B;
	}	BIT;
}	OBD2_DTC_STR;

#define		DTC_ECU_CODE_POW		0			/*	パワトレ診断コード								*/
#define		DTC_ECU_CODE_CHA		1			/*	シャシー診断コード								*/
#define		DTC_ECU_CODE_BDY		2			/*	ボディー診断コード								*/
#define		DTC_ECU_CODE_NET		3			/*	ネットワーク診断コード							*/

//----------------------------------------------------------------------------------------
//	変数定義
//----------------------------------------------------------------------------------------
extern	OBD2_QUERY_FRAME	obd2_req;	//	要求データ
extern	OBD2_QUERY_FRAME	obd2_ret;	//	応答データ

extern	int		obd2_ret_counter;

//----------------------------------------------------------------------------------------
//	MODE1	処理		0x0C,	0x0D,	0x1C,	0x2F,	0x31,	0x49,	0x51			
//----------------------------------------------------------------------------------------
extern	int obd2_mode1(int len);
//----------------------------------------------------------------------------------------
//	MODE2	処理(DTCレコード返信要求)
//----------------------------------------------------------------------------------------
extern	int obd2_mode2(int len);
//----------------------------------------------------------------------------------------
//	MODE3	処理(保存済みDTCレコードを返信する)
//----------------------------------------------------------------------------------------
extern	int obd2_mode3(int len);
//----------------------------------------------------------------------------------------
//	MODE4	処理(DTCレコードを消去する)
//----------------------------------------------------------------------------------------
extern	int obd2_mode4(int len);
//----------------------------------------------------------------------------------------
//	MODE5	処理
//----------------------------------------------------------------------------------------
extern	int obd2_mode5(int len);
//----------------------------------------------------------------------------------------
//	MODE6	試験結果（CANのみ酸素センサー監視）
//----------------------------------------------------------------------------------------
extern	int obd2_mode6(int len);
//----------------------------------------------------------------------------------------
//	MODE7	未知の診断トラブルコード情報取得
//----------------------------------------------------------------------------------------
extern	int obd2_mode7(int len);
//----------------------------------------------------------------------------------------
//	MODE8	オンボードシステム支援
//----------------------------------------------------------------------------------------
extern	int obd2_mode8(int len);
//----------------------------------------------------------------------------------------
//	MODE9	処理
//----------------------------------------------------------------------------------------
extern	int obd2_mode9(int len);
//----------------------------------------------------------------------------------------
//	MODE10	永続DTC情報
//----------------------------------------------------------------------------------------
extern	int obd2_modeA(int len);
//----------------------------------------------------------------------------------------
//	OBD2処理
//----------------------------------------------------------------------------------------
extern	int obd2_job(unsigned char *msg, int len, unsigned char *res);

#endif		/*__CAN_OBD2_PROTOCOL__*/

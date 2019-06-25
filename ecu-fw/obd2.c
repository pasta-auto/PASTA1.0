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

#include <sysio.h>
#include <string.h>
#include <stdio.h>
#include "iodefine.h"
#include "timer.h"
#include "ecu.h"			/*	ECU 共通定義			*/
#include "can3_spi2.h"		/*	CAN3 定義				*/
#include "cantp.h"			/*	CAN-TP 定義				*/
#include "obd2.h"			/*	CAN-OBDII 定義			*/

#if	0

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

#endif

//----------------------------------------------------------------------------------------
//	変数定義
//----------------------------------------------------------------------------------------
OBD2_QUERY_FRAME	obd2_req;	//	要求データ
OBD2_QUERY_FRAME	obd2_ret;	//	応答データ

int		obd2_ret_counter = 0;

//----------------------------------------------------------------------------------------
//	MODE1	処理		0x0C,	0x0D,	0x1C,	0x2F,	0x31,	0x49,	0x51			
//----------------------------------------------------------------------------------------
int obd2_mode1(int len)
{
	int	d;
	//	スタンダード要求
	if(len >= 2)
	{
		len = 2;								//	デフォルト2バイト
		obd2_ret.SAE_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;	//	応答フラグ
		obd2_ret.SAE_ECU.PID = obd2_req.SAE_OBD.PID;			//	パラメータIDコピー
		//	PID毎の処理
		switch(obd2_req.SAE_OBD.PID)
		{
		//------------------------------------------------------
		case 0x00:	//	サポートPID情報 [01 - 20]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x18;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x11;	//	0x0C,0x0D,0x1C,(0x20)
			break;
		case 0x01:	//	Monitor status since DTCs cleared. (Includes malfunction indicator lamp (MIL) status and number of DTCs.)
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;		//(A)	bit7:MIL発光チェック / bit6～0:DTC_CNT
			obd2_ret.SAE_ECU.VAL[1] = 0x00;		//(B)	bit7:[0] / bit3:0=点火プラグ式,1=ディーゼル / bit2:構成テスト,bit6:不完全 / bit1:燃料計テスト,bit5:不完全 /bit0:不発テスト,bit4:不完全
			obd2_ret.SAE_ECU.VAL[2] = 0x00;		//(C)	bit7:EGRシステムテスト / bit6:酸素センサー・ヒータテスト / bit5:酸素センサーテスト / bit4:AC冷媒テスト / bit3:第2空気システムテスト / bit2:蒸発システムテスト / bit1:加熱促進剤テスト / bit0:促進剤テスト
			obd2_ret.SAE_ECU.VAL[3] = 0x00;		//(D)	bit7:EGRシステム不完全 / bit6:酸素センサー・ヒータ不完全 / bit5:酸素センサー不完全 / bit4:AC冷媒不完全 / bit3:第2空気システム不完全 / bit2:蒸発システム不完全 / bit1:加熱促進剤不完全 / bit0:促進剤不完全
			break;
		case 0x02:	//	Freeze DTC (DTCの保存を停止する)
			break;
		case 0x03:	//	Fuel system status
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 2;	//	1:エンジン低温開ループ / 2:混合比フィードバック閉ループ / 4:減速閉ループ / 8:故障開ループ / 16:閉ループ欠陥
			break;
		case 0x04:	//	Calculated engine load
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～100[%]エンジントルク値		100/255*A
			break;
		case 0x05:	//	Engine coolant temperature
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = can_buf.ID[0x183].BYTE[0];	//	-40～215[℃]エンジン冷却剤温度	A-40
			break;
		case 0x06:	//	Short term fuel trim-Bank 1
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-100～99.2[%] 短期燃料バンク1	100/128*A-100
			break;
		case 0x07:	//	Long term fuel trim-Bank 1
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-100～99.2[%] 長期燃料バンク1	100/128*A-100
			break;
		case 0x08:	//	Short term fuel trim-Bank 2
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-100～99.2[%] 短期燃料バンク2	100/128*A-100
			break;
		case 0x09:	//	Long term fuel trim-Bank 2
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-100～99.2[%] 長期燃料バンク2	100/128*A-100
			break;
		case 0x0A:	//	Fuel pressure (gauge pressure)
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～765[kPa](/3) 燃料圧			3*A
			break;
		case 0x0B:	//	Intake manifold absolute pressure
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～255[kPa] 排気菅絶対圧		A
			break;
		case 0x0C:	//	Engine RPM
			len += 2;
			d = ((((int)can_buf.ID[0x043].BYTE[0]) << 8) | (((int)can_buf.ID[0x043].BYTE[1]) & 0xFF)) * 4;
			obd2_ret.SAE_ECU.VAL[0] = (unsigned char)(d >> 8);		//	0～16383.75[rpm] エンジン回転数	(256A+B)/4
			obd2_ret.SAE_ECU.VAL[1] = (unsigned char)(d & 0xFF);	//	↑下位8bit
			break;
		case 0x0D:	//	Vehicle speed
			len += 1;
			d = ((((int)can_buf.ID[0x043].BYTE[2]) << 8) | (((int)can_buf.ID[0x043].BYTE[3]) & 0xFF));
			if(d & 0x8000) d = 0x10000 - d;
			obd2_ret.SAE_ECU.VAL[0] = d;	//	0～255[km/h] 車速				A
			break;
		case 0x0E:	//	Timing advance
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-64～63.5[°] 前進タイミング	A/2-64
			break;
		case 0x0F:	//	Intake air temperature
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-40～215[℃] 吸気温度			A-40
			break;
		case 0x10:	//	 MAF air flow rate 
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～655.35[g/s] 混合気レート		(256A+B)/100
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	↑下位8bit
			break;
		case 0x11:	//	Throttle position
			len += 1;
			d = ((((int)can_buf.ID[0x02F].BYTE[0]) << 8) | (((int)can_buf.ID[0x02F].BYTE[1]) & 0xFF)) * 255 / 1023;
			obd2_ret.SAE_ECU.VAL[0] = (unsigned char)(d & 0xFF);	//	0～100[%] スロットル位置		100A/255
			break;
		case 0x12:	//	Commanded secondary air status
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 1;	//	1:上流 / 2:触媒下 / 4:外気 / 8:診断時
			break;
		case 0x13:	//	Oxygen sensors present (in 2 banks)
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 1;	//	bit0～3:Bank1,センサー1～4 / bit4～7:Bank2,センサー1～4	酸素センサー存在フラグ
			break;
		case 0x14:	//	Oxygen Sensor 1  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー1電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x15:	//	Oxygen Sensor 2  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー2電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x16:	//	Oxygen Sensor 3  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー3電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x17:	//	Oxygen Sensor 4  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー4電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x18:	//	Oxygen Sensor 5  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー5電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x19:	//	Oxygen Sensor 6  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー6電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x1A:	//	Oxygen Sensor 7  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー7電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x1B:	//	Oxygen Sensor 8  A: Voltage  B: Short term fuel trim
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	0～1.275[V] 酸素センサー8電圧	A/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//	-100～99.2[%] 短期燃料調整		100/128B-100
			break;
		case 0x1C:	//	OBD standards this vehicle conforms to
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 1;	//	1:CARB仕様のOBD2 / 11:JOBD,JOBD2 / 13:JOBD,EOBD,OBD2 他
			break;
		case 0x1D:	//	Oxygen sensors present (in 4 banks)
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 1;	//	bit0～1:Bank1,センサー1～2 / bit2～3:Bank2,センサー1～2 / bit4～5:Bank3,センサー1～2 / bit6～7:Bank4,センサー1～2	酸素センサー存在フラグ
			break;
		case 0x1E:	//	Auxiliary input status
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 1;	//	bit0:PTO(Power take Off) 1=有効,0=無効 / bit7～1:[0]	補助入力ステータス
			break;
		case 0x1F:	//	Run time since engine start
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～65535[秒] エンジンスタートからの経過時間	256*A+B
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		//------------------------------------------------------
		case 0x20:	//	PIDs supported [21 - 40]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x02;
			obd2_ret.SAE_ECU.VAL[2] = 0x80;
			obd2_ret.SAE_ECU.VAL[3] = 0x01;	//	0x2F,0x31,(0x40)
			break;
		case 0x21:	//	Distance traveled with malfunction indicator lamp (MIL) on
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～65535[km] MIL警報からの走行距離	256*A+B
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x22:	//	Fuel Rail Pressure (relative to manifold vacuum)
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～5177.265[kPa] 燃料レール圧	0.079*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x23:	//	Fuel Rail Gauge Pressure (diesel, or gasoline direct injection)
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～655350[kPa] 燃料レールゲージ圧	10*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x24:	//	Oxygen Sensor 1  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー1
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x25:	//	Oxygen Sensor 2  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー2
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x26:	//	Oxygen Sensor 3  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー3
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x27:	//	Oxygen Sensor 4  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー4
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x28:	//	Oxygen Sensor 5  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー5
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x29:	//	Oxygen Sensor 6  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー6
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x2A:	//	Oxygen Sensor 7  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー7
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x2B:	//	Oxygen Sensor 8  AB: Fuel-Air Equivalence Ratio CD: Voltage
			len += 2;		//	酸素センサー8
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	0～8[V] センサー電圧	8/65536*(256*C+D)
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x2C:	//	Commanded EGR
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] EGR命令		100/255*A
			break;
		case 0x2D:	//	EGR Error
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-100～99.2[%] EGRエラー	100/128*A-100
			break;
		case 0x2E:	//	Commanded evaporative purge
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] 気化量命令	100/255*A
			break;
		case 0x2F:	//	Fuel Tank Level Input
			len += 1;
			d = ((int)can_buf.ID[0x3D4].BYTE[0]) * 255 / 40;	//	0..40リットル
			if(d > 255) d = 255;
			obd2_ret.SAE_ECU.VAL[0] = (unsigned char)d;	//(A)	0～100[%] 燃料残量		100/255*A
			break;
		case 0x30:	//	Warm-ups since codes cleared
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～255[count] ウォームアップカウント	A
			break;
		case 0x31:	//	Distance traveled since codes cleared
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～65535[km] プリセット走行距離計	256*A+B
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x32:	//	Evap. System Vapor Pressure
			len += 2;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-8192～8191.75[Pa] システム蒸気圧	(256*A+B)/4
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x33:	//	Absolute Barometric Pressure
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～255[kPa] 絶対気圧	A
			break;
		case 0x34:	//	Oxygen Sensor 1  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー1
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x35:	//	Oxygen Sensor 2  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー2
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x36:	//	Oxygen Sensor 3  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー3
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x37:	//	Oxygen Sensor 4  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー4
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x38:	//	Oxygen Sensor 5  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー5
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x39:	//	Oxygen Sensor 6  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー6
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x3A:	//	Oxygen Sensor 7  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー7
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x3B:	//	Oxygen Sensor 8  AB: Fuel-Air Equivalence Ratio CD: Current
			len += 4;		//	酸素センサー8
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-128～128[mA] センサー電流	(256*C+D)/256-128
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	↑下位8bit
			break;
		case 0x3C:	//	Catalyst Temperature: Bank 1, Sensor 1
			len += 2;		//	促進剤温度バンク1センサー1
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-40～6513.5[℃] 温度	(256*A+B)/10-40
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x3D:	//	Catalyst Temperature: Bank 2, Sensor 1
			len += 2;		//	促進剤温度バンク2センサー1
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-40～6513.5[℃] 温度	(256*A+B)/10-40
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x3E:	//	Catalyst Temperature: Bank 1, Sensor 2
			len += 2;		//	促進剤温度バンク1センサー2
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-40～6513.5[℃] 温度	(256*A+B)/10-40
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x3F:	//	Catalyst Temperature: Bank 2, Sensor 2
			len += 2;		//	促進剤温度バンク2センサー2
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-40～6513.5[℃] 温度	(256*A+B)/10-40
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		//------------------------------------------------------
		case 0x40:	//	PIDs supported [41 - 60]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x80;
			obd2_ret.SAE_ECU.VAL[2] = 0x80;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;	//	0x49,0x51
			break;
		case 0x41:	//	Monitor status this drive cycle
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;		//(A)	[0]
			obd2_ret.SAE_ECU.VAL[1] = 0x00;		//(B)	bit7:[0] / bit3:0=点火プラグ式,1=ディーゼル / bit2:構成テスト,bit6:不完全 / bit1:燃料計テスト,bit5:不完全 /bit0:不発テスト,bit4:不完全
			obd2_ret.SAE_ECU.VAL[2] = 0x00;		//(C)	bit7:EGRシステムテスト / bit6:酸素センサー・ヒータテスト / bit5:酸素センサーテスト / bit4:AC冷媒テスト / bit3:第2空気システムテスト / bit2:蒸発システムテスト / bit1:加熱促進剤テスト / bit0:促進剤テスト
			obd2_ret.SAE_ECU.VAL[3] = 0x00;		//(D)	bit7:EGRシステム不完全 / bit6:酸素センサー・ヒータ不完全 / bit5:酸素センサー不完全 / bit4:AC冷媒不完全 / bit3:第2空気システム不完全 / bit2:蒸発システム不完全 / bit1:加熱促進剤不完全 / bit0:促進剤不完全
			break;
		case 0x42:	//	Control module voltage
			len += 2;		//	制御モジュール電圧
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～65.535[V] 電圧	(256*A+B)/1000
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x43:	//	Absolute load value
			len += 2;		//	絶対仕事量
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～25700[%] 仕事量	100/255*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x44:	//	Fuel-Air commanded equivalence ratio
			len += 2;		//	等価混合比指令
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～2[比率] 燃料混合比	2/65536*(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x45:	//	Relative throttle position
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] スロットル相対位置	100/255*A
			break;
		case 0x46:	//	Ambient air temperature
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-40～215[℃] 周囲温度	A-40
			break;
		case 0x47:	//	Absolute throttle position B
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] スロットル絶対位置B	100/255*A
			break;
		case 0x48:	//	Absolute throttle position C
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] スロットル絶対位置C	100/255*A
			break;
		case 0x49:	//	Accelerator pedal position D
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] アクセル位置D	100/255*A
			break;
		case 0x4A:	//	Accelerator pedal position E
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] アクセル位置E	100/255*A
			break;
		case 0x4B:	//	Accelerator pedal position F
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] アクセル位置F	100/255*A
			break;
		case 0x4C:	//	Commanded throttle actuator
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] スロットル起動指令	100/255*A
			break;
		case 0x4D:	//	Time run with MIL on
			len += 2;		//	MIL発報経過時間
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～65535[分] 経過時間	256*A+B
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x4E:	//	Time since trouble codes cleared
			len += 2;		//	警報解除からの経過時間
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～65535[分] 経過時間	256*A+B
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x4F:	//	Maximum value for Fuel-Air equivalence ratio, oxygen sensor voltage, oxygen sensor current, and intake manifold absolute pressure
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;		//(A)	0～255[比率] 混合気比率		A
			obd2_ret.SAE_ECU.VAL[1] = 0x00;		//(B)	0～255[V] 酸素センサー電圧	B
			obd2_ret.SAE_ECU.VAL[2] = 0x00;		//(C)	0～255[mA] 酸素センサー電流	C
			obd2_ret.SAE_ECU.VAL[3] = 0x00;		//(D)	0～2550[kPa] 吸気圧			D*10
			break;
		case 0x50:	//	Maximum value for air flow rate from mass air flow sensor
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;		//(A)	0～2550[g/s] 最大気流量		A*10
			obd2_ret.SAE_ECU.VAL[1] = 0x00;		//(B)	予備
			obd2_ret.SAE_ECU.VAL[2] = 0x00;		//(C)	予備
			obd2_ret.SAE_ECU.VAL[3] = 0x00;		//(D)	予備
			break;
		case 0x51:	//	Fuel Type
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 1;	//(A)	1:ガソリン / 2:メタノール / 3:エタノール / 4:ディーゼル / 5:LPG / 6:CNG / 7:プロパン / 8:電気 / 21:複合型継続電気と内燃機関(プリウス)
			break;
		case 0x52:	//	Ethanol fuel %
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] エタノール燃料	100/255*A
			break;
		case 0x53:	//	Absolute Evap system Vapor Pressure
			len += 2;		//	絶対システム蒸気圧
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～327.675[kPa] 気圧	(256*A+B)/200
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x54:	//	Evap system vapor pressure
			len += 2;		//	システム蒸気圧
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-32767～32767[Pa] 気圧	(256*A+B)-32767
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x55:	//	Short term secondary oxygen sensor trim, A: bank 1, B: bank 3
			len += 2;		//	短期第二酸素センサー調整
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-100～99.2[%] バンク1調整値		100/128*A-100
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	-100～99.2[%] バンク3調整値		100/128*A-100
			break;
		case 0x56:	//	Long term secondary oxygen sensor trim, A: bank 1, B: bank 3
			len += 2;		//	長期第二酸素センサー調整
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-100～99.2[%] バンク1調整値		100/128*A-100
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	-100～99.2[%] バンク3調整値		100/128*A-100
			break;
		case 0x57:	//	Short term secondary oxygen sensor trim, A: bank 2, B: bank 4
			len += 2;		//	短期第二酸素センサー調整
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-100～99.2[%] バンク2調整値		100/128*A-100
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	-100～99.2[%] バンク4調整値		100/128*A-100
			break;
		case 0x58:	//	Long term secondary oxygen sensor trim, A: bank 2, B: bank 4
			len += 2;		//	長期第二酸素センサー調整
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-100～99.2[%] バンク2調整値		100/128*A-100
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	-100～99.2[%] バンク4調整値		100/128*A-100
			break;
		case 0x59:	//	Fuel rail absolute pressure
			len += 2;		//	燃料レール絶対圧
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～655350[kPa] 気圧		10(256*A+B)
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x5A:	//	Relative accelerator pedal position
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] アクセル相対位置	100/255*A
			break;
		case 0x5B:	//	Hybrid battery pack remaining life
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～100[%] ハイブリッドバッテリー残量	100/255*A
			break;
		case 0x5C:	//	Engine oil temperature
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//	-40～210[℃] エンジンオイル温度		A-40
			break;
		case 0x5D:	//	Fuel injection timing
			len += 2;		//	燃料噴射タイミング
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-210.00～301.992[°] タイミング	(256*A+B)/128-210
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x5E:	//	Engine fuel rate
			len += 2;		//	エンジン燃料率
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～3276.75[L/h] 毎時燃料消費	(256*A+B)/20
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x5F:	//	Emission requirements to which vehicle is designed
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	車両放出条件設計値？
			break;
		//------------------------------------------------------
		case 0x60:	//	PIDs supported [61 - 80]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x00;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		case 0x61:	//	Driver's demand engine - percent torque
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-125～125[%] ドライバー需要トルク	A-125
			break;
		case 0x62:	//	Actual engine - percent torque
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-125～125[%] 実際のエンジントルク	A-125
			break;
		case 0x63:	//	Engine reference torque
			len += 2;		//	参照エンジントルク
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	0～655.35[Nm] トルク	256*A+B
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	↑下位8bit
			break;
		case 0x64:	//	Engine percent torque data
			len += 5;
			obd2_ret.SAE_ECU.VAL[0] = 0;	//(A)	-125～125[%] アイドルトルク	A-125
			obd2_ret.SAE_ECU.VAL[1] = 0;	//(B)	-125～125[%] エンジン点1	B-125
			obd2_ret.SAE_ECU.VAL[2] = 0;	//(C)	-125～125[%] エンジン点2	C-125
			obd2_ret.SAE_ECU.VAL[3] = 0;	//(D)	-125～125[%] エンジン点3	D-125
			obd2_ret.SAE_ECU.VAL[4] = 0;	//(E)	-125～125[%] エンジン点4	E-125
			break;
		case 0x65:	//	Auxiliary input / output supported
			break;
		case 0x66:	//	Mass air flow sensor
			break;
		case 0x67:	//	Engine coolant temperature
			break;
		case 0x68:	//	Intake air temperature sensor
			break;
		case 0x69:	//	Commanded EGR and EGR Error
			break;
		case 0x6A:	//	Commanded Diesel intake air flow control and relative intake air flow position
			break;
		case 0x6B:	//	Exhaust gas recirculation temperature
			break;
		case 0x6C:	//	Commanded throttle actuator control and relative throttle position
			break;
		case 0x6D:	//	Fuel pressure control system
			break;
		case 0x6E:	//	Injection pressure control system
			break;
		case 0x6F:	//	Turbocharger compressor inlet pressure
			break;
		case 0x70:	//	Boost pressure control
			break;
		case 0x71:	//	Variable Geometry turbo (VGT) control
			break;
		case 0x72:	//	Wastegate control
			break;
		case 0x73:	//	Exhaust pressure
			break;
		case 0x74:	//	Turbocharger RPM
			break;
		case 0x75:	//	Turbocharger temperature
			break;
		case 0x76:	//	Turbocharger temperature
			break;
		case 0x77:	//	Charge air cooler temperature (CACT)
			break;
		case 0x78:	//	Exhaust Gas temperature (EGT) Bank 1
			len += 4;		//	バンク1 排気ガス温度
			switch(obd2_ret_counter)
			{
			case 0:
				len = 1;
				obd2_ret.SAE_ECU.VAL[0] = 0x01;		//	1:EGTセンサー1 / 2:EGTセンサー2 / 4:EGTセンサー3 / 8:EGTセンサー4
				break;
			case 1:	//	EGTセンサー1, EGTセンサー2
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 2:	//	EGTセンサー3, EGTセンサー4
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			}
			obd2_ret_counter++;
			if(obd2_ret_counter >= 3) obd2_ret_counter = 0;
			break;
		case 0x79:	//	Exhaust Gas temperature (EGT) Bank 2
			len += 4;		//	バンク2 排気ガス温度
			switch(obd2_ret_counter)
			{
			case 0:
				len = 1;
				obd2_ret.SAE_ECU.VAL[0] = 0x01;		//	1:EGTセンサー1 / 2:EGTセンサー2 / 4:EGTセンサー3 / 8:EGTセンサー4
				break;
			case 1:	//	EGTセンサー1, EGTセンサー2
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 2:	//	EGTセンサー3, EGTセンサー4
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			}
			obd2_ret_counter++;
			if(obd2_ret_counter >= 3) obd2_ret_counter = 0;
			break;
		case 0x7A:	//	Diesel particulate filter (DPF)
			break;
		case 0x7B:	//	Diesel particulate filter (DPF)
			break;
		case 0x7C:	//	Diesel Particulate filter (DPF) temperature
			break;
		case 0x7D:	//	NOx NTE control area status
			break;
		case 0x7E:	//	PM NTE control area status
			break;
		case 0x7F:	//	Engine run time
			break;
		//------------------------------------------------------
		case 0x80:	//	PIDs supported [81 - A0]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x00;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		case 0x81:	//	Engine run time for Auxiliary Emissions Control Device(AECD)
			break;
		case 0x82:	//	Engine run time for Auxiliary Emissions Control Device(AECD)
			break;
		case 0x83:	//	NOx sensor
			break;
		case 0x84:	//	Manifold surface temperature
			break;
		case 0x85:	//	NOx reagent system
			break;
		case 0x86:	//	Particulate matter (PM) sensor
			break;
		case 0x87:	//	Intake manifold absolute pressure
			break;
		//------------------------------------------------------
		case 0xA0:	//	PIDs supported [A1 - C0]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x00;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		//------------------------------------------------------
		case 0xC0:	//	PIDs supported [C1 - E0]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x00;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		//------------------------------------------------------
		case 0xE0:	//	PIDs supported [E1 - FF]
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x00;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		}
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE2	処理(DTCレコード返信要求)
//----------------------------------------------------------------------------------------
int obd2_mode2(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 2;
		obd2_ret.SAE_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
		obd2_ret.SAE_ECU.PID = obd2_req.SAE_OBD.PID;
		switch(obd2_req.SAE_OBD.PID)
		{
		//------------------------------------------------------
		case 0x02:	//	DTC that caused freeze frame to be stored. 
		//------------------------------------------------------
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0x00;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		}
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE3	処理(保存済みDTCレコードを返信する)
//----------------------------------------------------------------------------------------
int obd2_mode3(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 7;
		obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
		obd2_ret.MD3_ECU.VAL[0] = 0x00;
		obd2_ret.MD3_ECU.VAL[1] = 0x00;
		obd2_ret.MD3_ECU.VAL[2] = 0x00;
		obd2_ret.MD3_ECU.VAL[3] = 0x00;
		obd2_ret.MD3_ECU.VAL[4] = 0x00;
		obd2_ret.MD3_ECU.VAL[5] = 0x00;
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE4	処理(DTCレコードを消去する)
//----------------------------------------------------------------------------------------
int obd2_mode4(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 1;
		obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE5	処理
//----------------------------------------------------------------------------------------
int obd2_mode5(int len)
{
	if(len == 3)
	{	//	スタンダード要求
		len = 6;
		obd2_ret.VS_ECU.MODE = obd2_req.VS_OBD.MODE + 0x40;
		obd2_ret.VS_ECU.PIDH = obd2_req.VS_OBD.PIDH;
		obd2_ret.VS_ECU.PIDL = obd2_req.VS_OBD.PIDL;
		switch(obd2_req.VS_OBD.PIDH)
		{
		case 0x01:	//	
			switch(obd2_req.VS_OBD.PIDL)
			{
			//------------------------------------------------------
			case 0x00:	//	PIDs supported [01 - 20]
			//------------------------------------------------------
				obd2_ret.VS_ECU.VAL[0] = 0x00;
				obd2_ret.VS_ECU.VAL[1] = 0x00;
				obd2_ret.VS_ECU.VAL[2] = 0x00;
				obd2_ret.VS_ECU.VAL[3] = 0x00;
				break;
			case 0x01:	//	O2 Sensor Monitor Bank 1 Sensor 1
				break;
			case 0x02:	//	O2 Sensor Monitor Bank 1 Sensor 2
				break;
			case 0x03:	//	O2 Sensor Monitor Bank 1 Sensor 3
				break;
			case 0x04:	//	O2 Sensor Monitor Bank 1 Sensor 4
				break;
			case 0x05:	//	O2 Sensor Monitor Bank 2 Sensor 1
				break;
			case 0x06:	//	O2 Sensor Monitor Bank 2 Sensor 2
				break;
			case 0x07:	//	O2 Sensor Monitor Bank 2 Sensor 3
				break;
			case 0x08:	//	O2 Sensor Monitor Bank 2 Sensor 4
				break;
			case 0x09:	//	O2 Sensor Monitor Bank 3 Sensor 1
				break;
			case 0x0A:	//	O2 Sensor Monitor Bank 3 Sensor 2
				break;
			case 0x0B:	//	O2 Sensor Monitor Bank 3 Sensor 3
				break;
			case 0x0C:	//	O2 Sensor Monitor Bank 3 Sensor 4
				break;
			case 0x0D:	//	O2 Sensor Monitor Bank 4 Sensor 1
				break;
			case 0x0E:	//	O2 Sensor Monitor Bank 4 Sensor 2
				break;
			case 0x0F:	//	O2 Sensor Monitor Bank 4 Sensor 3
				break;
			case 0x10:	//	O2 Sensor Monitor Bank 4 Sensor 4
				break;
			}
			break;
		case 0x02:	//	
			switch(obd2_req.VS_OBD.PIDL)
			{
			case 0x01:	//	O2 Sensor Monitor Bank 1 Sensor 1
				break;
			case 0x02:	//	O2 Sensor Monitor Bank 1 Sensor 2
				break;
			case 0x03:	//	O2 Sensor Monitor Bank 1 Sensor 3
				break;
			case 0x04:	//	O2 Sensor Monitor Bank 1 Sensor 4
				break;
			case 0x05:	//	O2 Sensor Monitor Bank 2 Sensor 1
				break;
			case 0x06:	//	O2 Sensor Monitor Bank 2 Sensor 2
				break;
			case 0x07:	//	O2 Sensor Monitor Bank 2 Sensor 3
				break;
			case 0x08:	//	O2 Sensor Monitor Bank 2 Sensor 4
				break;
			case 0x09:	//	O2 Sensor Monitor Bank 3 Sensor 1
				break;
			case 0x0A:	//	O2 Sensor Monitor Bank 3 Sensor 2
				break;
			case 0x0B:	//	O2 Sensor Monitor Bank 3 Sensor 3
				break;
			case 0x0C:	//	O2 Sensor Monitor Bank 3 Sensor 4
				break;
			case 0x0D:	//	O2 Sensor Monitor Bank 4 Sensor 1
				break;
			case 0x0E:	//	O2 Sensor Monitor Bank 4 Sensor 2
				break;
			case 0x0F:	//	O2 Sensor Monitor Bank 4 Sensor 3
				break;
			case 0x10:	//	O2 Sensor Monitor Bank 4 Sensor 4
				break;
			}
			break;
		}
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE6	試験結果（CANのみ酸素センサー監視）
//----------------------------------------------------------------------------------------
int obd2_mode6(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 1;
		obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE7	未知の診断トラブルコード情報取得
//----------------------------------------------------------------------------------------
int obd2_mode7(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 1;
		obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE8	オンボードシステム支援
//----------------------------------------------------------------------------------------
int obd2_mode8(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 1;
		obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE9	処理
//----------------------------------------------------------------------------------------
int obd2_mode9(int len)
{
	if(len >= 2)
	{	//	スタンダード要求
		len = 2;
		obd2_ret.SAE_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
		obd2_ret.SAE_ECU.PID = obd2_req.SAE_OBD.PID;
		switch(obd2_req.SAE_OBD.PID)
		{
		//------------------------------------------------------
		case 0x00:	//	Mode 9 supported PIDs (01 to 20)
		//------------------------------------------------------
			len += 4;
			obd2_ret.SAE_ECU.VAL[0] = 0x00;
			obd2_ret.SAE_ECU.VAL[1] = 0xC0;
			obd2_ret.SAE_ECU.VAL[2] = 0x00;
			obd2_ret.SAE_ECU.VAL[3] = 0x00;
			break;
		case 0x01:	//	VIN Message Count in PID 02. Only for ISO 9141-2, ISO 14230-4 and SAE J1850.
			break;
		case 0x02:	//	Vehicle Identification Number (VIN)
			break;
		case 0x03:	//	Calibration ID message count for PID 04. Only for ISO 9141-2, ISO 14230-4 and SAE J1850.
			break;
		case 0x04:	//	Calibration ID
			break;
		case 0x05:	//	Calibration verification numbers (CVN) message count for PID 06. Only for ISO 9141-2, ISO 14230-4 and SAE J1850.
			break;
		case 0x06:	//	Calibration Verification Numbers (CVN)
			break;
		case 0x07:	//	In-use performance tracking message count for PID 08 and 0B. Only for ISO 9141-2, ISO 14230-4 and SAE J1850.
			break;
		case 0x08:	//	In-use performance tracking for spark ignition vehicles
			len += 4;
			switch(obd2_ret_counter)
			{
			case 0:	//	OBDCOND, IGNCNTR
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 1:	//	HCCATCOMP, HCCATCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 2:	//	NCATCOMP, NCATCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 3:	//	NADSCOMP, NADSCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 4:	//	PMCOMP, PMCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 5:	//	EGSCOMP, EGSCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 6:	//	EGRCOMP, EGRCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 7:	//	BPCOMP, BPCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 8:	//	FUELCOMP, FUELCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			}
			obd2_ret_counter++;
			if(obd2_ret_counter >= 9) obd2_ret_counter = 0;
			break;
		case 0x09:	//	ECU name message count for PID 0A
			len += 1;
			obd2_ret.SAE_ECU.VAL[0] = 5;	//(A)	ECU名称文字長
			break;
		case 0x0A:	//	ECU name
			len += 5;
			switch(SELECT_ECU_UNIT)
			{
			case 0:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'P';
				obd2_ret.SAE_ECU.VAL[1] = 'O';
				obd2_ret.SAE_ECU.VAL[2] = 'W';
				obd2_ret.SAE_ECU.VAL[3] = 'E';
				break;
			case 1:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'C';
				obd2_ret.SAE_ECU.VAL[1] = 'H';
				obd2_ret.SAE_ECU.VAL[2] = 'A';
				obd2_ret.SAE_ECU.VAL[3] = 'S';
				break;
			case 2:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'B';
				obd2_ret.SAE_ECU.VAL[1] = 'O';
				obd2_ret.SAE_ECU.VAL[2] = 'D';
				obd2_ret.SAE_ECU.VAL[3] = 'Y';
				break;
			case 3:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'E';
				obd2_ret.SAE_ECU.VAL[1] = 'C';
				obd2_ret.SAE_ECU.VAL[2] = 'U';
				obd2_ret.SAE_ECU.VAL[3] = '3';
				break;
			case 4:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'E';
				obd2_ret.SAE_ECU.VAL[1] = 'C';
				obd2_ret.SAE_ECU.VAL[2] = 'U';
				obd2_ret.SAE_ECU.VAL[3] = '4';
				break;
			case 5:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'E';
				obd2_ret.SAE_ECU.VAL[1] = 'C';
				obd2_ret.SAE_ECU.VAL[2] = 'U';
				obd2_ret.SAE_ECU.VAL[3] = '5';
				break;
			case 6:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'E';
				obd2_ret.SAE_ECU.VAL[1] = 'C';
				obd2_ret.SAE_ECU.VAL[2] = 'U';
				obd2_ret.SAE_ECU.VAL[3] = '6';
				break;
			case 7:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = 'C';
				obd2_ret.SAE_ECU.VAL[1] = 'G';
				obd2_ret.SAE_ECU.VAL[2] = 'W';
				obd2_ret.SAE_ECU.VAL[3] = '1';
				break;
			default:	//(A)	ECU名称文字
				obd2_ret.SAE_ECU.VAL[0] = '?';
				obd2_ret.SAE_ECU.VAL[1] = '?';
				obd2_ret.SAE_ECU.VAL[2] = '?';
				obd2_ret.SAE_ECU.VAL[3] = '?';
				break;
			}
			break;
		case 0x0B:	//	In-use performance tracking for compression ignition vehicles
			len += 4;
			switch(obd2_ret_counter)
			{
			case 0:	//	OBDCOND, IGNCNTR
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 1:	//	HCCATCOMP, HCCATCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 2:	//	NCATCOMP, NCATCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 3:	//	NADSCOMP, NADSCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 4:	//	PMCOMP, PMCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 5:	//	EGSCOMP, EGSCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 6:	//	EGRCOMP, EGRCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 7:	//	BPCOMP, BPCOND
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			case 8:	//	FUELCOMP, FUELCOND
			default:
				obd2_ret.SAE_ECU.VAL[0] = 0x00;
				obd2_ret.SAE_ECU.VAL[1] = 0x00;
				obd2_ret.SAE_ECU.VAL[2] = 0x00;
				obd2_ret.SAE_ECU.VAL[3] = 0x00;
				break;
			}
			obd2_ret_counter++;
			if(obd2_ret_counter >= 9) obd2_ret_counter = 0;
			break;
		}
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	MODE10	永続DTC情報
//----------------------------------------------------------------------------------------
int obd2_modeA(int len)
{
	if(len == 2)
	{	//	スタンダード要求
		len = 1;
		obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
	}
	else
	{
		return 0;
	}
	return len;
}

//----------------------------------------------------------------------------------------
//	OBD2処理
//----------------------------------------------------------------------------------------
int obd2_job(unsigned char *msg, int len, unsigned char *res)	//int ch, int id, void *frame)
{
	memset(&obd2_ret, 0x00, sizeof(obd2_ret));
	memset(&obd2_req, 0x00, sizeof(obd2_req));
	memcpy(&obd2_req.BYTE[0], msg, len);
	switch(obd2_req.SAE_OBD.MODE)
	{
	case SHOW_CURRENT_DATA:		//	現在値
		len = obd2_mode1(len);
		break;
	case SHOW_FREEZE_FDATA:		//	停止フレーム
		len = obd2_mode2(len);
		break;
	case SHOW_STORED_DTC:		//	保存DTC取得
		len = obd2_mode3(len);
		break;
	case CLEAR_STORED_DTC:		//	保存DTC消去
		len = obd2_mode4(len);
		break;
	case TEST_RESULT_NON_CAN:	//	排気監視(NON-CAN)
		len = obd2_mode5(len);
		break;
	case TEST_RESULT_ONLY_CAN:	//	その他のモニタ、排気監視(CAN)
		len = obd2_mode6(len);
		break;
	case SHOW_PENDING_DTC:		//	最終DTC情報
		len = obd2_mode7(len);
		break;
	case CTRL_OPERATION_SYS:	//	制御系操作
		len = obd2_mode8(len);
		break;
	case REQUEST_VEHICLE_INFO:	//	車両情報
		len = obd2_mode9(len);
		break;
	case PERMANENT_DTC:			//	永久DTC情報
		len = obd2_modeA(len);
		break;
	default:	//	未対応モード
		len = 3;
		obd2_ret.NOT_ECU.X7F = 0x7F;
		obd2_ret.NOT_ECU.MODE = obd2_req.SAE_OBD.MODE;
		obd2_ret.NOT_ECU.X31 = 0x31;
		break;
	}
	if(len > 0)
	{	//	応答返信
		memcpy(&res[0], &obd2_ret.BYTE[0], len);
		return len;
	//	sw += 8;
	//	memcpy(&can_buf.ID[sw], obd2_ret.BYTE, 8);
	//	if(ch >= 0) add_mbox_frame(ch, 8, CAN_DATA_FRAME, sw);	//	送信待ちバッファ積み上げ
	//	return sw;	//	転送無し
	}
	return 0;
}




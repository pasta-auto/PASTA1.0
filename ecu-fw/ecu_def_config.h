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
// $RCSfile: ecu_def_powertrain.h,v $
// $Revision: 1.00 $
// $Date: 2017/01/15 11:05:00 $
// 
// Copyright (c) 2017 LandF Corporation.
//
// History:
//

#ifndef		__ECU_DEFAULT_SETTING__
#define		__ECU_DEFAULT_SETTING__

//---------------------------------------------------------------------------------------
//	ドライビングシミュレータ・パワートレイン間競合ID設定
//
//	競合に設定したIDをパワトレECUが受信すると、それ以降はIDの送受信方向を変更します。
//	LCDに対しては、該当IDのフレームデータの後ろにマークを付加して渡します。
//---------------------------------------------------------------------------------------

#define		DS_X_POWERTRAIN_ID		0x043		/*	エンジン回転数と速度	*/

//---------------------------------------------------------------------------------------
//	CGWのCAN-IDモード
//---------------------------------------------------------------------------------------

#define		CGW_ALL_ID_PASS			0			/*	0=全て転送, 1=指定方向のみ転送	*/

//---------------------------------------------------------------------------------------
//	ルーティングマップ初期値
//---------------------------------------------------------------------------------------

#ifdef		__LFY_RX63N__
const	unsigned char	CH_MAP_PAT[] = { 0x00, 0x22, 0x22, 0x22, 0x22 };	//	LFY-RX63NはCAN1のみ有効
#define		CH_MAP_CGW		0x22
#define		CH_MAP_SYS		0x22
#define		CH_MAP_RECV		0x20
#define		CH_MAP_SEND		0x02
#else
const	unsigned char	CH_MAP_PAT[] = { 0x00, 0x11, 0x33, 0x77, 0xFF };	//	CAN2ECUは指定チャンネル
#define		CH_MAP_CGW		0x7F
#define		CH_MAP_SYS		0xFF
#define		CH_MAP_RECV		0x10
#define		CH_MAP_SEND		0x01
#endif

//---------------------------------------------------------------------------------------
//	フレーム初期値
//---------------------------------------------------------------------------------------
typedef	struct	__def_frame_data__
{
	int				ID;		//	ID番号
	int				PAT;	//	パターン番号
	unsigned char	DAT[8];	//	データ
}	DEF_FRAME_DATA;
const DEF_FRAME_DATA	DEF_FRAME_BUFFER[] = {
	{0x024, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00}},	//	
	{0x146, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x46, 0x00, 0x00}},	//	
//	{0x150, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x50, 0x00, 0x00}},	//	
	{0x15A, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x5A, 0x00, 0x00}},	//	
//	{0x164, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x64, 0x00, 0x00}},	//	
//	{0x39E, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0x9E, 0x00, 0x00}},	//	
	{0x039, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x00, 0x00}},	//	
	{0x16F, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x6F, 0x00, 0x00}},	//	
	{0x043, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00}},	//	
//	{0x179, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x79, 0x00, 0x00}},	//	
	{0x183, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x83, 0x00, 0x00}},	//	
//	{0x3A9, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xA9, 0x00, 0x00}},	//	
//	{0x3B3, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xB3, 0x00, 0x00}},	//	
	{0x3BD, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xBD, 0x00, 0x00}},	//	
//	{0x3C7, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xC7, 0x00, 0x00}},	//	
	{0x18D, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x8D, 0x00, 0x00}},	//	
	{0x062, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x00, 0x00}},	//	
	{0x198, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x98, 0x00, 0x00}},	//	
//	{0x1A2, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xA2, 0x00, 0x00}},	//	
	{0x077, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00}},	//	
//	{0x1AD, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xAD, 0x00, 0x00}},	//	
	{0x19A, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x9A, 0x00, 0x00}},	//	
	{0x3D4, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xD4, 0x00, 0x00}},	//	
	{0x3DE, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xDE, 0x00, 0x00}},	//	
	{0x1D3, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xD3, 0x00, 0x00}},	//	
//	{0x42B, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x2B, 0x00, 0x00}},	//	
	{0x482, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x82, 0x00, 0x00}},	//	

	{0x01A, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x00}},	//	
	{0x02F, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00}},	//	
	{0x058, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x00, 0x00}},	//	
	{0x06D, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x00}},	//	
	{0x083, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0x00, 0x00}},	//	
	{0x098, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x00, 0x00}},	//	
	{0x1A7, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xA7, 0x00, 0x00}},	//	
	{0x1B1, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xB1, 0x00, 0x00}},	//	
	{0x1B8, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xB8, 0x00, 0x00}},	//	
	{0x1C9, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xC9, 0x00, 0x00}},	//	
	{0x25C, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x5C, 0x00, 0x00}},	//	
	{0x271, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x71, 0x00, 0x00}},	//	
	{0x286, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x86, 0x00, 0x00}},	//	
	{0x29C, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x9C, 0x00, 0x00}},	//	
	{0x2B1, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0xB1, 0x00, 0x00}},	//	

	{0x08D, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x00, 0x00}},	//	
//	{0x3E9, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xE9, 0x00, 0x00}},	//	
	{0x0A2, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0x00, 0x00}},	//	
//	{0x3F4, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xF4, 0x00, 0x00}},	//	
	{0x1BB, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xBB, 0x00, 0x00}},	//	
//	{0x3FF, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00}},	//	
	{0x266, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x66, 0x00, 0x00}},	//	
//	{0x40A, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x0A, 0x00, 0x00}},	//	
	{0x27B, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x7B, 0x00, 0x00}},	//	
//	{0x415, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x15, 0x00, 0x00}},	//	
	{0x290, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x90, 0x00, 0x00}},	//	
	{0x420, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x00, 0x00}},	//	
	{0x2A6, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0xA6, 0x00, 0x00}},	//	
//	{0x436, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x36, 0x00, 0x00}},	//	
	{0x2BB, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0xBB, 0x00, 0x00}},	//	
//	{0x441, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x41, 0x00, 0x00}},	//	
	{0x0B4, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0xB4, 0x00, 0x00}},	//	
//	{0x44C, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x4C, 0x00, 0x00}},	//	
	{0x457, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x57, 0x00, 0x00}},	//	
	{0x461, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x61, 0x00, 0x00}},	//	
	{0x46C, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x6C, 0x00, 0x00}},	//	
	{0x477, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x77, 0x00, 0x00}},	//	
	{   -1, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},	//	
};

//---------------------------------------------------------------------------------------
//  
//  機能   : マップ初期化
//  
//  引数   : 無し
//  
//  説明   : ルーティングマップの初期値設定
//  
//  戻り値 : 無し
//  
//  備考   : 初期値は全てのメッセージを受付・転送する。
//  
//---------------------------------------------------------------------------------------
#define		cgw_rootmap(a,b)	rout_map.ID[(a)].BYTE=(b)
void defset_rootmap(void)
{
	int	i;
	memset(&rout_map, 0, sizeof(rout_map));
	memset(can_to_exio, -1, sizeof(can_to_exio));
	if(SELECT_ECU_UNIT == ECU_UNIT_CGW)
	{	//	ECU_UNIT_POWERTRAIN
#if	(CGW_ALL_ID_PASS==0)
		memset(&rout_map, CH_MAP_SYS, sizeof(rout_map));
#else
		cgw_rootmap(0x024, PT_TO_CS);		//P	ブレーキ出力量
		cgw_rootmap(0x039, PT_TO_CS);		//P	スロットル位置
		cgw_rootmap(0x043, PT_TO_CS);		//P	エンジン回転数と速度
		cgw_rootmap(0x062, PT_TO_CS);		//P	パワステ出力量
		cgw_rootmap(0x077, PT_TO_CS);		//P	シフトポジション位置
		cgw_rootmap(0x146, PT_TO_CS);		//P	ブレーキオイル量
		cgw_rootmap(0x150, PT_TO_CS);		//P	ブレーキ摩耗警告・アイスバーン警報
		cgw_rootmap(0x15A, PT_TO_CS);		//P	アンチロックブレーキ作動
		cgw_rootmap(0x164, PT_TO_CS);		//P	ブレーキパッド温度・タイヤ温度
		cgw_rootmap(0x16F, PT_TO_CS);		//P	スロットル調整
		cgw_rootmap(0x179, PT_TO_CS);		//P	燃料消費率・混合気比率
		cgw_rootmap(0x183, PT_TO_CS);		//P	エンジン冷却水温度
		cgw_rootmap(0x18D, PT_TO_CS);		//P	エンジン故障
		cgw_rootmap(0x198, PT_TO_CS);		//P	パワステ故障
		cgw_rootmap(0x19A, PT_TO_CS);		//P	エンジンスタータ駆動
		cgw_rootmap(0x1A2, PT_TO_CS);		//P	横滑り防止発動
		cgw_rootmap(0x1AD, PT_TO_CS);		//P	ミッション故障
		cgw_rootmap(0x1D3, PT_TO_CS);		//P	サイドブレーキ動作状態
		cgw_rootmap(0x39E, PT_TO_CS);		//P	回生ブレーキ発電量
		cgw_rootmap(0x3A9, PT_TO_CS);		//P	外気温度・排気温度
		cgw_rootmap(0x3B3, PT_TO_CS);		//P	有害排気ガス濃度・粒子状物質濃度
		cgw_rootmap(0x3BD, PT_TO_CS);		//P	エンジンオイル量
		cgw_rootmap(0x3C7, PT_TO_CS);		//P	点火不良・点火タイミング異常
		cgw_rootmap(0x3D4, PT_TO_CS);		//P	エンジンスタータ故障
		cgw_rootmap(0x3DE, PT_TO_CS);		//P	バッテリー警報
		cgw_rootmap(0x42B, PT_TO_CS);		//P	サイドブレーキ警報
		cgw_rootmap(0x482, PT_TO_CS);		//P	エコドライブ判定
		//	ECU_UNIT_CHASSIS
		cgw_rootmap(0x01A, CS_TO_AL);		//C	ブレーキ操作量
		cgw_rootmap(0x02F, CS_TO_AL);		//C	アクセル操作量
		cgw_rootmap(0x058, CS_TO_AL);		//C	ハンドル操作位置
		cgw_rootmap(0x06D, CS_TO_AL);		//C	シフトポジションスイッチ
		cgw_rootmap(0x083, CS_TO_AL);		//C	ウィンカー左右・ハザードスイッチ
		cgw_rootmap(0x098, CS_TO_AL);		//C	クラクションスイッチ
		cgw_rootmap(0x1A7, CS_TO_AL);		//C	ポジション・ヘッドライト・ハイビームスイッチ
		cgw_rootmap(0x1B1, CS_TO_AL);		//C	パッシングスイッチ
		cgw_rootmap(0x1B8, CS_TO_AL);		//C	エンジンスタートボタン
		cgw_rootmap(0x1C9, CS_TO_AL);		//C	サイドブレーキ
		cgw_rootmap(0x25C, CS_TO_AL);		//C	フロントワイパー・間欠・LOW・HIGH・ウォッシャースイッチ
		cgw_rootmap(0x271, CS_TO_AL);		//C	リアワイパー・ウォッシャースイッチ
		cgw_rootmap(0x286, CS_TO_AL);		//C	ドアロックスイッチ・アンロックスイッチ
		cgw_rootmap(0x29C, CS_TO_AL);		//C	右ドア・ウィンドウ昇降スイッチ
		cgw_rootmap(0x2B1, CS_TO_AL);		//C	左ドア・ウィンドウ昇降スイッチ
		//	ECU_UNIT_BODY
		cgw_rootmap(0x08D, BD_TO_CS);		//B	ウィンカー左右点灯状態
		cgw_rootmap(0x0A2, BD_TO_CS);		//B	クラクション鳴動
		cgw_rootmap(0x0B4, BD_TO_CS);		//B	エアバッグ作動スイッチ
		cgw_rootmap(0x1BB, BD_TO_CS);		//B	ポジション・ヘッドライト・ハイビーム点灯状態
		cgw_rootmap(0x266, BD_TO_CS);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャー動作状態
		cgw_rootmap(0x27B, BD_TO_CS);		//B	リアワイパー・ウォッシャー動作状態
		cgw_rootmap(0x290, BD_TO_CS);		//B	ドア開閉・施錠状態
		cgw_rootmap(0x2A6, BD_TO_CS);		//B	右ドア・ウィンドウ位置・リミットスイッチ状態
		cgw_rootmap(0x2BB, BD_TO_CS);		//B	左ドア・ウィンドウ位置・リミットスイッチ状態
		cgw_rootmap(0x3E9, BD_TO_CS);		//B	ウィンカー球切れ警報
		cgw_rootmap(0x3F4, BD_TO_CS);		//B	クラクション故障
		cgw_rootmap(0x3FF, BD_TO_CS);		//B	ポジション・ヘッドライト・ハイビーム球切れ・バルブ制御故障
		cgw_rootmap(0x40A, BD_TO_CS);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャーモータ・ポンプ故障
		cgw_rootmap(0x415, BD_TO_CS);		//B	リアワイパー・ウォッシャーモータ・ポンプ故障
		cgw_rootmap(0x420, BD_TO_CS);		//B	ドアロック駆動装置故障
		cgw_rootmap(0x436, BD_TO_CS);		//B	右ドア・ウィンドウモータ故障
		cgw_rootmap(0x441, BD_TO_CS);		//B	左ドア・ウィンドウモータ故障
		cgw_rootmap(0x44C, BD_TO_CS);		//B	エアバッグ故障
		cgw_rootmap(0x457, BD_TO_CS);		//B	シートベルトセンサー
		cgw_rootmap(0x461, BD_TO_CS);		//B	シートベルト警報
		cgw_rootmap(0x46C, BD_TO_CS);		//B	ボンネット開閉スイッチ
		cgw_rootmap(0x477, BD_TO_CS);		//B	トランク開閉スイッチ
	
		//	トランスポートID
		cgw_rootmap(0x7DF, AL_TO_AL);		//ダイアグポート受信、全て送信
		cgw_rootmap(0x7E3, EX_TO_AL);		//ダイアグポート受信、全て送信
		cgw_rootmap(0x7EB, AL_TO_EX);		//ダイアグポート送信、全て受信
		cgw_rootmap(0x7E0 + ECU_UNIT_POWERTRAIN, EX_TO_PT);
		cgw_rootmap(0x7E0 + ECU_UNIT_CHASSIS,	 EX_TO_CS);
		cgw_rootmap(0x7E0 + ECU_UNIT_BODY,		 EX_TO_BD);
		cgw_rootmap(0x7E4 + ECU_UNIT_POWERTRAIN, EX_TO_PT);
		cgw_rootmap(0x7E4 + ECU_UNIT_CHASSIS,	 EX_TO_CS);
		cgw_rootmap(0x7E4 + ECU_UNIT_BODY,		 EX_TO_BD);
		cgw_rootmap(0x7E8 + ECU_UNIT_POWERTRAIN, PT_TO_EX);
		cgw_rootmap(0x7E8 + ECU_UNIT_CHASSIS,	 CS_TO_EX);
		cgw_rootmap(0x7E8 + ECU_UNIT_BODY,		 BD_TO_EX);
		cgw_rootmap(0x7EC + ECU_UNIT_POWERTRAIN, PT_TO_EX);
		cgw_rootmap(0x7EC + ECU_UNIT_CHASSIS,	 CS_TO_EX);
		cgw_rootmap(0x7EC + ECU_UNIT_BODY,		 BD_TO_EX);
#endif
	}
	else
	{	//	ECUは個別に送受信設定を行う
		rout_map.ID[0x7DF].BYTE = CH_MAP_RECV;						//	全てのECUが受信するブロードキャストID
		rout_map.ID[(0x7E0 + SELECT_ECU_UNIT)].BYTE = CH_MAP_RECV;	//	個別ECUのTP受信ID
		rout_map.ID[(0x7E8 + SELECT_ECU_UNIT)].BYTE = CH_MAP_SEND;	//	個別ECUのTP送信ID
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 周期・イベント初期値
//  
//  引数   : 無し
//  
//  説明   : 周期メッセージ・イベントメッセージの初期値設定
//  
//  戻り値 : 無し
//  
//  備考   : 最大255個の管理IDを定義可能
//  
//---------------------------------------------------------------------------------------
int	add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt);
void defset_confecu(void)
{
	//	ゼロ初期化
	memset(&conf_ecu, 0, sizeof(conf_ecu));			//	周期・イベント・リモート管理定義初期化
	conf_ecu.TOP = -1;
	/*
		登録関数
		int	add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt)

		引数
		int rtr			リモートフレーム指定		0:データ / 1:リモート
		int id			CANメッセージ番号			0x000～0x7FF(0～2047)
		int dlc			データバイト長				0～8
		int enb			処理許可フラグ				0:禁止 / 1:許可
		int rep			周期フレーム指定			0:イベント / 1:周期
		int time		周期時間又は遅延時間(ms)	0～65535
		int cnt			遅延増加時間(ms)			0～65535
	*/
													//P=パワトレ / B=ボディー / C=シャシー
	if(SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN)
	{
		add_cyceve_list(0, 0x024, 8, 1, 1, 20, 0);		//P	ブレーキ出力量
		add_cyceve_list(0, 0x039, 8, 1, 1, 20, 1);		//P	スロットル位置
		add_cyceve_list(0, 0x043, 8, 1, 1, 20, 2);		//P	エンジン回転数と速度
		add_cyceve_list(0, 0x062, 8, 1, 1, 20, 3);		//P	パワステ出力量
		add_cyceve_list(0, 0x077, 8, 1, 1, 20, 4);		//P	シフトポジション位置
		add_cyceve_list(0, 0x146, 8, 1, 1, 50, 5);		//P	ブレーキオイル量
	//X	add_cyceve_list(0, 0x150, 8, 1, 1, 50, 6);		//P	ブレーキ摩耗警告・アイスバーン警報
		add_cyceve_list(0, 0x15A, 8, 1, 1, 50, 7);		//P	アンチロックブレーキ作動
	//X	add_cyceve_list(0, 0x164, 8, 1, 1, 50, 8);		//P	ブレーキパッド温度・タイヤ温度
		add_cyceve_list(0, 0x16F, 8, 1, 1, 50, 9);		//P	スロットル調整
	//X	add_cyceve_list(0, 0x179, 8, 1, 1, 50,10);		//P	燃料消費率・混合気比率
		add_cyceve_list(0, 0x183, 8, 1, 1, 50,11);		//P	エンジン冷却水温度
		add_cyceve_list(0, 0x18D, 8, 1, 1, 50,12);		//P	エンジン故障
		add_cyceve_list(0, 0x198, 8, 1, 1, 50,13);		//P	パワステ故障
		add_cyceve_list(0, 0x19A, 8, 1, 1, 50,14);		//P	エンジンスタータ駆動
	//X	add_cyceve_list(0, 0x1A2, 8, 1, 1, 50,15);		//P	横滑り防止発動
	//X	add_cyceve_list(0, 0x1AD, 8, 1, 1, 50,16);		//P	ミッション故障
		add_cyceve_list(0, 0x1D3, 8, 1, 1, 50,17);		//P	サイドブレーキ動作状態
	//X	add_cyceve_list(0, 0x39E, 8, 1, 1,500,18);		//P	回生ブレーキ発電量
	//X	add_cyceve_list(0, 0x3A9, 8, 1, 1,500,19);		//P	外気温度・排気温度
	//X	add_cyceve_list(0, 0x3B3, 8, 1, 1,500,20);		//P	有害排気ガス濃度・粒子状物質濃度
		add_cyceve_list(0, 0x3BD, 8, 1, 1,500,21);		//P	エンジンオイル量
	//X	add_cyceve_list(0, 0x3C7, 8, 1, 1,500,22);		//P	点火不良・点火タイミング異常
		add_cyceve_list(0, 0x3D4, 8, 1, 1,500,23);		//P	エンジンスタータ故障
		add_cyceve_list(0, 0x3DE, 8, 1, 1,500,24);		//P	バッテリー警報
	//X	add_cyceve_list(0, 0x42B, 8, 1, 1,500,25);		//P	サイドブレーキ警報
		add_cyceve_list(0, 0x482, 8, 1, 1,500,26);		//P	エコドライブ判定
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_CHASSIS)
	{
		add_cyceve_list(0, 0x01A, 8, 1, 1, 20, 0);		//C	ブレーキ操作量
		add_cyceve_list(0, 0x02F, 8, 1, 1, 20, 1);		//C	アクセル操作量
		add_cyceve_list(0, 0x058, 8, 1, 1, 20, 2);		//C	ハンドル操作位置
		add_cyceve_list(0, 0x06D, 8, 1, 1, 20, 3);		//C	シフトポジションスイッチ
		add_cyceve_list(0, 0x083, 8, 1, 1, 20, 4);		//C	ウィンカー左右・ハザードスイッチ
		add_cyceve_list(0, 0x098, 8, 1, 1, 20, 5);		//C	クラクションスイッチ
		add_cyceve_list(0, 0x1A7, 8, 1, 1, 50, 6);		//C	ポジション・ヘッドライト・ハイビームスイッチ
		add_cyceve_list(0, 0x1B1, 8, 1, 1, 50, 7);		//C	パッシングスイッチ
		add_cyceve_list(0, 0x1B8, 8, 1, 1, 50, 8);		//C	エンジンスタートボタン
		add_cyceve_list(0, 0x1C9, 8, 1, 1, 50, 9);		//C	サイドブレーキ
		add_cyceve_list(0, 0x25C, 8, 1, 1,100,10);		//C	フロントワイパー・間欠・LOW・HIGH・ウォッシャースイッチ
		add_cyceve_list(0, 0x271, 8, 1, 1,100,11);		//C	リアワイパー・ウォッシャースイッチ
		add_cyceve_list(0, 0x286, 8, 1, 1,100,12);		//C	ドアロックスイッチ・アンロックスイッチ
		add_cyceve_list(0, 0x29C, 8, 1, 1,100,13);		//C	右ドア・ウィンドウ昇降スイッチ
		add_cyceve_list(0, 0x2B1, 8, 1, 1,100,14);		//C	左ドア・ウィンドウ昇降スイッチ
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_BODY)
	{
		add_cyceve_list(0, 0x08D, 8, 1, 1, 20, 0);		//B	ウィンカー左右点灯状態
		add_cyceve_list(0, 0x0A2, 8, 1, 1, 20, 1);		//B	クラクション鳴動
		add_cyceve_list(0, 0x0B4, 8, 1, 1, 20, 2);		//B	エアバッグ作動スイッチ
		add_cyceve_list(0, 0x1BB, 8, 1, 1, 50, 3);		//B	ポジション・ヘッドライト・ハイビーム点灯状態
		add_cyceve_list(0, 0x266, 8, 1, 1,100, 4);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャー動作状態
		add_cyceve_list(0, 0x27B, 8, 1, 1,100, 5);		//B	リアワイパー・ウォッシャー動作状態
		add_cyceve_list(0, 0x290, 8, 1, 1,100, 6);		//B	ドア開閉・施錠状態
		add_cyceve_list(0, 0x2A6, 8, 1, 1,100, 7);		//B	右ドア・ウィンドウ位置・リミットスイッチ状態
		add_cyceve_list(0, 0x2BB, 8, 1, 1,100, 8);		//B	左ドア・ウィンドウ位置・リミットスイッチ状態
	//X	add_cyceve_list(0, 0x3E9, 8, 1, 1,500, 9);		//B	ウィンカー球切れ警報
	//X	add_cyceve_list(0, 0x3F4, 8, 1, 1,500,10);		//B	クラクション故障
	//X	add_cyceve_list(0, 0x3FF, 8, 1, 1,500,11);		//B	ポジション・ヘッドライト・ハイビーム球切れ・バルブ制御故障
	//X	add_cyceve_list(0, 0x40A, 8, 1, 1,500,12);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャーモータ・ポンプ故障
	//X	add_cyceve_list(0, 0x415, 8, 1, 1,500,13);		//B	リアワイパー・ウォッシャーモータ・ポンプ故障
		add_cyceve_list(0, 0x420, 8, 1, 1,500,14);		//B	ドアロック駆動装置故障
	//X	add_cyceve_list(0, 0x436, 8, 1, 1,500,15);		//B	右ドア・ウィンドウモータ故障
	//X	add_cyceve_list(0, 0x441, 8, 1, 1,500,16);		//B	左ドア・ウィンドウモータ故障
	//X	add_cyceve_list(0, 0x44C, 8, 1, 1,500,17);		//B	エアバッグ故障
		add_cyceve_list(0, 0x457, 8, 1, 1,500,18);		//B	シートベルトセンサー
		add_cyceve_list(0, 0x461, 8, 1, 1,500,19);		//B	シートベルト警報
		add_cyceve_list(0, 0x46C, 8, 1, 1,500,20);		//B	ボンネット開閉スイッチ
		add_cyceve_list(0, 0x477, 8, 1, 1,500,21);		//B	トランク開閉スイッチ
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_CGW)
	{
//		add_cyceve_list(0, 0x7FD, 8, 1, 1, 1000, 0);	//C	毎秒通知
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 外部入出力定義初期値
//  
//  引数   : 無し
//  
//  説明   : 外部入出力の初期値設定
//  
//  戻り値 : 無し
//  
//  備考   : デフォルトでは、入出力はLF74が行いECUが直接参照するポートは無い
//  
//---------------------------------------------------------------------------------------
#define		DEF_IOPAT_XX	0	/*	パターン未定義	*/
//	チェックリスト変化パターン定義
const unsigned char	DEF_IOPAT_00[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_01[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_02[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_03[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_04[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_05[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_06[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};
const unsigned char	DEF_IOPAT_07[24] = {
//	0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	};

int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat);
void defset_extlist(void)
{
	memset(ext_list, 0, sizeof(ext_list));	//	ECU入出力チェックリスト初期化
	ext_list_count = 0;						//	チェックリスト数リセット
	/*
		登録関数
		int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat)

		引数
		int				id		フレームID番号		0～2047(000～7FF)
		int				mode	I/O処理モード設定	0:無効 / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit値 / 6:3bit値 / 7:4bit値
		int				neg		データ反転指定		0:通常 / 1:反転
		int				size	アクセスサイズ		0:BIT / 1～7:nBYTE
		int				bpos	バイト位置			0～7
		int				dlc		データバイト長		0～8
		int				nom		適用ポート番号		0～33
		int				msk		マスクパターン		00～FF
		unsigned char	*pat	パターンデータ		24bytes	(0=使用しない)
	*/
#if	0
	if(SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN)
	{
		//パワトレ
		add_extern_io(0x055, 3, 0, 2, 0, 8, 23, 0xFF, DEF_IOPAT_XX);		//P	アクセル／速度関連			word		A/Dアクセルペダル値
		add_extern_io(0x075, 3, 0, 2, 0, 8, 24, 0xFF, DEF_IOPAT_XX);		//P	ハンドル操作関連(陀角)		word		A/Dステアリング値
		add_extern_io(0x07F, 3, 0, 2, 0, 8, 25, 0xFF, DEF_IOPAT_XX);		//P	ブレーキペダル状態			word		A/Dブレーキペダル値
		add_extern_io(0x0B4, 1, 0, 2, 0, 8,  0, 0xFF, DEF_IOPAT_XX);		//P	ハンドル操作関連(パワステ)	0.bit0		ステータス
		add_extern_io(0x0F7, 1, 0, 1, 0, 8,  1, 0xFF, DEF_IOPAT_XX);		//P	エンジン始動時				0.bit6,1,0	スタータスイッチ
		add_extern_io(0x121, 6, 0, 2, 0, 8,  2, 0xFF, DEF_IOPAT_XX);		//P	エンジン始動時				word.pat[4]	ステータス
		add_extern_io(0x139, 6, 0, 2, 0, 8,  6, 0xFF, DEF_IOPAT_XX);		//P	エンジン始動時				word.pat[4]	ステータス
		add_extern_io(0x155, 1, 0, 1, 0, 8, 10, 0xFF, DEF_IOPAT_XX);		//P	シフトレバー位置状態
		add_extern_io(0x167, 1, 0, 1, 0, 8, 11, 0xFF, DEF_IOPAT_XX);		//P	アクセル／エンジン回転関連
		add_extern_io(0x196, 1, 0, 1, 0, 8, 12, 0xFF, DEF_IOPAT_XX);		//P	キー関連
		add_extern_io(0x1A0, 1, 0, 1, 0, 8, 13, 0xFF, DEF_IOPAT_XX);		//P	エンジン関連
		add_extern_io(0x1AE, 1, 0, 1, 0, 8, 14, 0xFF, DEF_IOPAT_XX);		//P	キー関連
		add_extern_io(0x207, 1, 0, 1, 0, 8, 15, 0xFF, DEF_IOPAT_XX);		//P	シートベルト警告灯状態	サイドブレーキ状態		ドア開閉状態(左右共有)
		add_extern_io(0x2DF, 1, 0, 1, 0, 8, 16, 0xFF, DEF_IOPAT_XX);		//P	アクセル／ブレーキ関連
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_CHASSIS)
	{
		//シャシー
		add_extern_io(0x0EF, 1, 0, 1, 0, 8,  0, 0xFF, DEF_IOPAT_XX);		//C	空調インパネボタン操作		0.bit3		ステータス
		add_extern_io(0x108, 5, 0, 1, 0, 8,  1, 0xFF, DEF_IOPAT_XX);		//C	空調風量調整レベル			byte.pat[8]	インパネ操作0～6
		add_extern_io(0x113, 1, 0, 1, 0, 8,  4, 0xFF, DEF_IOPAT_XX);		//C	リアDEF ON/OFF				0.bit7		インパネ操作
		add_extern_io(0x113, 1, 1, 1, 0, 8,  5, 0xFF, DEF_IOPAT_XX);		//C	A/C ON/OFF					1.bit3		インパネ操作
		add_extern_io(0x13F, 1, 0, 1, 0, 8,  6, 0xFF, DEF_IOPAT_XX);		//C	ブレーキランプ,ペダルON/OFF状態
		add_extern_io(0x1E1, 1, 0, 1, 0, 8,  7, 0xFF, DEF_IOPAT_XX);		//C	距離メーター関連
		add_extern_io(0x21F, 1, 0, 1, 0, 8,  8, 0xFF, DEF_IOPAT_XX);		//C	ドアロック関連
		add_extern_io(0x261, 1, 0, 1, 0, 8,  9, 0xFF, DEF_IOPAT_XX);		//C	ドアロック関連
		add_extern_io(0x31C, 1, 0, 1, 0, 8, 10, 0xFF, DEF_IOPAT_XX);		//C	シフトレバー(P状態)
		add_extern_io(0x3FB, 1, 0, 1, 0, 8, 11, 0xFF, DEF_IOPAT_XX);		//C	ファン風量
		add_extern_io(0x433, 1, 0, 1, 0, 8, 12, 0xFF, DEF_IOPAT_XX);		//C	エンジン回転数（受）
		add_extern_io(0x434, 1, 0, 1, 0, 8, 13, 0xFF, DEF_IOPAT_XX);		//C	車速 (受)
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_BODY)
	{
		//ボディー
		add_extern_io(0x0C7, 1, 0, 3, 0, 8,  0, 0xFF, DEF_IOPAT_XX);		//B	フロントDEF ON/OFF			0.bit7		インパネ操作
		add_extern_io(0x0C7, 1, 0, 3, 0, 8,  1, 0xFF, DEF_IOPAT_XX);		//B	空調AUTO/OFFスイッチ		1.bit7		インパネ操作
		add_extern_io(0x185, 1, 0, 1, 0, 8,  2, 0xFF, DEF_IOPAT_XX);		//B	ドアロック関連
		add_extern_io(0x224, 1, 0, 1, 0, 8,  3, 0xFF, DEF_IOPAT_XX);		//B	ライト点灯状態
		add_extern_io(0x27B, 1, 0, 1, 0, 8,  4, 0xFF, DEF_IOPAT_XX);		//B	ドアロック関連
		add_extern_io(0x2B1, 1, 0, 1, 0, 8,  5, 0xFF, DEF_IOPAT_XX);		//B	ウィンドウ動作状態(左右共有)	ドアロック関連
		add_extern_io(0x322, 1, 0, 1, 0, 8,  6, 0xFF, DEF_IOPAT_XX);		//B	ドアロック	ハザード	テールランプ,補助ランプ	ヘッドライト(Lo)	ヘッドライト(Hi)	OFF
		add_extern_io(0x441, 1, 0, 1, 0, 8,  7, 0xFF, DEF_IOPAT_XX);		//B	ウィンカー動作(模擬)
		add_extern_io(0x449, 1, 0, 1, 0, 8,  8, 0xFF, DEF_IOPAT_XX);		//B	ターンシグナル点灯(模擬)
		add_extern_io(0x452, 1, 0, 1, 0, 8,  9, 0xFF, DEF_IOPAT_XX);		//B	ホーン
		add_extern_io(0x501, 1, 0, 1, 0, 8, 10, 0xFF, DEF_IOPAT_XX);		//B	フロントワイパー	間欠・LOW・H/MST・ウォッシャー
		add_extern_io(0x503, 1, 0, 1, 0, 8, 14, 0xFF, DEF_IOPAT_XX);		//B	リアワイパー	リアウォッシャー
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_CGW)
	{
		add_extern_io(0x7FF, 1, 1, 1, 0, 8,  0, 0xFF, DEF_IOPAT_XX);		//G	
	}
#endif
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 通信経由外部入出力定義初期値
//  
//  引数   : 無し
//  
//  説明   : 外部入出力の初期値設定
//  
//  戻り値 : 無し
//  
//  備考   : デフォルトでは、入出力はLF74が行いECUが直接参照するポートは無い
//  
//---------------------------------------------------------------------------------------
void defset_extlist_ex(void)
{
	memset(ext_list, 0, sizeof(ext_list));	//	ECU入出力チェックリスト初期化
	ext_list_count = 0;						//	チェックリスト数リセット
	/*
		登録関数
		int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat)

		引数
		int				id		フレームID番号		0～2047(000～7FF)
		int				mode	I/O処理モード設定	入力 0:bit / 1:byte / 2:word / 3:long  出力 4:bit / 5:byte / 6:word / 7:long
		int				neg		データ反転指定		[0]無効
		int				size	アクセスサイズ		[0]無効
		int				bpos	バイト位置			0～7（通常=0）
		int				dlc		データバイト長		[0]無効
		int				nom		適用ポート番号		0～63
		int				msk		マスクパターン		00～FF
		unsigned char	*pat	パターンデータ		[0]=使用しない
	*/
#if	1
	if(SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN)
	{	//	パワトレはシャシーからの情報のみ出力する
		add_extern_io(0x01A, 6, 0, 0, 0, 0, 0, 1, 0);		//C	ブレーキ操作量
		add_extern_io(0x02F, 6, 0, 0, 0, 0, 1, 1, 0);		//C	アクセル操作量
		add_extern_io(0x058, 6, 0, 0, 0, 0, 2, 1, 0);		//C	ハンドル操作位置
		add_extern_io(0x06D, 5, 0, 0, 0, 0, 3, 1, 0);		//C	シフトポジションスイッチ
//		add_extern_io(0x083, 5, 0, 0, 0, 0, 4, 1, 0);		//C	ウィンカー左右・ハザードスイッチ
//		add_extern_io(0x098, 5, 0, 0, 0, 0, 5, 1, 0);		//C	クラクションスイッチ
//		add_extern_io(0x1A7, 5, 0, 0, 0, 0, 6, 1, 0);		//C	ポジション・ヘッドライト・ハイビームスイッチ
//		add_extern_io(0x1B1, 5, 0, 0, 0, 0, 7, 1, 0);		//C	パッシングスイッチ
		add_extern_io(0x1B8, 5, 0, 0, 0, 0, 8, 1, 0);		//C	エンジンスタートボタン
		add_extern_io(0x1C9, 5, 0, 0, 0, 0, 9, 1, 0);		//C	サイドブレーキ
//		add_extern_io(0x25C, 6, 0, 0, 0, 0,10, 1, 0);		//C	フロントワイパー・間欠・LOW・HIGH・ウォッシャースイッチ・間欠タイマー
//		add_extern_io(0x271, 5, 0, 0, 0, 0,11, 1, 0);		//C	リアワイパー・ウォッシャースイッチ
//		add_extern_io(0x286, 5, 0, 0, 0, 0,12, 1, 0);		//C	ドアロックスイッチ・アンロックスイッチ
//		add_extern_io(0x29C, 5, 0, 0, 0, 0,13, 1, 0);		//C	右ドア・ウィンドウ昇降スイッチ
//		add_extern_io(0x2B1, 5, 0, 0, 0, 0,14, 1, 0);		//C	左ドア・ウィンドウ昇降スイッチ
		//	パワトレの入力情報
		add_extern_io(0x024, 2, 0, 0, 0, 0,15, 1, 0);		//P	ブレーキ出力量
		add_extern_io(0x039, 2, 0, 0, 0, 0,16, 1, 0);		//P	スロットル位置
		add_extern_io(0x043, 3, 0, 0, 0, 0,17, 1, 0);		//P	エンジン回転数/速度
		add_extern_io(0x062, 3, 0, 0, 0, 0,18, 1, 0);		//P	パワステ出力量/トルク
		add_extern_io(0x077, 1, 0, 0, 0, 0,19, 1, 0);		//P	シフトポジション位置
		add_extern_io(0x146, 1, 0, 0, 0, 0,20, 1, 0);		//P	ブレーキオイル量
	//X	add_extern_io(0x150, 1, 0, 0, 0, 0,21, 1, 0);		//P	ブレーキ摩耗警告・アイスバーン警報
		add_extern_io(0x15A, 1, 0, 0, 0, 0,22, 1, 0);		//P	アンチロックブレーキ作動
	//X	add_extern_io(0x164, 2, 0, 0, 0, 0,23, 1, 0);		//P	ブレーキパッド温度・タイヤ温度
		add_extern_io(0x16F, 2, 0, 0, 0, 0,24, 1, 0);		//P	スロットル調整
	//X	add_extern_io(0x179, 2, 0, 0, 0, 0,25, 1, 0);		//P	燃料消費率・混合気比率
		add_extern_io(0x183, 1, 0, 0, 0, 0,26, 1, 0);		//P	エンジン冷却水温度
		add_extern_io(0x18D, 1, 0, 0, 0, 0,27, 1, 0);		//P	エンジン故障
		add_extern_io(0x198, 1, 0, 0, 0, 0,28, 1, 0);		//P	パワステ故障
		add_extern_io(0x19A, 1, 0, 0, 0, 0,29, 1, 0);		//P	エンジンスタータ駆動
	//X	add_extern_io(0x1A2, 1, 0, 0, 0, 0,30, 1, 0);		//P	横滑り防止発動
	//X	add_extern_io(0x1AD, 1, 0, 0, 0, 0,31, 1, 0);		//P	ミッション故障
		add_extern_io(0x1D3, 1, 0, 0, 0, 0,32, 1, 0);		//P	サイドブレーキ動作状態
	//X	add_extern_io(0x39E, 1, 0, 0, 0, 0,33, 1, 0);		//P	回生ブレーキ発電量
	//X	add_extern_io(0x3A9, 2, 0, 0, 0, 0,34, 1, 0);		//P	外気温度・排気温度
	//X	add_extern_io(0x3B3, 2, 0, 0, 0, 0,35, 1, 0);		//P	有害排気ガス濃度・粒子状物質濃度
		add_extern_io(0x3BD, 1, 0, 0, 0, 0,36, 1, 0);		//P	エンジンオイル量
	//X	add_extern_io(0x3C7, 1, 0, 0, 0, 0,37, 1, 0);		//P	点火不良・点火タイミング異常
		add_extern_io(0x3D4, 1, 0, 0, 0, 0,38, 1, 0);		//P	燃料残量
		add_extern_io(0x3DE, 1, 0, 0, 0, 0,39, 1, 0);		//P	バッテリー警報
	//X	add_extern_io(0x42B, 1, 0, 0, 0, 0,40, 1, 0);		//P	サイドブレーキ警報
		add_extern_io(0x482, 1, 0, 0, 0, 0,41, 1, 0);		//P	エコドライブ判定
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_CHASSIS)
	{	//	シャシーの入力
		add_extern_io(0x01A, 2, 0, 0, 0, 0, 0, 1, 0);		//C	ブレーキ操作量
		add_extern_io(0x02F, 2, 0, 0, 0, 0, 1, 1, 0);		//C	アクセル操作量
		add_extern_io(0x058, 2, 0, 0, 0, 0, 2, 1, 0);		//C	ハンドル操作位置
		add_extern_io(0x06D, 1, 0, 0, 0, 0, 3, 1, 0);		//C	シフトポジションスイッチ
		add_extern_io(0x083, 1, 0, 0, 0, 0, 4, 1, 0);		//C	ウィンカー左右・ハザードスイッチ
		add_extern_io(0x098, 1, 0, 0, 0, 0, 5, 1, 0);		//C	クラクションスイッチ
		add_extern_io(0x1A7, 1, 0, 0, 0, 0, 6, 1, 0);		//C	ポジション・ヘッドライト・ハイビームスイッチ
		add_extern_io(0x1B1, 1, 0, 0, 0, 0, 7, 1, 0);		//C	パッシングスイッチ
		add_extern_io(0x1B8, 1, 0, 0, 0, 0, 8, 1, 0);		//C	エンジンスタートボタン
		add_extern_io(0x1C9, 1, 0, 0, 0, 0, 9, 1, 0);		//C	サイドブレーキ
		add_extern_io(0x25C, 2, 0, 0, 0, 0,10, 1, 0);		//C	フロントワイパー・間欠・LOW・HIGH・ウォッシャースイッチ・間欠タイマー
		add_extern_io(0x271, 1, 0, 0, 0, 0,11, 1, 0);		//C	リアワイパー・ウォッシャースイッチ
		add_extern_io(0x286, 1, 0, 0, 0, 0,12, 1, 0);		//C	ドアロックスイッチ・アンロックスイッチ
		add_extern_io(0x29C, 1, 0, 0, 0, 0,13, 1, 0);		//C	右ドア・ウィンドウ昇降スイッチ
		add_extern_io(0x2B1, 1, 0, 0, 0, 0,14, 1, 0);		//C	左ドア・ウィンドウ昇降スイッチ
		//	シャシーはインパネに転送するための情報を全て出力する
		//	パワトレ情報の出力
//		add_extern_io(0x024, 6, 0, 0, 0, 0,15, 1, 0);		//P	ブレーキ出力量
//		add_extern_io(0x039, 6, 0, 0, 0, 0,16, 1, 0);		//P	スロットル位置
		add_extern_io(0x043, 7, 0, 0, 0, 0,17, 1, 0);		//P	エンジン回転数/速度
		add_extern_io(0x062, 7, 0, 0, 0, 0,18, 1, 0);		//P	パワステ出力量/トルク
		add_extern_io(0x077, 5, 0, 0, 0, 0,19, 1, 0);		//P	シフトポジション位置
		add_extern_io(0x146, 5, 0, 0, 0, 0,20, 1, 0);		//P	ブレーキオイル量
	//X	add_extern_io(0x150, 5, 0, 0, 0, 0,21, 1, 0);		//P	ブレーキ摩耗警告・アイスバーン警報
//		add_extern_io(0x15A, 5, 0, 0, 0, 0,22, 1, 0);		//P	アンチロックブレーキ作動
	//X	add_extern_io(0x164, 6, 0, 0, 0, 0,23, 1, 0);		//P	ブレーキパッド温度・タイヤ温度
//		add_extern_io(0x16F, 6, 0, 0, 0, 0,24, 1, 0);		//P	スロットル調整
	//X	add_extern_io(0x179, 6, 0, 0, 0, 0,25, 1, 0);		//P	燃料消費率・混合気比率
		add_extern_io(0x183, 5, 0, 0, 0, 0,26, 1, 0);		//P	エンジン冷却水温度
		add_extern_io(0x18D, 5, 0, 0, 0, 0,27, 1, 0);		//P	エンジン故障
//		add_extern_io(0x198, 5, 0, 0, 0, 0,28, 1, 0);		//P	パワステ故障
		add_extern_io(0x19A, 5, 0, 0, 0, 0,29, 1, 0);		//P	エンジンスタータ駆動
	//X	add_extern_io(0x1A2, 5, 0, 0, 0, 0,30, 1, 0);		//P	横滑り防止発動
	//X	add_extern_io(0x1AD, 5, 0, 0, 0, 0,31, 1, 0);		//P	ミッション故障
		add_extern_io(0x1D3, 5, 0, 0, 0, 0,32, 1, 0);		//P	サイドブレーキ動作状態
	//X	add_extern_io(0x39E, 5, 0, 0, 0, 0,33, 1, 0);		//P	回生ブレーキ発電量
	//X	add_extern_io(0x3A9, 6, 0, 0, 0, 0,34, 1, 0);		//P	外気温度・排気温度
	//X	add_extern_io(0x3B3, 6, 0, 0, 0, 0,35, 1, 0);		//P	有害排気ガス濃度・粒子状物質濃度
		add_extern_io(0x3BD, 5, 0, 0, 0, 0,36, 1, 0);		//P	エンジンオイル量
	//X	add_extern_io(0x3C7, 5, 0, 0, 0, 0,37, 1, 0);		//P	点火不良・点火タイミング異常
		add_extern_io(0x3D4, 5, 0, 0, 0, 0,38, 1, 0);		//P	燃料残量
		add_extern_io(0x3DE, 5, 0, 0, 0, 0,39, 1, 0);		//P	バッテリー警報
	//X	add_extern_io(0x42B, 5, 0, 0, 0, 0,40, 1, 0);		//P	サイドブレーキ警報
		add_extern_io(0x482, 5, 0, 0, 0, 0,41, 1, 0);		//P	エコドライブ判定
		//	ボディー情報の出力
		add_extern_io(0x08D, 5, 0, 0, 0, 0,42, 1, 0);		//B	ウィンカー左右点灯状態
		add_extern_io(0x0A2, 5, 0, 0, 0, 0,43, 1, 0);		//B	クラクション鳴動
		add_extern_io(0x0B4, 5, 0, 0, 0, 0,44, 1, 0);		//B	エアバッグ作動スイッチ
		add_extern_io(0x1BB, 5, 0, 0, 0, 0,45, 1, 0);		//B	ポジション・ヘッドライト・ハイビーム点灯状態
		add_extern_io(0x266, 6, 0, 0, 0, 0,46, 1, 0);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャー動作状態・間欠タイマー
		add_extern_io(0x27B, 5, 0, 0, 0, 0,47, 1, 0);		//B	リアワイパー・ウォッシャー動作状態
		add_extern_io(0x290, 5, 0, 0, 0, 0,48, 1, 0);		//B	ドア開閉・施錠状態
//		add_extern_io(0x2A6, 6, 0, 0, 0, 0,49, 1, 0);		//B	右ドア・ウィンドウ位置・リミットスイッチ状態
//		add_extern_io(0x2BB, 6, 0, 0, 0, 0,50, 1, 0);		//B	左ドア・ウィンドウ位置・リミットスイッチ状態
	//X	add_extern_io(0x3E9, 5, 0, 0, 0, 0,51, 1, 0);		//B	ウィンカー球切れ警報
	//X	add_extern_io(0x3F4, 5, 0, 0, 0, 0,52, 1, 0);		//B	クラクション故障
	//X	add_extern_io(0x3FF, 5, 0, 0, 0, 0,53, 1, 0);		//B	ポジション・ヘッドライト・ハイビーム球切れ・バルブ制御故障
	//X	add_extern_io(0x40A, 5, 0, 0, 0, 0,54, 1, 0);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャーモータ・ポンプ故障
	//X	add_extern_io(0x415, 5, 0, 0, 0, 0,55, 1, 0);		//B	リアワイパー・ウォッシャーモータ・ポンプ故障
//		add_extern_io(0x420, 5, 0, 0, 0, 0,56, 1, 0);		//B	ドアロック駆動装置故障
	//X	add_extern_io(0x436, 5, 0, 0, 0, 0,57, 1, 0);		//B	右ドア・ウィンドウモータ故障
	//X	add_extern_io(0x441, 5, 0, 0, 0, 0,58, 1, 0);		//B	左ドア・ウィンドウモータ故障
	//X	add_extern_io(0x44C, 5, 0, 0, 0, 0,59, 1, 0);		//B	エアバッグ故障
//		add_extern_io(0x457, 5, 0, 0, 0, 0,60, 1, 0);		//B	シートベルトセンサー
		add_extern_io(0x461, 5, 0, 0, 0, 0,61, 1, 0);		//B	シートベルト警報
		add_extern_io(0x46C, 5, 0, 0, 0, 0,62, 1, 0);		//B	ボンネット開閉スイッチ
		add_extern_io(0x477, 5, 0, 0, 0, 0,63, 1, 0);		//B	トランク開閉スイッチ
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_BODY)
	{	//	ボディーはシャシーからの情報のみ出力する
		add_extern_io(0x01A, 6, 0, 0, 0, 0, 0, 1, 0);		//C	ブレーキ操作量
//		add_extern_io(0x02F, 6, 0, 0, 0, 0, 1, 1, 0);		//C	アクセル操作量
//		add_extern_io(0x058, 6, 0, 0, 0, 0, 2, 1, 0);		//C	ハンドル操作位置
//		add_extern_io(0x06D, 5, 0, 0, 0, 0, 3, 1, 0);		//C	シフトポジションスイッチ
		add_extern_io(0x083, 5, 0, 0, 0, 0, 4, 1, 0);		//C	ウィンカー左右・ハザードスイッチ
		add_extern_io(0x098, 5, 0, 0, 0, 0, 5, 1, 0);		//C	クラクションスイッチ
		add_extern_io(0x1A7, 5, 0, 0, 0, 0, 6, 1, 0);		//C	ポジション・ヘッドライト・ハイビームスイッチ
		add_extern_io(0x1B1, 5, 0, 0, 0, 0, 7, 1, 0);		//C	パッシングスイッチ
//		add_extern_io(0x1B8, 5, 0, 0, 0, 0, 8, 1, 0);		//C	エンジンスタートボタン
//		add_extern_io(0x1C9, 5, 0, 0, 0, 0, 9, 1, 0);		//C	サイドブレーキ
		add_extern_io(0x25C, 6, 0, 0, 0, 0,10, 1, 0);		//C	フロントワイパー・間欠・LOW・HIGH・ウォッシャースイッチ・間欠タイマー
		add_extern_io(0x271, 5, 0, 0, 0, 0,11, 1, 0);		//C	リアワイパー・ウォッシャースイッチ
		add_extern_io(0x286, 5, 0, 0, 0, 0,12, 1, 0);		//C	ドアロックスイッチ・アンロックスイッチ
		add_extern_io(0x29C, 5, 0, 0, 0, 0,13, 1, 0);		//C	右ドア・ウィンドウ昇降スイッチ
		add_extern_io(0x2B1, 5, 0, 0, 0, 0,14, 1, 0);		//C	左ドア・ウィンドウ昇降スイッチ
		//	ボディーの入力
		add_extern_io(0x08D, 1, 0, 0, 0, 0,42, 1, 0);		//B	ウィンカー左右点灯状態
		add_extern_io(0x0A2, 1, 0, 0, 0, 0,43, 1, 0);		//B	クラクション鳴動
		add_extern_io(0x0B4, 1, 0, 0, 0, 0,44, 1, 0);		//B	エアバッグ作動スイッチ
		add_extern_io(0x1BB, 1, 0, 0, 0, 0,45, 1, 0);		//B	ポジション・ヘッドライト・ハイビーム点灯状態
		add_extern_io(0x266, 2, 0, 0, 0, 0,46, 1, 0);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャー動作状態・間欠タイマー
		add_extern_io(0x27B, 1, 0, 0, 0, 0,47, 1, 0);		//B	リアワイパー・ウォッシャー動作状態
		add_extern_io(0x290, 1, 0, 0, 0, 0,48, 1, 0);		//B	ドア開閉・施錠状態
		add_extern_io(0x2A6, 2, 0, 0, 0, 0,49, 1, 0);		//B	右ドア・ウィンドウ位置・リミットスイッチ状態
		add_extern_io(0x2BB, 2, 0, 0, 0, 0,50, 1, 0);		//B	左ドア・ウィンドウ位置・リミットスイッチ状態
	//X	add_extern_io(0x3E9, 1, 0, 0, 0, 0,51, 1, 0);		//B	ウィンカー球切れ警報
	//X	add_extern_io(0x3F4, 1, 0, 0, 0, 0,52, 1, 0);		//B	クラクション故障
	//X	add_extern_io(0x3FF, 1, 0, 0, 0, 0,53, 1, 0);		//B	ポジション・ヘッドライト・ハイビーム球切れ・バルブ制御故障
	//X	add_extern_io(0x40A, 1, 0, 0, 0, 0,54, 1, 0);		//B	フロントワイパー・間欠・LOW・HIGH・ウォッシャーモータ・ポンプ故障
	//X	add_extern_io(0x415, 1, 0, 0, 0, 0,55, 1, 0);		//B	リアワイパー・ウォッシャーモータ・ポンプ故障
		add_extern_io(0x420, 1, 0, 0, 0, 0,56, 1, 0);		//B	ドアロック駆動装置故障
	//X	add_extern_io(0x436, 1, 0, 0, 0, 0,57, 1, 0);		//B	右ドア・ウィンドウモータ故障
	//X	add_extern_io(0x441, 1, 0, 0, 0, 0,58, 1, 0);		//B	左ドア・ウィンドウモータ故障
	//X	add_extern_io(0x44C, 1, 0, 0, 0, 0,59, 1, 0);		//B	エアバッグ故障
		add_extern_io(0x457, 1, 0, 0, 0, 0,60, 1, 0);		//B	シートベルトセンサー
		add_extern_io(0x461, 1, 0, 0, 0, 0,61, 1, 0);		//B	シートベルト警報
		add_extern_io(0x46C, 1, 0, 0, 0, 0,62, 1, 0);		//B	ボンネット開閉スイッチ
		add_extern_io(0x477, 1, 0, 0, 0, 0,63, 1, 0);		//B	トランク開閉スイッチ
	}
	else
	if(SELECT_ECU_UNIT == ECU_UNIT_CGW)
	{
		//	無し
	}
#endif
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANフレームデータバッファ初期値
//  
//  引数   : 無し
//  
//  説明   : フレームデータの初期値設定
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void set_frame_data(int id, int dlc, unsigned char *dat);

void defset_framedat(void)
{
	int				i;
	DEF_FRAME_DATA	*vp;
	
	for(i = 0; ; i++)
	{
		vp = &DEF_FRAME_BUFFER[i];
		if(vp->ID < 0) break;
		set_frame_data(vp->ID, 8, vp->DAT);
//		can_chainge[vp->ID] = 0;
	}
//	can_chainge_cnt = 0;
}

#endif		/*__ECU_DEFAULT_SETTING__*/

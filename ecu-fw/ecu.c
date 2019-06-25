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
// $RCSfile: ecu.c,v $
// $Revision: 1.00 $
// $Date: 2016/12/15 14:14:48 $
// 
// Copyright (c) 2016 LandF Corporation.
//
// History:
//

// 
// システム定義
//
#include	<stdio.h>
#include	<string.h>
#include	<stdlib.h>
#include	<sysio.h>
#include	"altypes.h"
#include	"iodefine.h"
#include	"timer.h"
#include	"sci.h"
#include	"usb.h"
#include	"r_can_api.h"
#include	"flash_data.h"
#include	"r_Flash_API_RX600.h"

//
// ユーザ定義
// 
#include	"memo.h"			/*	ECU 開発メモ			*/
#include	"ecu.h"				/*	ECU 共通定義			*/
#include	"ecu_io.h"			/*	ECU 入出力ポート定義	*/
#include	"can3_spi2.h"
#include	"uSD_rspi1.h"
#include	"cantp.h"			/*	CAN-TP 定義				*/

//	初期値設定ヘッダ組み込み
#include	"ecu_def_config.h"

//---------------------------------------------------------------------------------------
//
// 変数宣言 =====
//
//	<<	E2DATAフラッシュ保存変数	>>
//	ルーティングマップ
ECU_ROUT_MAP	rout_map;	//	マップ変数
//	定義保持バッファ
CYCLE_EVENTS	conf_ecu;	//	周期・イベント・リモート管理定義変数
//	ECU入出力チェックリスト
EXTERNUL_IO		ext_list[ECU_EXT_MAX];
int				ext_list_count;		//	外部入出力処理の登録数

//	<<	RAM状のみの変数	>>
//	タイムアップ待ちバッファ
CYCLE_EVENTS	wait_tup;	//	周期・イベント待ち変数
//	メッセージボックス毎の送信待ちバッファ
SEND_WAIT_BUF	send_msg[CAN_CH_MAX];
//	CANデータバッファ変数
CAN_FRAME_BUF	can_buf;
CAN_FRAME_BUF	can_random_mask;
//	メッセージボックス範囲
MBOX_SELECT_ID	mbox_sel;

/*
//	CANデータ変化フラグ
unsigned char	can_chainge[CAN_ID_MAX];
int				can_chainge_cnt;
unsigned char	can_used_mark[CAN_ID_MAX];
*/

//	データ長変換テーブル
const int	DLC_VALUE_TABLE[] = {0,1,2,3,4,5,6,7,8,8,8,8,8,8,8,8};

//	リプロモードフラグ
int	repro_mode = 0;			//	0=通常モード / 1=リプロモード

//	1msタイマーカウンタ
int	timer_count;
//	状態送信タイマー
int	status_timer;

//	ログ機能
void	logging(char *fmt, ...);
void SendPC(char *msg);

//	CANモジュール一覧(CH0～3)
extern	const can_st_ptr CAN_CHANNELS[];

//	外部入出力機器状態
EXT_IO_STATUS	exiosts;
unsigned char	exio_chg[EX_IO_MAX];
int				exio_chg_mark;
//	CAN-ID -> EX-I/O-ID 変換テーブル
unsigned char	can_to_exio[CAN_ID_MAX];	//	CAN-IDの示すデータが外部I/O管理番号となる。00～3F=適用、FF=対応無し


//	ECUシーケンス変数
int		job = 0;				//	ECU処理フロー
int		led = 0;				//	LED点滅
int		stat_update_id = 0;		//	LCDへ通知するID位置
#ifdef		__LFY_RX63N__
int		stat_comm = 1;			//	LCDへ通知する通信ポート番号0～6
#else
int		stat_comm = 0;			//	LCDへ通知する通信ポート番号0～6
#endif
int		ds_xross_pt_index = -1;	//	DS競合I/Oリスト番号保持
int		ds_x_lost_counter = 0;	//	ドライビングシミュレータ競合終了検出用カウンタ
//int		ds_conect_active = 0;	//	ドライビングシミュレータ接続フラグ

RX_MB_BUF	rxmb_buf[3];		//	受信サブバッファ

//	LEDモニタリングID設定
int				led_monit_id = 0;				//	モニターID
unsigned char	led_monit_ch = 0;				//	モニターCHビットセット
int				led_monit_first = 0x7FFFFFFF;	//	最短時間
int				led_monit_slow = 0;				//	最長時間
int				led_monit_time = 0;				//	平均時間
int				led_monit_count = 0;			//	平均化回数
int				led_monit_sample = 0;			//	サンプル回数

//	16進文字列定義
const char	HEX_CHAR[] = "0123456789ABCDEF";

//---------------------------------------------------------------------------------------
//  
//  機能   : 自局が管理するIDかをチェックする
//  
//  引数   : int id		検索ID番号
//  
//  説明   : 定義バッファから指定ID番号を検索する
//  
//  戻り値 : int		0以上：定義バッファ番号 / -1：対象外ID
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int search_target_id(int id)
{
	int				i, sid;
	CAN_ID_FORM		*idf;
	
	for(i = 0; i < MESSAGE_MAX && i < conf_ecu.CNT; i++)
	{
		sid = conf_ecu.LIST[i].ID.BIT.SID & CAN_ID_MASK;
		if(sid == id) return i;	//	ID一致
	}
	return -1;
}
//---------------------------------------------------------------------------------------
//  
//  機能   : タイムアップ待ちリストから検索
//  
//  引数   : int id		検索ID番号
//  
//  説明   : 定義バッファから指定ID番号を検索する
//  
//  戻り値 : int		0以上：定義バッファ番号 / -1：対象外ID
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int search_wait_id(int id)
{
	int				i, j, n, sid;
	CAN_ID_FORM		*idf;
	
	n = wait_tup.TOP;
	if(n < 0) return -1;	//	待ち無し
	for(i = 0; i < MESSAGE_MAX && n < MESSAGE_MAX; i++)
	{
		sid = wait_tup.LIST[n].ID.BIT.SID & CAN_ID_MASK;
		if(sid == id) return n;	//	ID一致
		n = wait_tup.LIST[n].ID.BIT.NXT;	//	次の待ちメッセージ番号
	}
	return -1;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : メールボックスの状態取得
//  
//  引数   : int ch	CANポート番号
//           int mb メールボックス番号
//  
//  説明   : 送信待ちバッファのフレームをCANレジスタへ転送し送信開始する
//  
//  戻り値 : 状態	0=未使用(送受信可能) / 1=使用中
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int can_check_mb(int ch, int mb)
{
	unsigned char	f;
	switch(ch)
	{
	default:
		return 1;
	case 0:	//	CAN0
		f = CAN0.MCTL[mb].BYTE;
		break;
	case 1:	//	CAN1
		f = CAN1.MCTL[mb].BYTE;
		break;
	case 2:	//	CAN2
		f = CAN2.MCTL[mb].BYTE;
		break;
	case 3:	//	CAN3(MCSP2515-CANコントローラ)
		f = CAN3_GetTxMCTL(mb);
		break;
	}
	return (((f & 0xC0) == 0) ? 0 : 1);
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANnポートへ送信
//  
//  引数   : SEND_WAIT_FLAME *act	送信待ちリスト構造体へのポインタ
//			 int mb					送信メールボックス番号
//  
//  説明   : タイムアウトにより削除される送信待ちバッファのフレームを空いている
//			 CANレジスタへ転送し送信開始する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int can_do_txmb_ch0(SEND_WAIT_FLAME *act, int mb)
{
	int		i;
	
	while(CAN0.MCTL[mb].BYTE != 0) CAN0.MCTL[mb].BYTE = 0x00;
	CAN0.MB[mb].ID.LONG = 0;
	CAN0.MB[mb].ID.BIT.SID = act->ID.BIT.SID;
	CAN0.MB[mb].ID.BIT.RTR = act->ID.BIT.RTR;
	CAN0.MB[mb].DLC = act->ID.BIT.DLC;
	for(i = 0; i < 8; i++) CAN0.MB[mb].DATA[i] = act->FD.BYTE[i];
//	while(CAN0.MB[mb].ID.BIT.SID != act->ID.BIT.SID) CAN0.MB[mb].ID.BIT.SID = act->ID.BIT.SID;
	CAN0.MCTL[mb].BYTE = 0x80;
	return R_CAN_OK;
}
int can_do_txmb_ch1(SEND_WAIT_FLAME *act, int mb)
{
	int		i;
	
	while(CAN1.MCTL[mb].BYTE != 0) CAN1.MCTL[mb].BYTE = 0x00;
	CAN1.MB[mb].ID.LONG = 0;
	CAN1.MB[mb].ID.BIT.SID = act->ID.BIT.SID;
	CAN1.MB[mb].ID.BIT.RTR = act->ID.BIT.RTR;
	CAN1.MB[mb].DLC = act->ID.BIT.DLC;
	for(i = 0; i < 8; i++) CAN1.MB[mb].DATA[i] = act->FD.BYTE[i];
//	while(CAN1.MB[mb].ID.BIT.SID != act->ID.BIT.SID) CAN1.MB[mb].ID.BIT.SID = act->ID.BIT.SID;
	CAN1.MCTL[mb].BYTE = 0x80;
	return R_CAN_OK;
}
int can_do_txmb_ch2(SEND_WAIT_FLAME *act, int mb)
{
	int		i;
	
	while(CAN2.MCTL[mb].BYTE != 0) CAN2.MCTL[mb].BYTE = 0x00;
	CAN2.MB[mb].ID.LONG = 0;
	CAN2.MB[mb].ID.BIT.SID = act->ID.BIT.SID;
	CAN2.MB[mb].ID.BIT.RTR = act->ID.BIT.RTR;
	CAN2.MB[mb].DLC = act->ID.BIT.DLC;
	for(i = 0; i < 8; i++) CAN2.MB[mb].DATA[i] = act->FD.BYTE[i];
//	while(CAN2.MB[mb].ID.BIT.SID != act->ID.BIT.SID) CAN2.MB[mb].ID.BIT.SID = act->ID.BIT.SID;
	CAN2.MCTL[mb].BYTE = 0x80;
	return R_CAN_OK;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 削除されるメッセージの空きメールボックス送信処理
//  
//  引数   : int ch		CANポート番号
//           int mb 	メッセージボックス番号（メールボックス番号）
//           int mi		メッセージ番号
//  
//  説明   : タイムアウトにより削除される送信待ちバッファのフレームを空いている
//			 CANレジスタへ転送し送信開始する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void can_powtx_delmb(int ch, int mb, int mi)
{
	int				i, id, bkmb;
	SEND_WAIT_FLAME	*act;
	
	if(ch >= 3) return;	//	CPU内臓CHのみ有効
	
	//	空きMB検索
	for(bkmb = 3; bkmb < 16; bkmb++)
	{
		if(can_check_mb(ch, bkmb) == 0) break;
	}
	
	if(bkmb < 16)
	{	//	メールボックス使用可能
		if(mi >= 0 && mi < MESSAGE_MAX)
		{	//	待ち有り
			act = &send_msg[ch].BOX[mb].MSG[mi];
			id = act->ID.BIT.SID;
			if(act->ID.BIT.ENB != 0)
			{	//	送出許可
				switch(ch)
				{
				case 0:	//CAN0
					can_do_txmb_ch0(act, bkmb);
					break;
				case 1:	//CAN1
					can_do_txmb_ch1(act, bkmb);
					break;
				case 2:	//CAN2
					can_do_txmb_ch2(act, bkmb);
					break;
				}
			//	logging("PowTx CH[%d] MB[%d][%d] ID=%03X\r", ch, mb, mi, id);
			}
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : メッセージボックス内メッセージ削除処理
//  
//  引数   : int ch		送信CANチャンネル番号(0～3)
//         : int mb		メッセージボックス番号
//         : int mi		メッセージ番号
//  
//  説明   : 指定CANチャンネルの送信待ちメールボックス内の指定フレームを削除する
//  
//  戻り値 : なし
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void delete_mbox_frame(int ch, int mb, int mi)
{
	int				i, id;
	int				nxt;
	CAN_ID_FORM		*idf;
	
	idf = &send_msg[ch].BOX[mb].MSG[mi].ID;
	id = idf->BIT.SID;
	nxt = idf->BIT.NXT;	//	継続チェーン保持
	idf->LONG = 0;		//	メッセージ無効化
	//	チェーン検索
	if(send_msg[ch].BOX[mb].TOP == mi && mi != nxt)
	{	//	先頭のメッセージ
		send_msg[ch].BOX[mb].TOP = nxt;
	}
	else
	{	//	中間のメッセージ
		for(i = 0; i < MESSAGE_MAX; i++)
		{
			idf = &send_msg[ch].BOX[mb].MSG[i].ID;
			if(idf->BIT.ENB == 0) continue;
			if(idf->BIT.NXT == mi)
			{	//	接続元チェーン発見
				idf->BIT.NXT = nxt;	//	チェーンから外す
			//	logging("Del CH[%d] MB[%d] P%03d ID=%03X\r", ch, mb, mi, id);
				break;
			}
		}
	}
	//	カウンタ-1
	if(send_msg[ch].BOX[mb].CNT > 0) send_msg[ch].BOX[mb].CNT--;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 送信待ちメッセージのメールボックス送信処理
//  
//  引数   : int ch	CANポート番号
//           int mb メッセージボックス番号（メールボックス番号）
//  
//  説明   : 送信待ちバッファのフレームをCANレジスタへ転送し送信開始する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void can_tx_mb(int ch, int mb)
{
	int				i;
	int				mp;
	SEND_WAIT_FLAME	*act;
	uint32_t		lwk;
//	can_frame_t		ftp;
	
	if(can_check_mb(ch, mb) == 0)
	{	//	メールボックス使用可能
		mp = send_msg[ch].BOX[mb].TOP;	//	送信待ちチェーン
		if(mp >= 0 && mp < MESSAGE_MAX)
		{	//	待ち有り
			act = &send_msg[ch].BOX[mb].MSG[mp];
			if(act->ID.BIT.ENB != 0)
			{	//	送出許可
				switch(ch)
				{
				case 0:	//CAN0
					lwk = can_do_txmb_ch0(act, mb);
					break;
				case 1:	//CAN1
					lwk = can_do_txmb_ch1(act, mb);
					break;
				case 2:	//CAN2
					lwk = can_do_txmb_ch2(act, mb);
					break;
				case 3:	//CAN3(MCP2515)
					lwk = CAN3_TxSet(mb, act);
					break;
				}
				if(lwk == R_CAN_OK)
				{	//	セットアップOK
#ifdef	SORT_TXWAITLIST_ENABLE
					send_msg[ch].BOX[mb].TOP = act->ID.BIT.NXT;	//	次回送信フレーム
#else
					send_msg[ch].BOX[mb].TOP++;
					send_msg[ch].BOX[mb].TOP &= MESSAGE_MSK;	//	読み出しポインタ更新
#endif
					send_msg[ch].BOX[mb].CNT--;
					act->ID.LONG = 0;		//	削除
				//	logging("CAN_Tx OK = %03X\r", act->ID.BIT.SID);
				}
				else
				{	//	エラー発生
					logging("CAN_TxSet Err = %08lX\r",lwk);
				}
			}
			else
			{	//	チェーンエラー
				send_msg[ch].BOX[mb].WP = 0;
				send_msg[ch].BOX[mb].TOP = -1;
				send_msg[ch].BOX[mb].CNT = 0;
			}
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : メッセージボックスCANフレーム送信処理
//  
//  引数   : 無し
//  
//  説明   : 送信待ちバッファのフレームをCANレジスタへ転送し送信開始する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void send_mbox_frame(void)
{
	int				ch;
	int				mb;
	
#ifdef	__LFY_RX63N__
	ch = CAN_TEST_LFY_CH;
#else
	for(ch = 0; ch < CAN_CH_MAX; ch++)
#endif
	{	//	CANポート番号
		for(mb = 0; mb < MESSAGE_BOXS; mb++)
		{	//	MBOX番号
			can_tx_mb(ch, mb);	//	送信待ち検索と送信実行
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : メッセージボックスCANフレーム送信バッファ積み上げ処理
//  
//  引数   : int ch		送信CANチャンネル番号(0～3)
//         : int dlc	送信データ長
//         : int rtr	送信フレーム選択(0=データ/1=リモート)
//         : int id		送信ID
//  
//  説明   : 指定CANチャンネルの送信待ちバッファへ指定データを積み上げる
//  
//  戻り値 : なし
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void add_mbox_frame(int ch, int dlc, int rtr, int id)
{
	int				i;
	int				mb, mi, mp;
	SEND_WAIT_FLAME	*msg, *old, *act;
	//	MBOX選択
	mb = (id < mbox_sel.CH[ch].MB1) ? 0 : (id < mbox_sel.CH[ch].MB2) ? 1 : 2;
	
	mi = send_msg[ch].BOX[mb].WP;						//	書き込みポインタ取得
	send_msg[ch].BOX[mb].WP = (mi + 1) & MESSAGE_MSK;	//	書き込みポインタ更新
	act = &send_msg[ch].BOX[mb].MSG[mi];				//	バッファ取得
	//	使用中かチェック
	if(act->ID.BIT.ENB != 0)
	{	//	256個前の未送信メッセージはタイムアウト、削除処理
		can_powtx_delmb(ch, mb, mi);	//	メッセージ強制送信を試みる
		delete_mbox_frame(ch, mb, mi);	//	メッセージ削除
	}
	//	メッセージ登録
	act->ID.LONG = 0;
	act->ID.BIT.SID = id;
	act->ID.BIT.RTR = rtr;	//	フレーム設定
	act->ID.BIT.ENB = 1;	//	送信有効
	act->ID.BIT.NXT = MESSAGE_END;
	act->ID.BIT.DLC = dlc;
	act->FD.LONG[0] = can_buf.ID[id].LONG[0];
	act->FD.LONG[1] = can_buf.ID[id].LONG[1];
#ifdef	SORT_TXWAITLIST_ENABLE
	//	送信待ちチェーン
	mp = send_msg[ch].BOX[mb].TOP;
	if(mp < 0 || mp >= MESSAGE_MAX)
	{	//	待ち無しなので先頭にする
		send_msg[ch].BOX[mb].TOP = mi;	//	1個目のメッセージ
	//	logging("Send new = %d:%d:%d:%08lX\r", ch, mb, mi, act->ID.LONG);
	}
	else
	{	//	待ち有りなのでチェーン接続する
		msg = &send_msg[ch].BOX[mb].MSG[mp];
		if(msg->ID.BIT.SID > id)
		{	//	先頭のメッセージより優先させる
			send_msg[ch].BOX[mb].TOP = mi;
			act->ID.BIT.NXT = mp;
		//	logging("Send top = %d:%d:%d:%08lX\r", ch, mb, mi, act->ID.LONG);
		}
		else
		{	//	後ろに接続
			for(i = 0; i < MESSAGE_MAX; i++)
			{
				old = msg;	//	1個前を保持
				mp = msg->ID.BIT.NXT;	//	次のメッセージ
				if(mp >= MESSAGE_MAX && i != mi)
				{	//	継続無しなので終端に追記
					msg->ID.BIT.NXT = mi;
				//	logging("Send add = %d:%d:%d:%08lX\r", ch, mb, mi, act->ID.LONG);
					break;
				}
				//	継続メッセージ
				msg = &send_msg[ch].BOX[mb].MSG[mp];
				if(msg->ID.BIT.SID > id)
				{	//	優先度が低いメッセージなのでこれより前に割り込む
					act->ID.BIT.NXT = mp;
					old->ID.BIT.NXT = mi;
				//	logging("Send ins = %d:%d:%d:%08lX\r", ch, mb, mi, act->ID.LONG);
					break;
				}
			}
		}
	}
#endif
	send_msg[ch].BOX[mb].CNT++;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANデータ受信処理
//  
//  引数   : int ch
//           受信CANチャンネル番号(0～3)
//         : CAN_MBOX *mbox
//           受信メッセージへのポインタ
//  
//  説明   : メッセージボックスに届いたフレームを処理する。
//           データフレームはバッファへコピー後に転送処理を実施。
//           リモートフレームは対象データを返信する。
//  
//  戻り値 : データ更新　0=無し / 1=有り
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
extern	int  retport;
int can_recv_frame(int ch, CAN_MBOX *mbox)
{
	int				ret = 0;	//	戻り値
	int				i;
	int				id;			//	ID取得
	int				dlc;		//	データバイト数
	CAN_DATA_BYTE	data;		//	データバッファ
	unsigned char	rxmsk;		//	チャンネル受信マスク
	unsigned char	txmsk;		//	チャンネル送信マスク
	unsigned char	cgw;		//	転送フラグ
	unsigned char	c1, c2, c3;	//	カウンタ
	
	id = mbox->ID.BIT.SID;

	dlc = DLC_VALUE_TABLE[mbox->DLC & 15];
//	logging("RX%d - ID=%03X\r", ch, id);
	rxmsk = 0x10 << ch;			//	受信チャンネルビット
	txmsk = 0x01 << ch;			//	送信チャンネルビット
	cgw = rout_map.ID[id].BYTE;	//	転送MAPビット
	
	if(mbox->ID.BIT.RTR == 0)
	{	//	データフレーム
		if((cgw & rxmsk) == 0 && (cgw & txmsk) != 0)
		{	//	送信専用は受信拒否
			return 0;
		}
		for(i = 0; i < dlc; i++) data.BYTE[i] = mbox->DATA[i];
		for(; i < 8; i++) data.BYTE[i] = 0;	//can_buf.ID[id].BYTE[i];
		//---------------------------------
		//	ドライビングシミュレータ競合処理
		//---------------------------------
		if(id == DS_X_POWERTRAIN_ID)
		{	//	ドライブシミュレータ競合ID受信
			if(SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN)
			{	//	パワトレのみが処理対象
				if(ds_xross_pt_index >= 0)
				{	//	検出
					if(ext_list[ds_xross_pt_index].PORT.BIT.MODE < 4)
					{	//	ECU外部入力→CAN出力モードの場合に適用
						ext_list[ds_xross_pt_index].PORT.BIT.MODE |= 4;	//	CAN入力→ECU外部出力へ変更
					}
					ds_x_lost_counter = 0;	//	継続カウンタ初期化
					//	競合を伝えるため、受信データの先頭バイトのMSBを反転する
					data.BYTE[0] |= 0x80;
				}
			}
		}
		if(data.LONG[0] == can_buf.ID[id].LONG[0] && data.LONG[1] == can_buf.ID[id].LONG[1] && id < 0x700)
		{	//	データ変化無し
			return 0;
		}
		can_buf.ID[id].LONG[0] = data.LONG[0];
		can_buf.ID[id].LONG[1] = data.LONG[1];
		ret = 1;
		//---------------------------------
		//	トランスポート層処理
		//---------------------------------
		if(id >= 0x7DF && id <= 0x7EF)	// || (id >= 0x7BE && id <= 0x7CF))
		{	//CAN-TP専用ID
			if(can_tp_job(ch, id, data.BYTE) > 0 && id != 0x7DF) return 0;	//	単独TP(UDS,OBD2)処理
		}
		//---------------------------------
		//	他ポート転送処理
		//---------------------------------
		if((cgw & rxmsk) != 0)
		{	//	転送処理対象
			txmsk = cgw & ~txmsk;
			if((txmsk & 0x01) != 0)
			{	//	CAN0転送有効
				add_mbox_frame(0, dlc, CAN_DATA_FRAME, id);
			}
			if((txmsk & 0x02) != 0)
			{	//	CAN1転送有効
				add_mbox_frame(1, dlc, CAN_DATA_FRAME, id);
			}
			if((txmsk & 0x04) != 0)
			{	//	CAN2転送有効
				add_mbox_frame(2, dlc, CAN_DATA_FRAME, id);
			}
			if((txmsk & 0x08) != 0)
			{	//	CAN3転送有効
				add_mbox_frame(3, dlc, CAN_DATA_FRAME, id);
			}
		}
	}
	else
	{	//	リモートフレーム
		if((cgw & rxmsk) != 0)
		{	//	処理対象
			if(search_target_id(id) >= 0)
			{	//	対象IDなのでデータフレーム返信
				add_mbox_frame(ch, dlc, CAN_DATA_FRAME, id);
			}
			//	転送対象確認
			txmsk = cgw & ~txmsk;
			if((txmsk & 0x01) != 0)
			{	//	CAN0転送有効
				add_mbox_frame(0, dlc, CAN_REMOTE_FRAME, id);
			}
			if((txmsk & 0x02) != 0)
			{	//	CAN1転送有効
				add_mbox_frame(1, dlc, CAN_REMOTE_FRAME, id);
			}
			if((txmsk & 0x04) != 0)
			{	//	CAN2転送有効
				add_mbox_frame(2, dlc, CAN_REMOTE_FRAME, id);
			}
			if((txmsk & 0x08) != 0)
			{	//	CAN3転送有効
				add_mbox_frame(3, dlc, CAN_REMOTE_FRAME, id);
			}
		}
	}
	return ret;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANデータ送信待ち積み上げ処理
//  
//  引数   : ECU_CYC_EVE *ev
//           周期・イベント情報
//  
//  説明   : 周期・イベント情報に従い送信バッファへ積み上げる。
//           データフレーム送信又はリモートフレーム送信処理を実施する。
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void can_send_proc(ECU_CYC_EVE *ev)
{
	int				id;			//	ID取得
	int				dlc;		//	データバイト数
	unsigned char	msk;		//	チャンネルマスク
	CAN_DATA_BYTE	*act;		//	アクティブデータ
	CAN_DATA_BYTE	*rms;		//	ランダムデータマスク
	CAN_DATA_BYTE	val;		//	ランダムデータ
	
	id = ev->ID.BIT.SID;
	dlc = DLC_VALUE_TABLE[ev->ID.BIT.DLC];
	msk = rout_map.ID[id].BYTE & ((repro_mode == 0) ? 0x0F : 0xF0);
	
	if(ev->ID.BIT.RTR == 0)
	{	//	データフレーム
		if(id == DS_X_POWERTRAIN_ID)
		{
			if(ds_xross_pt_index >= 0)
			{
				if(ext_list[ds_xross_pt_index].PORT.BIT.MODE > 3)
				{	//	DS検出中
					if(ds_x_lost_counter < 1000)
					{	//	無受信10秒待ち
						ds_x_lost_counter++;
						if(ds_x_lost_counter >= 1000)
						{
							ext_list[ds_xross_pt_index].PORT.BIT.MODE &= 3;
							//	データ初期化
							exiosts.DATA[ext_list[ds_xross_pt_index].PORT.BIT.NOM].LONG = 0;
							can_buf.ID[id].LONG[0] = 0;
							can_buf.ID[id].LONG[1] = 0;
						}
						else return;	//	DS中は送信キャンセル
					}
				}
			}
		}
		if(id < 0x700)
		{	//	700～7FFとDS競合IDはランダム情報を乗せない
			//	ランダムデータ生成追加処理
			act = &can_buf.ID[id];
			rms = &can_random_mask.ID[id];
			val.WORD[0] = rand();
			val.WORD[3] = val.WORD[0];
			val.WORD[1] = rand();
			val.WORD[2] = val.WORD[1];
			act->LONG[0] = ((act->LONG[0] + val.LONG[0]) & (~rms->LONG[0])) | (act->LONG[0] & rms->LONG[0]);
			act->LONG[1] = ((act->LONG[1] - val.LONG[1]) & (~rms->LONG[1])) | (act->LONG[1] & rms->LONG[1]);
		}
		if((msk & 0x01) != 0)
		{	//	CAN0転送有効
			add_mbox_frame(0, dlc, CAN_DATA_FRAME, id);
		}
		if((msk & 0x02) != 0)
		{	//	CAN1転送有効
			add_mbox_frame(1, dlc, CAN_DATA_FRAME, id);
		}
		if((msk & 0x04) != 0)
		{	//	CAN2転送有効
			add_mbox_frame(2, dlc, CAN_DATA_FRAME, id);
		}
		if((msk & 0x08) != 0)
		{	//	CAN3転送有効
			add_mbox_frame(3, dlc, CAN_DATA_FRAME, id);
		}
	}
	else
	{	//	リモートフレーム
		if((msk & 0x01) != 0)
		{	//	CAN0転送有効
			add_mbox_frame(0, dlc, CAN_REMOTE_FRAME, id);
		}
		if((msk & 0x02) != 0)
		{	//	CAN1転送有効
			add_mbox_frame(1, dlc, CAN_REMOTE_FRAME, id);
		}
		if((msk & 0x04) != 0)
		{	//	CAN2転送有効
			add_mbox_frame(2, dlc, CAN_REMOTE_FRAME, id);
		}
		if((msk & 0x08) != 0)
		{	//	CAN3転送有効
			add_mbox_frame(3, dlc, CAN_REMOTE_FRAME, id);
		}
	}
	//	送信処理
	send_mbox_frame();
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANデータタイムアップ送信処理(1msec周期呼び出し)
//  
//  引数   : int tcnt	1msタイマーのカウント値
//  
//  説明   : 1ms毎に呼び出す送信タイミング待ち周期・イベント送信処理
//           残時間0となったメッセージを送信バッファに積み上げる
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void can_timer_send(int tcnt)
{
	int				i, n, c;	//	ポインタ
	int				t;
	int				id;			//	ID取得
	ECU_CYC_EVE		*act;		//	周期・イベント情報
	ECU_CYC_EVE		*old;		//	周期・イベント情報
	
	i = wait_tup.TOP;
	if(i < 0)
	{
	//	logging("Wait free\r");
		return;	//	待ち無し
	}
	old = 0;
	c = 0;
	while(i < MESSAGE_MAX && i >= 0)
	{	//	継続
		c++;
		act = &wait_tup.LIST[i];	//	タイムアップ待ち行列情報
		n = act->ID.BIT.NXT;	//	継続ポインタ
		if(act->ID.BIT.ENB == 0)
		{	//	謎の無効待ちを削除
			if(i == wait_tup.TOP)
			{
				if(n < MESSAGE_MAX)
				{	//	継続を先頭に
					wait_tup.TOP = n;
				}
				else
				{	//	待ち無し
					wait_tup.TOP = -1;
				}
			}
			else
			{	//	中間を削除
				if(old != 0)
				{
					old->ID.BIT.NXT = n;
				}
			}
			act->ID.LONG = 0;	//	停止
			wait_tup.CNT--;
		}
		else
		{	//	処理有効
			if(act->TIMER.WORD.TIME == 0)
			{	//	即送信（周期メッセージにTIME=0は存在しない）
				can_send_proc(act);	//	送信バッファ積み上げ
				if(i == wait_tup.TOP)
				{	//	先頭を除去
					if(n < MESSAGE_MAX)
					{	//	継続を先頭に
						wait_tup.TOP = n;
					}
					else
					{	//	待ち無し
						wait_tup.TOP = -1;
					}
				}
				else
				{	//	中間を削除
					if(old != 0)
					{
						old->ID.BIT.NXT = n;
					}
				}
				act->ID.LONG = 0;	//	停止
				wait_tup.CNT--;
			}
			else
		//	if(act->TIMER.WORD.CNT > 0)
			{	//	残時間あり
				t = (int)act->TIMER.WORD.CNT;
				t -= tcnt;
				if(t <= 0)
				{	//	送信タイミング到達
					can_send_proc(act);	//	送信バッファ積み上げ
					if(act->ID.BIT.REP != 0)
					{	//	周期メッセージのタイマーリセット
						t += (int)act->TIMER.WORD.TIME;
					}
					else
					{	//	イベント削除
						if(old != 0)
						{
							old->ID.BIT.NXT = n;
						}
						act->ID.LONG = 0;	//	停止
						wait_tup.CNT--;
					}
				}
				act->TIMER.WORD.CNT = t;
			}
		//	else
		//	{	//	エラーイベント削除
		//		act->ID.LONG = 0;	//	停止
		//		old->ID.BIT.NXT = n;
		//		wait_tup.CNT--;
		//	}
		}
		old = act;
		if(i == n || c >= MESSAGE_MAX)
		{	//	行列異常
			wait_tup.TOP = -1;
			logging("Chain Error\r");
			break;
		}
		i = n;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 周期イベントデータ追加
//  
//  引数   : int rtr			リモートフレーム指定	0/1
//           int id				CANメッセージ番号		0～2047
//           int dlc			データバイト長			0～8
//           int enb			処理許可フラグ			0/1
//           int rep			周期フレーム指定		0/1
//           int time			周期時間又は遅延時間(ms)0～65535
//           int cnt			遅延増加時間(ms)		0～65535
//  
//  説明   : 周期・イベントの登録
//  
//  戻り値 : バッファの追加位置
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int	add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt)
{
	int				i;
	int				mb, mi, mp;
	ECU_CYC_EVE		*msg, *old, *act;
	
	//	空いているバッファを検索
	for(i = 0; i < MESSAGE_MAX; i++)
	{
		if(conf_ecu.LIST[i].ID.BIT.ENB == 0) break;
	}
	if(i < MESSAGE_MAX)
	{
		mi = i;
	}
	else
	{
		mi = conf_ecu.WP;						//	書き込みポインタ取得
		conf_ecu.WP = (mi + 1) & MESSAGE_MSK;	//	書き込みポインタ更新
	}
	act = &conf_ecu.LIST[mi];				//	バッファ取得
	//	メッセージ登録
	act->ID.LONG = 0;
	act->ID.BIT.RTR = rtr;	//	フレーム設定
	act->ID.BIT.SID = id;	//	ID設定
	act->ID.BIT.DLC = dlc;	//	データバイト数
	act->ID.BIT.ENB = enb;	//	有効
	act->ID.BIT.REP = rep;	//	周期メッセージ設定
	act->ID.BIT.NXT = MESSAGE_END;
	act->TIMER.WORD.TIME = time;
	act->TIMER.WORD.CNT = cnt;
	//	送信待ちチェーン
	mp = conf_ecu.TOP;
	if(mp < 0)
	{	//	待ち無しなので先頭にする
		conf_ecu.TOP = mi;	//	1個目のメッセージ
		logging("conf new %d\r", mi);
	}
	else
	{	//	待ち有りなのでチェーン接続する
		msg = &conf_ecu.LIST[mp];
		if(msg->ID.BIT.SID > id)
		{	//	先頭のメッセージより優先させる
			conf_ecu.TOP = mi;
			act->ID.BIT.NXT = mp;
			logging("conf top %d\r", mi);
		}
		else
		{	//	後ろに接続
			for(i = 0; i < MESSAGE_MAX; i++)
			{
				old = msg;			//	1個前を保持
				mp = msg->ID.BIT.NXT;	//	次のメッセージ
				if(mp >= MESSAGE_MAX)
				{	//	継続無しなので終端に追記
					msg->ID.BIT.NXT = mi;
					logging("conf add %d\r", mi);
					break;
				}
				//	継続メッセージ
				msg = &conf_ecu.LIST[mp];
				if(msg->ID.BIT.SID > id)
				{	//	優先度が低いメッセージなのでこれより前に割り込む
					act->ID.BIT.NXT = mp;
					old->ID.BIT.NXT = mi;
					logging("conf ins %d\r", mi);
					break;
				}
			}
		}
	}
	conf_ecu.CNT++;
	return mi;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 周期イベントデータの途中追加
//  
//  引数   : int mi		バッファ番号		0～255
//  
//  説明   : 周期・イベントの登録済み情報を連結する
//  
//  戻り値 : バッファの追加位置
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void insert_cyceve_list(int mi)
{
	int				i, id;
	int				mb, mp;
	ECU_CYC_EVE		*msg, *old, *act;
	
	act = &conf_ecu.LIST[mi];	//	バッファ取得
	if(act->ID.BIT.ENB == 0) return;	//	運転しない情報は連結しない
	id = act->ID.BIT.SID;
	act->ID.BIT.NXT = MESSAGE_END;
	//	送信待ちチェーン
	mp = conf_ecu.TOP;
	if(mp < 0)
	{	//	待ち無しなので先頭にする
		conf_ecu.TOP = mi;	//	1個目のメッセージ
		logging("conf new %d\r", mi);
	}
	else
	{	//	待ち有りなのでチェーン接続する
		msg = &conf_ecu.LIST[mp];
		if(msg->ID.BIT.SID > id)
		{	//	先頭のメッセージより優先させる
			conf_ecu.TOP = mi;
			act->ID.BIT.NXT = mp;
			logging("conf top %d\r", mi);
		}
		else
		{	//	後ろに接続
			for(i = 0; i < MESSAGE_MAX; i++)
			{
				old = msg;			//	1個前を保持
				mp = msg->ID.BIT.NXT;	//	次のメッセージ
				if(mp >= MESSAGE_MAX)
				{	//	継続無しなので終端に追記
					msg->ID.BIT.NXT = mi;
					logging("conf add %d\r", mi);
					break;
				}
				//	継続メッセージ
				msg = &conf_ecu.LIST[mp];
				if(msg->ID.BIT.SID > id)
				{	//	優先度が低いメッセージなのでこれより前に割り込む
					act->ID.BIT.NXT = mp;
					old->ID.BIT.NXT = mi;
					logging("conf ins %d\r", mi);
					break;
				}
			}
		}
	}
	conf_ecu.CNT++;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 周期イベントデータ削除
//  
//  引数   : int id				CANメッセージ番号		0～2047
//  
//  説明   : 周期・イベントの削除
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void delete_cyceve_list(int id)
{
	int				mi, mp;
	ECU_CYC_EVE		*msg, *old;
	
	mp = conf_ecu.TOP;
	while(mp >= 0)
	{
		mi = mp;
		msg = &conf_ecu.LIST[mp];
		mp = msg->ID.BIT.NXT;	//	次のメッセージ
		if(msg->ID.BIT.SID == id)
		{	//	一致
			if(conf_ecu.TOP == mi)
			{	//	先頭を削除
				conf_ecu.TOP = mp;
			}
			else
			{	//	途中を削除
				old->ID.BIT.NXT = mp;
			}
			conf_ecu.CNT--;
			return;
		}
		old = msg;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : データ更新イベント発生処理
//  
//  引数   : int id		変化が生じたID番号
//         : int tp		追加待ち時間(ms)
//  
//  説明   : CAN以外の外部機器からのデータ更新要求によって呼び出される
//           イベント処理対象のIDはタイムアップ待ち行列に追加する
//  
//  戻り値 : 負数は登録失敗 / 0以上は増加時間(ms)
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int can_id_event(int id, int tp)
{
	int				i, n, p;	//	ポインタ
	int				at;			//	遅延時間(ms)
	ECU_CYC_EVE		*new;		//	周期・イベント情報
	ECU_CYC_EVE		*act;		//	周期・イベント情報
	ECU_CYC_EVE		*old;		//	周期・イベント情報
	
	//	対象IDかを調査
	i = search_target_id(id);
	if(i < 0) return -1;	//	対象外のID
	act = &conf_ecu.LIST[i];
	if(act->ID.BIT.REP != 0)
	{	//	周期メッセージの多重登録は禁止
		n = search_wait_id(id);
		if(n >= 0) return -2;	//	登録済み
	}
	p = wait_tup.WP++;
	wait_tup.WP &= MESSAGE_MSK;
	new = &wait_tup.LIST[p];
	if(new->ID.BIT.ENB != 0)
	{
		//	未使用待ち検索
		for(p = 0; p < MESSAGE_MAX; p++)
		{
			if(wait_tup.LIST[p].ID.BIT.ENB == 0) break;	//	未使用番号発見
		}
		if(p >= MESSAGE_MAX) return -3;	//	空き無し
	}
	//	登録
	new = &wait_tup.LIST[p];
	new->ID.LONG = act->ID.LONG;						//	メッセージ条件コピー
	new->TIMER.LONG = act->TIMER.LONG;					//	設定値コピー
	at = (int)act->TIMER.WORD.CNT;						//	遅延時間(ms)
	new->TIMER.WORD.CNT = new->TIMER.WORD.TIME + tp;	//	待ち時間(ms)
	new->ID.BIT.NXT = MESSAGE_END;						//	継続無し
	new->ID.BIT.ENB = 1;								//	処理有効化
	//	待ち先頭確認
	i = wait_tup.TOP;
	if(i < 0)
	{	//	待ち無し(先頭)
		wait_tup.TOP = p;								//	先頭にする
		wait_tup.CNT = 1;								//	現在の待ちは1つ
		logging("Wait new %08lX:%d\r", new->ID.LONG, p);
		return at;										//	継続登録の遅延時間(ms)
	}
	//	挿入先検索
	old = 0;
	while(i < MESSAGE_MAX)
	{
		act = &wait_tup.LIST[i];	//	タイムアップ待ち行列情報
		n = act->ID.BIT.NXT;	//	継続ポインタ
		if(act->ID.BIT.ENB != 0)
		{	//	処理有効
			if(act->ID.BIT.SID > id)
			{	//	優先度が低いメッセージ発見
				if(i == wait_tup.TOP)
				{	//	先頭を入れ替え
					wait_tup.TOP = p;
					new->ID.BIT.NXT = i;
					wait_tup.CNT++;	//	待ち数増加
					logging("Wait top %08lX:%d→%d\r", new->ID.LONG, p, i);
					return at;
				}
				else
				{	//	途中に追加
					old->ID.BIT.NXT = p;
					new->ID.BIT.NXT = i;
					wait_tup.CNT++;	//	待ち数増加
					logging("Wait ins %08lX:%d→%d\r", new->ID.LONG, p, i);
					return at;
				}
			}
			else
			if(n >= MESSAGE_MAX)
			{	//	最後尾に追加
				act->ID.BIT.NXT = p;
				wait_tup.CNT++;	//	待ち数増加
				logging("Wait add %08lX:→%d\r", new->ID.LONG, p);
				return at;
			}
		}
		old = act;	//	ひとつ前の情報
		if(i == n)
		{	//	チェーン異常
			logging("Wait chain error %08lX:%d→%d\r", new->ID.LONG, i, n);
			break;
		}
		i = n;		//	継続ポインタ
	}
	return -4;	//	チェーンリスト破壊
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 周期イベントタイムアップ待ちデータ削除
//  
//  引数   : int id				CANメッセージ番号		0～2047
//  
//  説明   : 周期・イベントタイムアップ待ちデータの削除
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void delete_waiting_list(int id)
{
	int				mi, mp;
	ECU_CYC_EVE		*msg, *old;
	
	mp = wait_tup.TOP;
	while(mp >= 0)
	{
		mi = mp;
		msg = &wait_tup.LIST[mp];
		mp = msg->ID.BIT.NXT;	//	次のメッセージ
		if(msg->ID.BIT.SID == id)
		{	//	一致
			if(wait_tup.TOP == mi)
			{	//	先頭を削除
				wait_tup.TOP = mp;
			}
			else
			{	//	途中を削除
				old->ID.BIT.NXT = mp;
			}
			wait_tup.CNT--;
			return;
		}
		old = msg;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 通信経由外部入力値取得
//  
//  引数   : int nom	バッファ番号0～63
//		     int md		モード		0:BIT / 1:BYTE / 2:WORD / 3:LONG
//  
//  説明   : ECU外部I/O接続コネクタの入力状態を取得する
//  
//  戻り値 : int	入力値（デジタルは0/1、アナログは0～FFF）
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int port_input_ex(int nom, int md)
{
	switch(md)
	{
	default:
		break;
	case 0:	//	ビット情報
		return ((int)exiosts.DATA[nom].BIT.B0);
	case 1:	//	バイト情報
		return ((int)exiosts.DATA[nom].BYTE[0]);
	case 2:	//	ワード情報
		return ((int)exiosts.DATA[nom].WORD[0]);
	case 3:	//	ロングワード情報
		return (exiosts.DATA[nom].INTE);
	}
	return 0;
}
//---------------------------------------------------------------------------------------
//  
//  機能   : 外部出力値更新
//  
//  引数   : int nom	バッファ番号0～63
//         : int val	出力値
//		     int md		モード		0:BIT / 1:BYTE / 2:WORD / 3:LONG
//  
//  説明   : ECU外部I/O接続コネクタの出力状態を更新する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void port_output_ex(int nom, int val, int md)
{
	switch(md)
	{
	case 4:	//	ビット情報
		val = (val == 0) ? 0 : 1;
		if(exiosts.DATA[nom].BIT.B0 != val)
		{
			if(exio_chg[nom] < 100) exio_chg[nom]++;		//	データ更新通知
			exio_chg_mark++;	//	更新マーク
		}
		exiosts.DATA[nom].BIT.B0 = (val == 0) ? 0 : 1;
		break;
	case 5:	//	バイト情報
		if(exiosts.DATA[nom].BYTE[0] != val)
		{
			if(exio_chg[nom] < 100) exio_chg[nom]++;		//	データ更新通知
			exio_chg_mark++;	//	更新マーク
		}
		exiosts.DATA[nom].BYTE[0] = (unsigned char)val;
		break;
	case 6:	//	ワード情報
		if(exiosts.DATA[nom].WORD[0] != val)
		{
			if(exio_chg[nom] < 100) exio_chg[nom]++;		//	データ更新通知
			exio_chg_mark++;	//	更新マーク
		}
		exiosts.DATA[nom].WORD[0] = (unsigned short)val;
		break;
	case 7:	//	ロングワード情報
		if(exiosts.DATA[nom].INTE != val)
		{
			if(exio_chg[nom] < 100) exio_chg[nom]++;		//	データ更新通知
			exio_chg_mark++;	//	更新マーク
		}
		exiosts.DATA[nom].INTE = val;
		break;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 通信経由外部入出力更新処理
//  
//  引数   : 無し
//  
//  説明   : ECU外部I/O接続コネクタの入力状態を取得し適用先データバッファを更新する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void extern_io_update_ex(void)
{
	int						i, d, m, tp;
	int						id;		//	ID番号
	int						md;		//	I/O処理モード
	int						bp;		//	バイト位置
	int						nm;		//	I/Oポート番号
	int						msk;	//	ビットモードマスク
	EXTERNUL_IO				*act;	//	アクティブ定義ポインタ
	CAN_DATA_BYTE			*buf;	//	フレームデータバッファ
	CAN_DATA_BYTE			val;	//	コピーフレームデータ
	
	tp = 0;	//	同時入力変化の時差設定
	
	for(i = 0; i < ext_list_count; i++)
	{
		act = &ext_list[i];
		id = act->SID;
		if(id >= 0 && id < CAN_ID_MAX)
		{	//	設定有効
			nm = act->PORT.BIT.NOM;			//	バッファ番号
			md = act->PORT.BIT.MODE;		//	モード			入力：0:bit / 1:byte / 2:word / 3:long　出力：4:bit / 5:byte / 6:word / 7:long
			bp = act->PORT.BIT.BPOS;		//	バイト位置
			msk = act->PORT.BIT.MSK;		//	マスクパターン
			buf = &can_buf.ID[id];			//	データバッファへのポインタ
			val.LONG[0] = buf->LONG[0];		//	データ取得1
			val.LONG[1] = buf->LONG[1];		//	データ取得2
			//	状態取得
			d = port_input_ex(nm, (md & 3));	//	通信経由入力状態取得
			//	モード処理
			switch(md)
			{
				default:	//	無効
					break;
				case 0:		//	ビット入力
					val.BYTE[bp] &= ~msk;
					val.BYTE[bp] |= (d == 0) ? 0 : msk;
					break;
				case 4:		//	ビット出力
					d = ((val.BYTE[bp] & msk) == 0) ? 0 : 1;
					port_output_ex(nm, d, md);
					break;
				case 1:		//	バイト入力
					val.BYTE[bp] = (unsigned char)d;
					break;
				case 5:		//	バイト出力
					d = (int)((unsigned long)val.BYTE[bp]);
					port_output_ex(nm, d, md);
					break;
				case 2:		//	ワード入力
					val.BYTE[bp] = (unsigned char)(d >> 8);
					val.BYTE[bp + 1] = (unsigned char)d;
					break;
				case 6:		//	ワード出力
					d = (int)((((unsigned long)val.BYTE[bp]) << 8) | ((unsigned long)val.BYTE[bp + 1]));
					port_output_ex(nm, d, md);
					break;
				case 3:		//	ロングワード入力
					val.BYTE[bp] = (unsigned char)(d >> 24);
					val.BYTE[bp + 1] = (unsigned char)(d >> 16);
					val.BYTE[bp + 2] = (unsigned char)(d >> 8);
					val.BYTE[bp + 3] = (unsigned char)d;
					break;
				case 7:		//	ロングワード出力
					d = (int)((((unsigned long)val.BYTE[bp]) << 24) | (((unsigned long)val.BYTE[bp + 1]) << 16) | (((unsigned long)val.BYTE[bp + 2]) << 8) | ((unsigned long)val.BYTE[bp + 3]));
					port_output_ex(nm, d, md);
					break;
			}
			if(val.LONG[0] != buf->LONG[0] || val.LONG[1] != buf->LONG[1])
			{	//	変化有り
				buf->LONG[0] = val.LONG[0];				//	データ更新1
				buf->LONG[1] = val.LONG[1];				//	データ更新2
				if(exio_chg[nm] < 100) exio_chg[nm]++;	//	CANデータ更新カウント
				exio_chg_mark++;						//	更新マーク
				tp = can_id_event(id, tp);				//	CANデータ更新通知
			}
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 外部入力値取得
//  
//  引数   : int nom	ポート番号（0～33）
//  
//  説明   : ECU外部I/O接続コネクタの入力状態を取得する
//  
//  戻り値 : int	入力値（デジタルは0/1、アナログは0～FFF）
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int port_input(int nom)
{
	switch(nom)
	{
		case 0:		return (X_DB_0 == 0) ? 0 : -1;
		case 1:		return (X_DB_1 == 0) ? 0 : -1;
		case 2:		return (X_DB_2 == 0) ? 0 : -1;
		case 3:		return (X_DB_3 == 0) ? 0 : -1;
		case 4:		return (X_DB_4 == 0) ? 0 : -1;
		case 5:		return (X_DB_5 == 0) ? 0 : -1;
		case 6:		return (X_DB_6 == 0) ? 0 : -1;
		case 7:		return (X_DB_7 == 0) ? 0 : -1;
		case 8:		return (X_DB_8 == 0) ? 0 : -1;
		case 9:		return (X_DB_9 == 0) ? 0 : -1;
		case 10:	return (X_DB_10 == 0) ? 0 : -1;
		case 11:	return (X_DB_11 == 0) ? 0 : -1;
		case 12:	return (X_DB_12 == 0) ? 0 : -1;
		case 13:	return (X_DB_13 == 0) ? 0 : -1;
		case 14:	return (X_DB_14 == 0) ? 0 : -1;
		case 15:	return (X_DB_15 == 0) ? 0 : -1;
		case 16:	return (X_DB_16 == 0) ? 0 : -1;
		case 17:	return (X_DB_17 == 0) ? 0 : -1;
		case 18:	return (X_DB_18 == 0) ? 0 : -1;
		case 19:	return (X_DB_19 == 0) ? 0 : -1;
		case 20:	return (X_DB_20 == 0) ? 0 : -1;
		case 21:	return (X_DB_21 == 0) ? 0 : -1;
		case 22:	return (X_DB_22 == 0) ? 0 : -1;
		case 23:	return (X_DB_23 == 0) ? 0 : -1;
		case 24:	return (X_AN_0 & 0x0FFF);
		case 25:	return (X_AN_1 & 0x0FFF);
		case 26:	return (X_AN_2 & 0x0FFF);
		case 27:	return (X_AN_3 & 0x0FFF);
		case 28:	return (X_AN_4 & 0x0FFF);
		case 29:	return (X_AN_5 & 0x0FFF);
		case 30:	return (X_AN_6 & 0x0FFF);
		case 31:	return (X_AN_7 & 0x0FFF);
	}
	return 0;
}
//---------------------------------------------------------------------------------------
//  
//  機能   : 外部出力値更新
//  
//  引数   : int nom	ポート番号0～33
//         : int val	出力値
//  
//  説明   : ECU外部I/O接続コネクタの出力状態を更新する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void port_output(int nom, int val)
{
	switch(nom)
	{
		case 0:		Y_DB_0 = (val == 0) ? 0 : 1;	break;
		case 1:		Y_DB_1 = (val == 0) ? 0 : 1;	break;
		case 2:		Y_DB_2 = (val == 0) ? 0 : 1;	break;
		case 3:		Y_DB_3 = (val == 0) ? 0 : 1;	break;
		case 4:		Y_DB_4 = (val == 0) ? 0 : 1;	break;
		case 5:		Y_DB_5 = (val == 0) ? 0 : 1;	break;
		case 6:		Y_DB_6 = (val == 0) ? 0 : 1;	break;
		case 7:		Y_DB_7 = (val == 0) ? 0 : 1;	break;
		case 8:		Y_DB_8 = (val == 0) ? 0 : 1;	break;
		case 9:		Y_DB_9 = (val == 0) ? 0 : 1;	break;
		case 10:	Y_DB_10 = (val == 0) ? 0 : 1;	break;
		case 11:	Y_DB_11 = (val == 0) ? 0 : 1;	break;
		case 12:	Y_DB_12 = (val == 0) ? 0 : 1;	break;
		case 13:	Y_DB_13 = (val == 0) ? 0 : 1;	break;
		case 14:	Y_DB_14 = (val == 0) ? 0 : 1;	break;
		case 15:	Y_DB_15 = (val == 0) ? 0 : 1;	break;
		case 16:	Y_DB_16 = (val == 0) ? 0 : 1;	break;
		case 17:	Y_DB_17 = (val == 0) ? 0 : 1;	break;
		case 18:	Y_DB_18 = (val == 0) ? 0 : 1;	break;
		case 19:	Y_DB_19 = (val == 0) ? 0 : 1;	break;
		case 20:	Y_DB_20 = (val == 0) ? 0 : 1;	break;
		case 21:	Y_DB_21 = (val == 0) ? 0 : 1;	break;
		case 22:	Y_DB_22 = (val == 0) ? 0 : 1;	break;
		case 23:	Y_DB_23 = (val == 0) ? 0 : 1;	break;
		case 24:	Y_AN_0 = (val >> 2) & 0x03FF;	break;
		case 25:	Y_AN_1 = (val >> 2) & 0x03FF;	break;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 外部入出力更新処理
//  
//  引数   : 無し
//  
//  説明   : ECU外部I/O接続コネクタの入力状態を取得し適用先データバッファを更新する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void extern_io_update(void)
{
	int						i, d, m, tp;
	int						id;		//	ID番号
	int						md;		//	I/O処理モード
	int						ng;		//	データ反転フラグ
	int						sz;		//	アクセスサイズ
	int						bp;		//	バイト位置
	int						dlc;	//	データ長
	int						nm;		//	I/Oポート番号
	int						msk;	//	ビットモードマスク
	unsigned char			*pat;	//	パターンデータ
	EXTERNUL_IO				*act;	//	アクティブ定義ポインタ
	CAN_DATA_BYTE			*buf;	//	フレームデータバッファ
	CAN_DATA_BYTE			val;	//	コピーフレームデータ
	
	tp = 0;	//	同時入力変化の時差設定

	for(i = 0; i < ext_list_count; i++)
	{
		act = &ext_list[i];
		id = act->SID;
		if(id >= 0 && id < CAN_ID_MAX)
		{	//	設定有効
			nm = act->PORT.BIT.NOM;			//	ポート番号
			md = act->PORT.BIT.MODE;		//	モード			0:無効 / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit値 / 6:3bit値 / 7:4bit値
			ng = act->PORT.BIT.NEG;			//	反転フラグ
			sz = act->PORT.BIT.SIZE;		//	アクセスサイズ	0:BIT / 1～7:nBYTE
			bp = act->PORT.BIT.BPOS;		//	バイト位置
			dlc = act->PORT.BIT.DLC;		//	比較データ長
			msk = act->PORT.BIT.MSK;		//	マスクパターン
			pat = act->PAT;					//	パターンデータ
			buf = &can_buf.ID[act->SID];		//	データバッファへのポインタ
			val.LONG[0] = buf->LONG[0];		//	データ取得1
			val.LONG[1] = buf->LONG[1];		//	データ取得2
			//	状態取得
			m = 1;
			d = port_input(nm);				//	入力状態取得
			if(md > 2 && md < 5)
			{	//	ADCワード処理
				m = 0x0FFF;	//	12bit
			}
			else
			{	//	ビット処理
				if(md > 4)
				{	//	複数ビット
					d = ((d & 1) << 1) | (port_input(nm + 1) & 1);
					m = (m << 1) | 1;
					if(md > 5)
					{	//	複数ビット
						d = ((d & 1) << 1) | (port_input(nm + 2) & 1);
						m = (m << 1) | 1;
						if(md > 6)
						{	//	複数ビット
							d = ((d & 1) << 1) | (port_input(nm + 3) & 1);
							m = (m << 1) | 1;
						}
					}
				}
			}
			//	反転要求
			if(ng)
			{	//	データ反転
				d = ~d;
				d &= m;
			}
			//	モード処理
			switch(md)
			{
				default:	//	無効
					break;
				case 5:		//	2bit値
				case 6:		//	3bit値
				case 7:		//	4bit値
				case 1:		//	ビット入力
					if(sz == 0)
					{	//	ビット操作
						val.BYTE[bp] &= ~msk;
						val.BYTE[bp] |= (d == 0) ? 0 : msk;
					}
					else
					{	//	パターン転写
						memcpy(&val.BYTE[bp], &pat[sz * d], sz);
					}
					tp = can_id_event(id, tp);	//	CANバスへ通知
					break;
				case 2:		//	ビット出力
					if(sz == 0)
					{	//	ビット操作
						d = ((val.BYTE[bp] & msk) == 0) ? 0 : 1;
						if(ng) d = ~d & 1;
						port_output(nm, d);
					}
					else
					{	//	パターン比較
						for(d = 0; d < 2; d++)
						{
							if(memcmp(&val.BYTE[bp], &pat[(sz * d)], sz) == 0)
							{	//	一致
								if(ng) d = ~d & 1;
								port_output(nm, d);
								break;
							}
						}
					}
					break;
				case 3:		//	ADCワード入力
					val.BYTE[bp] = (unsigned char)(d >> 8);
					val.BYTE[bp + 1] = (unsigned char)(d & 0xFF);
					tp = can_id_event(id, tp);	//	CANバスへ通知
					break;
				case 4:		//	DACワード出力
					d = (((unsigned long)val.BYTE[bp]) << 8) | ((unsigned long)val.BYTE[bp + 1]);
					if(act->PORT.BIT.MODE & 0x08)
					{	//	反転
						d = (0 - d) & 0x0FFF;	//	12bit化
					}
					port_output(nm, d);
					break;
			}
			if(val.LONG[0] != buf->LONG[0] || val.LONG[1] != buf->LONG[1])
			{	//	変化有り
				buf->LONG[0] = val.LONG[0];		//	データ更新1
				buf->LONG[1] = val.LONG[1];		//	データ更新2
			//	if(can_chainge[act->SID] < 100) can_chainge[act->SID]++;		//	CANデータ更新通知
			//	can_chainge_cnt = 1;	//	更新マーク
			}
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 外部入出力登録処理
//  
//  引数   : int id		フレームID番号		0～2047(000～7FF)
//           int mode	I/O処理モード		0:無効 / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit値 / 6:3bit値 / 7:4bit値
//           int neg	データ反転指定		0:通常 / 1:反転
//           int size	アクセスサイズ		0:BIT / 1～7:nBYTE
//           int bpos	バイト位置			0～7
//           int dlc	データバイト長		0～8
//           int nom	適用ポート番号		0～63
//           int msk	マスクパターン		00～FF
//           unsigned char *pat	パターンデータ	24byte
//  
//  説明   : ECU外部I/O接続コネクタの入力状態を取得し適用先データバッファを更新する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat)
{
	int				i;
	EXTERNUL_IO		*act;
	CAN_DATA_BYTE	*cmk;
	
	i = ext_list_count;
	if(i >= 0 && i < ECU_EXT_MAX)
	{
		ext_list_count++;				//	総数更新
		can_to_exio[id] = i;			//	逆引きマップ設定
		act = &ext_list[i];				//	登録先ポインタ
		act->SID = id;					//	フレームID番号
__break__
		act->PORT.BIT.MODE = mode;		//	I/O処理モード
		act->PORT.BIT.NEG = neg;		//	データ反転指定
		act->PORT.BIT.SIZE = size;		//	アクセスサイズ
		act->PORT.BIT.BPOS = bpos;		//	バイト位置
		act->PORT.BIT.DLC = dlc;		//	データバイト長
		act->PORT.BIT.MSK = msk;		//	マスクパターン
		act->PORT.BIT.NOM = nom;		//	適用ポート番号
		if(pat != 0)
		{
			memcpy(act->PAT, pat, 24);		//	パターンデータ
		}
		cmk = &can_random_mask.ID[id];
		//	マスク処理
		switch(mode)
		{
		default:	//	マスク無効
			cmk->LONG[0] = -1;
			cmk->LONG[1] = -1;
			break;
		case 0:		//	ビット入力
			cmk->BYTE[bpos] = msk;
			break;
		case 1:		//	バイト入力
			cmk->BYTE[bpos] = 0xFF;
			break;
		case 2:		//	ワード入力
			cmk->BYTE[bpos] = 0xFF;
			cmk->BYTE[bpos + 1] = 0xFF;
			break;
		case 3:		//	ロングワード入力
			cmk->BYTE[bpos] = 0xFF;
			cmk->BYTE[bpos + 1] = 0xFF;
			cmk->BYTE[bpos + 2] = 0xFF;
			cmk->BYTE[bpos + 3] = 0xFF;
			break;
		}
		//	DS競合IDのリスト番号
		if(id == DS_X_POWERTRAIN_ID)
		{	//	DS専用ルール
			ds_xross_pt_index = i;	//	保持
			rout_map.ID[id].BYTE = 0x11;
		}
		else
		{	//	ルーティングマップ設定
			rout_map.ID[id].BYTE = (mode < 4) ? 0x01 : 0x10;	//	入出力方向で送受信フラグを定義(0..3:入力値→CAN送信 / 4..7:出力値←CAN受信)
		}
		return i;
	}
	return -1;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : データバッファ直接設定
//  
//  引数   : int id		フレームID番号		0～2047(000～7FF)
//           int dlc	データ長
//           unsigned char	*dat	データバッファへのポインタ
//  
//  説明   : CANデータバッファへ値を設定する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void set_frame_data(int id, int dlc, unsigned char *dat)
{
	int				i;
	CAN_DATA_BYTE	*act;
	
	act = &can_buf.ID[id];	//	データバッファ選択
//	can_chainge[id]++;		//	状態変化フラグ
	memcpy(act, dat, dlc);	//	コピー
//	can_used_mark[id]++;
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECU初期化処理
//  
//  引数   : 無し
//  
//  説明   : ECUデータ領域の初期化と周期メッセージの登録
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_init(void)
{
	int						i, j, addr;
	POINTER_MULTI_ACCESS	s, d;
	EXTERNUL_IO				*act;
	CAN_DATA_BYTE			*cmk;
	
	Init_FlashData();

	//	変数初期化
	memset(&wait_tup, 0, sizeof(wait_tup));					//	周期・イベント待ち初期化
	memset(&send_msg, 0, sizeof(send_msg));					//	メッセージボックス毎の送信待ちバッファ初期化
	memset(&can_buf, 0, sizeof(can_buf));					//	CANデータバッファ初期化
//	memset(&can_chainge, 0, sizeof(can_chainge));			//	CANデータ変化フラグ初期化
	memset(&mbox_sel, 0, sizeof(mbox_sel));					//	メッセージボックス範囲初期化
	memset(&exiosts, 0, sizeof(exiosts));					//	外部入出力状態初期化
	memset(&exio_chg, 0, sizeof(exio_chg));					//	外部入出力状態初期化
	memset(&can_random_mask, 0, sizeof(can_random_mask));	//	ランダムコードマスク初期化
	memset(&rxmb_buf, 0, sizeof(rxmb_buf));					//	受信バッファ
	memset(&conf_ecu, 0, sizeof(conf_ecu));					//	イベントリスト
	memset(ext_list, 0, sizeof(ext_list));					//	ECU入出力チェックリスト初期化
	memset(can_to_exio, -1, sizeof(can_to_exio));			//	ECU入出力変換テーブル初期化

	exio_chg_mark = 0;
	
//	memset(&can_used_mark, 0, sizeof(can_used_mark));
	
	ext_list_count = 0;										//	チェックリスト数リセット
	wait_tup.TOP = -1;
//	can_chainge_cnt = 0;
	for(i = 0; i < CAN_CH_MAX; i++)
	{
		mbox_sel.CH[i].MB1 = MBOX_POINT_1;	//	MBOX0 : ID=000～MBOX_POINT_1
		mbox_sel.CH[i].MB2 = MBOX_POINT_2;	//	MBOX1 : ID=MBOX_POINT_1～MBOX_POINT_2 , これ以上は MBOX2
		for(j = 0; j < MESSAGE_BOXS; j++)
		{
			send_msg[i].BOX[j].TOP = -1;
		}
	}

	//	マップ取得
	addr = g_flash_BlockAddresses[BLOCK_DB0];
	//	ブランクチェック
	if((i = R_FlashDataAreaBlankCheck(addr, BLANK_CHECK_ENTIRE_BLOCK)) == FLASH_NOT_BLANK)
	{	//	データフラッシュ有効(8KB: 0x00100000 - 0x00101FFF)
		//	ルーティングマップ読み出し
		s.LONG = ADDRESS_OF_ROOTMAP;
		d.MAP = &rout_map;
		memcpy(d.UB, s.UB, sizeof(rout_map));			//	マップ初期化
		//	イベントリスト読み出し
		s.LONG = ADDRESS_OF_CYCEVE;
		d.CYE = &conf_ecu.LIST[0];
		memcpy(d.UB, s.UB, sizeof(ECU_CYC_EVE) * MESSAGE_MAX);	//	周期・イベント・リモート管理定義初期化
		j = CAN_ID_MAX;
		for(i = 0; i < MESSAGE_MAX; i++)
		{
			if(conf_ecu.LIST[i].ID.LONG == 0)	//ID.BIT.ENB != 0)
			{	//	終端
				conf_ecu.WP = i;
				break;
			}
			if(conf_ecu.LIST[i].ID.BIT.SID < j)
			{
				j = conf_ecu.LIST[i].ID.BIT.SID;
				conf_ecu.TOP = i;
			}
			conf_ecu.CNT++;
		}
		//	I/O設定読み出し
		s.LONG = ADDRESS_OF_IOLIST;
		d.EXL = ext_list;
		memcpy(d.UB, s.UB, sizeof(ext_list));			//	ECU入出力チェックリスト初期化
		//	リスト有効登録数チェック
		for(i = 0; i < ECU_EXT_MAX; i++, ext_list_count++)
		{
			if(ext_list[i].PORT.LONG == 0)	//ext_list[i].SID < 0 || ext_list[i].PORT.BIT.MODE == 0)
			{	//	終端
				break;
			}
			ext_list_count++;				//	総数更新
			act = &ext_list[i];				//	登録先ポインタ
			can_to_exio[act->SID] = i;		//	逆引きマップ設定
			cmk = &can_random_mask.ID[act->SID];
			//	マスク処理
			switch(act->PORT.BIT.MODE)
			{
			default:	//	マスク無効
				cmk->LONG[0] = -1;
				cmk->LONG[1] = -1;
				break;
			case 0:		//	ビット入力
				cmk->BYTE[act->PORT.BIT.BPOS] = act->PORT.BIT.MSK;
				break;
			case 1:		//	バイト入力
				cmk->BYTE[act->PORT.BIT.BPOS] = 0xFF;
				break;
			case 2:		//	ワード入力
				cmk->BYTE[act->PORT.BIT.BPOS] = 0xFF;
				cmk->BYTE[act->PORT.BIT.BPOS + 1] = 0xFF;
				break;
			case 3:		//	ロングワード入力
				cmk->BYTE[act->PORT.BIT.BPOS] = 0xFF;
				cmk->BYTE[act->PORT.BIT.BPOS + 1] = 0xFF;
				cmk->BYTE[act->PORT.BIT.BPOS + 2] = 0xFF;
				cmk->BYTE[act->PORT.BIT.BPOS + 3] = 0xFF;
				break;
			}
			//	DS競合IDのリスト番号
			if(act->SID == DS_X_POWERTRAIN_ID)
			{	//	DS専用ルール
				ds_xross_pt_index = i;	//	保持
				rout_map.ID[act->SID].BYTE = 0x11;
			}
		//	else
		//	{	//	ルーティングマップ設定
		//		//	入出力方向で送受信フラグを定義(0..3:入力値→CAN送信 / 4..7:出力値←CAN受信)
		//		rout_map.ID[act->SID].BYTE = (act->PORT.BIT.MODE < 4) ? 0x01 : 0x10;
		//	}
		}
	}
	else
	if(i == FLASH_BLANK)
	{	//	保存情報なし
		//	デフォルト設定呼び出し
		defset_rootmap();		//	マップ初期値
		defset_confecu();		//	周期・イベント初期値
	//	defset_extlist();		//	外部入出力定義初期値
		defset_extlist_ex();	//	通信経由外部入出力定義初期値
	}
	//	フレームデータ初期値
	defset_framedat();
	//	初回イベント登録
	for(i = 0; i < conf_ecu.CNT; i++)
	{
		if(conf_ecu.LIST[i].ID.BIT.ENB != 0)
		{	//	周期・イベント
			j = conf_ecu.LIST[i].ID.BIT.SID;
			can_id_event(j, 0);
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANポート設定
//  
//  引数   : 無し
//  
//  説明   : CANで使用するポート条件の初期化
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void can_pin_init(void)
{
	SYSTEM.PRCR.WORD = 0xA503;	//	ポート設定許可
	MPC.PWPR.BIT.B0WI = 0;		//	
	MPC.PWPR.BIT.PFSWE = 1;		//	

#ifdef	__LFY_RX63N__
	// CAN1の端子設定
	PORT5.PMR.BIT.B5 = 0;		//	周辺機能
	PORT5.PMR.BIT.B4 = 0;		//	周辺機能
	PORT5.PODR.BIT.B5 = 1;		//	P55 -- CRX1
	PORT5.PODR.BIT.B4 = 1;		//	P54 -- CTX1
	PORT5.PDR.BIT.B5 = 0;		//	P55 -- CRX1
	PORT5.PDR.BIT.B4 = 1;		//	P54 -- CTX1
	MPC.P55PFS.BYTE = 0x10;		//	CRX1
	MPC.P54PFS.BYTE = 0x10;		//	CTX1
	PORT5.PMR.BIT.B5 = 1;		//	周辺機能
	PORT5.PMR.BIT.B4 = 1;		//	周辺機能
#else
	//	CANドライバS端子制御ポート
	PORT6.PODR.BYTE = 0xF0;		//	ポート初期化
	PORT6.PDR.BIT.B6 = 1;
	PORT6.PDR.BIT.B7 = 0;
	PORT6.PDR.BIT.B0 = 1;		//	P60 -- Port-out CAN0S
	PORT6.PDR.BIT.B1 = 1;		//	P61 -- Port-out CAN1S
	PORT6.PDR.BIT.B2 = 1;		//	P62 -- Port-out CAN2S
	PORT6.PDR.BIT.B3 = 1;		//	P63 -- Port-out CAN3S
	PORT6.PDR.BIT.B5 = 1;		//	P65 -- Port-out LED
	
	//	IDチェックLED
	PORTE.PMR.BIT.B0 = 0;		//	PORT
	PORTE.PODR.BIT.B0 = 1;		//	PE0
	PORTE.PDR.BIT.B0 = 1;		//	LED output


	// CAN0の端子設定
	PORT3.PMR.BIT.B3 = 0;		//	周辺機能
	PORT3.PMR.BIT.B2 = 0;		//	周辺機能
	PORT3.PODR.BIT.B3 = 1;		//	P32 -- CRX0
	PORT3.PODR.BIT.B2 = 1;		//	P32 -- CTX0
	PORT3.PDR.BIT.B3 = 0;		//	P33 -- CRX0
	PORT3.PDR.BIT.B2 = 1;		//	P32 -- CTX0
	MPC.P33PFS.BYTE = 0x10;		//	CRX0
	MPC.P32PFS.BYTE = 0x10;		//	CTX0
	PORT3.PMR.BIT.B3 = 1;		//	周辺機能
	PORT3.PMR.BIT.B2 = 1;		//	周辺機能

	// CAN1の端子設定
	PORT5.PMR.BIT.B5 = 0;		//	周辺機能
	PORT5.PMR.BIT.B4 = 0;		//	周辺機能
	PORT5.PODR.BIT.B5 = 1;		//	P55 -- CRX1
	PORT5.PODR.BIT.B4 = 1;		//	P54 -- CTX1
	PORT5.PDR.BIT.B5 = 0;		//	P55 -- CRX1
	PORT5.PDR.BIT.B4 = 1;		//	P54 -- CTX1
	MPC.P55PFS.BYTE = 0x10;		//	CRX1
	MPC.P54PFS.BYTE = 0x10;		//	CTX1
	PORT5.PMR.BIT.B5 = 1;		//	周辺機能
	PORT5.PMR.BIT.B4 = 1;		//	周辺機能

	// CAN2の端子設定
	PORT6.PMR.BIT.B7 = 0;		//	周辺機能
	PORT6.PMR.BIT.B6 = 0;		//	周辺機能
	PORT6.PODR.BIT.B7 = 1;		//	P67 -- CRX2
	PORT6.PODR.BIT.B6 = 1;		//	P66 -- CTX2
	PORT6.PDR.BIT.B7 = 0;		//	P67 -- CRX2
	PORT6.PDR.BIT.B6 = 1;		//	P66 -- CTX2
	MPC.P67PFS.BYTE = 0x10;		//	CRX2
	MPC.P66PFS.BYTE = 0x10;		//	CTX2
	PORT6.PMR.BIT.B7 = 1;		//	周辺機能
	PORT6.PMR.BIT.B6 = 1;		//	周辺機能
#endif
/*
	// CAN3の端子設定(RSPI2経由)
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
	PORT9.PODR.BYTE = 0x00;		//	ポート初期化

	PORTD.PMR.BIT.B2 = 1;		//	周辺機能	MISOC
	PORTD.PMR.BIT.B1 = 1;		//	周辺機能	MOSIC
	PORTD.PMR.BIT.B3 = 1;		//	周辺機能	RSPCKC
	PORTD.PMR.BIT.B4 = 1;		//	周辺機能	SSLC0
	PORTD.PMR.BIT.B0 = 1;		//	周辺機能	CINT
	PORTD.PMR.BIT.B6 = 1;		//	周辺機能	CRX0BF
	PORTD.PMR.BIT.B7 = 1;		//	周辺機能	CRX1BF

	MPC.PD2PFS.BYTE = 0x0D;		//	MISOC		SO
	MPC.PD1PFS.BYTE = 0x0D;		//	MOSIC		SI
	MPC.PD3PFS.BYTE = 0x0D;		//	RSPCKC		SCK
	MPC.PD4PFS.BYTE = 0x0D;		//	SSLC0		/CS
	MPC.PD0PFS.BYTE = 0x40;		//	IRQ0		/INT
	MPC.PD6PFS.BYTE = 0x40;		//	IRQ6		/RX0BF
	MPC.PD7PFS.BYTE = 0x40;		//	IRQ7		/RX1BF
*/
	MPC.PWPR.BIT.PFSWE = 0;		//	
	MPC.PWPR.BIT.B0WI = 1;		//	
	SYSTEM.PRCR.WORD = 0xA500;	//	ポート設定禁止
}

//---------------------------------------------------------------------------------------
//  
//  機能   : CANモジュールレジスタ設定
//  
//  引数   : int ch		初期化チャンネル
//           int bps	通信速度(bps)	※車用:500kbps
//  
//  説明   : CANモジュール運転条件の初期化
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void can_init(int ch, int bps)
{
	int			i, j;
	int			x, tbit, tseg1, tseg2, sjw, brp, fc;
	uint32_t	lwk;
    volatile struct st_can __evenaccess * can_block_p = CAN_CHANNELS[ch];

	if(ch < 3)
	{	//	CPU内臓CAN0,1,2のみ有効
		//	CANポート初期化
		logging("CAN%d Init\r",ch);
		lwk = R_CAN_Create(ch);			// CANモジュール初期化
		if ( lwk != R_CAN_OK )
		{
			logging("CAN_Create = %08lX\r",lwk);
		}
		else
		{
			logging("CAN_Create OK\r");
		}
		//	リセット実施
		lwk = R_CAN_Control(ch, RESET_CANMODE); //RESETモード設定
		if ( lwk != R_CAN_OK )
		{
			logging("CAN_Control = %08lX\r",lwk);
		}
		else
		{
			logging("R_CAN_Control RESET_CANMODE\r");
		}
		//	設定モード
		lwk = R_CAN_Control(ch, HALT_CANMODE); //HALTモード設定
		if ( lwk != R_CAN_OK )
		{
			logging("CAN_Control = %08lX\r",lwk);
		}
		else
		{
			logging("R_CAN_Control HALT_CANMODE\r");
		}
		
		//	通信速度設定
		//	BRP = 1～1024(0～1023)
		//	SS = 1
		//	TSEG1 = 4～16(3～15) : TSEG2 = 2～8(1～7)
		//	SJW = 1～4(0～3)
		//	※TSEG1 > TSEG2 ≧ SJW
		//	最小	4 : 2 : 1	TCANBIT = SS + TSEG1 + TSEG2 = 1 + 4 + 2 = 5 ～ 1 + 16 + 8 = 25
		//	
		//	PCLK = 48,000,000 Hz
		//	BPS  =    500,000 bps
		//
		//	BRP = PCLK / BPS / TBIT		※TBIT = 16TQ (5～25TQ) になるように求める
		//		= 48000000 / 500000 = 96 / 16 = 6
		//		= 6
		//
		//	tBit= PCLK / BPS / BRP = 96 / 6 = 16Tq
		//
		//	TSEG2 = (tBit - SS) / 4 = (16 - 1) / 4 = 3.75
		//		  = 3
		//	TSEG1 = tBit - SS - TSEG2 = 16 - 1 - 3 = 12
		//		  = 12
		//	SJW	  = TSEG2 / 2 = 3 / 2 = 1.5
		//		  = 1
		//
		for(x = 25; x > 4; x--)
		{
			if((48000000 % x) != 0) continue;
			fc = 48000000 / x;
			if((fc % bps) > (bps * 17 / 1000)) continue;	//	誤差1.7%未満
			brp = 48000000 / bps / x;		//	1025 > brp > 0
			if(brp < 1) continue;			//	範囲外
			if(brp > 1024) continue;		//	範囲外
			tbit = 48000000 / bps / brp;	//	26 > tbit > 4
			if(tbit != x) continue;			//	逆算不一致
			if(tbit < 5) continue;			//	範囲外
			if(tbit > 25) continue;			//	範囲外
			tseg2 = tbit / 3;				//	9 > tseg2 > 1
			if(tseg2 < 2) continue;			//	範囲外
			if(tseg2 > 8) continue;			//	範囲外
			tseg1 = tbit - 1 - tseg2;		//	17 > tseg1 > 3
			if(tseg1 <= tseg2) continue;	//	範囲外
			if(tseg1 > 16) continue;		//	範囲外
			sjw = (tseg2 + 1) / 2;			//	5 > sjw > 0
			if(sjw < 1) continue;			//	範囲外
			if(sjw > 4) continue;			//	範囲外
			break;
		}
		if(x == 4)
		{	//	失敗したので500kbpsに初期化する
			logging("can_init: invalid parameter (%ld)\r", bps);
			brp = 6;
			tbit = 16;
			tseg1 = 10;
			tseg2 = 5;
			sjw = 4;
		}
		can_block_p->BCR.BIT.CCLKS = 0;		//	PCLK(48MHz)
		can_block_p->BCR.BIT.BRP = brp - 1;
		can_block_p->BCR.BIT.TSEG1 = tseg1 - 1;
		can_block_p->BCR.BIT.TSEG2 = tseg2 - 1;
		can_block_p->BCR.BIT.SJW = sjw - 1;
		logging("BPS=%d SS=1 BRP=%d TSEG1=%d TSEG2=%d SJW=%d TBIT=%d\r", bps, brp, tseg1, tseg2, sjw, tbit);
		
		//	メールボックスデータ初期化
		for(i = 0 ;i < 32 ;i++)
		{
			can_block_p->MKR[i].LONG = 0;
			can_block_p->MB[i].ID.LONG = 0;
		}
		//	マスク・フィルタ無効
		can_block_p->MKIVLR.LONG = 0;	//0x0000FFFF;
		
		//	メールボックス制御初期化
		for(i = 0 ;i < 32 ;i++)
		{
			while(can_block_p->MCTL[i].BYTE != 0) can_block_p->MCTL[i].BYTE = 0;
		}
		//	運転開始
		can_block_p->MIER.LONG = 0xFFFFFFFF;		//	割り込み許可
		lwk = R_CAN_Control(ch, OPERATE_CANMODE);	//	OPERATEモード設定
		if ( lwk != R_CAN_OK ) {
			logging("R_CAN_Control = %08lX\r", lwk);
		}
		else
		{
			logging("R_CAN_Control OPERATE_CANMODE\r");
		}
		//	受信許可
		for(i = 16 ;i < 32 ;i++)
		{
			can_block_p->MCTL[i].BYTE = 0x40;	//	MB16～32は受信専用
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECUタイマーカウンタ更新処理
//  
//  引数   : 無し
//  
//  説明   : 1msタイマーにより呼び出される
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_timeup(void)
{
	timer_count++;
	status_timer++;
	after_call(0, -1, ecu_timeup);	//	高速タイマー呼び出し
}

void ecu_rxmb_proc(void)
{
	int	ch;
	for(ch = 0; ch < 3; ch++)
	{
		while(rxmb_buf[ch].WP != rxmb_buf[ch].RP)
		{
			can_recv_frame(ch, (void *)&rxmb_buf[ch].MB[rxmb_buf[ch].RP++]);	//	受信データ取得
			rxmb_buf[ch].RP &= (RX_MB_BUF_MAX-1);
		}
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECU運転シーケンス
//  
//  引数   : 無し
//  
//  説明   : main()関数から呼び出す。CANフレームの送出管理、定刻処理、イベント処理を行う。
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_job(void)
{
	int				i, ch, t;

	if(job > 1)
	{
		ecu_rxmb_proc();	//	受信を毎回チェック
		send_mbox_frame();	//	送信を毎回チェック
	}

	switch(job)
	{
		case 0:	//	初期化処理
			job++;
			logging("Init ECU\r");
			ecu_init();			//	ECUバッファ初期化
			can_pin_init();		//	CPUピン設定
			timer_count = 0;
			status_timer = 0;
			ecu_timeup();
		case 1:
			if(timer_count > 0)
			{	//	タイマー更新有り
				led += timer_count;
				timer_count = 0;	//	タイムカウント初期化
				if(led >= 500)
				{
					led -= 500;
					PORT6.PODR.BYTE ^= 0x20;	//	LED反転
					logging("Start ECU\r");
					job++;
#ifdef	__LFY_RX63N__
					i = CAN_TEST_LFY_CH;
#else
					for(i = 0; i < CAN_CH_MAX; i++)
#endif
					{
						can_init(i, 500000);	//	CANポート初期化
					}
				}
			}
			break;
		//------------------------------------------
		//	以下は繰り返し処理
		//------------------------------------------
		case 2:	//	I/Oアップデート
			job++;
		//	extern_io_update();		//	ECU本体のI/O
			extern_io_update_ex();	//	RS-232C通信経由のI/O
			break;
		case 3:	//	1msサイクル処理
			job = 5;
			if(timer_count > 0)
			{	//	タイマー更新有り
			//	logging("Timer ECU\r");
				t = timer_count;
				timer_count = 0;	//	タイムカウント初期化
				led += t;
				if(led >= 500)
				{
					led -= 500;
					PORT6.PODR.BYTE ^= 0x20;	//	LED反転
				}
				can_timer_send(t);	//	タイムアップ処理
			}
			break;
		case 4:	//	CAN送信処理
		//	job = 6;
			job++;
		//	send_mbox_frame();
			break;
		case 5:	//	CAN受信処理
			job++;
#ifndef	CAN_RX_INT_ENB
			CAN0.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
			while(CAN0.MSSR.BIT.SEST == 0)
			{	//	結果有り
				i = (int)CAN0.MSSR.BIT.MBNST;
				if(!CAN0.MCTL[i].BIT.RX.INVALDATA)
				{
					if(CAN0.MCTL[i].BIT.RX.NEWDATA)
					{
						can_recv_frame(0, (void *)&CAN0.MB[i]);	//	受信データ取得
					}
					CAN0.MCTL[i].BYTE = 0x40;	//	受信再開
				}
			}
			CAN1.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
			while(CAN1.MSSR.BIT.SEST == 0)
			{	//	結果有り
				i = (int)CAN1.MSSR.BIT.MBNST;
				if(!CAN1.MCTL[i].BIT.RX.INVALDATA)
				{
					if(CAN1.MCTL[i].BIT.RX.NEWDATA)
					{
						can_recv_frame(1, (void *)&CAN1.MB[i]);	//	受信データ取得
					}
					CAN1.MCTL[i].BYTE = 0x40;	//	受信再開
				}
			}
			CAN2.MSMR.BYTE = 0;	//	受信MBのSENTDATA検索
			while(CAN2.MSSR.BIT.SEST == 0)
			{	//	結果有り
				i = (int)CAN2.MSSR.BIT.MBNST;
				if(!CAN2.MCTL[i].BIT.RX.INVALDATA)
				{
					if(CAN2.MCTL[i].BIT.RX.NEWDATA)
					{
						can_recv_frame(2, (void *)&CAN2.MB[i]);	//	受信データ取得
					}
					CAN2.MCTL[i].BYTE = 0x40;	//	受信再開
				}
			}
			break;
#endif
		case 6:	//	RS-232C送信処理
			job = 2;
#if	1
			if(stat_update_id < EX_IO_MAX)
			{	//	継続通知送信
				if(sci_txbytes(stat_comm) == 0)
				{	//	送信バッファ空き
					int		j, k;
					int		r;
					char	buf[128];
					r = 0;
					//	1電文は80文字程度として送出する
					for(; r < 80 && stat_update_id < EX_IO_MAX; stat_update_id++)
					{	//	1行分送信する
						if(exio_chg[stat_update_id] != 0)
						{	//	データ更新ID
							exio_chg[stat_update_id] = 0;
							if(r == 0)
							{	//	行頭は「EXU」で始まる
								buf[r++] = 'E';
								buf[r++] = 'X';
								buf[r++] = 'U';
							}
							//	I/O-IDコード2桁
							k = stat_update_id;
							buf[r++] = ' ';
							buf[r++] = HEX_CHAR[((k >> 4) & 15)];
							buf[r++] = HEX_CHAR[(k & 15)];
							//	更新データ4バイト
							k = exiosts.DATA[stat_update_id].INTE;
							j = 8;
							if(((k >> 28) & 15) == 0)
							{
								j--;
								if(((k >> 24) & 15) == 0)
								{
									j--;
									if(((k >> 20) & 15) == 0)
									{
										j--;
										if(((k >> 16) & 15) == 0)
										{
											j--;
											if(((k >> 12) & 15) == 0)
											{
												j--;
												if(((k >> 8) & 15) == 0)
												{
													j--;
													if(((k >> 4) & 15) == 0)
													{
														j--;
													}
												}
											}
										}
									}
								}
							}
							switch(j)
							{
							case 8:
								buf[r++] = HEX_CHAR[((k >> 28) & 15)];
							case 7:
								buf[r++] = HEX_CHAR[((k >> 24) & 15)];
							case 6:
								buf[r++] = HEX_CHAR[((k >> 20) & 15)];
							case 5:
								buf[r++] = HEX_CHAR[((k >> 16) & 15)];
							case 4:
								buf[r++] = HEX_CHAR[((k >> 12) & 15)];
							case 3:
								buf[r++] = HEX_CHAR[((k >> 8) & 15)];
							case 2:
								buf[r++] = HEX_CHAR[((k >> 4) & 15)];
							case 1:
								buf[r++] = HEX_CHAR[(k & 15)];
							case 0:
								break;
							}
						}
					}
					if(r > 0)
					{
						buf[r++] = '\r';
						buf[r] = 0;
						sci_puts(stat_comm, buf);	//	送信実行
					}
				}
			}
			else
			if(status_timer >= 500)
			{	//	500ms周期で強制全送信
				status_timer = 0;	//	タイマークリア
			//	if(sci_txbytes(stat_comm) == 0 && exio_chg_mark > 0)
			//	{	//	送信バッファ空き、状態変化有り
					stat_update_id = 0;		//	送信開始IDセット
					exio_chg_mark = 0;	//	変化フラグクリア
			//	}
				for(i = 0; i < ext_list_count; i++) exio_chg[(ext_list[i].PORT.BIT.NOM)] = 1;
			}
			else
			if(sci_txbytes(stat_comm) == 0 && exio_chg_mark > 0)
			{	//	送信バッファ空き、状態変化有りで個別送信
				status_timer = 0;	//	タイマークリア
				stat_update_id = 0;		//	送信開始IDセット
				exio_chg_mark = 0;	//	変化フラグクリア
			}
#else
			if(stat_update_id < CAN_ID_MAX)
			{	//	継続通知送信
				if(sci_txbytes(stat_comm) == 0)
				{	//	送信バッファ空き
					int		j, k;
					int		r;
					char	buf[128];
					r = 0;
					//	1電文は80文字程度として送出する
					for(; r < 80 && stat_update_id < EX_IO_MAX; stat_update_id++)
					{	//	1行分送信する
						if(can_chainge[stat_update_id] != 0)
						{	//	データ更新ID
							can_chainge[stat_update_id] = 0;
							if(r == 0)
							{	//	行頭は「ECU」で始まる
								buf[r++] = 'E';
								buf[r++] = 'C';
								buf[r++] = 'U';
							}
							//	IDコード3桁
							k = stat_update_id;
							buf[r++] = ' ';
							buf[r++] = HEX_CHAR[((k >> 8) & 15)];
							buf[r++] = HEX_CHAR[((k >> 4) & 15)];
							buf[r++] = HEX_CHAR[(k & 15)];
							//	フレームデータ8バイト
							for(j = 0; j < 8; j++)
							{	//	データ
								k = (int)can_buf.ID[stat_update_id].BYTE[j];
								buf[r++] = HEX_CHAR[((k >> 4) & 15)];
								buf[r++] = HEX_CHAR[(k & 15)];
							}
						}
					}
					if(r > 0)
					{
						buf[r++] = '\r';
						buf[r] = 0;
						sci_puts(stat_comm, buf);	//	送信実行
					}
				}
			}
			else
			if(status_timer >= 20)
			{	//	50Hz(20ms)周期
				status_timer = 0;	//	タイマークリア
				if(sci_txbytes(stat_comm) == 0 && can_chainge_cnt > 0)
				{	//	送信バッファ空き、状態変化有り
					stat_update_id = 0;		//	送信開始IDセット
					can_chainge_cnt = 0;	//	変化フラグクリア
				}
			}
#endif
			break;
		default:
			job = 2;
			break;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECU運転パラメータ通知
//  
//  引数   : 無し
//  
//  説明   : 現在のパラメータ値をCOM1に送信する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void SendPC(char *msg);

void ecu_status(char *cmd)
{
	int		i, j;
	int		ch, mb, id;
	char	tx[39], c;

	if(cmd == 0 || *cmd == 0) return;
	while(*cmd == ' ') cmd++;
	switch(*cmd++)
	{
	case 'L':	//	conf_ecu変数表示
		logging("conf_ecu WP=%d TOP=%d CNT=%d\r", conf_ecu.WP, conf_ecu.TOP, conf_ecu.CNT);
		for(i = 0; i < MESSAGE_MAX; i++)
		{
			if(conf_ecu.LIST[i].ID.BIT.ENB != 0)
			{
				logging("No.%d RTR=%d SID=%03X DLC=%d ENB=%d REP=%d NXT=%d TIM=%d CNT=%d DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r",
					i, 
					(int)conf_ecu.LIST[i].ID.BIT.RTR,
					(int)conf_ecu.LIST[i].ID.BIT.SID,
					(int)conf_ecu.LIST[i].ID.BIT.DLC,
					(int)conf_ecu.LIST[i].ID.BIT.ENB,
					(int)conf_ecu.LIST[i].ID.BIT.REP,
					(int)conf_ecu.LIST[i].ID.BIT.NXT,
					(int)conf_ecu.LIST[i].TIMER.WORD.TIME,
					(int)conf_ecu.LIST[i].TIMER.WORD.CNT,
					(int)can_buf.ID[i].BYTE[0], (int)can_buf.ID[i].BYTE[1], (int)can_buf.ID[i].BYTE[2], (int)can_buf.ID[i].BYTE[3],
					(int)can_buf.ID[i].BYTE[4], (int)can_buf.ID[i].BYTE[5], (int)can_buf.ID[i].BYTE[6], (int)can_buf.ID[i].BYTE[7]
					);
			}
		}
		break;
	case 'W':	//	wait_tup変数表示
		logging("wait_tup WP=%d TOP=%d CNT=%d\r", wait_tup.WP, wait_tup.TOP, wait_tup.CNT);
		for(i = 0; i < MESSAGE_MAX; i++)
		{
			if(wait_tup.LIST[i].ID.BIT.ENB != 0)
			{
				logging("No.%d RTR=%d SID=%03X DLC=%d ENB=%d REP=%d NXT=%d TIM=%d CNT=%d DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r",
					i, 
					(int)wait_tup.LIST[i].ID.BIT.RTR,
					(int)wait_tup.LIST[i].ID.BIT.SID,
					(int)wait_tup.LIST[i].ID.BIT.DLC,
					(int)wait_tup.LIST[i].ID.BIT.ENB,
					(int)wait_tup.LIST[i].ID.BIT.REP,
					(int)wait_tup.LIST[i].ID.BIT.NXT,
					(int)conf_ecu.LIST[i].TIMER.WORD.TIME,
					(int)conf_ecu.LIST[i].TIMER.WORD.CNT,
					(int)can_buf.ID[i].BYTE[0], (int)can_buf.ID[i].BYTE[1], (int)can_buf.ID[i].BYTE[2], (int)can_buf.ID[i].BYTE[3],
					(int)can_buf.ID[i].BYTE[4], (int)can_buf.ID[i].BYTE[5], (int)can_buf.ID[i].BYTE[6], (int)can_buf.ID[i].BYTE[7]
					);
			}
		}
		break;
		
	case 'S':	//	send_msg変数表示
		for(ch = 0; ch < CAN_CH_MAX; ch++)
		{
			for(mb = 0; mb < MESSAGE_BOXS; mb++)
			{
				logging("send_msg[%d].BOX[%d] WP=%d TOP=%d CNT=%d\r", ch, mb, send_msg[ch].BOX[mb].WP, send_msg[ch].BOX[mb].TOP, send_msg[ch].BOX[mb].CNT);
				for(i = 0; i < MESSAGE_MAX; i++)
				{
					if(send_msg[ch].BOX[mb].MSG[i].ID.BIT.ENB != 0)
					{
						logging("No.%d RTR=%d SID=%03X DLC=%d ENB=%d NXT=%d DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r",
							i,
							(int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.RTR,
							(int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.SID,
							(int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.DLC,
							(int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.ENB,
							(int)send_msg[ch].BOX[mb].MSG[i].ID.BIT.NXT,
							(int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[0], (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[1], 
							(int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[2], (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[3], 
							(int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[4], (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[5], 
							(int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[6], (int)send_msg[ch].BOX[mb].MSG[i].FD.BYTE[7]
							);
					}
				}
			}
		}
		break;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECUフレームデータ取得
//  
//  引数   : char *cmd	コマンド文字列
//  
//  説明   : 現在の指定IDフレームデータを返信する
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_get_command(char *cmd)
{
	int		i, j;
	int		id;
	char	tx[39], c;

	if(cmd == 0 || *cmd == 0) return;
	while(*cmd == ' ') cmd++;
	if(*cmd < '0' || *cmd > '7') return;
	SendPC("ECU ");
	while(*cmd >= '0' && *cmd <= '7')
	{
		id = 0;
		for(i = 0; i < 4; i++)
		{
			c = *cmd;
			if(c >= 'a' && c <= 'f') c -= 0x27;
			if(c >= 'A' && c <= 'F') c -= 7;
			if(c < 0x30 || c > 0x3F) break;
			id = (id << 4) | (int)(c & 0x0F);
			cmd++;
		}
		if(i == 3 && id < CAN_ID_MAX)
		{	//	ID正常
			j = 0;
			tx[j++] = HEX_CHAR[(id >> 8) & 0x0F];
			tx[j++] = HEX_CHAR[(id >> 4) & 0x0F];
			tx[j++] = HEX_CHAR[id & 0x0F];
			for(i = 0; i < 8; i++)
			{
				c = can_buf.ID[id].BYTE[i];
				tx[j++] = HEX_CHAR[(c >> 4) & 0x0F];
				tx[j++] = HEX_CHAR[c & 0x0F];
			}
			if(*cmd >= ' ') tx[j++] = ' ';
			tx[j++] = 0;
			SendPC(tx);
		}
		while(*cmd == ' ') cmd++;
	}
	SendPC("\r");
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECUフレームデータ書き換え
//  
//  引数   : char *cmg	コマンド文字列
//  
//  説明   : 指定IDのフレームデータを書き替える
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_set_command(char *cmd)
{
	int				i, j, f;
	int				id, tp;
	int				xid[32];
	int				xwp = 0;
	char			tx[39], c;
	unsigned char	d;

	if(cmd == 0 || *cmd == 0) return;
	while(*cmd == ' ') cmd++;
	if(*cmd < '0' || *cmd > '7') return;
	//	書き換え処理
	tp = 0;	//	遅延時間初期化
	while(*cmd >= '0' && *cmd <= '7')
	{
		//	対象ID取得
		id = 0;
		//	16進文字列からバイナリデータ16進3桁(0x000～FFF)取得
		for(i = 0; i < 3; i++)
		{
			c = *cmd;
			if(c >= 'a' && c <= 'f') c -= 0x27;
			if(c >= 'A' && c <= 'F') c -= 7;
			if(c < 0x30 || c > 0x3F) break;
			id = (id << 4) | (int)(c & 0x0F);
			cmd++;
		}
		//	IDチェック
		if(i == 3 && id < CAN_ID_MAX)
		{	//	ID正常、データ処理
			f = 0;
			//	16進文字列からバイナリデータ取得
			for(i = 0; i < 8; i++)
			{
				c = *cmd;
				if(c >= 'a' && c <= 'f') c -= 0x27;
				if(c >= 'A' && c <= 'F') c -= 7;
				if(c < 0x30 || c > 0x3F) break;
				d = (unsigned char)(c & 0x0F);
				cmd++;
				c = *cmd;
				if(c >= 'a' && c <= 'f') c -= 0x27;
				if(c >= 'A' && c <= 'F') c -= 7;
				if(c < 0x30 || c > 0x3F) break;
				d = (d << 4) | (unsigned char)(c & 0x0F);
				cmd++;
				can_buf.ID[id].BYTE[i] = d;
				f++;
			}
			if(f > 0)
			{	//	データ書き換え有り
				i = can_tp_job(-1, id, &can_buf.ID[id].BYTE);
				if(i > 0)
				{	//	応答有り
					xid[xwp++] = i;
				}
				else
				{	//	そのまま返す
					tp += can_id_event(id, tp);
					xid[xwp++] = id;
				}
				if(xwp >= 32) break;	//	個数制限
			}
		}
		while(*cmd == ' ') cmd++;
	}
/*
	//	応答電文
	if(xwp > 0)
	{	//	書き換え発生、通知
		SendPC("ECU ");
		for(f = 0; f < xwp; f++)
		{
			id = xid[f];
			j = 0;
			tx[j++] = HEX_CHAR[(id >> 8) & 0x0F];
			tx[j++] = HEX_CHAR[(id >> 4) & 0x0F];
			tx[j++] = HEX_CHAR[id & 0x0F];
			for(i = 0; i < 8; i++)
			{
				c = can_buf.ID[id].BYTE[i];
				tx[j++] = HEX_CHAR[(c >> 4) & 0x0F];
				tx[j++] = HEX_CHAR[c & 0x0F];
			}
			if((f + 1) < xwp) tx[j++] = ' ';
			tx[j++] = 0;
			SendPC(tx);
		}
		SendPC("\r");
	}*/
}

//---------------------------------------------------------------------------------------
//  
//  機能   : ECUフレームデータ送信
//  
//  引数   : char *cmg	コマンド文字列
//  
//  説明   : 指定IDのフレームデータを書き替える
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_put_command(char *cmd)
{
	ECU_CYC_EVE		mbox;
	int				i, j, f;
	int				id, tp;
	char			c;
	unsigned char	d;

	if(cmd == 0 || *cmd == 0) return;
	while(*cmd == ' ') cmd++;
	if(*cmd < '0' || *cmd > '7') return;
	//	書き換え処理
	while(*cmd >= '0' && *cmd <= '7')
	{
		//	対象ID取得
		id = 0;
		for(i = 0; i < 3; i++)
		{
			c = *cmd;
			if(c >= 'a' && c <= 'f') c -= 0x27;
			if(c >= 'A' && c <= 'F') c -= 7;
			if(c < 0x30 || c > 0x3F) break;
			id = (id << 4) | (int)(c & 0x0F);
			cmd++;
		}
		if(i == 3 && id < CAN_ID_MAX)
		{	//	ID正常、データ処理
			f = 0;
			for(i = 0; i < 8; i++)
			{
				c = *cmd;
				if(c >= 'a' && c <= 'f') c -= 0x27;
				if(c >= 'A' && c <= 'F') c -= 7;
				if(c < 0x30 || c > 0x3F) break;
				d = (unsigned char)(c & 0x0F);
				cmd++;
				c = *cmd;
				if(c >= 'a' && c <= 'f') c -= 0x27;
				if(c >= 'A' && c <= 'F') c -= 7;
				if(c < 0x30 || c > 0x3F) break;
				d = (d << 4) | (unsigned char)(c & 0x0F);
				cmd++;
				can_buf.ID[id].BYTE[i] = d;
				f++;
			}
			if(f > 0)
			{	//	データ書き換え有り
				i = can_tp_job(-1, id, &can_buf.ID[id].BYTE);
				if(i == 0)
				{	//	応答無し
					mbox.ID.LONG = 0;
					mbox.TIMER.LONG = 0;
					mbox.ID.BIT.SID = id;
					mbox.ID.BIT.ENB = 1;
					mbox.ID.BIT.DLC = f;
					can_send_proc(&mbox);
				}
			}
			else
			{	//	リモートフレーム発行
				mbox.ID.LONG = 0;
				mbox.TIMER.LONG = 0;
				mbox.ID.BIT.SID = id;
				mbox.ID.BIT.ENB = 1;
				mbox.ID.BIT.RTR = 1;
				mbox.ID.BIT.DLC = 8;
				can_send_proc(&mbox);
			}
		}
		while(*cmd == ' ') cmd++;
	}
}

//---------------------------------------------------------------------------------------
//  
//  機能   : 通信経由外部入力情報の更新
//  
//  引数   : char *cmg	コマンド文字列
//  
//  説明   : 指定入力バッファのデータを書き替える
//  
//  戻り値 : 無し
//  
//  備考   : 
//  
//---------------------------------------------------------------------------------------
void ecu_input_update(char *cmd)
{
	int				i, f;
	int				id;
	char			c;
	unsigned long	d;

	if(cmd == 0 || *cmd == 0) return;
	//	書き換え処理
	while(*cmd != 0)
	{
		while(*cmd == ' ') cmd++;
		//	対象ID取得
		id = 0;
		for(i = 0; i < 2; i++)
		{
			c = *cmd;
			if(c >= 'a' && c <= 'f') c -= 0x27;
			if(c >= 'A' && c <= 'F') c -= 7;
			if(c < 0x30 || c > 0x3F) break;
			id = (id << 4) | (int)(c & 0x0F);
			cmd++;
		}
		if(i == 2 && id < EX_IO_MAX)
		{	//	ID正常、データ処理
			f = 0;
			d = 0;
			for(i = 0; i < 8; i++)
			{
				c = *cmd;
				if(c >= 'a' && c <= 'f') c -= 0x27;
				if(c >= 'A' && c <= 'F') c -= 7;
				if(c < 0x30 || c > 0x3F) break;
				d = (d << 4) | (unsigned char)(c & 0x0F);
				cmd++;
				f++;
			}
			if(f > 0)
			{	//	データ書き換え有り
				exiosts.DATA[id].LONG = d;
			}
		}
		else break;
	}
}


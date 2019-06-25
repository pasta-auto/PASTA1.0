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
//	REPROGRAM	プロトコル処理
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
#include "altypes.h"
#include "timer.h"
#include "flash_data.h"
#include "r_Flash_API_RX600.h"
#include "flash_rom.h"
#include "ecu.h"			/*	ECU 共通定義			*/
#include "can3_spi2.h"		/*	CAN3 定義				*/

/*
	Reprogram 処理の概要

	CANフレーム定義
	
	ECU個別指定
	CAN-ID	0x7C0～7C7
	
					ECUクエリ受信(0x7DF)※すべてのECUが処理対象となる
				+---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
		Byte	|		0		|		1		|		2		|		3		|		4		|		5		|		6		|		7		|
				+---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
		意味	|	データ長	|	コマンド	|	付帯情報	|				|				|				|				|				|
				+---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
		範囲	|	  0～7		|	00 ～ FF	|	00 ～ FF	|				|				|				|				|				|
				+---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
	
	CAN-ID	0x7C0	ECU0(パワトレ系)への要求
			0x7C1	ECU1(ボディー系)への要求
			0x7C2	ECU2(シャシー系)への要求
			0x7C7	ECU3(CGW-DTC)への要求

	ECU応答ID
	CAN-ID	0x7C8	ECU0(パワトレ系)からの応答
			0x7C9	ECU1(ボディー系)からの応答
			0x7CA	ECU2(シャシー系)からの応答
			0x7CF	ECU3(CGW-DTC)からの応答
*/

extern  void	restart_pos(void);

//----------------------------------------------------------------------------------------
//	OBD2パケット共用体定義
//----------------------------------------------------------------------------------------
typedef	union	__repro_query_frame__	{
	unsigned char	BYTE[8];
	struct	{
		unsigned char	LEN;		//	データ長
		unsigned char	CMD;		//	コマンド
		unsigned char	ADH;		//	アドレス上位8ビット
		unsigned char	ADL;		//	アドレス下位8ビット
		unsigned char	DAT[4];		//	付帯情報
	}	PACK;
}	REPRO_QUERY_FRAME;

//----------------------------------------------------------------------------------------
//	コマンドコード定義
//----------------------------------------------------------------------------------------
#define		ALL_CONFIG_SAVE			0x07		/*	運用データをデータフラッシュへ一括保存			*/

#define		GET_ROUTING_MAP			0x08		/*	ルーティングマップ取得							*/
#define		SET_ROUTING_MAP			0x0A		/*	ルーティングマップ変更							*/
#define		ERA_ROUTING_MAP			0x0E		/*	ルーティングマップROM消去						*/
#define		SAV_ROUTING_MAP			0x0F		/*	ルーティングマップROM保存						*/

#define		DEL_CYCEVE_LIST			0x11		/*	稼働中の周期送信・イベントリストから指定IDを削除	*/
#define		NEW_CYCEVE_LIST			0x12		/*	稼働中の周期送信・イベントリストへ新規IDを追加	*/
#define		GET_CYCEVE_LIST			0x13		/*	周期送信・イベントリスト取得					*/
#define		SET_CYCEVE_LIST			0x14		/*	周期送信・イベントリスト変更					*/
#define		GET_CYCEVE_LIST1		0x18		/*	周期送信・イベントリストID取得					*/
#define		GET_CYCEVE_LIST2		0x19		/*	周期送信・イベントリストTimer取得				*/
#define		SET_CYCEVE_LIST1		0x1A		/*	周期送信・イベントリストID変更					*/
#define		SET_CYCEVE_LIST2		0x1B		/*	周期送信・イベントリストTimer変更				*/
#define		ERA_CYCEVE_LIST			0x1E		/*	周期送信・イベントリストROM消去					*/
#define		SAV_CYCEVE_LIST			0x1F		/*	周期送信・イベントリストROM保存					*/

#define		GET_EXT_IO_LIST			0x28		/*	外部入出力リスト取得							*/
#define		SET_EXT_IO_LIST			0x2A		/*	外部入出力リスト変更							*/
#define		ERA_EXT_IO_LIST			0x2E		/*	外部入出力リストROM消去							*/
#define		SAV_EXT_IO_LIST			0x2F		/*	外部入出力リストROM保存							*/

#define		READ_FIRMWARE			0x38		/*	ファームウェア取得								*/
#define		UPDATE_FIRMWARE			0x3C		/*	ファームウェアROM保存							*/
#define		REMOVE_FIRMWARE			0x3D		/*	ファームウェアROM消去							*/
#define		COPY_FIRMWARE			0x3E		/*	ファームウェアROM書き込み用バッファへの転送		*/
#define		WRITE_FIRMWARE			0x3F		/*	ファームウェアROM書き込み（消去を先に実行する）	*/

//----------------------------------------------------------------------------------------
//	変数定義
//----------------------------------------------------------------------------------------
REPRO_QUERY_FRAME	repro_req;		//	要求データ
REPRO_QUERY_FRAME	repro_ret;		//	応答データ
unsigned char		fw_image[128];	//	書き込み専用メモリバッファ
unsigned long		fw_address;		//	書き込み専用メモリアドレス

/*
//________________________________________________________________________________________
//
//  boot_copy
//----------------------------------------------------------------------------------------
//  機能説明
//	  APPをROMにコピーする
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
int fw_copy(unsigned long sapp, unsigned long eapp)
{
	int				err = 0;
	unsigned long   rom = 0xFFF40000;
	unsigned long   app = sapp; //0x00000000;
	unsigned long   ape = eapp; //0x00020000;
	unsigned long   stp = 0x00008000;

//	reset_fcu();	//  FCUリセット
//	flash_init();	//  FCUイニシャライズ
	for(; app < ape && err == 0; app += stp, rom += stp)
	{
		_di();
		//  ブロック消去(ブロックサイズ)
		if(flash_erase_rom(rom) != 0)
		{
			err++;
		}
		//  書き込み(128byte単位)
		if(flash_write_rom(rom, app, stp) != 0)
		{
			err++;
		}
		_ei();
	}
	return ((err == 0) ? 1 : 0);
}
*/
//----------------------------------------------------------------------------------------
//	BootCopy 処理
//----------------------------------------------------------------------------------------
int bootcopy(void)
{
	unsigned long   rom = 0xFFF40000;
	unsigned long   app = 0x00000000;
	unsigned long   ape = 0x00020000;
	unsigned long   stp = 128;

	*((unsigned long *)0x60) = (unsigned long)restart_pos;

//	reset_fcu();	//  FCUリセット
//	flash_init();	//  FCUイニシャライズ

	for(; app < ape; app += stp, rom += stp)
	{
		_di();
		if(R_FlashWrite(rom, app, (unsigned short)stp) != FLASH_SUCCESS)
		{
			_ei();
			return 0;
		}
		_ei();
	}
//	if(R_FlashWrite(0xFFF00000, 0, 0x00020000) != FLASH_SUCCESS) return 0;
//	return fw_copy(0x00000000, 0x00020000);
	return 1;
}

//________________________________________________________________________________________
//
//  erace_rom
//----------------------------------------------------------------------------------------
//  機能説明
//	  ROMの指定エリアを消去する(0xFFF00000～0xFFF7FFFF)
//  引数
//	  sadr		開始番地
//	  eadr		終了番地
//  戻り
//	  char*	   メッセージ
//________________________________________________________________________________________
//
int bootclear(void)
{
	int blk;

//	reset_fcu();	//  FCUリセット
//	flash_init();	//  FCUイニシャライズ

	for(blk = BLOCK_53; blk > BLOCK_37; blk--)
	{
		_di();
		if(R_FlashErase(blk) != FLASH_SUCCESS)
		{
			_ei();
			return 0;
		}
		_ei();
	}
	return 1;
}

//----------------------------------------------------------------------------------------
//	ECU運用データの一括保存
//----------------------------------------------------------------------------------------
int ecu_data_write(void)
{
	int	bk, i;
	int wp = 0;
	int fe = 0;
	
	//	領域消去実行
	for(i = 0, bk = BLOCK_DB0; bk <= BLOCK_DB2; bk++, i++)
	{
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[bk], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_NOT_BLANK)
		{	//	書き込み有り
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashErase(bk) == FLASH_SUCCESS)
			{
				while(R_FlashGetStatus() != FLASH_SUCCESS);
				if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[bk], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
				{	//	消去完了
					fe |= (1 << i);
				}
			}
		}
		else
		{
			fe |= (1 << i);
		}
	}

	//	MAP書き込み
	if((fe & 1) != 0)
	{	//	消去確認済み
		if(R_FlashWrite(ADDRESS_OF_ROOTMAP, (int)&rout_map, sizeof(ECU_ROUT_MAP)) == FLASH_SUCCESS)
		{	//	書き込み完了
			wp |= 1;
		}
	}
	//	CONF書き込み
	if((fe & 2) != 0)
	{	//	消去確認済み
		if(R_FlashWrite(ADDRESS_OF_CYCEVE, (int)&conf_ecu.LIST[0], (sizeof(ECU_CYC_EVE) * MESSAGE_MAX)) == FLASH_SUCCESS)
		{	//	書き込み完了
			wp |= 2;
		}
	}
	//	I/O書き込み
	if((fe & 4) != 0)
	{	//	消去確認済み
		if(R_FlashWrite(ADDRESS_OF_IOLIST, (int)&ext_list[0], sizeof(ext_list)) == FLASH_SUCCESS)
		{	//	書き込み完了
			wp |= 4;
		}
	}
	while(R_FlashGetStatus() != FLASH_SUCCESS);
	return wp;
}

//----------------------------------------------------------------------------------------
//	ECU運用データの一括消去
//----------------------------------------------------------------------------------------
int ecu_data_erase(void)
{
	int	bk, i;
	int fe = 0;
	
	for(i = 0, bk = BLOCK_DB0; bk <= BLOCK_DB15; bk++, i++)
	{
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[bk], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_NOT_BLANK)
		{	//	書き込み有り
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashErase(bk) == FLASH_SUCCESS)
			{
				while(R_FlashGetStatus() != FLASH_SUCCESS);
				if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[bk], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
				{	//	消去完了
					fe |= (1 << i);
				}
			}
		}
	}
	return fe;
}

//----------------------------------------------------------------------------------------
//	ECU運用データの書き込み状態確認
//----------------------------------------------------------------------------------------
int ecu_data_check(void)
{
	int	bk, i;
	int wf = 0;
	
	for(i = 0, bk = BLOCK_DB0; bk <= BLOCK_DB15; bk++, i++)
	{
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[bk], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_NOT_BLANK)
		{	//	書き込み有り
			wf |= (1 << i);
		}
	}
	return wf;
}

/*
//----------------------------------------------------------------------------------------
//	Reprogram 処理
//----------------------------------------------------------------------------------------
int repro_job(int ch, int id, void *frame)
{
	int	ret;
	unsigned long	adr;
	int	sw = SELECT_ECU_UNIT + 0x7C0;

	if(id == 0x7BF)
	{	//	リプロ要求
		unsigned char	buf[8];
		repro_mode = 0;
		memcpy(buf, (REPRO_QUERY_FRAME *)frame, sizeof(REPRO_QUERY_FRAME));
		//	フォーマット確認	43 41 4E 3X 00 00 00 00
		if(buf[0] == 0x43 && buf[1] == 0x41 && buf[2] == 0x4E && buf[3] == (SELECT_ECU_UNIT | 0x30))
		{	//	リプロコード確認
			repro_mode = 1;
			//	応答返信
			sw += 8;
			memcpy(&can_buf.ID[sw], buf, 8);
			if(ch >= 0) add_mbox_frame(ch, 8, CAN_DATA_FRAME, sw);	//	送信待ちバッファ積み上げ
			return sw;	//	転送無し
		}
	}
	if(id != sw || repro_mode == 0) return 0;	//	ID対象外

	memcpy(&repro_req, (REPRO_QUERY_FRAME *)frame, sizeof(REPRO_QUERY_FRAME));
	adr = ((int)repro_req.PACK.ADH << 8) | ((int)repro_req.PACK.ADL);
	repro_ret.PACK.LEN = 0;
	repro_ret.PACK.CMD = repro_req.PACK.CMD + 0x40;	//	応答コード
	repro_ret.PACK.ADH = repro_req.PACK.ADH;
	repro_ret.PACK.ADL = repro_req.PACK.ADL;
	switch(repro_req.PACK.CMD)
	{
	case ALL_CONFIG_SAVE:	//	データフラッシュへ一括保存
		repro_ret.PACK.LEN += 1;
		if(ecu_data_write() == 7) break;
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	//------------------------------------------------
	//	ルーティングマップ		adr=000h～7FFh
	//------------------------------------------------
	case GET_ROUTING_MAP:	//	ルーティングマップ取得
		repro_ret.PACK.LEN += 7;
		memcpy(&repro_ret.PACK.DAT[0], &rout_map.ID[adr].BYTE, 4);
		break;
	case SET_ROUTING_MAP:	//	ルーティングマップ変更
		repro_ret.PACK.LEN += 3;
		memcpy(&rout_map.ID[adr].BYTE, &repro_ret.PACK.DAT[0], (repro_req.PACK.LEN - 3));
		break;
	case SAV_ROUTING_MAP:	//	ルーティングマップROM保存
		repro_ret.PACK.LEN += 1;
		//	消去済み確認
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB0], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
		{	//	消去
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashWrite(ADDRESS_OF_ROOTMAP, (int)&rout_map, sizeof(ECU_ROUT_MAP)) == FLASH_SUCCESS)
			{	//	書き込み完了
				break;
			}
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	case ERA_ROUTING_MAP:	//	ルーティングマップROM消去
		repro_ret.PACK.LEN += 1;
		//	消去済み確認
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB0], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
		{	//	消去
			break;
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		if(R_FlashErase(BLOCK_DB0) == FLASH_SUCCESS)
		{
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB0], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
			{	//	消去
				break;
			}
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	//------------------------------------------------
	//	周期送信・イベントリスト	adr=0～255
	//------------------------------------------------
	case DEL_CYCEVE_LIST:	//	稼働中の周期送信・イベントリストから指定IDを削除
		repro_ret.PACK.LEN += 3;
		delete_cyceve_list(adr);	//	管理中のIDを削除
		delete_waiting_list(adr);	//	時間待ちIDを削除
		break;
	case NEW_CYCEVE_LIST:	//	稼働中の周期送信・イベントリストへ新規IDを追加
		repro_ret.PACK.LEN += 3;
		insert_cyceve_list(adr);	//	リスト連結
		can_id_event(adr, 0);		//	イベント登録
		break;
	case GET_CYCEVE_LIST:	//	周期送信・イベントIDリスト取得
		repro_ret.PACK.LEN += 7;
		memcpy(&repro_ret.PACK.DAT[0], &((unsigned char *)&conf_ecu.LIST[0])[adr], 4);
		break;
	case SET_CYCEVE_LIST:	//	周期送信・イベントIDリスト変更
		repro_ret.PACK.LEN += 3;
		memcpy(&((unsigned char *)&conf_ecu.LIST[0])[adr], &repro_ret.PACK.DAT[0], (repro_req.PACK.LEN - 3));
		break;
	case GET_CYCEVE_LIST1:	//	周期送信・イベントIDリスト取得
		repro_ret.PACK.LEN += 7;
		memcpy(&repro_ret.PACK.DAT[0], &conf_ecu.LIST[adr].ID, 4);
		break;
	case GET_CYCEVE_LIST2:	//	周期送信・イベントTimerリスト取得
		repro_ret.PACK.LEN += 7;
		memcpy(&repro_ret.PACK.DAT[0], &conf_ecu.LIST[adr].TIMER, 4);
		break;
	case SET_CYCEVE_LIST1:	//	周期送信・イベントIDリスト変更
		repro_ret.PACK.LEN += 3;
		memcpy(&conf_ecu.LIST[adr].ID, &repro_ret.PACK.DAT[0], (repro_req.PACK.LEN - 3));
		break;
	case SET_CYCEVE_LIST2:	//	周期送信・イベントリスト変更
		repro_ret.PACK.LEN += 3;
		memcpy(&conf_ecu.LIST[adr].TIMER, &repro_ret.PACK.DAT[0], (repro_req.PACK.LEN - 3));
		break;
	case ERA_CYCEVE_LIST:	//	周期送信・イベントリストROM消去
		repro_ret.PACK.LEN += 1;
		//	消去済み確認
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB1], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
		{	//	消去
			break;
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		if(R_FlashErase(BLOCK_DB1) == FLASH_SUCCESS)
		{
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB1], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
			{	//	消去
				break;
			}
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	case SAV_CYCEVE_LIST:	//	周期送信・イベントリストROM保存
		repro_ret.PACK.LEN += 1;
		//	消去済み確認
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB1], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
		{	//	消去
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashWrite(ADDRESS_OF_CYCEVE, (int)&conf_ecu.LIST[0], (sizeof(ECU_CYC_EVE) * MESSAGE_MAX)) == FLASH_SUCCESS)
			{	//	書き込み完了
				break;
			}
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	//------------------------------------------------
	//	外部入出力リスト
	//------------------------------------------------
	case GET_EXT_IO_LIST:	//	外部入出力リスト取得
		repro_ret.PACK.LEN += 7;
		memcpy(&repro_ret.PACK.DAT[0], &((unsigned char *)&ext_list[0])[adr], 4);
		break;
	case SET_EXT_IO_LIST:	//	外部入出力リスト変更
		repro_ret.PACK.LEN += 3;
		memcpy(&((unsigned char *)&ext_list[0])[adr], &repro_ret.PACK.DAT[0], (repro_req.PACK.LEN - 3));
		break;
	case ERA_EXT_IO_LIST:	//	外部入出力リストROM消去
		repro_ret.PACK.LEN += 1;
		//	消去済み確認
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB2], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
		{	//	消去
			break;
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		if(R_FlashErase(BLOCK_DB2) == FLASH_SUCCESS)
		{
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB2], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
			{	//	消去
				break;
			}
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	case SAV_EXT_IO_LIST:	//	外部入出力リストROM保存
		repro_ret.PACK.LEN += 1;
		//	消去済み確認
		if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[BLOCK_DB2], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_BLANK)
		{	//	消去
			while(R_FlashGetStatus() != FLASH_SUCCESS);
			if(R_FlashWrite(ADDRESS_OF_IOLIST, (int)&ext_list[0], sizeof(ext_list)) == FLASH_SUCCESS)
			{	//	書き込み完了
				break;
			}
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	//------------------------------------------------
	//	ファームウェア
	//------------------------------------------------
	case READ_FIRMWARE:	//	ファームウェア取得
		//	消去確認
		adr = 0xFFF00000 + (adr << 2);
		repro_ret.PACK.LEN += 7;
		memcpy(&repro_ret.PACK.DAT[0], ((unsigned char *)adr), 4);
		break;
	case UPDATE_FIRMWARE:	//	ファームウェアROM保存
		repro_ret.PACK.LEN += 1;
		if(bootcopy() == 1) break;	//	成功
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	case REMOVE_FIRMWARE:	//	ファームウェアROM消去
		repro_ret.PACK.LEN += 1;
		if(bootclear() == 1)
		{	//	消去成功
			break;
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	case COPY_FIRMWARE:	//	ファームウェアROM書き込み用バッファへの転送
		repro_ret.PACK.LEN += 3;
		memcpy(&fw_image[adr], &repro_ret.PACK.DAT[0], 4);
		break;
	case WRITE_FIRMWARE:	//	ファームウェアROM書き込み（消去を先に実行する）
		repro_ret.PACK.LEN += 3;
		adr = 0xFFF00000 + (adr << 2);
		if(R_FlashWrite(adr, (unsigned long)&fw_image[0], 128) == FLASH_SUCCESS)
		{	//	保存成功
			break;
		}
		while(R_FlashGetStatus() != FLASH_SUCCESS);
		repro_ret.PACK.CMD |= 0x80;	//	エラー
		break;
	}
	//	返信処理
	if(repro_ret.PACK.LEN > 0)
	{	//	応答返信
		sw += 8;
		memcpy(&can_buf.ID[sw], repro_ret.BYTE, 8);
		if(ch >= 0) add_mbox_frame(ch, 8, CAN_DATA_FRAME, sw);	//	送信待ちバッファ積み上げ
		return sw;	//	転送無し
	}
	return 0;
}
*/



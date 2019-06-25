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
// $RCSfile: ecu.h,v $
// $Revision: 1.00 $
// $Date: 2016/12/15 14:14:48 $
// 
// Copyright (c) 2016 LandF Corporation.
//
// History:
//


#ifndef		__CAN_ECU_CGW__
#define		__CAN_ECU_CGW__

//	LFY_RX63N基板を使用したCANポート1の試験用に生成するなら下記を有効にする
//#define		__LFY_RX63N__

//	LFY-RX63NのUSBコンソール有無
#ifdef		__LFY_RX63N__
//#define		__USE_LFY_USB__
#endif

//---------------------------------------------------------------------------------------
//	ECUユニットコードの取得
//---------------------------------------------------------------------------------------
#ifdef		__LFY_RX63N__
//	LFY試験環境
#define		SELECT_ECU_UNIT			0	/*	試験機能選定	0:パワトレ / 1:シャシー / 2:ボディー	*/
#define		ECU_UNIT_POWERTRAIN		0
#define		ECU_UNIT_CHASSIS		1
#define		ECU_UNIT_BODY			2
#define		ECU_UNIT_CGW			7
#else
//	CAN2ECU環境
#define		DPSW_ROM_BOOT			(PORTF.PIDR.BIT.B5)					/*	DipSwitch S1-8		*/
#define		DPSW_BOOTCOPY			(PORTJ.PIDR.BIT.B5)					/*	DipSwitch S1-7		*/
#define		SELECT_ECU_UNIT			((~PORT5.PIDR.BYTE >> 1) & 0x07)	/*	DipSwitch S1-6,5,4	*/
#define		ECU_UNIT_POWERTRAIN		0
#define		ECU_UNIT_CHASSIS		1
#define		ECU_UNIT_BODY			2
#define		ECU_UNIT_CGW			7
#endif

//---------------------------------------------------------------------------------------
//	CANメッセージバッファ定義
//---------------------------------------------------------------------------------------
#define		CAN_ID_MAX			0x800
#define		CAN_ID_MASK			0x7FF
#define		MESSAGE_MAX			0x100
#define		MESSAGE_MSK			0x0FF
#define		MESSAGE_END			0x1FF
#define		MESSAGE_BOXS		3

//	ポート数設定
#define		CAN_CH_MAX			4	/*	CAN2ECU基板はCAN-4ch	*/
#define		CAN_TEST_LFY_CH		1	/*	LFY-RX63NはCAN1のみ有効	*/

#define		DLC_MASK			0x0F

#define		CAN_DATA_FRAME		0
#define		CAN_REMOTE_FRAME	1

//	MBOX選択指定
#define		MBOX_POINT_1		0x200
#define		MBOX_POINT_2		0x400

//	外部入出力チェックリスト最大数
#define		ECU_EXT_MAX			64
//	外部I/O管理数
#define		EX_IO_MAX			64

//	送信待ちリストをID優先度で並べ替え
#define		SORT_TXWAITLIST_ENABLE

//	ルーティングマップ定義構造体(E2DATAに保存)
typedef	struct	__routing_map__
{
	union	{
		unsigned char	BYTE;
		struct {
			unsigned char	RE3	:	1;	//	CAN-Port3 受信	1=許可/0=禁止
			unsigned char	RE2	:	1;	//	CAN-Port2 受信	1=許可/0=禁止
			unsigned char	RE1	:	1;	//	CAN-Port1 受信	1=許可/0=禁止
			unsigned char	RE0	:	1;	//	CAN-Port0 受信	1=許可/0=禁止
			unsigned char	TE3	:	1;	//	CAN-Port3 送信	1=許可/0=禁止
			unsigned char	TE2	:	1;	//	CAN-Port2 送信	1=許可/0=禁止
			unsigned char	TE1	:	1;	//	CAN-Port1 送信	1=許可/0=禁止
			unsigned char	TE0	:	1;	//	CAN-Port0 送信	1=許可/0=禁止
		}	BIT;
	}	ID[CAN_ID_MAX];
}	ECU_ROUT_MAP;

//	CGWのポートルーティングマップ制御ビット定義
#define		EX_R_BIT	0x80	/*	外部CAN	受信	*/
#define		CS_R_BIT	0x40	/*	シャシー受信	*/
#define		BD_R_BIT	0x20	/*	ボディー受信	*/
#define		PT_R_BIT	0x10	/*	パワトレ受信	*/
#define		EX_W_BIT	0x08	/*	外部CAN	送信	*/
#define		CS_W_BIT	0x04	/*	シャシー送信	*/
#define		BD_W_BIT	0x02	/*	ボディー送信	*/
#define		PT_W_BIT	0x01	/*	パワトレ送信	*/

#define		CS_TO_PT	(CS_R_BIT|PT_W_BIT)						/*	シャシーからパワトレ	*/
#define		CS_TO_BD	(CS_R_BIT|BD_W_BIT)						/*	シャシーからボディー	*/
#define		PT_TO_CS	(PT_R_BIT|CS_W_BIT)						/*	パワトレからシャシー	*/
#define		BD_TO_CS	(BD_R_BIT|CS_W_BIT)						/*	ボディーからシャシー	*/
#define		PT_TO_BD	(PT_R_BIT|BD_W_BIT)						/*	パワトレからボディー	*/
#define		BD_TO_PT	(BD_R_BIT|PT_W_BIT)						/*	ボディーからパワトレ	*/
#define		CS_TO_AL	(CS_R_BIT|PT_W_BIT|BD_W_BIT)			/*	シャシーから全て		*/
#define		PT_TO_AL	(PT_R_BIT|CS_W_BIT|BD_W_BIT)			/*	パワトレから全て		*/
#define		BD_TO_AL	(BD_R_BIT|PT_W_BIT|CS_W_BIT)			/*	ボディーから全て		*/
#define		EX_TO_AL	(EX_R_BIT|PT_W_BIT|CS_W_BIT|BD_W_BIT)	/*	外部CAN	から全て		*/
#define		EX_TO_PT	(EX_R_BIT|PT_W_BIT)						/*	外部CAN	からパワトレ	*/
#define		EX_TO_CS	(EX_R_BIT|CS_W_BIT)						/*	外部CAN	からボディー	*/
#define		EX_TO_BD	(EX_R_BIT|BD_W_BIT)						/*	外部CAN	からシャシー	*/
#define		PT_TO_EX	(PT_R_BIT|EX_W_BIT)						/*	パワトレから外部CAN		*/
#define		BD_TO_EX	(BD_R_BIT|EX_W_BIT)						/*	ボディーから外部CAN		*/
#define		CS_TO_EX	(CS_R_BIT|EX_W_BIT)						/*	シャシーから外部CAN		*/
#define		AL_TO_EX	(PT_R_BIT|BD_R_BIT|CS_R_BIT|EX_W_BIT)	/*	全てから外部CAN			*/
#define		AL_TO_AL	(EX_R_BIT|PT_R_BIT|BD_R_BIT|CS_R_BIT|EX_W_BIT|PT_W_BIT|BD_W_BIT|CS_W_BIT)	/*	全て転送	*/

//	CAN-ID共用体
typedef	union	__can_id_form__
{
	unsigned long	LONG;
	struct	{
		unsigned short	H;
		unsigned short	L;
	}	WORD;
	struct {
		unsigned char	HH;
		unsigned char	HL;
		unsigned char	LH;
		unsigned char	LL;
	}	BYTE;
	struct {
		unsigned long	IDE	:	1;	//	[0]
		unsigned long	RTR	:	1;	//	リモート送信要求ビット(0=データフレーム/1=リモートフレーム)
		unsigned long		:	1;	//	[0]
		unsigned long	SID	:	11;	//	標準IDビット
		unsigned long		:	2;	//	[0]
		unsigned long	DLC	:	4;	//	フレームデータ長指定(0～8)
		unsigned long	ENB	:	1;	//	処理イネーブルフラグ(0=無効/1=有効)
		unsigned long	REP	:	1;	//	リピートフラグ(0=イベント/1=周期)
		unsigned long		:	1;	//	[0]
		unsigned long	NXT	:	9;	//	次の周期・イベント番号(実行時のみ有効：0～255／継続無し：256～511)
	}	BIT;
}	CAN_ID_FORM;

//	周期・イベント定義構造体(E2DATAに保存)
typedef	struct	__cycle_event_str__
{
	CAN_ID_FORM	ID;		//	対象ID番号
	union	{
		long	LONG;
		struct	{
			short	TIME;	//	周期時間又はイベント遅延時間(ms)
			short	CNT;	//	カウンタ又はイベント遅延増加時間(ms)
		}	WORD;
	}	TIMER;
}	ECU_CYC_EVE;

//	周期・イベント登録情報
typedef	struct	__cycle_event_info__
{
	int				WP;					//	書き込みポインタ
	int				TOP;				//	先頭ポインタ
	int				CNT;				//	登録数
	ECU_CYC_EVE		LIST[MESSAGE_MAX];	//	周期・イベント情報
}	CYCLE_EVENTS;

//	メッセージボックス使用ID範囲設定
typedef	struct	__mbox_select_id__
{
	struct	{
		unsigned short	MB1;	//	メッセージボックス0に適用するID範囲 0～MB1
		unsigned short	MB2;	//	メッセージボックス0に適用するID範囲 MB1～MB2
	}	CH[CAN_CH_MAX];
}	MBOX_SELECT_ID;

//	送信待ちフレーム管理構造体
typedef	struct	__send_wait_frame__
{
	CAN_ID_FORM		ID;		//	メッセージID番号(0～2047)
	union	{
		unsigned long	LONG[2];
		unsigned short	WORD[4];
		unsigned char	BYTE[8];
	}	FD;					//	フレームデータ
}	SEND_WAIT_FLAME;

//	送信待ちバッファ定義構造体
typedef	struct	__send_wait_buffer__
{
	struct	{
		int				WP;					//	登録ポインタ(0～MESSAGE_MAX-1)
		int				TOP;				//	先頭ポインタ(0～MESSAGE_MAX-1)
		int				CNT;				//	送信待ちメッセージ数(0～MESSAGE_MAX)
		SEND_WAIT_FLAME	MSG[MESSAGE_MAX];	//	保持メッセージバッファ(最大256個)
	}	BOX[MESSAGE_BOXS];
}	SEND_WAIT_BUF;

//	CANフレームデータ共用体
typedef	union	__can_frame_data__
{
	unsigned long	LONG[2];
	unsigned short	WORD[4];
	unsigned char	BYTE[8];
}	CAN_DATA_BYTE;

//	フレームデータバッファ定義構造体
typedef	struct	__can_frame_buffer__
{
	CAN_DATA_BYTE	ID[CAN_ID_MAX];
}	CAN_FRAME_BUF;

//	CANバスメッセージボックス定義構造体
typedef	struct	__can_message_box__
{
	union {
		unsigned long LONG;
		struct {
			unsigned short H;
			unsigned short L;
		} WORD;
		struct {
			unsigned char HH;
			unsigned char HL;
			unsigned char LH;
			unsigned char LL;
		} BYTE;
		struct {
			unsigned long IDE	:	1;		//	[0]
			unsigned long RTR	:	1;		//	リモート送信要求ビット(0=データフレーム/1=リモートフレーム)
			unsigned long 		:	1;		//	[0]
			unsigned long SID	:	11;		//	標準IDビット
			unsigned long EID	:	18;		//	[0]
		} BIT;
	} ID;
	unsigned short DLC;						//	データ長(0～8)
	unsigned char  DATA[8];					//	データ本体(8byte)
	unsigned short TS;						//	タイムスタンプ(読み出しのみ)
}	CAN_MBOX;

//	外部I/O定義構造体	4+4+24=32byte
typedef struct	__externul_io_str__
{
	int				SID;			//	ID番号				-1:無効 / 000～7FF:有効
	union {
		unsigned long	LONG;
		struct	{
			unsigned long			:	1;	//	予備				0
			unsigned long	MODE	:	3;	//	I/O処理モード設定	0:無効 / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit値 / 6:3bit値 / 7:4bit値
			unsigned long	NEG		:	1;	//	データ反転指定		ネガティブ条件
			unsigned long	SIZE	:	3;	//	アクセスサイズ		0:BIT / 1～7:nBYTE
			unsigned long	BPOS	:	4;	//	バイト位置			0～7
			unsigned long	DLC		:	4;	//	データバイト長		0～8
			unsigned long	NOM		:	8;	//	ポート番号			0～33
			unsigned long	MSK		:	8;	//	マスク				00～FF
		}	BIT;
	}	PORT;
	unsigned char	FRMCNT;			//	フレームカウンタ
	unsigned char	PAT[23];		//	パターンデータ	size>0の時、複数ビット参照によるコピー元となる
}	EXTERNUL_IO;

//	CAN-ID -> EX-I/O-ID 変換テーブル
extern	unsigned char	can_to_exio[CAN_ID_MAX];	//	CAN-IDの示すデータが外部I/O管理番号となる。00～3F=適用、FF=対応無し

//	ポインタマルチアクセス共用体定義
typedef	union __pointer_multi_access__
{
	unsigned long	LONG;
	void			*VP;
	signed char		*SB;
	unsigned char	*UB;
	signed short	*SW;
	unsigned short	*UW;
	signed int		*SI;
	unsigned int	*UI;
	signed long		*SL;
	unsigned long	*UL;
	ECU_CYC_EVE		*CYE;
	ECU_ROUT_MAP	*MAP;
	CYCLE_EVENTS	*CONF;
	CYCLE_EVENTS	*WAIT;
	SEND_WAIT_BUF	*SMSG;
	CAN_FRAME_BUF	*CAN;
	MBOX_SELECT_ID	*MBOX;
	EXTERNUL_IO		*EXL;
}	POINTER_MULTI_ACCESS;

//	E2DATAフラッシュ保存変数
//	ルーティングマップ
extern	ECU_ROUT_MAP	rout_map;	//	マップ変数
//	定義保持バッファ
extern	CYCLE_EVENTS	conf_ecu;	//	周期・イベント・リモート管理定義変数
//	ECU入出力チェックリスト	32*64=2048byte
extern	EXTERNUL_IO		ext_list[ECU_EXT_MAX];
extern	int				ext_list_count;		//	外部入出力処理の登録数

//	RAM上の変数
//	タイムアップ待ちバッファ
extern	CYCLE_EVENTS	wait_tup;	//	周期・イベント待ち変数
//	メッセージボックス毎の送信待ちバッファ
extern	SEND_WAIT_BUF	send_msg[CAN_CH_MAX];
//	CANデータバッファ変数
extern	CAN_FRAME_BUF	can_buf;
extern	CAN_FRAME_BUF	can_random_mask;

/*
//	CANデータ変化フラグ
extern	unsigned char	can_chainge[CAN_ID_MAX];
extern	int				can_chainge_cnt;
*/
//	メッセージボックス範囲
extern	MBOX_SELECT_ID	mbox_sel;
//	メッセージボックスCANフレーム送信バッファ積み上げ処理
extern	void add_mbox_frame(int ch, int dlc, int rtr, int id);

//	リプロモードフラグ
extern	int	repro_mode;			//	0=通常モード / 1=リプロモード

//	受信サブバッファ
#define		RX_MB_BUF_MAX	32
typedef	struct	__rx_mb_buffer__
{
	CAN_MBOX	MB[RX_MB_BUF_MAX];
	int			WP;
	int			RP;
}	RX_MB_BUF;

extern	RX_MB_BUF	rxmb_buf[3];	//	受信サブバッファ

//extern	int				ds_conect_active;	//	ドライビングシミュレータ接続フラグ

//	LEDモニタリングID設定
extern	int				led_monit_id;				//	モニターID
extern	unsigned char	led_monit_ch;				//	モニターCHビットセット
extern	int				led_monit_first;			//	最短時間
extern	int				led_monit_slow;				//	最長時間
extern	int				led_monit_time;				//	平均時間
extern	int				led_monit_count;			//	平均化回数
extern	int				led_monit_sample;			//	サンプル回数

//	E2DATAフラッシュ定義
#define		ADDRESS_OF_ROOTMAP		0x00100000		/*	ルートマップ	2048byte	0x00100000～0x001007FF	*/
#define		ADDRESS_OF_CYCEVE		0x00100800		/*	周期・イベント	2048byte	0x00100800～0x00100FFF	*/
#define		ADDRESS_OF_IOLIST		0x00101000		/*	入出力チェック	 272byte	0x00101000～0x001017FF	*/

//---------------------------------------------------------------------------------------
//  機能   : 自局が管理するIDかをチェックする
//  引数   : int id		検索ID番号
//  説明   : 定義バッファから指定ID番号を検索する
//  戻り値 : int		0以上：定義バッファ番号 / -1：対象外ID
//---------------------------------------------------------------------------------------
extern	int search_target_id(int id);
//---------------------------------------------------------------------------------------
//  機能   : 周期イベントデータ追加
//  引数   : int rtr			リモートフレーム指定	0/1
//           int id				CANメッセージ番号		0～2047
//           int dlc			データバイト長			0～8
//           int enb			処理許可フラグ			0/1
//           int rep			周期フレーム指定		0/1
//           int time			周期時間又は遅延時間(ms)0～65535
//           int cnt			遅延増加時間(ms)		0～65535
//  説明   : 周期・イベントの登録
//  戻り値 : バッファの追加位置
//---------------------------------------------------------------------------------------
extern	int	add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt);
//---------------------------------------------------------------------------------------
//  機能   : 周期イベントデータの途中追加
//  引数   : int mi		バッファ番号		0～255
//  説明   : 周期・イベントの登録済み情報を連結する
//  戻り値 : バッファの追加位置
//---------------------------------------------------------------------------------------
extern	void insert_cyceve_list(int mi);
//---------------------------------------------------------------------------------------
//  機能   : 周期イベントデータ削除
//---------------------------------------------------------------------------------------
extern	void delete_cyceve_list(int id);
//---------------------------------------------------------------------------------------
//  機能   : データ更新イベント発生処理
//  引数   : int id		変化が生じたID番号
//         : int tp		追加待ち時間(ms)
//  説明   : CAN以外の外部機器からのデータ更新要求によって呼び出される
//           イベント処理対象のIDはタイムアップ待ち行列に追加する
//  戻り値 : 負数は登録失敗 / 0以上は増加時間(ms)
//---------------------------------------------------------------------------------------
extern	int can_id_event(int id, int tp);
//---------------------------------------------------------------------------------------
//  機能   : 周期イベントタイムアップ待ちデータ削除
//---------------------------------------------------------------------------------------
extern	void delete_waiting_list(int id);
//----------------------------------------------------------------------------------------
//	BootCopy 処理
//----------------------------------------------------------------------------------------
extern	int bootcopy(void);
extern	int bootclear(void);
//----------------------------------------------------------------------------------------
//	ECU運用データの一括保存
//----------------------------------------------------------------------------------------
extern	int ecu_data_write(void);
//----------------------------------------------------------------------------------------
//	ECU運用データの一括消去
//----------------------------------------------------------------------------------------
extern	int ecu_data_erase(void);
//----------------------------------------------------------------------------------------
//	ECU運用データの書き込み状態確認
//----------------------------------------------------------------------------------------
extern	int ecu_data_check(void);

//----------------------------------------------------------------------------------------
//	受送信間時差計測関数
//----------------------------------------------------------------------------------------
extern	void monit_timeover(void);

#endif		/*__CAN_ECU_CGW__*/


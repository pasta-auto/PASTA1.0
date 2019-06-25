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
//  CAN2ECU メイン処理
//
//----------------------------------------------------------------------------------------
//  開発履歴
//
//  2017/12/10  コーディング開始（橘）
//
//----------------------------------------------------------------------------------------
//  T.Tachibana
//  ㈱L&F
//________________________________________________________________________________________
//

#include	<sysio.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	"altypes.h"
#include	"iodefine.h"
#include	"sci.h"
#include	"rtc.h"
#include	"timer.h"
#include	"ecu.h"
#include	"flash_data.h"
#include	"flash_rom.h"
#include	"r_Flash_API_RX600.h"
#include	"usb.h"
#include	"can3_spi2.h"
#include	"uSD_rspi1.h"
#include	"cantp.h"			/*	CAN-TP 定義				*/
#include	"uds.h"				/*	CAN-UDS 定義			*/


//________________________________________________________________________________________
//
//  CAN2ECU Main	変数定義
//________________________________________________________________________________________
//
#define		COMMAND_BUF_MAX		512
typedef	struct	__console_command_buffer__	{
	int		WP;
	char	BUF[COMMAND_BUF_MAX];
}	CONSOLE_CTRL;

CONSOLE_CTRL	sci_console;
#ifdef	SCI2_ACTIVATE
CONSOLE_CTRL	sci2_console;
#endif
#if	defined(USB_ACTIVATE) && defined(__LFY_RX63N__)
CONSOLE_CTRL	usb_console;
#endif
int		retport = 0;
#ifdef	__LFY_RX63N__
int		console_port = 1;
#else
int		console_port = 0;
#endif

const char	def_ecu_corp[16] = "TOYOTA-ITC";
const char	def_ecu_name[16] = "CAN2ECU";
const char	def_ecu_vars[16] = "Ver2.4.1";
const char	def_ecu_date[16] = __DATE__;
const char	def_ecu_time[16] = __TIME__;

//	通信一時保管データバッファ
#define		RAM_BUFFER_MAX		128
unsigned char	comm_ram_buffer[RAM_BUFFER_MAX];

//  未登録割り込み処理
interrupt void Dummy_Interrupt(void)
{
}

//	ECU処理
void ecu_job(void);					//	ECU運転
void ecu_status(char *cmd);			//	パラメータ状態確認
void ecu_get_command(char *cmd);	//	フレームデータ取得
void ecu_set_command(char *cmd);	//	フレームデータ書き換え
void ecu_put_command(char *cmd);	//	フレーム直接送信
void ecu_input_update(char *cmd);	//	通信経由I/O情報更新

//________________________________________________________________________________________
//
//	iwdt_refresh	ウォッチドックのリフレッシュ
//----------------------------------------------------------------------------------------
//	引数
//		無し
//	戻り
//		無し
//________________________________________________________________________________________
//
void iwdt_refresh(void)
{
	unsigned short	cnt = IWDT.IWDTSR.WORD;
	if((cnt & 0xC000) != 0 || cnt == 0) return;	//	リセット発生済みと無効時は無視する
	if(cnt > 0x2FFF) return;					//	3FFF の 75% 以上は範囲外
	//	IWDT リフレッシュ
	IWDT.IWDTRR = 0x00;
	IWDT.IWDTRR = 0xFF;
}

//________________________________________________________________________________________
//
//	wdt_init	ウォッチドックによる再起動を仕掛ける
//----------------------------------------------------------------------------------------
//	引数
//		無し
//	戻り
//		無し
//________________________________________________________________________________________
//
void wdt_init(void)
{
	WDT.WDTCR.BIT.TOPS = 3;		//	タイムアウト期間 0:1024  1:4096  2:8192  3:16384
	WDT.WDTCR.BIT.CKS = 8;		//	クロック分周　1:PCLK/4  4:PCLK/64  16:PCLK/128  6:PCLK/512 7:PCLK/2048  8:PCLK/8192
	WDT.WDTCR.BIT.RPES = 3;		//	ウィンドウ終了位置　0:75% 1:50% 2:75% 3:100%
	WDT.WDTCR.BIT.RPSS = 3;		//	ウィンドウ開始位置　0:75% 1:50% 2:75% 3:100%
	WDT.WDTRR = 0x00;			//	レジスタスタート1
	WDT.WDTRR = 0xff;			//	レジスタスタート2
}

//________________________________________________________________________________________
//
//	hex_to_byte
//----------------------------------------------------------------------------------------
//	引数
//		*p	  HEX文字列
//		*d	  バイト列格納先
//	戻り
//		int  変換バイト数
//________________________________________________________________________________________
//
int hex_to_byte(char *p, unsigned char *d)
{
	int  i = 0;
	char	c;
	
	while(*p != 0 && *(p + 1) != 0 && i < 32)
	{
		c = *p++;
		if((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))
		{
			if(c > '9') c -= 7;
			d[i] = ((int)c << 4) & 0xF0;
			c = *p++;
			if((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))
			{
				if(c > '9') c -= 7;
				d[i] |= c & 0x0F;
				i++;
			}
			else break;
		}
		else break;
	}
	return i;
}

//________________________________________________________________________________________
//
//	byte_to_ulong
//----------------------------------------------------------------------------------------
//	引数
//		unsigned char *data	バイト列(ビッグエンディアン)
//		int index			開始位置
//		int size			参照バイト数
//	戻り
//		unsigned long		取得値
//________________________________________________________________________________________
//
unsigned long   byte_to_ulong(unsigned char *data, int index, int size)
{
	int		  i;
	unsigned long   d = 0;
	unsigned char   *p = data + index;

	for(i = 0; i < size; i++)
	{
		d <<= 8;
		d |= ((unsigned long)p[i] & 0x00FF);
	}
	return d;
}

//________________________________________________________________________________________
//
//  send_var
//----------------------------------------------------------------------------------------
//  機能説明
//	  装置情報をCOMポートへ送信する
//  引数
//	  ch		送信先COMチャンネル番号
//  戻り
//	  無し
//________________________________________________________________________________________
//
void send_var(int ch)
{
	char	s[256];
//	sprintf(s, "%s %s %s %s %s\r", def_ecu_corp, def_ecu_name, def_ecu_vars, def_ecu_date, def_ecu_time);
	sprintf(s, "%s %s %s %s\r", def_ecu_name, def_ecu_vars, def_ecu_date, def_ecu_time);
	sci_puts(ch, s);
}

//________________________________________________________________________________________
//
//  SendPC
//----------------------------------------------------------------------------------------
//  機能説明
//	  受信コマンドを返信
//  引数
//	  *msg		返信電文
//  戻り
//	  無し
//________________________________________________________________________________________
//
void SendPC(char *msg)
{
	switch(retport)
	{
		case 0: //  COM0
#ifdef	SCI0_ACTIVATE
			sci_puts(0, msg);
			break;
#endif
		case 1: //  COM1
#if	defined(SCI1_ACTIVATE) || defined(__LFY_RX63N__)
			sci_puts(1, msg);
			break;
#endif
		case 2: //  COM2
#ifdef	SCI2_ACTIVATE
			sci_puts(2, msg);
			break;
#endif
		case 3: //  COM3
#ifdef	SCI3_ACTIVATE
			sci_puts(3, msg);
			break;
#endif
		case 4: //  USB
#if	defined(USB_ACTIVATE) && defined(__LFY_RX63N__)
			usb_puts(msg);
			usb_flush();
#endif
			break;
	}
}

//________________________________________________________________________________________
//
//	関数名		ログ出力
//----------------------------------------------------------------------------------------
//	書式		void	logging(char *fmt, ...)
//	引数		bp=作業用バッファポインタ
//				fmt=形式付出力
//	説明		printf関数と同様の書式指定可能なログ出力のクライアント。
//	戻り値		無し
//________________________________________________________________________________________
//
void	logging(char *fmt, ...)
{
	va_list		args;
	int			n;
	char		bp[256];

	va_start(args, fmt);
	strcpy(bp, "");
	vsprintf(bp, fmt, args);
	va_end(args);
	SendPC(bp);
}

//________________________________________________________________________________________
//
//  PortInit
//----------------------------------------------------------------------------------------
//  引数
//      無し
//  戻り
//      無し
//________________________________________________________________________________________
//
void    PortInit(void)
{
	unsigned char   d;

	SYSTEM.PRCR.WORD = 0xA502;      //  動作モード、消費電力低減機能関連レジスタプロテクト解除
	SYSTEM.SYSCR0.WORD = 0x5A01;	//	内臓ROM有効、外部バス無効
	while((SYSTEM.SYSCR0.WORD & 3) != 1);

//  MSTP_RIIC0 = 0;                 //  I2C0        ストップ状態解除
//  MSTP_SCI0 = 0;                  //  SCI0        ストップ状態解除
//  MSTP_SCI1 = 0;                  //  SCI1        ストップ状態解除←YellowREM-MONと競合
//  MSTP_TPU0 = 0;                  //  TPU0..5  ストップ状態解除
//  MSTP_MTU = 0;                   //  MTU0..5  ストップ状態解除
//  MSTP_S12AD = 0;                 //  AN000..003  ストップ状態解除

	MPC.PWPR.BIT.B0WI = 0;          //  始めにB0WIに0を書く
	MPC.PWPR.BIT.PFSWE = 1;         //  PFSレジスタへの書き込みを許可

	MPC.PFAOE0.BYTE = 0x00;
	MPC.PFAOE1.BYTE = 0x00;

	memset(&sci_console, 0, sizeof(CONSOLE_CTRL));
#ifdef	SCI2_ACTIVATE
	memset(&sci2_console, 0, sizeof(CONSOLE_CTRL));
#endif
#if	defined(USB_ACTIVATE) && defined(__LFY_RX63N__)
	memset(&usb_console, 0, sizeof(CONSOLE_CTRL));
#endif
//	TransitSuperMode();
	WriteINTB(0);
//	TransitUserMode();
}

//________________________________________________________________________________________
//
//  RTS コマンド処理
//----------------------------------------------------------------------------------------
//  引数
//      char *cmd	コマンド文字列
//  戻り
//      無し
//________________________________________________________________________________________
//
void rtc_command_job(char *cmd)
{
	int			year, mon, day, hour, min, sec;
	time_bcd_t	tm;
	//---------------------------------------------------
	//  コマンド解析
	//---------------------------------------------------
	if( strlen(cmd) == 0 )
	{   // RTCデータリード
		rtc_time_read(&tm);
		logging("RTC=%04X/%02X/%02X %02X:%02X:%02X\r"
			,(int)tm.year, (int)tm.month , (int)tm.day, (int)tm.hour, (int)tm.minute,(int)tm.second );
	}
	else
	if( strlen(cmd) >= 17 )
	{   // RTCデータセット
		if(strlen(cmd) == 17)
		{
			if(6 != sscanf(&cmd[3],"%02X/%02X/%02X %02X:%02X:%02X",&year ,&mon ,&day ,&hour ,&min ,&sec)) return;
		}
		else
		{
			if(6 != sscanf(&cmd[3],"%04X/%02X/%02X %02X:%02X:%02X",&year ,&mon ,&day ,&hour ,&min ,&sec)) return;
		}
		tm.year = 0x00ff & (char)year;
		tm.month = (char)mon;
		tm.day = (char)day;
		tm.hour = (char)hour;
		tm.minute = (char)min;
		tm.second = (char)sec;
		rtc_init(&tm);
		rtc_time_read(&tm);
		logging("RTC=%04X/%02X/%02X %02X:%02X:%02X\r"
			,(int)tm.year, (int)tm.month , (int)tm.day, (int)tm.hour, (int)tm.minute,(int)tm.second );
	}
}

//________________________________________________________________________________________
//
//  SCI/USB コマンド受信処理
//----------------------------------------------------------------------------------------
//  引数
//      無し
//  戻り
//      無し
//________________________________________________________________________________________
//
void command_job(char *cmd)
{
	int		id, dt;
	int		db[8];
	char	*p;
	//---------------------------------------------------
	//  コマンド解析
	//---------------------------------------------------
	switch(*cmd++)
	{
	case '?':	//	デバイスコード
		switch(SELECT_ECU_UNIT)
		{
		case ECU_UNIT_POWERTRAIN:
			logging("ECUPT\r");
			break;
		case ECU_UNIT_CHASSIS:
			logging("ECUIP\r");
			break;
		case ECU_UNIT_BODY:
			logging("ECUBD\r");
			break;
		case ECU_UNIT_CGW:
			logging("ECUGW\r");
			break;
		default:
			logging("ECUX%d\r", (int)(SELECT_ECU_UNIT));
			break;
		}
		break;
	case 'B':	//	ROM操作コマンド
		if(strncmp(cmd, "OOTCOPY", 7) == 0)
		{	//	実行中のプログラムをROMにコピーする
			if(bootcopy() == 1)
			{	//	成功
				logging("BootCopy OK\r");
			}
			else
			{	//	失敗
				logging("BootCopy NG\r");
			}
		}
		else
		if(strncmp(cmd, "OOTCLEAR", 8) == 0)
		{	//	ROM保存領域をクリアする
			if(bootclear() == 1)
			{	//	成功
				logging("BootClear OK\r");
			}
			else
			{	//	失敗
				logging("BootClear NG\r");
			}
		}
		break;
	case 'C':	//	周期・イベントリスト
		switch(*cmd++)
		{
		case 'C':	//	リストを全て削除
			if(cmd[0] == 'A' && cmd[1] == 'L' && cmd[2] == 'L')
			{	//	[CCALL]コマンド
				//	ゼロ初期化
				memset(&wait_tup, 0, sizeof(wait_tup));			//	周期・イベント待ち初期化
				wait_tup.TOP = -1;
				memset(&conf_ecu, 0, sizeof(conf_ecu));			//	周期・イベント・リモート管理定義初期化
				conf_ecu.TOP = -1;
				logging("CCALL OK\r");
			}
			break;
		case 'R':	//	リストからID消去
			while(*cmd == ' ') cmd++;
			if(sscanf(cmd, "%x", &id) == 1)
			{	//	設定値取得	[CR id]
				if(id >= 0 && id < CAN_ID_MAX)
				{
					delete_cyceve_list(id);		//	管理中のIDを削除
					delete_waiting_list(id);	//	時間待ちIDを削除
					logging("CR %03X OK\r", id);
				}
			}
			break;
		case 'A':	//	リストへID追加
			while(*cmd == ' ') cmd++;
			if(sscanf(cmd, "%x %d %d %d %d %d %d",&id, &db[0], &db[1], &db[2], &db[3], &db[4], &db[5]) == 7)
			{	//	設定値取得	[CA id rtr dlc enb rep time cnt]
				if(id >= 0 && id < CAN_ID_MAX)
				{
					dt = add_cyceve_list(db[0], id, db[1], db[2], db[3], db[4], db[5]);
					can_id_event(dt, 0);		//	イベント登録
					logging("CA %03X OK\r", id);
				}
			}
			break;
		}
		break;
	case 'M':
		if(cmd[0] == 'A')
		{	//	MA～
			if(cmd[1] == 'P')
			{	//	MAPコマンド
				cmd += 2;
				while(*cmd == ' ') cmd++;
				if(sscanf(cmd, "%x %x", &id, &dt) == 2)
				{	//	設定値取得
					if(id >= 0 && id < CAN_ID_MAX)
					{
						rout_map.ID[id].BYTE = (unsigned char)dt;
					}
				}
			}
		}
		else
		if(cmd[0] == 'O' && cmd[1] == 'N')
		{	//	MONコマンド(指定IDの受信から送信完了までの時差を得る)
			cmd += 2;
			while(*cmd == ' ') cmd++;
			db[0] = sscanf(cmd, "%x %x %d", &id, &dt, &db[1]);
			if(db[0] >= 2)
			{	//	設定値取得
				cmt1_stop();
				led_monit_id = id;				//	試験ID
				led_monit_ch = dt;				//	試験チャンネル
				led_monit_first = 0x7FFFFFFF;	//	最短時間
				led_monit_slow = 0;				//	最長時間
				led_monit_time = 0;				//	平均時間
				led_monit_count = 0;			//	平均化回数
				led_monit_sample = (db[0] == 3) ? db[1] : 50;
			}
		}
		break;
	case 'E':	//	ECUコマンド
		if(cmd[0] == 'X' && cmd[1] == 'D')
		{	//	EXD	通信経由I/O入力コマンド
			ecu_input_update(&cmd[2]);
		}
		else
		{	//	内部情報返信要求
			ecu_status(cmd);
		}
		break;
	case 'G':	//	ECU指定IDフレーム取得コマンド
		if(cmd[0] == 'E' && cmd[1] == 'T')
		{
			ecu_get_command(&cmd[2]);
		}
		break;
	case 'S':	//	ECU指定IDフレーム設定コマンド
		if(cmd[0] == 'E' && cmd[1] == 'T')
		{
			ecu_set_command(&cmd[2]);
		}
		break;
	case 'P':	//	ECU指定IDフレーム送信コマンド
		if(cmd[0] == 'U' && cmd[1] == 'T')
		{
			ecu_put_command(&cmd[2]);
		}
		break;
	case 'R':	//	R
		if(strncmp(cmd, "EBOOT", 5) == 0)
		{	//	ソフトリセットによる再起動
			logging("ReBoot OK\r");
			wdt_init();
		}
		else
		if(cmd[0] == 'T' && cmd[1] == 'C')
		{	//	[RTC]リアルタイムクロック操作コマンド
			while(*cmd == ' ') cmd++;
			rtc_command_job(cmd);
		}
		else
		if(cmd[0] == 'B' && cmd[1] == 'U')
		{	//	[RBU]RAM積み上げコマンド	RBU pointer length data.. 1回で最大32バイト
			while(*cmd == ' ') cmd++;
			db[0] = 0;	//	引数カウンタ
			db[1] = 0;	//	ポインタ
			db[2] = 0;	//	長さ
			while(*cmd != 0)
			{
				p = cmd;
				while(*cmd != ' ' && *cmd != 0) cmd++;
				switch(db[0])
				{
				case 0:	//	ポインタ設定
					if(sscanf(p, "%d", &db[1]) != 1)
					{
						logging("RBU Error 1\r");
						return;
					}
					if(db[1] < 0 || db[1] >= RAM_BUFFER_MAX)
					{
						logging("RBU Over 1\r");
						return;
					}
					break;
				case 1:	//	長さ設定
					if(sscanf(p, "%d", &db[2]) != 1)
					{
						logging("RBU Error 2\r");
						return;
					}
					if((db[1] + db[2]) >= RAM_BUFFER_MAX)
					{
						logging("RBU Over 2\r");
						return;
					}
					break;
				default:	//	書き込みデータ
					if(sscanf(p, "%x", &db[3]) != 1)
					{
						logging("RBU Error %d\r", db[0]);
						return;
					}
					if((db[0] - 2 + db[1]) < RAM_BUFFER_MAX)
					{	//	バッファ範囲内
						comm_ram_buffer[(db[0] - 2 + db[1])] = (unsigned char)db[3];
					}
					break;
				}
				db[0]++;
				if((db[0] - 2) >= db[2] || (db[0] - 2 + db[1]) >= RAM_BUFFER_MAX)
				{
					break;
				}
			}
			if((db[0] - 2) < db[2])
			{	//	予定データ数不足
				logging("RBU Lost %d\r", db[0]);
				return;
			}
		}
		else
		if(cmd[0] == 'W' && cmd[1] == 'L')
		{	//	[RWL]ROM書き込みコマンド	RWL address length
			while(*cmd == ' ') cmd++;
			if(sscanf(cmd, "%x %d", &db[0], &db[1]) == 2)
			{	//	パラメータ数一致
				_di();
				if(R_FlashWrite((unsigned long)db[0], (unsigned long)&comm_ram_buffer, (unsigned short)db[1]) != FLASH_SUCCESS)
				{	//	書き込み失敗
					_ei();
					logging("RWL Error\r");
					return;
				}
				_ei();
				logging("RWL Success %X\r", db[0]);
			}
		}
		break;
	case 'W':	//	データフラッシュへ保存
		if(cmd[0] == 'D' && cmd[1] == 'F')
		{	//	[WDF]コマンド
			id = ecu_data_write();
			if(id == 7)
			{	//	保存成功
				logging("WDF OK\r");
			}
			else
			{	//	保存失敗
				logging("WDF NG %d\r", id);
			}
		}
		break;
	default:
		logging("Command Error !\r");
		break;
	}
}

//________________________________________________________________________________________
//
//  SCI/USB コマンド受信処理
//----------------------------------------------------------------------------------------
//  引数
//      無し
//  戻り
//      無し
//________________________________________________________________________________________
//
void comm_job(void)
{
	int			i, n;
	int			numBytes;
	int			year, mon, day, hour, min, sec;
	time_bcd_t	tm;
	unsigned short d;
	char		buffer[64];
	char		c;

	//  RS-232C
	numBytes = sci_get_check(console_port);
	if(numBytes != 0)
	{   //  COM1受信処理
		for(i = 0; i < numBytes; i++)
		{
			c = sci_get_char(console_port);
		//	sci_putc(console_port, c); //  エコーバック
			if(c == 0x0D)
			{   //  [CR]
				sci_console.BUF[sci_console.WP] = 0;
				sci_console.WP = 0;
				retport = console_port;
				command_job(sci_console.BUF);
			}
			else
			if(c == 0x08 || c == 0x7F)
			{	//	1文字削除
				if(sci_console.WP > 0) sci_console.WP--;
			}
			else
			{
				if(c >= 0x61 && c <= 0x7a)
				{   // 小文字は大文字に変換する
					sci_console.BUF[sci_console.WP] = c - 0x20;
				}
				else
				{
					sci_console.BUF[sci_console.WP] = c;
				}
				sci_console.WP++;
				if(sci_console.WP >= COMMAND_BUF_MAX) sci_console.WP = 0;
			}
		}
	}
#ifdef	SCI2_ACTIVATE
	//  RS-232C(COM2)
	numBytes = sci_get_check(2);
	if(numBytes != 0)
	{   //  COM1受信処理
		for(i = 0; i < numBytes; i++)
		{
			c = sci_get_char(2);
		//	sci_putc(console_port, c); //  エコーバック
			if(c == 0x0D)
			{   //  [CR]
				sci2_console.BUF[sci2_console.WP] = 0;
				sci2_console.WP = 0;
				retport = 2;
				command_job(sci2_console.BUF);
			}
			else
			if(c == 0x08 || c == 0x7F)
			{	//	1文字削除
				if(sci2_console.WP > 0) sci2_console.WP--;
			}
			else
			{
				if(c >= 0x61 && c <= 0x7a)
				{   // 小文字は大文字に変換する
					sci2_console.BUF[sci2_console.WP] = c - 0x20;
				}
				else
				{
					sci2_console.BUF[sci2_console.WP] = c;
				}
				sci2_console.WP++;
				if(sci2_console.WP >= COMMAND_BUF_MAX) sci2_console.WP = 0;
			}
		}
	}
#endif
#ifdef	__LFY_RX63N__
#ifdef	__USE_LFY_USB__
	//  USB
	for(numBytes = 0; numBytes < sizeof(buffer); numBytes++)
	{
		n = usb_getch();
		if(n < 0) break;
		buffer[numBytes] = (char)n;
	}
	if(numBytes != 0)
	{
		for(i = 0; i < numBytes; i++)
		{
			c = buffer[i];
			if(c == 0x0D)
			{   //  [CR]
				usb_console.BUF[usb_console.WP] = 0;
				usb_console.WP = 0;
				retport = 4;	//	USB
				command_job(usb_console.BUF);
			}
			else
			if(c != 0)
			{
				if(c >= 0x61 && c <= 0x7a)
				{   // 小文字は大文字に変換する
					usb_console.BUF[usb_console.WP] = c - 0x20;
				}
				else
				if(c == 0x08 || c == 0x7F)
				{	//	1文字削除
					if(usb_console.WP > 0) usb_console.WP--;
				}
				else
				{
					usb_console.BUF[usb_console.WP] = c;
				}
				usb_console.WP++;
				if(usb_console.WP >= COMMAND_BUF_MAX) usb_console.WP = 0;
			}
		}
	}
#endif
#endif
}

//________________________________________________________________________________________
//
//	CAN制御部
//----------------------------------------------------------------------------------------
//	機能説明
//		CANポートのECU処理呼び出し
//	引数
//		無し
//	戻り
//		無し
//________________________________________________________________________________________
//
void can_ctrl(void)
{
		//	CAN呼び出し
#ifndef	__LFY_RX63N__
#ifdef	RSPI2_ACTIVATE
#if	(CAN_CH_MAX==4)
	if(can3_job())	//	CAN3はRSPI2経由での制御なので専用処理が必要
#endif
#endif
#endif
	ecu_job();		//	ECU 処理(CAN0～2の制御とECU処理)
}

//________________________________________________________________________________________
//
//	CAN2ECU メイン
//----------------------------------------------------------------------------------------
//  機能説明
//	  初期化＋メインルーチン
//  引数
//	  無し
//  戻り
//	  無し
//________________________________________________________________________________________
//
int main(void)
{
	//	起動初期化
	PortInit();						//  I/Oポート初期化
	cmt0_init();					//  CMT0モジュール設定
	cmt1_init();					//  CMT1モジュール設定
#ifdef	__LFY_RX63N__

#ifdef	SCI1_ACTIVATE
	sci1_init(38400, 8 ,1 ,0);		//  SCI1モジュール設定(RS-232C) <--> FWRITE2(debug-port)
#endif

#else	/*__LFY_RX63N__*/

#ifdef	SCI0_ACTIVATE
	sci0_init(9600, 8 ,2 ,0);		//  SCI0モジュール設定(RS-232C) <--> LF74
#endif
#ifdef	SCI1_ACTIVATE
	sci1_init(9600, 8 ,2 ,0);		//  SCI1モジュール設定(RS-232C) <--> FWRITE2(debug-port)
#endif
#ifdef	SCI2_ACTIVATE
	sci2_init(9600, 8 ,2 ,0);		//  SCI2モジュール設定(RS-232C) <--> EXTERNUL-LF74
#endif
#ifdef	SCI3_ACTIVATE
	sci3_init(9600, 8 ,2 ,0);		//  SCI3モジュール設定(RS-232C) <--> LF62(USBメモリ)
#endif

#endif	/*__LFY_RX63N__*/

	 _ei() ; /* 割り込み許可関数（Yスコープが割り込みを使用する）　*/

#ifdef	__LFY_RX63N__
#ifdef	__USE_LFY_USB__
	usb_init();						//  USB0モジュール設定
#endif
#endif
//  ADC_SD_Init(1);
//  rtc_init();

	//  起動通知
#ifdef	__LFY_RX63N__
	retport = 1;
	console_port = 1;
#ifdef	SCI1_ACTIVATE
	sci_puts(1, " \r\n \r\n \r\n");
	send_var(1);
//	sci_puts(1, VERSION_INFO);
#endif
#else
	retport = 0;		//	COMx
	console_port = 0;
#ifdef	SCI0_ACTIVATE
	sci_puts(0, " \r\n \r\n \r\n");
	send_var(0);
//	sci_puts(0, VERSION_INFO);
#endif
#ifdef	SCI1_ACTIVATE
	sci_puts(1, " \r\n \r\n \r\n");
	send_var(1);
//	sci_puts(1, VERSION_INFO);
#endif
#ifdef	SCI2_ACTIVATE
	sci_puts(2, " \r\n \r\n \r\n");
	send_var(2);
//	sci_puts(2, VERSION_INFO);
#endif
#ifdef	SCI3_ACTIVATE
	sci_puts(3, " \r\n \r\n \r\n");
	send_var(3);
//	usb_puts(VERSION_INFO);
#endif
#endif

	//	ROM操作初期化
	reset_fcu();	//  FCUリセット
	flash_init();	//  FCUイニシャライズ
	can_tp_init();	//	CAN-TP初期化
	can_uds_init();	//	CAN-UDS初期化

	//	起動時特殊処理		※　S1-7,8 共に[ON]状態でYScopeからF/W起動した場合にROM化
	if(DPSW_ROM_BOOT == 0)
	{	//	REM-MON 起動
		if(DPSW_BOOTCOPY == 0)
		{	//	F/W動作開始時ROM化強制
			if((*((unsigned long *)0x00000064)) < 0x00020000)
			{	//	F/Wの起動元がREM-MONならROM化する
				command_job("BOOTCLEAR");	//	ROM消去
				command_job("BOOTCOPY");	//	ROM書き込み
			}
		}
	}

#ifndef	__LFY_RX63N__
	//	MCP2515初期化
	can3_init();
#endif
	
	//	メインルーチン
	for(;;)
	{
		iwdt_refresh();	//	IWDTリフレッシュ
		cmt0_job();		//	タイムアップ呼び出し
		can_ctrl();		//	CAN コントロール
		comm_job();		//	SCI/USB コマンド処理
		if(tp_pack.TXIF)
		{	//	送信完了処理要求有り
			tp_pack.TXIF = 0;	//	要求解除
			can_tp_txendreq();	//	CAN-TP送信完了処理呼び出し
		}
		if(uds_reset_request != 0)
		{	//	ECU再起動
			switch(uds_reset_request)
			{
			case 1:	//	ハードリセット
				wdt_init();
				break;
			case 2:	//	状態クリア
				memset(&can_buf, 0, sizeof(can_buf));
				break;
			case 3:	//	ソフトリセット
				SYSTEM.PRCR.WORD = 0xA502;
				SYSTEM.SWRR = 0xA501;
				break;
			}
			uds_reset_request = 0;
		}
	}
	return 0;
}



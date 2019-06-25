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
//	UDS-CAN プロトコル処理
//
//----------------------------------------------------------------------------------------
//	開発履歴
//
//	2017/08/13	コーディング開始（橘）
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
#include "uds.h"			/*	CAN-UDS 定義			*/
#include "altypes.h"
#include "r_flash_api_rx600_config.h"
#include "mcu_info.h"
#include "r_flash_api_rx600.h"
#include "r_flash_api_rx600_private.h"

/*
	UDS（ユニファイド・ダイアグノスティックス・サービス） 処理の概要
	
	UDSは可変長パケットを送受信する統合診断サービスを提供するプロトコルです。
	本F/Wでは一部のサービスのみ実装しています。
	
	UDSサービス一覧(X=未実装/O=実装※但し最小限)
	+-------------------+-------+-------+-----------------------------------------------+-------+
	| Function group	|Request|Respons|	   Service					 				| 実装	|
	+-------------------+-------+-------+-----------------------------------------------+-------+
	|	Diagnostic and	|	10	|	50	|	Diagnostic Session Control					|	O	|
	+	Communications	+-------+-------+-----------------------------------------------+-------+
	|	Management		|	11	|	51	|	ECU Reset									|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	27	|	67	|	Security Access								|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	28	|	68	|	Communication Control						|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	3E	|	7E	|	Tester Present								|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	83	|	C3	|	Access Timing Parameters					|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	84	|	C4	|	Secured Data Transmission					|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	85	|	C5	|	Control DTC Setting							|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	86	|	C6	|	Response On Event							|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	87	|	C7	|	Link Control								|	X	|
	+-------------------+-------+-------+-----------------------------------------------+-------+
	|	Data			|	22	|	62	|	Read Data By Identifier						|	O	|
	+	Transmission	+-------+-------+-----------------------------------------------+-------+
	|					|	23	|	63	|	Read Memory By Address						|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	24	|	64	|	Read Scaling Data By Identifier				|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	2A	|	6A	|	Read Data By Identifire Periodic			|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	2C	|	6C	|	Dynamically Define Data Identifire			|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	2E	|	6E	|	Write Data By Identifire					|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	3D	|	7D	|	Write Memory By Address						|	O	|
	+-------------------+-------+-------+-----------------------------------------------+-------+
	|	Stored Data		|	14	|	54	|	Clear Diagnostic Information				|	X	|
	+	Transmission	+-------+-------+-----------------------------------------------+-------+
	|					|	19	|	59	|	Read DTC Information						|	X	|
	+-------------------+-------+-------+-----------------------------------------------+-------+
	|	I/O Control		|	2F	|	6F	|	Input Output Control By Identifire			|	X	|
	+-------------------+-------+-------+-----------------------------------------------+-------+
	|	Remote			|		|		|												|		|
	|	Activation of	|	31	|	71	|	Routine Control								|	X	|
	|	Routine			|		|		|												|		|
	+-------------------+-------+-------+-----------------------------------------------+-------+
	|	Upload /		|	34	|	74	|	Request Download							|	O	|
	+	Download		+-------+-------+-----------------------------------------------+-------+
	|					|	35	|	75	|	Request Upload								|	X	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	36	|	76	|	Transfer Data								|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	37	|	77	|	Request Transfer Exit						|	O	|
	+					+-------+-------+-----------------------------------------------+-------+
	|					|	38	|	78	|	Request File Transfer						|	X	|
	+-------------------+-------+-------+-----------------------------------------------+-------+
*/
#if	0
//----------------------------------------------------------------------------------------
//	UDSパケット共用体定義
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//	Diagnostic and Communications Management
//----------------------------------------------------------------------------------------
//	ダイアグセッションコントロールサービス
typedef	struct	__uds_10_diagnostics_sec__
{
	unsigned char	SID;	//	0x10
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char		:	2;	//	未使用
			unsigned char	FC	:	6;	//	セッションコード 01:デフォルト / 02:プログラミング / 03:拡張ダイアグ
		}	BIT;
	}	SUB;
	unsigned char	DATA[UDS_BUFFER_MAX-2];
}	UDS_DIAG_SC;

//	リセットサービス
typedef	struct	__uds_11_ecu_reset__
{
	unsigned char	SID;	//	0x11
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char		:	2;	//	未使用
			unsigned char	FC	:	6;	//	セッションコード 01:ハード / 02:KeyOnOff / 03:ソフト / 04:高速シャットダウン許可 / 05:高速シャットダウン禁止
		}	BIT;
	}	SUB;
	unsigned char	DATA[UDS_BUFFER_MAX-2];
}	UDS_ECU_RES;

//	セキュリティーアクセスサービス
typedef	struct	__uds_27_security_access__
{
	unsigned char	SID;	//	0x27
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char		:	2;	//	未使用
			unsigned char	FC	:	6;	//	セッションコード 01:シード要求 / 02:Key送信 / 03～奇数:シード要求 / 04～偶数:Key送信
		}	BIT;
	}	SUB;
	unsigned char	DATA[UDS_BUFFER_MAX-2];
}	UDS_SEC_ACC;

//	コミュニケーションコントロールサービス
typedef	struct	__uds_28_communication_control__
{
	unsigned char	SID;	//	0x28
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char		:	2;	//	未使用
			unsigned char	FC	:	6;	//	セッションコード 00:送受信許可 / 01:受信許可 / 02:送信許可 / 03:送受信禁止
		}	BIT;
	}	SUB;
	unsigned char	CTYPE;	//	コミュニケーションタイプ 1:アプリケーション / 2:ネットワークマネージメント
	unsigned char	DATA[UDS_BUFFER_MAX-3];
}	UDS_COM_CNT;

//	テスタープレゼントサービス
typedef	struct	__uds_3e_tester_present__
{
	unsigned char	SID;	//	0x3E
	unsigned char	ZERO;	//	ゼロサブファンクション 0:固定
	unsigned char	DATA[UDS_BUFFER_MAX-2];
}	UDS_TES_PRE;

//----------------------------------------------------------------------------------------
//	Data Transmission
//----------------------------------------------------------------------------------------
//	共通識別子データ読み取りサービス
typedef	struct	__uds_22_read_data_ident__
{
	unsigned char	SID;	//	0x22
	unsigned char	RCIDHB;	//	レコードCID上位バイト
	unsigned char	RCIDLB;	//	レコードCID下位バイト
	unsigned char	TXM;	//	送信モード 01,02,03,04,05=single,slow,medium,fast,stop
	unsigned char	MNORTS;	//	応答電文最大長
	unsigned char	DATA[UDS_BUFFER_MAX];
}	UDS_RDB_IDE;
//	指定アドレス読み取りサービス
typedef	struct	__uds_23_read_memory_addr__
{
	unsigned char	SID;	//	0x23
	unsigned char	MEMAHB;		//	メモリアドレス上位
	unsigned char	MEMAMB;		//	メモリアドレス中位
	unsigned char	MEMALB;		//	メモリアドレス下位
	unsigned char	MEMSIZE;	//	メモリサイズ
	unsigned char	TXM;	//	送信モード 01,02,03,04,05=single,slow,medium,fast,stop
	unsigned char	MNORTS;	//	応答電文最大長
	unsigned char	DATA[UDS_BUFFER_MAX];
}	UDS_RDM_ADR;
//	共通識別子データ書き込みサービス
typedef	struct	__uds_2e_write_data_ident__
{
	unsigned char	SID;	//	0x2E
	unsigned char	RECCIDHB;	//	レコードCID上位バイト
	unsigned char	RECCIDLB;	//	レコードCID下位バイト
	unsigned char	DATA[UDS_BUFFER_MAX];	//	レコードデータ
}	UDS_WDB_IDE;
//	指定アドレス書き込みサービス
typedef	struct	__uds_3d_write_memory_addr__
{
	unsigned char	SID;	//	0x3D
	unsigned char	MEMAHB;		//	メモリアドレス上位
	unsigned char	MEMAMB;		//	メモリアドレス中位
	unsigned char	MEMALB;		//	メモリアドレス下位
	unsigned char	MEMSIZE;	//	メモリサイズ
	unsigned char	DATA[UDS_BUFFER_MAX];	//	書き込むデータ
}	UDS_WRM_ADR;

//----------------------------------------------------------------------------------------
//	Upload / Download
//----------------------------------------------------------------------------------------
//	ダウンロード要求（書式はメーカ固有）
typedef	struct	__uds_34_request_download__
{
	unsigned char	SID;	//	0x34
	unsigned char	CMP;	//	0x00	圧縮の方式指定	00＝無し
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	ADDR	:	4;	//	アドレスバイト長
			unsigned char	SIZE	:	4;	//	サイズバイト長
		}	BIT;
	}	CONFIG;
	unsigned char	DATA[8];	//	アドレス情報、サイズ情報
}	UDS_REQ_DL;
//	ダウンロード応答（書式はメーカ固有）
typedef	struct	__uds_74_response_download__
{
	unsigned char	SID;	//	0x74
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SIZE	:	4;	//	2 サイズバイト長
			unsigned char			:	4;	//	0
		}	BIT;
	}	CONFIG;
	unsigned char	TXBHB;		//	転送ブロックサイズ情報上位(0x00～0x0F)
	unsigned char	TXBLB;		//	転送ブロックサイズ情報下位(0x00～0xFF)
	unsigned char	DATA[8];	//
}	UDS_RES_DL;
//	アップロード要求（書式はメーカ固有）
typedef	struct	__uds_35_request_upload__
{
	unsigned char	SID;	//	0x35
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	ADDR	:	4;
			unsigned char	SIZE	:	4;
		}	BIT;
	}	CONFIG;
	unsigned char	DATA[8];
}	UDS_REQ_UL;
//	アップロード応答（書式はメーカ固有）
typedef	struct	__uds_75_response_download__
{
	unsigned char	SID;	//	0x75
	union	{
		unsigned char	BYTE;
		struct	{
			unsigned char	SIZE	:	4;	//	2 サイズバイト長
			unsigned char			:	4;	//	0
		}	BIT;
	}	CONFIG;
	unsigned char	TXBHB;		//	転送ブロックサイズ情報上位(0x00～0x0F)
	unsigned char	TXBLB;		//	転送ブロックサイズ情報下位(0x00～0xFF)
	unsigned char	DATA[8];	//
}	UDS_RES_DL;
//	データ転送
typedef	struct	__uds_36_transfer_data__
{
	unsigned char	SID;	//	0x36
	unsigned char	DATA[UDS_BUFFER_MAX];
}	UDS_TR_DATA;
//	転送中断
typedef	struct	__uds_37_request_transfer_exit__
{
	unsigned char	SID;	//	0x37
	unsigned char	DATA[2];
}	UDS_REQ_EXT;

//----------------------------------------------------------------------------------------
//	コード定義
//----------------------------------------------------------------------------------------
#define		UDS_ERR_SID				0x7F		/*	エラーサービスID								*/
#define		UDS_RES_SID				0x40		/*	正常レスポンス加算ID							*/

//----------------------------------------------------------------------------------------
//	エラーレスポンスコード(SID=0x7Fに添付)
//----------------------------------------------------------------------------------------
#define		UDS_EC_NONE				0x00		/*	エラー無し														*/
#define		UDS_EC_GR				0x10		/*	一般拒否														*/
#define		UDS_EC_SNS				0x11		/*	サポートされていないサービス									*/
#define		UDS_EC_SFNS				0x12		/*	サブ機能はサポートされていません								*/
#define		UDS_EC_IML_IF			0x13		/*	不適切なメッセージの長さまたは無効な形式						*/
#define		UDS_EC_RTL				0x14		/*	応答が長すぎます												*/
#define		UDS_EC_BRR				0x21		/*	ビジーリピートリクエスト										*/
#define		UDS_EC_CNC				0x22		/*	条件が正しくない												*/
#define		UDS_EC_RSE				0x24		/*	リクエストシーケンスエラー										*/
#define		UDS_EC_NRFSNC			0x25		/*	サブネットコンポーネントからの応答がありません					*/
#define		UDS_EC_FPERA			0x26		/*	失敗は、要求されたアクションの実行を防ぎます					*/
#define		UDS_EC_ROOR				0x31		/*	範囲外のリクエスト												*/
#define		UDS_EC_SAD				0x33		/*	セキュリティアクセスが拒否されました							*/
#define		UDS_EC_IK				0x35		/*	無効キー														*/
#define		UDS_EC_ENOA				0x36		/*	試行回数超過													*/
#define		UDS_EC_RTDNE			0x37		/*	所要時間が期限切れになっていない								*/
#define		UDS_EC_UDNA				0x70		/*	アップロード/ダウンロードできません								*/
#define		UDS_EC_TDS				0x71		/*	転送データの一時停止											*/
#define		UDS_EC_GPF				0x72		/*	一般的なプログラミングの失敗									*/
#define		UDS_EC_WBSC				0x73		/*	間違ったブロックシーケンスカウンタ								*/
#define		UDS_EC_RCR				0x78		/*	要求は正しく受信されましたが、応答は保留中です					*/
#define		UDS_EC_SFNSAS			0x7E		/*	サブセッションがアクティブセッションでサポートされていません	*/
#define		UDS_EC_SNSAS			0x7F		/*	アクティブなセッションではサービスがサポートされていません		*/

//	ダウンロード・アップロード管理構造体
#define			UDS_TD_NONE			0			/*	アップロード、ダウンロード要求待ち					*/
#define			UDS_TD_DOWNLOAD		1			/*	ダウンロード中										*/
#define			UDS_TD_UPLOAD		2			/*	アップロード中										*/
typedef	struct	__uds_load_control_str__
{
		int				MODE;		//	転送モード
		unsigned long	ADDR;		//	アドレス
		int				SIZE;		//	サイズ
		int				BLKL;		//	ブロックサイズ
		int				CNT;		//	転送カウンタ
}	UDS_LOAD_STR;

#endif
//----------------------------------------------------------------------------------------
//	変数定義
//----------------------------------------------------------------------------------------
int				uds_diag_session = 0;			//	セッションコントロール
int				uds_p2_can_server_max = 0;		//	P2 Time
int				uds_p2e_can_server_max = 0;		//	P2E Time

int				uds_reset_request = 0;			//	リセット要求

int				uds_security_access = 0;		//	セキュリティーアクセス

UDS_LOAD_STR	uds_load;						//	ダウンロード・アップロード管理

/*
	リプロ規定
	
	ROM領域設定
	E2DataFlash
	0x00100000～0x00107FFF	 8K*4	Parameter		<--- 0x00100000～0x00107FFF	ダウンロード許可 / Erase size 0x00002000
	Program Block
	0xFFE00000～0xFFEFFFFF	64K*16	Data Area		<--- 0xFFE00000～0xFFEFFFFF	ダウンロード許可 / Erase size 0x00010000
	0xFFF00000～0xFFF3FFFF	32K*8	Download F/W	<--- 0xFFF00000～0xFFF3FFFF	ダウンロード許可 / Erase size 0x00008000
	0xFFF40000～0xFFF7FFFF	32K*8	Default F/W		<--- 0xFFF40000～0xFFF7FFFF	ダウンロード禁止 / Erase size 0x00008000
	0xFFF80000～0xFFFF7FFF	16K*29	Bootloader F/W	<--- 0xFFF80000～0xFFFEFFFF	ダウンロード禁止 / Erase size 0x00004000
	0xFFFF8000～0xFFFFFFFF	 4K*8	Configuration	<--- 0xFFFF0000～0xFFFFFFFF	ダウンロード禁止 / Erase size 0x00001000
*/

//----------------------------------------------------------------------------------------
//	CAN-UDS 変数初期化
//----------------------------------------------------------------------------------------
void	can_uds_init(void)
{
	memset(&uds_load, 0, sizeof(UDS_LOAD_STR));
}

//----------------------------------------------------------------------------------------
//	UDS 継続時間タイムアップ
//----------------------------------------------------------------------------------------
void uds_timeup(void)
{
	uds_diag_session = 0;			//	セッションコントロール
	uds_security_access = 0;		//	セキュリティーアクセス
	can_uds_init();
}

//----------------------------------------------------------------------------------------
//	UDS 0x10	Diagnostic Session Control
//----------------------------------------------------------------------------------------
int uds_sid_10(unsigned char *req, int sz, unsigned char *res, int *len)
{
	//	セッションコード 01:デフォルト / 02:プログラミング / 03:拡張ダイアグ
	switch(req[1] & 0x3F)
	{
	case 1:	//	デフォルトセッション
		uds_diag_session = 1;
		break;
	case 2:	//	ECUプログラミングセッション
		uds_diag_session = 2;
		break;
	case 3:	//	ECU拡張診断セッション
		uds_diag_session = 3;
		break;
	default:
		return UDS_EC_SFNS;
	}
	res[0] = req[0] | UDS_RES_SID;
	res[1] = req[1];
//	res[2] = (unsigned char)(uds_p2_can_server_max >> 8);
//	res[3] = (unsigned char)(uds_p2_can_server_max & 0xFF);
//	res[4] = (unsigned char)(uds_p2e_can_server_max >> 8);
//	res[5] = (unsigned char)(uds_p2e_can_server_max & 0xFF);
	*len = 2;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x11	ECU Reset
//----------------------------------------------------------------------------------------
int uds_sid_11(unsigned char *req, int sz, unsigned char *res, int *len)
{
	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	//	セッションコード 01:ハード / 02:KeyOnOff / 03:ソフト / 04:高速シャットダウン許可 / 05:高速シャットダウン禁止
	switch(req[1] & 0x3F)
	{
	case 1:	//	ハードリセット
		uds_reset_request = 1;	//	ハードリセット
		break;
	case 2:	//	Key-On/Off
		uds_reset_request = 2;	//	キーリセット
		break;
	case 3:	//	ソフトリセット
		uds_reset_request = 3;	//	ソフトリセット
		break;
	default:
		return UDS_EC_SFNS;
	}
	res[0] = req[0] | UDS_RES_SID;
	res[1] = req[1];
	*len = 2;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x27	Security Access
//----------------------------------------------------------------------------------------
int uds_sid_27(unsigned char *req, int sz, unsigned char *res, int *len)
{
	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	//	セッションコード 01:シード要求 / 02:Key送信 / 03～奇数:シード要求 / 04～偶数:Key送信
	switch(req[1] & 0x3F)
	{
	case 1:	//	シード要求
		if(uds_security_access == 0)
		{
			res[2] = 0x12;
			res[3] = 0x34;
		}
		else
		{
			res[2] = 0x00;
			res[3] = 0x00;
		}
		*len = 4;
		break;
	case 2:	//	キー設定
		if(req[2] == 0x17 && req[3] == 0xC0 && uds_security_access == 0)
		{	//	0x17C0 解除
			uds_security_access = 1;	//	Unlocked
			*len = 2;
		}
		else
		{	//	キー不一致
			uds_security_access = 0;	//	Locked
			return UDS_EC_IK;
		}
		break;
	case 3:	//	シード要求
		if(uds_security_access == 1)
		{
			res[2] = 0x34;
			res[3] = 0x56;
			*len = 4;
		}
		else
		{
			return UDS_EC_SAD;
		}
		break;
	case 4:	//	キー設定
		if(req[2] == 0x17 && req[3] == 0xC1 && uds_security_access == 1)
		{	//	0x17C1 解除
			uds_security_access = 2;	//	Unlocked
			*len = 2;
		}
		else
		{	//	キー不一致
			return UDS_EC_IK;
		}
		break;
	case 5:	//	シード要求
		if(uds_security_access == 2)
		{
			res[2] = 0x56;
			res[3] = 0x78;
			*len = 4;
		}
		else
		{
			return UDS_EC_SAD;
		}
		break;
	case 6:	//	キー設定
		if(req[2] == 0x17 && req[3] == 0xC2 && uds_security_access == 2)
		{	//	0x17C2 解除
			uds_security_access = 3;	//	Unlocked
			*len = 2;
		}
		else
		{	//	キー不一致
			return UDS_EC_IK;
		}
		break;
	default:
		uds_security_access = 0;	//	Locked
		return UDS_EC_SFNS;
	}
	res[0] = req[0] | UDS_RES_SID;
	res[1] = req[1];
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x3E	Tester Present
//----------------------------------------------------------------------------------------
int uds_sid_3e(unsigned char *req, int sz, unsigned char *res, int *len)
{
	//	テスター接続中の維持
	res[0] = req[0] | UDS_RES_SID;
	*len = 1;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x22	Read Data By Identifier
//----------------------------------------------------------------------------------------
extern	const char	def_ecu_corp[];	//	16] = "TOYOTA";
extern	const char	def_ecu_name[];	//	16] = "CAN2ECU";
extern	const char	def_ecu_vars[];	//	16] = "Ver1.3.0";
extern	const char	def_ecu_date[];	//	16] = __DATE__;
extern	const char	def_ecu_time[];	//	16] = __TIME__;
int uds_sid_22(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int				i, k, b;
	unsigned long	d;

	res[0] = req[0] | UDS_RES_SID;
	res[1] = req[1];	//	CID
	res[2] = req[2];	//	CID
	i = 3;
	switch(req[1])
	{
	default:
		return UDS_EC_SFNS;
	case 0xF1:	//	ECU情報
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	メーカ名
			memcpy(&res[3], &def_ecu_corp[0], 16);
			i += 16;
			break;
		case 0x01:	//	車両コード
			memcpy(&res[3], &def_ecu_name[0], 16);
			i += 16;
			break;
		case 0x02:	//	ECUバージョン
			memcpy(&res[3], &def_ecu_vars[0], 16);
			i += 16;
			break;
		case 0x03:	//	F/W日付
			memcpy(&res[3], &def_ecu_date[0], 16);
			i += 16;
			break;
		case 0x04:	//	F/W時刻
			memcpy(&res[3], &def_ecu_time[0], 16);
			i += 16;
			break;
		}
		break;
	case 0xF2:	//	メモリマップ情報
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	ルーティングマップアドレス
			d = (unsigned long)&rout_map;
			b = 1;
			break;
		case 0x01:	//	周期・イベント・リモート管理定義変数
			d = (unsigned long)&conf_ecu;
			b = sizeof(ECU_CYC_EVE);
			break;
		case 0x02:	//	ECU入出力チェックリスト
			d = (unsigned long)&ext_list;
			b = sizeof(EXTERNUL_IO);
			break;
		case 0x03:	//	CAN-ID -> EX-I/O-ID 変換テーブル
			d = (unsigned long)&can_to_exio;
			b = 1;
			break;
		}
		res[3] = (d >> 16);		//	アドレスH
		res[4] = (d >> 8);		//	アドレスM
		res[5] = (d & 0xFF);	//	アドレスL
		res[6] = (b & 0xFF);	//	アクセスサイズ(byte)
		i += 4;
		break;
	case 0xF3:	//	パラメータアクセス
		k = ((int)req[3] << 8) | ((int)req[4] & 0xFF);
		res[3] = req[3];
		res[4] = req[4];
		i += 2;
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	ルーティングマップ
			if(k >= CAN_ID_MAX) return UDS_EC_ROOR;
			res[5] = rout_map.ID[k].BYTE;
			i++;
			break;
		case 0x01:	//	周期・イベント・リモート管理定義変数
			if(k >= MESSAGE_MAX) return UDS_EC_ROOR;
			memcpy(&res[5], &conf_ecu.LIST[k], sizeof(ECU_CYC_EVE));
			i += sizeof(ECU_CYC_EVE);
			break;
		case 0x02:	//	ECU入出力チェックリスト
			if(k >= ECU_EXT_MAX) return UDS_EC_ROOR;
			memcpy(&res[5], &ext_list[k], sizeof(EXTERNUL_IO));
			i += sizeof(EXTERNUL_IO);
			break;
		case 0x03:	//	CAN-ID -> EX-I/O-ID 変換テーブル
			if(k >= CAN_ID_MAX) return UDS_EC_ROOR;
			res[5] = can_to_exio[k];
			i++;
			break;
		}
		break;
	case 0xF5:	//	データフラッシュ操作
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	書き込み状態の確認
			k = ecu_data_check();
			res[3] = (unsigned char)(k >> 8);
			res[4] = (unsigned char)(k & 0xFF);
			i += 2;
			break;
		}
		break;
	}
	*len = i;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x23	Read Memory By Address
//----------------------------------------------------------------------------------------
int uds_sid_23(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int	z;
	unsigned char	*p;

	p = (unsigned char *)(((int)req[1] << 16) | ((int)req[2] << 8) | ((int)req[3] & 0xFF));
	z = (int)req[4];
	if(z > 64)
	{	//	一括読み出しオーバー
		return UDS_EC_IML_IF;
	}
	res[0] = req[0] | UDS_RES_SID;
	memcpy(&res[1], p, z);
	*len = z + 1;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x2E	Write Data By Identifier
//----------------------------------------------------------------------------------------
int uds_sid_2e(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int				i, k;
	unsigned long	d;

	res[0] = req[0] | UDS_RES_SID;
	res[1] = req[1];	//	CID
	res[2] = req[2];	//	CID
	i = 3;
	switch(req[1])
	{
	default:
		return UDS_EC_SFNS;
/*	case 0xF1:	//	ECU情報
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	メーカ名
			memcpy(&res[3], def_ecu_corp, 16);
			i += 16;
			break;
		case 0x01:	//	車両コード
			memcpy(&res[3], def_ecu_name, 16);
			i += 16;
			break;
		case 0x02:	//	ECUバージョン
			memcpy(&res[3], def_ecu_vars, 16);
			i += 16;
			break;
		case 0x03:	//	F/W日付
			memcpy(&res[3], def_ecu_date, 16);
			i += 16;
			break;
		case 0x04:	//	F/W時刻
			memcpy(&res[3], def_ecu_time, 16);
			i += 16;
			break;
		}
	case 0xF2:	//	メモリマップ情報
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	ルーティングマップアドレス
			d = (unsigned long)&rout_map;
			break;
		case 0x01:	//	周期・イベント・リモート管理定義変数
			d = (unsigned long)&conf_ecu;
			break;
		case 0x02:	//	ECU入出力チェックリスト
			d = (unsigned long)&ext_list;
			break;
		case 0x03:	//	CAN-ID -> EX-I/O-ID 変換テーブル
			d = (unsigned long)&can_to_exio;
			break;
		}
		res[3] = (d >> 24);
		res[4] = (d >> 16);
		res[5] = (d >> 8);
		res[6] = (d & 0xFF);
		i += 4;
		break;*/
	case 0xF3:	//	パラメータアクセス
		if(uds_diag_session < 2)
		{	//	セッション低い
			return UDS_EC_GR;	//	一般拒否
		}
		if(uds_security_access < 1)
		{	//	許可されていない
			return UDS_EC_SAD;	//	セキュリティー拒否
		}
		k = ((int)req[3] << 8) | ((int)req[4] & 0xFF);
		res[3] = req[3];
		res[4] = req[4];
		i += 2;
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x00:	//	ルーティングマップ
			if(k >= CAN_ID_MAX) return UDS_EC_ROOR;
			rout_map.ID[k].BYTE = req[5];
			res[5] = rout_map.ID[k].BYTE;
			i++;
			break;
		case 0x01:	//	周期・イベント・リモート管理定義変数
			if(k >= MESSAGE_MAX) return UDS_EC_ROOR;
			memcpy(&conf_ecu.LIST[k], &req[5], sizeof(ECU_CYC_EVE));
			memcpy(&res[5], &conf_ecu.LIST[k], sizeof(ECU_CYC_EVE));
			i += sizeof(ECU_CYC_EVE);
			break;
		case 0x02:	//	ECU入出力チェックリスト
			if(k >= ECU_EXT_MAX) return UDS_EC_ROOR;
			memcpy(&ext_list[k], &req[5], sizeof(EXTERNUL_IO));
			memcpy(&res[5], &ext_list[k], sizeof(EXTERNUL_IO));
			i += sizeof(EXTERNUL_IO);
			break;
		case 0x03:	//	CAN-ID -> EX-I/O-ID 変換テーブル
			if(k >= CAN_ID_MAX) return UDS_EC_ROOR;
			can_to_exio[k] = req[5];
			res[5] = can_to_exio[k];
			i++;
			break;
		}
		break;
	case 0xF5:	//	データフラッシュ操作
		switch(req[2])
		{
		default:
			return UDS_EC_SNS;
		case 0x01:	//	保存
			k = ecu_data_write();
			res[3] = (unsigned char)(k >> 8);
			res[4] = (unsigned char)(k & 0xFF);
			i += 2;
			break;
		case 0x02:	//	消去
			k = ecu_data_erase();
			res[3] = (unsigned char)(k >> 8);
			res[4] = (unsigned char)(k & 0xFF);
			i += 2;
			break;
		}
		break;
	}
	*len = i;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x3D	Write Memory By Address
//----------------------------------------------------------------------------------------
int uds_sid_3d(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int	i;
	int	z;
	unsigned char	*p, *r;

	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	if(uds_security_access < 1)
	{	//	許可されていない
		return UDS_EC_SAD;	//	セキュリティー拒否
	}
	p = (unsigned char *)(((int)req[1] << 16) | ((int)req[2] << 8) | ((int)req[3] & 0xFF));
	z = (int)req[4];
	if(z > 64)
	{	//	一括書き込みオーバー
		return UDS_EC_IML_IF;
	}
	memcpy(p, &req[5], z);
	memcpy(res, req, 5);
	res[0] |= UDS_RES_SID;
	*len = 5;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x34	Request Download
//----------------------------------------------------------------------------------------
int uds_sid_34(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int	i, j, sb, eb, bs;
	int	siz;
	unsigned long	adr;

	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	if(uds_security_access == 0)
	{	//	ロック状態
		return UDS_EC_UDNA;
	}
	if(uds_load.MODE)
	{	//	実行中エラー
		return UDS_EC_BRR;
	}
	if(req[1] != 0x00)
	{	//	圧縮はサポート無し
		return UDS_EC_SFNS;
	}
	switch(req[2])
	{	//	フォーマット
	case 0x44:
		for(adr = 0, i = 3; i < 7; i++)
		{
			adr <<= 8;
			adr |= (unsigned long)req[i] & 0xFF;
		}
		for(siz = 0, i = 7; i < 11; i++)
		{
			siz <<= 8;
			siz |= (int)req[i] & 0xFF;
		}
		break;
	default:
		return UDS_EC_SFNS;
	}
	if(adr >= 0x00000000ul && adr <= 0x0003FFFF)
	{	//	RAM
		return UDS_EC_CNC;	//	範囲エラー
	//	if(siz > (0x40000 - adr) || siz < 0)
	//	{	//	サイズエラー
	//		return UDS_EC_UDNA;
	//	}
	}
	else
	if(adr >= 0x00100000ul && adr <= 0x00107FFF)
	{	//	E2Data
		if(siz > (0x00108000 - adr) || siz < 0)
		{	//	サイズエラー
			return UDS_EC_UDNA;
		}
		if((adr & 0x00001FFF) == 0)
		{	//	消去
			i = BLOCK_DB0 + (adr >> 13);	//	消去ブロック
			for(j = siz; j > 0; j -= 0x2000, i--)
			{	//	書き込むブロックを先行して消去する
				if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[i], BLANK_CHECK_ENTIRE_BLOCK) == FLASH_NOT_BLANK)
				{	//	消去
					while(R_FlashGetStatus() != FLASH_SUCCESS);
					if(R_FlashErase(i) != FLASH_SUCCESS)
					{
						return UDS_EC_GPF;
					}
					while(R_FlashGetStatus() != FLASH_SUCCESS);
					if(R_FlashDataAreaBlankCheck(g_flash_BlockAddresses[i], BLANK_CHECK_ENTIRE_BLOCK) != FLASH_BLANK)
					{	//	消去失敗
						return UDS_EC_GPF;
					}
				}
			}
		}
	}
	else
	if(adr >= 0xFFE00000ul)
	{	//	Program Flash ROM
		if(uds_security_access < 2 && (adr >= 0xFFF40000ul || (adr + (unsigned long)siz) >= 0xFFF40000ul)) return UDS_EC_CNC;	//	範囲エラー
		if(uds_security_access < 3 && (adr >= 0xFFF80000ul || (adr + (unsigned long)siz) >= 0xFFF80000ul)) return UDS_EC_CNC;	//	範囲エラー
		if(siz > (0x40000 - (adr & 0x0003FFFF)) || siz < 0)
		{	//	サイズエラー
			return UDS_EC_UDNA;
		}
		if((adr & 0x0001FFFF) == 0)
		{	//	消去
			if(adr < 0xFFF00000ul)
			{	//	64Kブロック(データROM領域)
				sb = BLOCK_69 - ((adr - 0xFFE00000ul) >> 16);
				eb = sb - ((siz + 0xFFFF) >> 16);
				bs = 0x10000;
				if(eb < BLOCK_53) return UDS_EC_CNC;	//	範囲エラー
			}
			else
			if(adr < 0xFFF80000ul)
			{	//	32Kブロック(プログラムROM領域)
				sb = BLOCK_53 - ((adr - 0xFFF00000ul) >> 15);
				eb = sb - ((siz + 0x7FFF) >> 15);
				bs = 0x8000;
				if(eb < BLOCK_37) return UDS_EC_CNC;	//	範囲エラー
			}
			else
			if(adr < 0xFFFF8000ul)
			{	//	16Kブロック(ブートローダROM領域)
				sb = BLOCK_37 - ((adr - 0xFFF80000ul) >> 14);
				eb = sb - ((siz + 0x3FFF) >> 14);
				bs = 0x4000;
				if(eb < BLOCK_7) return UDS_EC_CNC;	//	範囲エラー
			}
			else
			{	//	4Kブロック(起動初期化ROM領域)
				sb = BLOCK_7 - ((adr - 0xFFFF8000ul) >> 12);
				eb = sb - ((siz + 0x0FFF) >> 12);
				bs = 0x1000;
				if(eb < -1) return UDS_EC_CNC;	//	範囲エラー
			}
			for(i = sb, j = siz; j > 0; j -= bs, i--)
			{	//	書き込むブロックを先行して消去する
				_di();
				if(R_FlashErase(i) != FLASH_SUCCESS)
				{
					_ei();
					return UDS_EC_GPF;
				}
				_ei();
			}
		}
	}
	else
	{	//	プログラム範囲エラー
		return UDS_EC_CNC;
	}
	//	ダウンロード状態
	uds_load.ADDR = adr;
	uds_load.SIZE = siz;
	uds_load.MODE = UDS_TD_DOWNLOAD;
	uds_load.BLKL = 1 + 128;	//	SID + Data[128]
	uds_load.CNT = 0;
	//	ブロックサイズを通知
	res[0] = req[0] | UDS_RES_SID;
	res[1] = 0x20;	//	word block size record
	res[2] = (uds_load.BLKL >> 8);
	res[3] = (uds_load.BLKL & 0xFF);
	*len = 4;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x35	Request Upload
//----------------------------------------------------------------------------------------
int uds_sid_35(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int	i, blk;
	int	siz;
	unsigned long	adr;

	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	if(uds_security_access == 0)
	{	//	ロック状態
		return UDS_EC_UDNA;
	}
	if(uds_load.MODE)
	{	//	実行中エラー
		return UDS_EC_BRR;
	}
	if(req[1] != 0x00)
	{	//	圧縮はサポート無し
		return UDS_EC_SFNS;
	}
	switch(req[2])
	{	//	フォーマット
	case 0x44:
		for(adr = 0, i = 3; i < 7; i++)
		{
			adr <<= 8;
			adr |= (unsigned long)req[i] & 0xFF;
		}
		for(siz = 0, i = 7; i < 11; i++)
		{
			siz <<= 8;
			siz |= (int)req[i] & 0xFF;
		}
	//	adr = (unsigned long)(((unsigned long)req[3] << 24) | ((unsigned long)req[4] << 16) | ((unsigned long)req[5] << 8) | (unsigned long)req[6]);
	//	siz = (int)(((unsigned long)req[7] << 24) | ((unsigned long)req[8] << 16) | ((unsigned long)req[9] << 8) | (unsigned long)req[10]);
		if(sz > 12)
		{	//	指定サイズ
			blk = (((int)req[11]) << 8) & 0xFF00;
			blk |= ((int)req[12]) & 0xFF;;
		}
		else
		{	//	F/Wサイズ
			blk = 129;
		}
		break;
	default:
		return UDS_EC_SFNS;
	}
	if(blk > 0xFFF && blk < 6)
	{	//	ブロックサイズ異常
		return UDS_EC_UDNA;
	}
	if(siz > 0x40000 || siz < 0)
	{	//	サイズエラー
		return UDS_EC_UDNA;
	}
	//	ダウンロード状態
	uds_load.ADDR = adr;
	uds_load.SIZE = siz;
	uds_load.MODE = UDS_TD_UPLOAD;
	uds_load.BLKL = 1 + 128;	//	SID + Data[128]
	uds_load.CNT = 0;
	if(blk < uds_load.BLKL) uds_load.BLKL = blk;
	//	ブロックサイズを通知
	res[0] = req[0] | UDS_RES_SID;
	res[1] = 0x20;	//	word record
	res[2] = (uds_load.BLKL >> 8);
	res[3] = (uds_load.BLKL & 0xFF);
	*len = 4;
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS 0x36	Transfer Data
//----------------------------------------------------------------------------------------
int uds_sid_36(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int	i;
	unsigned long	adr;
	unsigned char	*p;

	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	if(uds_security_access == 0)
	{	//	ロック状態
		return UDS_EC_UDNA;
	}
	if(uds_load.MODE == UDS_TD_DOWNLOAD)
	{	//	ダウンロード中（ツール→ECU）
		if(sz > uds_load.BLKL)
		{	//	サイズオーバー
			uds_load.MODE = UDS_TD_NONE;	//	転送中止
			return UDS_EC_IML_IF;
		}
		sz--;	//	コマンドコードバイト分減らす
		if((uds_load.CNT + sz) > uds_load.SIZE) sz = uds_load.SIZE - uds_load.CNT;
		if((uds_load.ADDR & 0xFFFC0000) == 0)
		{	//	RAMへ書き込み
			p = (unsigned char *)uds_load.ADDR;
			uds_load.ADDR += sz;
			uds_load.CNT += sz;
			memcpy(p, &req[1], sz);
		}
		else
		if((uds_load.ADDR & 0xFFFF8000ul) == 0x00100000ul)
		{	//	E2Data
			i = sz;
			if((i & (DF_PROGRAM_SIZE_SMALL - 1)) != 0)
			{	//	最小書き込みバイトに到達していない時は00で埋める
				for(;(i & (DF_PROGRAM_SIZE_SMALL - 1)) != 0; i++) req[1 + i] = 0;
			}
			if(R_FlashWrite(uds_load.ADDR, (int)&req[1], i) != FLASH_SUCCESS)
			{	//	書き込み失敗
				uds_load.MODE = UDS_TD_NONE;
				return UDS_EC_GPF;
			}
			uds_load.ADDR += sz;
			uds_load.CNT += sz;
		}
		else
		if(uds_load.ADDR >= 0xFFE00000ul)
		{	//	Program Flash ROM
			i = sz;
			if(i < ROM_PROGRAM_SIZE)
			{	//	プログラム単位の不足分はFFで埋める
				for(;i < ROM_PROGRAM_SIZE; i++) req[1 + i] = 0xFF;
			}
			_di();
			if(R_FlashWrite(uds_load.ADDR, (int)&req[1], i) != FLASH_SUCCESS)
			{	//	書き込み失敗
				_ei();
				uds_load.MODE = UDS_TD_NONE;
				return UDS_EC_GPF;
			}
			_ei();
			uds_load.ADDR += sz;
			uds_load.CNT += sz;
		}
		if(uds_load.CNT >= uds_load.SIZE)
		{	//	ダウンロード完了
			uds_load.MODE = UDS_TD_NONE;
		}
		i = (int)(uds_load.SIZE - uds_load.CNT);
		//	正常応答(残りのバイト数を通知)
		res[0] = req[0] | UDS_RES_SID;
		res[1] = 0x04;	//	long counter record
		res[2] = (i >> 24);
		res[3] = (i >> 16);
		res[4] = (i >> 8);
		res[5] = (i & 0xFF);
		*len = 6;
		return UDS_EC_NONE;
	}
	else
	if(uds_load.MODE == UDS_TD_UPLOAD)
	{	//	アップロード中（ECU→ツール）
		i = uds_load.BLKL;	//	サイズ調整
	//	if((uds_load.CNT + sz) > uds_load.SIZE) sz = uds_load.SIZE - uds_load.CNT;
		//	正常応答(残りのバイト数を通知)
		res[0] = req[0] | UDS_RES_SID;
		p = (unsigned char *)uds_load.ADDR;
		sz = (int)(uds_load.SIZE - uds_load.CNT) + 1;
		if(sz > uds_load.BLKL) sz = uds_load.BLKL;
	//	uds_load.ADDR += sz;
	//	uds_load.CNT += sz;
		for(i = 1; i < sz; i++)
		{
			res[i] = *p++;
			uds_load.ADDR++;
			uds_load.CNT++;
		}
		*len = i;
		return UDS_EC_NONE;
	}
	return UDS_EC_RSE;	//	手続きエラー
}

//----------------------------------------------------------------------------------------
//	UDS 0x37	Request Transfer Exit
//----------------------------------------------------------------------------------------
int uds_sid_37(unsigned char *req, int sz, unsigned char *res, int *len)
{
	int	i;
	if(uds_diag_session < 2)
	{	//	セッション低い
		return UDS_EC_GR;	//	一般拒否
	}
	if(uds_security_access == 0)
	{	//	ロック状態
		return UDS_EC_UDNA;
	}
	if(uds_load.MODE != UDS_TD_NONE)
	{
		if(uds_load.MODE == UDS_TD_DOWNLOAD)
		{	//	ダウンロード中（ツール→ECU）
			if(uds_load.ADDR >= 0xFFF00000ul && uds_load.ADDR < 0xFFF20000ul)
			{	//	ユーザーファームウェア領域を中断する場合はROM消去する
				R_FlashErase(BLOCK_53);
			}
		}
		uds_load.MODE = UDS_TD_NONE;
		i = (int)(uds_load.SIZE - uds_load.CNT);
		//	中断時の残りのバイト数を通知
		res[0] = req[0] | UDS_RES_SID;
		res[1] = 0x04;	//	long counter record
		res[2] = (i >> 24);
		res[3] = (i >> 16);
		res[4] = (i >> 8);
		res[5] = (i & 0xFF);
		*len = 6;
	}
	else
	{	//	終了
		res[0] = req[0] | UDS_RES_SID;
		res[1] = 0x00;
		*len = 2;
	}
	return UDS_EC_NONE;
}

//----------------------------------------------------------------------------------------
//	UDS処理
//----------------------------------------------------------------------------------------
int uds_job(unsigned char *msg, int len, unsigned char *res)	//int ch, int id, void *frame)
{
	int	size = 0;
	int	ercd = UDS_EC_NONE;
	//	サービス実行
	switch(msg[0])
	{
	default:	//	サービス未サポート
		res[0] = UDS_ERR_SID;
		res[1] = msg[0];
		res[2] = UDS_EC_SNS;
		return 3;
	case 0x10:	//	Diagnostic Session Control
		ercd = uds_sid_10(msg, len, res, &size);
		break;
	case 0x11:	//	ECU Reset
		ercd = uds_sid_11(msg, len, res, &size);
		break;
	case 0x27:	//	Security Access
		ercd = uds_sid_27(msg, len, res, &size);
		break;
	case 0x3E:	//	Tester Present
		ercd = uds_sid_3e(msg, len, res, &size);
		break;
	case 0x22:	//	Read Data By Identifier
		ercd = uds_sid_22(msg, len, res, &size);
		break;
	case 0x23:	//	Read Memory By Address
		ercd = uds_sid_23(msg, len, res, &size);
		break;
	case 0x2E:	//	Write Data By Identifier
		ercd = uds_sid_2e(msg, len, res, &size);
		break;
	case 0x3D:	//	Write Memory By Address
		ercd = uds_sid_3d(msg, len, res, &size);
		break;
	case 0x34:	//	Request Download
		ercd = uds_sid_34(msg, len, res, &size);
		break;
	case 0x35:	//	Request Upload
		ercd = uds_sid_35(msg, len, res, &size);
		break;
	case 0x36:	//	Transfer Data
		ercd = uds_sid_36(msg, len, res, &size);
		break;
	case 0x37:	//	Request Transfer Exit
		ercd = uds_sid_37(msg, len, res, &size);
		break;
	}
	if(ercd != UDS_EC_NONE)
	{	//	エラー有り
		res[0] = UDS_ERR_SID;
		res[1] = msg[0];
		res[2] = ercd;
		return 3;
	}
	after_call(DTC_TIMER_ID, 10000, uds_timeup);	//	接続維持10秒タイマー
	return size;
}


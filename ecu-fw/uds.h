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

//	#include "uds.h"			/*	CAN-UDS 定義			*/

#ifndef		__CAN_UDS_PROTOCOL__
#define		__CAN_UDS_PROTOCOL__

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

//----------------------------------------------------------------------------------------
//	変数定義
//----------------------------------------------------------------------------------------
extern	int				uds_diag_session;			//	セッションコントロール
extern	int				uds_p2_can_server_max;		//	P2 Time
extern	int				uds_p2e_can_server_max;		//	P2E Time

extern	int				uds_reset_request;			//	リセット要求

extern	int				uds_security_access;		//	セキュリティーアクセス

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

extern	UDS_LOAD_STR	uds_load;						//	ダウンロード・アップロード管理

#define			UDS_BUFFER_MAX		(128+1)		/*	UDSで送受信可能な最大データサイズ					*/

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
extern	void	can_uds_init(void);
//----------------------------------------------------------------------------------------
//	UDS 0x10	Diagnostic Session Control
//----------------------------------------------------------------------------------------
extern	int uds_sid_10(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x11	ECU Reset
//----------------------------------------------------------------------------------------
extern	int uds_sid_11(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x27	Security Access
//----------------------------------------------------------------------------------------
extern	int uds_sid_27(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x3E	Tester Present
//----------------------------------------------------------------------------------------
extern	int uds_sid_3e(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x22	Read Data By Identifier
//----------------------------------------------------------------------------------------
extern	int uds_sid_22(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x23	Read Memory By Address
//----------------------------------------------------------------------------------------
extern	int uds_sid_23(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x2E	Write Data By Identifier
//----------------------------------------------------------------------------------------
extern	int uds_sid_2e(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x3D	Write Memory By Address
//----------------------------------------------------------------------------------------
extern	int uds_sid_3d(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x34	Request Download
//----------------------------------------------------------------------------------------
extern	int uds_sid_34(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x35	Request Upload
//----------------------------------------------------------------------------------------
extern	int uds_sid_35(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x36	Transfer Data
//----------------------------------------------------------------------------------------
extern	int uds_sid_36(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS 0x37	Request Transfer Exit
//----------------------------------------------------------------------------------------
extern	int uds_sid_37(unsigned char *req, int sz, unsigned char *res, int *len);
//----------------------------------------------------------------------------------------
//	UDS処理
//----------------------------------------------------------------------------------------
extern	int uds_job(unsigned char *msg, int len, unsigned char *res);

#endif		/*__CAN_UDS_PROTOCOL__*/

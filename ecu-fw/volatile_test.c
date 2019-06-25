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

#include "iodefine.h"

//	volatile属性のコンパイルエラー問題
//	モジュール定義行
//	#define	RSPI2	(*(volatile struct st_rspi    __evenaccess *)0x883C0)
void test(void)
{
	//	RSPI2.SPDR はSPI送受信のデータレジスタで、4フレームまで積み上げられる。
	RSPI2.SPDR = 0x321C0000;	//	コマンド＋ダミー0	正常
	RSPI2.SPDR = 0x00000000;	//	ダミー1				正常
	RSPI2.SPDR = 0x00000000;	//	ダミー2				ASMコード無し
	RSPI2.SPDR = 0x00000000;	//	ダミー3				ASMコード無し
}

/*
-----------------------------------------------------
	コンパイル結果に12,13行のコードが存在しない
-----------------------------------------------------
	segment	TEXT ATR_CODE
public	_test
_test:
	MOV.L	#_STACK_BTM,R1
	MVFC	PSW,R2
	BTST	#H'00000011,R2
	STNZ	#_USTACK_BTM,R1
	CMP	R1,SP
	BGTU	_test_stack_check_end
	MOV.L	#_stack_over_task,R1
	MOV.L	#_test,R2
	JMP	R1
_test_stack_check_end:
??FUNCDEF(???test) ??VOID
??FUNCEND
??BEGINBLOCK(8)
??CLINE 10
	MOV.L	#H'000883C0,R3
	MOV.L	#H'321C0000,R4
	MOV.L	R4,4[R3]
??CLINE 11
	SUB	R5,R5
	MOV.L	R5,4[R3]
??CLINE 12
??CLINE 13
??ENDBLOCK(14)
??CLINE 14
_test_end_:
	RTS

*/


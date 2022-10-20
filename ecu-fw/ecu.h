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

/* -*-c++-*-
 * $RCSfile: ecu.h,v $
 * $Revision: 1.00 $
 * $Date: 2016/12/15 14:14:48 $
 *
 * Copyright (c) 2016 LandF Corporation.
 *
 * History:
 */

#ifndef __CAN_ECU_CGW__
#define __CAN_ECU_CGW__

/* Enable the following if it is generated for testing CAN port 1 using LFY_RX63N
 * board
 *#define  __LFY_RX63N__*/

// Whether or not LFY-RX63N has USB console 
#ifdef __LFY_RX63N__
//#define  __USE_LFY_USB__ 
#endif

/* ---------------------------------------------------------------------------------------
 * Get ECU unit code
 * --------------------------------------------------------------------------------------- */
#ifdef __LFY_RX63N__
// LFY test environment 
#define SELECT_ECU_UNIT \
    0 // Select test function 0:Power train / 1:Chassis / 2:Body 
#define ECU_UNIT_POWERTRAIN 0
#define ECU_UNIT_CHASSIS 1
#define ECU_UNIT_BODY 2
#define ECU_UNIT_CGW 7
#else // ifdef __LFY_RX63N__
// CAN2ECU environment 
#define DPSW_ROM_BOOT (PORTF.PIDR.BIT.B5)                // DipSwitch S1-8 
#define DPSW_BOOTCOPY (PORTJ.PIDR.BIT.B5)                // DipSwitch S1-7 
#define SELECT_ECU_UNIT ((~PORT5.PIDR.BYTE >> 1) & 0x07) // DipSwitch S1-6,5,4 
#define ECU_UNIT_POWERTRAIN 0
#define ECU_UNIT_CHASSIS 1
#define ECU_UNIT_BODY 2
#define ECU_UNIT_PTBD 3
#define ECU_UNIT_CGW 7
#endif // ifdef __LFY_RX63N__

/* ---------------------------------------------------------------------------------------
 * CAN message buffer definition
 * --------------------------------------------------------------------------------------- */
#define CAN_ID_MAX   0x800
#define CAN_ID_MASK  0x7FF
#define MESSAGE_MAX  0x100
#define MESSAGE_MSK  0x0FF
#define MESSAGE_END  0x1FF
#define MESSAGE_BOXS 3

// Port number setting 
#define CAN_CH_MAX      4 // CAN2ECU board is CAN-4ch 
#define CAN_TEST_LFY_CH 1 // LFY-RX63N is valid only for CAN1 

#define DLC_MASK 0x0F

#define CAN_DATA_FRAME   0
#define CAN_REMOTE_FRAME 1

// MBOX selection designation 
#define MBOX_POINT_1 0x200
#define MBOX_POINT_2 0x400

// Maximum number of external I/O checklists 
#define ECU_EXT_MAX 64
// External I/O management 
#define EX_IO_MAX 64

// Sort the waiting list by ID priority 
#define SORT_TXWAITLIST_ENABLE

// Routing map definition structure (stored in E2DATA) 
typedef struct __routing_map__ {
    union {
        unsigned char BYTE;
        struct {
            unsigned char   RE3 : 1; // CAN-Port3 Receiving    1=Enable/0=Disable 
            unsigned char   RE2 : 1; // CAN-Port2 Receiving    1=Enable/0=Disable 
            unsigned char   RE1 : 1; // CAN-Port1 Receiving    1=Enable/0=Disable 
            unsigned char   RE0 : 1; // CAN-Port0 Receiving    1=Enable/0=Disable 
            unsigned char   TE3 : 1; // CAN-Port3 Transmission 1=Enable/0=Disable 
            unsigned char   TE2 : 1; // CAN-Port2 Transmission 1=Enable/0=Disable 
            unsigned char   TE1 : 1; // CAN-Port1 Transmission 1=Enable/0=Disable 
            unsigned char   TE0 : 1; // CAN-Port0 Transmission 1=Enable/0=Disable 
        } BIT;
    } ID[CAN_ID_MAX];
} ECU_ROUT_MAP;

// CGW port routing map control bit definition 
#define EX_R_BIT 0x80 //Receive  external CAN 
#define CS_R_BIT 0x40 //Receive  chassis 
#define BD_R_BIT 0x20 //Receive  body 
#define PT_R_BIT 0x10 //Receive  powertrain 
#define EX_W_BIT 0x08 //Transmit external CAN 
#define CS_W_BIT 0x04 //Transmit chassis 
#define BD_W_BIT 0x02 //Transmit body 
#define PT_W_BIT 0x01 //Transmit powertrain 

#define CS_TO_PT (CS_R_BIT | PT_W_BIT)            // Powertrain from chassis 
#define CS_TO_BD (CS_R_BIT | BD_W_BIT)            // Body from chassis 
#define PT_TO_CS (PT_R_BIT | CS_W_BIT)            // Chassis from powertrain 
#define BD_TO_CS (BD_R_BIT | CS_W_BIT)            // Chassis from body 
#define PT_TO_BD (PT_R_BIT | BD_W_BIT)            // Body from powertrain 
#define BD_TO_PT (BD_R_BIT | PT_W_BIT)            // Powertrain from body 
#define CS_TO_AL (CS_R_BIT | PT_W_BIT | BD_W_BIT) // All from chassis 
#define PT_TO_AL (PT_R_BIT | CS_W_BIT | BD_W_BIT) // All from powertrain 
#define BD_TO_AL (BD_R_BIT | PT_W_BIT | CS_W_BIT) // All from body 
#define EX_TO_AL \
    (EX_R_BIT | PT_W_BIT | CS_W_BIT | BD_W_BIT)   // All from external CAN 
#define EX_TO_PT (EX_R_BIT | PT_W_BIT)            // Powertrain from external CAN 
#define EX_TO_CS (EX_R_BIT | CS_W_BIT)            // Body from external CAN 
#define EX_TO_BD (EX_R_BIT | BD_W_BIT)            // Chassis from external CAN 
#define PT_TO_EX (PT_R_BIT | EX_W_BIT)            // External CAN from powertrain 
#define BD_TO_EX (BD_R_BIT | EX_W_BIT)            // External CAN from body 
#define CS_TO_EX (CS_R_BIT | EX_W_BIT)            // External CAN from chassis 
#define AL_TO_EX \
    (PT_R_BIT | BD_R_BIT | CS_R_BIT | \
     EX_W_BIT) // External CAN from all 
#define AL_TO_AL \
    (EX_R_BIT | PT_R_BIT | BD_R_BIT | CS_R_BIT | EX_W_BIT | PT_W_BIT | BD_W_BIT | \
     CS_W_BIT) // Forward all 

// CAN-ID union 
typedef union __can_id_form__ {
    unsigned long LONG;
    struct {
        unsigned short  H;
        unsigned short  L;
    } WORD;
    struct {
        unsigned char   HH;
        unsigned char   HL;
        unsigned char   LH;
        unsigned char   LL;
    } BYTE;
    struct {
        unsigned long   IDE : 1;  // [0] 
        unsigned long   RTR : 1;  // Remote transmission request bit (0=Data frame / 1=Remote frame) 
        unsigned long       : 1;  // [0] 
        unsigned long SID   : 11; // Standard ID bit 
        unsigned long       : 2;  // [0] 
        unsigned long   DLC : 4;  // Frame data length specification (0 to 8) 
        unsigned long   ENB : 1;  // Processing enable flag (0=Disable / 1=Enable) 
        unsigned long   REP : 1;  // Repeat flag (0=Event / 1=Period) 
        unsigned long       : 1;  // [0] 
        unsigned long NXT   : 9;  /* Next cycle / event number (Valid only during
                                   * execution: 0 to 255 / No continuation: 256 to 511)*/
    } BIT;
} CAN_ID_FORM;

// Period / event definition structure (stored in E2DATA) 
typedef struct __cycle_event_str__ {
    CAN_ID_FORM ID; // Target ID number 
    union {
        long LONG;
        struct {
            short   TIME; // Period time or event delay time (ms) 
            short   CNT;  // Counter or event delay increase time (ms) 
        } WORD;
    } TIMER;
} ECU_CYC_EVE;

// Period/event registration information 
typedef struct __cycle_event_info__ {
    int         WP;                // Write pointer 
    int         TOP;               // Start pointer 
    int         CNT;               // Registration number 
    ECU_CYC_EVE LIST[MESSAGE_MAX]; // Period/event information 
} CYCLE_EVENTS;

// Message box use ID range setting 
typedef struct __mbox_select_id__ {
    struct {
        unsigned short  MB1; // ID range to apply to message box 0   0 to MB1 
        unsigned short  MB2; // ID range to apply to message box 0 MB1 to MB2 
    } CH[CAN_CH_MAX];
} MBOX_SELECT_ID;

// Waiting transmission frame management structure 
typedef struct __send_wait_frame__ {
    CAN_ID_FORM ID; // Message ID number (0 to 2047) 
    union {
        unsigned long   LONG[2];
        unsigned short  WORD[4];
        unsigned char   BYTE[8];
    } FD; // Frame data 
} SEND_WAIT_FLAME;

// Waiting buffer definition structure 
typedef struct __send_wait_buffer__ {
    struct {
        int             WP;               // Registration pointer                  (0 to MESSAGE_MAX-1) 
        int             TOP;              // Start pointer                         (0 to MESSAGE_MAX-1) 
        int             CNT;              // Number of messages waiting to be sent (0 to MESSAGE_MAX) 
        SEND_WAIT_FLAME MSG[MESSAGE_MAX]; // Holding message buffer                (Max. 256) 
    } BOX[MESSAGE_BOXS];
} SEND_WAIT_BUF;

// CAN frame data union 
typedef union __can_frame_data__ {
    unsigned long   LONG[2];
    unsigned short  WORD[4];
    unsigned char   BYTE[8];
} CAN_DATA_BYTE;

// Frame data buffer definition structure 
typedef struct __can_frame_buffer__ {
    CAN_DATA_BYTE ID[CAN_ID_MAX];
} CAN_FRAME_BUF;

// CAN bus message box definition structure 
typedef struct __can_message_box__ {
    union {
        unsigned long LONG;
        struct {
            unsigned short  H;
            unsigned short  L;
        } WORD;
        struct {
            unsigned char   HH;
            unsigned char   HL;
            unsigned char   LH;
            unsigned char   LL;
        } BYTE;
        struct {
            unsigned long   IDE : 1;  // [0] 
            unsigned long   RTR : 1;  /* Remote transmission request bit (0=data frame
                                       *    / 1=remote frame)*/
            unsigned long       : 1;  // [0] 
            unsigned long   SID : 11; // Standard ID bits 
            unsigned long   EID : 18; // [0] 
        } BIT;
    } ID;
    unsigned short  DLC;     // Data length (0 to 8) 
    unsigned char   DATA[8]; // Data body (8byte) 
    unsigned short  TS;      // Timestamp (read only) 
} CAN_MBOX;

// External I/O definition structure 4+4+24=32byte 
typedef struct __externul_io_str__ {
    int SID; // ID number    -1:Disable / 000 to 7FF:Enable 
    union {
        unsigned long LONG;
        struct {
            unsigned long       : 1; // Reserve    0 
            unsigned long MODE  : 3; // I/O processing mode setting 0:Disable / 1:DI 
            /* / 2:DO / 3:AI / 4:AO / 5:2 bit value / 6:3 bit
             *value / 7:4 bit value*/
            unsigned long   NEG     : 1; // Data inversion specification  Negative conditions 
            unsigned long   SIZE    : 3; // Access size      0:BIT / 1 to 7:nBYTE 
            unsigned long   BPOS    : 4; // Byte position  0 to 7 
            unsigned long   DLC     : 4; // Data byte length 0 to 8 
            unsigned long   NOM     : 8; // Port number   0 to 33 
            unsigned long   MSK     : 8; // Mask    00 to FF 
        } BIT;
    } PORT;
    unsigned char   FRMCNT;  // Frame counter 
    unsigned char   PAT[23]; /* Pattern data When size>0, copy source by multiple
                              * bit reference*/
} EXTERNUL_IO;

// CAN-ID -> EX-I/O-ID Conversion table 
extern unsigned char can_to_exio[CAN_ID_MAX]; /* The data indicated by the CAN-ID
                                               * is the external I/O management
                                               * number. 00-3F=Applicable, FF=Not
                                               * supported*/

// Pointer multi-access union definition 
typedef union __pointer_multi_access__ {
    unsigned long   LONG;
    void *          VP;
    signed char *   SB;
    unsigned char * UB;
    signed short *  SW;
    unsigned short *UW;
    signed int *    SI;
    unsigned int *  UI;
    signed long *   SL;
    unsigned long * UL;
    ECU_CYC_EVE *   CYE;
    ECU_ROUT_MAP *  MAP;
    CYCLE_EVENTS *  CONF;
    CYCLE_EVENTS *  WAIT;
    SEND_WAIT_BUF * SMSG;
    CAN_FRAME_BUF * CAN;
    MBOX_SELECT_ID *MBOX;
    EXTERNUL_IO *   EXL;
} POINTER_MULTI_ACCESS;

/* E2DATA flash save variable
 * Routing map*/
extern ECU_ROUT_MAP rout_map; // Map variables 
// Definition holding buffer 
extern CYCLE_EVENTS conf_ecu; /* Period/event/remote management definition
                               * variables*/
// ECU I/O checklist 32*64=2048byte 
extern EXTERNUL_IO  ext_list[ECU_EXT_MAX];
extern int          ext_list_count; // Number of registered external I/O processes 
extern short        ds_conect_active[2]; //	DS Connection flag

/* Variable on RAM
 * Time-up waiting buffer*/
extern CYCLE_EVENTS wait_tup; // Cycle/event wait variable 
// Transmission waiting buffer for each message box 
extern SEND_WAIT_BUF send_msg[CAN_CH_MAX];
// CAN data buffer variables 
extern CAN_FRAME_BUF    can_buf;
extern CAN_FRAME_BUF    can_random_mask;

// Message box range 
extern MBOX_SELECT_ID mbox_sel;
// Stacking of message box CAN frame transmission buffer 
extern void add_mbox_frame(int ch, int dlc, int rtr, int id);

// Repro mode flag 
extern int repro_mode; // 0=Normal mode / 1=Repro mode 

// Receive sub-buffer 
#define RX_MB_BUF_MAX 32
typedef struct __rx_mb_buffer__ {
    CAN_MBOX    MB[RX_MB_BUF_MAX];
    int         WP;
    int         RP;
} RX_MB_BUF;

extern RX_MB_BUF rxmb_buf[3]; // Receive sub-buffer 

// extern int    ds_conect_active; // Driving simulator connection flag 

// LED monitoring ID setting 
extern int              led_monit_id;  // Monitor ID 
extern unsigned char    led_monit_ch;  // Monitor CH bit set 
extern int              led_monit_first; // Shortest time 
extern int              led_monit_slow; // Longest time 
extern int              led_monit_time; // Average time 
extern int              led_monit_count; // Number of averaging 
extern int              led_monit_sample; // Number of samples 

// E2DATA flash definition 
#define ADDRESS_OF_ROOTMAP \
    0x00100000 // Route map    2048byte   0x00100000 to 0x001007FF 
#define ADDRESS_OF_CYCEVE \
    0x00100800 // Period/Event 2048byte   0x00100800 to 0x00100FFF 
#define ADDRESS_OF_IOLIST \
    0x00101000 // I/O check     272byte   0x00101000 to 0x001017FF 

/* ---------------------------------------------------------------------------------------
 * CARLA mode selection setting 2021/02/22
 * --------------------------------------------------------------------------------------- */
#define		ECU_OPMODE_0		0		/*	PASTA.C -> RS232C -> ECU.C -> CAN.BUS -> ECU.P.B -> RS232C -> PASTA.P.B	*/
#define		ECU_OPMODE_1		1		/*	PASTA.C -> RS232C -> ECU.C -> CAN.BUS -> CARLA.P.B						*/
#define		ECU_OPMODE_2		2		/*	CARLA.C -> CAN.BUS -> CARLA.P.B											*/
#define		ECU_OPMODE_3		3		/*	PASTA.C -> RS232C -> ECU.C -> CAN.BUS -> ECU.P.B -> RS232C -> CARLA.P.B	*/
#define		ECU_OPMODE_4		4		/*	CARLA.C -> RS232C -> ECU.C -> CAN.BUS -> ECU.P.B -> RS232C -> CARLA.P.B	*/
#define		ECU_OPMODE_5		5		/*	Vi Mode Phase1 DS mode	*/
#define		ECU_OPMODE_6		6		/*	Vi Mode Phase1 DS mode	*/

typedef	struct	__ecu_opemode_str__
{
	int		mode;				//	operaion code
	int		mode_bk;			//	operaion code
	char	chgio[EX_IO_MAX];	//	mode chainged I/O list nomber (0..63 or -1)
	short	chgid[EX_IO_MAX];	//	mode chainged CAN-ID (0x000..0x7CF or -1)
}	ECU_OPE_MODE_STR;

//	CARLA operaion mode
extern	ECU_OPE_MODE_STR	ecu_opmode;

/* ---------------------------------------------------------------------------------------
 * search_target_id
 * 
 * Outline
 *     Check if the ID is managed by your own station
 * 
 * Argument
 *     int id  Search ID number
 * 
 * Description
 *     Searches the specified ID number from the definition buffer
 * 
 * Return
 *     int  0 or more:Definition buffer number / -1:Excluded ID
 * --------------------------------------------------------------------------------------- */
extern int search_target_id(int id);
/* ---------------------------------------------------------------------------------------
 * add_cyceve_list
 * 
 * Outline
 *     Add periodic event data
 * 
 * Argument
 *     int rtr   Remote frame specification      0/1
 *     int id    CAN message number              0 to 2047
 *     int dlc   Data byte length                0 to 8
 *     int enb   Processing permission flag      0/1
 *     int rep   Periodic frame specification    0/1
 *     int time  Period time or delay time (ms)  0 to 65535
 *     int cnt   Delay increase time (ms)        0 to 65535
 * 
 * Description
 *     Registration of cycle/event
 * 
 * Return
 *     Buffer addition position
 * --------------------------------------------------------------------------------------- */
extern int add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time,
                           int cnt);
/* ---------------------------------------------------------------------------------------
 * insert_cyceve_list
 * 
 * Outline
 *     Midway addition of cyclic event data
 * 
 * Argument
 *     int mi  Buffer number  0 to 255
 * 
 * Description
 *     Linking registered information of cycle / event
 * 
 * Return
 *     Buffer addition position
 * --------------------------------------------------------------------------------------- */
extern void insert_cyceve_list(int mi);
/* ---------------------------------------------------------------------------------------
 * Outline
 *     Delete cyclic event data
 * --------------------------------------------------------------------------------------- */
extern void delete_cyceve_list(int id);
/* ---------------------------------------------------------------------------------------
 * can_id_event
 * 
 * Outline
 *     Data update event generation processing
 * 
 * Argument
 *     int id  ID number where the change occurred
 *     int tp  Additional wait time (ms)
 * 
 * Description
 *     Called by a data update request from an external device other than CAN
 *     Add the ID of the event processing target to the time-up queue
 * 
 * Return
 *     Negative number is registration failure / 0 or more increases time(ms)
 * --------------------------------------------------------------------------------------- */
extern int can_id_event(int id, int tp);
/* ---------------------------------------------------------------------------------------
 * Outline
 *     Delete data waiting for periodic event time-up
 * --------------------------------------------------------------------------------------- */
extern void delete_waiting_list(int id);
/* ----------------------------------------------------------------------------------------
 * BootCopy processing
 * ---------------------------------------------------------------------------------------- */
extern int  bootcopy(void);
extern int  bootclear(void);
/* ----------------------------------------------------------------------------------------
 * Batch storage of ECU operation data
 * ---------------------------------------------------------------------------------------- */
extern int ecu_data_write(void);
/* ----------------------------------------------------------------------------------------
 * Batch deletion of ECU operation data
 * ---------------------------------------------------------------------------------------- */
extern int ecu_data_erase(void);
/* ----------------------------------------------------------------------------------------
 * Check the writing status of ECU operation data
 * ---------------------------------------------------------------------------------------- */
extern int ecu_data_check(void);

/* ----------------------------------------------------------------------------------------
 * Time difference measurement function between sending and receiving
 * ---------------------------------------------------------------------------------------- */
extern void monit_timeover(void);

#endif //__CAN_ECU_CGW__

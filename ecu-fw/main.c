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

/* ----------------------------------------------------------------------------------------
 *
 *  CAN2ECU Main processing
 *
 * ----------------------------------------------------------------------------------------
 *  Development history
 *
 *  2017/12/10  Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 *  T.Tachibana
 *  L&F
 * ----------------------------------------------------------------------------------------
 */

#include <sysio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "altypes.h"
#include "iodefine.h"
#include "sci.h"
#include "rtc.h"
#include "timer.h"
#include "ecu.h"
#include "flash_data.h"
#include "flash_rom.h"
#include "r_Flash_API_RX600.h"
#include "usb.h"
#include "can3_spi2.h"
#include "uSD_rspi1.h"
#include "cantp.h" // CAN-TP definition 
#include "uds.h"   // CAN-UDS definition 

/* ----------------------------------------------------------------------------------------
 *  CAN2ECU Main Variable definition
 * ----------------------------------------------------------------------------------------*/
#define COMMAND_BUF_MAX 512
typedef struct __console_command_buffer__ {
    int     WP;
    char    BUF[COMMAND_BUF_MAX];
} CONSOLE_CTRL;

CONSOLE_CTRL sci_console;
#ifdef SCI2_ACTIVATE
CONSOLE_CTRL sci2_console;
#endif
#if defined(USB_ACTIVATE) && defined(__LFY_RX63N__)
CONSOLE_CTRL usb_console;
#endif
int retport = 0;
#ifdef __LFY_RX63N__
int console_port = 1;
#else
int console_port = 0;
#endif

const char  def_ecu_corp[16]    = "PASTA";
const char  def_ecu_name[16]    = "ECU1S";
const char  def_ecu_vars[16]    = "Ver2.5";
const char  def_ecu_date[16]    = __DATE__;
const char  def_ecu_time[16]    = __TIME__;

// Communication temporary storage data buffer 
#define RAM_BUFFER_MAX 128
unsigned char comm_ram_buffer[RAM_BUFFER_MAX];

//  Unregistered interrupt handling 
interrupt void Dummy_Interrupt(void)
{}

// ECU processing 
void    ecu_job(void);               // ECU operation 
void    ecu_status(char *cmd);       // Parameter status check 
void    ecu_get_command(char *cmd);  // Frame data acquisition 
void    ecu_set_command(char *cmd);  // Rewrite frame data 
void    ecu_put_command(char *cmd);  // Direct frame transmission 
void    ecu_put_message(int id, int size, unsigned char *buf);
void    ecu_input_update(char *cmd); // Update I/O information via communication 

/* ----------------------------------------------------------------------------------------
 * iwdt_refresh Refresh watchdog
 * 
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void iwdt_refresh(void)
{
    unsigned short cnt = IWDT.IWDTSR.WORD;
    if ((cnt & 0xC000) != 0 || cnt == 0) {
        return; // Ignored when reset has occurred and is invalid 
    }
    if (cnt > 0x2FFF) {
        return; // Over 75% of 3FFF is out of range 
    }
    // IWDT refresh 
    IWDT.IWDTRR = 0x00;
    IWDT.IWDTRR = 0xFF;
}

/* ----------------------------------------------------------------------------------------
 * wdt_init Initiate a restart by watchdog
 * 
 *  Argument
 *      None
 * 
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void wdt_init(void)
{
    WDT.WDTCR.BIT.TOPS  = 3;    // Timeout period 0:1024  1:4096  2:8192  3:16384 
    WDT.WDTCR.BIT.CKS   = 8;    /* Clock division 1:PCLK/4  4:PCLK/64  16:PCLK/128
                                 * 6:PCLK/512 7:PCLK/2048  8:PCLK/8192*/
    WDT.WDTCR.BIT.RPES  = 3;    // Window end position 0:75% 1:50% 2:75% 3:100% 
    WDT.WDTCR.BIT.RPSS  = 3;    // Window start position 0:75% 1:50% 2:75% 3:100% 
    WDT.WDTRR           = 0x00; // Register start 1 
    WDT.WDTRR           = 0xff; // Register start 2 
}

/* ----------------------------------------------------------------------------------------
 * hex_to_byte
 * 
 *  Argument
 *      *p   HEX string
 *      *d   Byte string storage destination
 * 
 *  Return
 *      int  Number of conversion bytes
 * ----------------------------------------------------------------------------------------*/
int hex_to_byte(char *p, unsigned char *d)
{
    int     i = 0;
    char    c;

    while (*p != 0 && *(p + 1) != 0 && i < 32) {
        c = *p++;
        if (
            (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
            (c >= 'a' && c <= 'f')
        ) {
            if (c > '9') {
                c -= 7;
            }
            d[i]    = ((int)c << 4) & 0xF0;
            c       = *p++;
            if (
                (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
                (c >= 'a' && c <= 'f')
            ) {
                if (c > '9') {
                    c -= 7;
                }
                d[i] |= c & 0x0F;
                i++;
            } else {
                break;
            }
        } else {
            break;
        }
    }
    return i;
}

/* ----------------------------------------------------------------------------------------
 * byte_to_ulong
 * 
 *  Argument
 *      unsigned char *data  Byte sequence (big endian)
 *      int index   Start position
 *      int size   Reference bytes
 * 
 *  Return
 *      unsigned long  Acquisition value
 * ----------------------------------------------------------------------------------------*/
unsigned long byte_to_ulong(unsigned char *data, int index, int size)
{
    int             i;
    unsigned long   d   = 0;
    unsigned char * p   = data + index;

    for (i = 0; i < size; i++) {
        d   <<= 8;
        d   |= ((unsigned long)p[i] & 0x00FF);
    }
    return d;
}

/* ----------------------------------------------------------------------------------------
 * send_var
 * 
 *  Function description
 *      Send device information to COM port
 *  
 *  Argument
 *      ch  Destination COM channel number
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void send_var(int ch)
{
    char s[256];
	sprintf(s, "VER %s %s %s %s\r", def_ecu_name, def_ecu_vars, def_ecu_date, def_ecu_time);
    sci_puts(ch, s);
}

/* ----------------------------------------------------------------------------------------
 * SendPC
 * 
 *  Function description
 *      Reply receive command
 *  
 *  Argument
 *      *msg  Reply message
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void SendPC(char *msg)
{
    switch (retport) {
    case 0: //  COM0 
#ifdef SCI0_ACTIVATE
        sci_puts(0, msg);
        break;
#endif
    case 1: //  COM1 
#if defined(SCI1_ACTIVATE) || defined(__LFY_RX63N__)
        sci_puts(1, msg);
        break;
#endif
    case 2: //  COM2 
#ifdef SCI2_ACTIVATE
        sci_puts(2, msg);
        break;
#endif
    case 3: //  COM3 
#ifdef SCI3_ACTIVATE
        sci_puts(3, msg);
        break;
#endif
    case 4: //  USB 
#if defined(USB_ACTIVATE) && defined(__LFY_RX63N__)
        usb_puts(msg);
        usb_flush();
#endif
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * logging(char *fmt, ...)
 * 
 *  Outline
 *      Log output
 *
 *  Argument
 *      bp  working buffer pointer
 *      fmt formatted output
 * 
 *  
 *  Descriotion
 *      Client for log output that can be formatted similarly to the
 *      printf function
 * 
 *  Return value
 *      None
 * ----------------------------------------------------------------------------------------*/
void logging(char *fmt, ...)
{
    va_list args;
    int     n;
    char    bp[256];

    va_start(args, fmt);
    strcpy(bp, "");
    vsprintf(bp, fmt, args);
    va_end(args);
    SendPC(bp);
}

/* ----------------------------------------------------------------------------------------
 *  PortInit
 *
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void PortInit(void)
{
    unsigned char d;

    SYSTEM.PRCR.WORD = 0xA502;   /* Release of register protection for operation mode
                                  * and power consumption reduction function*/
    SYSTEM.SYSCR0.WORD = 0x5A01; // Internal ROM enabled, external bus disabled 
    while ((SYSTEM.SYSCR0.WORD & 3) != 1) {
        ;
    }

    MPC.PWPR.BIT.B0WI   = 0; // First, write 0 on B0WI 
    MPC.PWPR.BIT.PFSWE  = 1; // Enable writing to PFS register 

    MPC.PFAOE0.BYTE = 0x00;
    MPC.PFAOE1.BYTE = 0x00;

    memset(&sci_console, 0, sizeof(CONSOLE_CTRL));
#ifdef SCI2_ACTIVATE
    memset(&sci2_console, 0, sizeof(CONSOLE_CTRL));
#endif
#if defined(USB_ACTIVATE) && defined(__LFY_RX63N__)
    memset(&usb_console, 0, sizeof(CONSOLE_CTRL));
#endif
    WriteINTB(0);
}

/* ----------------------------------------------------------------------------------------
 * rtc_command_job
 *  
 *  Outline
 *      RTS Command processing
 * 
 *  Argument
 *      char *cmd Command string
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void rtc_command_job(char *cmd)
{
    int         year, mon, day, hour, min, sec;
    time_bcd_t  tm;
    /* ---------------------------------------------------
     *  Command analysis
     * --------------------------------------------------- */
    if (strlen(cmd) == 0) { // RTC data read 
        rtc_time_read(&tm);
        logging(
                    "RTC=%04X/%02X/%02X %02X:%02X:%02X\r", (int)tm.year, (int)tm.month,
                    (int)tm.day, (int)tm.hour, (int)tm.minute, (int)tm.second
        );
    } else if (strlen(cmd) >= 17) { // RTC data set 
        if (strlen(cmd) == 17) {
            if (
                6 != sscanf(
                            &cmd[3], "%02X/%02X/%02X %02X:%02X:%02X", &year, &mon,
                            &day, &hour, &min, &sec
                )
            ) {
                return;
            }
        } else {
            if (
                6 != sscanf(
                            &cmd[3], "%04X/%02X/%02X %02X:%02X:%02X", &year, &mon,
                            &day, &hour, &min, &sec
                )
            ) {
                return;
            }
        }
        tm.year     = 0x00ff & (char)year;
        tm.month    = (char)mon;
        tm.day      = (char)day;
        tm.hour     = (char)hour;
        tm.minute   = (char)min;
        tm.second   = (char)sec;
        rtc_init(&tm);
        rtc_time_read(&tm);
        logging(
                    "RTC=%04X/%02X/%02X %02X:%02X:%02X\r", (int)tm.year, (int)tm.month,
                    (int)tm.day, (int)tm.hour, (int)tm.minute, (int)tm.second
        );
    }
}

/* ----------------------------------------------------------------------------------------
 * command_job 
 *  
 *  Outline
 *      SCI/USB Command reception processing
 * 
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------
 */
void command_job(char *cmd)
{
    int     id, dt;
    int     db[8];
    char *  p;
    /* ---------------------------------------------------
     *  Command analysis
     * --------------------------------------------------- */
    switch (*cmd++) {
    case '?': // Device code 
        switch (SELECT_ECU_UNIT) {
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
    case 'B':                                  // ROM operation command 
        if (strncmp(cmd, "OOTCOPY", 7) == 0) { // Copy the running program to ROM 
            if (bootcopy() == 1) {             // Success 
                logging("BootCopy OK\r");
            } else { // Failure 
                logging("BootCopy NG\r");
            }
        } else if (strncmp(cmd, "OOTCLEAR", 8) == 0) { // Clear ROM storage area 
            if (bootclear() == 1) {                    // Success 
                logging("BootClear OK\r");
            } else { // Failure 
                logging("BootClear NG\r");
            }
        }
        break;
    case 'C': // Period / event list 
        switch (*cmd++) {
        case 'C': // Delete all lists 
            if (cmd[0] == 'A' && cmd[1] == 'L' && cmd[2] == 'L') { // [CCALL]Command 
                // Zero initialization 
                memset(
                            &wait_tup, 0,
                            sizeof(wait_tup)
                ); // Period / event wait initialization 
                wait_tup.TOP = -1;
                memset(&conf_ecu, 0, sizeof(conf_ecu)); /* Initialization of cycle /
                                                         * event / remote management
                                                         * definition*/
                conf_ecu.TOP = -1;
                logging("CCALL OK\r");
            }
            break;
        case 'R': // Delete ID from list 
            while (*cmd == ' ') {
                cmd++;
            }
            if (sscanf(cmd, "%x", &id) == 1) { // Set value acquisition [CR id] 
                if (id >= 0 && id < CAN_ID_MAX) {
                    delete_cyceve_list(id);  // Delete managed ID 
                    delete_waiting_list(id); // Delete time waiting ID 
                    logging("CR %03X OK\r", id);
                }
            }
            break;
        case 'A': // Add ID to list 
            while (*cmd == ' ') {
                cmd++;
            }
            if (
                sscanf(
                            cmd, "%x %d %d %d %d %d %d", &id, &db[0], &db[1], &db[2],
                            &db[3], &db[4], &db[5]
                ) == 7
            ) {      // Set value acquisition [CA id rtr dlc enb rep time cnt] 
                if (id >= 0 && id < CAN_ID_MAX) {
                    dt = add_cyceve_list(
                                db[0], id, db[1], db[2], db[3], db[4],
                                db[5]
                    );
                    can_id_event(dt, 0); // Event registration 
                    logging("CA %03X OK\r", id);
                }
            }
            break;
        }
        break;
    case 'M':
        if (cmd[0] == 'A') {     // MA? */
            if (cmd[1] == 'P') { // MAP command 
                cmd += 2;
                while (*cmd == ' ') {
                    cmd++;
                }
                if (sscanf(cmd, "%x %x", &id, &dt) == 2) { // Set value acquisition 
                    if (id >= 0 && id < CAN_ID_MAX) {
                        rout_map.ID[id].BYTE = (unsigned char)dt;
                    }
                }
            }
        } else if (cmd[0] == 'O') {
        	if (cmd[1] == 'N') { /* MON command (obtains the time difference
	             * between the reception of the specified ID and
	             * the completion of transmission)*/
	            cmd += 2;
	            while (*cmd == ' ') {
	                cmd++;
	            }
	            db[0] = sscanf(cmd, "%x %x %d", &id, &dt, &db[1]);
	            if (db[0] >= 2) { // Set value acquisition 
	                cmt1_stop();
	                led_monit_id        = id;     // Test ID 
	                led_monit_ch        = dt;     // Test channel 
	                led_monit_first     = 0x7FFFFFFF; // Shortest time 
	                led_monit_slow      = 0;      // Longest time 
	                led_monit_time      = 0;      // Average time 
	                led_monit_count     = 0;      // Averaging time 
	                led_monit_sample    = (db[0] == 3) ? db[1] : 50;
	            }
	        } else if (cmd[1] == 'D') { /* MOD command */
	            cmd += 2;
	            while (*cmd == ' ') {
	                cmd++;
	            }
	            dt = sscanf(cmd, "%d %d", &db[2], &db[0]);
	            if (dt >= 1) { // Set value acquisition
	            	switch(db[2]) {
	            	case 0:
	            	case 1:
			            if(dt <= 1) db[0] = 0x00;
		            	id = 0x7D0;
		            	db[1] = ~db[0] & 0xFF;
			           	logging("MOD%d\r", db[2]);
			           	ds_conect_active[1] = db[2];
			         	ds_conect_active[0] = (~db[2]) & 1;
			         	{
			         		unsigned char s[8];
			         		memset(s, 0xff, sizeof(s));
			         		s[0] = (unsigned char)db[0];
			         		s[1] = (unsigned char)db[1];
			         		s[2] = (unsigned char)db[2];
			            	ecu_put_message(id, 8, s);	//	CAN
			            }
	            		break;
	            	}
	            }
	        }
        }
        break;
    case 'E': // ECU command 
        if (
            cmd[0] == 'X' &&
            cmd[1] == 'D'
        ) { // EXD I/O input command via communication 
            ecu_input_update(&cmd[2]);
        } else { // Internal information return request 
            ecu_status(cmd);
        }
        break;
    case 'G': // ECU specified ID frame acquisition command 
        if (cmd[0] == 'E' && cmd[1] == 'T') {
            ecu_get_command(&cmd[2]);
        }
        break;
    case 'S': // ECU specified ID frame setting command 
        if (cmd[0] == 'E' && cmd[1] == 'T') {
            ecu_set_command(&cmd[2]);
        }
        break;
    case 'P': // ECU specified ID frame transmission command 
        if (cmd[0] == 'U' && cmd[1] == 'T') {
            ecu_put_command(&cmd[2]);
        }
        break;
    case 'R':                                // R 
        if (strncmp(cmd, "EBOOT", 5) == 0) { // Restart by soft reset 
            logging("ReBoot OK\r");
            wdt_init();
        } else if (
            cmd[0] == 'T' &&
            cmd[1] == 'C'
        ) { // [RTC]Real-time clock operation command 
            while (*cmd == ' ') {
                cmd++;
            }
            rtc_command_job(cmd);
        } else if (cmd[0] == 'B' && cmd[1] == 'U') { /* [RBU]RAM stack command RBU
                                                      * pointer length data.. Up to 32
                                                      * bytes at one time*/
            while (*cmd == ' ') {
                cmd++;
            }
            db[0]   = 0; // Argument counter 
            db[1]   = 0; // Pointer 
            db[2]   = 0; // Length 
            while (*cmd != 0) {
                p = cmd;
                while (*cmd != ' ' && *cmd != 0) {
                    cmd++;
                }
                switch (db[0]) {
                case 0: // Pointer setting 
                    if (sscanf(p, "%d", &db[1]) != 1) {
                        logging("RBU Error 1\r");
                        return;
                    }
                    if (db[1] < 0 || db[1] >= RAM_BUFFER_MAX) {
                        logging("RBU Over 1\r");
                        return;
                    }
                    break;
                case 1: // Length setting 
                    if (sscanf(p, "%d", &db[2]) != 1) {
                        logging("RBU Error 2\r");
                        return;
                    }
                    if ((db[1] + db[2]) >= RAM_BUFFER_MAX) {
                        logging("RBU Over 2\r");
                        return;
                    }
                    break;
                default: // Write data 
                    if (sscanf(p, "%x", &db[3]) != 1) {
                        logging("RBU Error %d\r", db[0]);
                        return;
                    }
                    if (
                        (db[0] - 2 + db[1]) <
                        RAM_BUFFER_MAX
                    ) { // Within buffer range 
                        comm_ram_buffer[(db[0] - 2 + db[1])] = (unsigned char)db[3];
                    }
                    break;
                }
                db[0]++;
                if ((db[0] - 2) >= db[2] || (db[0] - 2 + db[1]) >= RAM_BUFFER_MAX) {
                    break;
                }
            }
            if ((db[0] - 2) < db[2]) { // Insufficient data count 
                logging("RBU Lost %d\r", db[0]);
                return;
            }
        } else if (
            cmd[0] == 'W' &&
            cmd[1] == 'L'
        ) { // [RWL]ROM write command RWL address length 
            while (*cmd == ' ') {
                cmd++;
            }
            if (sscanf(cmd, "%x %d", &db[0], &db[1]) == 2) { // Parameter count match 
                _di();
                if (
                    R_FlashWrite(
                                (unsigned long) db[0], (unsigned long)&comm_ram_buffer,
                                (unsigned short)db[1]
                    ) != FLASH_SUCCESS
                ) { // Write failed 
                    _ei();
                    logging("RWL Error\r");
                    return;
                }
                _ei();
                logging("RWL Success %X\r", db[0]);
            }
        }
        break;
    case 'W':                                 // Save to data flash 
        if (cmd[0] == 'D' && cmd[1] == 'F') { // [WDF]Command 
            id = ecu_data_write();
            if (id == 7) { // Save successful 
                logging("WDF OK\r");
            } else { // Save failed 
                logging("WDF NG %d\r", id);
            }
        }
        break;
	case 'V':
		if(cmd[0] == 'E' && cmd[1] == 'R')
		{	//	[VER]
			send_var(retport);
		}
        break;
    default:
        logging("Command Error !\r");
        break;
    }
}

/* ----------------------------------------------------------------------------------------
 * comm_job 
 *  
 *  Outline
 *      SCI/USB Command reception processing
 * 
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
void comm_job(void)
{
    int             i, n;
    int             numBytes;
    int             year, mon, day, hour, min, sec;
    time_bcd_t      tm;
    unsigned short  d;
    char            buffer[64];
    char            c;

    //  RS-232C 
    numBytes = sci_get_check(console_port);
    if (numBytes != 0) { //  COM1 reception processing 
        for (i = 0; i < numBytes; i++) {
            c = sci_get_char(console_port);
            // sci_putc(console_port, c); //  Echo back 
            if (c == 0x0D) { //  [CR] 
                sci_console.BUF[sci_console.WP] = 0;
                sci_console.WP                  = 0;
                retport                         = console_port;
                command_job(sci_console.BUF);
            } else if (c == 0x08 || c == 0x7F) { // Delete 1 character 
                if (sci_console.WP > 0) {
                    sci_console.WP--;
                }
            } else {
                if (c >= 0x61 && c <= 0x7a) { // Convert lowercase to uppercase 
                    sci_console.BUF[sci_console.WP] = c - 0x20;
                } else {
                    sci_console.BUF[sci_console.WP] = c;
                }
                sci_console.WP++;
                if (sci_console.WP >= COMMAND_BUF_MAX) {
                    sci_console.WP = 0;
                }
            }
        }
    }
#ifdef SCI2_ACTIVATE
    //  RS-232C(COM2) 
    numBytes = sci_get_check(2);
    if (numBytes != 0) { //  COM1 reception processing 
        for (i = 0; i < numBytes; i++) {
            c = sci_get_char(2);
            // sci_putc(console_port, c); //  Echo back 
            if (c == 0x0D) { //  [CR] 
                sci2_console.BUF[sci2_console.WP]   = 0;
                sci2_console.WP                     = 0;
                retport                             = 2;
                command_job(sci2_console.BUF);
            } else if (c == 0x08 || c == 0x7F) { // Delete 1 character 
                if (sci2_console.WP > 0) {
                    sci2_console.WP--;
                }
            } else {
                if (c >= 0x61 && c <= 0x7a) { // Convert lowercase to uppercase 
                    sci2_console.BUF[sci2_console.WP] = c - 0x20;
                } else {
                    sci2_console.BUF[sci2_console.WP] = c;
                }
                sci2_console.WP++;
                if (sci2_console.WP >= COMMAND_BUF_MAX) {
                    sci2_console.WP = 0;
                }
            }
        }
    }
#endif // ifdef SCI2_ACTIVATE
#ifdef __LFY_RX63N__
#ifdef __USE_LFY_USB__
    //  USB 
    for (numBytes = 0; numBytes < sizeof(buffer); numBytes++) {
        n = usb_getch();
        if (n < 0) {
            break;
        }
        buffer[numBytes] = (char)n;
    }
    if (numBytes != 0) {
        for (i = 0; i < numBytes; i++) {
            c = buffer[i];
            if (c == 0x0D) { //  [CR] 
                usb_console.BUF[usb_console.WP] = 0;
                usb_console.WP                  = 0;
                retport                         = 4; // USB 
                command_job(usb_console.BUF);
            } else if (c != 0) {
                if (c >= 0x61 && c <= 0x7a) { // Convert lowercase to uppercase 
                    usb_console.BUF[usb_console.WP] = c - 0x20;
                } else if (c == 0x08 || c == 0x7F) { // Delete 1 character 
                    if (usb_console.WP > 0) {
                        usb_console.WP--;
                    }
                } else {
                    usb_console.BUF[usb_console.WP] = c;
                }
                usb_console.WP++;
                if (usb_console.WP >= COMMAND_BUF_MAX) {
                    usb_console.WP = 0;
                }
            }
        }
    }
#endif // ifdef __USE_LFY_USB__
#endif // ifdef __LFY_RX63N__
}

/* ----------------------------------------------------------------------------------------
 * can_ctrl
 * 
 *  Function description
 *      ECU processing call of CAN port
 * 
 *  Argument
 *      None
 * 
 *  Return
 *     None
 * ----------------------------------------------------------------------------------------*/
void can_ctrl(void)
{
// CAN call 
#ifndef __LFY_RX63N__
#ifdef RSPI2_ACTIVATE
#if (CAN_CH_MAX == 4)
    if (can3_job()) /* CAN3 requires special processing as it is controlled via
                     * RSPI2*/
#endif
#endif
#endif // ifndef __LFY_RX63N__
    ecu_job();     // ECU processing (control of CAN0 to 2 and ECU processing) 
}

/* ----------------------------------------------------------------------------------------
 * main
 *  
 *  Outline
 *      Initialization + CAN2ECU Main
 *  
 *  Argument
 *      None
 *  
 *  Return
 *      None
 * ----------------------------------------------------------------------------------------*/
int main(void)
{
    // Start initialization 
    PortInit();  // I/O port initialization 
    cmt0_init(); // CMT0 module setting 
    cmt1_init(); // CMT1 module setting 
#ifdef __LFY_RX63N__

#ifdef SCI1_ACTIVATE
    // SCI1 module setting (RS-232C) <--> FWRITE2(debug-port) 
    sci1_init(38400, 8, 1, 0);
#endif

#else //__LFY_RX63N__

#ifdef SCI0_ACTIVATE
    // SCI0 module setting (RS-232C) <--> LF74 
    sci0_init(9600, 8, 2, 0);
#endif
#ifdef SCI1_ACTIVATE
    // SCI1 module setting (RS-232C) <--> FWRITE2(debug-port) 
    sci1_init(9600, 8, 2, 0);
#endif
#ifdef SCI2_ACTIVATE
    // SCI2 module setting (RS-232C) <--> EXTERNUL-LF74 
    sci2_init(9600, 8, 2, 0);
#endif
#ifdef SCI3_ACTIVATE
    // SCI3 module setting (RS-232C) <--> LF62(USB memory) 
    sci3_init(9600, 8, 2, 0);
#endif

#endif //__LFY_RX63N__

    _ei();

// Interrupt enable function (Y scope uses interrupt) 

#ifdef  __LFY_RX63N__
#ifdef  __USE_LFY_USB__
    usb_init(); //  USB0 module setting 
#endif
#endif

    //  Startup notification 
#ifdef  __LFY_RX63N__
    retport         = 1;
    console_port    = 1;
#ifdef  SCI1_ACTIVATE
    sci_puts(1, " \r\n \r\n \r\n");
    send_var(1);
#endif
#else // ifdef  __LFY_RX63N__
    retport         = 0; // COMx 
    console_port    = 0;
#ifdef  SCI0_ACTIVATE
    sci_puts(0, " \r\n \r\n \r\n");
    send_var(0);
#endif
#ifdef  SCI1_ACTIVATE
    sci_puts(1, " \r\n \r\n \r\n");
    send_var(1);
#endif
#ifdef  SCI2_ACTIVATE
    sci_puts(2, " \r\n \r\n \r\n");
    send_var(2);
#endif
#ifdef  SCI3_ACTIVATE
    sci_puts(3, " \r\n \r\n \r\n");
    send_var(3);
#endif
#endif // ifdef  __LFY_RX63N__

    // ROM operation initialization 
    reset_fcu();    // FCU reset 
    flash_init();   // FCU initialize 
    can_tp_init();  // CAN-TP initialization 
    can_uds_init(); // CAN-UDS initialization 

    // Special processing at startup  * ROMization when firmware is started from YScope with both S1-7 and S8 ON 
    if (DPSW_ROM_BOOT == 0) { // REM-MON start-up 
        if (DPSW_BOOTCOPY == 0) { // ROMization compulsory at the start of F/W operation 
            if ((*((unsigned long *)0x00000064)) < 0x00020000) { // ROMization if F/W boot source is REM-MON 
                command_job("BOOTCLEAR");   // ROM erase 
                command_job("BOOTCOPY");    // ROM write 
            }
        }
    }

#ifndef __LFY_RX63N__
    // MCP2515 initialization 
    can3_init();
#endif

    // Main routine 
    for (;;) {
        iwdt_refresh(); // IWDT reflesh 
        cmt0_job();     // Time up call 
        can_ctrl();     // CAN control 
        comm_job();     // SCI/USB command processing 
        if (tp_pack.TXIF) {    // Transmission completion processing request 
            tp_pack.TXIF = 0;  // Release request 
            can_tp_txendreq(); // CAN-TP transmission completion processing call 
        }
        if (uds_reset_request != 0) { // ECU restart 
            switch (uds_reset_request) {
            case 1: // Hart reset 
                wdt_init();
                break;
            case 2: // Clear status 
                memset(&can_buf, 0, sizeof(can_buf));
                break;
            case 3: // Soft reset 
                SYSTEM.PRCR.WORD    = 0xA502;
                SYSTEM.SWRR         = 0xA501;
                break;
            }
            uds_reset_request = 0;
        }
    }
    return 0;
}

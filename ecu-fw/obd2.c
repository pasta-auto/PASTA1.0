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
 * OBD2-CAN Protocol processing
 *
 * ----------------------------------------------------------------------------------------
 * Development history
 *
 * 2017/02/13 Start coding (by Tachibana)
 *
 * ----------------------------------------------------------------------------------------
 * T.Tachibana
 * L&F
 * ----------------------------------------------------------------------------------------
 */

#include <sysio.h>
#include <string.h>
#include <stdio.h>
#include "iodefine.h"
#include "timer.h"
#include "ecu.h"       // ECU common definition 
#include "can3_spi2.h" // CAN3 definition 
#include "cantp.h"     // CAN-TP definition 
#include "obd2.h"      // CAN-OBDII definition 

/* ----------------------------------------------------------------------------------------
 * Variable definition
 * ---------------------------------------------------------------------------------------- */
OBD2_QUERY_FRAME    obd2_req; // Request data 
OBD2_QUERY_FRAME    obd2_ret; // Response data 

int obd2_ret_counter = 0;

/* ----------------------------------------------------------------------------------------
 * MODE1 Processing  0x0C, 0x0D, 0x1C, 0x2F, 0x31, 0x49, 0x51
 * ---------------------------------------------------------------------------------------- */
int obd2_mode1(int len)
{
    int d;
    // Standard requirements 
    if (len >= 2) {
        len = 2;                            // Default 2 bytes 
        obd2_ret.SAE_ECU.MODE   = obd2_req.SAE_OBD.MODE + 0x40; // Response flag 
        obd2_ret.SAE_ECU.PID    = obd2_req.SAE_OBD.PID;         // Parameter ID copy 
        // Processing for each PID 
        switch (obd2_req.SAE_OBD.PID) {
                   //------------------------------------------------------ 
        case 0x00: // Support PID information [01 - 20]
                   // ------------------------------------------------------
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x18;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x11; // 0x0C,0x0D,0x1C,(0x20) 
            break;
        case 0x01: /* Monitor status since DTCs cleared. (Includes malfunction
                    * indicator lamp (MIL) status and number of DTCs.)*/
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] =
                0x00; // (A) bit7:MIL emission check / bit6 to 0:DTC_CNT 
            obd2_ret.SAE_ECU.VAL[1] =
                0x00; /* (B) bit7:[0] / bit3: 0 = spark plug type, 1 = diesel /
                       * bit2: configuration test, bit6: incomplete / bit1: fuel gauge
                       * test, bit5: incomplete / bit0: misfire test, bit4: incomplete*/
            obd2_ret.SAE_ECU.VAL[2] =
                0x00; /* (C) bit7: EGR system test / bit6: Oxygen sensor heater
                       * test / bit5: Oxygen sensor test / bit4: AC refrigerant test /
                       * bit3: 2nd air system test / bit2: Evaporation system test /
                       * bit1: Heating accelerator test / bit0: Acceleration Agent
                       * test*/
            obd2_ret.SAE_ECU.VAL[3] =
                0x00; /* (D) bit7: Incomplete EGR system / bit6: Incomplete oxygen
                       * sensor heater / bit5: Incomplete oxygen sensor / bit4:
                       * Incomplete AC refrigerant / bit3: Incomplete 2nd air system /
                       * bit2: Incomplete evaporation system / bit1: Heating promotion
                       * Incomplete agent / bit0: Incomplete accelerator*/
            break;
        case 0x02: // Freeze DTC (Stop saving DTC) 
            break;
        case 0x03: // Fuel system status 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 2; /* 1: Engine low temperature open loop / 2:
                                          * Mix ratio feedback closed loop / 4:
                                          * Deceleration closed loop / 8: Fault open
                                          * loop / 16: Closed loop defect*/
            break;
        case 0x04: // Calculated engine load 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 100[%]Engine torque value  100/255*A 
            break;
        case 0x05: // Engine coolant temperature 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] =
                can_buf.ID[0x183]
                .BYTE[0];     // -40 to 215[° C]Engine coolant temperature A-40 
            break;
        case 0x06: // Short term fuel trim-Bank 1 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -100 to 99.2[%] Short-term fuel bank 1 100/128*A-100 
            break;
        case 0x07: // Long term fuel trim-Bank 1 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -100 to 99.2[%] Long-term fuel bank 1 100/128*A-100 
            break;
        case 0x08: // Short term fuel trim-Bank 2 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -100 to 99.2[%] Short-term fuel bank 2 100/128*A-100 
            break;
        case 0x09: // Long term fuel trim-Bank 2 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -100 to 99.2[%] Long-term fuel bank 2 100/128*A-100 
            break;
        case 0x0A: // Fuel pressure (gauge pressure) 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 765[kPa](/3) Fuel pressure 3*A 
            break;
        case 0x0B: // Intake manifold absolute pressure 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 255[kPa] Exhaust tube absolute pressure  A 
            break;
        case 0x0C: // Engine RPM 
            len += 2;
            d   = ((((int)can_buf.ID[0x043].BYTE[0]) << 8) |
                   (((int)can_buf.ID[0x043].BYTE[1]) & 0xFF)) *
                  4;
            obd2_ret.SAE_ECU.VAL[0] = (unsigned char)(d >> 8);   /* 0 to
                                                                  * 16383.75[rpm] Engine
                                                                  * RPM (256A+B)/4*/
            obd2_ret.SAE_ECU.VAL[1] = (unsigned char)(d & 0xFF); // ↑ Lower 8bit 
            break;
        case 0x0D: // Vehicle speed 
            len += 1;
            d   = ((((int)can_buf.ID[0x043].BYTE[2]) << 8) |
                   (((int)can_buf.ID[0x043].BYTE[3]) & 0xFF));
            if (d & 0x8000) {
                d = 0x10000 - d;
            }
            obd2_ret.SAE_ECU.VAL[0] =
                d; // 0 to 255[km/h] Vehicle speed    A 
            break;
        case 0x0E: // Timing advance 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -64 to 63.5[°] Forward timing A/2-64 
            break;
        case 0x0F: // Intake air temperature 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -40 to 215[° C] Intake air temperature   A-40 
            break;
        case 0x10: //  MAF air flow rate 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 655.35[g/s] Mixture rate  (256A+B)/100 
            obd2_ret.SAE_ECU.VAL[1] = 0; // ↑ Lower 8bit 
            break;
        case 0x11: // Throttle position 
            len += 1;
            d   = ((((int)can_buf.ID[0x02F].BYTE[0]) << 8) |
                   (((int)can_buf.ID[0x02F].BYTE[1]) & 0xFF)) *
                  255 / 1023;
            obd2_ret.SAE_ECU.VAL[0] = (unsigned char)(d & 0xFF); /* 0 to 100[%]
                                                                  * Throttle position
                                                                  * 100A/255*/
            break;
        case 0x12: // Commanded secondary air status 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 1; /* 1: Upstream / 2: Under catalyst / 4:
                                          * Outside air / 8: At diagnosis*/
            break;
        case 0x13: // Oxygen sensors present (in 2 banks) 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 1; /* bit0-3: Bank1, sensor 1-4 / bit4-7:
                                          * Bank2, sensor 1-4 Oxygen sensor presence
                                          * flag*/
            break;
        case 0x14: // Oxygen Sensor 1  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 1 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x15: // Oxygen Sensor 2  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 2 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x16: // Oxygen Sensor 3  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 3 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x17: // Oxygen Sensor 4  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 4 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x18: // Oxygen Sensor 5  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 5 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x19: // Oxygen Sensor 6  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 6 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x1A: // Oxygen Sensor 7  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 7 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x1B: // Oxygen Sensor 8  A: Voltage  B: Short term fuel trim 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // 0 to 1.275[V] Oxygen sensor 8 voltage A/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // -100 to 99.2[%] Short-term fuel adjustment  100/128B-100 
            break;
        case 0x1C: // OBD standards this vehicle conforms to 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 1; /* 1:OBD2 of CARB specification /
                                          *11:JOBD,JOBD2 / 13:JOBD,EOBD,OBD2 etc.*/
            break;
        case 0x1D: // Oxygen sensors present (in 4 banks) 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] =
                1; /* bit0 to 1:Bank1,Sensor 1 to 2 / bit2 to 3:Bank2,Sensor 1 to 2
                    * / bit4 to 5:Bank3,Sensor 1 to 2 / bit6 to 7:Bank4,Sensor 1 to 2
                    * Oxygen sensor presence flag*/
            break;
        case 0x1E: // Auxiliary input status 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 1; /* bit0:PTO(Power take Off)
                                          * 1=Enable,0=Disable / bit7 to 1:[0]
                                          * Auxiliary input status*/
            break;
        case 0x1F: // Run time since engine start 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 65535[sec.] Elapsed time since engine start 256*A+B 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
                   //------------------------------------------------------ 
        case 0x20: /* PIDs supported [21 - 40]
                    * ------------------------------------------------------ */
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x02;
            obd2_ret.SAE_ECU.VAL[2] = 0x80;
            obd2_ret.SAE_ECU.VAL[3] = 0x01; // 0x2F,0x31,(0x40) 
            break;
        case 0x21: // Distance traveled with malfunction indicator lamp (MIL) on 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 65535[km] Mileage after MIL warning 256*A+B 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x22: // Fuel Rail Pressure (relative to manifold vacuum) 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 5177.265[kPa] Fuel rail pressure 0.079*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x23
            : // Fuel Rail Gauge Pressure (diesel, or gasoline direct injection) 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 655350[kPa] Fuel rail gauge pressure 10*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x24:    // Oxygen Sensor 1  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 1 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x25:    // Oxygen Sensor 2  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 2 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x26:    // Oxygen Sensor 3  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 3 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x27:    // Oxygen Sensor 4  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 4 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x28:    // Oxygen Sensor 5  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 5 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x29:    // Oxygen Sensor 6  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 6 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x2A:    // Oxygen Sensor 7  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 7 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x2B:    // Oxygen Sensor 8  AB: Fuel-Air Equivalence Ratio CD: Voltage 
            len                    += 2; // Oxygen sensor 8 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) 0 to 8[V] Sensor voltage 8/65536*(256*C+D) 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x2C: // Commanded EGR 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] EGR instruction  100/255*A 
            break;
        case 0x2D: // EGR Error 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -100 to 99.2[%] EGR error 100/128*A-100 
            break;
        case 0x2E: // Commanded evaporative purge 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Vaporization order 100/255*A 
            break;
        case 0x2F: // Fuel Tank Level Input 
            len += 1;
            d   = ((int)can_buf.ID[0x3D4].BYTE[0]) * 255 / 40; // 0..40 liter 
            if (d > 255) {
                d = 255;
            }
            obd2_ret.SAE_ECU.VAL[0] =
                (unsigned char)d; // (A) 0 to 100[%] Fuel remaining  100/255*A 
            break;
        case 0x30: // Warm-ups since codes cleared 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 255[count] Warm-up count A 
            break;
        case 0x31: // Distance traveled since codes cleared 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 65535[km] Preset odometer 256*A+B 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x32: // Evap. System Vapor Pressure 
            len                    += 2;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -8192 to 8191.75[Pa] System vapor pressure (256*A+B)/4 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x33: // Absolute Barometric Pressure 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 255[kPa] Absolute pressure A 
            break;
        case 0x34:    // Oxygen Sensor 1  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 1 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x35:    // Oxygen Sensor 2  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 2 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x36:    // Oxygen Sensor 3  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 3 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x37:    // Oxygen Sensor 4  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 4 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x38:    // Oxygen Sensor 5  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 5 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x39:    // Oxygen Sensor 6  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 6 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x3A:    // Oxygen Sensor 7  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 7 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x3B:    // Oxygen Sensor 8  AB: Fuel-Air Equivalence Ratio CD: Current 
            len                    += 4; // Oxygen sensor 8 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -128 to 128[mA] Sensor current (256*C+D)/256-128 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) ↑ Lower 8bit 
            break;
        case 0x3C:    // Catalyst Temperature: Bank 1, Sensor 1 
            len                    += 2; // Accelerator temperature bank 1 sensor 1 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -40 to 6513.5[° C] Temperature (256*A+B)/10-40 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x3D:    // Catalyst Temperature: Bank 2, Sensor 1 
            len                    += 2; // Accelerator temperature bank 2 sensor 1 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -40 to 6513.5[° C] Temperature (256*A+B)/10-40 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x3E:    // Catalyst Temperature: Bank 1, Sensor 2 
            len                    += 2; // Accelerator temperature bank 1 sensor 2 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -40 to 6513.5[° C] Temperature (256*A+B)/10-40 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x3F:    // Catalyst Temperature: Bank 2, Sensor 2 
            len                    += 2; // Accelerator temperature bank 2 sensor 2 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -40 to 6513.5[° C] Temperature (256*A+B)/10-40 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
                   //------------------------------------------------------ 
        case 0x40: // PIDs supported [41 - 60]
                   // ------------------------------------------------------
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x80;
            obd2_ret.SAE_ECU.VAL[2] = 0x80;
            obd2_ret.SAE_ECU.VAL[3] = 0x00; // 0x49,0x51 
            break;
        case 0x41: // Monitor status this drive cycle 
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00; // (A) [0] 
            obd2_ret.SAE_ECU.VAL[1] =
                0x00; /* (B) bit7:[0] / bit3: 0 = spark plug type, 1 = diesel /
                       * bit2: configuration test, bit6: incomplete / bit1: fuel gauge
                       * test, bit5: incomplete / bit0: misfire test, bit4: incomplete*/
            obd2_ret.SAE_ECU.VAL[2] =
                0x00; /* (C) bit7: EGR system test / bit6: Oxygen sensor heater
                       * test / bit5: Oxygen sensor test / bit4: AC refrigerant test /
                       * bit3: 2nd air system test / bit2: Evaporation system test /
                       * bit1: Heating accelerator test / bit0: Acceleration Agent
                       * test*/
            obd2_ret.SAE_ECU.VAL[3] =
                0x00; /* (D) bit7: Incomplete EGR system / bit6: Incomplete oxygen
                       * sensor heater / bit5: Incomplete oxygen sensor / bit4:
                       * Incomplete AC refrigerant / bit3: Incomplete 2nd air system /
                       * bit2: Incomplete evaporation system / bit1: Heating promotion
                       * Incomplete agent / bit0: Incomplete accelerator*/
            break;
        case 0x42:    // Control module voltage 
            len                    += 2; // Control module voltage 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 65.535[V] Voltage (256*A+B)/1000 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x43:    // Absolute load value 
            len                    += 2; // Absolute load value 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 25700[%] Amount of work 100/255*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x44:    // Fuel-Air commanded equivalence ratio 
            len                    += 2; // Fuel-Air commanded equivalence ratio 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 2[ratio] Fuel mixture ratio 2/65536*(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x45: // Relative throttle position 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Throttle relative position 100/255*A 
            break;
        case 0x46: // Ambient air temperature 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -40 to 215[° C] Ambient temperature A-40 
            break;
        case 0x47: // Absolute throttle position B 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Throttle absolute position B 100/255*A 
            break;
        case 0x48: // Absolute throttle position C 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Throttle absolute position C 100/255*A 
            break;
        case 0x49: // Accelerator pedal position D 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Accelerator position D 100/255*A 
            break;
        case 0x4A: // Accelerator pedal position E 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Accelerator position E 100/255*A 
            break;
        case 0x4B: // Accelerator pedal position F 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Accelerator position F 100/255*A 
            break;
        case 0x4C: // Commanded throttle actuator 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Throttle start command 100/255*A 
            break;
        case 0x4D:    // Time run with MIL on 
            len                    += 2; // Elapsed MIL firing time 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 65535[min.] Elapsed time 256*A+B 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x4E:    // Time since trouble codes cleared 
            len                    += 2; // Time since trouble codes cleared 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 65535[min.] Elapsed time 256*A+B 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x4F: /* Maximum value for Fuel-Air equivalence ratio, oxygen sensor
                    * voltage, oxygen sensor current, and intake manifold absolute
                    * pressure. */
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00; // (A) 0 to 255[ratio] Mixture ratio  A 
            obd2_ret.SAE_ECU.VAL[1] = 0x00; // (B) 0 to 255[V]     Oxygen sensor voltage B 
            obd2_ret.SAE_ECU.VAL[2] = 0x00; // (C) 0 to 255[mA]    Oxygen sensor current C 
            obd2_ret.SAE_ECU.VAL[3] = 0x00; // (D) 0 to 2550[kPa]  Intake pressure  D*10 
            break;
        case 0x50: // Maximum value for air flow rate from mass air flow sensor 
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] =
                0x00; // (A) 0 to 2550[g/s] Maximum air flow  A*10 
            obd2_ret.SAE_ECU.VAL[1] = 0x00; // (B) Spare 
            obd2_ret.SAE_ECU.VAL[2] = 0x00; // (C) Spare 
            obd2_ret.SAE_ECU.VAL[3] = 0x00; // (D) Spare 
            break;
        case 0x51: // Fuel Type 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] =
                1; /* (A) 1: Gasoline / 2: Methanol / 3: Ethanol / 4: Diesel / 5:
                    * LPG / 6: CNG / 7: Propane / 8: Electric / 21: Combined
                    * continuous electric and internal combustion engine (Prius)*/
            break;
        case 0x52: // Ethanol fuel % 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Ethanol fuel 100/255*A 
            break;
        case 0x53:    // Absolute Evap system Vapor Pressure 
            len                    += 2; // Absolute Evap system Vapor Pressure 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 327.675[kPa] Barometric pressure (256*A+B)/200 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x54:    // Evap system vapor pressure 
            len                    += 2; // Evap system vapor pressure 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -32767 to 32767[Pa] Barometric pressure (256*A+B)-32767 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x55: // Short term secondary oxygen sensor trim, A: bank 1, B: bank 3 
            len                    += 2; // Short term secondary oxygen sensor trim 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -100 to 99.2[%] Bank 1 adjustment value  100/128*A-100 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) -100 to 99.2[%] Bank 3 adjustment value  100/128*A-100 
            break;
        case 0x56:    // Long term secondary oxygen sensor trim, A: bank 1, B: bank 3 
            len                    += 2; // Long term secondary oxygen sensor trim 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -100 to 99.2[%] Bank 1 adjustment value  100/128*A-100 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) -100 to 99.2[%] Bank 3 adjustment value  100/128*A-100 
            break;
        case 0x57: // Short term secondary oxygen sensor trim, A: bank 2, B: bank 4 
            len                    += 2; // Short term secondary oxygen sensor trim 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -100 to 99.2[%] Bank 2 adjustment value  100/128*A-100 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) -100 to 99.2[%] Bank 4 adjustment value  100/128*A-100 
            break;
        case 0x58:    // Long term secondary oxygen sensor trim, A: bank 2, B: bank 4 
            len                    += 2; // Long term secondary oxygen sensor trim 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -100 to 99.2[%] Bank 2 adjustment value  100/128*A-100 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) -100 to 99.2[%] Bank 4 adjustment value  100/128*A-100 
            break;
        case 0x59:    // Fuel rail absolute pressure 
            len                    += 2; // Fuel rail absolute pressure 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 655350[kPa] Barometric pressure  10(256*A+B) 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x5A: // Relative accelerator pedal position 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Accelerator relative position 100/255*A 
            break;
        case 0x5B: // Hybrid battery pack remaining life 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 100[%] Hybrid battery level 100/255*A 
            break;
        case 0x5C: // Engine oil temperature 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // -40 to 210[° C] Engine oil temperature  A-40 
            break;
        case 0x5D:    // Fuel injection timing 
            len                    += 2; // Fuel injection timing 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -210.00 to 301.992[°] Timing (256*A+B)/128-210 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x5E:    // Engine fuel rate 
            len                    += 2; // Engine fuel rate 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 3276.75[L/h] Hourly fuel consumption (256*A+B)/20 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x5F: // Emission requirements to which vehicle is designed 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) Vehicle release condition design value? 
            break;
                   //------------------------------------------------------ 
        case 0x60: // PIDs supported [61 - 80]
                   // ------------------------------------------------------ 
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x00;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
        case 0x61: // Driver's demand engine - percent torque 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -125 to 125[%] Driver demand torque  A-125 
            break;
        case 0x62: // Actual engine - percent torque 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -125 to 125[%] Actual engine torque  A-125 
            break;
        case 0x63: // Engine reference torque 
            len                    += 2; // Engine reference torque 
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) 0 to 655.35[Nm] Torque 256*A+B 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) ↑ Lower 8bit 
            break;
        case 0x64: // Engine percent torque data 
            len                    += 5;
            obd2_ret.SAE_ECU.VAL[0] = 0; // (A) -125 to 125[%] Idle torque A-125 
            obd2_ret.SAE_ECU.VAL[1] = 0; // (B) -125 to 125[%] Engine point 1 B-125 
            obd2_ret.SAE_ECU.VAL[2] = 0; // (C) -125 to 125[%] Engine point 2 C-125 
            obd2_ret.SAE_ECU.VAL[3] = 0; // (D) -125 to 125[%] Engine point 3 D-125 
            obd2_ret.SAE_ECU.VAL[4] = 0; // (E) -125 to 125[%] Engine point 4 E-125 
            break;
        case 0x65: // Auxiliary input / output supported 
            break;
        case 0x66: // Mass air flow sensor 
            break;
        case 0x67: // Engine coolant temperature 
            break;
        case 0x68: // Intake air temperature sensor 
            break;
        case 0x69: // Commanded EGR and EGR Error 
            break;
        case 0x6A: /* Commanded Diesel intake air flow control and relative intake
                    *   air flow position*/
            break;
        case 0x6B: // Exhaust gas recirculation temperature 
            break;
        case 0x6C: /* Commanded throttle actuator control and relative throttle
                    *   position*/
            break;
        case 0x6D: // Fuel pressure control system 
            break;
        case 0x6E: // Injection pressure control system 
            break;
        case 0x6F: // Turbocharger compressor inlet pressure 
            break;
        case 0x70: // Boost pressure control 
            break;
        case 0x71: // Variable Geometry turbo (VGT) control 
            break;
        case 0x72: // Wastegate control 
            break;
        case 0x73: // Exhaust pressure 
            break;
        case 0x74: // Turbocharger RPM 
            break;
        case 0x75: // Turbocharger temperature 
            break;
        case 0x76: // Turbocharger temperature 
            break;
        case 0x77: // Charge air cooler temperature (CACT) 
            break;
        case 0x78:    // Exhaust Gas temperature (EGT) Bank 1 
            len += 4; // Bank 1 exhaust gas temperature 
            switch (obd2_ret_counter) {
            case 0:
                len = 1;
                obd2_ret.SAE_ECU.VAL[0] = 0x01; /* 1: EGT sensor 1/2: EGT sensor
                                                 * 2/4: EGT sensor 3/8: EGT sensor 4*/
                break;
            case 1: // EGT sensor 1, EGT sensor 2 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 2: // EGT sensor 3, EGT sensor 4 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            }
            obd2_ret_counter++;
            if (obd2_ret_counter >= 3) {
                obd2_ret_counter = 0;
            }
            break;
        case 0x79:    // Exhaust Gas temperature (EGT) Bank 2 
            len += 4; // Bank 2 exhaust gas temperature 
            switch (obd2_ret_counter) {
            case 0:
                len = 1;
                obd2_ret.SAE_ECU.VAL[0] = 0x01; /* 1: EGT sensor 1/2: EGT sensor
                                                 * 2/4: EGT sensor 3/8: EGT sensor 4*/
                break;
            case 1: // EGT sensor 1, EGT sensor 2 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 2: // EGT sensor 3, EGT sensor 4 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            }
            obd2_ret_counter++;
            if (obd2_ret_counter >= 3) {
                obd2_ret_counter = 0;
            }
            break;
        case 0x7A: // Diesel particulate filter (DPF) 
            break;
        case 0x7B: // Diesel particulate filter (DPF) 
            break;
        case 0x7C: // Diesel Particulate filter (DPF) temperature 
            break;
        case 0x7D: // NOx NTE control area status 
            break;
        case 0x7E: // PM NTE control area status 
            break;
        case 0x7F: // Engine run time 
            break;
        //------------------------------------------------------ 
        case 0x80: /* PIDs supported [81 - A0]
                    * ------------------------------------------------------ */
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x00;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
        case 0x81: // Engine run time for Auxiliary Emissions Control Device(AECD) 
            break;
        case 0x82: // Engine run time for Auxiliary Emissions Control Device(AECD) 
            break;
        case 0x83: // NOx sensor 
            break;
        case 0x84: // Manifold surface temperature 
            break;
        case 0x85: // NOx reagent system 
            break;
        case 0x86: // Particulate matter (PM) sensor 
            break;
        case 0x87: // Intake manifold absolute pressure 
            break;
                   //------------------------------------------------------ 
        case 0xA0: // PIDs supported [A1 - C0]
                   // ------------------------------------------------------ 
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x00;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
                   //------------------------------------------------------ 
        case 0xC0: // PIDs supported [C1 - E0]
                   // ------------------------------------------------------
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x00;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
                   //------------------------------------------------------ 
        case 0xE0: // PIDs supported [E1 - FF]
                   // ------------------------------------------------------
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x00;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
        }
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE2 Processing (DTC record return request)
 * ---------------------------------------------------------------------------------------- */
int obd2_mode2(int len)
{
    if (len == 2) { // Standard requirements 
        len = 2;
        obd2_ret.SAE_ECU.MODE   = obd2_req.SAE_OBD.MODE + 0x40;
        obd2_ret.SAE_ECU.PID    = obd2_req.SAE_OBD.PID;
        switch (obd2_req.SAE_OBD.PID) {
                   //------------------------------------------------------ 
        case 0x02: // DTC that caused freeze frame to be stored.
                   // ------------------------------------------------------
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0x00;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
        }
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE3 Processing (reply saved DTC record)
 * ---------------------------------------------------------------------------------------- */
int obd2_mode3(int len)
{
    if (len == 2) { // Standard requirements 
        len = 7;
        obd2_ret.MD3_ECU.MODE   = obd2_req.SAE_OBD.MODE + 0x40;
        obd2_ret.MD3_ECU.VAL[0] = 0x00;
        obd2_ret.MD3_ECU.VAL[1] = 0x00;
        obd2_ret.MD3_ECU.VAL[2] = 0x00;
        obd2_ret.MD3_ECU.VAL[3] = 0x00;
        obd2_ret.MD3_ECU.VAL[4] = 0x00;
        obd2_ret.MD3_ECU.VAL[5] = 0x00;
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE4 Process (delete DTC record)
 * ---------------------------------------------------------------------------------------- */
int obd2_mode4(int len)
{
    if (len == 2) { // Standard requirements 
        len                   = 1;
        obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE5 Processing
 * ---------------------------------------------------------------------------------------- */
int obd2_mode5(int len)
{
    if (len == 3) { // Standard requirements 
        len                  = 6;
        obd2_ret.VS_ECU.MODE = obd2_req.VS_OBD.MODE + 0x40;
        obd2_ret.VS_ECU.PIDH = obd2_req.VS_OBD.PIDH;
        obd2_ret.VS_ECU.PIDL = obd2_req.VS_OBD.PIDL;
        switch (obd2_req.VS_OBD.PIDH) {
        case 0x01:
            switch (obd2_req.VS_OBD.PIDL) {
                       //------------------------------------------------------ 
            case 0x00: // PIDs supported [01 - 20]
                       //------------------------------------------------------
                obd2_ret.VS_ECU.VAL[0] = 0x00;
                obd2_ret.VS_ECU.VAL[1] = 0x00;
                obd2_ret.VS_ECU.VAL[2] = 0x00;
                obd2_ret.VS_ECU.VAL[3] = 0x00;
                break;
            case 0x01: // O2 Sensor Monitor Bank 1 Sensor 1 
                break;
            case 0x02: // O2 Sensor Monitor Bank 1 Sensor 2 
                break;
            case 0x03: // O2 Sensor Monitor Bank 1 Sensor 3 
                break;
            case 0x04: // O2 Sensor Monitor Bank 1 Sensor 4 
                break;
            case 0x05: // O2 Sensor Monitor Bank 2 Sensor 1 
                break;
            case 0x06: // O2 Sensor Monitor Bank 2 Sensor 2 
                break;
            case 0x07: // O2 Sensor Monitor Bank 2 Sensor 3 
                break;
            case 0x08: // O2 Sensor Monitor Bank 2 Sensor 4 
                break;
            case 0x09: // O2 Sensor Monitor Bank 3 Sensor 1 
                break;
            case 0x0A: // O2 Sensor Monitor Bank 3 Sensor 2 
                break;
            case 0x0B: // O2 Sensor Monitor Bank 3 Sensor 3 
                break;
            case 0x0C: // O2 Sensor Monitor Bank 3 Sensor 4 
                break;
            case 0x0D: // O2 Sensor Monitor Bank 4 Sensor 1 
                break;
            case 0x0E: // O2 Sensor Monitor Bank 4 Sensor 2 
                break;
            case 0x0F: // O2 Sensor Monitor Bank 4 Sensor 3 
                break;
            case 0x10: // O2 Sensor Monitor Bank 4 Sensor 4 
                break;
            }
            break;
        case 0x02:
            switch (obd2_req.VS_OBD.PIDL) {
            case 0x01: // O2 Sensor Monitor Bank 1 Sensor 1 
                break;
            case 0x02: // O2 Sensor Monitor Bank 1 Sensor 2 
                break;
            case 0x03: // O2 Sensor Monitor Bank 1 Sensor 3 
                break;
            case 0x04: // O2 Sensor Monitor Bank 1 Sensor 4 
                break;
            case 0x05: // O2 Sensor Monitor Bank 2 Sensor 1 
                break;
            case 0x06: // O2 Sensor Monitor Bank 2 Sensor 2 
                break;
            case 0x07: // O2 Sensor Monitor Bank 2 Sensor 3 
                break;
            case 0x08: // O2 Sensor Monitor Bank 2 Sensor 4 
                break;
            case 0x09: // O2 Sensor Monitor Bank 3 Sensor 1 
                break;
            case 0x0A: // O2 Sensor Monitor Bank 3 Sensor 2 
                break;
            case 0x0B: // O2 Sensor Monitor Bank 3 Sensor 3 
                break;
            case 0x0C: // O2 Sensor Monitor Bank 3 Sensor 4 
                break;
            case 0x0D: // O2 Sensor Monitor Bank 4 Sensor 1 
                break;
            case 0x0E: // O2 Sensor Monitor Bank 4 Sensor 2 
                break;
            case 0x0F: // O2 Sensor Monitor Bank 4 Sensor 3 
                break;
            case 0x10: // O2 Sensor Monitor Bank 4 Sensor 4 
                break;
            }
            break;
        }
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE6 Test result (CAN only oxygen sensor monitoring)
 * ---------------------------------------------------------------------------------------- */
int obd2_mode6(int len)
{
    if (len == 2) { // Standard requirements 
        len = 1;
        obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE7 Obtain unknown diagnostic trouble code information
 * ---------------------------------------------------------------------------------------- */
int obd2_mode7(int len)
{
    if (len == 2) { // Standard requirements 
        len = 1;
        obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE8 On-board system support
 * ---------------------------------------------------------------------------------------- */
int obd2_mode8(int len)
{
    if (len == 2) { // Standard requirements 
        len = 1;
        obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE9 Processing
 * ---------------------------------------------------------------------------------------- */
int obd2_mode9(int len)
{
    if (len >= 2) { // Standard requirements 
        len = 2;
        obd2_ret.SAE_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
        obd2_ret.SAE_ECU.PID  = obd2_req.SAE_OBD.PID;
        switch (obd2_req.SAE_OBD.PID) {
                   //------------------------------------------------------ 
        case 0x00: // Mode 9 supported PIDs (01 to 20)
                   //------------------------------------------------------ 
            len                    += 4;
            obd2_ret.SAE_ECU.VAL[0] = 0x00;
            obd2_ret.SAE_ECU.VAL[1] = 0xC0;
            obd2_ret.SAE_ECU.VAL[2] = 0x00;
            obd2_ret.SAE_ECU.VAL[3] = 0x00;
            break;
        case 0x01: /* VIN Message Count in PID 02. Only for ISO 9141-2, ISO 14230-4
                    * and SAE J1850.*/
            break;
        case 0x02: // Vehicle Identification Number (VIN) 
            break;
        case 0x03: /* Calibration ID message count for PID 04. Only for ISO 9141-2,
                    * ISO 14230-4 and SAE J1850.*/
            break;
        case 0x04: // Calibration ID 
            break;
        case 0x05: /* Calibration verification numbers (CVN) message count for PID
                    * 06. Only for ISO 9141-2, ISO 14230-4 and SAE J1850.*/
            break;
        case 0x06: // Calibration Verification Numbers (CVN) 
            break;
        case 0x07: /* In-use performance tracking message count for PID 08 and 0B.
                    * Only for ISO 9141-2, ISO 14230-4 and SAE J1850.*/
            break;
        case 0x08: // In-use performance tracking for spark ignition vehicles 
            len += 4;
            switch (obd2_ret_counter) {
            case 0: // OBDCOND, IGNCNTR 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 1: // HCCATCOMP, HCCATCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 2: // NCATCOMP, NCATCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 3: // NADSCOMP, NADSCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 4: // PMCOMP, PMCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 5: // EGSCOMP, EGSCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 6: // EGRCOMP, EGRCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 7: // BPCOMP, BPCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 8: // FUELCOMP, FUELCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            }
            obd2_ret_counter++;
            if (obd2_ret_counter >= 9) {
                obd2_ret_counter = 0;
            }
            break;
        case 0x09: // ECU name message count for PID 0A 
            len                    += 1;
            obd2_ret.SAE_ECU.VAL[0] = 5; // (A) ECU name character length 
            break;
        case 0x0A: // ECU name 
            len += 5;
            switch (SELECT_ECU_UNIT) {
            case 0: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'P';
                obd2_ret.SAE_ECU.VAL[1] = 'O';
                obd2_ret.SAE_ECU.VAL[2] = 'W';
                obd2_ret.SAE_ECU.VAL[3] = 'E';
                break;
            case 1: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'C';
                obd2_ret.SAE_ECU.VAL[1] = 'H';
                obd2_ret.SAE_ECU.VAL[2] = 'A';
                obd2_ret.SAE_ECU.VAL[3] = 'S';
                break;
            case 2: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'B';
                obd2_ret.SAE_ECU.VAL[1] = 'O';
                obd2_ret.SAE_ECU.VAL[2] = 'D';
                obd2_ret.SAE_ECU.VAL[3] = 'Y';
                break;
            case 3: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'E';
                obd2_ret.SAE_ECU.VAL[1] = 'C';
                obd2_ret.SAE_ECU.VAL[2] = 'U';
                obd2_ret.SAE_ECU.VAL[3] = '3';
                break;
            case 4: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'E';
                obd2_ret.SAE_ECU.VAL[1] = 'C';
                obd2_ret.SAE_ECU.VAL[2] = 'U';
                obd2_ret.SAE_ECU.VAL[3] = '4';
                break;
            case 5: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'E';
                obd2_ret.SAE_ECU.VAL[1] = 'C';
                obd2_ret.SAE_ECU.VAL[2] = 'U';
                obd2_ret.SAE_ECU.VAL[3] = '5';
                break;
            case 6: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'E';
                obd2_ret.SAE_ECU.VAL[1] = 'C';
                obd2_ret.SAE_ECU.VAL[2] = 'U';
                obd2_ret.SAE_ECU.VAL[3] = '6';
                break;
            case 7: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = 'C';
                obd2_ret.SAE_ECU.VAL[1] = 'G';
                obd2_ret.SAE_ECU.VAL[2] = 'W';
                obd2_ret.SAE_ECU.VAL[3] = '1';
                break;
            default: // (A) ECU name character 
                obd2_ret.SAE_ECU.VAL[0] = '?';
                obd2_ret.SAE_ECU.VAL[1] = '?';
                obd2_ret.SAE_ECU.VAL[2] = '?';
                obd2_ret.SAE_ECU.VAL[3] = '?';
                break;
            }
            break;
        case 0x0B: // In-use performance tracking for compression ignition vehicles 
            len += 4;
            switch (obd2_ret_counter) {
            case 0: // OBDCOND, IGNCNTR 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 1: // HCCATCOMP, HCCATCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 2: // NCATCOMP, NCATCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 3: // NADSCOMP, NADSCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 4: // PMCOMP, PMCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 5: // EGSCOMP, EGSCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 6: // EGRCOMP, EGRCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 7: // BPCOMP, BPCOND 
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            case 8: // FUELCOMP, FUELCOND 
            default:
                obd2_ret.SAE_ECU.VAL[0] = 0x00;
                obd2_ret.SAE_ECU.VAL[1] = 0x00;
                obd2_ret.SAE_ECU.VAL[2] = 0x00;
                obd2_ret.SAE_ECU.VAL[3] = 0x00;
                break;
            }
            obd2_ret_counter++;
            if (obd2_ret_counter >= 9) {
                obd2_ret_counter = 0;
            }
            break;
        }
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * MODE10 Persistent DTC information
 * ---------------------------------------------------------------------------------------- */
int obd2_modeA(int len)
{
    if (len == 2) { // Standard requirements 
        len = 1;
        obd2_ret.MD3_ECU.MODE = obd2_req.SAE_OBD.MODE + 0x40;
    } else {
        return 0;
    }
    return len;
}

/* ----------------------------------------------------------------------------------------
 * OBD2 processing
 * ---------------------------------------------------------------------------------------- */
int obd2_job(unsigned char *msg, int len,
             unsigned char *res) 
{
    memset(&obd2_ret, 0x00, sizeof(obd2_ret));
    memset(&obd2_req, 0x00, sizeof(obd2_req));
    memcpy(&obd2_req.BYTE[0], msg, len);
    switch (obd2_req.SAE_OBD.MODE) {
    case SHOW_CURRENT_DATA: // Present value 
        len = obd2_mode1(len);
        break;
    case SHOW_FREEZE_FDATA:    // Stop frame 
        len = obd2_mode2(len);
        break;
    case SHOW_STORED_DTC:      // Get saved DTC 
        len = obd2_mode3(len);
        break;
    case CLEAR_STORED_DTC:     // Delete saved DTC 
        len = obd2_mode4(len);
        break;
    case TEST_RESULT_NON_CAN:  // Exhaust monitoring (NON-CAN) 
        len = obd2_mode5(len);
        break;
    case TEST_RESULT_ONLY_CAN: // Other monitors, exhaust monitoring (CAN) 
        len = obd2_mode6(len);
        break;
    case SHOW_PENDING_DTC:     // Last DTC information 
        len = obd2_mode7(len);
        break;
    case CTRL_OPERATION_SYS:   // Control system operation 
        len = obd2_mode8(len);
        break;
    case REQUEST_VEHICLE_INFO: // Vehicle information 
        len = obd2_mode9(len);
        break;
    case PERMANENT_DTC: // Permanent DTC information 
        len = obd2_modeA(len);
        break;
    default: // Unsupported mode 
        len = 3;
        obd2_ret.NOT_ECU.X7F  = 0x7F;
        obd2_ret.NOT_ECU.MODE = obd2_req.SAE_OBD.MODE;
        obd2_ret.NOT_ECU.X31  = 0x31;
        break;
    }
    if (len > 0) { // Reply 
        memcpy(&res[0], &obd2_ret.BYTE[0], len);
        return len;
    }
    return 0;
}

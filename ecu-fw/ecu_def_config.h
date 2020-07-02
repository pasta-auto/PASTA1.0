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
 * $RCSfile: ecu_def_powertrain.h,v $
 * $Revision: 1.00 $
 * $Date: 2017/01/15 11:05:00 $
 *
 * Copyright (c) 2017 LandF Corporation.
 *
 * History:
 */

#ifndef     __ECU_DEFAULT_SETTING__
#define __ECU_DEFAULT_SETTING__

/* ---------------------------------------------------------------------------------------
 * Competition ID setting between driving simulator and power train
 *
 * When the powertrain ECU receives the ID set to conflict, it changes the ID transmission / reception direction thereafter.
 * To the LCD, a mark is added after the frame data of the corresponding ID and passed.
 * --------------------------------------------------------------------------------------- */
#define DS_X_POWERTRAIN_ID      0x043       // Engine RPM and speed 

/* ---------------------------------------------------------------------------------------
 * CGW CAN-ID mode
 * --------------------------------------------------------------------------------------- */
#define CGW_ALL_ID_PASS 0 // 0 = Transfer all, 1 = Transfer only in specified direction 

/* ---------------------------------------------------------------------------------------
 * Routing map initial value
 * --------------------------------------------------------------------------------------- */
#ifdef      __LFY_RX63N__
const unsigned char CH_MAP_PAT[] = { 0x00, 0x22, 0x22, 0x22, 0x22 }; // LFY-RX63N is valid only for CAN1 
#define CH_MAP_CGW  0x22
#define CH_MAP_SYS  0x22
#define CH_MAP_RECV 0x20
#define CH_MAP_SEND 0x02
#else // ifdef      __LFY_RX63N__
const unsigned char CH_MAP_PAT[] = { 0x00, 0x11, 0x33, 0x77, 0xFF }; // CAN2ECU is a designated channel 
#define CH_MAP_CGW  0x7F
#define CH_MAP_SYS  0xFF
#define CH_MAP_RECV 0x10
#define CH_MAP_SEND 0x01
#endif // ifdef      __LFY_RX63N__

/* ---------------------------------------------------------------------------------------
 * Frame initial value
 * --------------------------------------------------------------------------------------- */
typedef struct  __def_frame_data__ {
    int ID;  // ID number 
    int PAT; // Pattern number 
    unsigned char DAT[8]; // Data 
}   DEF_FRAME_DATA;
const DEF_FRAME_DATA DEF_FRAME_BUFFER[] = {
    {0x024, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00}},   // 
    {0x146, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x46, 0x00, 0x00}},   // 
    {0x15A, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x5A, 0x00, 0x00}},   // 
    {0x039, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x00, 0x00}},   // 
    {0x16F, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x6F, 0x00, 0x00}},   // 
    {0x043, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00}},   // 
    {0x183, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x83, 0x00, 0x00}},   // 
    {0x3BD, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xBD, 0x00, 0x00}},   // 
    {0x18D, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x8D, 0x00, 0x00}},   // 
    {0x062, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x00, 0x00}},   // 
    {0x198, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x98, 0x00, 0x00}},   // 
    {0x077, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00}},   // 
    {0x19A, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0x9A, 0x00, 0x00}},   // 
    {0x3D4, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xD4, 0x00, 0x00}},   // 
    {0x3DE, 0, {0x00, 0x00, 0x00, 0x00, 0x03, 0xDE, 0x00, 0x00}},   // 
    {0x1D3, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xD3, 0x00, 0x00}},   // 
    {0x482, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x82, 0x00, 0x00}},   // 

    {0x01A, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x00}},   // 
    {0x02F, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00}},   // 
    {0x058, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x00, 0x00}},   // 
    {0x06D, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x00}},   // 
    {0x083, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0x00, 0x00}},   // 
    {0x098, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x00, 0x00}},   // 
    {0x1A7, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xA7, 0x00, 0x00}},   // 
    {0x1B1, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xB1, 0x00, 0x00}},   // 
    {0x1B8, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xB8, 0x00, 0x00}},   // 
    {0x1C9, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xC9, 0x00, 0x00}},   // 
    {0x25C, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x5C, 0x00, 0x00}},   // 
    {0x271, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x71, 0x00, 0x00}},   // 
    {0x286, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x86, 0x00, 0x00}},   // 
    {0x29C, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x9C, 0x00, 0x00}},   // 
    {0x2B1, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0xB1, 0x00, 0x00}},   // 

    {0x08D, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x00, 0x00}},   // 
    {0x0A2, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0x00, 0x00}},   // 
    {0x1BB, 0, {0x00, 0x00, 0x00, 0x00, 0x01, 0xBB, 0x00, 0x00}},   // 
    {0x266, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x66, 0x00, 0x00}},   // 
    {0x27B, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x7B, 0x00, 0x00}},   // 
    {0x290, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0x90, 0x00, 0x00}},   // 
    {0x420, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x00, 0x00}},   // 
    {0x2A6, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0xA6, 0x00, 0x00}},   // 
    {0x2BB, 0, {0x00, 0x00, 0x00, 0x00, 0x02, 0xBB, 0x00, 0x00}},   // 
    {0x0B4, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0xB4, 0x00, 0x00}},   // 
    {0x457, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x57, 0x00, 0x00}},   // 
    {0x461, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x61, 0x00, 0x00}},   // 
    {0x46C, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x6C, 0x00, 0x00}},   // 
    {0x477, 0, {0x00, 0x00, 0x00, 0x00, 0x04, 0x77, 0x00, 0x00}},   // 
    {   -1, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},   // 
};

/* ---------------------------------------------------------------------------------------
 * defset_rootmap
 * 
 * Outline 
 *     Map initialization
 *
 * Argument
 *     None
 *
 * Description
 *     Initial setting of routing map
 *
 * Return
 *     None
 *
 * Note
 *     The initial value accepts and forwards all messages.
 *
 * --------------------------------------------------------------------------------------- */
#define cgw_rootmap(a,b)    rout_map.ID[(a)].BYTE=(b)
void defset_rootmap(void)
{
    int i;
    memset(&rout_map, 0, sizeof(rout_map));
    memset(can_to_exio, -1, sizeof(can_to_exio));
    if (SELECT_ECU_UNIT == ECU_UNIT_CGW) { // ECU_UNIT_POWERTRAIN 
#if (CGW_ALL_ID_PASS==0)
        memset(&rout_map, CH_MAP_SYS, sizeof(rout_map));
#else
        cgw_rootmap(0x024, PT_TO_CS); //P Brake output 
        cgw_rootmap(0x039, PT_TO_CS); //P Throttle position 
        cgw_rootmap(0x043, PT_TO_CS); //P Engine RPM and speed 
        cgw_rootmap(0x062, PT_TO_CS); //P Power steering output 
        cgw_rootmap(0x077, PT_TO_CS); //P Shift position 
        cgw_rootmap(0x146, PT_TO_CS); //P Brake oil amount 
        cgw_rootmap(0x150, PT_TO_CS); //P Brake wear warning / ice burn warning 
        cgw_rootmap(0x15A, PT_TO_CS); //P Anti-lock brake operation 
        cgw_rootmap(0x164, PT_TO_CS); //P Brake pad temperature and tire temperature 
        cgw_rootmap(0x16F, PT_TO_CS); //P Throttle adjustment 
        cgw_rootmap(0x179, PT_TO_CS); //P Fuel consumption rate and mixture ratio 
        cgw_rootmap(0x183, PT_TO_CS); //P Engine cooling water temperature 
        cgw_rootmap(0x18D, PT_TO_CS); //P Engine failure 
        cgw_rootmap(0x198, PT_TO_CS); //P Power steering failure 
        cgw_rootmap(0x19A, PT_TO_CS); //P Engine starter drive 
        cgw_rootmap(0x1A2, PT_TO_CS); //P Activate skid prevention 
        cgw_rootmap(0x1AD, PT_TO_CS); //P Mission failure 
        cgw_rootmap(0x1D3, PT_TO_CS); //P Side brake operation status 
        cgw_rootmap(0x39E, PT_TO_CS); //P Regenerative braking power generation 
        cgw_rootmap(0x3A9, PT_TO_CS); //P Outside air temperature / exhaust air temperature 
        cgw_rootmap(0x3B3, PT_TO_CS); //P Hazardous exhaust gas concentration / particulate matter concentration 
        cgw_rootmap(0x3BD, PT_TO_CS); //P Engine oil amount 
        cgw_rootmap(0x3C7, PT_TO_CS); //P Poor ignition / abnormal ignition timing 
        cgw_rootmap(0x3D4, PT_TO_CS); //P Engine starter failure 
        cgw_rootmap(0x3DE, PT_TO_CS); //P Battery alarm 
        cgw_rootmap(0x42B, PT_TO_CS); //P Side brake warning 
        cgw_rootmap(0x482, PT_TO_CS); //P Eco-drive judgment 
        // ECU_UNIT_CHASSIS 
        cgw_rootmap(0x01A, CS_TO_AL); //C Brake operation amount 
        cgw_rootmap(0x02F, CS_TO_AL); //C Accelerator operation amount 
        cgw_rootmap(0x058, CS_TO_AL); //C Handle operation position 
        cgw_rootmap(0x06D, CS_TO_AL); //C Shift position switch 
        cgw_rootmap(0x083, CS_TO_AL); //C Blinker left / right hazard switch 
        cgw_rootmap(0x098, CS_TO_AL); //C Horn switch 
        cgw_rootmap(0x1A7, CS_TO_AL); //C Position headlight high beam switch 
        cgw_rootmap(0x1B1, CS_TO_AL); //C Passing switch 
        cgw_rootmap(0x1B8, CS_TO_AL); //C Engine start button 
        cgw_rootmap(0x1C9, CS_TO_AL); //C Side brake 
        cgw_rootmap(0x25C, CS_TO_AL); //C Front wiper, intermittent, LOW, HIGH, washer switch 
        cgw_rootmap(0x271, CS_TO_AL); //C Rear wiper / washer switch 
        cgw_rootmap(0x286, CS_TO_AL); //C Door lock switch / unlock switch 
        cgw_rootmap(0x29C, CS_TO_AL); //C Right door / window lift switch 
        cgw_rootmap(0x2B1, CS_TO_AL); //C Left door / window lift switch 
        // ECU_UNIT_BODY 
        cgw_rootmap(0x08D, BD_TO_CS); //B Blinker left and right lighting state 
        cgw_rootmap(0x0A2, BD_TO_CS); //B Horn sound 
        cgw_rootmap(0x0B4, BD_TO_CS); //B Airbag activation switch 
        cgw_rootmap(0x1BB, BD_TO_CS); //B Position headlight high beam lighting state 
        cgw_rootmap(0x266, BD_TO_CS); //B Front wiper / intermittent / LOW / HIGH / washer operation status 
        cgw_rootmap(0x27B, BD_TO_CS); //B Rear wiper / washer operating state 
        cgw_rootmap(0x290, BD_TO_CS); //B Door open / closed / locked 
        cgw_rootmap(0x2A6, BD_TO_CS); //B Right door / window position / limit switch status 
        cgw_rootmap(0x2BB, BD_TO_CS); //B Left door / window position / limit switch status 
        cgw_rootmap(0x3E9, BD_TO_CS); //B Turn signal ball out alarm 
        cgw_rootmap(0x3F4, BD_TO_CS); //B Horn breakdown 
        cgw_rootmap(0x3FF, BD_TO_CS); //B Position / Headlight / High beam burnout / Bulb control failure 
        cgw_rootmap(0x40A, BD_TO_CS); //B Front wiper / intermittent / LOW / HIGH / washer motor / pump failure 
        cgw_rootmap(0x415, BD_TO_CS); //B Rear wiper / washer motor / pump failure 
        cgw_rootmap(0x420, BD_TO_CS); //B Door lock drive failure 
        cgw_rootmap(0x436, BD_TO_CS); //B Right door / window motor failure 
        cgw_rootmap(0x441, BD_TO_CS); //B Left door / window motor failure 
        cgw_rootmap(0x44C, BD_TO_CS); //B Airbag failure 
        cgw_rootmap(0x457, BD_TO_CS); //B Seat belt sensor 
        cgw_rootmap(0x461, BD_TO_CS); //B Seat belt alarm 
        cgw_rootmap(0x46C, BD_TO_CS); //B Bonnet open / close switch 
        cgw_rootmap(0x477, BD_TO_CS); //B Trunk open / close switch 

        // Transport ID 
        cgw_rootmap(0x7DF, AL_TO_AL); //Diag port reception, all transmission 
        cgw_rootmap(0x7E3, EX_TO_AL); //Diag port reception, all transmission 
        cgw_rootmap(0x7EB, AL_TO_EX); //Diag port transmission, all reception 
        cgw_rootmap(0x7E0 + ECU_UNIT_POWERTRAIN, EX_TO_PT);
        cgw_rootmap(0x7E0 + ECU_UNIT_CHASSIS,    EX_TO_CS);
        cgw_rootmap(0x7E0 + ECU_UNIT_BODY,       EX_TO_BD);
        cgw_rootmap(0x7E4 + ECU_UNIT_POWERTRAIN, EX_TO_PT);
        cgw_rootmap(0x7E4 + ECU_UNIT_CHASSIS,    EX_TO_CS);
        cgw_rootmap(0x7E4 + ECU_UNIT_BODY,       EX_TO_BD);
        cgw_rootmap(0x7E8 + ECU_UNIT_POWERTRAIN, PT_TO_EX);
        cgw_rootmap(0x7E8 + ECU_UNIT_CHASSIS,    CS_TO_EX);
        cgw_rootmap(0x7E8 + ECU_UNIT_BODY,       BD_TO_EX);
        cgw_rootmap(0x7EC + ECU_UNIT_POWERTRAIN, PT_TO_EX);
        cgw_rootmap(0x7EC + ECU_UNIT_CHASSIS,    CS_TO_EX);
        cgw_rootmap(0x7EC + ECU_UNIT_BODY,       BD_TO_EX);
#endif // if (CGW_ALL_ID_PASS==0)
    } else { // ECU sets transmission and reception individually 
        rout_map.ID[0x7DF].BYTE                     = CH_MAP_RECV;  // Broadcast ID received by all ECUs 
        rout_map.ID[(0x7E0 + SELECT_ECU_UNIT)].BYTE = CH_MAP_RECV;  // TP reception ID of individual ECU 
        rout_map.ID[(0x7E8 + SELECT_ECU_UNIT)].BYTE = CH_MAP_SEND;  // TP transmission ID of individual ECU 
    }
}

/* ---------------------------------------------------------------------------------------
 * add_cyceve_list
 * 
 * Outline 
 *    Period / event initial value
 *
 * Argument
 *     None
 *
 * Description
 *     Initial setting of cyclic message and event message
 *
 * Return value
 *     None
 *
 * Note
 *     Up to 255 management IDs can be defined
 *
 * --------------------------------------------------------------------------------------- */
int add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt);
void defset_confecu(void)
{
    // Zero initialization 
    memset(&conf_ecu, 0, sizeof(conf_ecu));         // Initialization of cycle / event / remote management 
    conf_ecu.TOP = -1;
    /*
     *  Registration function
     *  int add_cyceve_list(int rtr, int id, int dlc, int enb, int rep, int time, int cnt)
     *
     * Argument
     *  int rtr   Remote frame specification      0:Data / 1:Remote
     *  int id    CAN message number              0x000 to 0x7FF(0 to 2047)
     *  int dlc   Data byte length                0 to 8
     *  int enb   Processing permission flag      0:Disable / 1:Enable
     *  int rep   Periodic frame specification    0:Event / 1:Period
     *  int time  Period time or delay time (ms)  0 to 65535
     *  int cnt   Delay increase time (ms)        0 to 65535
     */
    //P=Powertrain / B=Body / C=Chassis 
    if (SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN) {
        add_cyceve_list(0, 0x024, 8, 1, 1, 20, 0); //P Brake output 
        add_cyceve_list(0, 0x039, 8, 1, 1, 20, 1); //P Throttle position 
        add_cyceve_list(0, 0x043, 8, 1, 1, 20, 2); //P Engine RPM and speed 
        add_cyceve_list(0, 0x062, 8, 1, 1, 20, 3); //P Power steering output 
        add_cyceve_list(0, 0x077, 8, 1, 1, 20, 4); //P Shift position 
        add_cyceve_list(0, 0x146, 8, 1, 1, 50, 5); //P Brake oil level 
        add_cyceve_list(0, 0x15A, 8, 1, 1, 50, 7); //P Anti-lock brake operation 
        add_cyceve_list(0, 0x16F, 8, 1, 1, 50, 9); //P Throttle adjustment 
        add_cyceve_list(0, 0x183, 8, 1, 1, 50,11); //P Engine coolant temperature 
        add_cyceve_list(0, 0x18D, 8, 1, 1, 50,12); //P Engine failure 
        add_cyceve_list(0, 0x198, 8, 1, 1, 50,13); //P Power steering failure 
        add_cyceve_list(0, 0x19A, 8, 1, 1, 50,14); //P Engine starter drive 
        add_cyceve_list(0, 0x1D3, 8, 1, 1, 50,17); //P Side brake operation status 
        add_cyceve_list(0, 0x3BD, 8, 1, 1,500,21); //P Engine oil 
        add_cyceve_list(0, 0x3D4, 8, 1, 1,500,23); //P Engine starter failure 
        add_cyceve_list(0, 0x3DE, 8, 1, 1,500,24); //P Battery alarm 
        add_cyceve_list(0, 0x482, 8, 1, 1,500,26); //P Eco-drive judgment 
    } else if (SELECT_ECU_UNIT == ECU_UNIT_CHASSIS) {
        add_cyceve_list(0, 0x01A, 8, 1, 1, 20, 0); //C Brake operation amount 
        add_cyceve_list(0, 0x02F, 8, 1, 1, 20, 1); //C A operation amount 
        add_cyceve_list(0, 0x058, 8, 1, 1, 20, 2); //C Handle operation position 
        add_cyceve_list(0, 0x06D, 8, 1, 1, 20, 3); //C Shift position switch 
        add_cyceve_list(0, 0x083, 8, 1, 1, 20, 4); //C Blinker left / right / hazard switch 
        add_cyceve_list(0, 0x098, 8, 1, 1, 20, 5); //C Horn switch 
        add_cyceve_list(0, 0x1A7, 8, 1, 1, 50, 6); //C Position headlight high beam switch 
        add_cyceve_list(0, 0x1B1, 8, 1, 1, 50, 7); //C Passing switch 
        add_cyceve_list(0, 0x1B8, 8, 1, 1, 50, 8); //C Engine start button 
        add_cyceve_list(0, 0x1C9, 8, 1, 1, 50, 9); //C Side brake 
        add_cyceve_list(0, 0x25C, 8, 1, 1,100,10); //C Front wiper / intermittent / LOW / HIGH / washer switch 
        add_cyceve_list(0, 0x271, 8, 1, 1,100,11); //C Rear wiper washer switch 
        add_cyceve_list(0, 0x286, 8, 1, 1,100,12); //C Door lock switch / unlock switch 
        add_cyceve_list(0, 0x29C, 8, 1, 1,100,13); //C Right door window lift switch 
        add_cyceve_list(0, 0x2B1, 8, 1, 1,100,14); //C Left door window lift switch 
    } else if (SELECT_ECU_UNIT == ECU_UNIT_BODY) {
        add_cyceve_list(0, 0x08D, 8, 1, 1, 20, 0); //B Blinker left and right lighting state 
        add_cyceve_list(0, 0x0A2, 8, 1, 1, 20, 1); //B Horn sound 
        add_cyceve_list(0, 0x0B4, 8, 1, 1, 20, 2); //B Airbag activation switch 
        add_cyceve_list(0, 0x1BB, 8, 1, 1, 50, 3); //B Position, headlight and high beam lighting state 
        add_cyceve_list(0, 0x266, 8, 1, 1,100, 4); //B Front wiper / intermittent / LOW / HIGH / washer operating state 
        add_cyceve_list(0, 0x27B, 8, 1, 1,100, 5); //B Rear wiper/washer operating state 
        add_cyceve_list(0, 0x290, 8, 1, 1,100, 6); //B Door open / closed / locked state 
        add_cyceve_list(0, 0x2A6, 8, 1, 1,100, 7); //B Right door/window position / limit switch status 
        add_cyceve_list(0, 0x2BB, 8, 1, 1,100, 8); //B Left door/window position / limit switch status 
        add_cyceve_list(0, 0x420, 8, 1, 1,500,14); //B Door lock drive failure 
        add_cyceve_list(0, 0x457, 8, 1, 1,500,18); //B Seat belt sensor 
        add_cyceve_list(0, 0x461, 8, 1, 1,500,19); //B Seat belt alarm 
        add_cyceve_list(0, 0x46C, 8, 1, 1,500,20); //B Bonnet open/close switch 
        add_cyceve_list(0, 0x477, 8, 1, 1,500,21); //B Trunk open/close switch 
    } else if(SELECT_ECU_UNIT == ECU_UNIT_CGW) {
        // add_cyceve_list(0, 0x7FD, 8, 1, 1, 1000, 0); //C Notify every second
    }
}

/* ---------------------------------------------------------------------------------------
 * defset_extlist
 * 
 * Outline 
 *  External I/O definition initial value
 *
 * Argument
 *     None
 *
 * Description
 *     External I/O initial value setting
 *
 * Return
 *     None
 *
 * Note
 *     By default, I/O is performed by the LF74 and there is no port directly referenced by the ECU
 *
 * --------------------------------------------------------------------------------------- */
#define DEF_IOPAT_XX    0   // Pattern undefined 
// Checklist change pattern definition 
const unsigned char DEF_IOPAT_00[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_01[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_02[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_03[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_04[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_05[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_06[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
const unsigned char DEF_IOPAT_07[24]    = {
//     0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };

int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat);
void defset_extlist(void)
{
    memset(ext_list, 0, sizeof(ext_list));  // Initialization of ECU I/O checklist 
    ext_list_count = 0;                     // Checklist reset 
    /*
     *  Registration function
     *  int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat)
     *
     * Argument
     *  int           id   Frame ID number              0 to 2047(000 to 7FF)
     *  int           mode I/O processing mode setting  0:Disable / 1:DI / 2:DO / 3:AI / 4:AO / 5:2bit value / 6:3bit value / 7:4bit value
     *  int           neg  Data inversion specification 0:Normal / 1:Invert
     *  int           size Access size                  0:BIT / 1 to 7:nBYTE
     *  int           bpos Byte position                0 to 7
     *  int           dlc  Data byte length             0 to 8
     *  int           nom  Applicable port number       0 to 33
     *  int           msk  Mask pattern                 00 to FF
     *  unsigned char *pat Pattern data                 24bytes (0=do not use)
     */
}

/* ---------------------------------------------------------------------------------------
 * defset_extlist_ex
 * 
 * Outline 
 *     External I/O definition initial value setting via communication
 *
 * Argument
 *     None
 * 
 * Return
 *     None
 *
 * Note
 *     By default, I/O is performed by the LF74 and there is no port directly referenced by the ECU
 *
 * --------------------------------------------------------------------------------------- */
void defset_extlist_ex(void)
{
    memset(ext_list, 0, sizeof(ext_list));  // Initialization of ECU I/O checklist 
    ext_list_count = 0;                     // Checklist reset 
    /*
     *  Registration function
     *  int add_extern_io(int id, int mode, int neg, int size, int bpos, int dlc, int nom, int msk, unsigned char *pat)
     *
     * Argument
     *  int           id   Frame ID number               0 to 2047(000 to 7FF)
     *  int           mode I/O processing mode setting   0:bit / 1:byte / 2:word / 3:long  Output 4:bit / 5:byte / 6:word / 7:long
     *  int           neg  Data inversion specification [0]Disable
     *  int           size Access size                  [0]Disable
     *  int           bpos Byte position                 0 to 7 (Normal=0)
     *  int           dlc  Data byte length             [0]Disable
     *  int           nom  Applicable port number        0 to 63
     *  int           msk  Mask pattern                  00 to FF
     *  unsigned char *pat Pattern data                 [0]=do not use
     */
    if (SELECT_ECU_UNIT == ECU_UNIT_POWERTRAIN) {           // Powertrain outputs only information from chassis 
        add_extern_io(0x01A, 6, 0, 0, 0, 0, 0, 1, 0); //C Brake operation amount 
        add_extern_io(0x02F, 6, 0, 0, 0, 0, 1, 1, 0); //C accelerator operation amount 
        add_extern_io(0x058, 6, 0, 0, 0, 0, 2, 1, 0); //C handle operation position 
        add_extern_io(0x06D, 5, 0, 0, 0, 0, 3, 1, 0); //C shift position switch 
        add_extern_io(0x1B8, 5, 0, 0, 0, 0, 8, 1, 0); //C engine start button 
        add_extern_io(0x1C9, 5, 0, 0, 0, 0, 9, 1, 0); //C side brake 
        // Powertrain input information 
        add_extern_io(0x024, 2, 0, 0, 0, 0,15, 1, 0); //P Brake output 
        add_extern_io(0x039, 2, 0, 0, 0, 0,16, 1, 0); //P throttle position 
        add_extern_io(0x043, 3, 0, 0, 0, 0,17, 1, 0); //P Engine RPM / speed 
        add_extern_io(0x062, 3, 0, 0, 0, 0,18, 1, 0); //P Power steering output / torque 
        add_extern_io(0x077, 1, 0, 0, 0, 0,19, 1, 0); //P shift position 
        add_extern_io(0x146, 1, 0, 0, 0, 0,20, 1, 0); //P Brake oil amount 
        add_extern_io(0x15A, 1, 0, 0, 0, 0,22, 1, 0); //P Anti-lock brake operation 
        add_extern_io(0x16F, 2, 0, 0, 0, 0,24, 1, 0); //P Throttle adjustment 
        add_extern_io(0x183, 1, 0, 0, 0, 0,26, 1, 0); //P Engine coolant temperature 
        add_extern_io(0x18D, 1, 0, 0, 0, 0,27, 1, 0); //P engine failure 
        add_extern_io(0x198, 1, 0, 0, 0, 0,28, 1, 0); //P Power steering failure 
        add_extern_io(0x19A, 1, 0, 0, 0, 0,29, 1, 0); //P Engine starter drive 
        add_extern_io(0x1D3, 1, 0, 0, 0, 0,32, 1, 0); //P side brake operation status 
        add_extern_io(0x3BD, 1, 0, 0, 0, 0,36, 1, 0); //P engine oil amount 
        add_extern_io(0x3D4, 1, 0, 0, 0, 0,38, 1, 0); //P fuel remaining 
        add_extern_io(0x3DE, 1, 0, 0, 0, 0,39, 1, 0); //P Battery alarm 
        add_extern_io(0x482, 1, 0, 0, 0, 0,41, 1, 0); //P Eco-drive judgment 
    } else if (SELECT_ECU_UNIT == ECU_UNIT_CHASSIS) {       //  Chassis input 
        add_extern_io(0x01A, 2, 0, 0, 0, 0, 0, 1, 0); //C Brake operation amount 
        add_extern_io(0x02F, 2, 0, 0, 0, 0, 1, 1, 0); //C accelerator operation amount 
        add_extern_io(0x058, 2, 0, 0, 0, 0, 2, 1, 0); //C handle operation position 
        add_extern_io(0x06D, 1, 0, 0, 0, 0, 3, 1, 0); //C shift position switch 
        add_extern_io(0x083, 1, 0, 0, 0, 0, 4, 1, 0); //C blinker left / right / hazard switch 
        add_extern_io(0x098, 1, 0, 0, 0, 0, 5, 1, 0); //C horn switch 
        add_extern_io(0x1A7, 1, 0, 0, 0, 0, 6, 1, 0); //C position headlight high beam switch 
        add_extern_io(0x1B1, 1, 0, 0, 0, 0, 7, 1, 0); //C passing switch 
        add_extern_io(0x1B8, 1, 0, 0, 0, 0, 8, 1, 0); //C engine start button 
        add_extern_io(0x1C9, 1, 0, 0, 0, 0, 9, 1, 0); //C side brake 
        add_extern_io(0x25C, 2, 0, 0, 0, 0,10, 1, 0); //C front wiper / intermittent / LOW / HIGH / washer switch / intermittent timer 
        add_extern_io(0x271, 1, 0, 0, 0, 0,11, 1, 0); //C rear wiper / washer switch 
        add_extern_io(0x286, 1, 0, 0, 0, 0,12, 1, 0); //C Door lock switch / unlock switch 
        add_extern_io(0x29C, 1, 0, 0, 0, 0,13, 1, 0); //C Right door / window lift switch 
        add_extern_io(0x2B1, 1, 0, 0, 0, 0,14, 1, 0); //C left door / window up / down switch 
        /* Chassis outputs all information for transfer to instrument panel
         * Output of powertrain information*/
        add_extern_io(0x043, 7, 0, 0, 0, 0,17, 1, 0); //P engine RPM / speed 
        add_extern_io(0x062, 7, 0, 0, 0, 0,18, 1, 0); //P Power steering output / torque 
        add_extern_io(0x077, 5, 0, 0, 0, 0,19, 1, 0); //P shift position 
        add_extern_io(0x146, 5, 0, 0, 0, 0,20, 1, 0); //P Brake oil amount 
        add_extern_io(0x183, 5, 0, 0, 0, 0,26, 1, 0); //P Engine coolant temperature 
        add_extern_io(0x18D, 5, 0, 0, 0, 0,27, 1, 0); //P engine failure 
        add_extern_io(0x19A, 5, 0, 0, 0, 0,29, 1, 0); //P Engine starter drive 
        add_extern_io(0x1D3, 5, 0, 0, 0, 0,32, 1, 0); //P side brake operation status 
        add_extern_io(0x3BD, 5, 0, 0, 0, 0,36, 1, 0); //P engine oil 
        add_extern_io(0x3D4, 5, 0, 0, 0, 0,38, 1, 0); //P fuel remaining 
        add_extern_io(0x3DE, 5, 0, 0, 0, 0,39, 1, 0); //P Battery alarm 
        add_extern_io(0x482, 5, 0, 0, 0, 0,41, 1, 0); //P Eco-drive judgment 
        // Output of body information 
        add_extern_io(0x08D, 5, 0, 0, 0, 0,42, 1, 0); //B Blinker left and right lighting state 
        add_extern_io(0x0A2, 5, 0, 0, 0, 0,43, 1, 0); //B horn sound 
        add_extern_io(0x0B4, 5, 0, 0, 0, 0,44, 1, 0); //B airbag activation switch 
        add_extern_io(0x1BB, 5, 0, 0, 0, 0,45, 1, 0); //B position headlight high beam lighting state 
        add_extern_io(0x266, 6, 0, 0, 0, 0,46, 1, 0); //B Front wiper / Intermittent / LOW / HIGH / Washer operation status / Intermittent timer 
        add_extern_io(0x27B, 5, 0, 0, 0, 0,47, 1, 0); //B Rear wiper / washer operating state 
        add_extern_io(0x290, 5, 0, 0, 0, 0,48, 1, 0); //B Door open / closed / locked state 
        add_extern_io(0x461, 5, 0, 0, 0, 0,61, 1, 0); //B seat belt alarm 
        add_extern_io(0x46C, 5, 0, 0, 0, 0,62, 1, 0); //B Bonnet open / close switch 
        add_extern_io(0x477, 5, 0, 0, 0, 0,63, 1, 0); //B Trunk open / close switch 
    } else if (SELECT_ECU_UNIT == ECU_UNIT_BODY) {          //  The body outputs only information from the chassis 
        add_extern_io(0x01A, 6, 0, 0, 0, 0, 0, 1, 0); //C Brake operation amount 
        add_extern_io(0x083, 5, 0, 0, 0, 0, 4, 1, 0); //C blinker left / right / hazard switch 
        add_extern_io(0x098, 5, 0, 0, 0, 0, 5, 1, 0); //C horn switch 
        add_extern_io(0x1A7, 5, 0, 0, 0, 0, 6, 1, 0); //C position headlight high beam switch 
        add_extern_io(0x1B1, 5, 0, 0, 0, 0, 7, 1, 0); //C passing switch 
        add_extern_io(0x25C, 6, 0, 0, 0, 0,10, 1, 0); //C front wiper / intermittent / LOW / HIGH / washer switch / intermittent timer 
        add_extern_io(0x271, 5, 0, 0, 0, 0,11, 1, 0); //C Rear wiper / washer switch 
        add_extern_io(0x286, 5, 0, 0, 0, 0,12, 1, 0); //C Door lock switch / unlock switch 
        add_extern_io(0x29C, 5, 0, 0, 0, 0,13, 1, 0); //C Right door / window lift switch 
        add_extern_io(0x2B1, 5, 0, 0, 0, 0,14, 1, 0); //C Left door window up / down switch 
        // Body input 
        add_extern_io(0x08D, 1, 0, 0, 0, 0,42, 1, 0); //B Blinker left and right lighting state 
        add_extern_io(0x0A2, 1, 0, 0, 0, 0,43, 1, 0); //B horn sound 
        add_extern_io(0x0B4, 1, 0, 0, 0, 0,44, 1, 0); //B airbag activation switch 
        add_extern_io(0x1BB, 1, 0, 0, 0, 0,45, 1, 0); //B position headlight high beam lighting state 
        add_extern_io(0x266, 2, 0, 0, 0, 0,46, 1, 0); //B front wiper / intermittent / LOW / HIGH / washer operation status / intermittent timer 
        add_extern_io(0x27B, 1, 0, 0, 0, 0,47, 1, 0); //B Rear wiper / washer operating state 
        add_extern_io(0x290, 1, 0, 0, 0, 0,48, 1, 0); //B Door open / closed / locked state 
        add_extern_io(0x2A6, 2, 0, 0, 0, 0,49, 1, 0); //B Right door / window position / limit switch status 
        add_extern_io(0x2BB, 2, 0, 0, 0, 0,50, 1, 0); //B left door / window position / limit switch status 
        add_extern_io(0x420, 1, 0, 0, 0, 0,56, 1, 0); //B door lock drive failure 
        add_extern_io(0x457, 1, 0, 0, 0, 0,60, 1, 0); //B Seat belt sensor 
        add_extern_io(0x461, 1, 0, 0, 0, 0,61, 1, 0); //B seat belt alarm 
        add_extern_io(0x46C, 1, 0, 0, 0, 0,62, 1, 0); //B Bonnet open / close switch 
        add_extern_io(0x477, 1, 0, 0, 0, 0,63, 1, 0); //B Trunk open / close switch 
    } else if (SELECT_ECU_UNIT == ECU_UNIT_CGW) {
        // None 
    }
}

/* ---------------------------------------------------------------------------------------
 * set_frame_data
 * 
 * Outline 
 *     CAN frame data buffer initial value
 *
 * Argument
 *     None
 * 
 * Return
 *     None
 * --------------------------------------------------------------------------------------- */
void set_frame_data(int id, int dlc, unsigned char *dat);

void defset_framedat(void)
{
    int             i;
    DEF_FRAME_DATA *vp;

    for (i = 0; ; i++) {
        vp = &DEF_FRAME_BUFFER[i];
        if (vp->ID < 0) {
            break;
        }
        set_frame_data(vp->ID, 8, vp->DAT);
    }
}

#endif //__ECU_DEFAULT_SETTING__

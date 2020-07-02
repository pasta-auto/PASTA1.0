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

/*
 *
 * LFT-RX63N-ECU F/W Specification
 *
 * All handling of CAN ports is the same
 * Messages arriving at CAN3 are unconditionally transferred to CAN0, CAN1, and CAN2
 *
 * Transmission rules for CAN messages
 *
 * Message type
 * 1) Event      message (  data frame)
 * 2) Cycle time message (  data frame)
 * 3) Cycle time message (remote frame)
 * 4) Transfer   message (  data frame)
 * 5) Transfer   message (remote frame)
 *
 * Send message buffer
 * 1) For MBOX1 (ID: 000-1FF) Priority: High
 * 2) For MBOX2 (ID: 200-3FF) Priority: Medium
 * 3) For MBOX3 (ID: 400-7FF) Priority: Low
 *
 *
 * When an event occurs, the event message is stored in the event buffer according to
 * the priority indicated by the ID.
 * The event buffers are chained according to priority, with higher priority messages at
 * the top.
 * Simultaneous events are added every 1 ms in priority order to the delay time
 * determined for each ID priority.
 *
 *  When the number of concurrent events is 3
 *                    ID    Delay  Time  Next  Repeat
 *                  +-----+-------+-----+-----+-------+
 *  High priority   | 000 |  +0ms |   0 | 100 | False |
 *      ^           +-----+-------+-----+-----+-------+
 *      |           | 100 |  +1ms |   0 | 7FF | False |
 *      v           +-----+-------+-----+-----+-------+
 *  Low priority    | 7FF |  +2ms |   0 |  -1 | False |
 *                  +-----+-------+-----+-----+-------+
 *
 *  The delay time is updated at the time of event transfer (the highest-priority
 *  message is sent every ms).
 *
 *  When a new event is interrupted
 *                    ID    Delay  Time  Next  Repeat
 *                  +-----+-------+-----+-----+-------+
 *  High priority   | 000 |  +0ms |   1 | 080 | False | <-Chain change
 *      ^           +-----+-------+-----+-----+-------+
 *      |           | 080 |  +0ms |   0 | 100 | False | <-Insert event
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 100 |  +1ms |   1 | 7FF | False |
 *      v           +-----+-------+-----+-----+-------+
 *  Low priority    | 7FF |  +2ms |   1 |  -1 | False |
 *                  +-----+-------+-----+-----+-------+
 *
 *  The transmission order changes according to the priority.
 *  The interrupting event is inserted into the event chain by changing the Next variable of 
 *  the previous event and setting the new event's Next variable to be the original value
 *  of the Next variable of the event that was interrupted (link-list insert).
 *
 *  The cycle time message is a buffer sent according to the cycle time of the ID.
 *  The timer advances every 1 ms, and when it reaches the set value, it is connected to
 *  the transmission buffer chain.
 *  The cycle time buffers are arranged in the order according to the priority of the ID,
 *  and are always processed from the highest priority.
 *
 *                      ID    Cycle  Time  Next  Repeat
 *                  +-----+-------+-----+-----+-------+
 *  High priority   | 001 |  10ms |   9 | 020 | True  |
 *      ^           +-----+-------+-----+-----+-------+
 *      |           | 020 |  15ms |  14 | 200 | True  |
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 200 | 100ms |  99 | 7FE | True  |
 *      v           +-----+-------+-----+-----+-------+
 *  Low priority    | 7FE |  50ms |  49 |  -1 | True  |
 *                  +-----+-------+-----+-----+-------+
 *
 *  If the time is up at the same time, they are stored in the transmission buffer in
 *  the order of priority of ID.
 *  Even if transmission fails due to frequent collisions, if there is free space in
 *  the buffer, it is loaded.
 *
 * Time-up Queue
 * =============
 * The event message and the cycle time message are both connected to the same time-out
 * waiting chain.
 * The chains are connected according to the priority order, and the order of stacking
 * in the transmission chain is maintained even if the time is up at the same time.
 *
 *  The time-up waiting chain is in a state where the above two types are combined
 *
 *                    ID   UpValue Time  Next  Repeat
 *                  +-----+-------+-----+-----+-------+
 *  High priority   | 000 |  +0ms |   1 | 001 | False |
 *      ^           +-----+-------+-----+-----+-------+
 *      |           | 001 |  10ms |   9 | 020 | True  |
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 020 |  15ms |  14 | 080 | True  |
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 080 |  +0ms |   0 | 100 | False | <-Insert event
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 100 |  +1ms |   1 | 200 | False |
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 200 | 100ms |  99 | 7FE | True  |
 *      |           +-----+-------+-----+-----+-------+
 *      |           | 7FE |  50ms |  49 | 7FF | True  |
 *      v           +-----+-------+-----+-----+-------+
 *  Low priority    | 7FF |  +2ms |   1 |  -1 | False |
 *                  +-----+-------+-----+-----+-------+
 *
 *
 * The transmission buffers are transmission queues connected in a chain, and
 * sequentially transmit according to priority.
 * If the next transmission message is NULL, do nothing.
 * Since the transmission message is a chain, it is possible to interrupt a high
 * priority ID.
 * The structure is simpler than the wait-up chain.
 * Transfer occours when the corresponding outgoing mailbox can be sent.
 *
 *  Transmit buffer chain
 *
 *                    ID    Next
 *                  +-----+-----+
 *  High priority   | 000 | 001 |   Transfer to send mailbox 0, issue send request
 *      ^           +-----+-----+
 *      |           | 001 | 200 |   Waiting when outgoing mailbox 0 is in use
 *      |           +-----+-----+
 *      |           | 200 | 7FF |   Transfer to send mailbox 1 and issue send request
 *      v           +-----+-----+
 *  Low priority    | 7FF |  -1 |   Transfer to outgoing mailbox 2 and issue send req.
 *                  +-----+-----+
 *
 *  ID = 000 is sent, and 200 and 7FF are waiting.
 *  After the transmission of 000 is completed and the transmission of 200 is started
 *  in a period before the preparation of transmission of 001, 001 enters a waiting
 *  state.
 *  When 001 is transmitted, 7FF is transmitted.
 *
 *               _________ _______________
 *  MBOX0   ____/__000____X__001__________X____________
 *               _________________
 *  MBOX1   ____/__200____________X____________________
 *               _________________________________
 *  MBOX2   ____/__7FF____________________________X____
 *                _______ _______ _______ _______
 *  CAN     _____/__000__X__200__X__001__X__7FF__X_____
 *             ^
 *
 *             4 simultaneous messages
 *
 *
 * ID-compatible bit data buffer structure
 * =======================================
 * 8 bytes (64 bits) of memory are allocated for ID = 000 to 7FF (16,384 bytes: 0 to
 * 3FFF)
 *
 *  Address calculation formula
 *  Addr = ID * 8
 *
 *
 * Event definition table
 * ======================
 * Table showing whether or not event processing is performed for the byte / bit
 * position of the ID when updating data
 * Each ID has 1-byte information, and 1 to 255 indicate the delay time (ms) of the
 * event validity.
 * If the value is 0, no event processing is performed.
 * Default value
 *
 *  ID range        Delay(ms)
 *  000 to 0FF      1
 *  100 to 1FF      10
 *  200 to 2FF      20
 *  300 to 3FF      30
 *  400 to 4FF      40
 *  500 to 5FF      50
 *  600 to 6FF      60
 *  700 to 7FF      70
 *
 *  The event definition table is referenced only when updated from an I/F other than
 * CAN.
 *
 *
 * Central gateway function (CGW)
 * ==============================
 * The function of connecting the four CAN ports to each other is CGW.
 * Do not echo back to the received CAN port.
 * At the time of transfer, the transfer is performed only to the target port by
 * referring to the routing map.
 *
 *  CAN port    Apply
 *      0       Powertrain
 *      1       Body
 *      2       Chassis
 *      3       OBD2
 *
 *  The message received by CAN0 is transferred to CAN1, CAN2, and CAN3.
 *  At the time of transfer, since the data is stored in the transmission buffer, the
 * priority by ID is applied.
 *
 * The routing map is owned by every ECU and defines the reception ID restrictions and
 * transmission channels.
 * IDs with no transfer destination are only copied to their own station buffer.
 * Normally, the powertrain body chassis has no transfer definition.
 * The CGW forwards all received messages to all channels by default.
 *
 *  Definition of routing table
 *
 *  One byte is required for each ID, and the transfer channel is specified in bits.
 *  +------+---+---+---+---+---+---+---+---+
 *  | bit  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 *  +------+---+---+---+---+---+---+---+---+
 *  | CAN0 | − | − | − | v | − | − | − | o |
 *  +------+---+---+---+---+---+---+---+---+
 *  | CAN1 | − | − | v | − | − | − | o | − |
 *  +------+---+---+---+---+---+---+---+---+
 *  | CAN2 | − | v | − | − | − | o | − | − |
 *  +------+---+---+---+---+---+---+---+---+
 *  | CAN3 | v | − | − | − | o | − | − | − |
 *  +------+---+---+---+---+---+---+---+---+
 *  v Normal mode transfer destination
 *  o Repro mode transfer destination
 *
 *  Each ECU has a normal mode and a repro mode, and accepts data transfer by a repro
 * dedicated ID.
 *  Repro uses a different ID for each ECU, so it is possible to update all ECUs
 * during their own activities.
 *
 *
 * ECU setting information definition E2 data map
 * ==============================================
 *
 *  Memory capacity 32KB (0000-7FFF)
 * +-----------------------+-----------+-----------+-----------+
 * |       Use             |  Address  |   Range   |    Size   |
 * +-----------------------+-----------+-----------+-----------+
 * | Routing map           | 00000000  | 000007FF  | 00000800  |    2048 pc
 * +-----------------------+-----------+-----------+-----------+
 * | Period / event def.   | 00000800  | 00000FFF  | 00000800  |    256 pc
 * +-----------------------+-----------+-----------+-----------+
 *
 *
 * Allocation on RAM
 *
 *  Memory capacity 256KB (00000000 to 0003FFFF)
 * +-----------------------+-----------+-----------+-----------+
 * |      Use              |  Address  |  Range    |   Size    |
 * +-----------------------+-----------+-----------+-----------+
 * | Routing map           | 00000000  | 000007FF  | 00000800  |   2048 pc
 * +-----------------------+-----------+-----------+-----------+
 * | Period / event def.   | 00000800  | 00000FFF  | 00000800  |   256 pc
 * +-----------------------+-----------+-----------+-----------+
 * | CAN frame data        | 00001000  | 00004FFF  | 00004000  |   2048idx8bytex8bit
 * +-----------------------+-----------+-----------+-----------+
 * | CAN trans. waiting buf| 00005000  | 00006FFF  | 00002000  |   256x4ch
 * +-----------------------+-----------+-----------+-----------+
 * | System                | 00007000  | 0002FFFF  |           |
 * +-----------------------+-----------+-----------+-----------+
 * | Vector/stack/DTC      | 00030000  | 0003FFFF  | 00010000  |   65536byte
 * +-----------------------+-----------+-----------+-----------+
 *
 */

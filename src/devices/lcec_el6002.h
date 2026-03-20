// SPDX-License-Identifier: GPL-2.0-or-later
/// @file lcec_el6002.h
/// @brief LinuxCNC EtherCAT HAL driver for Beckhoff EL6002 (2× RS232)
///
/// PDO layout from: ethercat pdos -p 8
///
/// Control word  0x7001:01 / 0x7011:01  (16-bit, master → EL6002)
///   bit  0      Output accept  — RX acknowledgement toggle
///   bit  1      Send request   — TX trigger toggle
///   bits 7:2    TX data length (0–22)
///   bits 15:8   (reserved)
///
/// Status word   0x6001:01 / 0x6011:01  (16-bit, EL6002 → master)
///   bit  0      Input accepted  — TX acknowledgement toggle
///   bit  1      Receive request — RX new-data toggle
///   bits 7:2    RX data length (0–22)
///   bits 15:8   (reserved)
///
/// Data bytes
///   TX  0x7000:0x11 … 0x7000:0x26  (CH1),  0x7010:0x11 … 0x7010:0x26  (CH2)
///   RX  0x6000:0x11 … 0x6000:0x26  (CH1),  0x6010:0x11 … 0x6010:0x26  (CH2)
///
/// HAL pins per channel N (0 or 1):
///   lcec.<master>.<slave>.chN.tx-len    u32 IN   bytes to send (0–22)
///   lcec.<master>.<slave>.chN.tx-data-0 u32 IN   bytes  0– 3 little-endian
///   lcec.<master>.<slave>.chN.tx-data-1 u32 IN   bytes  4– 7
///   lcec.<master>.<slave>.chN.tx-data-2 u32 IN   bytes  8–11
///   lcec.<master>.<slave>.chN.tx-data-3 u32 IN   bytes 12–15
///   lcec.<master>.<slave>.chN.tx-data-4 u32 IN   bytes 16–19
///   lcec.<master>.<slave>.chN.tx-data-5 u32 IN   bytes 20–21
///   lcec.<master>.<slave>.chN.tx-busy   bit OUT  high while EL6002 processes TX
///   lcec.<master>.<slave>.chN.rx-len    u32 OUT  bytes received (0–22)
///   lcec.<master>.<slave>.chN.rx-data-0 u32 OUT  bytes  0– 3
///   lcec.<master>.<slave>.chN.rx-data-1 u32 OUT  bytes  4– 7
///   lcec.<master>.<slave>.chN.rx-data-2 u32 OUT  bytes  8–11
///   lcec.<master>.<slave>.chN.rx-data-3 u32 OUT  bytes 12–15
///   lcec.<master>.<slave>.chN.rx-data-4 u32 OUT  bytes 16–19
///   lcec.<master>.<slave>.chN.rx-data-5 u32 OUT  bytes 20–21
///   lcec.<master>.<slave>.chN.rx-ready  bit OUT  toggles each time new data arrives

#ifndef _LCEC_EL6002_H_
#define _LCEC_EL6002_H_

#include "../lcec.h"

#define LCEC_EL6002_PID       0x17723052u

#define LCEC_EL6002_CHANS     2
#define LCEC_EL6002_MAX_DATA  22
#define LCEC_EL6002_DATA_PINS 6    ///< 6 × u32 = 24 bytes capacity ≥ 22

#endif /* _LCEC_EL6002_H_ */

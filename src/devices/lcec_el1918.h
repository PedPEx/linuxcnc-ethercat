//
//    Copyright (C) 2021 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//

/// @file
/// @brief Driver for Beckhoff EL1918 TwinSAFE logic terminal.
///
/// Same functionality as the EL1918_LOGIC driver (state, cycle-counter,
/// standard in/out, FSoE connections + CRC) plus the 8 local safe-input
/// states from the non-safe Device-Info PDO (0xf180 "Local Inputs"),
/// exported under the "local" subnode as local.in-1..8 (input state) and
/// local.fault-1..8 (module fault), for diagnostics/HMI. Registered as type
/// "EL1918" so it can be used directly in the bus XML.
///
/// NOTE: local-in-* is a non-safe diagnostic mirror, never a safety signal.

#ifndef _LCEC_EL1918_H_
#define _LCEC_EL1918_H_

#include "../lcec.h"

#define LCEC_EL1918_PARAM_SLAVEID     1
#define LCEC_EL1918_PARAM_STDIN_NAME  2
#define LCEC_EL1918_PARAM_STDOUT_NAME 3

#define LCEC_EL1918_DIO_MAX_COUNT  8
#define LCEC_EL1918_LOCAL_CH_COUNT 8  // 8 local safe-input channels; 0xf180 packs them
                                       // as (input, module-fault) bit pairs -> 16 bits

#endif

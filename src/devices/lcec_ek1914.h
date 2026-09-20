//
//    Driver for Beckhoff EK1914 EtherCAT coupler with TwinSAFE I/O
//    (4x standard DI, 4x standard DO, 2x fail-safe DI, 2x fail-safe DO).
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//

/// @file
/// @brief Driver for Beckhoff EK1914 TwinSAFE coupler

#ifndef _LCEC_EK1914_H_
#define _LCEC_EK1914_H_

#include "../lcec.h"

// Identity (ethercat slaves -v -p 0):
//   Vendor Id:       0x00000002  (Beckhoff)
//   Product code:    0x077A2C52
//   Revision number: 0x00120000
#define LCEC_EK1914_PID 0x077A2C52

#define LCEC_EK1914_DIN_COUNT   4
#define LCEC_EK1914_DOUT_COUNT  4
#define LCEC_EK1914_SLOUT_COUNT 2  // safety-linked standard outputs
#define LCEC_EK1914_SIN_COUNT   2  // fail-safe inputs  (FSoE payload)
#define LCEC_EK1914_SOUT_COUNT  2  // fail-safe outputs (FSoE payload)

#endif

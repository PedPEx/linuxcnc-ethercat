//
//    Copyright (C) 2025  lcec-danfoss-fc302 contributors
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
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file
/// @brief Driver for Danfoss FC302 VFD with MCA124 EtherCAT option card.
///
///
/// HARDWARE CONSTRAINT – fixed, non-configurable PDO map
/// ======================================================
/// The MCA124 firmware has a completely fixed PDO object directory.
/// The IgH master attempts to dynamically assign PDO container indices
/// via SDO 0x1C12/0x1C13, but the drive rejects any container index
/// other than its two factory-fixed entries:
///
///   RxPDO 0x1616  (SM2, master -> drive):
///     0x6040:00  ControlWord   u16
///     0x6042:00  Target VL     s16
///
///   TxPDO 0x1A16  (SM3, drive -> master):
///     0x6041:00  StatusWord    u16
///     0x6044:00  Actual VL     s16
///
/// ANY additional PDO container (0x1617, 0x1618, 0x1A17, ...) causes:
///   SDO abort 0x06020000 "This object does not exist in the object directory"
///
/// This means the following objects confirmed M RW/RO P in TwinCAT3 are
/// PDO-mappable ONLY within TwinCAT3's own ESI-based fixed mapping, NOT
/// via the dynamic IgH master PDO assignment protocol:
///   0x6043 (VL demand), 0x6046:01/02 (VL min/max)
///   0x2155/0x2156/0x215F/0x2160 (ramp times)
///   0x217C/0x217D (jog/qstop ramps)
///   0x237A/0x237B (bus jog speeds)
///   0x224E (digital relay ctrl)
///   0x264A/0x264E/0x2652/... (monitoring)
///   0x200F (actual setup readout)
///
/// Consequence: ALL of the above must be handled via SDO writes at
/// startup (modParams) or accepted as unavailable at runtime.
///
///
/// HAL pins  (master="m0", slave="FC302")
/// -----------------------------------------
///
///   Cyclic - updated every servo cycle via PDO:
///     lcec.m0.FC302.srv-cia-controlword   u32  IN
///     lcec.m0.FC302.srv-cia-statusword    u32  OUT
///     lcec.m0.FC302.srv-target-vl         s32  IN
///     lcec.m0.FC302.srv-actual-vl         s32  OUT
///
///   Acyclic SDO monitoring – Infos-RO group (11 objects)
///   Startup delay 5s, then 1s cooldown between requests.
///   All pins under  lcec.m0.FC302.Infos-RO.*
///
///     Infos-RO.power-kw              s32  OUT  par.16-10  0x264A  [0.01 kW]
///     Infos-RO.motor-current         s32  OUT  par.16-14  0x264E  [0.01 A]
///     Infos-RO.dc-link-voltage       u32  OUT  par.16-30  0x265E  [V]
///     Infos-RO.motor-thermal-pct     u32  OUT  par.16-18  0x2652  [%]
///     Infos-RO.kty-temperature       s32  OUT  par.16-19  0x2653  [C]
///     Infos-RO.heatsink-temp         s32  OUT  par.16-34  0x2662  [C]
///     Infos-RO.inverter-thermal-pct  u32  OUT  par.16-35  0x2663  [%]
///     Infos-RO.ctrl-card-temp        s32  OUT  par.16-39  0x2667  [C]
///     Infos-RO.torque-nm             s32  OUT  par.16-16  0x2650  [Nm]
///     Infos-RO.kwh-counter           u32  OUT  par.15-02  0x25DE  [kWh]
///     Infos-RO.torque-pct            s32  OUT  par.16-22  0x2656  [%]
///     Infos-RO.sdo-busy              bit  OUT  TRUE while SDO request pending
///
/// modParams (SDO writes at PREOP->SAFEOP transition via startup SDO list)
/// -----------------------------------------------------------------------
///   [vlMinimum / vlMaximum: NOT writable via SDO on this firmware.]
///   [Set speed limits via FC302 front panel: P4-11 / P4-12.]
///   accelDeltaSpeed  0x6048:01  u32  Acceleration ramp numerator
///   accelDeltaTime   0x6048:02  u16  Acceleration ramp denominator
///   decelDeltaSpeed  0x6049:01  u32  Deceleration ramp numerator
///   decelDeltaTime   0x6049:02  u16  Deceleration ramp denominator
///   vlDimNumerator   0x604C:01  s32  VL dimension factor numerator
///   vlDimDenominator 0x604C:02  s32  VL dimension factor denominator
///   ramp1Up          0x2155:00  u32  Ramp 1 up time   (par. 3-41)
///   ramp1Down        0x2156:00  u32  Ramp 1 down time (par. 3-42)
///   ramp2Up          0x215F:00  u32  Ramp 2 up time   (par. 3-51)
///   ramp2Down        0x2160:00  u32  Ramp 2 down time (par. 3-52)
///   jogRampTime      0x217C:00  u32  Jog ramp time    (par. 3-80)
///   qstopRampTime    0x217D:00  u32  Quick-stop ramp  (par. 3-81)
///   busJog1Speed     0x237A:00  u16  Bus jog 1 speed  (par. 8-90)
///   busJog2Speed     0x237B:00  u16  Bus jog 2 speed  (par. 8-91)
///   digitalRelayCtrl 0x224E:00  u32  Digital/relay bus control (par. 5-90)
///
///
/// Important constraints
/// ---------------------
///  - NO Distributed Clocks (<dcConf> must NOT be in ethercat-conf.xml).
///  - EoE MUST be disabled in the IgH master build (CONFIG_EC_EOE=n).
///  - 0x6060/0x6061 have no P-flag on the MCA124 -> enable_opmode = 0.
///  - 0x6502 (Supported drive modes): the lcec_cia402 framework reads
///    this object internally via SDO during init. The FC302 MCA124 does
///    not support this object, resulting in the dmesg message:
///      "Received mailbox protocol 0x02 / Failed to process SDO request"
///    This warning is HARMLESS and expected. The srv-supported-modes and
///    srv-supports-mode-* HAL pins will show 0 at runtime.
///    The Infos-RO SDO monitoring deliberately waits 3000 servo cycles
///    (~3s) after OP before issuing the first request, to allow the
///    pending 0x6502 response to drain from the FC302 mailbox buffer.
///    Without this delay the first Infos-RO request collides with the
///    stale 0x6502 response, causing "wrong SDO" errors and EtherCAT
///    datagram timeouts that trigger Sync Manager Watchdog faults.
///  - Digital inputs/outputs of the FC302 are NOT accessible via
///    EtherCAT on the MCA124 card. Use FC302 front panel parameters
///    (group 5-xx) to configure digital I/O locally.
///  - 0x232A:00 = 0x0007 is registered as a startup SDO so the master
///    writes it after the PDO assignment, matching the TwinCAT3 <PS>
///    startup list sequence.

#include "../lcec.h"
#include "lcec_class_cia402.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

// ---------------------------------------------------------------------------
// Hardware identity
// ---------------------------------------------------------------------------

#define LCEC_DANFOSS_VID  0x0200008DU
#define LCEC_FC302_PID    0x00000064U

// ---------------------------------------------------------------------------
// Fixed PDO container indices (factory-fixed, non-configurable)
// ---------------------------------------------------------------------------

#define FC302_RXPDO  0x1616U   ///< Only valid RxPDO container
#define FC302_TXPDO  0x1A16U   ///< Only valid TxPDO container

// ---------------------------------------------------------------------------
// modParam IDs  (must be < 0x1000)
// ---------------------------------------------------------------------------

// CiA 402 ramps (SDO)
// NOTE: 0x6046:01 (vlMinimum) and 0x6046:02 (vlMaximum) are NOT writable
// via SDO on the FC302 MCA124 firmware (abort 0x08000020). Set speed limits
// via FC302 front panel: P4-11 (Motor Speed Low Limit) and P4-12 (High Limit).
#define M_ACCEL_DELTA_SPEED  0x0003   // 0x6048:01  u32
#define M_ACCEL_DELTA_TIME   0x0004   // 0x6048:02  u16
#define M_DECEL_DELTA_SPEED  0x0005   // 0x6049:01  u32
#define M_DECEL_DELTA_TIME   0x0006   // 0x6049:02  u16
// CiA 402 dimension factor (SDO)
#define M_VL_DIM_NUMERATOR   0x0007   // 0x604C:01  s32
#define M_VL_DIM_DENOMINATOR 0x0008   // 0x604C:02  s32
// Danfoss-specific ramps (SDO)
#define M_RAMP1_UP           0x0009   // 0x2155:00  u32  par. 3-41
#define M_RAMP1_DOWN         0x000A   // 0x2156:00  u32  par. 3-42
#define M_RAMP2_UP           0x000B   // 0x215F:00  u32  par. 3-51
#define M_RAMP2_DOWN         0x000C   // 0x2160:00  u32  par. 3-52
#define M_JOG_RAMP           0x000D   // 0x217C:00  u32  par. 3-80
#define M_QSTOP_RAMP         0x000E   // 0x217D:00  u32  par. 3-81
// Danfoss-specific jog speeds (SDO)
#define M_BUS_JOG1_SPEED     0x000F   // 0x237A:00  u16  par. 8-90
#define M_BUS_JOG2_SPEED     0x0010   // 0x237B:00  u16  par. 8-91
// Danfoss digital/relay control (SDO)
#define M_DIGITAL_RELAY_CTRL 0x0011   // 0x224E:00  u32  par. 5-90
// Infos-RO monitoring enable bitmask
// Each bit enables one SDO monitoring channel (see documentation below)
#define M_SDO_READ_CONFIG    0x0012   // u32 bitmask

// ---------------------------------------------------------------------------
// modParam descriptor tables
// ---------------------------------------------------------------------------

static const lcec_modparam_desc_t modparams_perchannel[] = {
    // CiA 402 ramps
    {"accelDeltaSpeed",  M_ACCEL_DELTA_SPEED,  MODPARAM_TYPE_U32},
    {"accelDeltaTime",   M_ACCEL_DELTA_TIME,   MODPARAM_TYPE_U32},
    {"decelDeltaSpeed",  M_DECEL_DELTA_SPEED,  MODPARAM_TYPE_U32},
    {"decelDeltaTime",   M_DECEL_DELTA_TIME,   MODPARAM_TYPE_U32},
    // Dimension factor
    {"vlDimNumerator",   M_VL_DIM_NUMERATOR,   MODPARAM_TYPE_S32},
    {"vlDimDenominator", M_VL_DIM_DENOMINATOR, MODPARAM_TYPE_S32},
    // Danfoss ramps
    {"ramp1Up",          M_RAMP1_UP,           MODPARAM_TYPE_U32},
    {"ramp1Down",        M_RAMP1_DOWN,         MODPARAM_TYPE_U32},
    {"ramp2Up",          M_RAMP2_UP,           MODPARAM_TYPE_U32},
    {"ramp2Down",        M_RAMP2_DOWN,         MODPARAM_TYPE_U32},
    {"jogRampTime",      M_JOG_RAMP,           MODPARAM_TYPE_U32},
    {"qstopRampTime",    M_QSTOP_RAMP,         MODPARAM_TYPE_U32},
    // Danfoss jog speeds
    {"busJog1Speed",     M_BUS_JOG1_SPEED,     MODPARAM_TYPE_U32},
    {"busJog2Speed",     M_BUS_JOG2_SPEED,     MODPARAM_TYPE_U32},
    // Danfoss digital/relay
    {"digitalRelayCtrl", M_DIGITAL_RELAY_CTRL, MODPARAM_TYPE_U32},
    // Infos-RO SDO monitoring enable bitmask
    // Bit 0=power-kw  1=motor-current  2=dc-link-voltage  3=motor-thermal-pct
    // Bit 4=kty-temperature  5=heatsink-temp  6=inverter-thermal-pct
    // Bit 7=ctrl-card-temp   8=torque-nm  9=kwh-counter  10=torque-pct
    // Example: 0x7FF enables all 11 channels
    {"sdoReadConfig",    M_SDO_READ_CONFIG,    MODPARAM_TYPE_U32},
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {NULL},
};

static const lcec_modparam_doc_t chan_docs[] = {
    {"accelDeltaSpeed",
     "SDO 0x6048:01 (U32). CiA 402 acceleration ramp numerator "
     "[velocity-unit]. accel = accelDeltaSpeed / accelDeltaTime."},
    {"accelDeltaTime",
     "SDO 0x6048:02 (U16, max 65535). CiA 402 acceleration ramp denominator "
     "[time-unit]. Default on device: 3."},
    {"decelDeltaSpeed",
     "SDO 0x6049:01 (U32). CiA 402 deceleration ramp numerator [velocity-unit]."},
    {"decelDeltaTime",
     "SDO 0x6049:02 (U16, max 65535). CiA 402 deceleration ramp denominator. "
     "Default on device: 3."},
    {"vlDimNumerator",
     "SDO 0x604C:01 (S32). VL dimension factor numerator. "
     "v_physical = v_raw * num / denom. Default: 1."},
    {"vlDimDenominator",
     "SDO 0x604C:02 (S32, != 0). VL dimension factor denominator. Default: 1."},
    {"ramp1Up",
     "SDO 0x2155:00 (U32). FC302 par. 3-41: Ramp 1 up time."},
    {"ramp1Down",
     "SDO 0x2156:00 (U32). FC302 par. 3-42: Ramp 1 down time."},
    {"ramp2Up",
     "SDO 0x215F:00 (U32). FC302 par. 3-51: Ramp 2 up time."},
    {"ramp2Down",
     "SDO 0x2160:00 (U32). FC302 par. 3-52: Ramp 2 down time."},
    {"jogRampTime",
     "SDO 0x217C:00 (U32). FC302 par. 3-80: Jog ramp time."},
    {"qstopRampTime",
     "SDO 0x217D:00 (U32). FC302 par. 3-81: Quick-stop ramp time."},
    {"busJog1Speed",
     "SDO 0x237A:00 (U16). FC302 par. 8-90: Bus jog 1 speed."},
    {"busJog2Speed",
     "SDO 0x237B:00 (U16). FC302 par. 8-91: Bus jog 2 speed."},
    {"digitalRelayCtrl",
     "SDO 0x224E:00 (U32). FC302 par. 5-90: Digital and Relay Bus Control. "
     "Bit-coded control of digital outputs and relays via bus."},
    {"sdoReadConfig",
     "Bitmask enabling Infos-RO acyclic SDO monitoring channels. "
     "Bit 0=power-kw (0x264A) | Bit 1=motor-current (0x264E) | "
     "Bit 2=dc-link-voltage (0x265E) | Bit 3=motor-thermal-pct (0x2652) | "
     "Bit 4=kty-temperature (0x2653) | Bit 5=heatsink-temp (0x2662) | "
     "Bit 6=inverter-thermal-pct (0x2663) | Bit 7=ctrl-card-temp (0x2667) | "
     "Bit 8=torque-nm (0x2650) | Bit 9=kwh-counter (0x25DE) | "
     "Bit 10=torque-pct (0x2656). "
     "0=disable all (default). 0x7FF=enable all 11 channels. "
     "Only enabled channels get HAL pins and are polled. "
     "The Infos-RO.sdo-busy pin is only created if at least one bit is set."},
    {NULL},
};

static const lcec_modparam_doc_t base_docs[] = {
    {NULL},
};

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static int  lcec_danfoss_fc302_init (int comp_id, lcec_slave_t *slave);
static void lcec_danfoss_fc302_read (lcec_slave_t *slave, long period);
static void lcec_danfoss_fc302_write(lcec_slave_t *slave, long period);

// ---------------------------------------------------------------------------
// Type registration
// ---------------------------------------------------------------------------

static lcec_typelist_t types[] = {
    {
        .name      = "FC302",
        .vid       = LCEC_DANFOSS_VID,
        .pid       = LCEC_FC302_PID,
        .proc_init = lcec_danfoss_fc302_init,
    },
    {NULL},
};

ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1,
                                 modparams_perchannel, modparams_base,
                                 chan_docs, base_docs)

// ---------------------------------------------------------------------------
// Per-slave HAL data
// ---------------------------------------------------------------------------

// SDO monitoring descriptor
typedef struct {
    uint16_t          idx;
    uint8_t           sidx;
    size_t            size;
    int               is_signed;
    const char       *pin_name;
    ec_sdo_request_t *req;
    hal_s32_t        *pin_s32;
    hal_u32_t        *pin_u32;
} fc302_mon_t;

#define FC302_MON_COUNT      11    // number of Infos-RO channels
// Target: 2 updates/s per OBJECT = full cycle in 500ms.
// With 13 objects: 500ms / 13 = ~38ms per object.
// FC302 mailbox RTT is ~30-50ms → no artificial cooldown needed.
// The next request fires immediately after SUCCESS/ERROR.
// Actual update rate per object: 13 x ~40ms = ~520ms cycle → ~1.9x/s.
#define FC302_MON_COOLDOWN   0     // no artificial wait – mailbox RTT limits rate
#define FC302_MON_STARTDELAY 5000  // startup delay before first request (5s @ 1kHz)

typedef struct {
    lcec_class_cia402_channels_t *cia402;

    // VL target and actual velocity -- mapped manually because
    // enable_vl=1 also registers 0x6043/0x6046 which are not PDO-mappable.
    unsigned int target_vl_os;
    unsigned int actual_vl_os;
    hal_s32_t *target_vl;   // srv-target-vl  s32  IN
    hal_s32_t *actual_vl;   // srv-actual-vl  s32  OUT

    // Infos-RO acyclic SDO monitoring
    fc302_mon_t  mon[FC302_MON_COUNT];  // only enabled entries are used
    int          mon_count;        // number of enabled channels (0..FC302_MON_COUNT)
    int          mon_current;      // current channel index
    int          mon_cooldown;     // cycles remaining in cooldown
    int          mon_startup;      // remaining startup delay cycles
    int          mon_running;      // 1 after first request fired
    hal_bit_t   *mon_sdo_busy;     // TRUE while request in-flight
} lcec_danfoss_fc302_data_t;

// ---------------------------------------------------------------------------
// Device-specific HAL pins – none: only the 4 fixed PDO objects are
// available cyclically; everything else is SDO-only via modParams.
// ---------------------------------------------------------------------------

// srv-target-vl and srv-actual-vl are registered directly via hal_pin_newf()
// in lcec_danfoss_fc302_init() to ensure the correct lcec.m0.FC302.* prefix.
// lcec_pin_newf_list() does not prepend the module/master/slave path for
// device-specific pins, so it cannot be used here.

// ---------------------------------------------------------------------------
// Helper: register a startup SDO (16-bit) with range check
// ---------------------------------------------------------------------------

static int fc302_sdo16(lcec_slave_t *slave, uint16_t idx, uint8_t sidx,
                        uint16_t val, const char *name) {
    if (ecrt_slave_config_sdo16(slave->config, idx, sidx, val) != 0) {
        rtapi_print_msg(RTAPI_MSG_WARN,
            LCEC_MSG_PFX "FC302 slave %s.%s: could not register startup "
            "SDO 0x%04X:%02X (%s) - drive keeps stored value\n",
            slave->master->name, slave->name, idx, sidx, name);
    }
    return 0;
}


static int fc302_sdo32(lcec_slave_t *slave, uint16_t idx, uint8_t sidx,
                        uint32_t val, const char *name) {
    if (ecrt_slave_config_sdo32(slave->config, idx, sidx, val) != 0) {
        rtapi_print_msg(RTAPI_MSG_WARN,
            LCEC_MSG_PFX "FC302 slave %s.%s: could not register startup "
            "SDO 0x%04X:%02X (%s) – drive keeps stored value\n",
            slave->master->name, slave->name, idx, sidx, name);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// modParam handler – all params become startup SDOs
// ---------------------------------------------------------------------------

static int handle_modparams(lcec_slave_t *slave,
                             lcec_class_cia402_options_t *options) {
    lcec_master_t *master = slave->master;
    lcec_slave_modparam_t *p;
    int v, ret;

    for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
        switch (p->id) {

            // ---- CiA 402 ramps --------------------------------------------
            case M_ACCEL_DELTA_SPEED:
                fc302_sdo32(slave, 0x6048, 0x01,
                                        p->value.u32,
                                        "accelDeltaSpeed");
                break;
            case M_ACCEL_DELTA_TIME: {
                uint32_t val = p->value.u32;
                if (val > 0xFFFF) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "accelDeltaTime %u > U16 "
                        "for slave %s.%s\n", val, master->name, slave->name);
                    return -EINVAL;
                }
                if ((ret = fc302_sdo16(slave, 0x6048, 0x02,
                                        (uint16_t)val,
                                        "accelDeltaTime")) != 0) return ret;
                break;
            }
            case M_DECEL_DELTA_SPEED:
                fc302_sdo32(slave, 0x6049, 0x01,
                                        p->value.u32,
                                        "decelDeltaSpeed");
                break;
            case M_DECEL_DELTA_TIME: {
                uint32_t val = p->value.u32;
                if (val > 0xFFFF) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "decelDeltaTime %u > U16 "
                        "for slave %s.%s\n", val, master->name, slave->name);
                    return -EINVAL;
                }
                if ((ret = fc302_sdo16(slave, 0x6049, 0x02,
                                        (uint16_t)val,
                                        "decelDeltaTime")) != 0) return ret;
                break;
            }

            // ---- VL dimension factor -------------------------------------
            case M_VL_DIM_NUMERATOR:
                if (p->value.s32 == 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "vlDimNumerator must not be 0 "
                        "for slave %s.%s\n", master->name, slave->name);
                    return -EINVAL;
                }
                if ((ret = fc302_sdo32(slave, 0x604C, 0x01,
                                        (uint32_t)p->value.s32,
                                        "vlDimNumerator")) != 0) return ret;
                break;
            case M_VL_DIM_DENOMINATOR:
                if (p->value.s32 == 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "vlDimDenominator must not be 0 "
                        "for slave %s.%s\n", master->name, slave->name);
                    return -EINVAL;
                }
                if ((ret = fc302_sdo32(slave, 0x604C, 0x02,
                                        (uint32_t)p->value.s32,
                                        "vlDimDenominator")) != 0) return ret;
                break;

            // ---- Danfoss ramp times (par. 3-xx) --------------------------
            case M_RAMP1_UP:
                if ((ret = fc302_sdo32(slave, 0x2155, 0x00,
                                        p->value.u32, "ramp1Up")) != 0)
                    return ret;
                break;
            case M_RAMP1_DOWN:
                if ((ret = fc302_sdo32(slave, 0x2156, 0x00,
                                        p->value.u32, "ramp1Down")) != 0)
                    return ret;
                break;
            case M_RAMP2_UP:
                if ((ret = fc302_sdo32(slave, 0x215F, 0x00,
                                        p->value.u32, "ramp2Up")) != 0)
                    return ret;
                break;
            case M_RAMP2_DOWN:
                if ((ret = fc302_sdo32(slave, 0x2160, 0x00,
                                        p->value.u32, "ramp2Down")) != 0)
                    return ret;
                break;
            case M_JOG_RAMP:
                if ((ret = fc302_sdo32(slave, 0x217C, 0x00,
                                        p->value.u32, "jogRampTime")) != 0)
                    return ret;
                break;
            case M_QSTOP_RAMP:
                if ((ret = fc302_sdo32(slave, 0x217D, 0x00,
                                        p->value.u32, "qstopRampTime")) != 0)
                    return ret;
                break;

            // ---- Danfoss jog speeds (par. 8-xx) --------------------------
            case M_BUS_JOG1_SPEED: {
                uint32_t val = p->value.u32;
                if (val > 0xFFFF) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "busJog1Speed %u > U16 "
                        "for slave %s.%s\n", val, master->name, slave->name);
                    return -EINVAL;
                }
                if ((ret = fc302_sdo16(slave, 0x237A, 0x00,
                                        (uint16_t)val,
                                        "busJog1Speed")) != 0) return ret;
                break;
            }
            case M_BUS_JOG2_SPEED: {
                uint32_t val = p->value.u32;
                if (val > 0xFFFF) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "busJog2Speed %u > U16 "
                        "for slave %s.%s\n", val, master->name, slave->name);
                    return -EINVAL;
                }
                if ((ret = fc302_sdo16(slave, 0x237B, 0x00,
                                        (uint16_t)val,
                                        "busJog2Speed")) != 0) return ret;
                break;
            }

            // ---- Danfoss digital/relay bus control (par. 5-90) -----------
            case M_DIGITAL_RELAY_CTRL:
                fc302_sdo32(slave, 0x224E, 0x00,
                                        p->value.u32,
                                        "digitalRelayCtrl");
                break;

            // ---- Infos-RO SDO monitoring bitmask ------------------------
            // Stored in slave->hal_data for use in init() after modparams.
            // We store it temporarily in mon_count (abused as u32 here).
            case M_SDO_READ_CONFIG: {
                lcec_danfoss_fc302_data_t *hd =
                    (lcec_danfoss_fc302_data_t *)slave->hal_data;
                hd->mon_count = (int)p->value.u32; // temp: bitmask
                break;
            }

            // ---- Generic CiA 402 fallback --------------------------------
            default:
                v = lcec_cia402_handle_modparam(slave, p, options);
                if (v < 0) return v;
                if (v > 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "unknown modparam %s for slave %s.%s\n",
                        p->name, master->name, slave->name);
                    return -EINVAL;
                }
                break;
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

static int lcec_danfoss_fc302_init(int comp_id, lcec_slave_t *slave) {
    lcec_danfoss_fc302_data_t *hal_data;
    int err;

    hal_data = LCEC_HAL_ALLOCATE(lcec_danfoss_fc302_data_t);
    slave->hal_data   = hal_data;
    slave->proc_read  = lcec_danfoss_fc302_read;
    slave->proc_write = lcec_danfoss_fc302_write;

    // -----------------------------------------------------------------------
    // CiA 402 feature selection – see file header for full rationale.
    // -----------------------------------------------------------------------
    lcec_class_cia402_options_t *options = lcec_cia402_options();

    options->channels = 1;

    for (int ch = 0; ch < options->channels; ch++) {
        // 0x6060/0x6061: no PDO-mappable P-flag on FC302 MCA124.
        options->channel[ch]->enable_opmode         = 0;

        // enable_vl = 0: the VL flag also registers 0x6043/0x6046:01/02
        // which are not PDO-mappable on this device, causing fatal startup
        // errors. 0x6042 (target) and 0x6044 (actual) are mapped manually.
        options->channel[ch]->enable_vl             = 0;

        // 0x603F not mappable on MCA124.
        options->channel[ch]->enable_error_code     = 0;

        // Digital I/O not accessible via EtherCAT on MCA124.
        options->channel[ch]->enable_digital_input  = 0;
        options->channel[ch]->enable_digital_output = 0;
    }

    // -----------------------------------------------------------------------
    // Startup SDO: 0x232A:00 = 0x0007  ("Control Word Profile", par. P8-10)
    //
    // Configures the MCA124 to accept CiA 402 VL control words.
    // MUST be written AFTER the PDO assignment (0x1C12/0x1C13).
    // ecrt_slave_config_sdo16() places it in the master's startup SDO list,
    // which is processed at the end of the PREOP->SAFEOP transition.
    // This matches the TwinCAT3 <PS> startup list sequence exactly.
    // -----------------------------------------------------------------------
    if (ecrt_slave_config_sdo16(slave->config, 0x232A, 0x00, 0x0007) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            LCEC_MSG_PFX "FC302 slave %s.%s: failed to register startup SDO "
            "0x232A:00 (Control Word Profile = VL mode)\n",
            slave->master->name, slave->name);
        return -EIO;
    }

    // -----------------------------------------------------------------------
    // Apply XML modParams (all become startup SDOs via ecrt_slave_config_sdo*)
    // -----------------------------------------------------------------------
    if (handle_modparams(slave, options) != 0) {
        return -EIO;
    }

    // -----------------------------------------------------------------------
    // Sync manager / PDO mapping
    //
    // The FC302 MCA124 supports ONLY two fixed PDO containers:
    //   RxPDO 0x1616:  ControlWord (6040/u16) + Target VL (6042/s16)
    //   TxPDO 0x1A16:  StatusWord  (6041/u16) + Actual VL (6044/s16)
    //
    // Any additional container index (0x1617, 0x1618, 0x1A17, ...) causes
    // SDO abort 0x06020000 "object does not exist" during PDO assignment.
    // This has been confirmed by dmesg output.
    //
    // We do NOT use lcec_cia402_add_output_sync / add_input_sync because
    // those helpers generate container indices from the CiA 402 default
    // base (0x1600/0x1A00), which also cause AL error 0x001E on this device.
    // -----------------------------------------------------------------------
    lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);

    // SM2: exactly one RxPDO container with exactly two entries
    lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE);
    lcec_syncs_add_pdo_info(syncs, FC302_RXPDO);           // 0x1616 only
    lcec_syncs_add_pdo_entry(syncs, 0x6040, 0x00, 16);    // ControlWord  u16
    lcec_syncs_add_pdo_entry(syncs, 0x6042, 0x00, 16);    // Target VL    s16

    // SM3: exactly one TxPDO container with exactly two entries
    lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);
    lcec_syncs_add_pdo_info(syncs, FC302_TXPDO);           // 0x1A16 only
    lcec_syncs_add_pdo_entry(syncs, 0x6041, 0x00, 16);    // StatusWord   u16
    lcec_syncs_add_pdo_entry(syncs, 0x6044, 0x00, 16);    // Actual VL    s16

    slave->sync_info = &syncs->syncs[0];

    // -----------------------------------------------------------------------
    // Register CiA 402 channel
    // HAL pins created (full names: lcec.m0.FC302.*):
    //   srv-cia-controlword   u32  IN
    //   srv-cia-statusword    u32  OUT
    //   srv-target-vl         s32  IN
    //   srv-actual-vl         s32  OUT
    // (srv-vl-demand, srv-vl-minimum, srv-vl-maximum will exist as pins
    //  but their process data offsets will be 0 – harmless at runtime)
    // -----------------------------------------------------------------------
    hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);

    for (int ch = 0; ch < options->channels; ch++) {
        // lcec_cia402_register_channel() attempts an SDO upload of 0x6502
        // (Supported Drive Modes). The FC302 MCA124 rejects this with
        // error -5 / abort_code 0x00000000, producing two harmless but
        // confusing error messages at every startup.
        // Workaround: suppress all RTAPI messages during this one call,
        // then restore the previous level immediately after.
        // Genuine errors are still caught by the NULL-pointer check below.
        int prev_msg_level = rtapi_get_msg_level();
        rtapi_set_msg_level(RTAPI_MSG_NONE);

        hal_data->cia402->channels[ch] =
            lcec_cia402_register_channel(slave,
                                          0x6000 + 0x800 * ch,
                                          options->channel[ch]);

        rtapi_set_msg_level(prev_msg_level);

        if (hal_data->cia402->channels[ch] == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                LCEC_MSG_PFX "lcec_cia402_register_channel failed "
                "for slave %s.%s ch%d\n",
                slave->master->name, slave->name, ch);
            return -EIO;
        }
    }

    // Register VL PDO entries manually (cannot use enable_vl=1, see above)
    lcec_pdo_init(slave, 0x6042, 0x00, &hal_data->target_vl_os, NULL);
    lcec_pdo_init(slave, 0x6044, 0x00, &hal_data->actual_vl_os, NULL);

    // Register VL HAL pins via hal_pin_new() with pre-formatted name.
    // hal_pin_newf() is not available in this LinuxCNC build; use
    // rtapi_snprintf() + hal_pin_new() instead.
    {
        char pin_name[HAL_NAME_LEN + 1];

        rtapi_snprintf(pin_name, sizeof(pin_name), "%s.%s.%s.srv-target-vl",
                       LCEC_MODULE_NAME, slave->master->name, slave->name);
        if ((err = hal_pin_new(pin_name, HAL_S32, HAL_IN,
                               (void **)&hal_data->target_vl,
                               comp_id)) != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                LCEC_MSG_PFX "Failed to create pin %s\n", pin_name);
            return err;
        }
        *(hal_data->target_vl) = 0;

        rtapi_snprintf(pin_name, sizeof(pin_name), "%s.%s.%s.srv-actual-vl",
                       LCEC_MODULE_NAME, slave->master->name, slave->name);
        if ((err = hal_pin_new(pin_name, HAL_S32, HAL_OUT,
                               (void **)&hal_data->actual_vl,
                               comp_id)) != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                LCEC_MSG_PFX "Failed to create pin %s\n", pin_name);
            return err;
        }
        *(hal_data->actual_vl) = 0;
    }

    // -----------------------------------------------------------------------
    // Infos-RO acyclic SDO monitoring setup
    //
    // Channels enabled by sdoReadConfig modParam bitmask.
    // Each bit corresponds to one object (bit 0 = 0x264A, bit 1 = 0x264E, ...).
    // If no bits set: no pins, no requests, no sdo-busy pin created.
    // -----------------------------------------------------------------------
    {
        // Full definition table (bit position = index into this array)
        struct { uint16_t idx; uint8_t sidx; size_t sz; int sgn; const char *nm; } all[] = {
            {0x264A, 0x00, 4, 1, "Infos-RO.power-kw"},           // bit  0  par.16-10 s32
            {0x264E, 0x00, 4, 1, "Infos-RO.motor-current"},       // bit  1  par.16-14 s32
            {0x265E, 0x00, 2, 0, "Infos-RO.dc-link-voltage"},     // bit  2  par.16-30 u16
            {0x2652, 0x00, 1, 0, "Infos-RO.motor-thermal-pct"},   // bit  3  par.16-18 u8
            {0x2653, 0x00, 2, 1, "Infos-RO.kty-temperature"},     // bit  4  par.16-19 s16
            {0x2662, 0x00, 1, 1, "Infos-RO.heatsink-temp"},       // bit  5  par.16-34 s8
            {0x2663, 0x00, 1, 0, "Infos-RO.inverter-thermal-pct"},// bit  6  par.16-35 u8
            {0x2667, 0x00, 1, 1, "Infos-RO.ctrl-card-temp"},      // bit  7  par.16-39 s8
            {0x2650, 0x00, 2, 1, "Infos-RO.torque-nm"},           // bit  8  par.16-16 s16
            {0x25DE, 0x00, 4, 0, "Infos-RO.kwh-counter"},         // bit  9  par.15-02 u32
            {0x2656, 0x00, 2, 1, "Infos-RO.torque-pct"},          // bit 10  par.16-22 s16
        };

        // Read bitmask stored temporarily in mon_count by handle_modparams
        uint32_t mask = (uint32_t)hal_data->mon_count;
        hal_data->mon_count = 0;

        if (mask == 0) {
            rtapi_print_msg(RTAPI_MSG_INFO,
                LCEC_MSG_PFX "FC302 %s.%s: sdoReadConfig=0, "
                "Infos-RO monitoring disabled\n",
                slave->master->name, slave->name);
        } else {
            char pname[HAL_NAME_LEN + 1];
            int  n = 0;

            for (int i = 0; i < FC302_MON_COUNT; i++) {
                if (!(mask & (1u << i))) continue;

                fc302_mon_t *m = &hal_data->mon[n];
                m->idx       = all[i].idx;
                m->sidx      = all[i].sidx;
                m->size      = all[i].sz;
                m->is_signed = all[i].sgn;
                m->pin_name  = all[i].nm;

                m->req = ecrt_slave_config_create_sdo_request(
                             slave->config, m->idx, m->sidx, m->size);
                if (!m->req) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        LCEC_MSG_PFX "FC302 %s.%s: SDO request failed "
                        "0x%04X:%02X\n", slave->master->name, slave->name,
                        m->idx, m->sidx);
                    return -EIO;
                }
                ecrt_sdo_request_timeout(m->req, 1000);

                rtapi_snprintf(pname, sizeof(pname), "%s.%s.%s.%s",
                    LCEC_MODULE_NAME, slave->master->name,
                    slave->name, m->pin_name);
                if (m->is_signed) {
                    if ((err = hal_pin_new(pname, HAL_S32, HAL_OUT,
                                           (void **)&m->pin_s32, comp_id)) != 0)
                        return err;
                    *(m->pin_s32) = 0;
                } else {
                    if ((err = hal_pin_new(pname, HAL_U32, HAL_OUT,
                                           (void **)&m->pin_u32, comp_id)) != 0)
                        return err;
                    *(m->pin_u32) = 0;
                }
                n++;
            }
            hal_data->mon_count = n;

            // Infos-RO.sdo-busy (only created when at least one channel active)
            rtapi_snprintf(pname, sizeof(pname), "%s.%s.%s.Infos-RO.sdo-busy",
                LCEC_MODULE_NAME, slave->master->name, slave->name);
            if ((err = hal_pin_new(pname, HAL_BIT, HAL_OUT,
                                   (void **)&hal_data->mon_sdo_busy, comp_id)) != 0)
                return err;
            *(hal_data->mon_sdo_busy) = 0;

            rtapi_print_msg(RTAPI_MSG_INFO,
                LCEC_MSG_PFX "FC302 %s.%s: Infos-RO %d channel(s) active "
                "(mask=0x%03X)\n",
                slave->master->name, slave->name, n, mask);
        }

        hal_data->mon_current  = 0;
        hal_data->mon_cooldown = 0;
        hal_data->mon_startup  = FC302_MON_STARTDELAY;
        hal_data->mon_running  = 0;
    }

    return 0;
}

// ---------------------------------------------------------------------------
// Cyclic read  (EtherCAT -> HAL)
// ---------------------------------------------------------------------------

static void lcec_danfoss_fc302_read(lcec_slave_t *slave, long period) {
    lcec_danfoss_fc302_data_t *hal_data =
        (lcec_danfoss_fc302_data_t *)slave->hal_data;

    if (!slave->state.operational) {
        return;
    }

    lcec_cia402_read_all(slave, hal_data->cia402);

    // Read VL actual velocity from process data
    uint8_t *pd = slave->master->process_data;
    *(hal_data->actual_vl) = (int16_t)EC_READ_U16(&pd[hal_data->actual_vl_os]);

    // -----------------------------------------------------------------------
    // Infos-RO acyclic SDO monitoring – round-robin with startup delay
    // -----------------------------------------------------------------------
    // Skip monitoring entirely if no channels configured
    if (hal_data->mon_count == 0) return;

    // Startup delay: let the stale 0x6502 mailbox response drain first
    if (hal_data->mon_startup > 0) {
        hal_data->mon_startup--;
        *(hal_data->mon_sdo_busy) = 0;
        return;
    }
    // Fire first request after startup delay
    if (!hal_data->mon_running) {
        ecrt_sdo_request_read(hal_data->mon[0].req);
        hal_data->mon_running  = 1;
        *(hal_data->mon_sdo_busy) = 1;
        return;
    }
    // Cooldown between requests
    if (hal_data->mon_cooldown > 0) {
        hal_data->mon_cooldown--;
        *(hal_data->mon_sdo_busy) = 0;
        return;
    }
    // Check current request state
    {
        int cur = hal_data->mon_current;
        fc302_mon_t *m = &hal_data->mon[cur];
        ec_request_state_t state = ecrt_sdo_request_state(m->req);

        if (state == EC_REQUEST_SUCCESS) {
            uint8_t *data = ecrt_sdo_request_data(m->req);
            if (m->is_signed) {
                int32_t val = 0;
                switch (m->size) {
                    case 1: val = (int8_t) EC_READ_U8(data);  break;
                    case 2: val = (int16_t)EC_READ_U16(data); break;
                    default:val = (int32_t)EC_READ_U32(data); break;
                }
                *(m->pin_s32) = val;
            } else {
                uint32_t val = 0;
                switch (m->size) {
                    case 1: val = EC_READ_U8(data);  break;
                    case 2: val = EC_READ_U16(data); break;
                    default:val = EC_READ_U32(data); break;
                }
                *(m->pin_u32) = val;
            }
            hal_data->mon_current  = (cur + 1) % hal_data->mon_count;
            hal_data->mon_cooldown = FC302_MON_COOLDOWN;
            ecrt_sdo_request_read(hal_data->mon[hal_data->mon_current].req);
            *(hal_data->mon_sdo_busy) = 1;

        } else if (state == EC_REQUEST_ERROR) {
            hal_data->mon_current  = (cur + 1) % hal_data->mon_count;
            hal_data->mon_cooldown = FC302_MON_COOLDOWN;
            ecrt_sdo_request_read(hal_data->mon[hal_data->mon_current].req);
            *(hal_data->mon_sdo_busy) = 1;

        } else {
            *(hal_data->mon_sdo_busy) = (state == EC_REQUEST_BUSY) ? 1 : 0;
        }
    }
}

// ---------------------------------------------------------------------------
// Cyclic write  (HAL -> EtherCAT)
// ---------------------------------------------------------------------------

static void lcec_danfoss_fc302_write(lcec_slave_t *slave, long period) {
    lcec_danfoss_fc302_data_t *hal_data =
        (lcec_danfoss_fc302_data_t *)slave->hal_data;

    if (!slave->state.operational) {
        return;
    }

    lcec_cia402_write_all(slave, hal_data->cia402);

    // Write VL target velocity to process data
    uint8_t *pd = slave->master->process_data;
    EC_WRITE_U16(&pd[hal_data->target_vl_os], (uint16_t)(int16_t)*(hal_data->target_vl));

}

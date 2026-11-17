// SPDX-License-Identifier: GPL-2.0-or-later
//
/// @file
/// @brief Driver for Beckhoff EP5101-0011 incremental encoder box (IP67, D-SUB15), 32-bit mode
///
/// Replaces the previous EP5101 driver. The HAL interface (pins and
/// modParams) is identical to lcec_el5101_32.c ("EL5101-32"), so EL5101 and
/// EP5101 are interchangeable by changing type= in ethercat-conf.xml.
/// Only the device definition block (object indices, PDO numbers, entry
/// lists) differs between the two files.
///
/// PDO assignment (Beckhoff documentation EP5101-0011; the device default is
/// the 16-bit "compact" pair 0x1600 / 0x1A00)
///   SM2 RxPDO 0x1601 "ENC Control"     6 bytes  (always)
///   SM3 TxPDO 0x1A01 "ENC Status"     10 bytes  (always)
///       TxPDO 0x1A02 "ENC Frequency"   4 bytes  (modParam enableFrequency)
///       TxPDO 0x1A03 "ENC Period"      4 bytes  (modParam enablePeriod)
///   Frequency and period PDOs exclude each other; enabling both is
///   rejected at init.
///
/// The entry lists follow the Beckhoff documentation and the previous
/// (working) driver; they are NOT yet verified against hardware. Check once:
///   for idx in 0x1601 0x1A01; do
///     n=$(ethercat upload -p <pos> -t uint8 $idx 0 | awk '{print $2}')
///     for s in $(seq 1 $n); do ethercat upload -p <pos> -t uint32 $idx $s; done
///   done
/// and look for "does not support changing the PDO mapping" in dmesg.
/// Process data offsets only depend on the bit lengths, which match the
/// documented layout in any case.
///
/// Layout:
///   0x1A01  bits  0..12  6000:01..0D status bits
///           bit  13      1C32:20 Sync error
///           bit  14      1800:07 TxPDO State
///           bit  15      1800:09 TxPDO Toggle
///           bits 16..47  6000:11 Counter value (32 bit)
///           bits 48..79  6000:12 Latch value   (32 bit)
///   0x1A02  6000:13 Frequency value (32 bit), unit 1/freqScaling Hz
///   0x1A03  6000:14 Period value    (32 bit), unit periodScaling ns
///   0x1601  bits  0..3   7000:01..04 control bits, bits 4..15 gap
///           bits 16..47  7000:11 Set counter value (32 bit)
///
/// Startup SDOs (0x8000 ENC Settings)
///   All boolean/enum settings are written on EVERY start, independent of
///   the configured value. The terminal retains CoE values (e.g. from a
///   TwinCAT session), so factory defaults can never be assumed.
///   The numeric frequency/period settings (8000:11, :13 .. :17) are only
///   written when the corresponding modParam is present in the XML.
///
/// Diagnostics
///   Cyclic (PDO): open circuit, extrapolation stall, sync error, TxPDO state,
///                 TxPDO toggle, overflow/underflow, input states A/B/C/gate
///   Mailbox (SDO, throttled to once per SDO_READ_INTERVAL cycles):
///                 A000:01..03 open circuit A / B / C
///   The SDO requests are only created when open circuit detection is
///   enabled for at least one track.
///
/// Example ethercat-conf.xml:
///   <slave idx="1" type="EP5101" name="spindle-enc">
///     <modParam name="enableFrequency"       value="true"/>
///     <modParam name="enablePeriod"          value="false"/>
///     <modParam name="openCircuitDetection"  value="true"/>   <!-- A + B, RS422 only -->
///     <modParam name="openCircuitDetectionC" value="true"/>   <!-- C, RS422 only -->
///     <modParam name="reversionOfRotation"   value="false"/>
///   </slave>
///
/// Counter handling
///   The 32-bit hardware counter is accumulated as signed deltas into a
///   64-bit position, so enc-pos never jumps when the counter wraps.
///   enc-index-c-enable follows the LinuxCNC index-enable convention
///   (e.g. spindle.N.index-enable / joint.N.index-enable): set it TRUE to arm
///   the C-track latch; the driver resets enc-count/enc-pos to zero at the
///   latched index position and clears the pin.
///   enc-velocity is the signed velocity in user units per second (pos-scale
///   counts per unit), derived from the counter delta and filtered with the
///   time constant enc-velocity-tc (default 0.01 s). It is suitable for
///   spindle.N.speed-in. enc-frequency is unsigned (no direction).
///   Do NOT combine enc-index-c-enable with enableCReset / enableExtReset:
///   a hardware counter reset looks like a large position jump to the driver.

#include "../lcec.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/* ======================================================================
 * Device definition (the only part that differs from lcec_el5101_32.c)
 * ====================================================================== */
#define DEV_NAME        "EP5101"
#define DEV_PID         0x13ED4052
#define OBJ_IN          0x6000   /* ENC Inputs     */
#define OBJ_OUT         0x7000   /* ENC Outputs    */
#define OBJ_SET         0x8000   /* ENC Settings   */
#define OBJ_DIAG        0xA000   /* ENC Diag data (not verified on hardware) */
#define PDO_CTRL        0x1601   /* ENC Control    */
#define PDO_STATUS      0x1A01   /* ENC Status     */
#define PDO_FREQ        0x1A02   /* ENC Frequency  */
#define PDO_PERIOD      0x1A03   /* ENC Period     */

/* Consecutive failed diagnostic SDO reads before polling of that object
 * is suspended for SDO_BACKOFF_INTERVALS read intervals. Failures are
 * usually transient (e.g. "ethercat upload" from the command line sharing
 * the mailbox), so polling is never stopped permanently. Only the first
 * failure of a series is logged; a successful read re-arms the message.  */
#define SDO_MAX_ERRORS         3
#define SDO_BACKOFF_INTERVALS 60   /* 60 x 1 s at 1 kHz */

/* Read diagnostic SDOs every N servo cycles (1000 @ 1 kHz = once per second). */
#define SDO_READ_INTERVAL 1000

/* ======================================================================
 * Defaults (Beckhoff factory values) used when a modParam is absent.
 * They are always written to the terminal, see file header.
 * ====================================================================== */
#define DEF_ENABLE_FREQUENCY       0
#define DEF_ENABLE_PERIOD          0
#define DEF_ENABLE_C_RESET         0  /* 8000:01 */
#define DEF_ENABLE_EXT_RESET       0  /* 8000:02 */
#define DEF_ENABLE_UP_DOWN         0  /* 8000:03 0 = quadrature A/B */
#define DEF_GATE_POLARITY          0  /* 8000:04 0 = gate disabled */
#define DEF_DISABLE_FILTER         0  /* 8000:08 */
#define DEF_ENABLE_MICRO_INC       0  /* 8000:0A */
#define DEF_OPEN_CIRCUIT_AB        1  /* 8000:0B/0C factory ON  */
#define DEF_OPEN_CIRCUIT_C         0  /* 8000:0D factory OFF    */
#define DEF_REVERSION_OF_ROTATION  0  /* 8000:0E */
#define DEF_EXT_RESET_POLARITY     1  /* 8000:10 1 = rising edge */
#define DEF_FREQ_SCALING           100 /* 8000:13 100 -> 0.01 Hz  */
#define DEF_PERIOD_SCALING         100 /* 8000:14 100 -> 100 ns   */

/* ======================================================================
 * modParam IDs
 * ====================================================================== */
enum {
  MP_ENABLE_FREQUENCY = 0,
  MP_ENABLE_PERIOD,
  MP_ENABLE_C_RESET,
  MP_ENABLE_EXT_RESET,
  MP_ENABLE_UP_DOWN,
  MP_GATE_POLARITY,
  MP_DISABLE_FILTER,
  MP_ENABLE_MICRO_INC,
  MP_OPEN_CIRCUIT_AB,
  MP_OPEN_CIRCUIT_C,
  MP_REVERSION_OF_ROTATION,
  MP_EXT_RESET_POLARITY,
  MP_FREQ_WINDOW,
  MP_FREQ_SCALING,
  MP_PERIOD_SCALING,
  MP_FREQ_RESOLUTION,
  MP_PERIOD_RESOLUTION,
  MP_FREQ_WAIT_TIME,
};

static const lcec_modparam_desc_t lcec_ep5101_modparams[] = {
  {"enableFrequency",       MP_ENABLE_FREQUENCY,      MODPARAM_TYPE_BIT, "false",
    "Assign TxPDO 0x1A02 (frequency value 6000:13)"},
  {"enablePeriod",          MP_ENABLE_PERIOD,         MODPARAM_TYPE_BIT, "false",
    "Assign TxPDO 0x1A03 (period value 6000:14)"},
  {"enableCReset",          MP_ENABLE_C_RESET,        MODPARAM_TYPE_BIT, "false",
    "8000:01 Hardware counter reset via C input"},
  {"enableExtReset",        MP_ENABLE_EXT_RESET,      MODPARAM_TYPE_BIT, "false",
    "8000:02 Hardware counter reset via external latch input"},
  {"enableUpDown",          MP_ENABLE_UP_DOWN,        MODPARAM_TYPE_BIT, "false",
    "8000:03 Up/down counter mode (A = pulse, B = direction)"},
  {"gatePolarity",          MP_GATE_POLARITY,         MODPARAM_TYPE_U32, "0",
    "8000:04 Gate polarity (0 = disabled, 1 = enable pos., 2 = enable neg.)"},
  {"disableFilter",         MP_DISABLE_FILTER,        MODPARAM_TYPE_BIT, "false",
    "8000:08 Disable input filter"},
  {"enableMicroInc",        MP_ENABLE_MICRO_INC,      MODPARAM_TYPE_BIT, "false",
    "8000:0A Micro increments (DC mode only, lower 8 bits = fraction)"},
  {"openCircuitDetection",  MP_OPEN_CIRCUIT_AB,       MODPARAM_TYPE_BIT, "true",
    "8000:0B/0C Open circuit detection tracks A and B (RS422 only, disable for TTL)"},
  {"openCircuitDetectionC", MP_OPEN_CIRCUIT_C,        MODPARAM_TYPE_BIT, "false",
    "8000:0D Open circuit detection track C (RS422 only, disable for TTL)"},
  {"reversionOfRotation",   MP_REVERSION_OF_ROTATION, MODPARAM_TYPE_BIT, "false",
    "8000:0E Reverse counting direction"},
  {"extResetPolarity",      MP_EXT_RESET_POLARITY,    MODPARAM_TYPE_BIT, "true",
    "8000:10 External reset polarity (0 = falling, 1 = rising edge)"},
  {"freqWindow",            MP_FREQ_WINDOW,           MODPARAM_TYPE_U32, "10000",
    "8000:11 Frequency window [us] (written only if set)"},
  {"freqScaling",           MP_FREQ_SCALING,          MODPARAM_TYPE_U32, "100",
    "8000:13 Frequency scaling, 100 = 0.01 Hz (written only if set)"},
  {"periodScaling",         MP_PERIOD_SCALING,        MODPARAM_TYPE_U32, "100",
    "8000:14 Period scaling, 100 = 100 ns (written only if set)"},
  {"freqResolution",        MP_FREQ_RESOLUTION,       MODPARAM_TYPE_U32, "100",
    "8000:15 Frequency resolution, 100 = 0.01 Hz (written only if set)"},
  {"periodResolution",      MP_PERIOD_RESOLUTION,     MODPARAM_TYPE_U32, "100",
    "8000:16 Period resolution, 100 = 100 ns (written only if set)"},
  {"freqWaitTime",          MP_FREQ_WAIT_TIME,        MODPARAM_TYPE_U32, "1600",
    "8000:17 Frequency wait time [ms] (written only if set)"},
  {NULL},
};

static int lcec_ep5101_init(int comp_id, lcec_slave_t *slave);
static void lcec_ep5101_read(lcec_slave_t *slave, long period);
static void lcec_ep5101_write(lcec_slave_t *slave, long period);

static lcec_typelist_t types[] = {
  {DEV_NAME, LCEC_BECKHOFF_VID, DEV_PID, 0, NULL,
    lcec_ep5101_init, lcec_ep5101_modparams},
  {NULL},
};
ADD_TYPES(types);

/* ======================================================================
 * Status word (0x1A01 bits 0..15) and control word (0x1601 bits 0..15)
 * ====================================================================== */
#define STS_LATCH_C_VALID    (1u <<  0)  /* 6000:01 */
#define STS_LATCH_EXT_VALID  (1u <<  1)  /* 6000:02 */
#define STS_SET_CNT_DONE     (1u <<  2)  /* 6000:03 */
#define STS_UNDERFLOW        (1u <<  3)  /* 6000:04 */
#define STS_OVERFLOW         (1u <<  4)  /* 6000:05 */
#define STS_INPUT_STATUS     (1u <<  5)  /* 6000:06 */
#define STS_OPEN_CIRCUIT     (1u <<  6)  /* 6000:07 */
#define STS_EXTRAP_STALL     (1u <<  7)  /* 6000:08 */
#define STS_INPUT_A          (1u <<  8)  /* 6000:09 */
#define STS_INPUT_B          (1u <<  9)  /* 6000:0A */
#define STS_INPUT_C          (1u << 10)  /* 6000:0B */
#define STS_INPUT_GATE       (1u << 11)  /* 6000:0C */
#define STS_EXT_LATCH        (1u << 12)  /* 6000:0D */
#define STS_SYNC_ERROR       (1u << 13)  /* 6000:0E */
#define STS_TXPDO_STATE      (1u << 14)  /* 6000:0F */
#define STS_TXPDO_TOGGLE     (1u << 15)  /* 6000:10 */

#define CTRL_ENA_LATCH_C       (1u << 0) /* 7000:01 */
#define CTRL_ENA_LATCH_EXT_POS (1u << 1) /* 7000:02 */
#define CTRL_SET_COUNTER       (1u << 2) /* 7000:03 */
#define CTRL_ENA_LATCH_EXT_NEG (1u << 3) /* 7000:04 */

/* ======================================================================
 * Fixed PDO mappings (Beckhoff documentation EP5101-0011). They must match
 * the device bit for bit, otherwise lcec computes wrong offsets.
 * ====================================================================== */
static ec_pdo_entry_info_t lcec_ep5101_ctrl_entries[] = {   /* PDO_CTRL   */
  {OBJ_OUT, 0x01,  1},  /* Enable latch C              */
  {OBJ_OUT, 0x02,  1},  /* Enable latch extern on pos  */
  {OBJ_OUT, 0x03,  1},  /* Set counter                 */
  {OBJ_OUT, 0x04,  1},  /* Enable latch extern on neg  */
  {0x0000, 0x00,  4},  /* gap                         */
  {0x0000, 0x00,  8},  /* gap                         */
  {OBJ_OUT, 0x11, 32},  /* Set counter value           */
};

static ec_pdo_entry_info_t lcec_ep5101_status_entries[] = { /* PDO_STATUS */
  {OBJ_IN,  0x01,  1},  /* Latch C valid               */
  {OBJ_IN,  0x02,  1},  /* Latch extern valid          */
  {OBJ_IN,  0x03,  1},  /* Set counter done            */
  {OBJ_IN,  0x04,  1},  /* Counter underflow           */
  {OBJ_IN,  0x05,  1},  /* Counter overflow            */
  {OBJ_IN,  0x06,  1},  /* Status of input status      */
  {OBJ_IN,  0x07,  1},  /* Open circuit                */
  {OBJ_IN,  0x08,  1},  /* Extrapolation stall         */
  {OBJ_IN,  0x09,  1},  /* Status of input A           */
  {OBJ_IN,  0x0A,  1},  /* Status of input B           */
  {OBJ_IN,  0x0B,  1},  /* Status of input C           */
  {OBJ_IN,  0x0C,  1},  /* Status of input gate        */
  {OBJ_IN,  0x0D,  1},  /* Status of extern latch      */
  {0x1C32,  0x20,  1},  /* Sync error                  */
  {0x1800,  0x07,  1},  /* TxPDO State                 */
  {0x1800,  0x09,  1},  /* TxPDO Toggle                */
  {OBJ_IN,  0x11, 32},  /* Counter value               */
  {OBJ_IN,  0x12, 32},  /* Latch value                 */
};

static ec_pdo_entry_info_t lcec_ep5101_freq_entries[] = {   /* PDO_FREQ   */
  {OBJ_IN,  0x13, 32},  /* Frequency value             */
};

static ec_pdo_entry_info_t lcec_ep5101_period_entries[] = { /* PDO_PERIOD */
  {OBJ_IN,  0x14, 32},  /* Period value                */
};

/* ======================================================================
 * HAL data
 * ====================================================================== */
typedef struct {
  /* HAL pins: control (IO / IN) */
  hal_bit_t *ena_latch_c;        /* enc-index-c-enable        IO */
  hal_bit_t *ena_latch_ext_pos;  /* enc-index-ext-pos-enable  IO */
  hal_bit_t *ena_latch_ext_neg;  /* enc-index-ext-neg-enable  IO */
  hal_bit_t *reset;              /* enc-reset                 IN */
  hal_bit_t *set_raw_count;      /* enc-set-raw-count         IO */
  hal_s32_t *set_raw_count_val;  /* enc-set-raw-count-val     IN */
  hal_float_t *pos_scale;        /* enc-pos-scale             IO */

  /* HAL pins: status bits */
  hal_bit_t *latch_c_valid;
  hal_bit_t *latch_ext_valid;
  hal_bit_t *set_cnt_done;
  hal_bit_t *underflow;
  hal_bit_t *overflow;
  hal_bit_t *inext;
  hal_bit_t *open_circuit;
  hal_bit_t *extrap_stall;
  hal_bit_t *input_a;
  hal_bit_t *input_b;
  hal_bit_t *input_c;
  hal_bit_t *input_gate;
  hal_bit_t *ext_latch_status;
  hal_bit_t *sync_error;
  hal_bit_t *txpdo_state;
  hal_bit_t *txpdo_toggle;
  hal_bit_t *error;              /* summary: open circuit | txpdo state | sync error */

  /* HAL pins: SDO diagnostics (A000) */
  hal_bit_t *open_circuit_a;
  hal_bit_t *open_circuit_b;
  hal_bit_t *open_circuit_c;

  /* HAL pins: values */
  hal_u32_t *status_raw;
  hal_s32_t *raw_count;
  hal_s32_t *raw_latch;
  hal_s32_t *count;
  hal_float_t *pos;
  hal_float_t *velocity;         /* signed, units/s, filtered            */
  hal_float_t *velocity_tc;      /* IO: filter time constant [s]         */
  hal_u32_t *raw_frequency;
  hal_float_t *frequency;        /* Hz */
  hal_u32_t *raw_period;
  hal_float_t *period;           /* s  */

  /* PDO offsets */
  unsigned int status_os, status_bp;
  unsigned int count_os;
  unsigned int latch_os;
  unsigned int freq_os;
  unsigned int period_os;
  unsigned int ctrl_os, ctrl_bp;
  unsigned int setval_os;

  /* Per-slave sync manager configuration (assignment depends on modParams) */
  ec_pdo_info_t  pdos_out[1];
  ec_pdo_info_t  pdos_in[2];
  ec_sync_info_t syncs[5];

  /* Configuration */
  int    freq_enabled;
  int    period_enabled;
  double freq_unit;              /* Hz per raw count */
  double period_unit;            /* s per raw count  */

  /* Diagnostic SDO requests (NULL if open circuit detection is disabled) */
  ec_sdo_request_t *sdo_oc[3];
  unsigned int      sdo_err_cnt[3];
  unsigned int      sdo_backoff[3];
  int               sdo_quiet[3];
  unsigned int      sdo_read_timer;

  /* Velocity estimation (independent of latch/reset handling) */
  uint32_t vel_last_raw;
  int      vel_valid;
  double   vel_filt;

  /* Latch handshake: after a latch was taken, the enable bit is held low
   * until the terminal has cleared its "latch valid" flag, so a re-armed
   * latch can never pick up a stale latch value.                          */
  int      latch_c_block;
  int      latch_ext_block;

  /* Internal state */
  uint32_t last_raw;
  int64_t  accum;
  int      do_init;
  int      last_operational;
  int      layout_checked;
  double   scale;
  double   old_scale;
} lcec_ep5101_data_t;

static const lcec_pindesc_t slave_pins[] = {
  /* control */
  {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_c),       "%s.%s.%s.enc-index-c-enable"},
  {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_ext_pos), "%s.%s.%s.enc-index-ext-pos-enable"},
  {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_ext_neg), "%s.%s.%s.enc-index-ext-neg-enable"},
  {HAL_BIT,   HAL_IN,  offsetof(lcec_ep5101_data_t, reset),             "%s.%s.%s.enc-reset"},
  {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, set_raw_count),     "%s.%s.%s.enc-set-raw-count"},
  {HAL_S32,   HAL_IN,  offsetof(lcec_ep5101_data_t, set_raw_count_val), "%s.%s.%s.enc-set-raw-count-val"},
  {HAL_FLOAT, HAL_IO,  offsetof(lcec_ep5101_data_t, pos_scale),         "%s.%s.%s.enc-pos-scale"},
  /* status bits */
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, latch_c_valid),     "%s.%s.%s.enc-latch-c-valid"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, latch_ext_valid),   "%s.%s.%s.enc-latch-ext-valid"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, set_cnt_done),      "%s.%s.%s.enc-set-count-done"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, underflow),         "%s.%s.%s.enc-underflow"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, overflow),          "%s.%s.%s.enc-overflow"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, inext),             "%s.%s.%s.enc-inext"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, open_circuit),      "%s.%s.%s.enc-open-circuit"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, extrap_stall),      "%s.%s.%s.enc-extrap-stall"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_a),           "%s.%s.%s.enc-input-a"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_b),           "%s.%s.%s.enc-input-b"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_c),           "%s.%s.%s.enc-input-c"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_gate),        "%s.%s.%s.enc-input-gate"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, ext_latch_status),  "%s.%s.%s.enc-ext-latch-status"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, sync_error),        "%s.%s.%s.enc-sync-error"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, txpdo_state),       "%s.%s.%s.enc-txpdo-state"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, txpdo_toggle),      "%s.%s.%s.enc-txpdo-toggle"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, error),             "%s.%s.%s.enc-error"},
  /* SDO diagnostics */
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, open_circuit_a),    "%s.%s.%s.enc-open-circuit-a"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, open_circuit_b),    "%s.%s.%s.enc-open-circuit-b"},
  {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, open_circuit_c),    "%s.%s.%s.enc-open-circuit-c"},
  /* values */
  {HAL_U32,   HAL_OUT, offsetof(lcec_ep5101_data_t, status_raw),        "%s.%s.%s.enc-status-raw"},
  {HAL_S32,   HAL_OUT, offsetof(lcec_ep5101_data_t, raw_count),         "%s.%s.%s.enc-raw-count"},
  {HAL_S32,   HAL_OUT, offsetof(lcec_ep5101_data_t, raw_latch),         "%s.%s.%s.enc-raw-latch"},
  {HAL_S32,   HAL_OUT, offsetof(lcec_ep5101_data_t, count),             "%s.%s.%s.enc-count"},
  {HAL_FLOAT, HAL_OUT, offsetof(lcec_ep5101_data_t, pos),               "%s.%s.%s.enc-pos"},
  {HAL_FLOAT, HAL_OUT, offsetof(lcec_ep5101_data_t, velocity),          "%s.%s.%s.enc-velocity"},
  {HAL_FLOAT, HAL_IO,  offsetof(lcec_ep5101_data_t, velocity_tc),       "%s.%s.%s.enc-velocity-tc"},
  {HAL_U32,   HAL_OUT, offsetof(lcec_ep5101_data_t, raw_frequency),     "%s.%s.%s.enc-raw-freq"},
  {HAL_FLOAT, HAL_OUT, offsetof(lcec_ep5101_data_t, frequency),         "%s.%s.%s.enc-frequency"},
  {HAL_U32,   HAL_OUT, offsetof(lcec_ep5101_data_t, raw_period),        "%s.%s.%s.enc-raw-period"},
  {HAL_FLOAT, HAL_OUT, offsetof(lcec_ep5101_data_t, period),            "%s.%s.%s.enc-period"},
  {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/* ======================================================================
 * Helpers
 * ====================================================================== */
static void lcec_ep5101_sdo_warn(lcec_slave_t *slave, uint8_t sidx, const char *name) {
  rtapi_print_msg(RTAPI_MSG_WARN,
    LCEC_MSG_PFX "slave %s.%s: SDO 0x%04X:%02X (%s) write failed\n",
    slave->master->name, slave->name, OBJ_SET, sidx, name);
}

static void lcec_ep5101_write8(lcec_slave_t *slave, uint8_t sidx, uint8_t val, const char *name) {
  if (lcec_write_sdo8(slave, OBJ_SET, sidx, val) != 0)
    lcec_ep5101_sdo_warn(slave, sidx, name);
}

static void lcec_ep5101_write16(lcec_slave_t *slave, uint8_t sidx, uint16_t val, const char *name) {
  if (lcec_write_sdo16(slave, OBJ_SET, sidx, val) != 0)
    lcec_ep5101_sdo_warn(slave, sidx, name);
}

/* Poll one diagnostic SDO request (A000:xx, BOOLEAN) and re-trigger it. */
/* Poll one diagnostic SDO request (BOOLEAN) and re-trigger it.
 * Returns the number of read intervals to skip (0 = keep polling).       */
static unsigned int lcec_ep5101_poll_sdo(lcec_slave_t *slave, ec_sdo_request_t *req,
                                hal_bit_t *pin, uint8_t sidx,
                                unsigned int *err_cnt, int *quiet) {
  switch (ecrt_sdo_request_state(req)) {
    case EC_REQUEST_SUCCESS:
      *pin = (EC_READ_U8(ecrt_sdo_request_data(req)) & 0x01) ? 1 : 0;
      *err_cnt = 0;
      *quiet   = 0;
      ecrt_sdo_request_read(req);
      break;
    case EC_REQUEST_UNUSED:
      ecrt_sdo_request_read(req);
      break;
    case EC_REQUEST_BUSY:
      break;
    case EC_REQUEST_ERROR:
      if (!*quiet) {
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "SDO read failed on 0x%04X:%02X for slave %s.%s - retrying\n",
          OBJ_DIAG, sidx, slave->master->name, slave->name);
        *quiet = 1;
      }
      ecrt_sdo_request_read(req);
      if (++(*err_cnt) >= SDO_MAX_ERRORS) {
        *err_cnt = 0;
        return SDO_BACKOFF_INTERVALS;
      }
      break;
  }
  return 0;
}

/* ======================================================================
 * Init
 * ====================================================================== */
static int lcec_ep5101_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_ep5101_data_t *hal_data;
  lcec_slave_modparam_t *p;
  unsigned int n_in;
  int err, i;

  /* Settings with defaults, overridden by XML modParams */
  int cfg_freq        = DEF_ENABLE_FREQUENCY;
  int cfg_period      = DEF_ENABLE_PERIOD;
  int cfg_c_reset     = DEF_ENABLE_C_RESET;
  int cfg_ext_reset   = DEF_ENABLE_EXT_RESET;
  int cfg_up_down     = DEF_ENABLE_UP_DOWN;
  int cfg_gate_pol    = DEF_GATE_POLARITY;
  int cfg_dis_filter  = DEF_DISABLE_FILTER;
  int cfg_micro_inc   = DEF_ENABLE_MICRO_INC;
  int cfg_oc_ab       = DEF_OPEN_CIRCUIT_AB;
  int cfg_oc_c        = DEF_OPEN_CIRCUIT_C;
  int cfg_reversion   = DEF_REVERSION_OF_ROTATION;
  int cfg_ext_rst_pol = DEF_EXT_RESET_POLARITY;

  /* Numeric frequency/period settings: -1 = not configured, do not write */
  int32_t cfg_freq_window = -1, cfg_freq_scaling = -1, cfg_period_scaling = -1;
  int32_t cfg_freq_res = -1, cfg_period_res = -1, cfg_wait_time = -1;

  slave->proc_read  = lcec_ep5101_read;
  slave->proc_write = lcec_ep5101_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_ep5101_data_t);
  slave->hal_data = hal_data;

  /* --- Collect modParams --- */
  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    switch (p->id) {
      case MP_ENABLE_FREQUENCY:      cfg_freq        = p->value.bit ? 1 : 0; break;
      case MP_ENABLE_PERIOD:         cfg_period      = p->value.bit ? 1 : 0; break;
      case MP_ENABLE_C_RESET:        cfg_c_reset     = p->value.bit ? 1 : 0; break;
      case MP_ENABLE_EXT_RESET:      cfg_ext_reset   = p->value.bit ? 1 : 0; break;
      case MP_ENABLE_UP_DOWN:        cfg_up_down     = p->value.bit ? 1 : 0; break;
      case MP_GATE_POLARITY:         cfg_gate_pol    = (int)(p->value.u32 & 0x03); break;
      case MP_DISABLE_FILTER:        cfg_dis_filter  = p->value.bit ? 1 : 0; break;
      case MP_ENABLE_MICRO_INC:      cfg_micro_inc   = p->value.bit ? 1 : 0; break;
      case MP_OPEN_CIRCUIT_AB:       cfg_oc_ab       = p->value.bit ? 1 : 0; break;
      case MP_OPEN_CIRCUIT_C:        cfg_oc_c        = p->value.bit ? 1 : 0; break;
      case MP_REVERSION_OF_ROTATION: cfg_reversion   = p->value.bit ? 1 : 0; break;
      case MP_EXT_RESET_POLARITY:    cfg_ext_rst_pol = p->value.bit ? 1 : 0; break;
      case MP_FREQ_WINDOW:           cfg_freq_window    = (int32_t)(p->value.u32 & 0xFFFF); break;
      case MP_FREQ_SCALING:          cfg_freq_scaling   = (int32_t)(p->value.u32 & 0xFFFF); break;
      case MP_PERIOD_SCALING:        cfg_period_scaling = (int32_t)(p->value.u32 & 0xFFFF); break;
      case MP_FREQ_RESOLUTION:       cfg_freq_res       = (int32_t)(p->value.u32 & 0xFFFF); break;
      case MP_PERIOD_RESOLUTION:     cfg_period_res     = (int32_t)(p->value.u32 & 0xFFFF); break;
      case MP_FREQ_WAIT_TIME:        cfg_wait_time      = (int32_t)(p->value.u32 & 0xFFFF); break;
      default:
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "unknown modParam id %d for slave %s.%s\n",
          p->id, master->name, slave->name);
        break;
    }
  }

  if (cfg_gate_pol > 2) {
    rtapi_print_msg(RTAPI_MSG_WARN,
      LCEC_MSG_PFX "slave %s.%s: gatePolarity %d invalid, using 0\n",
      master->name, slave->name, cfg_gate_pol);
    cfg_gate_pol = 0;
  }
  if (cfg_freq && cfg_period) {
    rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX "slave %s.%s: enableFrequency and enablePeriod are mutually exclusive "
      "(PDO 0x%04X excludes 0x%04X)\n", master->name, slave->name, PDO_FREQ, PDO_PERIOD);
    return -EINVAL;
  }
  if (cfg_freq_scaling == 0 || cfg_period_scaling == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX "slave %s.%s: freqScaling/periodScaling must not be 0\n",
      master->name, slave->name);
    return -EINVAL;
  }

  hal_data->freq_enabled   = cfg_freq;
  hal_data->period_enabled = cfg_period;
  hal_data->freq_unit   = 1.0 / (double)(cfg_freq_scaling > 0 ? cfg_freq_scaling : DEF_FREQ_SCALING);
  hal_data->period_unit = (double)(cfg_period_scaling > 0 ? cfg_period_scaling : DEF_PERIOD_SCALING) * 1e-9;

  /* --- Sync manager / PDO assignment (per slave, depends on modParams) --- */
  hal_data->pdos_out[0].index     = PDO_CTRL;
  hal_data->pdos_out[0].n_entries = ARRAY_SIZE(lcec_ep5101_ctrl_entries);
  hal_data->pdos_out[0].entries   = lcec_ep5101_ctrl_entries;

  n_in = 0;
  hal_data->pdos_in[n_in].index     = PDO_STATUS;
  hal_data->pdos_in[n_in].n_entries = ARRAY_SIZE(lcec_ep5101_status_entries);
  hal_data->pdos_in[n_in++].entries = lcec_ep5101_status_entries;
  if (cfg_freq) {
    hal_data->pdos_in[n_in].index     = PDO_FREQ;
    hal_data->pdos_in[n_in].n_entries = ARRAY_SIZE(lcec_ep5101_freq_entries);
    hal_data->pdos_in[n_in++].entries = lcec_ep5101_freq_entries;
  }
  if (cfg_period) {
    hal_data->pdos_in[n_in].index     = PDO_PERIOD;
    hal_data->pdos_in[n_in].n_entries = ARRAY_SIZE(lcec_ep5101_period_entries);
    hal_data->pdos_in[n_in++].entries = lcec_ep5101_period_entries;
  }

  hal_data->syncs[0].index = 0; hal_data->syncs[0].dir = EC_DIR_OUTPUT;
  hal_data->syncs[0].n_pdos = 0; hal_data->syncs[0].pdos = NULL;
  hal_data->syncs[0].watchdog_mode = EC_WD_DEFAULT;

  hal_data->syncs[1].index = 1; hal_data->syncs[1].dir = EC_DIR_INPUT;
  hal_data->syncs[1].n_pdos = 0; hal_data->syncs[1].pdos = NULL;
  hal_data->syncs[1].watchdog_mode = EC_WD_DEFAULT;

  hal_data->syncs[2].index = 2; hal_data->syncs[2].dir = EC_DIR_OUTPUT;
  hal_data->syncs[2].n_pdos = 1; hal_data->syncs[2].pdos = hal_data->pdos_out;
  hal_data->syncs[2].watchdog_mode = EC_WD_DEFAULT;

  hal_data->syncs[3].index = 3; hal_data->syncs[3].dir = EC_DIR_INPUT;
  hal_data->syncs[3].n_pdos = n_in; hal_data->syncs[3].pdos = hal_data->pdos_in;
  hal_data->syncs[3].watchdog_mode = EC_WD_DEFAULT;

  hal_data->syncs[4].index = 0xFF;

  slave->sync_info = hal_data->syncs;

  /* --- PDO entry registration --- */
  lcec_pdo_init(slave, OBJ_IN, 0x01, &hal_data->status_os, &hal_data->status_bp);
  lcec_pdo_init(slave, OBJ_IN, 0x11, &hal_data->count_os,  NULL);
  lcec_pdo_init(slave, OBJ_IN, 0x12, &hal_data->latch_os,  NULL);
  if (cfg_freq)
    lcec_pdo_init(slave, OBJ_IN, 0x13, &hal_data->freq_os, NULL);
  if (cfg_period)
    lcec_pdo_init(slave, OBJ_IN, 0x14, &hal_data->period_os, NULL);
  lcec_pdo_init(slave, OBJ_OUT, 0x01, &hal_data->ctrl_os,   &hal_data->ctrl_bp);
  lcec_pdo_init(slave, OBJ_OUT, 0x11, &hal_data->setval_os, NULL);

  /* --- HAL pins --- */
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  /* hal_malloc() memory is not guaranteed zeroed */
  *(hal_data->pos_scale)  = 1.0;
  hal_data->old_scale     = 2.0;   /* force scale update in first read */
  hal_data->scale         = 1.0;
  hal_data->do_init          = 1;
  hal_data->last_operational = 0;
  hal_data->layout_checked   = 0;
  hal_data->accum            = 0;
  hal_data->last_raw         = 0;
  hal_data->sdo_read_timer   = 0;
  *(hal_data->velocity_tc)   = 0.01;
  hal_data->vel_valid        = 0;
  hal_data->vel_filt         = 0.0;
  hal_data->vel_last_raw     = 0;
  hal_data->latch_c_block    = 0;
  hal_data->latch_ext_block  = 0;

  /* --- OBJ_SET ENC settings: always written, see file header --- */
  lcec_ep5101_write8(slave, 0x01, cfg_c_reset,     "enableCReset");
  lcec_ep5101_write8(slave, 0x02, cfg_ext_reset,   "enableExtReset");
  lcec_ep5101_write8(slave, 0x03, cfg_up_down,     "enableUpDown");
  lcec_ep5101_write8(slave, 0x04, cfg_gate_pol,    "gatePolarity");
  lcec_ep5101_write8(slave, 0x08, cfg_dis_filter,  "disableFilter");
  lcec_ep5101_write8(slave, 0x0A, cfg_micro_inc,   "enableMicroInc");
  lcec_ep5101_write8(slave, 0x0B, cfg_oc_ab,       "openCircuitDetection (A)");
  lcec_ep5101_write8(slave, 0x0C, cfg_oc_ab,       "openCircuitDetection (B)");
  lcec_ep5101_write8(slave, 0x0D, cfg_oc_c,        "openCircuitDetectionC");
  lcec_ep5101_write8(slave, 0x0E, cfg_reversion,   "reversionOfRotation");
  lcec_ep5101_write8(slave, 0x10, cfg_ext_rst_pol, "extResetPolarity");

  /* Numeric settings: only when configured */
  if (cfg_freq_window    >= 0) lcec_ep5101_write16(slave, 0x11, cfg_freq_window,    "freqWindow");
  if (cfg_freq_scaling   >= 0) lcec_ep5101_write16(slave, 0x13, cfg_freq_scaling,   "freqScaling");
  if (cfg_period_scaling >= 0) lcec_ep5101_write16(slave, 0x14, cfg_period_scaling, "periodScaling");
  if (cfg_freq_res       >= 0) lcec_ep5101_write16(slave, 0x15, cfg_freq_res,       "freqResolution");
  if (cfg_period_res     >= 0) lcec_ep5101_write16(slave, 0x16, cfg_period_res,     "periodResolution");
  if (cfg_wait_time      >= 0) lcec_ep5101_write16(slave, 0x17, cfg_wait_time,      "freqWaitTime");

  /* --- Diagnostic SDO requests A000:01..03 (open circuit A/B/C) --- */
  for (i = 0; i < 3; i++) {
    hal_data->sdo_oc[i]      = NULL;
    hal_data->sdo_err_cnt[i] = 0;
    hal_data->sdo_backoff[i] = 0;
    hal_data->sdo_quiet[i]   = 0;
  }
  if (cfg_oc_ab || cfg_oc_c) {
    for (i = 0; i < 3; i++) {
      if (i < 2 && !cfg_oc_ab) continue;
      if (i == 2 && !cfg_oc_c) continue;
      if (!(hal_data->sdo_oc[i] = ecrt_slave_config_create_sdo_request(slave->config, OBJ_DIAG, i + 1, 1))) {
        rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "ecrt_slave_config_create_sdo_request(0x%04X:%02X) failed for slave %s.%s\n",
          OBJ_DIAG, i + 1, master->name, slave->name);
        return -EIO;
      }
      ecrt_sdo_request_timeout(hal_data->sdo_oc[i], 500);
    }
  }

  return 0;
}

/* ======================================================================
 * Read (EtherCAT -> HAL)
 * ====================================================================== */
static void lcec_ep5101_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ep5101_data_t *hal_data = (lcec_ep5101_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  uint32_t raw, latch;
  int32_t delta;
  hal_bit_t *oc_pins[3];
  int i;

  if (!slave->state.operational) {
    hal_data->last_operational = 0;
    hal_data->vel_valid = 0;
    hal_data->latch_c_block   = 0;
    hal_data->latch_ext_block = 0;
    return;
  }

  /* One-time sanity check: status/control words must start at bit 0 */
  if (!hal_data->layout_checked) {
    hal_data->layout_checked = 1;
    if (hal_data->status_bp != 0 || hal_data->ctrl_bp != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "slave %s.%s: unexpected PDO layout (status bp=%u, ctrl bp=%u), check 'ethercat pdos'\n",
        master->name, slave->name, hal_data->status_bp, hal_data->ctrl_bp);
    }
  }

  /* Scale update, avoid division by zero */
  if (*(hal_data->pos_scale) != hal_data->old_scale) {
    if (*(hal_data->pos_scale) < 1e-20 && *(hal_data->pos_scale) > -1e-20) {
      *(hal_data->pos_scale) = 1.0;
    }
    hal_data->old_scale = *(hal_data->pos_scale);
    hal_data->scale = 1.0 / *(hal_data->pos_scale);
  }

  status = EC_READ_U16(&pd[hal_data->status_os]);
  raw    = EC_READ_U32(&pd[hal_data->count_os]);
  latch  = EC_READ_U32(&pd[hal_data->latch_os]);

  /* Status bits */
  *(hal_data->status_raw)       = status;
  *(hal_data->latch_c_valid)    = (status & STS_LATCH_C_VALID)   ? 1 : 0;
  *(hal_data->latch_ext_valid)  = (status & STS_LATCH_EXT_VALID) ? 1 : 0;
  *(hal_data->set_cnt_done)     = (status & STS_SET_CNT_DONE)    ? 1 : 0;
  *(hal_data->underflow)        = (status & STS_UNDERFLOW)       ? 1 : 0;
  *(hal_data->overflow)         = (status & STS_OVERFLOW)        ? 1 : 0;
  *(hal_data->inext)            = (status & STS_INPUT_STATUS)    ? 1 : 0;
  *(hal_data->open_circuit)     = (status & STS_OPEN_CIRCUIT)    ? 1 : 0;
  *(hal_data->extrap_stall)     = (status & STS_EXTRAP_STALL)    ? 1 : 0;
  *(hal_data->input_a)          = (status & STS_INPUT_A)         ? 1 : 0;
  *(hal_data->input_b)          = (status & STS_INPUT_B)         ? 1 : 0;
  *(hal_data->input_c)          = (status & STS_INPUT_C)         ? 1 : 0;
  *(hal_data->input_gate)       = (status & STS_INPUT_GATE)      ? 1 : 0;
  *(hal_data->ext_latch_status) = (status & STS_EXT_LATCH)       ? 1 : 0;
  *(hal_data->sync_error)       = (status & STS_SYNC_ERROR)      ? 1 : 0;
  *(hal_data->txpdo_state)      = (status & STS_TXPDO_STATE)     ? 1 : 0;
  *(hal_data->txpdo_toggle)     = (status & STS_TXPDO_TOGGLE)    ? 1 : 0;
  *(hal_data->error) = (status & (STS_OPEN_CIRCUIT | STS_TXPDO_STATE | STS_SYNC_ERROR)) ? 1 : 0;

  *(hal_data->raw_count) = (hal_s32_t)raw;

  /* Signed velocity from the hardware counter delta, first-order low-pass.
   * Uses its own reference so index latch / enc-reset do not disturb it;
   * the cycle in which a counter preset is acknowledged is skipped.       */
  {
    int jump = *(hal_data->set_raw_count) && (status & STS_SET_CNT_DONE);
    if (hal_data->vel_valid && !jump && period > 0) {
      double dt = (double)period * 1e-9;
      double tc = *(hal_data->velocity_tc);
      double v  = (double)(int32_t)(raw - hal_data->vel_last_raw) * hal_data->scale / dt;
      if (tc < 0.0) tc = 0.0;
      hal_data->vel_filt += (v - hal_data->vel_filt) * dt / (tc + dt);
    }
    hal_data->vel_last_raw = raw;
    hal_data->vel_valid    = 1;
    *(hal_data->velocity)  = hal_data->vel_filt;
  }

  /* (Re)start: take current counter as reference without moving enc-count */
  if (!hal_data->last_operational) {
    hal_data->last_raw = raw;
  }
  hal_data->last_operational = 1;

  /* Initialization / reset: enc-count = 0 */
  if (hal_data->do_init || *(hal_data->reset)) {
    hal_data->do_init  = 0;
    hal_data->last_raw = raw;
    hal_data->accum    = 0;
  }

  /* Counter preset acknowledged: counter jumped to set value, keep enc-count continuous */
  if (*(hal_data->set_raw_count) && (status & STS_SET_CNT_DONE)) {
    hal_data->last_raw = raw;
    *(hal_data->set_raw_count) = 0;
  }

  /* Release latch block once the terminal has cleared "latch valid" */
  if (hal_data->latch_c_block   && !(status & STS_LATCH_C_VALID))   hal_data->latch_c_block   = 0;
  if (hal_data->latch_ext_block && !(status & STS_LATCH_EXT_VALID)) hal_data->latch_ext_block = 0;

  /* Index / external latch: enc-count = 0 at the latched position */
  if (*(hal_data->ena_latch_c) && !hal_data->latch_c_block && (status & STS_LATCH_C_VALID)) {
    *(hal_data->raw_latch) = (hal_s32_t)latch;
    hal_data->last_raw = latch;
    hal_data->accum    = 0;
    *(hal_data->ena_latch_c) = 0;
    hal_data->latch_c_block  = 1;
  }
  if ((*(hal_data->ena_latch_ext_pos) || *(hal_data->ena_latch_ext_neg)) &&
      !hal_data->latch_ext_block && (status & STS_LATCH_EXT_VALID)) {
    *(hal_data->raw_latch) = (hal_s32_t)latch;
    hal_data->last_raw = latch;
    hal_data->accum    = 0;
    *(hal_data->ena_latch_ext_pos) = 0;
    *(hal_data->ena_latch_ext_neg) = 0;
    hal_data->latch_ext_block    = 1;
  }

  /* Wrap-safe delta of the 32-bit counter, accumulated in 64 bit */
  delta = (int32_t)(raw - hal_data->last_raw);
  hal_data->last_raw = raw;
  hal_data->accum += delta;

  *(hal_data->count) = (hal_s32_t)hal_data->accum;
  *(hal_data->pos)   = (double)hal_data->accum * hal_data->scale;

  /* Optional frequency / period */
  if (hal_data->freq_enabled) {
    uint32_t f = EC_READ_U32(&pd[hal_data->freq_os]);
    *(hal_data->raw_frequency) = f;
    *(hal_data->frequency)     = (double)f * hal_data->freq_unit;
  }
  if (hal_data->period_enabled) {
    uint32_t t = EC_READ_U32(&pd[hal_data->period_os]);
    *(hal_data->raw_period) = t;
    *(hal_data->period)     = (double)t * hal_data->period_unit;
  }

  /* Diagnostic SDO reads, throttled */
  if (++hal_data->sdo_read_timer >= SDO_READ_INTERVAL) {
    hal_data->sdo_read_timer = 0;
    oc_pins[0] = hal_data->open_circuit_a;
    oc_pins[1] = hal_data->open_circuit_b;
    oc_pins[2] = hal_data->open_circuit_c;
    for (i = 0; i < 3; i++) {
      if (!hal_data->sdo_oc[i]) continue;
      if (hal_data->sdo_backoff[i]) {
        hal_data->sdo_backoff[i]--;
        continue;
      }
      hal_data->sdo_backoff[i] = lcec_ep5101_poll_sdo(slave, hal_data->sdo_oc[i], oc_pins[i],
                                        (uint8_t)(i + 1), &hal_data->sdo_err_cnt[i],
                                        &hal_data->sdo_quiet[i]);
    }
  }
}

/* ======================================================================
 * Write (HAL -> EtherCAT)
 * ====================================================================== */
static void lcec_ep5101_write(lcec_slave_t *slave, long period) {
  lcec_ep5101_data_t *hal_data = (lcec_ep5101_data_t *)slave->hal_data;
  uint8_t *pd = slave->master->process_data;
  uint16_t ctrl = 0;

  if (!slave->state.operational) return;

  if (*(hal_data->ena_latch_c)       && !hal_data->latch_c_block)   ctrl |= CTRL_ENA_LATCH_C;
  if (*(hal_data->ena_latch_ext_pos) && !hal_data->latch_ext_block) ctrl |= CTRL_ENA_LATCH_EXT_POS;
  if (*(hal_data->ena_latch_ext_neg) && !hal_data->latch_ext_block) ctrl |= CTRL_ENA_LATCH_EXT_NEG;
  if (*(hal_data->set_raw_count))                                   ctrl |= CTRL_SET_COUNTER;

  /* Bits 4..15 are gap bits in 0x1601 and are written as zero */
  EC_WRITE_U16(&pd[hal_data->ctrl_os], ctrl);
  EC_WRITE_S32(&pd[hal_data->setval_os], *(hal_data->set_raw_count_val));
}
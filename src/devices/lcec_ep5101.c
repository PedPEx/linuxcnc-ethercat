// SPDX-License-Identifier: GPL-2.0-or-later
//
/// @file
/// @brief Driver for Beckhoff EP5101-0011
///        1-channel incremental encoder interface, IP67, D-SUB15
///
/// PDO mapping:
///   SM2 RxPDO 0x1601 "ENC Control"    6 bytes
///   SM3 TxPDO 0x1A01 "ENC Status"    10 bytes
///
/// All 0x8000 device configuration registers are exposed as modParams
/// in ethercat-conf.xml, e.g.:
///   <slave idx="9" type="EP5101" name="enc1">
///     <modParam name="enableCReset"    value="1"/>
///     <modParam name="enableUpDown"    value="1"/>
///     <modParam name="gatePolarity"    value="0"/>
///     <modParam name="disableFilter"   value="0"/>
///     <modParam name="enableMicroInc"  value="0"/>
///     <modParam name="revRotation"     value="0"/>
///   </slave>

#include "../lcec.h"

// ── Forward declarations ─────────────────────────────────────────────────────
static int  lcec_ep5101_init(int comp_id, lcec_slave_t *slave);
static void lcec_ep5101_read(lcec_slave_t *slave, long period);
static void lcec_ep5101_write(lcec_slave_t *slave, long period);

// ── modParam IDs ─────────────────────────────────────────────────────────────
// Each ID maps to one SDO subindex in object 0x8000.
enum {
    MP_ENABLE_C_RESET    = 1,  // 0x8000:01  Enable C reset
    MP_ENABLE_EXT_RESET  = 2,  // 0x8000:02  Enable extern reset
    MP_ENABLE_UP_DOWN    = 3,  // 0x8000:03  Enable up/down counter
    MP_GATE_POLARITY     = 4,  // 0x8000:04  Gate polarity
    MP_DISABLE_FILTER    = 5,  // 0x8000:08  Disable filter
    MP_ENABLE_MICRO_INC  = 6,  // 0x8000:0A  Enable micro increments
    MP_REV_ROTATION      = 7,  // 0x8000:0E  Reversion of rotation
    MP_EXT_RESET_POL     = 8,  // 0x8000:10  Extern reset polarity
    MP_FREQ_WINDOW       = 9,  // 0x8000:11  Frequency window
    MP_FREQ_SCALING      = 10, // 0x8000:13  Frequency scaling
    MP_PERIOD_SCALING    = 11, // 0x8000:14  Period scaling
    MP_FREQ_RESOLUTION   = 12, // 0x8000:15  Frequency resolution
    MP_PERIOD_RESOLUTION = 13, // 0x8000:16  Period resolution
    MP_WAIT_TIME         = 14, // 0x8000:17  Frequency wait time
};

// ── modParam descriptor table ────────────────────────────────────────────────
// Columns: name, id, type, default_value, config_comment
static lcec_modparam_desc_t lcec_ep5101_modparams[] = {
    {"enableCReset",     MP_ENABLE_C_RESET,    MODPARAM_TYPE_BIT,
     "1",  "0x8000:01 Enable C reset (index pulse resets counter)"},
    {"enableExtReset",   MP_ENABLE_EXT_RESET,  MODPARAM_TYPE_BIT,
     "0",  "0x8000:02 Enable extern reset via external input"},
    {"enableUpDown",     MP_ENABLE_UP_DOWN,    MODPARAM_TYPE_BIT,
     "1",  "0x8000:03 Enable up/down counter (direction from B-track)"},
    {"gatePolarity",     MP_GATE_POLARITY,     MODPARAM_TYPE_BIT,
     "0",  "0x8000:04 Gate polarity (0=active LOW, 1=active HIGH)"},
    {"disableFilter",    MP_DISABLE_FILTER,    MODPARAM_TYPE_BIT,
     "0",  "0x8000:08 Disable input filter (0=filter active, 1=disabled)"},
    {"enableMicroInc",   MP_ENABLE_MICRO_INC,  MODPARAM_TYPE_BIT,
     "0",  "0x8000:0A Enable micro increments"},
    {"revRotation",      MP_REV_ROTATION,      MODPARAM_TYPE_BIT,
     "0",  "0x8000:0E Reversion of rotation (0=normal, 1=inverted)"},
    {"extResetPolarity", MP_EXT_RESET_POL,     MODPARAM_TYPE_BIT,
     "0",  "0x8000:10 Extern reset polarity (0=rising edge)"},
    {"freqWindow",       MP_FREQ_WINDOW,       MODPARAM_TYPE_U32,
     "0",  "0x8000:11 Frequency window"},
    {"freqScaling",      MP_FREQ_SCALING,      MODPARAM_TYPE_U32,
     "0",  "0x8000:13 Frequency scaling"},
    {"periodScaling",    MP_PERIOD_SCALING,    MODPARAM_TYPE_U32,
     "0",  "0x8000:14 Period scaling"},
    {"freqResolution",   MP_FREQ_RESOLUTION,   MODPARAM_TYPE_U32,
     "0",  "0x8000:15 Frequency resolution"},
    {"periodResolution", MP_PERIOD_RESOLUTION, MODPARAM_TYPE_U32,
     "0",  "0x8000:16 Period resolution"},
    {"waitTime",         MP_WAIT_TIME,         MODPARAM_TYPE_U32,
     "0",  "0x8000:17 Frequency wait time"},
    {NULL},
};

// ── Self-registration ────────────────────────────────────────────────────────
static lcec_typelist_t types[] = {
    {"EP5101", LCEC_BECKHOFF_VID, 0x13ED4052, 0, NULL, lcec_ep5101_init, lcec_ep5101_modparams},
    {NULL},
};
ADD_TYPES(types);

// ── Status word bit masks (TxPDO 0x1A01, bytes 0-1) ─────────────────────────
#define EP5101_STS_LATCH_C_VALID   (1u <<  0)  // 0x6000:01
#define EP5101_STS_LATCH_EXT_VALID (1u <<  1)  // 0x6000:02
#define EP5101_STS_CNT_SET_DONE    (1u <<  2)  // 0x6000:03
#define EP5101_STS_UNDERFLOW       (1u <<  3)  // 0x6000:04
#define EP5101_STS_OVERFLOW        (1u <<  4)  // 0x6000:05
#define EP5101_STS_INPUT_STATUS    (1u <<  5)  // 0x6000:06
#define EP5101_STS_OPEN_CIRCUIT    (1u <<  6)  // 0x6000:07
#define EP5101_STS_EXTRAP_STALL    (1u <<  7)  // 0x6000:08
#define EP5101_STS_INPUT_A         (1u <<  8)  // 0x6000:09
#define EP5101_STS_INPUT_B         (1u <<  9)  // 0x6000:0A
#define EP5101_STS_INPUT_C         (1u << 10)  // 0x6000:0B
#define EP5101_STS_INPUT_GATE      (1u << 11)  // 0x6000:0C
#define EP5101_STS_EXT_LATCH       (1u << 12)  // 0x6000:0D
#define EP5101_STS_SYNC_ERROR      (1u << 13)  // 0x1C32:20
#define EP5101_STS_TXPDO_STATE     (1u << 14)  // 0x1801:09
#define EP5101_STS_TXPDO_TOGGLE    (1u << 15)  // gap bit

// ── Control word bit masks (RxPDO 0x1601, bytes 0-1) ────────────────────────
#define EP5101_CTRL_ENA_LATCH_C       (1u << 0)  // 0x7000:01
#define EP5101_CTRL_ENA_LATCH_EXT_POS (1u << 1)  // 0x7000:02
#define EP5101_CTRL_SET_COUNTER       (1u << 2)  // 0x7000:03
#define EP5101_CTRL_ENA_LATCH_EXT_NEG (1u << 3)  // 0x7000:04

// ── PDO object addresses ─────────────────────────────────────────────────────
#define EP5101_STATUS_IDX    0x6000
#define EP5101_STATUS_SIDX   0x01
#define EP5101_COUNT_IDX     0x6000
#define EP5101_COUNT_SIDX    0x11
#define EP5101_LATCH_IDX     0x6000
#define EP5101_LATCH_SIDX    0x12
#define EP5101_CTRL_IDX      0x7000
#define EP5101_CTRL_SIDX     0x01
#define EP5101_SETCNT_IDX    0x7000
#define EP5101_SETCNT_SIDX   0x11

// ── PDO entry definitions ─────────────────────────────────────────────────────
static ec_pdo_entry_info_t lcec_ep5101_out[] = {
    {0x7000, 0x01,  1},  // Enable latch C
    {0x7000, 0x02,  1},  // Enable latch extern pos edge
    {0x7000, 0x03,  1},  // Set counter
    {0x7000, 0x04,  1},  // Enable latch extern neg edge
    {0x0000, 0x00,  4},  // Padding bits 4-7
    {0x0000, 0x00,  8},  // Padding byte 1
    {0x7000, 0x11, 32},  // Set counter value
};

static ec_pdo_entry_info_t lcec_ep5101_in[] = {
    {0x6000, 0x01,  1},  // Latch C valid
    {0x6000, 0x02,  1},  // Latch extern valid
    {0x6000, 0x03,  1},  // Set counter done
    {0x6000, 0x04,  1},  // Counter underflow
    {0x6000, 0x05,  1},  // Counter overflow
    {0x6000, 0x06,  1},  // Status of input status
    {0x6000, 0x07,  1},  // Open circuit
    {0x6000, 0x08,  1},  // Extrapolation stall
    {0x6000, 0x09,  1},  // Status of input A
    {0x6000, 0x0A,  1},  // Status of input B
    {0x6000, 0x0B,  1},  // Status of input C
    {0x6000, 0x0C,  1},  // Status of input gate
    {0x6000, 0x0D,  1},  // Status of extern latch
    {0x1C32, 0x20,  1},  // Sync error
    {0x1801, 0x09,  1},  // TxPDO State
    {0x0000, 0x00,  1},  // TxPDO Toggle (gap)
    {0x6000, 0x11, 32},  // Counter value
    {0x6000, 0x12, 32},  // Latch value
};

static ec_pdo_info_t lcec_ep5101_pdos_out[] = {
    {0x1601, 7, lcec_ep5101_out},
};

static ec_pdo_info_t lcec_ep5101_pdos_in[] = {
    {0x1A01, 18, lcec_ep5101_in},
};

static ec_sync_info_t lcec_ep5101_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,                 EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL,                 EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, lcec_ep5101_pdos_out, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, lcec_ep5101_pdos_in,  EC_WD_DISABLE},
    {0xff},
};

// ── HAL data structure ───────────────────────────────────────────────────────
typedef struct {
    unsigned int status_pdo_os;
    unsigned int status_pdo_bp;
    unsigned int count_pdo_os;
    unsigned int latch_pdo_os;
    unsigned int ctrl_pdo_os;
    unsigned int ctrl_pdo_bp;
    unsigned int setcnt_pdo_os;

    // Status bit pins (OUT)
    hal_bit_t   *latch_c_valid;
    hal_bit_t   *latch_ext_valid;
    hal_bit_t   *cnt_set_done;
    hal_bit_t   *underflow;
    hal_bit_t   *overflow;
    hal_bit_t   *input_status;
    hal_bit_t   *open_circuit;
    hal_bit_t   *extrap_stall;
    hal_bit_t   *input_a;
    hal_bit_t   *input_b;
    hal_bit_t   *input_c;
    hal_bit_t   *input_gate;
    hal_bit_t   *ext_latch_status;
    hal_bit_t   *sync_error;
    hal_bit_t   *txpdo_state;
    hal_bit_t   *txpdo_toggle;

    // Data pins (OUT)
    hal_u32_t   *status_raw;
    hal_s32_t   *raw_count;
    hal_s32_t   *count;
    hal_s32_t   *latch_val;
    hal_float_t *pos;
    hal_float_t *pos_scale;

    // Control bit pins (IO)
    hal_bit_t   *ena_latch_c;
    hal_bit_t   *ena_latch_ext_pos;
    hal_bit_t   *set_counter;
    hal_bit_t   *ena_latch_ext_neg;

    // Preset / reset pins
    hal_s32_t   *set_raw_count_val;
    hal_bit_t   *set_raw_count;
    hal_bit_t   *reset;

    // Internal state
    int32_t     last_count;
    int64_t     accum;
    int         do_init;
    double      scale;
    double      old_scale;
    int         last_operational;
} lcec_ep5101_data_t;

// ── HAL pin descriptor table ─────────────────────────────────────────────────
static const lcec_pindesc_t slave_pins[] = {
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, latch_c_valid),
                "%s.%s.%s.enc-latch-c-valid"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, latch_ext_valid),
                "%s.%s.%s.enc-latch-ext-valid"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, cnt_set_done),
                "%s.%s.%s.enc-cnt-set-done"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, underflow),
                "%s.%s.%s.enc-underflow"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, overflow),
                "%s.%s.%s.enc-overflow"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_status),
                "%s.%s.%s.enc-input-status"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, open_circuit),
                "%s.%s.%s.enc-open-circuit"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, extrap_stall),
                "%s.%s.%s.enc-extrap-stall"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_a),
                "%s.%s.%s.enc-input-a"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_b),
                "%s.%s.%s.enc-input-b"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_c),
                "%s.%s.%s.enc-input-c"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, input_gate),
                "%s.%s.%s.enc-input-gate"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, ext_latch_status),
                "%s.%s.%s.enc-ext-latch-status"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, sync_error),
                "%s.%s.%s.enc-sync-error"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, txpdo_state),
                "%s.%s.%s.enc-txpdo-state"},
    {HAL_BIT,   HAL_OUT, offsetof(lcec_ep5101_data_t, txpdo_toggle),
                "%s.%s.%s.enc-txpdo-toggle"},
    {HAL_U32,   HAL_OUT, offsetof(lcec_ep5101_data_t, status_raw),
                "%s.%s.%s.enc-status-raw"},
    {HAL_S32,   HAL_OUT, offsetof(lcec_ep5101_data_t, raw_count),
                "%s.%s.%s.enc-raw-count"},
    {HAL_S32,   HAL_OUT, offsetof(lcec_ep5101_data_t, count),
                "%s.%s.%s.enc-count"},
    {HAL_S32,   HAL_OUT, offsetof(lcec_ep5101_data_t, latch_val),
                "%s.%s.%s.enc-latch-val"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_ep5101_data_t, pos),
                "%s.%s.%s.enc-pos"},
    {HAL_FLOAT, HAL_IO,  offsetof(lcec_ep5101_data_t, pos_scale),
                "%s.%s.%s.enc-pos-scale"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_c),
                "%s.%s.%s.enc-ena-latch-c"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_ext_pos),
                "%s.%s.%s.enc-ena-latch-ext-pos"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, set_counter),
                "%s.%s.%s.enc-set-counter"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_ext_neg),
                "%s.%s.%s.enc-ena-latch-ext-neg"},
    {HAL_S32,   HAL_IN,  offsetof(lcec_ep5101_data_t, set_raw_count_val),
                "%s.%s.%s.enc-set-raw-count-val"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, set_raw_count),
                "%s.%s.%s.enc-set-raw-count"},
    {HAL_BIT,   HAL_IN,  offsetof(lcec_ep5101_data_t, reset),
                "%s.%s.%s.enc-reset"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// ── modParam SDO write helper ─────────────────────────────────────────────────
// Writes only the 0x8000 SDOs that are explicitly set in the XML.
// Parameters not listed in the XML keep their device power-on defaults.
static int lcec_ep5101_write_modparams(lcec_slave_t *slave) {
    lcec_master_t        *master = slave->master;
    lcec_slave_modparam_t *p;

    for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
        switch (p->id) {
            case MP_ENABLE_C_RESET:
                if (lcec_write_sdo8(slave, 0x8000, 0x01,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_ENABLE_EXT_RESET:
                if (lcec_write_sdo8(slave, 0x8000, 0x02,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_ENABLE_UP_DOWN:
                if (lcec_write_sdo8(slave, 0x8000, 0x03,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_GATE_POLARITY:
                if (lcec_write_sdo8(slave, 0x8000, 0x04,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_DISABLE_FILTER:
                if (lcec_write_sdo8(slave, 0x8000, 0x08,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_ENABLE_MICRO_INC:
                if (lcec_write_sdo8(slave, 0x8000, 0x0A,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_REV_ROTATION:
                if (lcec_write_sdo8(slave, 0x8000, 0x0E,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_EXT_RESET_POL:
                if (lcec_write_sdo8(slave, 0x8000, 0x10,
                                     p->value.bit ? 1 : 0) != 0) goto sdo_err;
                break;
            case MP_FREQ_WINDOW:
                if (lcec_write_sdo16(slave, 0x8000, 0x11,
                                      (uint16_t)p->value.u32) != 0) goto sdo_err;
                break;
            case MP_FREQ_SCALING:
                if (lcec_write_sdo16(slave, 0x8000, 0x13,
                                      (uint16_t)p->value.u32) != 0) goto sdo_err;
                break;
            case MP_PERIOD_SCALING:
                if (lcec_write_sdo16(slave, 0x8000, 0x14,
                                      (uint16_t)p->value.u32) != 0) goto sdo_err;
                break;
            case MP_FREQ_RESOLUTION:
                if (lcec_write_sdo16(slave, 0x8000, 0x15,
                                      (uint16_t)p->value.u32) != 0) goto sdo_err;
                break;
            case MP_PERIOD_RESOLUTION:
                if (lcec_write_sdo16(slave, 0x8000, 0x16,
                                      (uint16_t)p->value.u32) != 0) goto sdo_err;
                break;
            case MP_WAIT_TIME:
                if (lcec_write_sdo16(slave, 0x8000, 0x17,
                                      (uint16_t)p->value.u32) != 0) goto sdo_err;
                break;
            default:
                rtapi_print_msg(RTAPI_MSG_ERR,
                    LCEC_MSG_PFX "EP5101 %s.%s: unknown modparam id %d\n",
                    master->name, slave->name, p->id);
                return -1;
        }
    }
    return 0;

sdo_err:
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "EP5101 %s.%s: modparam SDO write failed (id=%d, name=%s)\n",
        master->name, slave->name, p->id, p->name);
    return -1;
}

// ── Init ─────────────────────────────────────────────────────────────────────
static int lcec_ep5101_init(int comp_id, lcec_slave_t *slave) {
    lcec_master_t      *master = slave->master;
    lcec_ep5101_data_t *hal_data;
    int err;

    slave->proc_read  = lcec_ep5101_read;
    slave->proc_write = lcec_ep5101_write;

    hal_data = LCEC_HAL_ALLOCATE(lcec_ep5101_data_t);
    slave->hal_data  = hal_data;
    slave->sync_info = lcec_ep5101_syncs;

    // ── PDO assignment SDOs ──────────────────────────────────────────────────
    // SM2 → 0x1601 "ENC Control"
    lcec_write_sdo8 (slave, 0x1C12, 0x00, 0x00);
    lcec_write_sdo16(slave, 0x1C12, 0x01, 0x1601);
    lcec_write_sdo8 (slave, 0x1C12, 0x00, 0x01);

    // SM3 → 0x1A01 "ENC Status"
    lcec_write_sdo8 (slave, 0x1C13, 0x00, 0x00);
    lcec_write_sdo16(slave, 0x1C13, 0x01, 0x1A01);
    lcec_write_sdo8 (slave, 0x1C13, 0x00, 0x01);

    // ── 0x8000 device config from modparams ─────────────────────────────────
    if (lcec_ep5101_write_modparams(slave) != 0)
        return -EIO;

    // ── PDO offset registration ──────────────────────────────────────────────
    lcec_pdo_init(slave, EP5101_STATUS_IDX, EP5101_STATUS_SIDX,
                  &hal_data->status_pdo_os, &hal_data->status_pdo_bp);
    lcec_pdo_init(slave, EP5101_COUNT_IDX,  EP5101_COUNT_SIDX,
                  &hal_data->count_pdo_os,  NULL);
    lcec_pdo_init(slave, EP5101_LATCH_IDX,  EP5101_LATCH_SIDX,
                  &hal_data->latch_pdo_os,  NULL);
    lcec_pdo_init(slave, EP5101_CTRL_IDX,   EP5101_CTRL_SIDX,
                  &hal_data->ctrl_pdo_os,   &hal_data->ctrl_pdo_bp);
    lcec_pdo_init(slave, EP5101_SETCNT_IDX, EP5101_SETCNT_SIDX,
                  &hal_data->setcnt_pdo_os, NULL);

    // ── HAL pins ─────────────────────────────────────────────────────────────
    if ((err = lcec_pin_newf_list(hal_data, slave_pins,
                                  LCEC_MODULE_NAME,
                                  master->name,
                                  slave->name)) != 0) {
        return err;
    }

    *(hal_data->pos_scale)     = 1.0;
    hal_data->old_scale        = 2.0;
    hal_data->scale            = 1.0;
    hal_data->do_init          = 1;
    hal_data->last_operational = 0;

    return 0;
}

// ── Read callback (realtime) ─────────────────────────────────────────────────
static void lcec_ep5101_read(lcec_slave_t *slave, long period) {
    lcec_master_t      *master   = slave->master;
    lcec_ep5101_data_t *hal_data = (lcec_ep5101_data_t *)slave->hal_data;
    uint8_t            *pd       = master->process_data;
    uint16_t status;
    int32_t  raw, delta;

    if (!slave->state.operational) {
        hal_data->last_operational = 0;
        return;
    }

    if (*(hal_data->pos_scale) != hal_data->old_scale) {
        if (*(hal_data->pos_scale) == 0.0) *(hal_data->pos_scale) = 1.0;
        hal_data->scale     = 1.0 / *(hal_data->pos_scale);
        hal_data->old_scale = *(hal_data->pos_scale);
    }

    status = EC_READ_U16(pd + hal_data->status_pdo_os);
    raw    = EC_READ_S32(pd + hal_data->count_pdo_os);

    if (hal_data->do_init || !hal_data->last_operational) {
        hal_data->last_count = raw;
        hal_data->accum      = raw;
        hal_data->do_init    = 0;
    }
    hal_data->last_operational = 1;

    if (*(hal_data->reset)) {
        hal_data->accum      = 0;
        hal_data->last_count = raw;
    }

    if (*(hal_data->set_raw_count)) {
        *(hal_data->set_raw_count) = 0;
        hal_data->accum      = *(hal_data->set_raw_count_val);
        hal_data->last_count = raw;
        EC_WRITE_S32(pd + hal_data->setcnt_pdo_os,
                     *(hal_data->set_raw_count_val));
        EC_WRITE_U16(pd + hal_data->ctrl_pdo_os,
                     EC_READ_U16(pd + hal_data->ctrl_pdo_os) |
                     EP5101_CTRL_SET_COUNTER);
    }

    delta = raw - hal_data->last_count;
    hal_data->accum      += delta;
    hal_data->last_count  = raw;

    *(hal_data->status_raw) = status;
    *(hal_data->raw_count)  = raw;
    *(hal_data->count)      = (int32_t)hal_data->accum;
    *(hal_data->latch_val)  = EC_READ_S32(pd + hal_data->latch_pdo_os);
    *(hal_data->pos)        = (double)hal_data->accum * hal_data->scale;

    *(hal_data->latch_c_valid)    = (status & EP5101_STS_LATCH_C_VALID)   ? 1 : 0;
    *(hal_data->latch_ext_valid)  = (status & EP5101_STS_LATCH_EXT_VALID) ? 1 : 0;
    *(hal_data->cnt_set_done)     = (status & EP5101_STS_CNT_SET_DONE)    ? 1 : 0;
    *(hal_data->underflow)        = (status & EP5101_STS_UNDERFLOW)       ? 1 : 0;
    *(hal_data->overflow)         = (status & EP5101_STS_OVERFLOW)        ? 1 : 0;
    *(hal_data->input_status)     = (status & EP5101_STS_INPUT_STATUS)    ? 1 : 0;
    *(hal_data->open_circuit)     = (status & EP5101_STS_OPEN_CIRCUIT)    ? 1 : 0;
    *(hal_data->extrap_stall)     = (status & EP5101_STS_EXTRAP_STALL)    ? 1 : 0;
    *(hal_data->input_a)          = (status & EP5101_STS_INPUT_A)         ? 1 : 0;
    *(hal_data->input_b)          = (status & EP5101_STS_INPUT_B)         ? 1 : 0;
    *(hal_data->input_c)          = (status & EP5101_STS_INPUT_C)         ? 1 : 0;
    *(hal_data->input_gate)       = (status & EP5101_STS_INPUT_GATE)      ? 1 : 0;
    *(hal_data->ext_latch_status) = (status & EP5101_STS_EXT_LATCH)       ? 1 : 0;
    *(hal_data->sync_error)       = (status & EP5101_STS_SYNC_ERROR)      ? 1 : 0;
    *(hal_data->txpdo_state)      = (status & EP5101_STS_TXPDO_STATE)     ? 1 : 0;
    *(hal_data->txpdo_toggle)     = (status & EP5101_STS_TXPDO_TOGGLE)    ? 1 : 0;
}

// ── Write callback (realtime) ────────────────────────────────────────────────
static void lcec_ep5101_write(lcec_slave_t *slave, long period) {
    lcec_master_t      *master   = slave->master;
    lcec_ep5101_data_t *hal_data = (lcec_ep5101_data_t *)slave->hal_data;
    uint8_t            *pd       = master->process_data;
    uint16_t ctrl = 0;

    if (!slave->state.operational) return;

    if (*(hal_data->ena_latch_c))       ctrl |= EP5101_CTRL_ENA_LATCH_C;
    if (*(hal_data->ena_latch_ext_pos)) ctrl |= EP5101_CTRL_ENA_LATCH_EXT_POS;
    if (*(hal_data->set_counter))       ctrl |= EP5101_CTRL_SET_COUNTER;
    if (*(hal_data->ena_latch_ext_neg)) ctrl |= EP5101_CTRL_ENA_LATCH_EXT_NEG;

    EC_WRITE_U16(pd + hal_data->ctrl_pdo_os, ctrl);
}

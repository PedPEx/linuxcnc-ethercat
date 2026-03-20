// SPDX-License-Identifier: GPL-2.0-or-later
//
/// @file
/// @brief Driver for Beckhoff EP5101-0011
///        1-channel incremental encoder interface, IP67, D-SUB15
///
/// PDO mapping:
///   SM2 RxPDO 0x1601 "ENC Control"       6 bytes
///             0x7000:01..04  Control bits (4x 1-bit)
///             0x7000:11      Set counter value (UDINT 32-bit)
///
///   SM3 TxPDO 0x1A01 "ENC Status"        10 bytes
///             0x6000:01..0D  Status bits  (13x 1-bit)
///             0x1C32:20      Sync error   (1-bit)
///             0x1801:09      TxPDO State  (1-bit)
///             gap            TxPDO Toggle (1-bit)
///             0x6000:11      Counter value (UDINT 32-bit)
///             0x6000:12      Latch value   (UDINT 32-bit)

#include "../lcec.h"

// ── Forward declarations ─────────────────────────────────────────────────────
static int  lcec_ep5101_init(int comp_id, lcec_slave_t *slave);
static void lcec_ep5101_read(lcec_slave_t *slave, long period);
static void lcec_ep5101_write(lcec_slave_t *slave, long period);

// ── Self-registration ────────────────────────────────────────────────────────
static lcec_typelist_t types[] = {
    {"EP5101", LCEC_BECKHOFF_VID, 0x13ED4052, 0, NULL, lcec_ep5101_init},
    {NULL},
};
ADD_TYPES(types);

// ── Status word bit masks (16-bit, TxPDO 0x1A01, bytes 0-1) ─────────────────
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

// ── Control word bit masks (16-bit, RxPDO 0x1601, bytes 0-1) ────────────────
#define EP5101_CTRL_ENA_LATCH_C       (1u << 0)  // 0x7000:01
#define EP5101_CTRL_ENA_LATCH_EXT_POS (1u << 1)  // 0x7000:02
#define EP5101_CTRL_SET_COUNTER       (1u << 2)  // 0x7000:03
#define EP5101_CTRL_ENA_LATCH_EXT_NEG (1u << 3)  // 0x7000:04

// ── PDO object addresses (used for lcec_pdo_init offset registration) ────────
#define EP5101_STATUS_IDX    0x6000
#define EP5101_STATUS_SIDX   0x01    // anchor: first status bit -> byte offset

#define EP5101_COUNT_IDX     0x6000
#define EP5101_COUNT_SIDX    0x11    // Counter value (UDINT 32-bit)

#define EP5101_LATCH_IDX     0x6000
#define EP5101_LATCH_SIDX    0x12    // Latch value (UDINT 32-bit)

#define EP5101_CTRL_IDX      0x7000
#define EP5101_CTRL_SIDX     0x01    // anchor: first control bit -> byte offset

#define EP5101_SETCNT_IDX    0x7000
#define EP5101_SETCNT_SIDX   0x11    // Set counter value (UDINT 32-bit)

// ── PDO entry definitions ─────────────────────────────────────────────────────
// RxPDO 0x1601 "ENC Control" -- 6 bytes total
//   Byte 0:  bits 0-3 used, bits 4-7 padding
//   Byte 1:  padding
//   Bytes 2-5: Set counter value (UDINT)
static ec_pdo_entry_info_t lcec_ep5101_out[] = {
    {0x7000, 0x01,  1},   // Enable latch C
    {0x7000, 0x02,  1},   // Enable latch extern on positive edge
    {0x7000, 0x03,  1},   // Set counter
    {0x7000, 0x04,  1},   // Enable latch extern on negative edge
    {0x0000, 0x00,  4},   // Padding (bits 4-7 of byte 0)
    {0x0000, 0x00,  8},   // Padding (byte 1)
    {0x7000, 0x11, 32},   // Set counter value
};

// TxPDO 0x1A01 "ENC Status" -- 10 bytes total
//   Bytes 0-1: Status word (16 x 1-bit entries)
//   Bytes 2-5: Counter value (UDINT)
//   Bytes 6-9: Latch value (UDINT)
static ec_pdo_entry_info_t lcec_ep5101_in[] = {
    {0x6000, 0x01,  1},   // Latch C valid
    {0x6000, 0x02,  1},   // Latch extern valid
    {0x6000, 0x03,  1},   // Set counter done
    {0x6000, 0x04,  1},   // Counter underflow
    {0x6000, 0x05,  1},   // Counter overflow
    {0x6000, 0x06,  1},   // Status of input status
    {0x6000, 0x07,  1},   // Open circuit
    {0x6000, 0x08,  1},   // Extrapolation stall
    {0x6000, 0x09,  1},   // Status of input A
    {0x6000, 0x0A,  1},   // Status of input B
    {0x6000, 0x0B,  1},   // Status of input C
    {0x6000, 0x0C,  1},   // Status of input gate
    {0x6000, 0x0D,  1},   // Status of extern latch
    {0x1C32, 0x20,  1},   // Sync error
    {0x1801, 0x09,  1},   // TxPDO State
    {0x0000, 0x00,  1},   // TxPDO Toggle (gap)
    {0x6000, 0x11, 32},   // Counter value
    {0x6000, 0x12, 32},   // Latch value
};

static ec_pdo_info_t lcec_ep5101_pdos_out[] = {
    {0x1601, 7, lcec_ep5101_out},
};

static ec_pdo_info_t lcec_ep5101_pdos_in[] = {
    {0x1A01, 18, lcec_ep5101_in},
};

static ec_sync_info_t lcec_ep5101_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,                  EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL,                  EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, lcec_ep5101_pdos_out,  EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, lcec_ep5101_pdos_in,   EC_WD_DISABLE},
    {0xff},
};

// ── HAL data structure ───────────────────────────────────────────────────────
typedef struct {
    // PDO byte offsets (set by lcec_pdo_init)
    unsigned int status_pdo_os;    // byte offset of status word (0x6000:01)
    unsigned int status_pdo_bp;    // bit position (should be 0)
    unsigned int count_pdo_os;     // byte offset of counter value
    unsigned int latch_pdo_os;     // byte offset of latch value
    unsigned int ctrl_pdo_os;      // byte offset of control word (0x7000:01)
    unsigned int ctrl_pdo_bp;      // bit position (should be 0)
    unsigned int setcnt_pdo_os;    // byte offset of set-counter value

    // -- TxPDO status bits -- HAL OUT pins ------------------------------------
    hal_bit_t   *latch_c_valid;     // Status bit  0: Latch C valid
    hal_bit_t   *latch_ext_valid;   // Status bit  1: Latch extern valid
    hal_bit_t   *cnt_set_done;      // Status bit  2: Set counter done
    hal_bit_t   *underflow;         // Status bit  3: Counter underflow
    hal_bit_t   *overflow;          // Status bit  4: Counter overflow
    hal_bit_t   *input_status;      // Status bit  5: Status of input (ext. signal OK)
    hal_bit_t   *open_circuit;      // Status bit  6: Open circuit detected
    hal_bit_t   *extrap_stall;      // Status bit  7: Extrapolation stall
    hal_bit_t   *input_a;           // Status bit  8: State of input A
    hal_bit_t   *input_b;           // Status bit  9: State of input B
    hal_bit_t   *input_c;           // Status bit 10: State of input C (index)
    hal_bit_t   *input_gate;        // Status bit 11: State of gate input
    hal_bit_t   *ext_latch_status;  // Status bit 12: State of extern latch input
    hal_bit_t   *sync_error;        // Status bit 13: EtherCAT sync error
    hal_bit_t   *txpdo_state;       // Status bit 14: TxPDO state
    hal_bit_t   *txpdo_toggle;      // Status bit 15: TxPDO toggle

    // -- TxPDO data -- HAL OUT pins -------------------------------------------
    hal_u32_t   *status_raw;        // Full 16-bit status word (raw, for debugging)
    hal_s32_t   *raw_count;         // Raw 32-bit counter value from device
    hal_s32_t   *count;             // Accumulated count (corrects for 32-bit rollover)
    hal_s32_t   *latch_val;         // Latch value
    hal_float_t *pos;               // Position = count / pos_scale
    hal_float_t *pos_scale;         // Scaling: increments per machine unit (IO)

    // -- RxPDO control bits -- HAL IO pins ------------------------------------
    hal_bit_t   *ena_latch_c;       // Control bit 0: Enable latch on C/index pulse
    hal_bit_t   *ena_latch_ext_pos; // Control bit 1: Enable latch on ext pos edge
    hal_bit_t   *set_counter;       // Control bit 2: Pulse HIGH to load preset
    hal_bit_t   *ena_latch_ext_neg; // Control bit 3: Enable latch on ext neg edge

    // -- Preset / reset -- HAL pins -------------------------------------------
    hal_s32_t   *set_raw_count_val; // IN:  value to preset counter to
    hal_bit_t   *set_raw_count;     // IO:  pulse HIGH to trigger preset (auto-clears)
    hal_bit_t   *reset;             // IN:  set HIGH to zero the software accumulator

    // -- Internal state -------------------------------------------------------
    int32_t     last_count;         // previous raw_count for delta calculation
    int64_t     accum;              // 64-bit software accumulator
    int         do_init;            // 1 = seed accumulator on first valid cycle
    double      scale;              // cached 1.0 / pos_scale
    double      old_scale;          // detect scale changes
    int         last_operational;   // edge detection for PREOP->OP transition
} lcec_ep5101_data_t;

// ── HAL pin descriptor table ─────────────────────────────────────────────────
static const lcec_pindesc_t slave_pins[] = {
    // Status bits (OUT)
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
    // Data (OUT)
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
    // Control bits (IO)
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_c),
                "%s.%s.%s.enc-ena-latch-c"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_ext_pos),
                "%s.%s.%s.enc-ena-latch-ext-pos"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, set_counter),
                "%s.%s.%s.enc-set-counter"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, ena_latch_ext_neg),
                "%s.%s.%s.enc-ena-latch-ext-neg"},
    // Preset / reset
    {HAL_S32,   HAL_IN,  offsetof(lcec_ep5101_data_t, set_raw_count_val),
                "%s.%s.%s.enc-set-raw-count-val"},
    {HAL_BIT,   HAL_IO,  offsetof(lcec_ep5101_data_t, set_raw_count),
                "%s.%s.%s.enc-set-raw-count"},
    {HAL_BIT,   HAL_IN,  offsetof(lcec_ep5101_data_t, reset),
                "%s.%s.%s.enc-reset"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// ── Init ─────────────────────────────────────────────────────────────────────
static int lcec_ep5101_init(int comp_id, lcec_slave_t *slave) {
    lcec_master_t      *master = slave->master;
    lcec_ep5101_data_t *hal_data;
    int err;

    // Callbacks
    slave->proc_read  = lcec_ep5101_read;
    slave->proc_write = lcec_ep5101_write;

    // HAL memory
    hal_data = LCEC_HAL_ALLOCATE(lcec_ep5101_data_t);
    slave->hal_data  = hal_data;

    // PDO sync manager configuration (0x1A01 / 0x1601)
    slave->sync_info = lcec_ep5101_syncs;

    // ── SDO startup sequence ─────────────────────────────────────────────────
    //
    // Switch SM2 RxPDO assignment to 0x1601 "ENC Control" (6 bytes)
    // "Enable PDO Assign: yes" allows reassignment via SDO
    lcec_write_sdo8 (slave, 0x1C12, 0x00, 0x00);    // clear assignment
    lcec_write_sdo16(slave, 0x1C12, 0x01, 0x1601);   // assign 0x1601
    lcec_write_sdo8 (slave, 0x1C12, 0x00, 0x01);     // count = 1

    // Switch SM3 TxPDO assignment to 0x1A01 "ENC Status" (10 bytes)
    lcec_write_sdo8 (slave, 0x1C13, 0x00, 0x00);
    lcec_write_sdo16(slave, 0x1C13, 0x01, 0x1A01);
    lcec_write_sdo8 (slave, 0x1C13, 0x00, 0x01);

    // ── Object 0x8000: Device configuration ─────────────────────────────────
    lcec_write_sdo8(slave, 0x8000, 0x01, 0x01);  // Enable C reset (index pulse)
    lcec_write_sdo8(slave, 0x8000, 0x02, 0x00);  // Disable extern reset
    lcec_write_sdo8(slave, 0x8000, 0x03, 0x01);  // Enable up/down counter
    lcec_write_sdo8(slave, 0x8000, 0x04, 0x00);  // Gate polarity (0 = active LOW)
    lcec_write_sdo8(slave, 0x8000, 0x08, 0x00);  // Input filter active
    lcec_write_sdo8(slave, 0x8000, 0x0A, 0x00);  // Normal rotation direction

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

    // ── Defaults ─────────────────────────────────────────────────────────────
    *(hal_data->pos_scale)     = 1.0;
    hal_data->old_scale        = 2.0;  // force scale recalc on first read
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
    int32_t  raw;
    int32_t  delta;

    // Guard: wait for OP state
    if (!slave->state.operational) {
        hal_data->last_operational = 0;
        return;
    }

    // Scale change detection
    if (*(hal_data->pos_scale) != hal_data->old_scale) {
        if (*(hal_data->pos_scale) == 0.0)
            *(hal_data->pos_scale) = 1.0;
        hal_data->scale     = 1.0 / *(hal_data->pos_scale);
        hal_data->old_scale = *(hal_data->pos_scale);
    }

    // Read PDO data
    // Status word: 16 bits little-endian at status_pdo_os
    // status_pdo_bp = 0 because 0x6000:01 is bit 0 of byte 0
    status = EC_READ_U16(pd + hal_data->status_pdo_os);
    raw    = EC_READ_S32(pd + hal_data->count_pdo_os);

    // Initialize accumulator on first operational cycle
    if (hal_data->do_init || !hal_data->last_operational) {
        hal_data->last_count = raw;
        hal_data->accum      = raw;
        hal_data->do_init    = 0;
    }
    hal_data->last_operational = 1;

    // Software accumulator reset (enc-reset pin)
    if (*(hal_data->reset)) {
        hal_data->accum      = 0;
        hal_data->last_count = raw;
    }

    // Hardware counter preset (enc-set-raw-count pulse)
    // Writes preset value + Set counter bit; device confirms via cnt_set_done
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

    // Accumulate delta (correctly handles 32-bit hardware counter rollover)
    delta = raw - hal_data->last_count;
    hal_data->accum      += delta;
    hal_data->last_count  = raw;

    // Write data output pins
    *(hal_data->status_raw) = status;
    *(hal_data->raw_count)  = raw;
    *(hal_data->count)      = (int32_t)hal_data->accum;
    *(hal_data->latch_val)  = EC_READ_S32(pd + hal_data->latch_pdo_os);
    *(hal_data->pos)        = (double)hal_data->accum * hal_data->scale;

    // Write status bit pins
    *(hal_data->latch_c_valid)   = (status & EP5101_STS_LATCH_C_VALID)   ? 1 : 0;
    *(hal_data->latch_ext_valid) = (status & EP5101_STS_LATCH_EXT_VALID) ? 1 : 0;
    *(hal_data->cnt_set_done)    = (status & EP5101_STS_CNT_SET_DONE)    ? 1 : 0;
    *(hal_data->underflow)       = (status & EP5101_STS_UNDERFLOW)       ? 1 : 0;
    *(hal_data->overflow)        = (status & EP5101_STS_OVERFLOW)        ? 1 : 0;
    *(hal_data->input_status)    = (status & EP5101_STS_INPUT_STATUS)    ? 1 : 0;
    *(hal_data->open_circuit)    = (status & EP5101_STS_OPEN_CIRCUIT)    ? 1 : 0;
    *(hal_data->extrap_stall)    = (status & EP5101_STS_EXTRAP_STALL)    ? 1 : 0;
    *(hal_data->input_a)         = (status & EP5101_STS_INPUT_A)         ? 1 : 0;
    *(hal_data->input_b)         = (status & EP5101_STS_INPUT_B)         ? 1 : 0;
    *(hal_data->input_c)         = (status & EP5101_STS_INPUT_C)         ? 1 : 0;
    *(hal_data->input_gate)      = (status & EP5101_STS_INPUT_GATE)      ? 1 : 0;
    *(hal_data->ext_latch_status)= (status & EP5101_STS_EXT_LATCH)       ? 1 : 0;
    *(hal_data->sync_error)      = (status & EP5101_STS_SYNC_ERROR)      ? 1 : 0;
    *(hal_data->txpdo_state)     = (status & EP5101_STS_TXPDO_STATE)     ? 1 : 0;
    *(hal_data->txpdo_toggle)    = (status & EP5101_STS_TXPDO_TOGGLE)    ? 1 : 0;
}

// ── Write callback (realtime) ────────────────────────────────────────────────
static void lcec_ep5101_write(lcec_slave_t *slave, long period) {
    lcec_master_t      *master   = slave->master;
    lcec_ep5101_data_t *hal_data = (lcec_ep5101_data_t *)slave->hal_data;
    uint8_t            *pd       = master->process_data;
    uint16_t ctrl = 0;

    if (!slave->state.operational)
        return;

    // Assemble 16-bit control word from HAL pins (bits 4-15 always 0)
    if (*(hal_data->ena_latch_c))
        ctrl |= EP5101_CTRL_ENA_LATCH_C;
    if (*(hal_data->ena_latch_ext_pos))
        ctrl |= EP5101_CTRL_ENA_LATCH_EXT_POS;
    if (*(hal_data->set_counter))
        ctrl |= EP5101_CTRL_SET_COUNTER;
    if (*(hal_data->ena_latch_ext_neg))
        ctrl |= EP5101_CTRL_ENA_LATCH_EXT_NEG;

    EC_WRITE_U16(pd + hal_data->ctrl_pdo_os, ctrl);
    // Note: setcnt_pdo_os (0x7000:11) is written in the read callback
    // when enc-set-raw-count is triggered, ensuring preset value and
    // Set counter bit are written atomically in the same EtherCAT cycle.
}

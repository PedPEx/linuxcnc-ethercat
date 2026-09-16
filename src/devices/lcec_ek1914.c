/* lcec_ek1914.c
 *
 * LinuxCNC EtherCAT (lcec) driver for Beckhoff EK1914
 * (EtherCAT coupler with integrated standard + TwinSAFE I/O).
 *
 * PDO layout verified via `ethercat pdos -p 0` / `ethercat cstruct -p 0`.
 * Fixed mapping: Enable PDO Assign: no / Enable PDO Configuration: no.
 *
 *  SM2 Outputs (master -> slave)                                    8 bytes
 *  0x1600 "FSOE RxPDO-Map"  (master->slave FSoE frame, 6 bytes)
 *  ┌───────────┬──────────────────────────────────────────────────────────┐
 *  │ 7000:01   │  8b  FSoE Master CMD                                      │
 *  │ 7001:01   │  1b  OutputChannel1   (safe output 1)                     │
 *  │ 7001:02   │  1b  OutputChannel2   (safe output 2)                     │
 *  │ gap       │  6b                                                       │
 *  │ 7000:03   │ 16b  FSoE Master ConnID                                   │
 *  │ 7000:02   │ 16b  FSoE Master CRC_0                                    │
 *  └───────────┴──────────────────────────────────────────────────────────┘
 *  0x1601 "DIO RxPDO-Map Outputs"  (2 bytes)
 *  ┌───────────┬──────────────────────────────────────────────────────────┐
 *  │ 7010:01-04│  1b each  Output 0..3   (standard DO)                     │
 *  │ 7010:05   │  1b  Safety Linked Output 0                               │
 *  │ 7010:06   │  1b  Safety Linked Output 1                               │
 *  │ gap       │ 10b                                                       │
 *  └───────────┴──────────────────────────────────────────────────────────┘
 *
 *  SM3 Inputs (slave -> master)                                     8 bytes
 *  0x1a00 "FSOE TxPDO-Map"  (slave->master FSoE frame, 6 bytes)
 *  ┌───────────┬──────────────────────────────────────────────────────────┐
 *  │ 6000:01   │  8b  FSoE Slave CMD                                       │
 *  │ 6001:01   │  1b  InputChannel1    (safe input 1)                      │
 *  │ 6001:02   │  1b  InputChannel2    (safe input 2)                      │
 *  │ gap       │  6b                                                       │
 *  │ 6000:03   │ 16b  FSoE Slave ConnID                                    │
 *  │ 6000:02   │ 16b  FSoE Slave CRC_0                                     │
 *  └───────────┴──────────────────────────────────────────────────────────┘
 *  0x1a01 "DIO TxPDO-Map Inputs"  (2 bytes)
 *  ┌───────────┬──────────────────────────────────────────────────────────┐
 *  │ 6010:01-04│  1b each  Input 0..3    (standard DI)                     │
 *  │ gap       │ 12b                                                       │
 *  └───────────┴──────────────────────────────────────────────────────────┘
 *
 * HAL pins (prefix: lcec.<master>.<slave>):
 *   Standard I/O (functional):
 *     din-0..3        HAL_OUT   6010:01..04   standard digital inputs
 *     dout-0..3       HAL_IN    7010:01..04   standard digital outputs
 *     safe-linked-out-0..1  HAL_IN  7010:05..06  standard out ANDed with safe release
 *   Safe I/O + FSoE frame (DIAGNOSTIC, read-only — NOT a safety interface):
 *     safe-in-0..1    HAL_OUT   6001:01..02   raw safe-input payload bit
 *     safe-out-0..1   HAL_OUT   7001:01..02   raw safe-output payload bit (echo of logic)
 *     fsoe-slave-cmd/-connid/-crc     HAL_U32  6000:01/03/02
 *     fsoe-master-cmd/-connid/-crc    HAL_U32  7000:01/03/02
 */

#include "../lcec.h"
#include "lcec_ek1914.h"

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/* ======================================================================
 * Forward declarations
 * ====================================================================== */
static void lcec_ek1914_read(lcec_slave_t *slave, long period);
static void lcec_ek1914_write(lcec_slave_t *slave, long period);

/* ======================================================================
 * Type registration
 * ====================================================================== */
static lcec_typelist_t types[] = {
  {"EK1914", LCEC_EK1914_VID, LCEC_EK1914_PID, 0, NULL, lcec_ek1914_init, NULL},
  {NULL},
};
ADD_TYPES(types);

/* ======================================================================
 * HAL data structure
 * ====================================================================== */
typedef struct {

  /* --- Standard inputs (0x1a01 -> HAL_OUT) --- */
  unsigned int din_os[4];   unsigned int din_bp[4];
  hal_bit_t   *din[4];

  /* --- Standard outputs (0x1601 -> HAL_IN) --- */
  unsigned int dout_os[4];  unsigned int dout_bp[4];
  hal_bit_t   *dout[4];

  /* --- Safety-linked standard outputs (0x1601 -> HAL_IN) --- */
  unsigned int slout_os[2]; unsigned int slout_bp[2];
  hal_bit_t   *slout[2];

  /* --- Safe input payload (0x1a00 -> HAL_OUT, diagnostic) --- */
  unsigned int safe_in_os[2];  unsigned int safe_in_bp[2];
  hal_bit_t   *safe_in[2];

  /* --- Safe output payload (0x1600 -> HAL_OUT, diagnostic echo) --- */
  unsigned int safe_out_os[2]; unsigned int safe_out_bp[2];
  hal_bit_t   *safe_out[2];

  /* --- FSoE slave frame header (0x1a00, diagnostic) --- */
  unsigned int fsoe_slave_cmd_os;      /* 6000:01  8b  -> also frame start */
  unsigned int fsoe_slave_connid_os;   /* 6000:03 16b */
  unsigned int fsoe_slave_crc_os;      /* 6000:02 16b */
  hal_u32_t   *fsoe_slave_cmd;
  hal_u32_t   *fsoe_slave_connid;
  hal_u32_t   *fsoe_slave_crc;

  /* --- FSoE master frame header (0x1600, diagnostic) --- */
  unsigned int fsoe_master_cmd_os;     /* 7000:01  8b  -> also frame start */
  unsigned int fsoe_master_connid_os;  /* 7000:03 16b */
  unsigned int fsoe_master_crc_os;     /* 7000:02 16b */
  hal_u32_t   *fsoe_master_cmd;
  hal_u32_t   *fsoe_master_connid;
  hal_u32_t   *fsoe_master_crc;

  /* --- Domain byte offsets of the FSoE frame starts (published to the
   *     EL1918_LOGIC via slave->fsoe_master_offset / fsoe_slave_offset) --- */
  unsigned int fsoe_master_frame_os;   /* start of 0x1600 (== 7000:01 offset) */
  unsigned int fsoe_slave_frame_os;    /* start of 0x1a00 (== 6000:01 offset) */

} lcec_ek1914_data_t;

/* ======================================================================
 * HAL pin descriptor table
 * ====================================================================== */
static const lcec_pindesc_t slave_pins[] = {
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, din[0]),  "%s.%s.%s.din-0"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, din[1]),  "%s.%s.%s.din-1"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, din[2]),  "%s.%s.%s.din-2"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, din[3]),  "%s.%s.%s.din-3"},

  {HAL_BIT, HAL_IN,  offsetof(lcec_ek1914_data_t, dout[0]), "%s.%s.%s.dout-0"},
  {HAL_BIT, HAL_IN,  offsetof(lcec_ek1914_data_t, dout[1]), "%s.%s.%s.dout-1"},
  {HAL_BIT, HAL_IN,  offsetof(lcec_ek1914_data_t, dout[2]), "%s.%s.%s.dout-2"},
  {HAL_BIT, HAL_IN,  offsetof(lcec_ek1914_data_t, dout[3]), "%s.%s.%s.dout-3"},

  {HAL_BIT, HAL_IN,  offsetof(lcec_ek1914_data_t, slout[0]), "%s.%s.%s.safe-linked-out-0"},
  {HAL_BIT, HAL_IN,  offsetof(lcec_ek1914_data_t, slout[1]), "%s.%s.%s.safe-linked-out-1"},

  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, safe_in[0]),  "%s.%s.%s.safe-in-0"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, safe_in[1]),  "%s.%s.%s.safe-in-1"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, safe_out[0]), "%s.%s.%s.safe-out-0"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_ek1914_data_t, safe_out[1]), "%s.%s.%s.safe-out-1"},

  {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_slave_cmd),     "%s.%s.%s.fsoe-slave-cmd"},
  {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_slave_connid),  "%s.%s.%s.fsoe-slave-connid"},
  {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_slave_crc),     "%s.%s.%s.fsoe-slave-crc"},
  {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_master_cmd),    "%s.%s.%s.fsoe-master-cmd"},
  {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_master_connid), "%s.%s.%s.fsoe-master-connid"},
  {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_master_crc),    "%s.%s.%s.fsoe-master-crc"},

  {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/* ======================================================================
 * PDO entry arrays  (mirror the fixed hardware mapping EXACTLY)
 * ====================================================================== */

/* SM2 / 0x1600 — master->slave FSoE frame (6 bytes) */
static ec_pdo_entry_info_t ek1914_out_1600[] = {
  {0x7000, 0x01,  8},  /* FSoE Master CMD    */
  {0x7001, 0x01,  1},  /* OutputChannel1     */
  {0x7001, 0x02,  1},  /* OutputChannel2     */
  {0x0000, 0x00,  6},  /* Gap                */
  {0x7000, 0x03, 16},  /* FSoE Master ConnID */
  {0x7000, 0x02, 16},  /* FSoE Master CRC_0  */
};

/* SM2 / 0x1601 — standard outputs (2 bytes) */
static ec_pdo_entry_info_t ek1914_out_1601[] = {
  {0x7010, 0x01,  1},  /* Output 0               */
  {0x7010, 0x02,  1},  /* Output 1               */
  {0x7010, 0x03,  1},  /* Output 2               */
  {0x7010, 0x04,  1},  /* Output 3               */
  {0x7010, 0x05,  1},  /* Safety Linked Output 0 */
  {0x7010, 0x06,  1},  /* Safety Linked Output 1 */
  {0x0000, 0x00, 10},  /* Gap                    */
};

/* SM3 / 0x1a00 — slave->master FSoE frame (6 bytes) */
static ec_pdo_entry_info_t ek1914_in_1a00[] = {
  {0x6000, 0x01,  8},  /* FSoE Slave CMD    */
  {0x6001, 0x01,  1},  /* InputChannel1     */
  {0x6001, 0x02,  1},  /* InputChannel2     */
  {0x0000, 0x00,  6},  /* Gap               */
  {0x6000, 0x03, 16},  /* FSoE Slave ConnID */
  {0x6000, 0x02, 16},  /* FSoE Slave CRC_0  */
};

/* SM3 / 0x1a01 — standard inputs (2 bytes) */
static ec_pdo_entry_info_t ek1914_in_1a01[] = {
  {0x6010, 0x01,  1},  /* Input 0 */
  {0x6010, 0x02,  1},  /* Input 1 */
  {0x6010, 0x03,  1},  /* Input 2 */
  {0x6010, 0x04,  1},  /* Input 3 */
  {0x0000, 0x00, 12},  /* Gap     */
};

static ec_pdo_info_t ek1914_pdos_out[] = {
  {0x1600, ARRAY_SIZE(ek1914_out_1600), ek1914_out_1600},
  {0x1601, ARRAY_SIZE(ek1914_out_1601), ek1914_out_1601},
};

static ec_pdo_info_t ek1914_pdos_in[] = {
  {0x1a00, ARRAY_SIZE(ek1914_in_1a00), ek1914_in_1a00},
  {0x1a01, ARRAY_SIZE(ek1914_in_1a01), ek1914_in_1a01},
};

static ec_sync_info_t ek1914_syncs[] = {
  {0, EC_DIR_OUTPUT, 0, NULL},
  {1, EC_DIR_INPUT,  0, NULL},
  {2, EC_DIR_OUTPUT, ARRAY_SIZE(ek1914_pdos_out), ek1914_pdos_out},
  {3, EC_DIR_INPUT,  ARRAY_SIZE(ek1914_pdos_in),  ek1914_pdos_in},
  {0xFF},
};

/* ======================================================================
 * FSoE connection registration
 *
 *   *** VERIFY THIS BLOCK AGAINST YOUR TREE'S lcec_el1904.c + lcec.h ***
 *
 * This is the only fork-specific part. In sittner-master (inherited by the
 * community fork) a regular FSoE slave must publish two things so the
 * EL1918_LOGIC second-pass init can wire the connection referenced by its
 * fsoeSlaveIdx:
 *   1. slave->fsoeConf         -> FSoE frame dimensions
 *   2. slave->fsoe_master_offset / slave->fsoe_slave_offset
 *                              -> domain byte offset of each frame start
 *
 * Confirm: the struct name (LCEC_CONF_FSOE_T) and its field names, whether
 * it must be set in a preinit callback (5th typelist slot) rather than in
 * init(), and how the frame-start offsets are captured. Everything above
 * this block is plain PDO/HAL work and is independent of these details.
 * ====================================================================== */
static const LCEC_CONF_FSOE_T ek1914_fsoe_conf = {
  .slave_data_len  = 1,   /* slave->master: 1 safe-data byte (2 safe inputs)  */
  .master_data_len = 1,   /* master->slave: 1 safe-data byte (2 safe outputs) */
  .data_channels   = 1,
};

/* ======================================================================
 * lcec_ek1914_init()
 * ====================================================================== */
int lcec_ek1914_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t      *master = slave->master;
  lcec_ek1914_data_t *hal_data;
  int i, err;

  slave->proc_read  = lcec_ek1914_read;
  slave->proc_write = lcec_ek1914_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_ek1914_data_t);
  slave->hal_data  = hal_data;
  slave->sync_info = ek1914_syncs;

  /* --- PDO entry registration --- */

  /* FSoE slave frame (0x1a00). The CMD offset is the frame start. */
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->fsoe_slave_cmd_os,    NULL);
  lcec_pdo_init(slave, 0x6001, 0x01, &hal_data->safe_in_os[0],        &hal_data->safe_in_bp[0]);
  lcec_pdo_init(slave, 0x6001, 0x02, &hal_data->safe_in_os[1],        &hal_data->safe_in_bp[1]);
  lcec_pdo_init(slave, 0x6000, 0x03, &hal_data->fsoe_slave_connid_os, NULL);
  lcec_pdo_init(slave, 0x6000, 0x02, &hal_data->fsoe_slave_crc_os,    NULL);
  hal_data->fsoe_slave_frame_os = hal_data->fsoe_slave_cmd_os;

  /* Standard inputs (0x1a01). */
  for (i = 0; i < 4; i++)
    lcec_pdo_init(slave, 0x6010, 0x01 + i, &hal_data->din_os[i], &hal_data->din_bp[i]);

  /* FSoE master frame (0x1600). The CMD offset is the frame start. */
  lcec_pdo_init(slave, 0x7000, 0x01, &hal_data->fsoe_master_cmd_os,    NULL);
  lcec_pdo_init(slave, 0x7001, 0x01, &hal_data->safe_out_os[0],        &hal_data->safe_out_bp[0]);
  lcec_pdo_init(slave, 0x7001, 0x02, &hal_data->safe_out_os[1],        &hal_data->safe_out_bp[1]);
  lcec_pdo_init(slave, 0x7000, 0x03, &hal_data->fsoe_master_connid_os, NULL);
  lcec_pdo_init(slave, 0x7000, 0x02, &hal_data->fsoe_master_crc_os,    NULL);
  hal_data->fsoe_master_frame_os = hal_data->fsoe_master_cmd_os;

  /* Standard outputs (0x1601). */
  for (i = 0; i < 4; i++)
    lcec_pdo_init(slave, 0x7010, 0x01 + i, &hal_data->dout_os[i], &hal_data->dout_bp[i]);
  lcec_pdo_init(slave, 0x7010, 0x05, &hal_data->slout_os[0], &hal_data->slout_bp[0]);
  lcec_pdo_init(slave, 0x7010, 0x06, &hal_data->slout_os[1], &hal_data->slout_bp[1]);

  /* --- FSoE slave registration (see VERIFY block above) --- */
  slave->fsoeConf           = &ek1914_fsoe_conf;
  slave->fsoe_master_offset = &hal_data->fsoe_master_frame_os;
  slave->fsoe_slave_offset  = &hal_data->fsoe_slave_frame_os;

  /* --- HAL pins --- */
  if ((err = lcec_pin_newf_list(hal_data, slave_pins,
        LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  return 0;
}

/* ======================================================================
 * lcec_ek1914_read()  —  EtherCAT cycle -> HAL
 * ====================================================================== */
static void lcec_ek1914_read(lcec_slave_t *slave, long period) {
  lcec_ek1914_data_t *hal_data = (lcec_ek1914_data_t *)slave->hal_data;
  uint8_t            *pd       = slave->master->process_data;
  int i;

  if (!slave->state.operational) return;

  /* Standard digital inputs */
  for (i = 0; i < 4; i++)
    *hal_data->din[i] = EC_READ_BIT(pd + hal_data->din_os[i], hal_data->din_bp[i]);

  /* Safe-payload diagnostics (raw bits, not safety-rated) */
  for (i = 0; i < 2; i++) {
    *hal_data->safe_in[i]  = EC_READ_BIT(pd + hal_data->safe_in_os[i],  hal_data->safe_in_bp[i]);
    *hal_data->safe_out[i] = EC_READ_BIT(pd + hal_data->safe_out_os[i], hal_data->safe_out_bp[i]);
  }

  /* FSoE frame header diagnostics */
  *hal_data->fsoe_slave_cmd     = EC_READ_U8 (pd + hal_data->fsoe_slave_cmd_os);
  *hal_data->fsoe_slave_connid  = EC_READ_U16(pd + hal_data->fsoe_slave_connid_os);
  *hal_data->fsoe_slave_crc     = EC_READ_U16(pd + hal_data->fsoe_slave_crc_os);
  *hal_data->fsoe_master_cmd    = EC_READ_U8 (pd + hal_data->fsoe_master_cmd_os);
  *hal_data->fsoe_master_connid = EC_READ_U16(pd + hal_data->fsoe_master_connid_os);
  *hal_data->fsoe_master_crc    = EC_READ_U16(pd + hal_data->fsoe_master_crc_os);
}

/* ======================================================================
 * lcec_ek1914_write()  —  HAL -> EtherCAT cycle
 *
 * Only the standard outputs (0x1601) are written here. The FSoE master
 * frame (0x1600) is owned by the EL1918_LOGIC driver and must NOT be
 * touched, or the FSoE CRC breaks and the connection drops.
 * ====================================================================== */
static void lcec_ek1914_write(lcec_slave_t *slave, long period) {
  lcec_ek1914_data_t *hal_data = (lcec_ek1914_data_t *)slave->hal_data;
  uint8_t            *pd       = slave->master->process_data;
  int i;

  if (!slave->state.operational) return;

  for (i = 0; i < 4; i++)
    EC_WRITE_BIT(pd + hal_data->dout_os[i], hal_data->dout_bp[i], *hal_data->dout[i] ? 1 : 0);

  for (i = 0; i < 2; i++)
    EC_WRITE_BIT(pd + hal_data->slout_os[i], hal_data->slout_bp[i], *hal_data->slout[i] ? 1 : 0);
}

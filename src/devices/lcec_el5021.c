/* lcec_el5021.c
 *
 * LinuxCNC EtherCAT (lcec) driver for Beckhoff EL5021 / EL5021-0090
 * SinCos Encoder Interface.
 *
 * Objects verified present on hardware (TwinCAT CoE-Online, ethercat sdos):
 *
 *  SM2 RxPDO 0x1600 – ENC Outputs (master → slave)           6 bytes
 *  ┌──────────────────────────────────────────────────────────────────┐
 *  │ Bit 0     │ 7000:01  Enable latch C                            │
 *  │ Bit 1     │ gap      (7000:02 undefined)                       │
 *  │ Bit 2     │ 7000:03  Set counter                               │
 *  │ Bits 3-15 │ gap      (7000:04..10 undefined, 13 bits)          │
 *  │ Bits16-47 │ 7000:11  Set counter value (UINT32)                │
 *  └──────────────────────────────────────────────────────────────────┘
 *
 *  SM3 TxPDO 0x1A00 – ENC Inputs (slave → master)           10 bytes
 *  FIXED mapping – Enable PDO Assign: no / Enable PDO Configuration: no
 *  Layout verified via dmesg (IgH master read from device):
 *  ┌──────────────────────────────────────────────────────────────────┐
 *  │ Bit  0    │ 6000:01  Latch C valid                             │
 *  │ Bit  1    │ gap      (6000:02)                                 │
 *  │ Bit  2    │ 6000:03  Set counter done                          │
 *  │ Bit  3    │ 6001:04  Frequency error  ← in 1A00, NOT 1A01!    │
 *  │ Bit  4    │ 6001:05  Amplitude error  ← in 1A00, NOT 1A01!    │
 *  │ Bits 5-9  │ gap      (5 bits)                                  │
 *  │ Bit 10    │ 6000:0B  Status of input C                        │
 *  │ Bits11-12 │ gap      (6000:0C..0D)                            │
 *  │ Bit 13    │ 6000:0E  Sync error                               │
 *  │ Bit 14    │ 6000:0F  TxPDO State                              │
 *  │ Bit 15    │ 6000:10  TxPDO Toggle                             │
 *  │ Bits16-47 │ 6000:11  Counter value (UINT32)                   │
 *  │ Bits48-79 │ 6000:12  Latch value   (UINT32)                   │
 *  └──────────────────────────────────────────────────────────────────┘
 *  NOTE: 0x1A01 does not exist on this HW revision.
 *
 * Startup SDOs (written before SAFEOP via lcec_write_sdo*):
 *   8000:01  Enable C reset          (modParam enableCReset,        def 0)
 *   8000:0E  Reversion of rotation   (modParam reversionOfRotation, def 0)
 *   8001:01  Enable frequency error  (modParam enableFrequencyError, def 1)
 *   8001:02  Enable amplitude error  (modParam enableAmplitudeError, def 1)
 *   8001:11  Analog resolution       (modParam analogResolution,     def 10)
 *
 * Cyclic SDO reads (mailbox, via ec_sdo_request_t):
 *   A000:11  frequency error counter (UINT16)
 *   A000:12  amplitude error counter (UINT16)
 *
 * HAL pins (prefix: lcec.<master>.<slave>):
 *   Inputs  (HAL_OUT): enc.latch-c-valid  enc.set-counter-done
 *                      enc.status-input-c enc.sync-error
 *                      enc.txpdo-state    enc.txpdo-toggle
 *                      enc.counter-value  enc.latch-value
 *                      enc.frequency-error enc.amplitude-error
 *                      enc.frequency-error-count enc.amplitude-error-count
 *   Outputs (HAL_IN):  enc.enable-latch-c enc.set-counter
 *                      enc.set-counter-value
 */

#include "../lcec.h"
#include "lcec_el5021.h"

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/* ======================================================================
 * Factory defaults (confirmed on hardware)
 * ====================================================================== */
#define DEF_ENABLE_C_RESET         0  /* 8000:01 */
#define DEF_REVERSION_OF_ROTATION  0  /* 8000:0E */
#define DEF_ENABLE_FREQ_ERROR      1  /* 8001:01 – factory default is ON */
#define DEF_ENABLE_AMP_ERROR       1  /* 8001:02 – factory default is ON */
#define DEF_ANALOG_RESOLUTION      10 /* 8001:11 – 10 bit, fmax=250kHz  */

#define SDO_ERR_CNT_SIZE           2  /* A000:11/:12 are UINT16          */

/* Read error counters every N servo cycles.
 * At 1 kHz servo thread: 1000 = once per second.  Adjust to taste.      */
#define SDO_READ_INTERVAL          1000

/* ======================================================================
 * modParam descriptor table
 * ====================================================================== */
const lcec_modparam_desc_t lcec_el5021_modparams[] = {
  {"enableCReset",
    LCEC_EL5021_MODPARAM_ENABLE_C_RESET,        MODPARAM_TYPE_BIT,
    "false", "Enable counter reset via C input (8000:01)"},
  {"reversionOfRotation",
    LCEC_EL5021_MODPARAM_REVERSION_OF_ROTATION, MODPARAM_TYPE_BIT,
    "false", "Reverse counting direction (8000:0E)"},
  {"enableFrequencyError",
    LCEC_EL5021_MODPARAM_ENABLE_FREQ_ERROR,     MODPARAM_TYPE_BIT,
    "true",  "Enable frequency error detection, factory default ON (8001:01)"},
  {"enableAmplitudeError",
    LCEC_EL5021_MODPARAM_ENABLE_AMP_ERROR,      MODPARAM_TYPE_BIT,
    "true",  "Enable amplitude error detection, factory default ON (8001:02)"},
  {"analogResolution",
    LCEC_EL5021_MODPARAM_ANALOG_RESOLUTION,     MODPARAM_TYPE_U32,
    "10",    "Period resolution in bits, 10-bit -> fmax=250kHz (8001:11)"},
  {NULL},
};

/* ======================================================================
 * Forward declarations
 * ====================================================================== */
static void lcec_el5021_read(lcec_slave_t *slave, long period);
static void lcec_el5021_write(lcec_slave_t *slave, long period);

/* ======================================================================
 * Type registration
 * ====================================================================== */
static lcec_typelist_t types[] = {
  {"EL5021",      LCEC_EL5021_VID, LCEC_EL5021_PID,      0, NULL,
    lcec_el5021_init, lcec_el5021_modparams},
  {"EL5021-0090", LCEC_EL5021_VID, LCEC_EL5021_0090_PID, 0, NULL,
    lcec_el5021_init, lcec_el5021_modparams},
  {NULL},
};
ADD_TYPES(types);

/* ======================================================================
 * HAL data structure
 * ====================================================================== */
typedef struct {

  /* TxPDO 0x1A00: all entries (6001:04+05 embedded here) */
  unsigned int latch_c_valid_os;       unsigned int latch_c_valid_bp;
  unsigned int set_counter_done_os;    unsigned int set_counter_done_bp;
  unsigned int status_input_c_os;      unsigned int status_input_c_bp;
  unsigned int sync_error_os;          unsigned int sync_error_bp;
  unsigned int txpdo_state_os;         unsigned int txpdo_state_bp;
  unsigned int txpdo_toggle_os;        unsigned int txpdo_toggle_bp;
  unsigned int counter_value_os;       /* UINT32 */
  unsigned int latch_value_os;         /* UINT32 */

  /* (6001:04 and 6001:05 registered above as part of 0x1A00) */
  unsigned int frequency_error_os;     unsigned int frequency_error_bp;
  unsigned int amplitude_error_os;     unsigned int amplitude_error_bp;

  /* RxPDO 0x1600: 7000:xx ENC Outputs */
  unsigned int enable_latch_c_os;      unsigned int enable_latch_c_bp;
  unsigned int set_counter_os;         unsigned int set_counter_bp;
  unsigned int set_counter_value_os;   /* UINT32 */

  /* Cyclic SDO requests: A000:11 / A000:12 */
  ec_sdo_request_t *sdo_freq_err_cnt;
  ec_sdo_request_t *sdo_amp_err_cnt;
  unsigned int      sdo_read_timer;  /**< throttle counter for SDO reads    */

  /* HAL input pins (device → LinuxCNC) */
  hal_bit_t *latch_c_valid;       /*!< 6000:01 counter latched via C input   */
  hal_bit_t *set_counter_done;    /*!< 6000:03 counter preset executed        */
  hal_bit_t *status_input_c;      /*!< 6000:0B physical state of C input      */
  hal_bit_t *sync_error;          /*!< 6000:0E DC sync error                  */
  hal_bit_t *txpdo_state;         /*!< 6000:0F TRUE on freq/amplitude error   */
  hal_bit_t *txpdo_toggle;        /*!< 6000:10 toggles on each PDO update     */
  hal_u32_t *counter_value;       /*!< 6000:11 encoder counter (32-bit)       */
  hal_u32_t *latch_value;         /*!< 6000:12 latched counter value          */
  hal_bit_t *frequency_error;     /*!< 6001:04 fmax exceeded                  */
  hal_bit_t *amplitude_error;     /*!< 6001:05 SinCos amplitude too low       */
  hal_u32_t *frequency_error_cnt; /*!< A000:11 cumulative freq error count    */
  hal_u32_t *amplitude_error_cnt; /*!< A000:12 cumulative amp error count     */

  /* HAL output pins (LinuxCNC → device) */
  hal_bit_t *enable_latch_c;      /*!< 7000:01 arm C-input latch              */
  hal_bit_t *set_counter;         /*!< 7000:03 trigger counter preset         */
  hal_u32_t *set_counter_value;   /*!< 7000:11 preset target value            */

} lcec_el5021_data_t;

/* ======================================================================
 * HAL pin descriptor table
 * ====================================================================== */
static const lcec_pindesc_t slave_pins[] = {
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, latch_c_valid),
    "%s.%s.%s.enc.latch-c-valid"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, set_counter_done),
    "%s.%s.%s.enc.set-counter-done"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, status_input_c),
    "%s.%s.%s.enc.status-input-c"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, sync_error),
    "%s.%s.%s.enc.sync-error"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, txpdo_state),
    "%s.%s.%s.enc.txpdo-state"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, txpdo_toggle),
    "%s.%s.%s.enc.txpdo-toggle"},
  {HAL_U32, HAL_OUT, offsetof(lcec_el5021_data_t, counter_value),
    "%s.%s.%s.enc.counter-value"},
  {HAL_U32, HAL_OUT, offsetof(lcec_el5021_data_t, latch_value),
    "%s.%s.%s.enc.latch-value"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, frequency_error),
    "%s.%s.%s.enc.frequency-error"},
  {HAL_BIT, HAL_OUT, offsetof(lcec_el5021_data_t, amplitude_error),
    "%s.%s.%s.enc.amplitude-error"},
  {HAL_U32, HAL_OUT, offsetof(lcec_el5021_data_t, frequency_error_cnt),
    "%s.%s.%s.enc.frequency-error-count"},
  {HAL_U32, HAL_OUT, offsetof(lcec_el5021_data_t, amplitude_error_cnt),
    "%s.%s.%s.enc.amplitude-error-count"},
  {HAL_BIT, HAL_IN,  offsetof(lcec_el5021_data_t, enable_latch_c),
    "%s.%s.%s.enc.enable-latch-c"},
  {HAL_BIT, HAL_IN,  offsetof(lcec_el5021_data_t, set_counter),
    "%s.%s.%s.enc.set-counter"},
  {HAL_U32, HAL_IN,  offsetof(lcec_el5021_data_t, set_counter_value),
    "%s.%s.%s.enc.set-counter-value"},
  {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/* ======================================================================
 * PDO entry arrays
 * ====================================================================== */

/* TxPDO 0x1A00: complete fixed mapping as read from device (10 bytes).
 * 6001:04 and 6001:05 are embedded here at bits 3+4 – 0x1A01 does NOT exist.
 * Layout confirmed: dmesg "Currently mapped PDO entries" output.            */
static ec_pdo_entry_info_t lcec_el5021_in_entries[] = {
  {0x6000, 0x01,  1},  /* bit  0: Latch C valid                            */
  {0x0000, 0x00,  1},  /* bit  1: gap (6000:02 undefined)                  */
  {0x6000, 0x03,  1},  /* bit  2: Set counter done                         */
  {0x6001, 0x04,  1},  /* bit  3: Frequency error  (embedded in 0x1A00)    */
  {0x6001, 0x05,  1},  /* bit  4: Amplitude error  (embedded in 0x1A00)    */
  {0x0000, 0x00,  5},  /* bits 5-9: gap                                    */
  {0x6000, 0x0B,  1},  /* bit 10: Status of input C                        */
  {0x0000, 0x00,  2},  /* bits11-12: gap (6000:0C..0D undefined)           */
  {0x6000, 0x0E,  1},  /* bit 13: Sync error                               */
  {0x6000, 0x0F,  1},  /* bit 14: TxPDO State                              */
  {0x6000, 0x10,  1},  /* bit 15: TxPDO Toggle                             */
  {0x6000, 0x11, 32},  /* bits16-47: Counter value (UINT32)                */
  {0x6000, 0x12, 32},  /* bits48-79: Latch value   (UINT32)                */
};

/* RxPDO 0x1600: ENC Outputs to 7000 – 6 bytes                            */
static ec_pdo_entry_info_t lcec_el5021_out_7000_entries[] = {
  {0x7000, 0x01,  1},  /* Enable latch C                                  */
  {0x0000, 0x00,  1},  /* gap: 7000:02 undefined                          */
  {0x7000, 0x03,  1},  /* Set counter                                     */
  {0x0000, 0x00, 13},  /* gap: 7000:04..10 undefined (13 bits)            */
  {0x7000, 0x11, 32},  /* Set counter value (UINT32)                      */
};

/* ======================================================================
 * PDO info and sync manager configuration
 * ====================================================================== */
static ec_pdo_info_t lcec_el5021_pdos_in[] = {
  {0x1A00, ARRAY_SIZE(lcec_el5021_in_entries), lcec_el5021_in_entries},
  /* 0x1A01 does not exist on this HW revision */
};

static ec_pdo_info_t lcec_el5021_pdos_out[] = {
  {0x1600, ARRAY_SIZE(lcec_el5021_out_7000_entries), lcec_el5021_out_7000_entries},
};

static ec_sync_info_t lcec_el5021_syncs[] = {
  {0, EC_DIR_OUTPUT, 0, NULL},
  {1, EC_DIR_INPUT,  0, NULL},
  {2, EC_DIR_OUTPUT, ARRAY_SIZE(lcec_el5021_pdos_out), lcec_el5021_pdos_out},
  {3, EC_DIR_INPUT,  ARRAY_SIZE(lcec_el5021_pdos_in),  lcec_el5021_pdos_in},
  {0xFF},
};

/* ======================================================================
 * lcec_el5021_init()
 * ====================================================================== */
int lcec_el5021_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_el5021_data_t *hal_data;
  lcec_slave_modparam_t *p;
  int err;

  /* Settings with factory defaults – overridden by XML modParams          */
  uint8_t cfg_enable_c_reset        = DEF_ENABLE_C_RESET;
  uint8_t cfg_reversion_of_rotation = DEF_REVERSION_OF_ROTATION;
  uint8_t cfg_enable_freq_error     = DEF_ENABLE_FREQ_ERROR;
  uint8_t cfg_enable_amp_error      = DEF_ENABLE_AMP_ERROR;
  uint8_t cfg_analog_resolution     = DEF_ANALOG_RESOLUTION;

  slave->proc_read  = lcec_el5021_read;
  slave->proc_write = lcec_el5021_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_el5021_data_t);
  slave->hal_data  = hal_data;
  slave->sync_info = lcec_el5021_syncs;

  /* --- PDO entry registration --- */

  /* TxPDO 0x1A00: all entries (6001:04+05 embedded here) */
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->latch_c_valid_os,    &hal_data->latch_c_valid_bp);
  lcec_pdo_init(slave, 0x6000, 0x03, &hal_data->set_counter_done_os, &hal_data->set_counter_done_bp);
  lcec_pdo_init(slave, 0x6000, 0x0B, &hal_data->status_input_c_os,   &hal_data->status_input_c_bp);
  lcec_pdo_init(slave, 0x6000, 0x0E, &hal_data->sync_error_os,       &hal_data->sync_error_bp);
  lcec_pdo_init(slave, 0x6000, 0x0F, &hal_data->txpdo_state_os,      &hal_data->txpdo_state_bp);
  lcec_pdo_init(slave, 0x6000, 0x10, &hal_data->txpdo_toggle_os,     &hal_data->txpdo_toggle_bp);
  lcec_pdo_init(slave, 0x6000, 0x11, &hal_data->counter_value_os,    NULL);
  lcec_pdo_init(slave, 0x6000, 0x12, &hal_data->latch_value_os,      NULL);

  /* (6001:04 and 6001:05 registered above as part of 0x1A00) */
  lcec_pdo_init(slave, 0x6001, 0x04, &hal_data->frequency_error_os,  &hal_data->frequency_error_bp);
  lcec_pdo_init(slave, 0x6001, 0x05, &hal_data->amplitude_error_os,  &hal_data->amplitude_error_bp);

  /* RxPDO 0x1600: 7000:xx */
  lcec_pdo_init(slave, 0x7000, 0x01, &hal_data->enable_latch_c_os,   &hal_data->enable_latch_c_bp);
  lcec_pdo_init(slave, 0x7000, 0x03, &hal_data->set_counter_os,      &hal_data->set_counter_bp);
  lcec_pdo_init(slave, 0x7000, 0x11, &hal_data->set_counter_value_os, NULL);

  /* --- HAL pins --- */
  if ((err = lcec_pin_newf_list(hal_data, slave_pins,
        LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  /* --- Collect modParam overrides --- */
  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    switch (p->id) {
      case LCEC_EL5021_MODPARAM_ENABLE_C_RESET:
        cfg_enable_c_reset        = p->value.bit ? 1 : 0;  break;
      case LCEC_EL5021_MODPARAM_REVERSION_OF_ROTATION:
        cfg_reversion_of_rotation = p->value.bit ? 1 : 0;  break;
      case LCEC_EL5021_MODPARAM_ENABLE_FREQ_ERROR:
        cfg_enable_freq_error     = p->value.bit ? 1 : 0;  break;
      case LCEC_EL5021_MODPARAM_ENABLE_AMP_ERROR:
        cfg_enable_amp_error      = p->value.bit ? 1 : 0;  break;
      case LCEC_EL5021_MODPARAM_ANALOG_RESOLUTION:
        cfg_analog_resolution     = (uint8_t)(p->value.u32 & 0xFF); break;
      default:
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "unknown modParam id %d for slave %s.%s\n",
          p->id, master->name, slave->name);
        break;
    }
  }

  /* --- Write 8000:xx ENC Settings (max subindex 0x0E on this HW revision)
   *
   * Only written when the value differs from the factory default, to avoid
   * unnecessary SDO traffic to already-correct registers.  Failures are
   * non-fatal: the device retains its current (factory-default) value, which
   * is safe for normal operation.  Use XML <sdoConfig> if a hard write is
   * required regardless of current value.                                    */

  /* 8000:01  Enable C reset (BOOLEAN, factory default: 0 = disabled) */
  if (cfg_enable_c_reset != DEF_ENABLE_C_RESET) {
    if (lcec_write_sdo8(slave, 0x8000, 0x01, cfg_enable_c_reset) != 0)
      rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "slave %s.%s: SDO 0x8000:01 (enableCReset) write failed"
        " – device keeps current value\n", master->name, slave->name);
  }

  /* 8000:0E  Reversion of rotation (BOOLEAN, factory default: 0 = normal) */
  if (cfg_reversion_of_rotation != DEF_REVERSION_OF_ROTATION) {
    if (lcec_write_sdo8(slave, 0x8000, 0x0E, cfg_reversion_of_rotation) != 0)
      rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "slave %s.%s: SDO 0x8000:0E (reversionOfRotation) write failed"
        " – device keeps current value\n", master->name, slave->name);
  }

  /* --- Write 8001:xx ENC SinCos Settings --- */

  /* 8001:01  Enable frequency error (BOOLEAN, factory default: 1 = enabled) */
  if (cfg_enable_freq_error != DEF_ENABLE_FREQ_ERROR) {
    if (lcec_write_sdo8(slave, 0x8001, 0x01, cfg_enable_freq_error) != 0)
      rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "slave %s.%s: SDO 0x8001:01 (enableFrequencyError) write failed"
        " – device keeps current value\n", master->name, slave->name);
  }

  /* 8001:02  Enable amplitude error (BOOLEAN, factory default: 1 = enabled) */
  if (cfg_enable_amp_error != DEF_ENABLE_AMP_ERROR) {
    if (lcec_write_sdo8(slave, 0x8001, 0x02, cfg_enable_amp_error) != 0)
      rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "slave %s.%s: SDO 0x8001:02 (enableAmplitudeError) write failed"
        " – device keeps current value\n", master->name, slave->name);
  }

  /* 8001:11  Analog resolution in bits (UINT8, factory default: 10) */
  if (cfg_analog_resolution != DEF_ANALOG_RESOLUTION) {
    if (lcec_write_sdo8(slave, 0x8001, 0x11, cfg_analog_resolution) != 0)
      rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "slave %s.%s: SDO 0x8001:11 (analogResolution) write failed"
        " – device keeps current value\n", master->name, slave->name);
  }

  /* --- Cyclic SDO read requests for A000:11 / A000:12 error counters --- */
  if (!(hal_data->sdo_freq_err_cnt = ecrt_slave_config_create_sdo_request(
        slave->config, 0xA000, 0x11, SDO_ERR_CNT_SIZE))) {
    rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX "ecrt_slave_config_create_sdo_request(0xA000:11) failed "
      "for slave %s.%s\n", master->name, slave->name);
    return -EIO;
  }
  ecrt_sdo_request_timeout(hal_data->sdo_freq_err_cnt, 500);

  if (!(hal_data->sdo_amp_err_cnt = ecrt_slave_config_create_sdo_request(
        slave->config, 0xA000, 0x12, SDO_ERR_CNT_SIZE))) {
    rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX "ecrt_slave_config_create_sdo_request(0xA000:12) failed "
      "for slave %s.%s\n", master->name, slave->name);
    return -EIO;
  }
  ecrt_sdo_request_timeout(hal_data->sdo_amp_err_cnt, 500);

  return 0;
}

/* ======================================================================
 * lcec_el5021_read()  –  EtherCAT cycle → HAL
 * ====================================================================== */
static void lcec_el5021_read(lcec_slave_t *slave, long period) {
  lcec_master_t      *master   = slave->master;
  lcec_el5021_data_t *hal_data = (lcec_el5021_data_t *)slave->hal_data;
  uint8_t            *pd       = master->process_data;

  if (!slave->state.operational) return;

  /* TxPDO 0x1A00: all entries (6001:04+05 embedded here) */
  *hal_data->latch_c_valid    = EC_READ_BIT(pd + hal_data->latch_c_valid_os,    hal_data->latch_c_valid_bp);
  *hal_data->set_counter_done = EC_READ_BIT(pd + hal_data->set_counter_done_os, hal_data->set_counter_done_bp);
  *hal_data->status_input_c   = EC_READ_BIT(pd + hal_data->status_input_c_os,   hal_data->status_input_c_bp);
  *hal_data->sync_error       = EC_READ_BIT(pd + hal_data->sync_error_os,       hal_data->sync_error_bp);
  *hal_data->txpdo_state      = EC_READ_BIT(pd + hal_data->txpdo_state_os,      hal_data->txpdo_state_bp);
  *hal_data->txpdo_toggle     = EC_READ_BIT(pd + hal_data->txpdo_toggle_os,     hal_data->txpdo_toggle_bp);
  *hal_data->counter_value    = EC_READ_U32(pd + hal_data->counter_value_os);
  *hal_data->latch_value      = EC_READ_U32(pd + hal_data->latch_value_os);

  /* (6001:04 and 6001:05 registered above as part of 0x1A00) */
  *hal_data->frequency_error  = EC_READ_BIT(pd + hal_data->frequency_error_os,  hal_data->frequency_error_bp);
  *hal_data->amplitude_error  = EC_READ_BIT(pd + hal_data->amplitude_error_os,  hal_data->amplitude_error_bp);

  /* Cyclic SDO reads for A000:11 / A000:12 – throttled to SDO_READ_INTERVAL.
   * Both counters share the same timer so they fire in the same cycle.      */
  if (++hal_data->sdo_read_timer >= SDO_READ_INTERVAL) {
    hal_data->sdo_read_timer = 0;

    /* A000:11  frequency error counter */
    switch (ecrt_sdo_request_state(hal_data->sdo_freq_err_cnt)) {
      case EC_REQUEST_UNUSED:
      case EC_REQUEST_SUCCESS:
        if (ecrt_sdo_request_state(hal_data->sdo_freq_err_cnt) == EC_REQUEST_SUCCESS)
          *hal_data->frequency_error_cnt =
            (hal_u32_t)EC_READ_U16(ecrt_sdo_request_data(hal_data->sdo_freq_err_cnt));
        ecrt_sdo_request_read(hal_data->sdo_freq_err_cnt);
        break;
      case EC_REQUEST_BUSY:
        break;
      case EC_REQUEST_ERROR:
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "SDO read failed on 0xA000:11 for slave %s.%s – retrying\n",
          master->name, slave->name);
        ecrt_sdo_request_read(hal_data->sdo_freq_err_cnt);
        break;
    }

    /* A000:12  amplitude error counter */
    switch (ecrt_sdo_request_state(hal_data->sdo_amp_err_cnt)) {
      case EC_REQUEST_UNUSED:
      case EC_REQUEST_SUCCESS:
        if (ecrt_sdo_request_state(hal_data->sdo_amp_err_cnt) == EC_REQUEST_SUCCESS)
          *hal_data->amplitude_error_cnt =
            (hal_u32_t)EC_READ_U16(ecrt_sdo_request_data(hal_data->sdo_amp_err_cnt));
        ecrt_sdo_request_read(hal_data->sdo_amp_err_cnt);
        break;
      case EC_REQUEST_BUSY:
        break;
      case EC_REQUEST_ERROR:
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "SDO read failed on 0xA000:12 for slave %s.%s – retrying\n",
          master->name, slave->name);
        ecrt_sdo_request_read(hal_data->sdo_amp_err_cnt);
        break;
    }
  }
}

/* ======================================================================
 * lcec_el5021_write()  –  HAL → EtherCAT cycle
 * ====================================================================== */
static void lcec_el5021_write(lcec_slave_t *slave, long period) {
  lcec_el5021_data_t *hal_data = (lcec_el5021_data_t *)slave->hal_data;
  uint8_t            *pd       = slave->master->process_data;

  if (!slave->state.operational) return;

  /* RxPDO 0x1600: 7000:xx */
  EC_WRITE_BIT(pd + hal_data->enable_latch_c_os, hal_data->enable_latch_c_bp,
    *hal_data->enable_latch_c ? 1 : 0);
  EC_WRITE_BIT(pd + hal_data->set_counter_os, hal_data->set_counter_bp,
    *hal_data->set_counter ? 1 : 0);
  EC_WRITE_U32(pd + hal_data->set_counter_value_os, *hal_data->set_counter_value);
}

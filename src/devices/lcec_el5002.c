//
//    Copyright (C) 2023 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Driver for Beckhoff EL5002 Encoder modules
///
/// FIX 1 (id decoding): the modParam descriptor table combined the channel
/// selector and the function selector with the logical OR "||" instead of the
/// bitwise OR "|".  "A || B" evaluates to 0/1, so every modParam id collapsed
/// to the same value and the id decoding ((p->id & CH_MASK) / (p->id &
/// FNK_MASK)) could never recover channel or function.  All 22 entries now use
/// bitwise "|".
///
/// FIX 2 (defaults): the SSI settings (object 0x80n0) are now pre-loaded with
/// the Beckhoff factory defaults and only overridden by modParams that are
/// actually present in the XML config.  A parameter left out of the XML
/// therefore results in the Beckhoff default.  To keep the mailbox quiet, an
/// object is only written when the effective value differs from that default,
/// so an unset parameter produces no SDO write and the device keeps its own
/// (identical) default.  Writes are non-fatal warnings now instead of aborting
/// the whole master init.

#include "lcec_el5002.h"

#include "../lcec.h"

/* ======================================================================
 * Beckhoff factory defaults for the EL5002 SSI settings object 0x80n0.
 * Source: Beckhoff Information System, "EL500x SSI Geber Interface",
 * object 0x80n0 (n = 0 -> Ch.1 at 0x8000, n = 1 -> Ch.2 at 0x8010).
 *   :01 Disable frame error       BOOLEAN  0
 *   :02 Enable power failure bit   BOOLEAN  0
 *   :03 Enable inhibit time        BOOLEAN  0
 *   :06 SSI coding                 BIT1     1  (0 = binary, 1 = gray)
 *   :09 SSI baudrate               BIT3     3  (3 = 500 kBaud)      (FW03+)
 *   :0C SSI clock jitter comp.     BIT3     0                      (FW03+)
 *   :0F SSI frame type             BIT2     0  (0 = multiturn, 25-bit frame)
 *   :11 SSI frame size [bit]       UINT16   25
 *   :12 SSI data length [bit]      UINT16   24
 *   :13 Min. inhibit time [us]     UINT16   0
 *   :14 Number of clock bursts     UINT16   1                      (FW03+)
 * (:04 Enable test mode is intentionally not exposed as a modParam.)
 * ====================================================================== */
#define DEF_DIS_FRAME_ERR     0
#define DEF_EN_PWR_FAIL_CHK   0
#define DEF_EN_INHIBIT_TIME   0
#define DEF_CODING            1   /* gray code         */
#define DEF_BAUDRATE          3   /* 500 kBaud         */
#define DEF_CLK_JIT_COMP      0
#define DEF_FRAME_TYPE        0   /* multiturn, 25-bit */
#define DEF_FRAME_SIZE        25
#define DEF_DATA_LEN          24
#define DEF_MIN_INHIBIT_TIME  0
#define DEF_NO_CLK_BURSTS     1

static int lcec_el5002_init(int comp_id, lcec_slave_t *slave);

static lcec_modparam_desc_t lcec_el5002_modparams[] = {
    {"ch0DisFrameErr", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_DIS_FRAME_ERR, MODPARAM_TYPE_BIT},
    {"ch0EnPwrFailChk", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_EN_PWR_FAIL_CHK, MODPARAM_TYPE_BIT},
    {"ch0EnInhibitTime", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_EN_INHIBIT_TIME, MODPARAM_TYPE_BIT},
    {"ch0Coding", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_CODING, MODPARAM_TYPE_U32},
    {"ch0Baudrate", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_BAUDRATE, MODPARAM_TYPE_U32},
    {"ch0ClkJitComp", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_CLK_JIT_COMP, MODPARAM_TYPE_BIT},
    {"ch0FrameType", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_FRAME_TYPE, MODPARAM_TYPE_U32},
    {"ch0FrameSize", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_FRAME_SIZE, MODPARAM_TYPE_U32},
    {"ch0DataLen", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_DATA_LEN, MODPARAM_TYPE_U32},
    {"ch0MinInhibitTime", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_MIN_INHIBIT_TIME, MODPARAM_TYPE_U32},
    {"ch0NoClkBursts", LCEC_EL5002_PARAM_CH_0 | LCEC_EL5002_PARAM_NO_CLK_BURSTS, MODPARAM_TYPE_U32},
    {"ch1DisFrameErr", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_DIS_FRAME_ERR, MODPARAM_TYPE_BIT},
    {"ch1EnPwrFailChk", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_EN_PWR_FAIL_CHK, MODPARAM_TYPE_BIT},
    {"ch1EnInhibitTime", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_EN_INHIBIT_TIME, MODPARAM_TYPE_BIT},
    {"ch1Coding", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_CODING, MODPARAM_TYPE_U32},
    {"ch1Baudrate", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_BAUDRATE, MODPARAM_TYPE_U32},
    {"ch1ClkJitComp", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_CLK_JIT_COMP, MODPARAM_TYPE_BIT},
    {"ch1FrameType", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_FRAME_TYPE, MODPARAM_TYPE_U32},
    {"ch1FrameSize", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_FRAME_SIZE, MODPARAM_TYPE_U32},
    {"ch1DataLen", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_DATA_LEN, MODPARAM_TYPE_U32},
    {"ch1MinInhibitTime", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_MIN_INHIBIT_TIME, MODPARAM_TYPE_U32},
    {"ch1NoClkBursts", LCEC_EL5002_PARAM_CH_1 | LCEC_EL5002_PARAM_NO_CLK_BURSTS, MODPARAM_TYPE_U32},
    {NULL},
};

static lcec_typelist_t types[] = {
    {"EL5002", LCEC_BECKHOFF_VID, 0x138a3052, 0, NULL, lcec_el5002_init, lcec_el5002_modparams},
    {"EJ5002", LCEC_BECKHOFF_VID, 0x138a2852, 0, NULL, lcec_el5002_init, lcec_el5002_modparams},
    {NULL},
};
ADD_TYPES(types);

typedef struct {
  hal_bit_t *reset;
  hal_bit_t *abs_mode;
  hal_bit_t *err_data;
  hal_bit_t *err_frame;
  hal_bit_t *err_power;
  hal_bit_t *err_sync;
  hal_bit_t *tx_state;
  hal_bit_t *tx_toggle;
  hal_s32_t *raw_count;
  hal_s32_t *count;
  hal_float_t *pos;
  hal_float_t *pos_scale;

  unsigned int err_data_os;
  unsigned int err_data_bp;
  unsigned int err_frame_os;
  unsigned int err_frame_bp;
  unsigned int err_power_os;
  unsigned int err_power_bp;
  unsigned int err_sync_os;
  unsigned int err_sync_bp;
  unsigned int tx_state_os;
  unsigned int tx_state_bp;
  unsigned int tx_toggle_os;
  unsigned int tx_toggle_bp;
  unsigned int count_pdo_os;

  int do_init;
  int32_t last_count;
  double old_scale;
  double scale;
} lcec_el5002_chan_t;

typedef struct {
  lcec_el5002_chan_t chans[LCEC_EL5002_CHANS];
  int last_operational;
} lcec_el5002_data_t;

/* Per-channel SSI settings, pre-loaded with the Beckhoff factory defaults and
 * overridden by any modParam present in the XML config.                       */
typedef struct {
  uint8_t  dis_frame_err;     /* 80n0:01 */
  uint8_t  en_pwr_fail_chk;   /* 80n0:02 */
  uint8_t  en_inhibit_time;   /* 80n0:03 */
  uint8_t  coding;            /* 80n0:06 */
  uint8_t  baudrate;          /* 80n0:09 */
  uint8_t  clk_jit_comp;      /* 80n0:0C */
  uint8_t  frame_type;        /* 80n0:0F */
  uint16_t frame_size;        /* 80n0:11 */
  uint16_t data_len;          /* 80n0:12 */
  uint16_t min_inhibit_time;  /* 80n0:13 */
  uint16_t no_clk_bursts;     /* 80n0:14 */
} el5002_chan_cfg_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_BIT, HAL_IN, offsetof(lcec_el5002_chan_t, reset), "%s.%s.%s.enc-%d-reset"},
    {HAL_BIT, HAL_IN, offsetof(lcec_el5002_chan_t, abs_mode), "%s.%s.%s.enc-%d-abs-mode"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5002_chan_t, err_data), "%s.%s.%s.enc-%d-err-data"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5002_chan_t, err_frame), "%s.%s.%s.enc-%d-err-frame"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5002_chan_t, err_power), "%s.%s.%s.enc-%d-err-power"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5002_chan_t, err_sync), "%s.%s.%s.enc-%d-err-sync"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5002_chan_t, tx_state), "%s.%s.%s.enc-%d-tx-state"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5002_chan_t, tx_toggle), "%s.%s.%s.enc-%d-tx-toggle"},
    {HAL_S32, HAL_OUT, offsetof(lcec_el5002_chan_t, raw_count), "%s.%s.%s.enc-%d-raw-count"},
    {HAL_S32, HAL_OUT, offsetof(lcec_el5002_chan_t, count), "%s.%s.%s.enc-%d-count"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_el5002_chan_t, pos), "%s.%s.%s.enc-%d-pos"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_el5002_chan_t, pos_scale), "%s.%s.%s.enc-%d-pos-scale"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static ec_pdo_entry_info_t lcec_el5002_channel1_in[] = {
    {0x6000, 0x01, 1},   // Data error
    {0x6000, 0x02, 1},   // Frame error
    {0x6000, 0x03, 1},   // Power fail
    {0x6000, 0x04, 1},   // ?
    {0x0000, 0x00, 9},   // Gap
    {0x6000, 0x0e, 1},   // Sync error
    {0x6000, 0x0f, 1},   // TxPDO state
    {0x6000, 0x10, 1},   // TxPDO toggle
    {0x6000, 0x11, 32},  // counter
};

static ec_pdo_entry_info_t lcec_el5002_channel2_in[] = {
    {0x6010, 0x01, 1},   // Data error
    {0x6010, 0x02, 1},   // Frame error
    {0x6010, 0x03, 1},   // Power fail
    {0x6010, 0x04, 1},   // ?
    {0x0010, 0x00, 9},   // Gap
    {0x6010, 0x0e, 1},   // Sync error
    {0x6010, 0x0f, 1},   // TxPDO state
    {0x6010, 0x10, 1},   // TxPDO toggle
    {0x6010, 0x11, 32},  // counter
};

static ec_pdo_info_t lcec_el5002_pdos_in[] = {
    {0x1A00, 9, lcec_el5002_channel1_in},
    {0x1A01, 9, lcec_el5002_channel2_in},
};

static ec_sync_info_t lcec_el5002_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT, 0, NULL},
    {2, EC_DIR_OUTPUT, 0, NULL},
    {3, EC_DIR_INPUT, 2, lcec_el5002_pdos_in},
    {0xff},
};

static void lcec_el5002_read(lcec_slave_t *slave, long period);

static int lcec_el5002_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_slave_modparam_t *p;
  lcec_el5002_data_t *hal_data;
  int i;
  int ch;
  int base;
  lcec_el5002_chan_t *chan;
  el5002_chan_cfg_t cfg[LCEC_EL5002_CHANS];
  int err;

  // pre-load every channel with the Beckhoff factory defaults; any modParam
  // present in the XML overrides its field below.
  for (i = 0; i < LCEC_EL5002_CHANS; i++) {
    cfg[i].dis_frame_err    = DEF_DIS_FRAME_ERR;
    cfg[i].en_pwr_fail_chk  = DEF_EN_PWR_FAIL_CHK;
    cfg[i].en_inhibit_time  = DEF_EN_INHIBIT_TIME;
    cfg[i].coding           = DEF_CODING;
    cfg[i].baudrate         = DEF_BAUDRATE;
    cfg[i].clk_jit_comp     = DEF_CLK_JIT_COMP;
    cfg[i].frame_type       = DEF_FRAME_TYPE;
    cfg[i].frame_size       = DEF_FRAME_SIZE;
    cfg[i].data_len         = DEF_DATA_LEN;
    cfg[i].min_inhibit_time = DEF_MIN_INHIBIT_TIME;
    cfg[i].no_clk_bursts    = DEF_NO_CLK_BURSTS;
  }

  // collect modParam overrides into the per-channel config
  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    ch = p->id & LCEC_EL5002_PARAM_CH_MASK;   // 0 or 1
    if (ch < 0 || ch >= LCEC_EL5002_CHANS) {
      rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "slave %s.%s: modParam id 0x%x has out-of-range channel %d, ignored\n",
        master->name, slave->name, p->id, ch);
      continue;
    }
    switch (p->id & LCEC_EL5002_PARAM_FNK_MASK) {
      case LCEC_EL5002_PARAM_DIS_FRAME_ERR:    cfg[ch].dis_frame_err    = p->value.bit ? 1 : 0; break;
      case LCEC_EL5002_PARAM_EN_PWR_FAIL_CHK:  cfg[ch].en_pwr_fail_chk  = p->value.bit ? 1 : 0; break;
      case LCEC_EL5002_PARAM_EN_INHIBIT_TIME:  cfg[ch].en_inhibit_time  = p->value.bit ? 1 : 0; break;
      case LCEC_EL5002_PARAM_CODING:           cfg[ch].coding           = (uint8_t)p->value.u32; break;
      case LCEC_EL5002_PARAM_BAUDRATE:         cfg[ch].baudrate         = (uint8_t)p->value.u32; break;
      case LCEC_EL5002_PARAM_CLK_JIT_COMP:     cfg[ch].clk_jit_comp     = (uint8_t)p->value.bit; break;
      case LCEC_EL5002_PARAM_FRAME_TYPE:       cfg[ch].frame_type       = (uint8_t)p->value.u32; break;
      case LCEC_EL5002_PARAM_FRAME_SIZE:       cfg[ch].frame_size       = (uint16_t)p->value.u32; break;
      case LCEC_EL5002_PARAM_DATA_LEN:         cfg[ch].data_len         = (uint16_t)p->value.u32; break;
      case LCEC_EL5002_PARAM_MIN_INHIBIT_TIME: cfg[ch].min_inhibit_time = (uint16_t)p->value.u32; break;
      case LCEC_EL5002_PARAM_NO_CLK_BURSTS:    cfg[ch].no_clk_bursts    = (uint16_t)p->value.u32; break;
      default:
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "slave %s.%s: unknown modParam id 0x%x ignored\n",
          master->name, slave->name, p->id);
        break;
    }
  }

  // write SSI settings (object 0x80n0) per channel.  Only objects that differ
  // from the Beckhoff factory default are written, so an unset XML parameter
  // leaves the device at its own (identical) default and produces no traffic.
  // Failures are non-fatal: the device keeps its current value.
  for (ch = 0; ch < LCEC_EL5002_CHANS; ch++) {
    base = 0x8000 + (ch << 4);

    if (cfg[ch].dis_frame_err != DEF_DIS_FRAME_ERR)
      if (lcec_write_sdo8(slave, base, 0x01, cfg[ch].dis_frame_err) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:01 (DisFrameErr) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].en_pwr_fail_chk != DEF_EN_PWR_FAIL_CHK)
      if (lcec_write_sdo8(slave, base, 0x02, cfg[ch].en_pwr_fail_chk) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:02 (EnPwrFailChk) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].en_inhibit_time != DEF_EN_INHIBIT_TIME)
      if (lcec_write_sdo8(slave, base, 0x03, cfg[ch].en_inhibit_time) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:03 (EnInhibitTime) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].coding != DEF_CODING)
      if (lcec_write_sdo8(slave, base, 0x06, cfg[ch].coding) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:06 (Coding) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].baudrate != DEF_BAUDRATE)
      if (lcec_write_sdo8(slave, base, 0x09, cfg[ch].baudrate) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:09 (Baudrate) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].clk_jit_comp != DEF_CLK_JIT_COMP)
      if (lcec_write_sdo8(slave, base, 0x0c, cfg[ch].clk_jit_comp) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:0C (ClkJitComp) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].frame_type != DEF_FRAME_TYPE)
      if (lcec_write_sdo8(slave, base, 0x0f, cfg[ch].frame_type) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:0F (FrameType) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].frame_size != DEF_FRAME_SIZE)
      if (lcec_write_sdo16(slave, base, 0x11, cfg[ch].frame_size) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:11 (FrameSize) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].data_len != DEF_DATA_LEN)
      if (lcec_write_sdo16(slave, base, 0x12, cfg[ch].data_len) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:12 (DataLen) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].min_inhibit_time != DEF_MIN_INHIBIT_TIME)
      if (lcec_write_sdo16(slave, base, 0x13, cfg[ch].min_inhibit_time) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:13 (MinInhibitTime) write failed\n", master->name, slave->name, ch, base);

    if (cfg[ch].no_clk_bursts != DEF_NO_CLK_BURSTS)
      if (lcec_write_sdo16(slave, base, 0x14, cfg[ch].no_clk_bursts) != 0)
        rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX
          "slave %s.%s ch%d: SDO %04x:14 (NoClkBursts) write failed\n", master->name, slave->name, ch, base);
  }

  // initialize callbacks
  slave->proc_read = lcec_el5002_read;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_el5002_data_t);
  slave->hal_data = hal_data;

  // initialize sync info
  slave->sync_info = lcec_el5002_syncs;

  // initialize global data
  hal_data->last_operational = 0;

  // initialize pins
  for (i = 0; i < LCEC_EL5002_CHANS; i++) {
    chan = &hal_data->chans[i];

    // initialize POD entries
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x01, &chan->err_data_os, &chan->err_data_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x02, &chan->err_frame_os, &chan->err_frame_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x03, &chan->err_power_os, &chan->err_power_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x0e, &chan->err_sync_os, &chan->err_sync_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x0f, &chan->tx_state_os, &chan->tx_state_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x10, &chan->tx_toggle_os, &chan->tx_toggle_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x11, &chan->count_pdo_os, NULL);

    // export pins
    if ((err = lcec_pin_newf_list(chan, slave_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0) {
      return err;
    }

    // initialize pins
    LCEC_PIN_FLOAT_SET(chan->pos_scale, 1.0);

    // initialize variables
    chan->do_init = 1;
    chan->last_count = 0;
    chan->old_scale = LCEC_PIN_FLOAT_GET(chan->pos_scale) + 1.0;
    chan->scale = 1.0;
  }

  return 0;
}

static void lcec_el5002_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_el5002_data_t *hal_data = (lcec_el5002_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  int i;
  lcec_el5002_chan_t *chan;
  int32_t raw_count, raw_delta;

  // wait for slave to be operational
  if (!slave->state.operational) {
    hal_data->last_operational = 0;
    return;
  }

  // check inputs
  for (i = 0; i < LCEC_EL5002_CHANS; i++) {
    chan = &hal_data->chans[i];

    // check for change in scale value
    if (LCEC_PIN_FLOAT_GET(chan->pos_scale) != chan->old_scale) {
      // scale value has changed, test and update it
      if ((LCEC_PIN_FLOAT_GET(chan->pos_scale) < 1e-20) && (LCEC_PIN_FLOAT_GET(chan->pos_scale) > -1e-20)) {
        // value too small, divide by zero is a bad thing
        LCEC_PIN_FLOAT_SET(chan->pos_scale, 1.0);
      }
      // save new scale to detect future changes
      chan->old_scale = LCEC_PIN_FLOAT_GET(chan->pos_scale);
      // we actually want the reciprocal
      chan->scale = 1.0 / LCEC_PIN_FLOAT_GET(chan->pos_scale);
    }

    // get bit states
    LCEC_PIN_BIT_SET(chan->err_data, EC_READ_BIT(&pd[chan->err_data_os], chan->err_data_bp));
    LCEC_PIN_BIT_SET(chan->err_frame, EC_READ_BIT(&pd[chan->err_frame_os], chan->err_frame_bp));
    LCEC_PIN_BIT_SET(chan->err_power, EC_READ_BIT(&pd[chan->err_power_os], chan->err_power_bp));
    LCEC_PIN_BIT_SET(chan->err_sync, EC_READ_BIT(&pd[chan->err_sync_os], chan->err_sync_bp));
    LCEC_PIN_BIT_SET(chan->tx_state, EC_READ_BIT(&pd[chan->tx_state_os], chan->tx_state_bp));
    LCEC_PIN_BIT_SET(chan->tx_toggle, EC_READ_BIT(&pd[chan->tx_toggle_os], chan->tx_toggle_bp));

    // read raw values
    raw_count = EC_READ_S32(&pd[chan->count_pdo_os]);

    // check for operational change of slave
    if (!hal_data->last_operational) {
      chan->last_count = raw_count;
    }

    // update raw values
    LCEC_PIN_S32_SET(chan->raw_count, raw_count);

    // handle initialization
    if (chan->do_init || LCEC_PIN_BIT_GET(chan->reset)) {
      chan->do_init = 0;
      chan->last_count = raw_count;
      LCEC_PIN_S32_SET(chan->count, 0);
    }

    // compute net counts
    raw_delta = raw_count - chan->last_count;
    chan->last_count = raw_count;
    LCEC_PIN_S32_SET(chan->count, LCEC_PIN_S32_GET(chan->count) + raw_delta);

    // scale count to make floating point position
    if (LCEC_PIN_BIT_GET(chan->abs_mode)) {
      LCEC_PIN_FLOAT_SET(chan->pos, LCEC_PIN_S32_GET(chan->raw_count) * chan->scale);
    } else {
      LCEC_PIN_FLOAT_SET(chan->pos, LCEC_PIN_S32_GET(chan->count) * chan->scale);
    }
  }

  hal_data->last_operational = 1;
}

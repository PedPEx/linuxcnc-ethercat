// SPDX-License-Identifier: GPL-2.0-or-later
/// @file lcec_el6002.c
/// @brief LinuxCNC EtherCAT HAL driver for Beckhoff EL6002 (2× RS232)
///
/// ── INSTALLATION ──────────────────────────────────────────────────────────
///   cp lcec_el6002.{c,h}  <linuxcnc-ethercat>/src/devices/
///   make && sudo make install
///   No changes to lcec_main.c, Kbuild or Makefile needed.
///   ADD_TYPES() registers the driver automatically via a constructor.
///
/// ── ethercat-conf.xml ─────────────────────────────────────────────────────
///   Replace the <slave idx="8" type="generic" …> entry with:
///
///   <slave idx="8" type="EL6002" name="D9">
///     <!-- Kanal 1: 9600 Baud, 8N1 -->
///     <sdoConfig idx="0x8000" subIdx="0x01" type="uint8" value="6"/>
///     <sdoConfig idx="0x8000" subIdx="0x02" type="uint8" value="3"/>
///     <sdoConfig idx="0x8000" subIdx="0x03" type="uint8" value="1"/>
///     <sdoConfig idx="0x8000" subIdx="0x04" type="uint8" value="0"/>
///     <!-- Kanal 2: 0x8010 statt 0x8000 -->
///   </slave>
///
///   Baudrate-Codes: 1=1200 3=2400 4=4800 6=9600 7=19200
///                   9=38400 0xA=57600 0xC=115200

#include "lcec_el6002.h"
#include <string.h>

// ═══════════════════════════════════════════════════════════════════════════
// PDO object indices — verified with `ethercat pdos -p 8`
// ═══════════════════════════════════════════════════════════════════════════

// Control/Status word: separate object (base+1), 16-bit
// CH1: Ctrl=0x7001:01  Status=0x6001:01
// CH2: Ctrl=0x7011:01  Status=0x6011:01
#define EL6002_CTRL_IDX(ch)    (0x7001u + (uint16_t)((ch) * 0x10))
#define EL6002_STAT_IDX(ch)    (0x6001u + (uint16_t)((ch) * 0x10))
#define EL6002_CTRLSTAT_SI     0x01u

// Data bytes: different base object, subindex 0x11..0x26
// CH1 TX: 0x7000:0x11..0x26   CH2 TX: 0x7010:0x11..0x26
// CH1 RX: 0x6000:0x11..0x26   CH2 RX: 0x6010:0x11..0x26
#define EL6002_TXDATA_IDX(ch)  (0x7000u + (uint16_t)((ch) * 0x10))
#define EL6002_RXDATA_IDX(ch)  (0x6000u + (uint16_t)((ch) * 0x10))
#define EL6002_DATA_SI_FIRST   0x11u   // subindex of first data byte

// Control word bit fields
#define EL6002_CTRL_OUT_ACCEPT  0x0001u   // bit 0: RX ack toggle
#define EL6002_CTRL_SEND_REQ    0x0002u   // bit 1: TX send request toggle
#define EL6002_CTRL_LEN_SHIFT   2         // bits 7:2 = TX byte count
#define EL6002_CTRL_LEN_MASK    0x00FCu

// Status word bit fields
#define EL6002_STAT_IN_ACCEPT   0x0001u   // bit 0: TX ack from slave
#define EL6002_STAT_RECV_REQ    0x0002u   // bit 1: new RX data toggle
#define EL6002_STAT_LEN_SHIFT   2         // bits 7:2 = RX byte count
#define EL6002_STAT_LEN_MASK    0x00FCu

// ═══════════════════════════════════════════════════════════════════════════
// PDO sync-manager configuration
// SM2 (output): RxPDO 0x1604 (CH1) + 0x1605 (CH2)
// SM3 (input):  TxPDO 0x1A04 (CH1) + 0x1A05 (CH2)
// ═══════════════════════════════════════════════════════════════════════════

#define DATA_ENTRIES_8BIT(base)                                                \
  {(base), 0x11, 8}, {(base), 0x12, 8}, {(base), 0x13, 8}, {(base), 0x14, 8}, \
  {(base), 0x15, 8}, {(base), 0x16, 8}, {(base), 0x17, 8}, {(base), 0x18, 8}, \
  {(base), 0x19, 8}, {(base), 0x1a, 8}, {(base), 0x1b, 8}, {(base), 0x1c, 8}, \
  {(base), 0x1d, 8}, {(base), 0x1e, 8}, {(base), 0x1f, 8}, {(base), 0x20, 8}, \
  {(base), 0x21, 8}, {(base), 0x22, 8}, {(base), 0x23, 8}, {(base), 0x24, 8}, \
  {(base), 0x25, 8}, {(base), 0x26, 8}

static ec_pdo_entry_info_t lcec_el6002_pdo_entries[] = {
  // RxPDO 0x1604 — CH1 output
  {0x7001, 0x01, 16}, DATA_ENTRIES_8BIT(0x7000),
  // RxPDO 0x1605 — CH2 output
  {0x7011, 0x01, 16}, DATA_ENTRIES_8BIT(0x7010),
  // TxPDO 0x1A04 — CH1 input
  {0x6001, 0x01, 16}, DATA_ENTRIES_8BIT(0x6000),
  // TxPDO 0x1A05 — CH2 input
  {0x6011, 0x01, 16}, DATA_ENTRIES_8BIT(0x6010),
};

#define ENTRIES_PER_PDO 23  // 1 ctrl/status + 22 data bytes

static ec_pdo_info_t lcec_el6002_pdos[] = {
  {0x1604, ENTRIES_PER_PDO, lcec_el6002_pdo_entries +  0},  // RxPDO CH1
  {0x1605, ENTRIES_PER_PDO, lcec_el6002_pdo_entries + 23},  // RxPDO CH2
  {0x1a04, ENTRIES_PER_PDO, lcec_el6002_pdo_entries + 46},  // TxPDO CH1
  {0x1a05, ENTRIES_PER_PDO, lcec_el6002_pdo_entries + 69},  // TxPDO CH2
};

static ec_sync_info_t lcec_el6002_syncs[] = {
  {0, EC_DIR_OUTPUT, 0, NULL,                  EC_WD_DISABLE},
  {1, EC_DIR_INPUT,  0, NULL,                  EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 2, lcec_el6002_pdos + 0,  EC_WD_ENABLE },
  {3, EC_DIR_INPUT,  2, lcec_el6002_pdos + 2,  EC_WD_DISABLE},
  {0xff},
};

// ═══════════════════════════════════════════════════════════════════════════
// HAL data structures
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
  // TX HAL pins (master → serial port)
  hal_u32_t *tx_len;
  hal_u32_t *tx_data[LCEC_EL6002_DATA_PINS];
  hal_bit_t *tx_busy;

  // RX HAL pins (serial port → master)
  hal_u32_t *rx_len;
  hal_u32_t *rx_data[LCEC_EL6002_DATA_PINS];
  hal_bit_t *rx_ready;

  // PDO byte offsets in the process image
  unsigned int ctrl_os;                           // output: Ctrl word
  unsigned int tx_byte_os[LCEC_EL6002_MAX_DATA];  // output: data bytes
  unsigned int stat_os;                           // input:  Status word
  unsigned int rx_byte_os[LCEC_EL6002_MAX_DATA];  // input:  data bytes

  // Protocol state
  uint16_t tx_ctrl;          // current ctrl word value we write each cycle
  uint16_t tx_send_toggle;   // current "Send request" bit state
  uint16_t rx_req_last;      // last seen "Receive request" bit
  uint16_t tx_ack_last;      // last seen "Input accepted" bit
} lcec_el6002_chan_t;

typedef struct {
  lcec_el6002_chan_t chans[LCEC_EL6002_CHANS];
} lcec_el6002_data_t;

// Forward declarations
static int  lcec_el6002_preinit(lcec_slave_t *slave);
static int  lcec_el6002_init(int comp_id, lcec_slave_t *slave);
static void lcec_el6002_read(lcec_slave_t *slave, long period);
static void lcec_el6002_write(lcec_slave_t *slave, long period);

// ═══════════════════════════════════════════════════════════════════════════
// Device type registration
// Positional: {name, vid, pid, is_fsoe, preinit, init, modparams}
// ═══════════════════════════════════════════════════════════════════════════

static lcec_typelist_t types[] = {
  {"EL6002", LCEC_BECKHOFF_VID, LCEC_EL6002_PID, 0, lcec_el6002_preinit, lcec_el6002_init, NULL},
  {NULL},
};
ADD_TYPES(types);

// ═══════════════════════════════════════════════════════════════════════════
// preinit — configure PDO mapping before the domain is activated
// Called with slave->config already set up by the framework.
// Signature: int (*lcec_slave_preinit_t)(lcec_slave_t *slave)
// ═══════════════════════════════════════════════════════════════════════════

static int lcec_el6002_preinit(lcec_slave_t *slave) {
  if (!slave->config) return 0;

  if (ecrt_slave_config_pdos(slave->config, EC_END, lcec_el6002_syncs)) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "EL6002 %s.%s: ecrt_slave_config_pdos failed\n",
        slave->master->name, slave->name);
    return -EIO;
  }
  return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers: pack/unpack bytes ↔ u32 HAL pins (little-endian)
// ═══════════════════════════════════════════════════════════════════════════

static inline uint8_t chan_tx_byte(const lcec_el6002_chan_t *c, int i) {
  return (uint8_t)((*c->tx_data[i / 4] >> ((i % 4) * 8)) & 0xFFu);
}

static inline void chan_set_rx_byte(lcec_el6002_chan_t *c, int i, uint8_t v) {
  int w = i / 4, s = (i % 4) * 8;
  *c->rx_data[w] = (*c->rx_data[w] & ~(0xFFu << s)) | ((uint32_t)v << s);
}

// ═══════════════════════════════════════════════════════════════════════════
// init — register PDOs and create HAL pins
// Signature: int (*lcec_slave_init_t)(int comp_id, lcec_slave_t *slave)
// ═══════════════════════════════════════════════════════════════════════════

static int lcec_el6002_init(int comp_id, lcec_slave_t *slave) {
  lcec_el6002_data_t *hal;
  int ch, i, err;

  // Allocate HAL shared memory
  hal = LCEC_HAL_ALLOCATE(*hal);
  if (!hal) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "EL6002 %s.%s: HAL alloc failed\n",
        slave->master->name, slave->name);
    return -ENOMEM;
  }
  memset(hal, 0, sizeof(*hal));
  slave->hal_data = hal;

  for (ch = 0; ch < LCEC_EL6002_CHANS; ch++) {
    lcec_el6002_chan_t *c = &hal->chans[ch];

    // ── PDO entry registration ──────────────────────────────────────────
    // lcec_pdo_init(slave, object_idx, subidx, &byte_offset, &bit_pos)
    // bit_pos may be NULL for byte-aligned entries

    // Output side: Ctrl word (16-bit) at 0x7001:01 or 0x7011:01
    if ((err = lcec_pdo_init(slave,
            EL6002_CTRL_IDX(ch), EL6002_CTRLSTAT_SI,
            &c->ctrl_os, NULL)) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "EL6002: pdo_init ctrl ch%d failed\n", ch);
      return err;
    }

    // Output side: 22 data bytes at 0x7000:0x11..0x26
    for (i = 0; i < LCEC_EL6002_MAX_DATA; i++) {
      if ((err = lcec_pdo_init(slave,
              EL6002_TXDATA_IDX(ch),
              (uint8_t)(EL6002_DATA_SI_FIRST + i),
              &c->tx_byte_os[i], NULL)) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            LCEC_MSG_PFX "EL6002: pdo_init txdata[%d] ch%d failed\n", i, ch);
        return err;
      }
    }

    // Input side: Status word (16-bit) at 0x6001:01 or 0x6011:01
    if ((err = lcec_pdo_init(slave,
            EL6002_STAT_IDX(ch), EL6002_CTRLSTAT_SI,
            &c->stat_os, NULL)) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "EL6002: pdo_init stat ch%d failed\n", ch);
      return err;
    }

    // Input side: 22 data bytes at 0x6000:0x11..0x26
    for (i = 0; i < LCEC_EL6002_MAX_DATA; i++) {
      if ((err = lcec_pdo_init(slave,
              EL6002_RXDATA_IDX(ch),
              (uint8_t)(EL6002_DATA_SI_FIRST + i),
              &c->rx_byte_os[i], NULL)) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            LCEC_MSG_PFX "EL6002: pdo_init rxdata[%d] ch%d failed\n", i, ch);
        return err;
      }
    }

    // ── HAL pin creation ─────────────────────────────────────────────────
#define MKPIN(type, dir, ptr, fmt, ...)                                     \
    do {                                                                    \
      if ((err = hal_pin_##type##_newf(dir, ptr, comp_id,                   \
               "lcec.%s.%s.ch%d." fmt,                                     \
               slave->master->name, slave->name, ch,                       \
               ##__VA_ARGS__)) != 0) {                                      \
        rtapi_print_msg(RTAPI_MSG_ERR,                                      \
            LCEC_MSG_PFX "EL6002: pin alloc failed (%d)\n", err);           \
        return err;                                                         \
      }                                                                     \
    } while (0)

    MKPIN(u32, HAL_IN,  &c->tx_len,  "tx-len");
    MKPIN(bit, HAL_OUT, &c->tx_busy, "tx-busy");
    for (i = 0; i < LCEC_EL6002_DATA_PINS; i++)
      MKPIN(u32, HAL_IN, &c->tx_data[i], "tx-data-%d", i);

    MKPIN(u32, HAL_OUT, &c->rx_len,   "rx-len");
    MKPIN(bit, HAL_OUT, &c->rx_ready, "rx-ready");
    for (i = 0; i < LCEC_EL6002_DATA_PINS; i++)
      MKPIN(u32, HAL_OUT, &c->rx_data[i], "rx-data-%d", i);

#undef MKPIN

    // Initialise protocol state
    c->tx_ctrl        = 0;
    c->tx_send_toggle = 0;
    c->rx_req_last    = 0;
    c->tx_ack_last    = 0;
  }

  slave->proc_read  = lcec_el6002_read;
  slave->proc_write = lcec_el6002_write;
  return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// read — process image → HAL pins  (every servo cycle)
//
// RX: detect toggle edge on "Receive request" → copy bytes → toggle rx-ready
//     → schedule acknowledgement (written in write())
// TX: detect toggle edge on "Input accepted"  → clear tx-busy
// ═══════════════════════════════════════════════════════════════════════════

static void lcec_el6002_read(lcec_slave_t *slave, long period) {
  lcec_el6002_data_t *hal = slave->hal_data;
  uint8_t *pd = slave->master->process_data;
  int ch, i;

  if (!slave->state.operational) return;

  for (ch = 0; ch < LCEC_EL6002_CHANS; ch++) {
    lcec_el6002_chan_t *c = &hal->chans[ch];
    uint16_t stat = EC_READ_U16(pd + c->stat_os);

    // ── TX acknowledgement ─────────────────────────────────────────────
    uint16_t tx_ack = stat & EL6002_STAT_IN_ACCEPT ? 1u : 0u;
    if (tx_ack != c->tx_ack_last) {
      c->tx_ack_last = tx_ack;
      *c->tx_busy = 0;
    }

    // ── RX new data ────────────────────────────────────────────────────
    uint16_t rx_req = stat & EL6002_STAT_RECV_REQ ? 1u : 0u;
    if (rx_req != c->rx_req_last) {
      c->rx_req_last = rx_req;

      uint32_t len = (stat & EL6002_STAT_LEN_MASK) >> EL6002_STAT_LEN_SHIFT;
      if (len > LCEC_EL6002_MAX_DATA) len = LCEC_EL6002_MAX_DATA;
      *c->rx_len = len;

      for (i = 0; i < LCEC_EL6002_DATA_PINS; i++) *c->rx_data[i] = 0;
      for (i = 0; i < (int)len; i++)
        chan_set_rx_byte(c, i, EC_READ_U8(pd + c->rx_byte_os[i]));

      // Toggle rx-ready so HAL consumers see the edge
      *c->rx_ready ^= 1;

      // Acknowledge: mirror "Receive request" in "Output accept" (bit 0)
      if (rx_req)
        c->tx_ctrl |=  EL6002_CTRL_OUT_ACCEPT;
      else
        c->tx_ctrl &= ~EL6002_CTRL_OUT_ACCEPT;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// write — HAL pins → process image  (every servo cycle)
//
// If tx-len > 0 and not busy: pack bytes, encode length, toggle Send request.
// Always writes the Ctrl word (keeps the RX ack bit current).
// ═══════════════════════════════════════════════════════════════════════════

static void lcec_el6002_write(lcec_slave_t *slave, long period) {
  lcec_el6002_data_t *hal = slave->hal_data;
  uint8_t *pd = slave->master->process_data;
  int ch, i;

  for (ch = 0; ch < LCEC_EL6002_CHANS; ch++) {
    lcec_el6002_chan_t *c = &hal->chans[ch];

    uint32_t tx_len = *c->tx_len;
    if (tx_len > LCEC_EL6002_MAX_DATA) tx_len = LCEC_EL6002_MAX_DATA;

    if (tx_len > 0 && !*c->tx_busy) {
      // Write data bytes
      for (i = 0; i < (int)tx_len; i++)
        EC_WRITE_U8(pd + c->tx_byte_os[i], chan_tx_byte(c, i));
      for (i = (int)tx_len; i < LCEC_EL6002_MAX_DATA; i++)
        EC_WRITE_U8(pd + c->tx_byte_os[i], 0);

      // Encode length in ctrl bits 7:2
      c->tx_ctrl &= ~EL6002_CTRL_LEN_MASK;
      c->tx_ctrl |= (uint16_t)((tx_len << EL6002_CTRL_LEN_SHIFT) & EL6002_CTRL_LEN_MASK);

      // Toggle "Send request" bit
      c->tx_send_toggle ^= 1u;
      if (c->tx_send_toggle)
        c->tx_ctrl |=  EL6002_CTRL_SEND_REQ;
      else
        c->tx_ctrl &= ~EL6002_CTRL_SEND_REQ;

      *c->tx_busy = 1;
    }

    // Always write the ctrl word every cycle
    EC_WRITE_U16(pd + c->ctrl_os, c->tx_ctrl);
  }
}

//
//    Driver for Beckhoff EK1914 EtherCAT coupler with TwinSAFE I/O.
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//

/// @file
/// @brief Driver for Beckhoff EK1914 (2 Ch. Safety I/O 24V, TwinSAFE)
///
/// The EK1914 is an FSoE *slave*. It exchanges its safe I/O with a TwinSAFE
/// logic device (here: the EL1918 at bus index 1) over one FSoE connection.
/// This driver exposes the standard (non-safe) DI/DO to HAL and registers the
/// terminal as an FSoE slave (fsoeConf in preinit + copy_fsoe_data in read),
/// so the EL1918_LOGIC connection referenced by its fsoeSlaveIdx resolves.
///
/// The decoded state of the *safe* I/O is not available on the black channel;
/// the fsoe-in-*/fsoe-out-* pins are raw-payload diagnostics only. To read a
/// safe input in HAL, route it to a Standard Output of the EL1918 safety
/// program and read the EL1918 std-out pin.
///
/// PDO layout (ethercat pdos -p 0 / cstruct -p 0), fixed mapping:
///   SM2 OUT  0x1600 FSOE RxPDO (master->slave frame): 7000:01 CMD,
///            7001:01/02 safe out, 7000:03 ConnID, 7000:02 CRC
///   SM2 OUT  0x1601 DIO Outputs: 7010:01-04 std out, 7010:05/06 safety-linked
///   SM3 IN   0x1a00 FSOE TxPDO (slave->master frame): 6000:01 CMD,
///            6001:01/02 safe in, 6000:03 ConnID, 6000:02 CRC
///   SM3 IN   0x1a01 DIO Inputs: 6010:01-04 std in

#include "lcec_ek1914.h"

#include "../lcec.h"

static void lcec_ek1914_read(lcec_slave_t *slave, long period);
static void lcec_ek1914_write(lcec_slave_t *slave, long period);
static int lcec_ek1914_preinit(lcec_slave_t *slave);
static int lcec_ek1914_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {"EK1914", LCEC_BECKHOFF_VID, LCEC_EK1914_PID, 0, lcec_ek1914_preinit, lcec_ek1914_init},
    {NULL},
};
ADD_TYPES(types);

// per-channel structs (each gets its own HAL memory, addressed via &array[i])
typedef struct {
  hal_bit_t *fsoe_in;
  hal_bit_t *fsoe_in_not;
  unsigned int os;
  unsigned int bp;
} ek1914_sin_t;  // fail-safe input channel (diagnostic)

typedef struct {
  hal_bit_t *fsoe_out;
  unsigned int os;
  unsigned int bp;
} ek1914_sout_t;  // fail-safe output channel (diagnostic)

typedef struct {
  hal_bit_t *pin;
  unsigned int os;
  unsigned int bp;
} ek1914_dio_t;  // standard DI or DO channel

typedef struct {
  // FSoE frame headers (diagnostic)
  hal_u32_t *fsoe_master_cmd;
  hal_u32_t *fsoe_master_crc;
  hal_u32_t *fsoe_master_connid;
  hal_u32_t *fsoe_slave_cmd;
  hal_u32_t *fsoe_slave_crc;
  hal_u32_t *fsoe_slave_connid;

  unsigned int fsoe_master_cmd_os;
  unsigned int fsoe_master_crc_os;
  unsigned int fsoe_master_connid_os;
  unsigned int fsoe_slave_cmd_os;
  unsigned int fsoe_slave_crc_os;
  unsigned int fsoe_slave_connid_os;

  ek1914_sin_t  sin[LCEC_EK1914_SIN_COUNT];
  ek1914_sout_t sout[LCEC_EK1914_SOUT_COUNT];
  ek1914_dio_t  din[LCEC_EK1914_DIN_COUNT];
  ek1914_dio_t  dout[LCEC_EK1914_DOUT_COUNT];
  ek1914_dio_t  slout[LCEC_EK1914_SLOUT_COUNT];
} lcec_ek1914_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_master_cmd), "%s.%s.%s.fsoe-master-cmd"},
    {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_master_crc), "%s.%s.%s.fsoe-master-crc"},
    {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_master_connid), "%s.%s.%s.fsoe-master-connid"},
    {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_slave_cmd), "%s.%s.%s.fsoe-slave-cmd"},
    {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_slave_crc), "%s.%s.%s.fsoe-slave-crc"},
    {HAL_U32, HAL_OUT, offsetof(lcec_ek1914_data_t, fsoe_slave_connid), "%s.%s.%s.fsoe-slave-connid"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static const lcec_pindesc_t sin_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(ek1914_sin_t, fsoe_in), "%s.%s.%s.fsoe-in-%d"},
    {HAL_BIT, HAL_OUT, offsetof(ek1914_sin_t, fsoe_in_not), "%s.%s.%s.fsoe-in-%d-not"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};
static const lcec_pindesc_t sout_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(ek1914_sout_t, fsoe_out), "%s.%s.%s.fsoe-out-%d"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};
static const lcec_pindesc_t din_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(ek1914_dio_t, pin), "%s.%s.%s.din-%d"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};
static const lcec_pindesc_t dout_pins[] = {
    {HAL_BIT, HAL_IN, offsetof(ek1914_dio_t, pin), "%s.%s.%s.dout-%d"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};
static const lcec_pindesc_t slout_pins[] = {
    {HAL_BIT, HAL_IN, offsetof(ek1914_dio_t, pin), "%s.%s.%s.safe-linked-out-%d"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// FSoE frame dimensions: 1 safe-data byte per direction, 1 channel.
// LCEC_FSOE_SIZE(1, 1) = 1 + 1*(1+2) + 2 = 6 bytes, matches 0x1600 / 0x1a00.
static const LCEC_CONF_FSOE_T fsoe_conf = {
    .slave_data_len = 1,
    .master_data_len = 1,
    .data_channels = 1,
};

static int lcec_ek1914_preinit(lcec_slave_t *slave) {
  // publish FSoE config so the EL1918_LOGIC second pass accepts this slave
  slave->fsoeConf = &fsoe_conf;
  return 0;
}

static int lcec_ek1914_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_ek1914_data_t *hal_data;
  int i, err;

  slave->proc_read = lcec_ek1914_read;
  slave->proc_write = lcec_ek1914_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_ek1914_data_t);
  slave->hal_data = hal_data;

  // --- PDO entries: master->slave FSoE frame (0x1600) ---
  lcec_pdo_init(slave, 0x7000, 0x01, &hal_data->fsoe_master_cmd_os, NULL);
  lcec_pdo_init(slave, 0x7000, 0x02, &hal_data->fsoe_master_crc_os, NULL);
  lcec_pdo_init(slave, 0x7000, 0x03, &hal_data->fsoe_master_connid_os, NULL);
  for (i = 0; i < LCEC_EK1914_SOUT_COUNT; i++)
    lcec_pdo_init(slave, 0x7001, 0x01 + i, &hal_data->sout[i].os, &hal_data->sout[i].bp);

  // --- PDO entries: standard outputs (0x1601) ---
  for (i = 0; i < LCEC_EK1914_DOUT_COUNT; i++)
    lcec_pdo_init(slave, 0x7010, 0x01 + i, &hal_data->dout[i].os, &hal_data->dout[i].bp);
  for (i = 0; i < LCEC_EK1914_SLOUT_COUNT; i++)
    lcec_pdo_init(slave, 0x7010, 0x05 + i, &hal_data->slout[i].os, &hal_data->slout[i].bp);

  // --- PDO entries: slave->master FSoE frame (0x1a00) ---
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->fsoe_slave_cmd_os, NULL);
  lcec_pdo_init(slave, 0x6000, 0x02, &hal_data->fsoe_slave_crc_os, NULL);
  lcec_pdo_init(slave, 0x6000, 0x03, &hal_data->fsoe_slave_connid_os, NULL);
  for (i = 0; i < LCEC_EK1914_SIN_COUNT; i++)
    lcec_pdo_init(slave, 0x6001, 0x01 + i, &hal_data->sin[i].os, &hal_data->sin[i].bp);

  // --- PDO entries: standard inputs (0x1a01) ---
  for (i = 0; i < LCEC_EK1914_DIN_COUNT; i++)
    lcec_pdo_init(slave, 0x6010, 0x01 + i, &hal_data->din[i].os, &hal_data->din[i].bp);

  // --- export pins (per-channel lists use a per-element base pointer) ---
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0)
    return err;
  for (i = 0; i < LCEC_EK1914_SIN_COUNT; i++)
    if ((err = lcec_pin_newf_list(&hal_data->sin[i], sin_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0)
      return err;
  for (i = 0; i < LCEC_EK1914_SOUT_COUNT; i++)
    if ((err = lcec_pin_newf_list(&hal_data->sout[i], sout_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0)
      return err;
  for (i = 0; i < LCEC_EK1914_DIN_COUNT; i++)
    if ((err = lcec_pin_newf_list(&hal_data->din[i], din_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0)
      return err;
  for (i = 0; i < LCEC_EK1914_DOUT_COUNT; i++)
    if ((err = lcec_pin_newf_list(&hal_data->dout[i], dout_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0)
      return err;
  for (i = 0; i < LCEC_EK1914_SLOUT_COUNT; i++)
    if ((err = lcec_pin_newf_list(&hal_data->slout[i], slout_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0)
      return err;

  return 0;
}

static void lcec_ek1914_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ek1914_data_t *hal_data = (lcec_ek1914_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  int i;

  // black-channel transport of the FSoE frame for this connection
  copy_fsoe_data(slave, hal_data->fsoe_slave_cmd_os, hal_data->fsoe_master_cmd_os);

  LCEC_PIN_U32_SET(hal_data->fsoe_slave_cmd, EC_READ_U8(&pd[hal_data->fsoe_slave_cmd_os]));
  LCEC_PIN_U32_SET(hal_data->fsoe_slave_crc, EC_READ_U16(&pd[hal_data->fsoe_slave_crc_os]));
  LCEC_PIN_U32_SET(hal_data->fsoe_slave_connid, EC_READ_U16(&pd[hal_data->fsoe_slave_connid_os]));
  LCEC_PIN_U32_SET(hal_data->fsoe_master_cmd, EC_READ_U8(&pd[hal_data->fsoe_master_cmd_os]));
  LCEC_PIN_U32_SET(hal_data->fsoe_master_crc, EC_READ_U16(&pd[hal_data->fsoe_master_crc_os]));
  LCEC_PIN_U32_SET(hal_data->fsoe_master_connid, EC_READ_U16(&pd[hal_data->fsoe_master_connid_os]));

  // fail-safe payload (diagnostic)
  for (i = 0; i < LCEC_EK1914_SIN_COUNT; i++) {
    LCEC_PIN_BIT_SET(hal_data->sin[i].fsoe_in, EC_READ_BIT(&pd[hal_data->sin[i].os], hal_data->sin[i].bp));
    LCEC_PIN_BIT_SET(hal_data->sin[i].fsoe_in_not, !LCEC_PIN_BIT_GET(hal_data->sin[i].fsoe_in));
  }
  for (i = 0; i < LCEC_EK1914_SOUT_COUNT; i++)
    LCEC_PIN_BIT_SET(hal_data->sout[i].fsoe_out, EC_READ_BIT(&pd[hal_data->sout[i].os], hal_data->sout[i].bp));

  // standard digital inputs
  for (i = 0; i < LCEC_EK1914_DIN_COUNT; i++)
    LCEC_PIN_BIT_SET(hal_data->din[i].pin, EC_READ_BIT(&pd[hal_data->din[i].os], hal_data->din[i].bp));
}

static void lcec_ek1914_write(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ek1914_data_t *hal_data = (lcec_ek1914_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  int i;

  // Only standard outputs are written here. The FSoE master frame (0x1600)
  // is owned by the EL1918_LOGIC / copy_fsoe_data path and must not be touched.
  for (i = 0; i < LCEC_EK1914_DOUT_COUNT; i++)
    EC_WRITE_BIT(&pd[hal_data->dout[i].os], hal_data->dout[i].bp, LCEC_PIN_BIT_GET(hal_data->dout[i].pin));
  for (i = 0; i < LCEC_EK1914_SLOUT_COUNT; i++)
    EC_WRITE_BIT(&pd[hal_data->slout[i].os], hal_data->slout[i].bp, LCEC_PIN_BIT_GET(hal_data->slout[i].pin));
}

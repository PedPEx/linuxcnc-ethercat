// SPDX-License-Identifier: GPL-2.0-or-later
//
// lcec_el6224.c
//
// lcec driver for Beckhoff EL6224 IO-Link Master (4-port)
// Supports up to four Siemens IO-Link devices (one per port).
//
// EtherCAT identity (verified via `ethercat slaves -v`):
//   Vendor ID   : 0x00000002  (Beckhoff)
//   Product code: 0x18503052  (EL6224)
//
// ── HAL pin structure ────────────────────────────────────────────────────
//
// Root level (always present):
//   <slave>.portN-active      Static bit: was portN configured active? (HAL_OUT)
//   <slave>.all-ready         1 if ALL active ports report ready       (HAL_OUT)
//   <slave>.any-error         1 if ANY active port reports an error    (HAL_OUT)
//
// Port subfolder (present for active ports only):
//   <slave>.portN.ready       IO-Link device operational
//   <slave>.portN.ready-not
//   <slave>.portN.error       IO-Link group error
//   <slave>.portN.error-not
//   <slave>.portN.chM-in      Digital input  (+ chM-in-not)
//   <slave>.portN.chM-out     Digital output
//
// ── modParams ────────────────────────────────────────────────────────────
//   portNactive   "true"/"false"  Siemens device on port N? (default: false)
//   portNOutMask  hex string      Output channel bitmask    (default: "0x00")
//                 e.g. "0x02"    bit 1 set -> ch1 is output
//
// ── CoE objects (offset +0x10 per port) ──────────────────────────────────
//   Config:  0x8000 + (N-1)*0x10
//   TxPDO:   0x1A00 + (N-1)       (inputs)
//   RxPDO:   0x1600 + (N-1)       (outputs)
//   InObj:   0x6000 + (N-1)*0x10  (sub 0x01=ready, 0x02=grperr, 0x09-0x10=in0-7)
//   OutObj:  0x7000 + (N-1)*0x10  (sub 0x01-0x08=out0-7)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

// ── Identity ─────────────────────────────────────────────────────────────
#define LCEC_EL6224_VID      LCEC_BECKHOFF_VID
#define LCEC_EL6224_PID      0x18503052

#define EL6224_NUM_PORTS     4
#define EL6224_NUM_CHANNELS  8

// CoE base addresses
#define EL6224_CFG_BASE      0x8000
#define EL6224_IN_BASE       0x6000
#define EL6224_OUT_BASE      0x7000
#define EL6224_TXPDO_BASE    0x1A00
#define EL6224_RXPDO_BASE    0x1600

// TxPDO subindices within InObj
#define EL6224_SUB_READY     0x01
#define EL6224_SUB_GRPERR    0x02
#define EL6224_SUB_IN_BASE   0x09   // Input 0 at 0x09, ..., Input 7 at 0x10

// RxPDO subindices within OutObj
#define EL6224_SUB_OUT_BASE  0x01   // Output 0 at 0x01, ..., Output 7 at 0x08

// ── modParam IDs ──────────────────────────────────────────────────────────
#define MODPARAM_PORT_ACTIVE(n)   (n)
#define MODPARAM_PORT_OUTMASK(n)  (4 + (n))

static const lcec_modparam_desc_t modparams[] = {
    {"port1active",  MODPARAM_PORT_ACTIVE(0), MODPARAM_TYPE_STRING, "false",
     "Enable Siemens IO-Link device on port 1 (true/false)"},
    {"port2active",  MODPARAM_PORT_ACTIVE(1), MODPARAM_TYPE_STRING, "false",
     "Enable Siemens IO-Link device on port 2 (true/false)"},
    {"port3active",  MODPARAM_PORT_ACTIVE(2), MODPARAM_TYPE_STRING, "false",
     "Enable Siemens IO-Link device on port 3 (true/false)"},
    {"port4active",  MODPARAM_PORT_ACTIVE(3), MODPARAM_TYPE_STRING, "false",
     "Enable Siemens IO-Link device on port 4 (true/false)"},
    {"port1OutMask", MODPARAM_PORT_OUTMASK(0), MODPARAM_TYPE_STRING, "0x00",
     "Port 1 output channel bitmask (e.g. 0x02 = ch1 is output)"},
    {"port2OutMask", MODPARAM_PORT_OUTMASK(1), MODPARAM_TYPE_STRING, "0x00",
     "Port 2 output channel bitmask"},
    {"port3OutMask", MODPARAM_PORT_OUTMASK(2), MODPARAM_TYPE_STRING, "0x00",
     "Port 3 output channel bitmask"},
    {"port4OutMask", MODPARAM_PORT_OUTMASK(3), MODPARAM_TYPE_STRING, "0x00",
     "Port 4 output channel bitmask"},
    {NULL},
};

// ── Per-port data ─────────────────────────────────────────────────────────
typedef struct {
    int active;
    int out_mask;
    int ch_is_out[EL6224_NUM_CHANNELS];

    // portN.ready / portN.error  (in port subfolder)
    lcec_class_din_channel_t *ready;
    lcec_class_din_channel_t *error;

    // portN.chM-in / portN.chM-out  (in port subfolder)
    lcec_class_din_channel_t  *din [EL6224_NUM_CHANNELS];
    lcec_class_dout_channel_t *dout[EL6224_NUM_CHANNELS];
} lcec_el6224_port_t;

// ── Root-level driver data ────────────────────────────────────────────────
typedef struct {
    lcec_el6224_port_t ports[EL6224_NUM_PORTS];

    // Root-level static active bits (portN-active)
    hal_bit_t *port_active[EL6224_NUM_PORTS];

    // Root-level aggregate bits
    hal_bit_t *all_ready;   // AND of ready across all active ports
    hal_bit_t *any_error;   // OR  of error across all active ports
} lcec_el6224_data_t;

// ── Forward declarations ──────────────────────────────────────────────────
static int  lcec_el6224_init (int comp_id, lcec_slave_t *slave);
static void lcec_el6224_read (lcec_slave_t *slave, long period);
static void lcec_el6224_write(lcec_slave_t *slave, long period);

// ── Type registration ─────────────────────────────────────────────────────
static lcec_typelist_t types[] = {
    {"EL6224", LCEC_EL6224_VID, LCEC_EL6224_PID, 0,
     NULL, lcec_el6224_init, modparams, 0},
    {NULL},
};
ADD_TYPES(types);

// ── Configure one port via startup SDOs (runs for all 4 ports) ───────────
//
// ecrt_slave_config_sdo*() queues SDOs in the PREOP->SAFEOP transition,
// matching TwinCAT startup behaviour. All four 0x8000/8010/8020/8030
// objects must be written identically per TwinCAT export.
static void el6224_config_port_sdo(lcec_slave_t *slave, int port_idx) {
    uint16_t base = EL6224_CFG_BASE + port_idx * 0x10;
    uint32_t val32;

    val32 = htole32(0x00000000);
    ecrt_slave_config_sdo(slave->config, base, 0x04,
                          (uint8_t *)&val32, sizeof(val32));  // Device ID

    val32 = htole32(0x0000002A);
    ecrt_slave_config_sdo(slave->config, base, 0x05,
                          (uint8_t *)&val32, sizeof(val32));  // Vendor ID (Siemens)

    ecrt_slave_config_sdo8 (slave->config, base, 0x20, 0x11);    // IO-Link v1.1
    ecrt_slave_config_sdo8 (slave->config, base, 0x22, 0x49);    // Cycle 7.3 ms
    ecrt_slave_config_sdo8 (slave->config, base, 0x24, 0x50);    // PD in  80 bit
    ecrt_slave_config_sdo8 (slave->config, base, 0x25, 0x50);    // PD out 80 bit
    ecrt_slave_config_sdo16(slave->config, base, 0x28, 0x0023);  // Master Control
}

// ── Init ──────────────────────────────────────────────────────────────────
static int lcec_el6224_init(int comp_id, lcec_slave_t *slave) {
    lcec_master_t *master = slave->master;
    lcec_el6224_data_t *hal_data;
    LCEC_CONF_MODPARAM_VAL_T *pval;
    int port_idx, ch, err;
    int any_active = 0;
    char pin_name[128];

    hal_data = LCEC_HAL_ALLOCATE(lcec_el6224_data_t);
    slave->hal_data   = hal_data;
    slave->proc_read  = lcec_el6224_read;
    slave->proc_write = lcec_el6224_write;

    // ── Parse modParams ─────────────────────────────────────────────
    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        lcec_el6224_port_t *port = &hal_data->ports[port_idx];

        port->active = 0;
        pval = lcec_modparam_get(slave, MODPARAM_PORT_ACTIVE(port_idx));
        if (pval != NULL &&
            (strcasecmp(pval->str, "true") == 0 ||
             strcmp(pval->str, "1") == 0)) {
            port->active = 1;
            any_active   = 1;
        }

        port->out_mask = 0x00;
        pval = lcec_modparam_get(slave, MODPARAM_PORT_OUTMASK(port_idx));
        if (pval != NULL)
            port->out_mask = (int)strtoul(pval->str, NULL, 0);

        for (ch = 0; ch < EL6224_NUM_CHANNELS; ch++)
            port->ch_is_out[ch] = (port->out_mask >> ch) & 1;

        // SDO startup for ALL four ports (required by EL6224 firmware)
        el6224_config_port_sdo(slave, port_idx);
    }

    // ── Root-level HAL pins ─────────────────────────────────────────

    // portN-active: static read bits reflecting modParam, always present
    // Use LCEC_MODULE_NAME prefix so pins appear under lcec.m0.<slave>.*
    // matching the naming of all other lcec pins.
    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        snprintf(pin_name, sizeof(pin_name), "%s.%s.%s.port%d-active",
                 LCEC_MODULE_NAME, master->name, slave->name, port_idx + 1);
        err = hal_pin_bit_new(pin_name, HAL_OUT,
                              &hal_data->port_active[port_idx],
                              comp_id);
        if (err) return err;
        // Set static value immediately - will not change at runtime
        *hal_data->port_active[port_idx] =
            hal_data->ports[port_idx].active ? 1 : 0;
    }

    // all-ready: AND of ready across all active ports
    snprintf(pin_name, sizeof(pin_name), "%s.%s.%s.all-ready",
             LCEC_MODULE_NAME, master->name, slave->name);
    err = hal_pin_bit_new(pin_name, HAL_OUT,
                          &hal_data->all_ready, comp_id);
    if (err) return err;

    // any-error: OR of error across all active ports
    snprintf(pin_name, sizeof(pin_name), "%s.%s.%s.any-error",
             LCEC_MODULE_NAME, master->name, slave->name);
    err = hal_pin_bit_new(pin_name, HAL_OUT,
                          &hal_data->any_error, comp_id);
    if (err) return err;

    // ── PDO sync-manager configuration ─────────────────────────────
    lcec_syncs_t *syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
    lcec_syncs_init(slave, syncs);

    lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DISABLE);  // SM0 mailbox out
    lcec_syncs_add_sync(syncs, EC_DIR_INPUT,  EC_WD_DISABLE);  // SM1 mailbox in

    // SM2: RxPDOs (outputs) for all active ports
    lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE);
    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        if (!hal_data->ports[port_idx].active) continue;
        uint16_t out_obj = EL6224_OUT_BASE + port_idx * 0x10;
        lcec_syncs_add_pdo_info(syncs, EL6224_RXPDO_BASE + port_idx);
        for (ch = 0; ch < EL6224_NUM_CHANNELS; ch++)
            lcec_syncs_add_pdo_entry(syncs, out_obj,
                                     EL6224_SUB_OUT_BASE + ch, 1);
        lcec_syncs_add_pdo_entry(syncs, 0x0000, 0x00, 8);  // padding
    }

    // SM3: TxPDOs (inputs) for all active ports
    lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);
    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        if (!hal_data->ports[port_idx].active) continue;
        uint16_t in_obj = EL6224_IN_BASE + port_idx * 0x10;
        lcec_syncs_add_pdo_info(syncs, EL6224_TXPDO_BASE + port_idx);
        lcec_syncs_add_pdo_entry(syncs, in_obj, EL6224_SUB_READY,  1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, EL6224_SUB_GRPERR, 1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, 0x03, 1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, 0x04, 1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, 0x05, 1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, 0x06, 1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, 0x07, 1);
        lcec_syncs_add_pdo_entry(syncs, in_obj, 0x08, 1);
        for (ch = 0; ch < EL6224_NUM_CHANNELS; ch++)
            lcec_syncs_add_pdo_entry(syncs, in_obj,
                                     EL6224_SUB_IN_BASE + ch, 1);
    }

    slave->sync_info = &syncs->syncs[0];

    // ── Port subfolder HAL pins for active ports ────────────────────
    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        lcec_el6224_port_t *port = &hal_data->ports[port_idx];
        if (!port->active) continue;

        uint16_t in_obj  = EL6224_IN_BASE  + port_idx * 0x10;
        uint16_t out_obj = EL6224_OUT_BASE + port_idx * 0x10;
        int port_num = port_idx + 1;

        // portN.ready  (in port subfolder)
        snprintf(pin_name, sizeof(pin_name), "port%d.ready", port_num);
        port->ready = lcec_din_register_channel_named(
            slave, in_obj, EL6224_SUB_READY, pin_name);
        if (port->ready == NULL) return -EIO;

        // portN.error  (in port subfolder)
        snprintf(pin_name, sizeof(pin_name), "port%d.error", port_num);
        port->error = lcec_din_register_channel_named(
            slave, in_obj, EL6224_SUB_GRPERR, pin_name);
        if (port->error == NULL) return -EIO;

        // portN.chM-in / portN.chM-out  (in port subfolder)
        for (ch = 0; ch < EL6224_NUM_CHANNELS; ch++) {
            if (port->ch_is_out[ch]) {
                snprintf(pin_name, sizeof(pin_name),
                         "port%d.ch%d-out", port_num, ch);
                port->dout[ch] = lcec_dout_register_channel_named(
                    slave, out_obj, EL6224_SUB_OUT_BASE + ch, pin_name);
                if (port->dout[ch] == NULL) return -EIO;
            } else {
                snprintf(pin_name, sizeof(pin_name),
                         "port%d.ch%d-in", port_num, ch);
                port->din[ch] = lcec_din_register_channel_named(
                    slave, in_obj, EL6224_SUB_IN_BASE + ch, pin_name);
                if (port->din[ch] == NULL) return -EIO;
            }
        }
    }

    if (!any_active)
        rtapi_print_msg(RTAPI_MSG_WARN,
            LCEC_MSG_PFX "slave %s.%s: no ports active. "
            "Set portNactive=true in modParams.\n",
            master->name, slave->name);

    return 0;
}

// ── Read (TxPDO -> HAL) ───────────────────────────────────────────────────
static void lcec_el6224_read(lcec_slave_t *slave, long period) {
    lcec_el6224_data_t *hal_data = (lcec_el6224_data_t *)slave->hal_data;
    int port_idx, ch;
    int all_ready = 1;   // assume all ready until proven otherwise
    int any_error = 0;   // assume no error until proven otherwise
    int any_active = 0;

    if (!slave->state.operational) {
        *hal_data->all_ready = 0;
        *hal_data->any_error = 1;
        return;
    }

    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        lcec_el6224_port_t *port = &hal_data->ports[port_idx];
        if (!port->active) continue;
        any_active = 1;

        // Read status bits (portN.ready / portN.error)
        lcec_din_read(slave, port->ready);
        lcec_din_read(slave, port->error);

        // Contribute to aggregate bits
        // port->ready->in is the HAL pin pointer
        if (!(*port->ready->in)) all_ready = 0;
        if (*port->error->in)    any_error = 1;

        // Read input channels
        for (ch = 0; ch < EL6224_NUM_CHANNELS; ch++) {
            if (!port->ch_is_out[ch] && port->din[ch] != NULL)
                lcec_din_read(slave, port->din[ch]);
        }
    }

    // Update aggregate root pins
    *hal_data->all_ready = any_active ? all_ready : 0;
    *hal_data->any_error = any_error;
}

// ── Write (HAL -> RxPDO) ──────────────────────────────────────────────────
static void lcec_el6224_write(lcec_slave_t *slave, long period) {
    lcec_el6224_data_t *hal_data = (lcec_el6224_data_t *)slave->hal_data;
    int port_idx, ch;

    if (!slave->state.operational) return;

    for (port_idx = 0; port_idx < EL6224_NUM_PORTS; port_idx++) {
        lcec_el6224_port_t *port = &hal_data->ports[port_idx];
        if (!port->active) continue;

        for (ch = 0; ch < EL6224_NUM_CHANNELS; ch++) {
            if (port->ch_is_out[ch] && port->dout[ch] != NULL)
                lcec_dout_write(slave, port->dout[ch]);
        }
    }
}

/* lcec_el7332.c
 *
 * LinuxCNC EtherCAT (lcec) driver for Beckhoff EL7332
 * 2-channel DC motor output terminal (24V, 1.5A).
 *
 * Hardware verified: Revision 0x00180000 (Rev 7), both slaves confirmed
 * identical via  ethercat slaves -v -p 6/7.
 *
 * PDO layout (Enable PDO Assign: no, Enable PDO Configuration: no)
 * The slave's PDO mapping is fixed in hardware and cannot be changed.
 * slave->sync_info is set to NULL to suppress the IgH master's
 * ecrt_slave_config_pdos() call, which would otherwise produce harmless
 * but noisy dmesg WARNINGs on every PREOP→SAFEOP transition.
 * PDO entry offsets are registered directly via lcec_pdo_init() / slave->regs.
 * Layout confirmed via  ethercat pdos -p 6  on hardware Rev 14 / SW 07:
 *
 *  SM2 RxPDO – master → slave
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │ 0x1600  CNT RxPDO Control compact Ch.1  (counter, not used)        │
 *  │ 0x1602  CNT RxPDO Control compact Ch.2  (counter, not used)        │
 *  │ 0x1604  DCM RxPDO Control Ch.1                                     │
 *  │   7020:01  1 bit   Enable                                           │
 *  │   7020:02  1 bit   Reset                                            │
 *  │   7020:03  1 bit   Reduce torque                                    │
 *  │   gap      5 bit                                                    │
 *  │   gap      8 bit                                                    │
 *  │ 0x1605  DCM RxPDO Velocity Ch.1                                    │
 *  │   7020:21 16 bit   Velocity (INT16)                                 │
 *  │ 0x1606  DCM RxPDO Control Ch.2  (same layout, 7030:xx)            │
 *  │ 0x1607  DCM RxPDO Velocity Ch.2 (7030:21)                         │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  SM3 TxPDO – slave → master
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │ 0x1A00  CNT TxPDO Status compact Ch.1  (counter, not used)        │
 *  │ 0x1A02  CNT TxPDO Status compact Ch.2  (counter, not used)        │
 *  │ 0x1A04  DCM TxPDO Status Ch.1                                      │
 *  │   6020:01  Ready to enable                                          │
 *  │   6020:02  Ready                                                    │
 *  │   6020:03  Warning                                                  │
 *  │   6020:04  Error                                                    │
 *  │   6020:05  Moving positive                                          │
 *  │   6020:06  Moving negative                                          │
 *  │   6020:07  Torque reduced                                           │
 *  │   gap       1 bit                                                   │
 *  │   gap       3 bit                                                   │
 *  │   6020:0C  Digital input 1                                          │
 *  │   6020:0D  Digital input 2                                          │
 *  │   6020:0E  Sync error                                               │
 *  │   gap       1 bit                                                   │
 *  │   gap       1 bit  (TxPDO Toggle – not exposed, internal protocol) │
 *  │   6020:11  Info data 1  (UINT16, source selected by chN_info_data_1)│
 *  │   6020:12  Info data 2  (UINT16, source selected by chN_info_data_2)│
 *  │ 0x1A06  DCM TxPDO Status Ch.2  (same layout, 6030:xx)            │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 * Info data source enum (modParam chN_info_data_1 / chN_info_data_2):
 *   0  Status word          → HAL pin chN.info-status-word   (s32)
 *   1  Motor coil voltage   → HAL pin chN.info-coil-voltage  (s32, mV)
 *   2  Motor coil current   → HAL pin chN.info-coil-current  (s32, mA)
 *   3  Current limit        → HAL pin chN.info-current-limit (s32, mA)
 *   4  Control error        → HAL pin chN.info-control-error (s32)
 *   5  Duty cycle           → HAL pin chN.info-duty-cycle    (s32, %)
 *   6  Overload time        → HAL pin chN.info-overload-time (s32, ms)
 *   7  Internal temperature → HAL pin chN.info-internal-temp (s32, °C)
 *   8  Control voltage      → HAL pin chN.info-ctrl-voltage  (s32, mV)
 *   9  Motor supply voltage → HAL pin chN.info-supply-voltage(s32, mV)
 *
 * KNOWN HARDWARE BUG – EL7332 Rev 14 / SW 07:
 *   9020:02 (Motor coil voltage) saturates at approximately -344 mV when the
 *   motor runs in the negative direction. This has been reproduced identically
 *   in TwinCAT3 and is a firmware defect in this hardware revision; it is not
 *   a driver issue. Positive-direction voltage readings are correct.
 *   Workaround: chN.info.coil-voltage-calc is computed each cycle as:
 *     (duty_cycle_pct / 100.0) * supply_voltage_mV
 *   using 9020:06 (duty cycle, INT8, %) and F900:05 (supply voltage, mV).
 *   Both duty cycle and supply voltage are read correctly in both directions.
 *   Note: 9020:11 / 9020:12 (info data 1/2 mirror) do not exist in this
 *   firmware revision – the subindices are absent from the object dictionary.
 *   The chN.info.data-1 / chN.info.data-2 pins have been removed accordingly.
 *
 *   9020:02 / 9030:02  Motor coil voltage  → chN.sdo-coil-voltage  (s32, mV)
 *   9020:03 / 9030:03  Motor coil current  → chN.sdo-coil-current  (s32, mA)
 *   9020:05 / 9030:05  Control error       → chN.sdo-control-error (s32)
 *   9020:06 / 9030:06  Duty cycle          → chN.sdo-duty-cycle    (s32, %)
 *   9020:09 / 9030:09  Overload time       → chN.sdo-overload-time (s32, ms)
 *
 * Startup SDOs (written before SAFEOP, conditional on deviation from default,
 * non-fatal on failure – device retains its current value):
 *
 *  Per channel (N=1: 0x8020/0x8022, N=2: 0x8030/0x8032):
 *   80N0:01  Maximal current          (modParam chN_max_current,        def 2400)
 *   80N0:02  Nominal current          (modParam chN_nom_current,        def 1500)
 *   80N0:03  Nominal voltage          (modParam chN_nom_voltage,        def 24000)
 *   80N0:04  Motor coil resistance    (modParam chN_coil_resistance,    def 100)
 *   80N0:05  Reduced current (pos)    (modParam chN_reduced_current_pos,def 500)
 *   80N0:06  Reduced current (neg)    (modParam chN_reduced_current_neg,def 500)
 *   80N0:0C  Time for switch-off      (modParam chN_overload_time,      def 200)
 *   80N0:0D  Time for curr. lowering  (modParam chN_current_lower_time, def 2000)
 *   80N0:0E  Torque reduction thresh+ (modParam chN_torque_reduction_pos,def 0)
 *   80N0:0F  Torque reduction thresh- (modParam chN_torque_reduction_neg,def 0)
 *   80N2:01  Operation mode           (modParam chN_mode: "velocity"=1,"chopper"=15)
 *   80N2:09  Invert motor polarity    (modParam chN_invert_polarity,    def false)
 *   80N2:0A  Torque error enable      (modParam chN_torque_error_enable,def false)
 *   80N2:0B  Torque auto reduce       (modParam chN_torque_auto_reduce, def false)
 *   80N2:11  Select info data 1       (modParam chN_info_data_1,        def 1)
 *   80N2:19  Select info data 2       (modParam chN_info_data_2,        def 2)
 *   80N2:30  Invert digital input 1   (modParam chN_invert_din1,        def false)
 *   80N2:31  Invert digital input 2   (modParam chN_invert_din2,        def false)
 *   80N2:32  Function for input 1     (modParam chN_function_din1,      def 0)
 *   80N2:36  Function for input 2     (modParam chN_function_din2,      def 0)
 *
 *  Device-wide (0xF80F Vendor data):
 *   F80F:01  PWM frequency            (modParam pwm_frequency,          def 30000)
 *   F80F:04  Warning temperature      (modParam warning_temp,           def 80)
 *   F80F:05  Switch-off temperature   (modParam switchoff_temp,         def 100)
 *
 * Cyclic SDO reads (~1 Hz throttle, shared timer):
 *   0xF900:02  Internal temperature   (INT8,   °C)
 *   0xF900:05  Motor supply voltage   (UINT16, mV)
 *   0xA020:xx  DCM Diag data Ch.1    (individual bool fields)
 *   0xA030:xx  DCM Diag data Ch.2
 *
 * HAL pins (prefix: lcec.<master>.<slave>):
 *
 *  Per channel (chN = ch1 / ch2):
 *   HAL_IN  (LinuxCNC → device):
 *     chN.enable          bit   External enable gate (e.g. machine-on)
 *     chN.velocity-cmd    float Velocity setpoint  -1.0 … +1.0
 *     chN.reduce-torque   bit   Activate reduced current mode
 *     chN.reset           bit   Reset errors (rising edge)
 *   HAL_OUT (device → LinuxCNC):
 *     chN.ready-to-enable bit   6020:01 / 6030:01
 *     chN.ready           bit   6020:02 / 6030:02
 *     chN.warning         bit   6020:03 / 6030:03
 *     chN.error           bit   6020:04 / 6030:04
 *     chN.moving-pos      bit   6020:05 / 6030:05
 *     chN.moving-neg      bit   6020:06 / 6030:06
 *     chN.torque-reduced  bit   6020:07 / 6030:07
 *     chN.din1            bit   6020:0C / 6030:0C
 *     chN.din2            bit   6020:0D / 6030:0D
 *     chN.sync-error      bit   6020:0E / 6030:0E
 *     chN.tx-toggle       bit   6020:10 / 6030:10
 *     chN.diag-saturated      bit  A020:01 / A030:01  (cyclic SDO)
 *     chN.diag-over-temp      bit  A020:02 / A030:02
 *     chN.diag-torque-overload bit A020:03 / A030:03
 *     chN.diag-under-voltage  bit  A020:04 / A030:04
 *     chN.diag-over-voltage   bit  A020:05 / A030:05
 *     chN.diag-short-circuit  bit  A020:06 / A030:06
 *     chN.diag-no-ctrl-power  bit  A020:08 / A030:08
 *     chN.diag-misc-error     bit  A020:09 / A030:09
 *
 *  Device-wide:
 *     internal-temp   s32  F900:02 (°C, cyclic SDO)
 *     supply-voltage  u32  F900:05 (mV, cyclic SDO)
 */

#include "../lcec.h"
#include <stdio.h>

/* ======================================================================
 * Vendor / Product identifiers
 * Confirmed via:  ethercat slaves -v -p 6
 *   Vendor Id:    0x00000002
 *   Product code: 0x1ca43052
 *   Revision:     0x00180000
 * ====================================================================== */
#define LCEC_EL7332_VID  LCEC_BECKHOFF_VID   /* 0x00000002 */
#define LCEC_EL7332_PID  0x1ca43052

/* ======================================================================
 * Factory defaults (confirmed via TwinCAT CoE-Online and ethercat sdos)
 * ====================================================================== */

/* 0x8020 / 0x8030  Motor Settings */
#define DEF_MAX_CURRENT          2400   /* mA  */
#define DEF_NOM_CURRENT          1500   /* mA  */
#define DEF_NOM_VOLTAGE          24000  /* mV  */
#define DEF_COIL_RESISTANCE      100    /* mΩ  */
#define DEF_REDUCED_CURRENT_POS  500    /* mA  */
#define DEF_REDUCED_CURRENT_NEG  500    /* mA  */
#define DEF_OVERLOAD_TIME        200    /* ms  */
#define DEF_CURRENT_LOWER_TIME   2000   /* ms  */
#define DEF_TORQUE_REDUCTION_POS 0
#define DEF_TORQUE_REDUCTION_NEG 0

/* 0x8022 / 0x8032  DCM Features */
#define DEF_OP_MODE_VELOCITY     1      /* Velocity direct */
#define DEF_OP_MODE_CHOPPER      15     /* Chopper resistor */
#define DEF_INVERT_POLARITY      0
#define DEF_TORQUE_ERROR_ENABLE  0
#define DEF_TORQUE_AUTO_REDUCE   0
#define DEF_INFO_DATA_1          1      /* Motor coil voltage */
#define DEF_INFO_DATA_2          2      /* Motor coil current */
#define DEF_INVERT_DIN1          0
#define DEF_INVERT_DIN2          0
#define DEF_FUNCTION_DIN1        0      /* Normal input */
#define DEF_FUNCTION_DIN2        0

/* 0xF80F  Vendor data */
#define DEF_PWM_FREQUENCY        30000  /* Hz */
#define DEF_WARNING_TEMP         80     /* °C */
#define DEF_SWITCHOFF_TEMP       100    /* °C */

/* Cyclic SDO read throttle: one request per SDO_READ_INTERVAL servo cycles.
 * At 1 kHz servo thread this gives ~100 ms between consecutive requests.
 * The EL7332 mailbox round-trip is typically <10 ms, so this is safe.
 * With 23 slots (2 device + 16 diag + 5 info) all data refreshes in ~2.3 s. */
#define SDO_READ_INTERVAL        100

/* SDO data sizes */
#define SDO_SIZE_TEMP            1      /* F900:02: INT8  */
#define SDO_SIZE_VOLTAGE         2      /* F900:05: UINT16 */

/* ======================================================================
 * modParam IDs
 * ====================================================================== */
enum {
  /* Channel 1 */
  MODPARAM_CH1_ENABLE = 0,
  MODPARAM_CH1_MODE,
  MODPARAM_CH1_MAX_CURRENT,
  MODPARAM_CH1_NOM_CURRENT,
  MODPARAM_CH1_NOM_VOLTAGE,
  MODPARAM_CH1_COIL_RESISTANCE,
  MODPARAM_CH1_REDUCED_CURRENT_POS,
  MODPARAM_CH1_REDUCED_CURRENT_NEG,
  MODPARAM_CH1_OVERLOAD_TIME,
  MODPARAM_CH1_CURRENT_LOWER_TIME,
  MODPARAM_CH1_TORQUE_REDUCTION_POS,
  MODPARAM_CH1_TORQUE_REDUCTION_NEG,
  MODPARAM_CH1_INVERT_POLARITY,
  MODPARAM_CH1_TORQUE_ERROR_ENABLE,
  MODPARAM_CH1_TORQUE_AUTO_REDUCE,
  MODPARAM_CH1_INFO_DATA_1,
  MODPARAM_CH1_INFO_DATA_2,
  MODPARAM_CH1_INVERT_DIN1,
  MODPARAM_CH1_INVERT_DIN2,
  MODPARAM_CH1_FUNCTION_DIN1,
  MODPARAM_CH1_FUNCTION_DIN2,
  MODPARAM_CH1_ENABLE_INFO_SDO,
  /* Channel 2 */
  MODPARAM_CH2_ENABLE,
  MODPARAM_CH2_MODE,
  MODPARAM_CH2_MAX_CURRENT,
  MODPARAM_CH2_NOM_CURRENT,
  MODPARAM_CH2_NOM_VOLTAGE,
  MODPARAM_CH2_COIL_RESISTANCE,
  MODPARAM_CH2_REDUCED_CURRENT_POS,
  MODPARAM_CH2_REDUCED_CURRENT_NEG,
  MODPARAM_CH2_OVERLOAD_TIME,
  MODPARAM_CH2_CURRENT_LOWER_TIME,
  MODPARAM_CH2_TORQUE_REDUCTION_POS,
  MODPARAM_CH2_TORQUE_REDUCTION_NEG,
  MODPARAM_CH2_INVERT_POLARITY,
  MODPARAM_CH2_TORQUE_ERROR_ENABLE,
  MODPARAM_CH2_TORQUE_AUTO_REDUCE,
  MODPARAM_CH2_INFO_DATA_1,
  MODPARAM_CH2_INFO_DATA_2,
  MODPARAM_CH2_INVERT_DIN1,
  MODPARAM_CH2_INVERT_DIN2,
  MODPARAM_CH2_FUNCTION_DIN1,
  MODPARAM_CH2_FUNCTION_DIN2,
  MODPARAM_CH2_ENABLE_INFO_SDO,
  /* Device-wide */
  MODPARAM_PWM_FREQUENCY,
  MODPARAM_WARNING_TEMP,
  MODPARAM_SWITCHOFF_TEMP,
};

/* ======================================================================
 * modParam descriptor table
 * ====================================================================== */
static const lcec_modparam_desc_t lcec_el7332_modparams[] = {
  /* --- Channel 1 --- */
  {"ch1_enable",               MODPARAM_CH1_ENABLE,               MODPARAM_TYPE_BIT,
    "true",  "Enable channel 1 (combined with HAL enable pin)"},
  {"ch1_mode",                 MODPARAM_CH1_MODE,                 MODPARAM_TYPE_STRING,
    "velocity", "Operation mode: velocity or chopper (8022:01)"},
  {"ch1_max_current",          MODPARAM_CH1_MAX_CURRENT,          MODPARAM_TYPE_U32,
    "2400",  "Maximum current in mA (8020:01)"},
  {"ch1_nom_current",          MODPARAM_CH1_NOM_CURRENT,          MODPARAM_TYPE_U32,
    "1500",  "Nominal current in mA (8020:02)"},
  {"ch1_nom_voltage",          MODPARAM_CH1_NOM_VOLTAGE,          MODPARAM_TYPE_U32,
    "24000", "Nominal voltage in mV (8020:03)"},
  {"ch1_coil_resistance",      MODPARAM_CH1_COIL_RESISTANCE,      MODPARAM_TYPE_U32,
    "100",   "Motor coil resistance in mΩ (8020:04)"},
  {"ch1_reduced_current_pos",  MODPARAM_CH1_REDUCED_CURRENT_POS,  MODPARAM_TYPE_U32,
    "500",   "Reduced current positive in mA (8020:05)"},
  {"ch1_reduced_current_neg",  MODPARAM_CH1_REDUCED_CURRENT_NEG,  MODPARAM_TYPE_U32,
    "500",   "Reduced current negative in mA (8020:06)"},
  {"ch1_overload_time",        MODPARAM_CH1_OVERLOAD_TIME,        MODPARAM_TYPE_U32,
    "200",   "Time for switch-off at overload in ms (8020:0C)"},
  {"ch1_current_lower_time",   MODPARAM_CH1_CURRENT_LOWER_TIME,   MODPARAM_TYPE_U32,
    "2000",  "Time for current lowering at overload in ms (8020:0D)"},
  {"ch1_torque_reduction_pos", MODPARAM_CH1_TORQUE_REDUCTION_POS, MODPARAM_TYPE_U32,
    "0",     "Torque auto-reduction threshold positive (8020:0E)"},
  {"ch1_torque_reduction_neg", MODPARAM_CH1_TORQUE_REDUCTION_NEG, MODPARAM_TYPE_U32,
    "0",     "Torque auto-reduction threshold negative (8020:0F)"},
  {"ch1_invert_polarity",      MODPARAM_CH1_INVERT_POLARITY,      MODPARAM_TYPE_BIT,
    "false", "Invert motor polarity (8022:09)"},
  {"ch1_torque_error_enable",  MODPARAM_CH1_TORQUE_ERROR_ENABLE,  MODPARAM_TYPE_BIT,
    "false", "Enable torque error (8022:0A)"},
  {"ch1_torque_auto_reduce",   MODPARAM_CH1_TORQUE_AUTO_REDUCE,   MODPARAM_TYPE_BIT,
    "false", "Torque auto reduce (8022:0B)"},
  {"ch1_info_data_1",          MODPARAM_CH1_INFO_DATA_1,          MODPARAM_TYPE_U32,
    "1",     "Select info data 1 source (8022:11), 1=coil voltage"},
  {"ch1_info_data_2",          MODPARAM_CH1_INFO_DATA_2,          MODPARAM_TYPE_U32,
    "2",     "Select info data 2 source (8022:19), 2=coil current"},
  {"ch1_invert_din1",          MODPARAM_CH1_INVERT_DIN1,          MODPARAM_TYPE_BIT,
    "false", "Invert digital input 1 (8022:30)"},
  {"ch1_invert_din2",          MODPARAM_CH1_INVERT_DIN2,          MODPARAM_TYPE_BIT,
    "false", "Invert digital input 2 (8022:31)"},
  {"ch1_function_din1",        MODPARAM_CH1_FUNCTION_DIN1,        MODPARAM_TYPE_U32,
    "0",     "Function for digital input 1 (8022:32), 0=normal input"},
  {"ch1_function_din2",        MODPARAM_CH1_FUNCTION_DIN2,        MODPARAM_TYPE_U32,
    "0",     "Function for digital input 2 (8022:36), 0=normal input"},
  {"ch1_enable_info_sdo",      MODPARAM_CH1_ENABLE_INFO_SDO,      MODPARAM_TYPE_BIT,
    "false", "Enable cyclic SDO reads of 9020 info data for channel 1"},
  /* --- Channel 2 --- */
  {"ch2_enable",               MODPARAM_CH2_ENABLE,               MODPARAM_TYPE_BIT,
    "true",  "Enable channel 2 (combined with HAL enable pin)"},
  {"ch2_mode",                 MODPARAM_CH2_MODE,                 MODPARAM_TYPE_STRING,
    "velocity", "Operation mode: velocity or chopper (8032:01)"},
  {"ch2_max_current",          MODPARAM_CH2_MAX_CURRENT,          MODPARAM_TYPE_U32,
    "2400",  "Maximum current in mA (8030:01)"},
  {"ch2_nom_current",          MODPARAM_CH2_NOM_CURRENT,          MODPARAM_TYPE_U32,
    "1500",  "Nominal current in mA (8030:02)"},
  {"ch2_nom_voltage",          MODPARAM_CH2_NOM_VOLTAGE,          MODPARAM_TYPE_U32,
    "24000", "Nominal voltage in mV (8030:03)"},
  {"ch2_coil_resistance",      MODPARAM_CH2_COIL_RESISTANCE,      MODPARAM_TYPE_U32,
    "100",   "Motor coil resistance in mΩ (8030:04)"},
  {"ch2_reduced_current_pos",  MODPARAM_CH2_REDUCED_CURRENT_POS,  MODPARAM_TYPE_U32,
    "500",   "Reduced current positive in mA (8030:05)"},
  {"ch2_reduced_current_neg",  MODPARAM_CH2_REDUCED_CURRENT_NEG,  MODPARAM_TYPE_U32,
    "500",   "Reduced current negative in mA (8030:06)"},
  {"ch2_overload_time",        MODPARAM_CH2_OVERLOAD_TIME,        MODPARAM_TYPE_U32,
    "200",   "Time for switch-off at overload in ms (8030:0C)"},
  {"ch2_current_lower_time",   MODPARAM_CH2_CURRENT_LOWER_TIME,   MODPARAM_TYPE_U32,
    "2000",  "Time for current lowering at overload in ms (8030:0D)"},
  {"ch2_torque_reduction_pos", MODPARAM_CH2_TORQUE_REDUCTION_POS, MODPARAM_TYPE_U32,
    "0",     "Torque auto-reduction threshold positive (8030:0E)"},
  {"ch2_torque_reduction_neg", MODPARAM_CH2_TORQUE_REDUCTION_NEG, MODPARAM_TYPE_U32,
    "0",     "Torque auto-reduction threshold negative (8030:0F)"},
  {"ch2_invert_polarity",      MODPARAM_CH2_INVERT_POLARITY,      MODPARAM_TYPE_BIT,
    "false", "Invert motor polarity (8032:09)"},
  {"ch2_torque_error_enable",  MODPARAM_CH2_TORQUE_ERROR_ENABLE,  MODPARAM_TYPE_BIT,
    "false", "Enable torque error (8032:0A)"},
  {"ch2_torque_auto_reduce",   MODPARAM_CH2_TORQUE_AUTO_REDUCE,   MODPARAM_TYPE_BIT,
    "false", "Torque auto reduce (8032:0B)"},
  {"ch2_info_data_1",          MODPARAM_CH2_INFO_DATA_1,          MODPARAM_TYPE_U32,
    "1",     "Select info data 1 source (8032:11), 1=coil voltage"},
  {"ch2_info_data_2",          MODPARAM_CH2_INFO_DATA_2,          MODPARAM_TYPE_U32,
    "2",     "Select info data 2 source (8032:19), 2=coil current"},
  {"ch2_invert_din1",          MODPARAM_CH2_INVERT_DIN1,          MODPARAM_TYPE_BIT,
    "false", "Invert digital input 1 (8032:30)"},
  {"ch2_invert_din2",          MODPARAM_CH2_INVERT_DIN2,          MODPARAM_TYPE_BIT,
    "false", "Invert digital input 2 (8032:31)"},
  {"ch2_function_din1",        MODPARAM_CH2_FUNCTION_DIN1,        MODPARAM_TYPE_U32,
    "0",     "Function for digital input 1 (8032:32), 0=normal input"},
  {"ch2_function_din2",        MODPARAM_CH2_FUNCTION_DIN2,        MODPARAM_TYPE_U32,
    "0",     "Function for digital input 2 (8032:36), 0=normal input"},
  {"ch2_enable_info_sdo",      MODPARAM_CH2_ENABLE_INFO_SDO,      MODPARAM_TYPE_BIT,
    "false", "Enable cyclic SDO reads of 9030 info data for channel 2"},
  /* --- Device-wide --- */
  {"pwm_frequency",            MODPARAM_PWM_FREQUENCY,            MODPARAM_TYPE_U32,
    "30000", "PWM frequency in Hz (F80F:01), default 30000"},
  {"warning_temp",             MODPARAM_WARNING_TEMP,             MODPARAM_TYPE_U32,
    "80",    "Warning temperature threshold in °C (F80F:04)"},
  {"switchoff_temp",           MODPARAM_SWITCHOFF_TEMP,           MODPARAM_TYPE_U32,
    "100",   "Switch-off temperature threshold in °C (F80F:05)"},
  {NULL},
};

/* ======================================================================
 * Forward declarations
 * ====================================================================== */
static int  lcec_el7332_init(int comp_id, lcec_slave_t *slave);
static void lcec_el7332_read(lcec_slave_t *slave, long period);
static void lcec_el7332_write(lcec_slave_t *slave, long period);

/* ======================================================================
 * Type registration
 * ====================================================================== */
static lcec_typelist_t types[] = {
  {"EL7332", LCEC_EL7332_VID, LCEC_EL7332_PID, 0, NULL,
    lcec_el7332_init, lcec_el7332_modparams},
  {NULL},
};
ADD_TYPES(types);

/* ======================================================================
 * Per-channel data (PDO offsets + HAL pins)
 * ====================================================================== */
typedef struct {
  /* RxPDO offsets – DCM Outputs 0x7020 (ch1) / 0x7030 (ch2) */
  unsigned int enable_os;         unsigned int enable_bp;
  unsigned int reset_os;          unsigned int reset_bp;
  unsigned int reduce_torque_os;  unsigned int reduce_torque_bp;
  unsigned int velocity_os;       /* INT16 */

  /* TxPDO offsets – DCM Inputs 0x6020 (ch1) / 0x6030 (ch2) */
  unsigned int ready_to_enable_os; unsigned int ready_to_enable_bp;
  unsigned int ready_os;           unsigned int ready_bp;
  unsigned int warning_os;         unsigned int warning_bp;
  unsigned int error_os;           unsigned int error_bp;
  unsigned int moving_pos_os;      unsigned int moving_pos_bp;
  unsigned int moving_neg_os;      unsigned int moving_neg_bp;
  unsigned int torque_reduced_os;  unsigned int torque_reduced_bp;
  unsigned int din1_os;            unsigned int din1_bp;
  unsigned int din2_os;            unsigned int din2_bp;
  unsigned int sync_error_os;      unsigned int sync_error_bp;
  /* tx_toggle (6020:10 / 6030:10) present in hardware fixed PDO layout,
   * registered to keep offset accounting correct, not exposed as HAL pin */
  unsigned int tx_toggle_os;       unsigned int tx_toggle_bp;

  /* Cyclic SDO requests – A020:xx / A030:xx diag fields (each BOOLEAN/uint8) */
  ec_sdo_request_t *sdo_diag_saturated;
  ec_sdo_request_t *sdo_diag_over_temp;
  ec_sdo_request_t *sdo_diag_torque_overload;
  ec_sdo_request_t *sdo_diag_under_voltage;
  ec_sdo_request_t *sdo_diag_over_voltage;
  ec_sdo_request_t *sdo_diag_short_circuit;
  ec_sdo_request_t *sdo_diag_no_ctrl_power;
  ec_sdo_request_t *sdo_diag_misc_error;

  /* Optional cyclic SDO requests – 9020:xx / 9030:xx info data
   * Only allocated when modParam chN_enable_info_sdo = true.
   * HAL pin prefix: chN.info.*
   * NOTE: 9020:11 / 9020:12 are absent in Rev14/SW07 firmware – not requested.
   * coil-voltage-calc is computed from duty_cycle × supply_voltage as a
   * workaround for the hardware bug in 9020:02 (saturates at ~-344 mV).     */
  ec_sdo_request_t *sdo_info_coil_voltage;  /* 90N0:02 UINT16 mV (pos. dir. only, see bug note) */
  ec_sdo_request_t *sdo_info_coil_current;  /* 90N0:03 INT16  mA  */
  ec_sdo_request_t *sdo_info_control_error; /* 90N0:05 INT16      */
  ec_sdo_request_t *sdo_info_duty_cycle;    /* 90N0:06 INT8   %   */
  ec_sdo_request_t *sdo_info_overload_time; /* 90N0:09 UINT16 ms  */

  /* HAL pins – HAL_IN (LinuxCNC → device) */
  hal_bit_t   *enable;            /*!< external enable gate                    */
  hal_float_t *velocity_cmd;      /*!< velocity setpoint -1.0 … +1.0          */
  hal_bit_t   *reduce_torque;     /*!< activate reduced current mode           */
  hal_bit_t   *reset;             /*!< reset errors (rising edge)              */

  /* HAL pins – HAL_OUT (device → LinuxCNC): status */
  hal_bit_t   *ready_to_enable;   /*!< 6020:01 / 6030:01                       */
  hal_bit_t   *ready;             /*!< 6020:02 / 6030:02                       */
  hal_bit_t   *warning;           /*!< 6020:03 / 6030:03                       */
  hal_bit_t   *error;             /*!< 6020:04 / 6030:04                       */
  hal_bit_t   *moving_pos;        /*!< 6020:05 / 6030:05                       */
  hal_bit_t   *moving_neg;        /*!< 6020:06 / 6030:06                       */
  hal_bit_t   *torque_reduced;    /*!< 6020:07 / 6030:07                       */
  hal_bit_t   *din1;              /*!< 6020:0C / 6030:0C                       */
  hal_bit_t   *din2;              /*!< 6020:0D / 6030:0D                       */
  hal_bit_t   *sync_error;        /*!< 6020:0E / 6030:0E                       */

  /* HAL pins – HAL_OUT: optional SDO info data (chN.info.*) */
  hal_s32_t   *info_coil_voltage;      /*!< 90N0:02 mV – positive direction only, see HW bug note */
  hal_s32_t   *info_coil_voltage_calc; /*!< (duty_cycle/100) * supply_voltage – workaround        */
  hal_s32_t   *info_coil_current;      /*!< 90N0:03 mA  */
  hal_s32_t   *info_control_error;     /*!< 90N0:05     */
  hal_s32_t   *info_duty_cycle;        /*!< 90N0:06 %   */
  hal_s32_t   *info_overload_time;     /*!< 90N0:09 ms  */

  /* HAL pins – HAL_OUT: diagnostics from A020 / A030 (cyclic SDO) */
  hal_bit_t   *diag_saturated;
  hal_bit_t   *diag_over_temp;
  hal_bit_t   *diag_torque_overload;
  hal_bit_t   *diag_under_voltage;
  hal_bit_t   *diag_over_voltage;
  hal_bit_t   *diag_short_circuit;
  hal_bit_t   *diag_no_ctrl_power;
  hal_bit_t   *diag_misc_error;

  /* HAL pins – optional SDO info data (only when enable_info_sdo = true) */
  /* declared above in the info.* block */

  /* Runtime state */
  int          modparam_enabled;   /*!< channel enabled by modParam            */
  int          enable_info_sdo;    /*!< enable cyclic SDO info reads           */
  uint8_t      info_data_1_sel;    /*!< modParam value for info data 1         */
  uint8_t      info_data_2_sel;    /*!< modParam value for info data 2         */
  hal_bit_t    prev_reset;         /*!< previous reset pin value (edge detect) */

} lcec_el7332_ch_t;

/* ======================================================================
 * Device-level HAL data
 * ====================================================================== */
typedef struct {
  lcec_el7332_ch_t ch[2];

  /* Cyclic SDO requests – F900 device info */
  ec_sdo_request_t *sdo_internal_temp;
  ec_sdo_request_t *sdo_supply_voltage;

  /* Shared SDO read throttle counter and serialization index.
   * sdo_read_timer: counts servo cycles; fires when >= SDO_READ_INTERVAL.
   * sdo_seq_index:  which request slot is currently active (round-robin).
   * Only one SDO request is started per timer expiry to prevent mailbox
   * collisions (abort 0x08000022 / "wrong SDO" desync on EL7332 Rev14). */
  unsigned int sdo_read_timer;
  unsigned int sdo_seq_index;

  /* Device-wide HAL pins */
  hal_s32_t   *internal_temp;     /*!< F900:02 internal temperature (°C)       */
  hal_u32_t   *supply_voltage;    /*!< F900:05 motor supply voltage (mV)       */

} lcec_el7332_data_t;

/* ======================================================================
 * HAL pin descriptor tables
 * Per-channel pins use channel-index substitution in the name string.
 * The lcec_pin_newf() calls are done manually (not via table) because
 * the channel offset and index must be interpolated at runtime.
 * ====================================================================== */

/* ======================================================================
 * PDO entry arrays
 *
 * CNT PDOs are included in the sync manager mapping (they are part of the
 * fixed default layout and cannot be removed without PDO reconfiguration).
 * Their entries are declared with index 0x0000 / subindex 0x00 so the IgH
 * master maps them as gaps – no lcec_pdo_init() calls are made for them.
 * ====================================================================== */

/* ======================================================================
 * Helper: write UINT16 SDO, warn on failure, non-fatal
 * ====================================================================== */
static void el7332_sdo_write16(lcec_slave_t *slave, uint16_t idx, uint8_t sub,
                                uint16_t val, uint16_t def, const char *name) {
  lcec_master_t *master = slave->master;
  if (val == def) return;
  if (lcec_write_sdo16(slave, idx, sub, val) != 0)
    rtapi_print_msg(RTAPI_MSG_WARN,
      LCEC_MSG_PFX "slave %s.%s: SDO 0x%04X:%02X (%s) write failed"
      " – device keeps current value\n",
      master->name, slave->name, idx, sub, name);
}

static void el7332_sdo_write8(lcec_slave_t *slave, uint16_t idx, uint8_t sub,
                               uint8_t val, uint8_t def, const char *name) {
  lcec_master_t *master = slave->master;
  if (val == def) return;
  if (lcec_write_sdo8(slave, idx, sub, val) != 0)
    rtapi_print_msg(RTAPI_MSG_WARN,
      LCEC_MSG_PFX "slave %s.%s: SDO 0x%04X:%02X (%s) write failed"
      " – device keeps current value\n",
      master->name, slave->name, idx, sub, name);
}

/* ======================================================================
 * Helper: parse mode string → CoE value
 * Returns -1 on unknown string.
 * ====================================================================== */
static int el7332_parse_mode(const char *s) {
  if (s == NULL)           return DEF_OP_MODE_VELOCITY;
  if (strcmp(s, "velocity") == 0) return DEF_OP_MODE_VELOCITY;
  if (strcmp(s, "chopper")  == 0) return DEF_OP_MODE_CHOPPER;
  return -1;
}

/* ======================================================================
 * Helper: allocate and register a cyclic SDO read request
 * Returns NULL and prints an error on failure.
 * ====================================================================== */
static ec_sdo_request_t *el7332_sdo_request(lcec_slave_t *slave,
                                              uint16_t idx, uint8_t sub,
                                              size_t size) {
  lcec_master_t    *master = slave->master;
  ec_sdo_request_t *req;
  req = ecrt_slave_config_create_sdo_request(slave->config, idx, sub, size);
  if (!req) {
    rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX "ecrt_slave_config_create_sdo_request(0x%04X:%02X) failed"
      " for slave %s.%s\n", idx, sub, master->name, slave->name);
    return NULL;
  }
  ecrt_sdo_request_timeout(req, 500);
  return req;
}

/* ======================================================================
 * Helper: allocate a single HAL pin and store pointer in *ptr
 * ====================================================================== */
static int el7332_pin_new(int comp_id, hal_type_t type, hal_pin_dir_t dir,
                           void **ptr, const char *fmt, ...) {
  char name[HAL_NAME_LEN + 1];
  va_list ap;
  int err;
  va_start(ap, fmt);
  vsnprintf(name, sizeof(name), fmt, ap);
  va_end(ap);
  err = hal_pin_new(name, type, dir, ptr, comp_id);
  if (err)
    rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX "hal_pin_new(%s) failed: %d\n", name, err);
  return err;
}

/* ======================================================================
 * lcec_el7332_init()
 * ====================================================================== */
static int lcec_el7332_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t       *master   = slave->master;
  lcec_el7332_data_t  *hal_data;
  lcec_slave_modparam_t *p;
  int err, ch, mode_val;

  /* --- Per-channel configuration with factory defaults --- */
  struct {
    int      enabled;
    int      mode;          /* CoE value */
    uint16_t max_current;
    uint16_t nom_current;
    uint16_t nom_voltage;
    uint16_t coil_resistance;
    uint16_t reduced_current_pos;
    uint16_t reduced_current_neg;
    uint16_t overload_time;
    uint16_t current_lower_time;
    uint8_t  torque_reduction_pos;
    uint8_t  torque_reduction_neg;
    uint8_t  invert_polarity;
    uint8_t  torque_error_enable;
    uint8_t  torque_auto_reduce;
    uint8_t  info_data_1;
    uint8_t  info_data_2;
    uint8_t  invert_din1;
    uint8_t  invert_din2;
    uint8_t  function_din1;
    uint8_t  function_din2;
    int      enable_info_sdo;
  } cfg[2] = {
    /* ch1 defaults */
    { 1, DEF_OP_MODE_VELOCITY,
      DEF_MAX_CURRENT, DEF_NOM_CURRENT, DEF_NOM_VOLTAGE, DEF_COIL_RESISTANCE,
      DEF_REDUCED_CURRENT_POS, DEF_REDUCED_CURRENT_NEG,
      DEF_OVERLOAD_TIME, DEF_CURRENT_LOWER_TIME,
      DEF_TORQUE_REDUCTION_POS, DEF_TORQUE_REDUCTION_NEG,
      DEF_INVERT_POLARITY, DEF_TORQUE_ERROR_ENABLE, DEF_TORQUE_AUTO_REDUCE,
      DEF_INFO_DATA_1, DEF_INFO_DATA_2,
      DEF_INVERT_DIN1, DEF_INVERT_DIN2,
      DEF_FUNCTION_DIN1, DEF_FUNCTION_DIN2, 0 },
    /* ch2 defaults */
    { 1, DEF_OP_MODE_VELOCITY,
      DEF_MAX_CURRENT, DEF_NOM_CURRENT, DEF_NOM_VOLTAGE, DEF_COIL_RESISTANCE,
      DEF_REDUCED_CURRENT_POS, DEF_REDUCED_CURRENT_NEG,
      DEF_OVERLOAD_TIME, DEF_CURRENT_LOWER_TIME,
      DEF_TORQUE_REDUCTION_POS, DEF_TORQUE_REDUCTION_NEG,
      DEF_INVERT_POLARITY, DEF_TORQUE_ERROR_ENABLE, DEF_TORQUE_AUTO_REDUCE,
      DEF_INFO_DATA_1, DEF_INFO_DATA_2,
      DEF_INVERT_DIN1, DEF_INVERT_DIN2,
      DEF_FUNCTION_DIN1, DEF_FUNCTION_DIN2, 0 },
  };

  /* Device-wide configuration defaults */
  uint16_t cfg_pwm_frequency  = DEF_PWM_FREQUENCY;
  uint8_t  cfg_warning_temp   = DEF_WARNING_TEMP;
  uint8_t  cfg_switchoff_temp = DEF_SWITCHOFF_TEMP;

  slave->proc_read  = lcec_el7332_read;
  slave->proc_write = lcec_el7332_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_el7332_data_t);
  slave->hal_data  = hal_data;
  /* slave->sync_info is intentionally set to NULL.
   * The EL7332 has Enable PDO Configuration: no and Enable PDO Assign: no –
   * its PDO layout is fixed in hardware and cannot be changed.  Setting
   * sync_info would cause the IgH master to attempt ecrt_slave_config_pdos()
   * on every PREOP→SAFEOP transition, which the slave rejects with a harmless
   * but noisy WARNING in dmesg.  PDO entry offsets are registered directly
   * via lcec_pdo_init() / slave->regs, which does not require sync_info.    */
  slave->sync_info = NULL;

  /* --- Collect modParam overrides --- */
  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    switch (p->id) {
      /* Channel 1 */
      case MODPARAM_CH1_ENABLE:
        cfg[0].enabled = p->value.bit ? 1 : 0; break;
      case MODPARAM_CH1_MODE:
        mode_val = el7332_parse_mode(p->value.str);
        if (mode_val < 0) {
          rtapi_print_msg(RTAPI_MSG_ERR,
            LCEC_MSG_PFX "slave %s.%s: ch1_mode invalid value \"%s\""
            " – use \"velocity\" or \"chopper\"\n",
            master->name, slave->name, p->value.str);
          return -EINVAL;
        }
        cfg[0].mode = mode_val; break;
      case MODPARAM_CH1_MAX_CURRENT:
        cfg[0].max_current          = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_NOM_CURRENT:
        cfg[0].nom_current          = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_NOM_VOLTAGE:
        cfg[0].nom_voltage          = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_COIL_RESISTANCE:
        cfg[0].coil_resistance      = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_REDUCED_CURRENT_POS:
        cfg[0].reduced_current_pos  = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_REDUCED_CURRENT_NEG:
        cfg[0].reduced_current_neg  = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_OVERLOAD_TIME:
        cfg[0].overload_time        = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_CURRENT_LOWER_TIME:
        cfg[0].current_lower_time   = (uint16_t)p->value.u32; break;
      case MODPARAM_CH1_TORQUE_REDUCTION_POS:
        cfg[0].torque_reduction_pos = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH1_TORQUE_REDUCTION_NEG:
        cfg[0].torque_reduction_neg = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH1_INVERT_POLARITY:
        cfg[0].invert_polarity      = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH1_TORQUE_ERROR_ENABLE:
        cfg[0].torque_error_enable  = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH1_TORQUE_AUTO_REDUCE:
        cfg[0].torque_auto_reduce   = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH1_INFO_DATA_1:
        cfg[0].info_data_1          = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH1_INFO_DATA_2:
        cfg[0].info_data_2          = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH1_INVERT_DIN1:
        cfg[0].invert_din1          = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH1_INVERT_DIN2:
        cfg[0].invert_din2          = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH1_FUNCTION_DIN1:
        cfg[0].function_din1        = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH1_FUNCTION_DIN2:
        cfg[0].function_din2        = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH1_ENABLE_INFO_SDO:
        cfg[0].enable_info_sdo      = p->value.bit ? 1 : 0;   break;
      /* Channel 2 */
      case MODPARAM_CH2_ENABLE:
        cfg[1].enabled = p->value.bit ? 1 : 0; break;
      case MODPARAM_CH2_MODE:
        mode_val = el7332_parse_mode(p->value.str);
        if (mode_val < 0) {
          rtapi_print_msg(RTAPI_MSG_ERR,
            LCEC_MSG_PFX "slave %s.%s: ch2_mode invalid value \"%s\""
            " – use \"velocity\" or \"chopper\"\n",
            master->name, slave->name, p->value.str);
          return -EINVAL;
        }
        cfg[1].mode = mode_val; break;
      case MODPARAM_CH2_MAX_CURRENT:
        cfg[1].max_current          = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_NOM_CURRENT:
        cfg[1].nom_current          = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_NOM_VOLTAGE:
        cfg[1].nom_voltage          = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_COIL_RESISTANCE:
        cfg[1].coil_resistance      = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_REDUCED_CURRENT_POS:
        cfg[1].reduced_current_pos  = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_REDUCED_CURRENT_NEG:
        cfg[1].reduced_current_neg  = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_OVERLOAD_TIME:
        cfg[1].overload_time        = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_CURRENT_LOWER_TIME:
        cfg[1].current_lower_time   = (uint16_t)p->value.u32; break;
      case MODPARAM_CH2_TORQUE_REDUCTION_POS:
        cfg[1].torque_reduction_pos = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH2_TORQUE_REDUCTION_NEG:
        cfg[1].torque_reduction_neg = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH2_INVERT_POLARITY:
        cfg[1].invert_polarity      = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH2_TORQUE_ERROR_ENABLE:
        cfg[1].torque_error_enable  = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH2_TORQUE_AUTO_REDUCE:
        cfg[1].torque_auto_reduce   = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH2_INFO_DATA_1:
        cfg[1].info_data_1          = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH2_INFO_DATA_2:
        cfg[1].info_data_2          = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH2_INVERT_DIN1:
        cfg[1].invert_din1          = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH2_INVERT_DIN2:
        cfg[1].invert_din2          = p->value.bit ? 1 : 0;   break;
      case MODPARAM_CH2_FUNCTION_DIN1:
        cfg[1].function_din1        = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH2_FUNCTION_DIN2:
        cfg[1].function_din2        = (uint8_t)p->value.u32;  break;
      case MODPARAM_CH2_ENABLE_INFO_SDO:
        cfg[1].enable_info_sdo      = p->value.bit ? 1 : 0;   break;
      /* Device-wide */
      case MODPARAM_PWM_FREQUENCY:
        cfg_pwm_frequency  = (uint16_t)p->value.u32; break;
      case MODPARAM_WARNING_TEMP:
        cfg_warning_temp   = (uint8_t)p->value.u32;  break;
      case MODPARAM_SWITCHOFF_TEMP:
        cfg_switchoff_temp = (uint8_t)p->value.u32;  break;
      default:
        rtapi_print_msg(RTAPI_MSG_WARN,
          LCEC_MSG_PFX "unknown modParam id %d for slave %s.%s\n",
          p->id, master->name, slave->name);
        break;
    }
  }

  /* --- PDO entry registration --- */
  /* Ch1 RxPDO (7020) */
  lcec_pdo_init(slave, 0x7020, 0x01, &hal_data->ch[0].enable_os,        &hal_data->ch[0].enable_bp);
  lcec_pdo_init(slave, 0x7020, 0x02, &hal_data->ch[0].reset_os,         &hal_data->ch[0].reset_bp);
  lcec_pdo_init(slave, 0x7020, 0x03, &hal_data->ch[0].reduce_torque_os, &hal_data->ch[0].reduce_torque_bp);
  lcec_pdo_init(slave, 0x7020, 0x21, &hal_data->ch[0].velocity_os,      NULL);
  /* Ch1 TxPDO (6020) */
  lcec_pdo_init(slave, 0x6020, 0x01, &hal_data->ch[0].ready_to_enable_os, &hal_data->ch[0].ready_to_enable_bp);
  lcec_pdo_init(slave, 0x6020, 0x02, &hal_data->ch[0].ready_os,           &hal_data->ch[0].ready_bp);
  lcec_pdo_init(slave, 0x6020, 0x03, &hal_data->ch[0].warning_os,         &hal_data->ch[0].warning_bp);
  lcec_pdo_init(slave, 0x6020, 0x04, &hal_data->ch[0].error_os,           &hal_data->ch[0].error_bp);
  lcec_pdo_init(slave, 0x6020, 0x05, &hal_data->ch[0].moving_pos_os,      &hal_data->ch[0].moving_pos_bp);
  lcec_pdo_init(slave, 0x6020, 0x06, &hal_data->ch[0].moving_neg_os,      &hal_data->ch[0].moving_neg_bp);
  lcec_pdo_init(slave, 0x6020, 0x07, &hal_data->ch[0].torque_reduced_os,  &hal_data->ch[0].torque_reduced_bp);
  lcec_pdo_init(slave, 0x6020, 0x0C, &hal_data->ch[0].din1_os,            &hal_data->ch[0].din1_bp);
  lcec_pdo_init(slave, 0x6020, 0x0D, &hal_data->ch[0].din2_os,            &hal_data->ch[0].din2_bp);
  lcec_pdo_init(slave, 0x6020, 0x0E, &hal_data->ch[0].sync_error_os,      &hal_data->ch[0].sync_error_bp);
  lcec_pdo_init(slave, 0x6020, 0x10, &hal_data->ch[0].tx_toggle_os,       &hal_data->ch[0].tx_toggle_bp);
  /* 6020:11 / 6020:12 info data not in hardware fixed PDO – read via SDO */
  /* Ch2 RxPDO (7030) */
  lcec_pdo_init(slave, 0x7030, 0x01, &hal_data->ch[1].enable_os,        &hal_data->ch[1].enable_bp);
  lcec_pdo_init(slave, 0x7030, 0x02, &hal_data->ch[1].reset_os,         &hal_data->ch[1].reset_bp);
  lcec_pdo_init(slave, 0x7030, 0x03, &hal_data->ch[1].reduce_torque_os, &hal_data->ch[1].reduce_torque_bp);
  lcec_pdo_init(slave, 0x7030, 0x21, &hal_data->ch[1].velocity_os,      NULL);
  /* Ch2 TxPDO (6030) */
  lcec_pdo_init(slave, 0x6030, 0x01, &hal_data->ch[1].ready_to_enable_os, &hal_data->ch[1].ready_to_enable_bp);
  lcec_pdo_init(slave, 0x6030, 0x02, &hal_data->ch[1].ready_os,           &hal_data->ch[1].ready_bp);
  lcec_pdo_init(slave, 0x6030, 0x03, &hal_data->ch[1].warning_os,         &hal_data->ch[1].warning_bp);
  lcec_pdo_init(slave, 0x6030, 0x04, &hal_data->ch[1].error_os,           &hal_data->ch[1].error_bp);
  lcec_pdo_init(slave, 0x6030, 0x05, &hal_data->ch[1].moving_pos_os,      &hal_data->ch[1].moving_pos_bp);
  lcec_pdo_init(slave, 0x6030, 0x06, &hal_data->ch[1].moving_neg_os,      &hal_data->ch[1].moving_neg_bp);
  lcec_pdo_init(slave, 0x6030, 0x07, &hal_data->ch[1].torque_reduced_os,  &hal_data->ch[1].torque_reduced_bp);
  lcec_pdo_init(slave, 0x6030, 0x0C, &hal_data->ch[1].din1_os,            &hal_data->ch[1].din1_bp);
  lcec_pdo_init(slave, 0x6030, 0x0D, &hal_data->ch[1].din2_os,            &hal_data->ch[1].din2_bp);
  lcec_pdo_init(slave, 0x6030, 0x0E, &hal_data->ch[1].sync_error_os,      &hal_data->ch[1].sync_error_bp);
  lcec_pdo_init(slave, 0x6030, 0x10, &hal_data->ch[1].tx_toggle_os,       &hal_data->ch[1].tx_toggle_bp);
  /* 6030:11 / 6030:12 info data not in hardware fixed PDO – read via SDO */

  /* --- HAL pins – per channel --- */
  for (ch = 0; ch < 2; ch++) {
    lcec_el7332_ch_t *c = &hal_data->ch[ch];
    int n = ch + 1;

    c->modparam_enabled  = cfg[ch].enabled;
    c->enable_info_sdo   = cfg[ch].enable_info_sdo;
    c->info_data_1_sel   = cfg[ch].info_data_1;
    c->info_data_2_sel   = cfg[ch].info_data_2;
    c->prev_reset        = 0;

#define PIN_NEW(type, dir, ptr, fmt, ...) \
    do { \
      if ((err = el7332_pin_new(comp_id, (type), (dir), (void **)(ptr), \
                                 (fmt), __VA_ARGS__)) != 0) return err; \
    } while(0)

    PIN_NEW(HAL_BIT,   HAL_IN,  &c->enable,        "%s.%s.%s.ch%d.enable",        LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_FLOAT, HAL_IN,  &c->velocity_cmd,  "%s.%s.%s.ch%d.velocity-cmd",  LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_IN,  &c->reduce_torque, "%s.%s.%s.ch%d.reduce-torque", LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_IN,  &c->reset,         "%s.%s.%s.ch%d.reset",         LCEC_MODULE_NAME, master->name, slave->name, n);

    PIN_NEW(HAL_BIT,   HAL_OUT, &c->ready_to_enable, "%s.%s.%s.ch%d.ready-to-enable", LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->ready,           "%s.%s.%s.ch%d.ready",           LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->warning,         "%s.%s.%s.ch%d.warning",         LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->error,           "%s.%s.%s.ch%d.error",           LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->moving_pos,      "%s.%s.%s.ch%d.moving-pos",      LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->moving_neg,      "%s.%s.%s.ch%d.moving-neg",      LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->torque_reduced,  "%s.%s.%s.ch%d.torque-reduced",  LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->din1,            "%s.%s.%s.ch%d.din1",            LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->din2,            "%s.%s.%s.ch%d.din2",            LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->sync_error,      "%s.%s.%s.ch%d.sync-error",      LCEC_MODULE_NAME, master->name, slave->name, n);

    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_saturated,       "%s.%s.%s.ch%d.diagnose.saturated",       LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_over_temp,       "%s.%s.%s.ch%d.diagnose.over-temp",       LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_torque_overload, "%s.%s.%s.ch%d.diagnose.torque-overload", LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_under_voltage,   "%s.%s.%s.ch%d.diagnose.under-voltage",   LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_over_voltage,    "%s.%s.%s.ch%d.diagnose.over-voltage",    LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_short_circuit,   "%s.%s.%s.ch%d.diagnose.short-circuit",   LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_no_ctrl_power,   "%s.%s.%s.ch%d.diagnose.no-ctrl-power",   LCEC_MODULE_NAME, master->name, slave->name, n);
    PIN_NEW(HAL_BIT,   HAL_OUT, &c->diag_misc_error,      "%s.%s.%s.ch%d.diagnose.misc-error",      LCEC_MODULE_NAME, master->name, slave->name, n);

    /* Optional info.* pins – only created when chN_enable_info_sdo = true.
     * All read via SDO from 9020 / 9030; 9020:11/12 mirror the info_data
     * selection configured by modParam chN_info_data_1/2.              */
    if (c->enable_info_sdo) {
      PIN_NEW(HAL_S32, HAL_OUT, &c->info_coil_voltage,      "%s.%s.%s.ch%d.info.coil-voltage",      LCEC_MODULE_NAME, master->name, slave->name, n);
      PIN_NEW(HAL_S32, HAL_OUT, &c->info_coil_voltage_calc, "%s.%s.%s.ch%d.info.coil-voltage-calc", LCEC_MODULE_NAME, master->name, slave->name, n);
      PIN_NEW(HAL_S32, HAL_OUT, &c->info_coil_current,      "%s.%s.%s.ch%d.info.coil-current",      LCEC_MODULE_NAME, master->name, slave->name, n);
      PIN_NEW(HAL_S32, HAL_OUT, &c->info_control_error,     "%s.%s.%s.ch%d.info.control-error",     LCEC_MODULE_NAME, master->name, slave->name, n);
      PIN_NEW(HAL_S32, HAL_OUT, &c->info_duty_cycle,        "%s.%s.%s.ch%d.info.duty-cycle",        LCEC_MODULE_NAME, master->name, slave->name, n);
      PIN_NEW(HAL_S32, HAL_OUT, &c->info_overload_time,     "%s.%s.%s.ch%d.info.overload-time",     LCEC_MODULE_NAME, master->name, slave->name, n);
    }

#undef PIN_NEW
  }

  /* --- HAL pins – device-wide --- */
  if ((err = el7332_pin_new(comp_id, HAL_S32, HAL_OUT,
        (void **)&hal_data->internal_temp,
        "%s.%s.%s.internal-temp",
        LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

  if ((err = el7332_pin_new(comp_id, HAL_U32, HAL_OUT,
        (void **)&hal_data->supply_voltage,
        "%s.%s.%s.supply-voltage",
        LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

  /* --- Startup SDOs – Channel 1 (0x8020 / 0x8022) --- */
  el7332_sdo_write16(slave, 0x8020, 0x01, cfg[0].max_current,         DEF_MAX_CURRENT,         "ch1_max_current");
  el7332_sdo_write16(slave, 0x8020, 0x02, cfg[0].nom_current,         DEF_NOM_CURRENT,         "ch1_nom_current");
  el7332_sdo_write16(slave, 0x8020, 0x03, cfg[0].nom_voltage,         DEF_NOM_VOLTAGE,         "ch1_nom_voltage");
  el7332_sdo_write16(slave, 0x8020, 0x04, cfg[0].coil_resistance,     DEF_COIL_RESISTANCE,     "ch1_coil_resistance");
  el7332_sdo_write16(slave, 0x8020, 0x05, cfg[0].reduced_current_pos, DEF_REDUCED_CURRENT_POS, "ch1_reduced_current_pos");
  el7332_sdo_write16(slave, 0x8020, 0x06, cfg[0].reduced_current_neg, DEF_REDUCED_CURRENT_NEG, "ch1_reduced_current_neg");
  el7332_sdo_write16(slave, 0x8020, 0x0C, cfg[0].overload_time,       DEF_OVERLOAD_TIME,       "ch1_overload_time");
  el7332_sdo_write16(slave, 0x8020, 0x0D, cfg[0].current_lower_time,  DEF_CURRENT_LOWER_TIME,  "ch1_current_lower_time");
  el7332_sdo_write8 (slave, 0x8020, 0x0E, cfg[0].torque_reduction_pos,DEF_TORQUE_REDUCTION_POS,"ch1_torque_reduction_pos");
  el7332_sdo_write8 (slave, 0x8020, 0x0F, cfg[0].torque_reduction_neg,DEF_TORQUE_REDUCTION_NEG,"ch1_torque_reduction_neg");
  el7332_sdo_write8 (slave, 0x8022, 0x01, (uint8_t)cfg[0].mode,       DEF_OP_MODE_VELOCITY,    "ch1_mode");
  el7332_sdo_write8 (slave, 0x8022, 0x09, cfg[0].invert_polarity,     DEF_INVERT_POLARITY,     "ch1_invert_polarity");
  el7332_sdo_write8 (slave, 0x8022, 0x0A, cfg[0].torque_error_enable, DEF_TORQUE_ERROR_ENABLE, "ch1_torque_error_enable");
  el7332_sdo_write8 (slave, 0x8022, 0x0B, cfg[0].torque_auto_reduce,  DEF_TORQUE_AUTO_REDUCE,  "ch1_torque_auto_reduce");
  el7332_sdo_write8 (slave, 0x8022, 0x11, cfg[0].info_data_1,         DEF_INFO_DATA_1,         "ch1_info_data_1");
  el7332_sdo_write8 (slave, 0x8022, 0x19, cfg[0].info_data_2,         DEF_INFO_DATA_2,         "ch1_info_data_2");
  el7332_sdo_write8 (slave, 0x8022, 0x30, cfg[0].invert_din1,         DEF_INVERT_DIN1,         "ch1_invert_din1");
  el7332_sdo_write8 (slave, 0x8022, 0x31, cfg[0].invert_din2,         DEF_INVERT_DIN2,         "ch1_invert_din2");
  el7332_sdo_write8 (slave, 0x8022, 0x32, cfg[0].function_din1,       DEF_FUNCTION_DIN1,       "ch1_function_din1");
  el7332_sdo_write8 (slave, 0x8022, 0x36, cfg[0].function_din2,       DEF_FUNCTION_DIN2,       "ch1_function_din2");

  /* --- Startup SDOs – Channel 2 (0x8030 / 0x8032) --- */
  el7332_sdo_write16(slave, 0x8030, 0x01, cfg[1].max_current,         DEF_MAX_CURRENT,         "ch2_max_current");
  el7332_sdo_write16(slave, 0x8030, 0x02, cfg[1].nom_current,         DEF_NOM_CURRENT,         "ch2_nom_current");
  el7332_sdo_write16(slave, 0x8030, 0x03, cfg[1].nom_voltage,         DEF_NOM_VOLTAGE,         "ch2_nom_voltage");
  el7332_sdo_write16(slave, 0x8030, 0x04, cfg[1].coil_resistance,     DEF_COIL_RESISTANCE,     "ch2_coil_resistance");
  el7332_sdo_write16(slave, 0x8030, 0x05, cfg[1].reduced_current_pos, DEF_REDUCED_CURRENT_POS, "ch2_reduced_current_pos");
  el7332_sdo_write16(slave, 0x8030, 0x06, cfg[1].reduced_current_neg, DEF_REDUCED_CURRENT_NEG, "ch2_reduced_current_neg");
  el7332_sdo_write16(slave, 0x8030, 0x0C, cfg[1].overload_time,       DEF_OVERLOAD_TIME,       "ch2_overload_time");
  el7332_sdo_write16(slave, 0x8030, 0x0D, cfg[1].current_lower_time,  DEF_CURRENT_LOWER_TIME,  "ch2_current_lower_time");
  el7332_sdo_write8 (slave, 0x8030, 0x0E, cfg[1].torque_reduction_pos,DEF_TORQUE_REDUCTION_POS,"ch2_torque_reduction_pos");
  el7332_sdo_write8 (slave, 0x8030, 0x0F, cfg[1].torque_reduction_neg,DEF_TORQUE_REDUCTION_NEG,"ch2_torque_reduction_neg");
  el7332_sdo_write8 (slave, 0x8032, 0x01, (uint8_t)cfg[1].mode,       DEF_OP_MODE_VELOCITY,    "ch2_mode");
  el7332_sdo_write8 (slave, 0x8032, 0x09, cfg[1].invert_polarity,     DEF_INVERT_POLARITY,     "ch2_invert_polarity");
  el7332_sdo_write8 (slave, 0x8032, 0x0A, cfg[1].torque_error_enable, DEF_TORQUE_ERROR_ENABLE, "ch2_torque_error_enable");
  el7332_sdo_write8 (slave, 0x8032, 0x0B, cfg[1].torque_auto_reduce,  DEF_TORQUE_AUTO_REDUCE,  "ch2_torque_auto_reduce");
  el7332_sdo_write8 (slave, 0x8032, 0x11, cfg[1].info_data_1,         DEF_INFO_DATA_1,         "ch2_info_data_1");
  el7332_sdo_write8 (slave, 0x8032, 0x19, cfg[1].info_data_2,         DEF_INFO_DATA_2,         "ch2_info_data_2");
  el7332_sdo_write8 (slave, 0x8032, 0x30, cfg[1].invert_din1,         DEF_INVERT_DIN1,         "ch2_invert_din1");
  el7332_sdo_write8 (slave, 0x8032, 0x31, cfg[1].invert_din2,         DEF_INVERT_DIN2,         "ch2_invert_din2");
  el7332_sdo_write8 (slave, 0x8032, 0x32, cfg[1].function_din1,       DEF_FUNCTION_DIN1,       "ch2_function_din1");
  el7332_sdo_write8 (slave, 0x8032, 0x36, cfg[1].function_din2,       DEF_FUNCTION_DIN2,       "ch2_function_din2");

  /* --- Startup SDOs – Device-wide (0xF80F Vendor data) --- */
  el7332_sdo_write16(slave, 0xF80F, 0x01, cfg_pwm_frequency,  DEF_PWM_FREQUENCY,  "pwm_frequency");
  el7332_sdo_write8 (slave, 0xF80F, 0x04, cfg_warning_temp,   DEF_WARNING_TEMP,   "warning_temp");
  el7332_sdo_write8 (slave, 0xF80F, 0x05, cfg_switchoff_temp, DEF_SWITCHOFF_TEMP, "switchoff_temp");

  /* --- Cyclic SDO read requests --- */

  /* F900:02  Internal temperature (INT8) */
  hal_data->sdo_internal_temp = el7332_sdo_request(slave, 0xF900, 0x02, SDO_SIZE_TEMP);
  if (!hal_data->sdo_internal_temp) return -EIO;

  /* F900:05  Motor supply voltage (UINT16) */
  hal_data->sdo_supply_voltage = el7332_sdo_request(slave, 0xF900, 0x05, SDO_SIZE_VOLTAGE);
  if (!hal_data->sdo_supply_voltage) return -EIO;

  /* A020:xx / A030:xx  DCM Diag data – 8 individual BOOLEAN requests per channel.
   * Complete Access is not supported by this IgH master version; each subindex
   * is confirmed readable as uint8 via  ethercat upload --type uint8.          */
  for (ch = 0; ch < 2; ch++) {
    lcec_el7332_ch_t *c       = &hal_data->ch[ch];
    uint16_t          diag_idx = (ch == 0) ? 0xA020 : 0xA030;

    c->sdo_diag_saturated       = el7332_sdo_request(slave, diag_idx, 0x01, 1);
    c->sdo_diag_over_temp       = el7332_sdo_request(slave, diag_idx, 0x02, 1);
    c->sdo_diag_torque_overload = el7332_sdo_request(slave, diag_idx, 0x03, 1);
    c->sdo_diag_under_voltage   = el7332_sdo_request(slave, diag_idx, 0x04, 1);
    c->sdo_diag_over_voltage    = el7332_sdo_request(slave, diag_idx, 0x05, 1);
    c->sdo_diag_short_circuit   = el7332_sdo_request(slave, diag_idx, 0x06, 1);
    c->sdo_diag_no_ctrl_power   = el7332_sdo_request(slave, diag_idx, 0x08, 1);
    c->sdo_diag_misc_error      = el7332_sdo_request(slave, diag_idx, 0x09, 1);

    if (!c->sdo_diag_saturated       || !c->sdo_diag_over_temp       ||
        !c->sdo_diag_torque_overload || !c->sdo_diag_under_voltage   ||
        !c->sdo_diag_over_voltage    || !c->sdo_diag_short_circuit   ||
        !c->sdo_diag_no_ctrl_power   || !c->sdo_diag_misc_error)
      return -EIO;

    /* Optional 9020 / 9030 info data SDO requests */
    if (c->enable_info_sdo) {
      uint16_t info_idx = (ch == 0) ? 0x9020 : 0x9030;
      /* NOTE: 9020:11 / 9020:12 absent in Rev14/SW07 – not requested */
      c->sdo_info_coil_voltage  = el7332_sdo_request(slave, info_idx, 0x02, 2);
      c->sdo_info_coil_current  = el7332_sdo_request(slave, info_idx, 0x03, 2);
      c->sdo_info_control_error = el7332_sdo_request(slave, info_idx, 0x05, 2);
      c->sdo_info_duty_cycle    = el7332_sdo_request(slave, info_idx, 0x06, 1);
      c->sdo_info_overload_time = el7332_sdo_request(slave, info_idx, 0x09, 2);
      if (!c->sdo_info_coil_voltage  || !c->sdo_info_coil_current  ||
          !c->sdo_info_control_error || !c->sdo_info_duty_cycle     ||
          !c->sdo_info_overload_time)
        return -EIO;
    }
  }

  hal_data->sdo_read_timer = SDO_READ_INTERVAL - 1;
  hal_data->sdo_seq_index  = 0;

  return 0;
}

/* ======================================================================
 * lcec_el7332_read()  –  EtherCAT cycle → HAL
 * ====================================================================== */
static void lcec_el7332_read(lcec_slave_t *slave, long period) {
  lcec_master_t      *master   = slave->master;
  lcec_el7332_data_t *hal_data = (lcec_el7332_data_t *)slave->hal_data;
  uint8_t            *pd       = master->process_data;
  int                 ch;

  if (!slave->state.operational) return;

  /* --- PDO reads – per channel --- */
  for (ch = 0; ch < 2; ch++) {
    lcec_el7332_ch_t *c = &hal_data->ch[ch];

    *c->ready_to_enable = EC_READ_BIT(pd + c->ready_to_enable_os, c->ready_to_enable_bp);
    *c->ready           = EC_READ_BIT(pd + c->ready_os,           c->ready_bp);
    *c->warning         = EC_READ_BIT(pd + c->warning_os,         c->warning_bp);
    *c->error           = EC_READ_BIT(pd + c->error_os,           c->error_bp);
    *c->moving_pos      = EC_READ_BIT(pd + c->moving_pos_os,      c->moving_pos_bp);
    *c->moving_neg      = EC_READ_BIT(pd + c->moving_neg_os,      c->moving_neg_bp);
    *c->torque_reduced  = EC_READ_BIT(pd + c->torque_reduced_os,  c->torque_reduced_bp);
    *c->din1            = EC_READ_BIT(pd + c->din1_os,            c->din1_bp);
    *c->din2            = EC_READ_BIT(pd + c->din2_os,            c->din2_bp);
    *c->sync_error      = EC_READ_BIT(pd + c->sync_error_os,      c->sync_error_bp);
    /* tx_toggle read to maintain correct PDO offset accounting – not exposed */
    (void)EC_READ_BIT(pd + c->tx_toggle_os, c->tx_toggle_bp);
  }

  /* --- Cyclic SDO reads – serialized, one request per tick.
   *
   * All SDO requests are collected into a flat array each cycle.
   * sdo_read_timer counts servo cycles; when it reaches SDO_READ_INTERVAL
   * the current slot (sdo_seq_index) is serviced and the result is stored.
   * sdo_seq_index then advances to the next slot.  This guarantees that at
   * most ONE SDO upload is in flight at any time, preventing mailbox
   * collisions (EtherCAT abort 0x08000022 / "wrong SDO" desync) that occur
   * when multiple requests are queued simultaneously on EL7332 Rev14/SW07.
   *
   * Slot layout (max 28 slots):
   *   0   F900:02  internal temperature
   *   1   F900:05  supply voltage
   *   2   A020:01  ch1 diag saturated
   *   3   A020:02  ch1 diag over-temp
   *   4   A020:03  ch1 diag torque-overload
   *   5   A020:04  ch1 diag under-voltage
   *   6   A020:05  ch1 diag over-voltage
   *   7   A020:06  ch1 diag short-circuit
   *   8   A020:08  ch1 diag no-ctrl-power
   *   9   A020:09  ch1 diag misc-error
   *  10   A030:01  ch2 diag saturated
   *  ...  (same pattern)
   *  17   A030:09  ch2 diag misc-error
   *  18+  9020:xx  ch1 info (only if enable_info_sdo)
   *  23+  9030:xx  ch2 info (only if enable_info_sdo)
   */
  {
    /* Build flat slot array – pointers only, no allocation */
    typedef struct {
      ec_sdo_request_t  *req;
      volatile void     *out;  /* hal_s32_t*, hal_u32_t*, or hal_bit_t* – all volatile */
      int                is_s8;
      int                is_u32;
      int                is_bool;
    } sdo_slot_t;

    sdo_slot_t slots[32];
    unsigned int n = 0;

#define SLOT_U16(r, p)  do { slots[n].req=(r); slots[n].out=(p); slots[n].is_s8=0; slots[n].is_u32=0; slots[n].is_bool=0; n++; } while(0)
#define SLOT_S8(r, p)   do { slots[n].req=(r); slots[n].out=(p); slots[n].is_s8=1; slots[n].is_u32=0; slots[n].is_bool=0; n++; } while(0)
#define SLOT_U32(r, p)  do { slots[n].req=(r); slots[n].out=(p); slots[n].is_s8=0; slots[n].is_u32=1; slots[n].is_bool=0; n++; } while(0)
#define SLOT_BIT(r, p)  do { slots[n].req=(r); slots[n].out=(p); slots[n].is_s8=0; slots[n].is_u32=0; slots[n].is_bool=1; n++; } while(0)

    SLOT_S8 (hal_data->sdo_internal_temp,         hal_data->internal_temp);
    SLOT_U32(hal_data->sdo_supply_voltage,         hal_data->supply_voltage);

    for (ch = 0; ch < 2; ch++) {
      lcec_el7332_ch_t *c = &hal_data->ch[ch];
      SLOT_BIT(c->sdo_diag_saturated,       c->diag_saturated);
      SLOT_BIT(c->sdo_diag_over_temp,       c->diag_over_temp);
      SLOT_BIT(c->sdo_diag_torque_overload, c->diag_torque_overload);
      SLOT_BIT(c->sdo_diag_under_voltage,   c->diag_under_voltage);
      SLOT_BIT(c->sdo_diag_over_voltage,    c->diag_over_voltage);
      SLOT_BIT(c->sdo_diag_short_circuit,   c->diag_short_circuit);
      SLOT_BIT(c->sdo_diag_no_ctrl_power,   c->diag_no_ctrl_power);
      SLOT_BIT(c->sdo_diag_misc_error,      c->diag_misc_error);
    }

    for (ch = 0; ch < 2; ch++) {
      lcec_el7332_ch_t *c = &hal_data->ch[ch];
      if (c->enable_info_sdo) {
        SLOT_U16(c->sdo_info_coil_voltage,  c->info_coil_voltage);
        SLOT_U16(c->sdo_info_coil_current,  c->info_coil_current);
        SLOT_U16(c->sdo_info_control_error, c->info_control_error);
        SLOT_S8 (c->sdo_info_duty_cycle,    c->info_duty_cycle);
        SLOT_U16(c->sdo_info_overload_time, c->info_overload_time);
      }
    }

#undef SLOT_U16
#undef SLOT_S8
#undef SLOT_U32
#undef SLOT_BIT

    /* Wrap sequence index */
    if (hal_data->sdo_seq_index >= n)
      hal_data->sdo_seq_index = 0;

    if (++hal_data->sdo_read_timer >= SDO_READ_INTERVAL) {
      hal_data->sdo_read_timer = 0;

      /* Service exactly one slot */
      sdo_slot_t *s = &slots[hal_data->sdo_seq_index];

      switch (ecrt_sdo_request_state(s->req)) {
        case EC_REQUEST_UNUSED:
          ecrt_sdo_request_read(s->req);
          break;

        case EC_REQUEST_SUCCESS:
          if (s->is_bool) {
            *(volatile hal_bit_t *)s->out = EC_READ_U8(ecrt_sdo_request_data(s->req)) ? 1 : 0;
          } else if (s->is_s8) {
            *(volatile hal_s32_t *)s->out = (hal_s32_t)(int8_t)EC_READ_U8(ecrt_sdo_request_data(s->req));
          } else if (s->is_u32) {
            *(volatile hal_u32_t *)s->out = (hal_u32_t)EC_READ_U16(ecrt_sdo_request_data(s->req));
          } else {
            *(volatile hal_s32_t *)s->out = (hal_s32_t)(int16_t)EC_READ_U16(ecrt_sdo_request_data(s->req));
          }
          /* Advance to next slot and immediately kick off its read */
          hal_data->sdo_seq_index = (hal_data->sdo_seq_index + 1) % n;
          ecrt_sdo_request_read(slots[hal_data->sdo_seq_index].req);
          break;

        case EC_REQUEST_BUSY:
          /* Still waiting – do not advance, do not reset timer */
          hal_data->sdo_read_timer = SDO_READ_INTERVAL; /* re-fire next cycle */
          break;

        case EC_REQUEST_ERROR:
          rtapi_print_msg(RTAPI_MSG_WARN,
            LCEC_MSG_PFX "SDO read slot %u failed for slave %s.%s – skipping\n",
            hal_data->sdo_seq_index, master->name, slave->name);
          /* Advance past failed slot to avoid permanent stall */
          hal_data->sdo_seq_index = (hal_data->sdo_seq_index + 1) % n;
          ecrt_sdo_request_read(slots[hal_data->sdo_seq_index].req);
          break;
      }
    }

    /* Update coil-voltage-calc for any channel with info SDO enabled.
     * Uses the most recently read supply_voltage and duty_cycle values.  */
    for (ch = 0; ch < 2; ch++) {
      lcec_el7332_ch_t *c = &hal_data->ch[ch];
      if (c->enable_info_sdo) {
        *c->info_coil_voltage_calc =
          (hal_s32_t)((*c->info_duty_cycle * (hal_s32_t)*hal_data->supply_voltage) / 100);
      }
    }
  }
}

/* ======================================================================
 * lcec_el7332_write()  –  HAL → EtherCAT cycle
 *
 * Enable logic:
 *   channel_active = modparam_enabled AND pin_enable
 *   PDO Enable bit = channel_active AND ready_to_enable
 *
 * Velocity scaling:
 *   pin_velocity_cmd (-1.0 … +1.0) → INT16 (-32768 … +32767)
 *   Clamped to [-1.0, +1.0] before scaling.
 *
 * Reset edge detection:
 *   Reset PDO bit is held high only on the rising edge of the reset pin.
 * ====================================================================== */
static void lcec_el7332_write(lcec_slave_t *slave, long period) {
  lcec_el7332_data_t *hal_data = (lcec_el7332_data_t *)slave->hal_data;
  uint8_t            *pd       = slave->master->process_data;
  int                 ch;

  if (!slave->state.operational) return;

  for (ch = 0; ch < 2; ch++) {
    lcec_el7332_ch_t *c   = &hal_data->ch[ch];
    int    ch_active       = c->modparam_enabled && *c->enable;
    int    enable_bit      = ch_active && *c->ready_to_enable;
    hal_float_t vel        = *c->velocity_cmd;
    int16_t     vel_raw;
    int         reset_edge;

    /* Clamp and scale velocity */
    if (!ch_active) {
      vel_raw = 0;
    } else {
      if (vel >  1.0) vel =  1.0;
      if (vel < -1.0) vel = -1.0;
      vel_raw = (int16_t)(vel * 32767.0);
    }

    /* Rising-edge detection for reset */
    reset_edge       = (*c->reset && !c->prev_reset) ? 1 : 0;
    c->prev_reset    = *c->reset;

    /* Write RxPDO */
    EC_WRITE_BIT(pd + c->enable_os,        c->enable_bp,        enable_bit);
    EC_WRITE_BIT(pd + c->reset_os,         c->reset_bp,         reset_edge);
    EC_WRITE_BIT(pd + c->reduce_torque_os, c->reduce_torque_bp, ch_active ? (*c->reduce_torque ? 1 : 0) : 0);
    EC_WRITE_S16(pd + c->velocity_os,      vel_raw);
  }
}

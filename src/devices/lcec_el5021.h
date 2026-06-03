/* lcec_el5021.h
 *
 * LinuxCNC EtherCAT (lcec) driver for Beckhoff EL5021 / EL5021-0090
 * SinCos Encoder Interface.
 *
 * Objects confirmed present on hardware (via TwinCAT CoE-Online / ethercat sdos):
 *   6000:01/03/0B/0E/0F/10/11/12   ENC Inputs
 *   6001:04/05                     SinCos error flags
 *   7000:01/03/11                  ENC Outputs
 *   8000:01/0E                     ENC Settings  (max subindex 0x0E)
 *   8001:01/02/11                  ENC SinCos Settings
 *   A000:11/12                     ENC Diag counters (SDO read)
 *
 * NOT present on this revision (removed):
 *   6000:13    Frequency value
 *   8000:0F    Frequency window base
 *   8000:11    Frequency window
 *   8000:1D    Frequency numerator
 *   8000:1E    Frequency denominator
 *   8000:1F    Frequency filter
 *
 * PRODUCT CODES: verify with  ethercat slaves -v  (field "Product code")
 *   EL5021      → 0x139D3052
 *   EL5021-0090 → verify separately
 */

#ifndef _LCEC_EL5021_H_
#define _LCEC_EL5021_H_

/* ======================================================================
 * Vendor and Product Identifiers
 * ====================================================================== */
#define LCEC_EL5021_VID          LCEC_BECKHOFF_VID   /* 0x00000002           */
#define LCEC_EL5021_PID          0x139D3052          /* VERIFY with ethercat slaves -v */
#define LCEC_EL5021_0090_PID     0x139D9052          /* VERIFY with ethercat slaves -v */

/* ======================================================================
 * modParam IDs  (confirmed writable on this hardware revision)
 * XML: <modParam name="..." value="..."/>
 * ====================================================================== */

/** 8000:01  Enable counter reset via C input     BOOLEAN  def: false */
#define LCEC_EL5021_MODPARAM_ENABLE_C_RESET          0

/** 8000:0E  Reverse counting direction            BOOLEAN  def: false */
#define LCEC_EL5021_MODPARAM_REVERSION_OF_ROTATION   1

/** 8001:01  Enable frequency error detection      BOOLEAN  def: true  */
#define LCEC_EL5021_MODPARAM_ENABLE_FREQ_ERROR       2

/** 8001:02  Enable amplitude error detection      BOOLEAN  def: true  */
#define LCEC_EL5021_MODPARAM_ENABLE_AMP_ERROR        3

/** 8001:11  Period resolution in bits             UINT32   def: 10    */
#define LCEC_EL5021_MODPARAM_ANALOG_RESOLUTION       4

extern const lcec_modparam_desc_t lcec_el5021_modparams[];

int lcec_el5021_init(int comp_id, lcec_slave_t *slave);

#endif /* _LCEC_EL5021_H_ */

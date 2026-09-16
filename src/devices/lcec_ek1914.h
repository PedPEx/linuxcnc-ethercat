/* lcec_ek1914.h
 *
 * LinuxCNC EtherCAT (lcec) driver for the Beckhoff EK1914 —
 * EtherCAT coupler with integrated standard and TwinSAFE I/O:
 *   4x standard DI, 4x standard DO,
 *   2x fail-safe DI, 2x fail-safe DO (exchanged via FSoE).
 *
 * The EK1914 is an FSoE *slave*. It exchanges its safe I/O with a
 * TwinSAFE-Logic-capable device (here: the EL1918 at bus index 1,
 * TwinSAFE address 1) over an FSoE connection. This driver's job is
 *   (a) to expose the standard (non-safe) DI/DO to HAL, and
 *   (b) to register the terminal as an FSoE slave so the EL1918_LOGIC
 *       driver can wire the connection referenced by its fsoeSlaveIdx.
 *
 * The decoded state of the *safe* inputs is NOT available here — it lives
 * inside the CRC-protected FSoE payload (black channel). To read a safe
 * input in HAL, route it to a Standard Output of the EL1918 safety program
 * and read the EL1918 std-out pin. The safe-I/O pins below are diagnostic
 * mirrors of the raw FSoE payload only, never a safety-rated interface.
 *
 * Identity (from `ethercat slaves -v -p 0`):
 *   Vendor Id:       0x00000002   (Beckhoff)
 *   Product code:    0x077A2C52
 *   Revision number: 0x00120000
 *
 * PDO layout is fixed (Enable PDO Assign: no / Enable PDO Configuration: no),
 * so the mapping below must mirror the hardware exactly.
 */

#ifndef _LCEC_EK1914_H_
#define _LCEC_EK1914_H_

/* ======================================================================
 * Vendor and Product Identifiers
 * ====================================================================== */
#define LCEC_EK1914_VID   LCEC_BECKHOFF_VID   /* 0x00000002                    */
#define LCEC_EK1914_PID   0x077A2C52          /* verify: ethercat slaves -v    */

int lcec_ek1914_init(int comp_id, lcec_slave_t *slave);

#endif /* _LCEC_EK1914_H_ */

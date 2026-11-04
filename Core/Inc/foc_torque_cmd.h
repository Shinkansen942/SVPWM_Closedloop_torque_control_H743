/*
 * foc_torque_cmd.h
 *
 * [3] Torque_cmd — Simulink: Torque_cmd block
 *     T_cmd (last_percent) → Id_ref / Iq_ref
 *     (regen limit, torque → Iq, field weakening, MTPA, current circle limit)
 *
 * Formerly FOC.c + target calculation in foc_control_step() (foc_loop.c).
 */

#ifndef INC_FOC_TORQUE_CMD_H_
#define INC_FOC_TORQUE_CMD_H_

#include "foc_loop.h"   // dq_t, foc, motor

/*
 * reads : foc.last_percent, foc.filtered_Id / filtered_Iq, foc.filtered_RPM,
 *         foc.Id/Iq_controller_output (previous cycle = Simulink Feedback),
 *         motor.voltage_power_supply
 * writes: foc.last_percent (regen / derate limited), foc.target_Id / target_Iq,
 *         foc.Id_fw / Iq_fw / Id_MTPA
 */
void torque_cmd_step(void);

/* FW_STARTUP_ID_FIX: preset the field-weakening Id filter (called from can_app.c on enable) */
void torque_cmd_preset_fw_filter(float id_fw);

#endif /* INC_FOC_TORQUE_CMD_H_ */

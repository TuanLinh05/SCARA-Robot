#ifndef SCARA_CART_CORE_INTERNAL_H
#define SCARA_CART_CORE_INTERNAL_H

#include "cart_core.h"

/* Shared transitions for the command adapter and motion/home worker only.
 * Callers serialize state access and handle CT_START through motor I/O. */
const char *ct_inputs(struct ct_state *control, uint32_t now);
void ct_beta_update(struct ct_state *control);
bool ct_pose_valid(const struct ct_state *control, const int32_t position[CT_AXES], int margin_md);
enum ct_result ct_raw_move(struct ct_state *control, const int32_t target[CT_AXES], uint32_t rate, bool park);
uint32_t ct_path_ramp(uint32_t peak, uint32_t end, uint32_t accel);
enum ct_result ct_begin_cal(struct ct_state *control, unsigned axis, enum ct_stage stage, uint32_t now);

#endif

/*
 * calib.h - LeRobot calibration files and joint-space mapping.
 *
 * LeRobot stores one JSON per arm, e.g. on the Pi:
 *   ~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/<id>.json
 *   ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json
 * with, per joint: id, drive_mode, homing_offset, range_min, range_max.
 *
 * The homing offset already lives in each servo's EEPROM, so Present_Position
 * is offset-corrected. Leader -> follower mapping reproduces LeRobot's default
 * teleoperation (MotorNormMode.RANGE_M100_100 for the body, RANGE_0_100 for
 * the gripper):
 *     norm   = (clamp(raw, min, max) - min) / (max - min) * 200 - 100
 *     goal   = (norm + 100) / 200 * (max_f - min_f) + min_f
 */
#ifndef SO101_CALIB_H
#define SO101_CALIB_H

#include <stdint.h>

#define CALIB_NJ 6

typedef struct {
    uint8_t id;
    int     drive_mode;
    int     homing_offset;
    int     range_min;
    int     range_max;
    int     is_gripper;   /* 1 -> 0..100 normalization */
} calib_joint;

typedef struct {
    calib_joint j[CALIB_NJ];   /* ordered as calib_joint_names[] */
} calib_arm;

extern const char *const calib_joint_names[CALIB_NJ];

/* Returns 0 on success, -1 on I/O error, -2 on missing/invalid field.
 * On error, err (if not NULL) receives a human-readable reason. */
int   calib_load(const char *path, calib_arm *arm, char *err, int errlen);

float calib_normalize(const calib_joint *j, int raw);
int   calib_unnormalize(const calib_joint *j, float norm);

#endif /* SO101_CALIB_H */

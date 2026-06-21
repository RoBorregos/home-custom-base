/**
 ******************************************************************************
 * @file    ekf.h
 * @brief   6-state Extended Kalman Filter for mecanum-drive base.
 *
 * State vector x = [px, py, theta, vx_body, vy_body, omega]^T
 *   px, py    : world-frame position [m]
 *   theta     : world-frame heading [rad], wrapped to (-pi, pi]
 *   vx_body   : body-frame forward velocity [m/s]
 *   vy_body   : body-frame lateral velocity [m/s]
 *   omega     : yaw rate [rad/s]
 *
 * Two correction sources:
 *   - wheel-derived body twist (vx_body, vy_body, omega)  via mecanum FK
 *   - BNO085 IMU                (yaw, omega_z)            gated by sequence #
 *
 * All matrices are heap-free and reentrant per-instance; bulky intermediates
 * inside ekf.c are `static` because the filter is only run from one task.
 ******************************************************************************
 */
#ifndef __EKF_H
#define __EKF_H

#include <stdint.h>

#define EKF_N 6

typedef struct {
    double sample_time;                              /* nominal dt [s] */

    /* Process noise std-dev per state (per second). */
    double q_x, q_y, q_theta;
    double q_vx, q_vy, q_omega;

    /* Wheel-twist measurement noise std-dev (body-frame). */
    double r_wheel_vx, r_wheel_vy, r_wheel_omega;

    /* IMU measurement noise std-dev. */
    double r_imu_yaw;
    double r_imu_omega;

    /* Initial covariance diagonal (variance). */
    double init_cov_pos;
    double init_cov_yaw;
    double init_cov_vel;
    double init_cov_omega;
} EKFParams;

typedef struct {
    double x[EKF_N];                /* state */
    double P[EKF_N * EKF_N];        /* covariance, row-major */
    EKFParams params;

    /* When 0, IMU correction is skipped — EKF reduces to a wheel-only
     * kinematic filter (still useful: it carries proper covariance). */
    uint8_t use_imu;

    /* Sequence number of the last IMU sample consumed, so each fresh
     * BNO085 reading produces at most one correction. */
    uint32_t last_imu_seq;

    /* Mirror of robot_localization's `imu0_relative: true`: zero the IMU
     * yaw on the first sample so the robot boots heading-0 regardless of
     * the absolute heading the BNO085 happens to be reporting. Only the
     * yaw is zeroed; omega is always absolute (rate doesn't need a frame).
     * Default: 1 (enabled). */
    uint8_t imu_relative;
    uint8_t imu_yaw_initialized;
    double  imu_yaw_offset;
} EKF;

/* If params is NULL, sensible mecanum defaults are used. */
void ekf_init(EKF *e, const EKFParams *params);

/* Predict step with explicit dt (seconds). */
void ekf_predict(EKF *e, double dt);

/* Wheel-derived body twist correction (3-dim measurement). */
void ekf_correct_wheel_twist(EKF *e, double vx_body, double vy_body, double omega);

/* Combined IMU correction: yaw [rad] + omega_z [rad/s], gated by seq.
 * Returns 1 if the correction was applied, 0 if skipped (stale seq or
 * use_imu == 0). */
int ekf_correct_imu(EKF *e, double yaw, double omega_z, uint32_t seq);

#endif /* __EKF_H */

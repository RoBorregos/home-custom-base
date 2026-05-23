/*
 * ekf.h — 6-state EKF for a planar 4-wheel mecanum mobile base.
 *
 * State:  x = [ x, y, theta, vx, vy, omega ]^T
 *           x, y     position in odom frame [m]
 *           theta    yaw heading           [rad]  (wrapped to (-pi, pi])
 *           vx, vy   body-frame linear velocity [m/s]
 *           omega    yaw rate              [rad/s]
 *
 * Inputs fused:
 *   - body-frame twist (vx, vy, omega) from mecanum forward kinematics
 *   - IMU yaw from quaternion (BNO085 SH2_ROTATION_VECTOR)
 *   - IMU yaw rate (BNO085 SH2_GYROSCOPE_CALIBRATED.z)
 *   - optional body-frame linear acceleration as predict control input
 *
 * Output:
 *   Pose (x, y, z=0, qx, qy, qz, qw) + twist (vx, vy, 0, 0, 0, wz) plus
 *   ROS-style row-major 6x6 covariance arrays — see ekf_fill_*_covariance.
 */
#ifndef EKF_H_
#define EKF_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_N 6   /* state dimension */

typedef struct {
    double sample_time;              /* fallback dt [s] */

    bool   use_imu_yaw;
    bool   use_imu_omega;
    bool   use_imu_acceleration;

    double imu_yaw_sign;             /* +1 or -1 */
    double imu_yaw_offset;           /* [rad] */
    bool   zero_imu_yaw_on_boot;     /* subtract first IMU yaw sample */

    double imu_omega_sign;
    double imu_ax_sign;
    double imu_ay_sign;

    /* outlier gates */
    double max_imu_yaw_jump;         /* [rad] */
    double max_odom_vx;              /* [m/s] */
    double max_odom_vy;              /* [m/s] */
    double max_odom_omega;           /* [rad/s] */

    /* initial covariance diagonal */
    double initial_cov_x;
    double initial_cov_y;
    double initial_cov_theta;
    double initial_cov_vx;
    double initial_cov_vy;
    double initial_cov_omega;

    /* process noise diagonal (variance/second, scaled by dt internally) */
    double process_noise_x;
    double process_noise_y;
    double process_noise_theta;
    double process_noise_vx;
    double process_noise_vy;
    double process_noise_omega;

    /* fallback measurement variances (used when caller passes <= 0) */
    double odom_vx_var;
    double odom_vy_var;
    double odom_omega_var;
    double imu_yaw_var;
    double imu_omega_var;

    double min_variance;             /* floor for safe_variance */
} EKFParams;

typedef struct {
    double    x[EKF_N];
    double    P[EKF_N][EKF_N];
    EKFParams p;

    /* first-IMU-sample zeroing state */
    bool   yaw_boot_captured;
    double yaw_boot;
} EKF;

/* Populate p with the default tuning. */
void ekf_params_defaults(EKFParams *p);

/* Initialise filter at origin with the supplied (or default) parameters. */
void ekf_init(EKF *e, const EKFParams *p);

/* Hard-reset pose; velocities cleared, off-diagonal covariance cleared. */
void ekf_reset_pose(EKF *e, double x, double y, double theta);

/* Predict step. dt is clamped to the params sample_time when out of range. */
void ekf_predict(EKF *e, double dt, double ax_body, double ay_body);

/* Wheel-FK body twist correction. Pass var <= 0 to use the param fallback. */
void ekf_correct_odom_twist(EKF *e,
                            double vx_meas, double vy_meas, double omega_meas,
                            double vx_var,  double vy_var,  double omega_var);

/* IMU yaw correction (quaternion yaw, raw).  Returns true if applied. */
bool ekf_correct_imu_yaw(EKF *e, double yaw_raw, double yaw_var);

/* IMU yaw-rate correction. Returns true if applied. */
bool ekf_correct_imu_omega(EKF *e, double wz_meas, double wz_var);

/* Accessors. */
static inline double ekf_get_x(const EKF *e)     { return e->x[0]; }
static inline double ekf_get_y(const EKF *e)     { return e->x[1]; }
static inline double ekf_get_theta(const EKF *e) { return e->x[2]; }
static inline double ekf_get_vx(const EKF *e)    { return e->x[3]; }
static inline double ekf_get_vy(const EKF *e)    { return e->x[4]; }
static inline double ekf_get_omega(const EKF *e) { return e->x[5]; }

/* Yaw (rad) -> z-axis quaternion (x=y=0). */
void ekf_yaw_to_quaternion(double yaw,
                           float *qx, float *qy, float *qz, float *qw);

/* Fill a ROS-style row-major 6x6 covariance, axes (x,y,z,roll,pitch,yaw).
 * The 3x3 block on (x, y, yaw) comes from the filter; the three planar-base
 * "unobservable" dimensions (z, roll, pitch) are tagged with a large variance
 * (1e6) so downstream consumers know to ignore them. */
void ekf_fill_pose_covariance(const EKF *e, double cov[36]);

/* Same convention, axes (vx, vy, vz, wx, wy, wz). */
void ekf_fill_twist_covariance(const EKF *e, double cov[36]);

#ifdef __cplusplus
}
#endif
#endif /* EKF_H_ */

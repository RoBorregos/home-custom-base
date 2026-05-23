/*
 * ekf.c — 6-state EKF for a planar mecanum mobile base.
 * See ekf.h for the state vector, inputs and outputs.
 *
 * Implementation notes:
 *   - All matrix ops are hand-rolled fixed-size loops. With N=6 and m<=3
 *     a full predict+correct cycle is < 100 us on STM32H7 with the DP-FPU.
 *   - The correction step uses the symmetric Joseph form, which preserves
 *     positive-semidefiniteness of P even after many iterations.
 *   - Matrix inversion is Gauss-Jordan with partial pivoting (max 3x3 in
 *     practice — odom-twist correction).
 */
#include "ekf.h"

#include <math.h>
#include <string.h>

#define EKF_M_MAX 6   /* max measurement-vector size supported by correct_generic */

static double normalize_angle(double a)
{
    return atan2(sin(a), cos(a));
}

static double safe_variance(double v, double fallback, double minv)
{
    if (!isfinite(v) || v <= minv) {
        return (fallback > minv) ? fallback : minv;
    }
    return v;
}

void ekf_params_defaults(EKFParams *p)
{
    p->sample_time = 0.01;             /* 100 Hz telemetry tick */

    p->use_imu_yaw          = true;
    p->use_imu_omega        = true;
    p->use_imu_acceleration = false;   /* off by default — robust starting point */

    p->imu_yaw_sign        = 1.0;
    p->imu_yaw_offset      = 0.0;
    p->zero_imu_yaw_on_boot = true;

    p->imu_omega_sign = 1.0;
    p->imu_ax_sign    = 1.0;
    p->imu_ay_sign    = 1.0;

    p->max_imu_yaw_jump  = 1.0;
    p->max_odom_vx       = 3.0;
    p->max_odom_vy       = 3.0;
    p->max_odom_omega    = 6.0;

    p->initial_cov_x     = 0.10;
    p->initial_cov_y     = 0.10;
    p->initial_cov_theta = 0.10;
    p->initial_cov_vx    = 0.20;
    p->initial_cov_vy    = 0.20;
    p->initial_cov_omega = 0.20;

    p->process_noise_x     = 0.02;
    p->process_noise_y     = 0.02;
    p->process_noise_theta = 0.02;
    p->process_noise_vx    = 0.20;
    p->process_noise_vy    = 0.20;
    p->process_noise_omega = 0.20;

    p->odom_vx_var    = 0.05;
    p->odom_vy_var    = 0.05;
    p->odom_omega_var = 0.05;
    p->imu_yaw_var    = 0.03;
    p->imu_omega_var  = 0.03;

    p->min_variance   = 1.0e-6;
}

void ekf_init(EKF *e, const EKFParams *p)
{
    memset(e, 0, sizeof(*e));
    if (p) e->p = *p;
    else   ekf_params_defaults(&e->p);

    e->P[0][0] = e->p.initial_cov_x;
    e->P[1][1] = e->p.initial_cov_y;
    e->P[2][2] = e->p.initial_cov_theta;
    e->P[3][3] = e->p.initial_cov_vx;
    e->P[4][4] = e->p.initial_cov_vy;
    e->P[5][5] = e->p.initial_cov_omega;
}

void ekf_reset_pose(EKF *e, double x, double y, double theta)
{
    e->x[0] = x;
    e->x[1] = y;
    e->x[2] = normalize_angle(theta);
    e->x[3] = 0.0;
    e->x[4] = 0.0;
    e->x[5] = 0.0;

    e->P[0][0] = e->p.initial_cov_x;
    e->P[1][1] = e->p.initial_cov_y;
    e->P[2][2] = e->p.initial_cov_theta;
    e->P[0][1] = e->P[1][0] = 0.0;
    e->P[0][2] = e->P[2][0] = 0.0;
    e->P[1][2] = e->P[2][1] = 0.0;
}

void ekf_predict(EKF *e, double dt, double ax_body, double ay_body)
{
    if (!isfinite(dt) || dt <= 0.0 || dt > 1.0) {
        dt = e->p.sample_time;
    }

    const double theta = e->x[2];
    const double vx    = e->x[3];
    const double vy    = e->x[4];
    const double omega = e->x[5];

    const double c = cos(theta);
    const double s = sin(theta);

    /* --- state propagation --- */
    double x_new     = e->x[0] + (c * vx - s * vy) * dt;
    double y_new     = e->x[1] + (s * vx + c * vy) * dt;
    double theta_new = theta + omega * dt;
    double vx_new    = vx;
    double vy_new    = vy;
    double omega_new = omega;

    double ax = 0.0, ay = 0.0;
    if (e->p.use_imu_acceleration && isfinite(ax_body) && isfinite(ay_body)) {
        ax = e->p.imu_ax_sign * ax_body;
        ay = e->p.imu_ay_sign * ay_body;
        x_new  += 0.5 * (c * ax - s * ay) * dt * dt;
        y_new  += 0.5 * (s * ax + c * ay) * dt * dt;
        vx_new += ax * dt;
        vy_new += ay * dt;
    }

    /* --- Jacobian F = df/dx --- */
    double F[EKF_N][EKF_N];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_N; i++) F[i][i] = 1.0;
    F[0][2] = -s * vx * dt - c * vy * dt;
    F[0][3] =  c * dt;
    F[0][4] = -s * dt;
    F[1][2] =  c * vx * dt - s * vy * dt;
    F[1][3] =  s * dt;
    F[1][4] =  c * dt;
    F[2][5] =  dt;
    if (e->p.use_imu_acceleration) {
        F[0][2] += -0.5 * (s * ax + c * ay) * dt * dt;
        F[1][2] +=  0.5 * (c * ax - s * ay) * dt * dt;
    }

    /* --- P <- F P F^T + Q --- */
    double FP[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            double sum = 0.0;
            for (int k = 0; k < EKF_N; k++) sum += F[i][k] * e->P[k][j];
            FP[i][j] = sum;
        }
    }
    double FPFt[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            double sum = 0.0;
            for (int k = 0; k < EKF_N; k++) sum += FP[i][k] * F[j][k]; /* F^T[k][j] = F[j][k] */
            FPFt[i][j] = sum;
        }
    }
    FPFt[0][0] += e->p.process_noise_x     * dt;
    FPFt[1][1] += e->p.process_noise_y     * dt;
    FPFt[2][2] += e->p.process_noise_theta * dt;
    FPFt[3][3] += e->p.process_noise_vx    * dt;
    FPFt[4][4] += e->p.process_noise_vy    * dt;
    FPFt[5][5] += e->p.process_noise_omega * dt;
    memcpy(e->P, FPFt, sizeof(e->P));

    /* --- commit state --- */
    e->x[0] = x_new;
    e->x[1] = y_new;
    e->x[2] = normalize_angle(theta_new);
    e->x[3] = vx_new;
    e->x[4] = vy_new;
    e->x[5] = omega_new;
}

/* In-place Gauss-Jordan inversion of an m-by-m matrix with partial pivoting.
 * Returns 0 on success, -1 if the matrix is numerically singular. */
static int invert_mxm(double M[EKF_M_MAX][EKF_M_MAX], int m,
                      double Minv[EKF_M_MAX][EKF_M_MAX])
{
    double A[EKF_M_MAX][2 * EKF_M_MAX];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            A[i][j]     = M[i][j];
            A[i][m + j] = (i == j) ? 1.0 : 0.0;
        }
    }
    for (int col = 0; col < m; col++) {
        int piv = col;
        double best = fabs(A[col][col]);
        for (int r = col + 1; r < m; r++) {
            const double v = fabs(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-15) return -1;
        if (piv != col) {
            for (int j = 0; j < 2 * m; j++) {
                const double tmp = A[col][j]; A[col][j] = A[piv][j]; A[piv][j] = tmp;
            }
        }
        const double diag = A[col][col];
        const double inv  = 1.0 / diag;
        for (int j = 0; j < 2 * m; j++) A[col][j] *= inv;
        for (int r = 0; r < m; r++) {
            if (r == col) continue;
            const double f = A[r][col];
            if (f == 0.0) continue;
            for (int j = 0; j < 2 * m; j++) A[r][j] -= f * A[col][j];
        }
    }
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++)
            Minv[i][j] = A[i][m + j];
    return 0;
}

/* Generic Kalman correction.
 *   m         : measurement-vector size (1..EKF_M_MAX)
 *   z         : measurement vector, length m
 *   H[m][N]   : observation matrix (row-major)
 *   R[m*m]    : measurement noise covariance (row-major)
 *   angle_mask: optional length-m bool array; element i true => innovation[i]
 *               is wrapped to (-pi, pi]
 *
 * Applies Joseph-form covariance update. Silently skips the correction if R is
 * numerically singular.
 */
static void ekf_correct_generic(EKF *e, int m,
                                const double *z,
                                const double H[][EKF_N],
                                const double *R,
                                const bool *angle_mask)
{
    /* y = z - H x  (length m) */
    double y[EKF_M_MAX];
    for (int i = 0; i < m; i++) {
        double s = 0.0;
        for (int k = 0; k < EKF_N; k++) s += H[i][k] * e->x[k];
        y[i] = z[i] - s;
        if (angle_mask && angle_mask[i]) y[i] = normalize_angle(y[i]);
    }

    /* HP = H * P   (m x N) */
    double HP[EKF_M_MAX][EKF_N];
    for (int i = 0; i < m; i++) {
        for (int k = 0; k < EKF_N; k++) {
            double s = 0.0;
            for (int j = 0; j < EKF_N; j++) s += H[i][j] * e->P[j][k];
            HP[i][k] = s;
        }
    }

    /* S = HP * H^T + R   (m x m) */
    double S[EKF_M_MAX][EKF_M_MAX];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; k++) s += HP[i][k] * H[j][k];
            S[i][j] = s + R[i * m + j];
        }
    }

    /* S^-1 */
    double Sinv[EKF_M_MAX][EKF_M_MAX];
    if (invert_mxm(S, m, Sinv) != 0) return;

    /* PHt = P * H^T   (N x m) */
    double PHt[EKF_N][EKF_M_MAX];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < m; j++) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; k++) s += e->P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
    }

    /* K = PHt * Sinv   (N x m) */
    double K[EKF_N][EKF_M_MAX];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < m; j++) {
            double s = 0.0;
            for (int k = 0; k < m; k++) s += PHt[i][k] * Sinv[k][j];
            K[i][j] = s;
        }
    }

    /* x <- x + K y */
    for (int i = 0; i < EKF_N; i++) {
        double s = 0.0;
        for (int j = 0; j < m; j++) s += K[i][j] * y[j];
        e->x[i] += s;
    }
    e->x[2] = normalize_angle(e->x[2]);

    /* Joseph form: P <- (I - K H) P (I - K H)^T + K R K^T */
    double IKH[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            double s = (i == j) ? 1.0 : 0.0;
            for (int k = 0; k < m; k++) s -= K[i][k] * H[k][j];
            IKH[i][j] = s;
        }
    }
    double IKHP[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; k++) s += IKH[i][k] * e->P[k][j];
            IKHP[i][j] = s;
        }
    }
    double term1[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; k++) s += IKHP[i][k] * IKH[j][k];
            term1[i][j] = s;
        }
    }
    double KR[EKF_N][EKF_M_MAX];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < m; j++) {
            double s = 0.0;
            for (int k = 0; k < m; k++) s += K[i][k] * R[k * m + j];
            KR[i][j] = s;
        }
    }
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            double s = 0.0;
            for (int k = 0; k < m; k++) s += KR[i][k] * K[j][k];
            e->P[i][j] = term1[i][j] + s;
        }
    }
}

void ekf_correct_odom_twist(EKF *e,
                            double vx_meas, double vy_meas, double omega_meas,
                            double vx_var,  double vy_var,  double omega_var)
{
    if (!isfinite(vx_meas) || !isfinite(vy_meas) || !isfinite(omega_meas)) return;
    if (fabs(vx_meas)    > e->p.max_odom_vx)    return;
    if (fabs(vy_meas)    > e->p.max_odom_vy)    return;
    if (fabs(omega_meas) > e->p.max_odom_omega) return;

    const double vx_v = safe_variance(vx_var,    e->p.odom_vx_var,    e->p.min_variance);
    const double vy_v = safe_variance(vy_var,    e->p.odom_vy_var,    e->p.min_variance);
    const double w_v  = safe_variance(omega_var, e->p.odom_omega_var, e->p.min_variance);

    const double z[3] = { vx_meas, vy_meas, omega_meas };
    const double H[3][EKF_N] = {
        { 0, 0, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 1, 0 },
        { 0, 0, 0, 0, 0, 1 },
    };
    const double R[9] = {
        vx_v, 0.0,  0.0,
        0.0,  vy_v, 0.0,
        0.0,  0.0,  w_v
    };
    const bool angle_mask[3] = { false, false, false };
    ekf_correct_generic(e, 3, z, H, R, angle_mask);
}

bool ekf_correct_imu_yaw(EKF *e, double yaw_raw, double yaw_var)
{
    if (!e->p.use_imu_yaw)    return false;
    if (!isfinite(yaw_raw))   return false;

    double yaw_used = yaw_raw;
    if (e->p.zero_imu_yaw_on_boot) {
        if (!e->yaw_boot_captured) {
            e->yaw_boot           = yaw_raw;
            e->yaw_boot_captured  = true;
        }
        yaw_used = normalize_angle(yaw_raw - e->yaw_boot);
    }
    const double yaw = normalize_angle(e->p.imu_yaw_sign * yaw_used + e->p.imu_yaw_offset);
    const double innov = normalize_angle(yaw - e->x[2]);
    if (fabs(innov) > e->p.max_imu_yaw_jump) return false;

    const double yaw_v = safe_variance(yaw_var, e->p.imu_yaw_var, e->p.min_variance);
    const double z[1] = { yaw };
    const double H[1][EKF_N] = { { 0, 0, 1, 0, 0, 0 } };
    const double R[1] = { yaw_v };
    const bool angle_mask[1] = { true };
    ekf_correct_generic(e, 1, z, H, R, angle_mask);
    return true;
}

bool ekf_correct_imu_omega(EKF *e, double wz_meas, double wz_var)
{
    if (!e->p.use_imu_omega) return false;
    if (!isfinite(wz_meas))  return false;

    const double wz_corr = e->p.imu_omega_sign * wz_meas;
    const double wz_v    = safe_variance(wz_var, e->p.imu_omega_var, e->p.min_variance);
    const double z[1] = { wz_corr };
    const double H[1][EKF_N] = { { 0, 0, 0, 0, 0, 1 } };
    const double R[1] = { wz_v };
    const bool angle_mask[1] = { false };
    ekf_correct_generic(e, 1, z, H, R, angle_mask);
    return true;
}

void ekf_yaw_to_quaternion(double yaw, float *qx, float *qy, float *qz, float *qw)
{
    const double half = 0.5 * yaw;
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = (float)sin(half);
    *qw = (float)cos(half);
}

/* ROS-style covariance index in a row-major 6x6 (axes x,y,z,roll,pitch,yaw).
 *   IDX(a,b) -> a*6 + b
 */
#define IDX(a, b) ((a) * 6 + (b))

void ekf_fill_pose_covariance(const EKF *e, double cov[36])
{
    for (int i = 0; i < 36; i++) cov[i] = 0.0;

    /* state indices in P:  x=0, y=1, theta=2 → ROS axes 0, 1, 5 */
    cov[IDX(0,0)] = e->P[0][0];
    cov[IDX(0,1)] = e->P[0][1];
    cov[IDX(0,5)] = e->P[0][2];
    cov[IDX(1,0)] = e->P[1][0];
    cov[IDX(1,1)] = e->P[1][1];
    cov[IDX(1,5)] = e->P[1][2];
    cov[IDX(5,0)] = e->P[2][0];
    cov[IDX(5,1)] = e->P[2][1];
    cov[IDX(5,5)] = e->P[2][2];

    /* z, roll, pitch are not observed for a planar base. */
    cov[IDX(2,2)] = 1.0e6;
    cov[IDX(3,3)] = 1.0e6;
    cov[IDX(4,4)] = 1.0e6;
}

void ekf_fill_twist_covariance(const EKF *e, double cov[36])
{
    for (int i = 0; i < 36; i++) cov[i] = 0.0;

    /* state indices: vx=3, vy=4, omega=5 → ROS axes 0, 1, 5 */
    cov[IDX(0,0)] = e->P[3][3];
    cov[IDX(0,1)] = e->P[3][4];
    cov[IDX(0,5)] = e->P[3][5];
    cov[IDX(1,0)] = e->P[4][3];
    cov[IDX(1,1)] = e->P[4][4];
    cov[IDX(1,5)] = e->P[4][5];
    cov[IDX(5,0)] = e->P[5][3];
    cov[IDX(5,1)] = e->P[5][4];
    cov[IDX(5,5)] = e->P[5][5];

    cov[IDX(2,2)] = 1.0e6;
    cov[IDX(3,3)] = 1.0e6;
    cov[IDX(4,4)] = 1.0e6;
}

#undef IDX

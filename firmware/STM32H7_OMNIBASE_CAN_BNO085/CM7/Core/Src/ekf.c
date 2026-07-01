/**
 ******************************************************************************
 * @file    ekf.c
 * @brief   6-state EKF implementation. See ekf.h for state + measurement model.
 ******************************************************************************
 */
#include "ekf.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IDX(r, c)  ((r) * EKF_N + (c))

/* ---- helpers ------------------------------------------------------------ */

static double wrap_pi(double a) {
    while (a >   M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

static void default_params(EKFParams *p) {
    /* Tuned to match the omnibase robot_localization config used in the home2
     * stack (omni_basics.launch.py — no `process_noise_covariance` override,
     * so r_l uses its built-in defaults). r_l adds Q ONCE PER CYCLE at 30 Hz,
     * so its per-second variance injection is (Q_rl * 30). We use a Q*dt
     * convention here: P += q^2 * dt every step, so the per-second injection
     * sums to q^2 — REGARDLESS of our cycle rate. Matching r_l therefore
     * means q^2 = Q_rl * 30, i.e. q = sqrt(Q_rl * 30). This is rate-
     * independent: the same constants are correct at 30 Hz, 50 Hz, 100 Hz,
     * or any other tick the firmware happens to run at.
     *
     *      r_l per-cycle var (diag)   →   per-second var   →   std-dev (q_*)
     *      x,  y    = 0.05             1.5  m^2/s          1.2247  m/sqrt(s)
     *      yaw      = 0.06             1.8  rad^2/s        1.3416  rad/sqrt(s)
     *      vx, vy   = 0.025            0.75 m^2/s^3        0.8660  m/s/sqrt(s)
     *      vyaw     = 0.02             0.6  rad^2/s^3      0.7746  rad/s/sqrt(s)
     *
     * Effect: the filter trusts its propagation less than the previous tight
     * defaults — corrections (wheels + IMU) dominate, which is exactly what
     * robot_localization does today on home2.
     */
    p->sample_time     = 0.010;

    p->q_x     = 1.2247;
    p->q_y     = 1.2247;
    p->q_theta = 1.3416;
    p->q_vx    = 0.8660;
    p->q_vy    = 0.8660;
    p->q_omega = 0.7746;

    p->r_wheel_vx    = 0.30;   /* raised from 0.05 — reduced wheel trust due to one dead motor biasing FK */
    p->r_wheel_vy    = 0.30;
    /* r_wheel_omega is set to a huge std-dev so wheel-derived yaw rate is
     * effectively ignored — matches r_l's `odom0_config[11] = False` and is
     * the right choice for a mecanum base where slip + mechanical play make
     * wheel-only yaw rate untrustworthy. Variance of 1e6 ⇒ Kalman gain on
     * this row collapses to ~0. */
    p->r_wheel_omega = 1000.0;

    p->r_imu_yaw     = 0.03;
    p->r_imu_omega   = 0.03;

    /* r_l defaults to ~1e-9 initial variance (very confident), but with
     * imu_relative on we don't really know yaw until the first IMU sample —
     * a slightly looser start lets the first correction settle without
     * fighting an over-confident prior. */
    p->init_cov_pos   = 0.10;
    p->init_cov_yaw   = 0.10;
    p->init_cov_vel   = 0.20;
    p->init_cov_omega = 0.20;
}

/* Gauss-Jordan inversion of an m x m matrix, m <= EKF_N.
 * Returns 0 on success, -1 if singular (rejects update at the caller). */
static int gj_inverse(double *A, int m) {
    double B[EKF_N * EKF_N];
    memset(B, 0, sizeof(B));
    for (int i = 0; i < m; ++i) B[i * m + i] = 1.0;

    for (int i = 0; i < m; ++i) {
        double maxv = fabs(A[i * m + i]);
        int    piv  = i;
        for (int r = i + 1; r < m; ++r) {
            if (fabs(A[r * m + i]) > maxv) { maxv = fabs(A[r * m + i]); piv = r; }
        }
        if (maxv < 1e-12) return -1;
        if (piv != i) {
            for (int c = 0; c < m; ++c) {
                double t;
                t = A[i * m + c]; A[i * m + c] = A[piv * m + c]; A[piv * m + c] = t;
                t = B[i * m + c]; B[i * m + c] = B[piv * m + c]; B[piv * m + c] = t;
            }
        }
        double inv = 1.0 / A[i * m + i];
        for (int c = 0; c < m; ++c) { A[i * m + c] *= inv; B[i * m + c] *= inv; }
        for (int r = 0; r < m; ++r) {
            if (r == i) continue;
            double f = A[r * m + i];
            if (f == 0.0) continue;
            for (int c = 0; c < m; ++c) {
                A[r * m + c] -= f * A[i * m + c];
                B[r * m + c] -= f * B[i * m + c];
            }
        }
    }
    memcpy(A, B, m * m * sizeof(double));
    return 0;
}

/* ---- init --------------------------------------------------------------- */

void ekf_init(EKF *e, const EKFParams *params) {
    memset(e, 0, sizeof(*e));
    if (params) e->params = *params;
    else        default_params(&e->params);

    const EKFParams *p = &e->params;
    e->P[IDX(0, 0)] = p->init_cov_pos;
    e->P[IDX(1, 1)] = p->init_cov_pos;
    e->P[IDX(2, 2)] = p->init_cov_yaw;
    e->P[IDX(3, 3)] = p->init_cov_vel;
    e->P[IDX(4, 4)] = p->init_cov_vel;
    e->P[IDX(5, 5)] = p->init_cov_omega;

    e->use_imu             = 1;
    e->last_imu_seq        = 0;
    e->imu_relative        = 1;   /* matches home2 omnibase robot_localization */
    e->imu_yaw_initialized = 0;
    e->imu_yaw_offset      = 0.0;
}

/* ---- predict ------------------------------------------------------------ */

void ekf_predict(EKF *e, double dt) {
    if (dt <= 0.0) return;

    const double th = e->x[2], vxb = e->x[3], vyb = e->x[4], w = e->x[5];
    const double c = cos(th), s = sin(th);

    /* Nonlinear state propagation */
    e->x[0] += dt * (c * vxb - s * vyb);
    e->x[1] += dt * (s * vxb + c * vyb);
    e->x[2]  = wrap_pi(e->x[2] + dt * w);
    /* vx_body, vy_body, omega: random-walk (no model update) */

    /* Jacobian F = df/dx evaluated at prior mean */
    static double F[EKF_N * EKF_N];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_N; ++i) F[IDX(i, i)] = 1.0;
    F[IDX(0, 2)] = dt * (-s * vxb - c * vyb);
    F[IDX(0, 3)] = dt *  c;
    F[IDX(0, 4)] = dt * -s;
    F[IDX(1, 2)] = dt * ( c * vxb - s * vyb);
    F[IDX(1, 3)] = dt *  s;
    F[IDX(1, 4)] = dt *  c;
    F[IDX(2, 5)] = dt;

    /* P = F P F^T + Q*dt */
    static double FP[EKF_N * EKF_N];
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s2 = 0.0;
            for (int k = 0; k < EKF_N; ++k) s2 += F[IDX(i, k)] * e->P[IDX(k, j)];
            FP[IDX(i, j)] = s2;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s2 = 0.0;
            for (int k = 0; k < EKF_N; ++k) s2 += FP[IDX(i, k)] * F[IDX(j, k)];
            e->P[IDX(i, j)] = s2;
        }

    const EKFParams *p = &e->params;
    e->P[IDX(0, 0)] += p->q_x     * p->q_x     * dt;
    e->P[IDX(1, 1)] += p->q_y     * p->q_y     * dt;
    e->P[IDX(2, 2)] += p->q_theta * p->q_theta * dt;
    e->P[IDX(3, 3)] += p->q_vx    * p->q_vx    * dt;
    e->P[IDX(4, 4)] += p->q_vy    * p->q_vy    * dt;
    e->P[IDX(5, 5)] += p->q_omega * p->q_omega * dt;
}

/* ---- generic correction (Joseph form) ----------------------------------- */

/*  Performs one EKF correction step.
 *    z[m]              measurement
 *    h[m]              predicted measurement (always linear here, since
 *                      every observation in this filter directly samples a
 *                      single state, so h(x) = H * x)
 *    H[m * EKF_N]      measurement Jacobian, row-major
 *    R[m * m]          measurement covariance, row-major
 *    angle_indices[]   indices into z/y that must be angle-wrapped (-pi,pi]
 *
 *  Uses module-level static scratch — safe because the EKF is only stepped
 *  from a single FreeRTOS task.
 */
static void correct(EKF *e,
                    const double *z, const double *h,
                    const double *H, const double *R,
                    int m,
                    const int *angle_indices, int n_angle)
{
    static double y[EKF_N];                  /* innovation */
    static double HP[EKF_N * EKF_N];
    static double S[EKF_N * EKF_N];
    static double PHt[EKF_N * EKF_N];
    static double K[EKF_N * EKF_N];
    static double IKH[EKF_N * EKF_N];
    static double T1[EKF_N * EKF_N];

    /* y = z - h */
    for (int i = 0; i < m; ++i) y[i] = z[i] - h[i];
    for (int k = 0; k < n_angle; ++k) y[angle_indices[k]] = wrap_pi(y[angle_indices[k]]);

    /* HP = H * P  (m x N) */
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += H[i * EKF_N + k] * e->P[IDX(k, j)];
            HP[i * EKF_N + j] = s;
        }
    /* S = H P H^T + R   (m x m) */
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += HP[i * EKF_N + k] * H[j * EKF_N + k];
            S[i * m + j] = s + R[i * m + j];
        }
    if (gj_inverse(S, m) != 0) return;

    /* PHt = P H^T   (N x m) */
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += e->P[IDX(i, k)] * H[j * EKF_N + k];
            PHt[i * m + j] = s;
        }
    /* K = PHt * S_inv  (N x m) */
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0.0;
            for (int k = 0; k < m; ++k) s += PHt[i * m + k] * S[k * m + j];
            K[i * m + j] = s;
        }

    /* x += K * y */
    for (int i = 0; i < EKF_N; ++i) {
        double s = 0.0;
        for (int j = 0; j < m; ++j) s += K[i * m + j] * y[j];
        e->x[i] += s;
    }
    e->x[2] = wrap_pi(e->x[2]);

    /* Joseph form: P = (I-KH) P (I-KH)^T + K R K^T */
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = (i == j) ? 1.0 : 0.0;
            for (int k = 0; k < m; ++k) s -= K[i * m + k] * H[k * EKF_N + j];
            IKH[IDX(i, j)] = s;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += IKH[IDX(i, k)] * e->P[IDX(k, j)];
            T1[IDX(i, j)] = s;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += T1[IDX(i, k)] * IKH[IDX(j, k)];
            e->P[IDX(i, j)] = s;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int a = 0; a < m; ++a) {
                double Kia = K[i * m + a];
                for (int b = 0; b < m; ++b)
                    s += Kia * R[a * m + b] * K[j * m + b];
            }
            e->P[IDX(i, j)] += s;
        }
}

/* ---- observation-specific corrections ----------------------------------- */

void ekf_correct_wheel_twist(EKF *e, double vx_body, double vy_body, double omega) {
    /* H selects states [3, 4, 5] = [vx_body, vy_body, omega]. */
    static double H[3 * EKF_N];
    static double R[3 * 3];
    memset(H, 0, sizeof(H));
    H[0 * EKF_N + 3] = 1.0;
    H[1 * EKF_N + 4] = 1.0;
    H[2 * EKF_N + 5] = 1.0;

    const EKFParams *p = &e->params;
    memset(R, 0, sizeof(R));
    R[0 * 3 + 0] = p->r_wheel_vx    * p->r_wheel_vx;
    R[1 * 3 + 1] = p->r_wheel_vy    * p->r_wheel_vy;
    R[2 * 3 + 2] = p->r_wheel_omega * p->r_wheel_omega;

    double z[3] = { vx_body, vy_body, omega };
    double h[3] = { e->x[3],  e->x[4],  e->x[5] };
    correct(e, z, h, H, R, 3, NULL, 0);
}

int ekf_correct_imu(EKF *e, double yaw, double omega_z, uint32_t seq) {
    if (!e->use_imu)               return 0;
    if (seq == e->last_imu_seq)    return 0;  /* same sample as last time */

    /* `imu_relative` mirrors robot_localization's `imu0_relative: true`.
     * On the first IMU sample we latch the current yaw as the offset; from
     * then on, the EKF sees (yaw - offset). The very first correction has
     * effective_yaw == 0 == prior x[2], so the innovation is zero and the
     * step is a no-op for the mean (but covariance still tightens). */
    if (e->imu_relative && !e->imu_yaw_initialized) {
        e->imu_yaw_offset      = yaw;
        e->imu_yaw_initialized = 1;
    }
    const double effective_yaw = e->imu_relative
        ? (yaw - e->imu_yaw_offset)
        : yaw;

    /* Outlier rejection — ported from the home2 omnidriver dashboard, which
     * dropped whole UART lines if |Δyaw| between consecutive 50 Hz samples
     * exceeded 40 deg (~0.70 rad). The threshold corresponds to ~2000 deg/s
     * angular velocity which a mecanum base cannot physically reach; an
     * apparent jump that large is almost certainly a corrupted SH2 packet,
     * a transient I2C glitch, or a quaternion wrap-around the BNO085 hasn't
     * smoothed yet. Letting one through "flashes" the EKF heading and takes
     * ~1 s to recover; cheaper to skip the correction this cycle. We only
     * arm the check once the EKF has a real yaw estimate (after the first
     * accepted sample), so legitimate boot offsets don't trigger it. */
    if (e->imu_yaw_initialized) {
        double dy = effective_yaw - e->x[2];
        while (dy >   M_PI) dy -= 2.0 * M_PI;
        while (dy <= -M_PI) dy += 2.0 * M_PI;
        if (fabs(dy) > 0.70) {
            /* Advance the sequence so we don't re-evaluate this same bad
             * sample on the next tick — the next fresh BNO085 reading
             * (seq+2) gets a clean chance. */
            e->last_imu_seq = seq;
            return 0;
        }
    }

    /* Yaw (angle-wrapped) — 1-dim */
    {
        static double H[1 * EKF_N];
        memset(H, 0, sizeof(H));
        H[2] = 1.0;
        double R[1] = { e->params.r_imu_yaw * e->params.r_imu_yaw };
        double z[1] = { effective_yaw };
        double h[1] = { e->x[2] };
        int    ang[1] = { 0 };
        correct(e, z, h, H, R, 1, ang, 1);
    }
    /* Omega — 1-dim */
    {
        static double H[1 * EKF_N];
        memset(H, 0, sizeof(H));
        H[5] = 1.0;
        double R[1] = { e->params.r_imu_omega * e->params.r_imu_omega };
        double z[1] = { omega_z };
        double h[1] = { e->x[5] };
        correct(e, z, h, H, R, 1, NULL, 0);
    }

    e->last_imu_seq = seq;
    return 1;
}

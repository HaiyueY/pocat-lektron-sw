/**
 * @file adcs_igrf.c
 * @brief IGRF-14 geomagnetic field model with selectable truncation degree.
 *
 * Potential (geocentric r, colatitude θ, longitude φ, a = 6371.2 km):
 *
 *   V = a Σ_{n=1}^{N} (a/r)^{n+1} Σ_{m=0}^{n} (g_nm cos mφ + h_nm sin mφ) P_n^m(cos θ)
 *
 *   B_r = −∂V/∂r            =  Σ (n+1) (a/r)^{n+2} (g cos + h sin) P
 *   B_θ = −(1/r) ∂V/∂θ      = −Σ (a/r)^{n+2} (g cos + h sin) dP/dθ
 *   B_φ = −1/(r sinθ) ∂V/∂φ =  Σ (a/r)^{n+2} m (g sin − h cos) P / sinθ
 *
 * P_n^m are Schmidt semi-normalised. They are generated column by column
 * (outer loop over m, inner loop over n) with the three-term recursion
 *
 *   P_n^m = [(2n−1) cosθ P_{n−1}^m − √((n−1)²−m²) P_{n−2}^m] / √(n²−m²)
 *   P_m^m = √((2m−1)/(2m)) sinθ P_{m−1}^{m−1}   (m ≥ 2),  P_1^1 = sinθ
 *
 * so only a handful of scalars are live at any time: stack use does not
 * depend on the degree and no (N+1)² table is needed (the reference
 * igrf13syn kept 29 KB on the stack). The square-root factors come from
 * tables generated with the coefficients, so a term costs only
 * multiply/adds — no sqrt or division in soft double.
 *
 * Near the poles sinθ is floored at 1e-10. Every m ≥ 1 term of B_φ carries
 * a factor sin^m θ, so the ratio stays exact; only an exact pole would
 * otherwise divide 0 by 0.
 */

#include "adcs_igrf.h"
#include "adcs_igrf14_coeffs.h"
#include <math.h>

/** Smallest |r| accepted [km]: rejects zero/garbage positions */
#define IGRF_MIN_RADIUS_KM   1000.0

/** Floor on sinθ near the poles (see file header) */
#define IGRF_MIN_SIN_THETA   1.0e-10

/** Packed coefficient index for degree n ≥ 1, order 0 ≤ m ≤ n */
#define IGRF_INDEX(n, m)     ((n) * ((n) + 1) / 2 + (m) - 1)

int adcs_igrf_field_ecef(vec3d_t r_ecef_m, double decimal_year, int n_max,
                         vec3d_t *b_ecef_t)
{
    if (b_ecef_t == 0 || n_max < 1 || n_max > IGRF14_NMAX) {
        return ADCS_IGRF_ERR_INVALID;
    }

    const double x = r_ecef_m.x * 1.0e-3;
    const double y = r_ecef_m.y * 1.0e-3;
    const double z = r_ecef_m.z * 1.0e-3;
    const double rho = sqrt(x * x + y * y);
    const double r = sqrt(rho * rho + z * z);

    /* Also rejects NaN */
    if (!(r >= IGRF_MIN_RADIUS_KM)) {
        return ADCS_IGRF_ERR_INVALID;
    }

    const double cos_t = z / r;
    double sin_t = rho / r;
    if (sin_t < IGRF_MIN_SIN_THETA) {
        sin_t = IGRF_MIN_SIN_THETA;
    }
    const double phi = atan2(y, x);
    const double cos_phi = cos(phi);
    const double sin_phi = sin(phi);
    const double ar = ADCS_IGRF_REF_RADIUS_KM / r;
    const double dt = decimal_year - IGRF14_EPOCH;

    double b_r = 0.0, b_t = 0.0, b_p = 0.0;

    /* Diagonal terms P_m^m, dP_m^m/dθ, carried from one m to the next */
    double p_mm = 1.0, dp_mm = 0.0;
    /* cos(mφ), sin(mφ) by angle addition */
    double cos_mphi = 1.0, sin_mphi = 0.0;
    /* (a/r)^(m+2) */
    double ar_m = ar * ar;

    for (int m = 0; m <= n_max; m++) {
        if (m >= 1) {
            double f = igrf14_pmm[m];
            double p_new  = f * sin_t * p_mm;
            double dp_new = f * (cos_t * p_mm + sin_t * dp_mm);
            p_mm = p_new;
            dp_mm = dp_new;

            double c = cos_mphi * cos_phi - sin_mphi * sin_phi;
            double s = sin_mphi * cos_phi + cos_mphi * sin_phi;
            cos_mphi = c;
            sin_mphi = s;

            ar_m *= ar;
        }

        /* P_{n-1}^m, P_{n-2}^m and their θ-derivatives */
        double p1 = 0.0, p2 = 0.0, dp1 = 0.0, dp2 = 0.0;
        double p = p_mm, dp = dp_mm;
        double ar_n = ar_m;   /* (a/r)^(n+2) */

        for (int n = m; n <= n_max; n++) {
            if (n > m) {
                int kr = IGRF_INDEX(n, m);
                double k1 = 2.0 * n - 1.0;
                double k2 = igrf14_k2[kr];
                double k3 = igrf14_k3[kr];
                p  = (k1 * cos_t * p1 - k2 * p2) * k3;
                dp = (k1 * (cos_t * dp1 - sin_t * p1) - k2 * dp2) * k3;
                ar_n *= ar;
            }
            p2 = p1;
            dp2 = dp1;
            p1 = p;
            dp1 = dp;

            if (n == 0) {
                continue;
            }

            int k = IGRF_INDEX(n, m);
            double g = igrf14_g[k];
            double h = igrf14_h[k];
            if (n <= IGRF14_SV_NMAX) {
                g += igrf14_gsv[k] * dt;
                h += igrf14_hsv[k] * dt;
            }

            double gc_hs = g * cos_mphi + h * sin_mphi;
            double gs_hc = g * sin_mphi - h * cos_mphi;

            b_r += (double)(n + 1) * ar_n * gc_hs * p;
            b_t -= ar_n * gc_hs * dp;
            b_p += ar_n * (double)m * gs_hc * p;
        }
    }
    b_p /= sin_t;

    /* Local (r̂, θ̂, φ̂) → Earth-fixed Cartesian, nT → T */
    const double sin_t_cos_p = sin_t * cos_phi;
    const double sin_t_sin_p = sin_t * sin_phi;
    const double cos_t_cos_p = cos_t * cos_phi;
    const double cos_t_sin_p = cos_t * sin_phi;

    b_ecef_t->x = (b_r * sin_t_cos_p + b_t * cos_t_cos_p - b_p * sin_phi) * 1.0e-9;
    b_ecef_t->y = (b_r * sin_t_sin_p + b_t * cos_t_sin_p + b_p * cos_phi) * 1.0e-9;
    b_ecef_t->z = (b_r * cos_t - b_t * sin_t) * 1.0e-9;

    if (decimal_year < IGRF14_EPOCH || decimal_year > IGRF14_VALID_END) {
        return ADCS_IGRF_EXTRAPOLATED;
    }
    return ADCS_IGRF_OK;
}

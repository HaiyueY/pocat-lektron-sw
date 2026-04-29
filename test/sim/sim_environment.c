/**
 * @file sim_environment.c
 * @brief Simulated orbital environment for ADCS testing.
 *
 * Provides a circular polar orbit at ~400 km altitude.
 * Magnetic field uses IGRF tilted dipole model (matching MATLAB BDipole.m).
 * Sun direction is fixed in ECI (simplified).
 * Eclipse occurs when the satellite is in Earth's shadow.
 */

#include "sim_environment.h"
#include "adcs_config.h"
#include <math.h>

/** Earth radius [m] */
#define EARTH_RADIUS    6371000.0

/** Earth magnetic reference radius [m] (IGRF) */
#define EARTH_A_MAG     6371200.0

/** Orbit altitude [m] */
#define ORBIT_ALT       400000.0

/** Orbital radius [m] */
#define ORBIT_RADIUS    (EARTH_RADIUS + ORBIT_ALT)

/** Orbital period [s] (~92 min at 400 km) */
#define ORBIT_PERIOD    5540.0

/** Orbital angular rate [rad/s] */
#define ORBIT_OMEGA     (2.0 * M_PI / ORBIT_PERIOD)

/** Earth rotation rate [rad/s] */
#define EARTH_ROT_RATE  7.292115e-5

/* -----------------------------------------------------------------------
 * IGRF Tilted Dipole Model (from BDipole.m)
 * Coefficients: 1995 IGRF extrapolated to ~2024 (dJD ≈ 29 years)
 *   g10 = -29002.4 nT, g11 = -1459.8 nT, h11 = 4906.4 nT
 *   h0  = sqrt(g10²+g11²+h11²) ≈ 29451.5 nT
 *   thetaM ≈ 170° (dipole colatitude), phiM ≈ 107° (dipole longitude)
 * ----------------------------------------------------------------------- */

/* Dipole unit vector in ECEF frame (precomputed) */
static const double dipole_ef[3] = {-0.05009, 0.16664, -0.98474};

/* aCuH = a³ · h0 · 1e-9  [m³·T] */
static const double aCuH = 2.5853e20 * 29451.5 * 1.0e-9;  /* 7.615e15 */

/**
 * Compute B-field in ECI using tilted dipole model.
 * Matches MATLAB BDipole.m: B_ecef = (aCuH/r³)(3(m·û)û - m),
 * then transform ECEF→ECI via Earth rotation.
 */
static void compute_bfield(vec3d_t r_eci, double time_s, vec3d_t *b_eci)
{
    double gmst = EARTH_ROT_RATE * time_s;
    double cg = cos(gmst), sg = sin(gmst);

    /* ECI → ECEF rotation about Z */
    double rx =  cg * r_eci.x + sg * r_eci.y;
    double ry = -sg * r_eci.x + cg * r_eci.y;
    double rz =  r_eci.z;

    double r_mag = sqrt(rx*rx + ry*ry + rz*rz);
    double inv_r = 1.0 / r_mag;
    double ux = rx * inv_r, uy = ry * inv_r, uz = rz * inv_r;

    double m_dot_u = dipole_ef[0]*ux + dipole_ef[1]*uy + dipole_ef[2]*uz;
    double scale = aCuH * inv_r * inv_r * inv_r;

    double bx = scale * (3.0 * m_dot_u * ux - dipole_ef[0]);
    double by = scale * (3.0 * m_dot_u * uy - dipole_ef[1]);
    double bz = scale * (3.0 * m_dot_u * uz - dipole_ef[2]);

    /* ECEF → ECI */
    b_eci->x = cg * bx - sg * by;
    b_eci->y = sg * bx + cg * by;
    b_eci->z = bz;
}

void sim_env_init(sim_env_t *env)
{
    env->time_s = 0.0;

    /* Initial position: ascending node, polar orbit in X-Z plane */
    env->pos_eci.x = ORBIT_RADIUS;
    env->pos_eci.y = 0.0;
    env->pos_eci.z = 0.0;

    env->vel_eci.x = 0.0;
    env->vel_eci.y = 0.0;
    env->vel_eci.z = sqrt(3.986e14 / ORBIT_RADIUS);

    /* Sun direction (fixed for simplicity) */
    env->sun_eci.x = 1.0;
    env->sun_eci.y = 0.0;
    env->sun_eci.z = 0.0;

    /* Compute initial B-field and nadir */
    compute_bfield(env->pos_eci, 0.0, &env->b_field_eci);

    double r = sqrt(env->pos_eci.x * env->pos_eci.x +
                    env->pos_eci.y * env->pos_eci.y +
                    env->pos_eci.z * env->pos_eci.z);
    env->nadir_eci.x = -env->pos_eci.x / r;
    env->nadir_eci.y = -env->pos_eci.y / r;
    env->nadir_eci.z = -env->pos_eci.z / r;

    env->eclipse = 0;
}

void sim_env_step(sim_env_t *env, double dt)
{
    env->time_s += dt;
    double theta = ORBIT_OMEGA * env->time_s;

    /* Circular polar orbit propagation */
    env->pos_eci.x = ORBIT_RADIUS * cos(theta);
    env->pos_eci.y = 0.0;
    env->pos_eci.z = ORBIT_RADIUS * sin(theta);

    env->vel_eci.x = -ORBIT_RADIUS * ORBIT_OMEGA * sin(theta);
    env->vel_eci.y = 0.0;
    env->vel_eci.z = ORBIT_RADIUS * ORBIT_OMEGA * cos(theta);

    /* IGRF tilted dipole B-field */
    compute_bfield(env->pos_eci, env->time_s, &env->b_field_eci);

    /* Nadir direction */
    double r = sqrt(env->pos_eci.x * env->pos_eci.x +
                    env->pos_eci.y * env->pos_eci.y +
                    env->pos_eci.z * env->pos_eci.z);
    env->nadir_eci.x = -env->pos_eci.x / r;
    env->nadir_eci.y = -env->pos_eci.y / r;
    env->nadir_eci.z = -env->pos_eci.z / r;

    /* Eclipse: satellite behind Earth relative to sun */
    double dot_ps = env->pos_eci.x * env->sun_eci.x +
                    env->pos_eci.y * env->sun_eci.y +
                    env->pos_eci.z * env->sun_eci.z;

    if (dot_ps < 0.0) {
        double perp_dist_sq = (r * r) - (dot_ps * dot_ps);
        env->eclipse = (perp_dist_sq < EARTH_RADIUS * EARTH_RADIUS) ? 1 : 0;
    } else {
        env->eclipse = 0;
    }
}

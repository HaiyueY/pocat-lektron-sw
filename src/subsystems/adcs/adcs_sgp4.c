/**
 * @file adcs_sgp4.c
 * @brief Near-Earth SGP4 orbit propagator (Vallado revision, WGS-72).
 *
 * Port of initl(), the near-Earth part of sgp4init() and sgp4() from
 * Vallado's SGP4.cpp (AIAA 2006-6753, CelesTrak distribution, 2021).
 * Statement order and floating-point expressions are kept as in the
 * reference so that results match the published verification output
 * (SGP4-VER.TLE / tcppver.out) to round-off level. Variable names follow
 * the reference; the comments marked "sgp4fix" are Vallado's.
 */

#include "adcs_sgp4.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- WGS-72 gravity constants (getgravconst, case wgs72) ---- */
#define SGP4_MU             398600.8              /* [km^3/s^2] */
#define SGP4_RADIUS_KM      6378.135              /* [km] */
#define SGP4_J2             0.001082616
#define SGP4_J3             (-0.00000253881)
#define SGP4_J4             (-0.00000165597)
#define SGP4_J3OJ2          (SGP4_J3 / SGP4_J2)

/** Deep-space threshold on the (un-Kozai'd) period [min] */
#define SGP4_DEEP_SPACE_PERIOD_MIN  225.0

/** sqrt(GM) in earth radii^1.5 per minute */
static double sgp4_xke(void)
{
    return 60.0 / sqrt(SGP4_RADIUS_KM * SGP4_RADIUS_KM * SGP4_RADIUS_KM / SGP4_MU);
}

int adcs_sgp4_init(const adcs_tle_t *tle, adcs_sgp4_t *rec)
{
    const double deg2rad = M_PI / 180.0;
    const double xpdotp = 1440.0 / (2.0 * M_PI);   /* [rev/day]/[rad/min] */
    const double temp4 = 1.5e-12;
    const double x2o3 = 2.0 / 3.0;
    const double xke = sgp4_xke();
    const double j2 = SGP4_J2;
    const double j4 = SGP4_J4;
    const double j3oj2 = SGP4_J3OJ2;
    const double radiusearthkm = SGP4_RADIUS_KM;

    double ak, d1, del, adel, ao, po, con42, cosio, cosio2, eccsq, omeosq,
           posq, rp, rteosq, sinio, ss, qzms2t, qzms2ttemp, sfour, qzms24,
           qzms24temp, perige, pinvsq, tsi, etasq, eeta, psisq, coef, coef1,
           cc2, cc3, cosio4, temp1, temp2, temp3, xhdot1, delmotemp, cc1sq,
           temp, no_kozai;
    adcs_sgp4_t s = {0};

    if (tle == 0 || rec == 0 || tle->mean_motion <= 0.0 ||
        tle->ecc < 0.0 || tle->ecc >= 1.0) {
        return ADCS_SGP4_ERR_INVALID;
    }

    /* ---- twoline2rv: TLE units → SGP4 units ---- */
    no_kozai = tle->mean_motion / xpdotp;            /* [rad/min] */
    s.bstar = tle->bstar;
    s.ecco  = tle->ecc;
    s.inclo = tle->incl_deg * deg2rad;
    s.nodeo = tle->raan_deg * deg2rad;
    s.argpo = tle->argp_deg * deg2rad;
    s.mo    = tle->mean_anom_deg * deg2rad;

    ss = 78.0 / radiusearthkm + 1.0;
    /* sgp4fix use multiply for speed instead of pow */
    qzms2ttemp = (120.0 - 78.0) / radiusearthkm;
    qzms2t = qzms2ttemp * qzms2ttemp * qzms2ttemp * qzms2ttemp;

    /* ---- initl ---- */
    eccsq  = s.ecco * s.ecco;
    omeosq = 1.0 - eccsq;
    rteosq = sqrt(omeosq);
    cosio  = cos(s.inclo);
    cosio2 = cosio * cosio;

    /* un-kozai the mean motion */
    ak   = pow(xke / no_kozai, x2o3);
    d1   = 0.75 * j2 * (3.0 * cosio2 - 1.0) / (rteosq * omeosq);
    del  = d1 / (ak * ak);
    adel = ak * (1.0 - del * del - del *
           (1.0 / 3.0 + 134.0 * del * del / 81.0));
    del  = d1 / (adel * adel);
    s.no_unkozai = no_kozai / (1.0 + del);

    ao     = pow(xke / s.no_unkozai, x2o3);
    sinio  = sin(s.inclo);
    po     = ao * omeosq;
    con42  = 1.0 - 5.0 * cosio2;
    s.con41 = -con42 - cosio2 - cosio2;
    posq   = po * po;
    rp     = ao * (1.0 - s.ecco);

    /* ---- deep space is not supported ---- */
    if ((2.0 * M_PI / s.no_unkozai) >= SGP4_DEEP_SPACE_PERIOD_MIN) {
        return ADCS_SGP4_ERR_DEEP_SPACE;
    }

    /* ---- sgp4init, near-Earth part ---- */
    if ((omeosq >= 0.0) || (s.no_unkozai >= 0.0)) {
        s.isimp = 0;
        if (rp < (220.0 / radiusearthkm + 1.0)) {
            s.isimp = 1;
        }
        sfour  = ss;
        qzms24 = qzms2t;
        perige = (rp - 1.0) * radiusearthkm;

        /* for perigees below 156 km, s and qoms2t are altered */
        if (perige < 156.0) {
            sfour = perige - 78.0;
            if (perige < 98.0) {
                sfour = 20.0;
            }
            /* sgp4fix use multiply for speed instead of pow */
            qzms24temp = (120.0 - sfour) / radiusearthkm;
            qzms24 = qzms24temp * qzms24temp * qzms24temp * qzms24temp;
            sfour = sfour / radiusearthkm + 1.0;
        }
        pinvsq = 1.0 / posq;

        tsi   = 1.0 / (ao - sfour);
        s.eta = ao * s.ecco * tsi;
        etasq = s.eta * s.eta;
        eeta  = s.ecco * s.eta;
        psisq = fabs(1.0 - etasq);
        coef  = qzms24 * pow(tsi, 4.0);
        coef1 = coef / pow(psisq, 3.5);
        cc2   = coef1 * s.no_unkozai * (ao * (1.0 + 1.5 * etasq + eeta *
                (4.0 + etasq)) + 0.375 * j2 * tsi / psisq * s.con41 *
                (8.0 + 3.0 * etasq * (8.0 + etasq)));
        s.cc1 = s.bstar * cc2;
        cc3   = 0.0;
        if (s.ecco > 1.0e-4) {
            cc3 = -2.0 * coef * tsi * j3oj2 * s.no_unkozai * sinio / s.ecco;
        }
        s.x1mth2 = 1.0 - cosio2;
        s.cc4 = 2.0 * s.no_unkozai * coef1 * ao * omeosq *
                (s.eta * (2.0 + 0.5 * etasq) + s.ecco *
                (0.5 + 2.0 * etasq) - j2 * tsi / (ao * psisq) *
                (-3.0 * s.con41 * (1.0 - 2.0 * eeta + etasq *
                (1.5 - 0.5 * eeta)) + 0.75 * s.x1mth2 *
                (2.0 * etasq - eeta * (1.0 + etasq)) * cos(2.0 * s.argpo)));
        s.cc5 = 2.0 * coef1 * ao * omeosq * (1.0 + 2.75 *
                (etasq + eeta) + eeta * etasq);
        cosio4 = cosio2 * cosio2;
        temp1  = 1.5 * j2 * pinvsq * s.no_unkozai;
        temp2  = 0.5 * temp1 * j2 * pinvsq;
        temp3  = -0.46875 * j4 * pinvsq * pinvsq * s.no_unkozai;
        s.mdot = s.no_unkozai + 0.5 * temp1 * rteosq * s.con41 + 0.0625 *
                 temp2 * rteosq * (13.0 - 78.0 * cosio2 + 137.0 * cosio4);
        s.argpdot = -0.5 * temp1 * con42 + 0.0625 * temp2 *
                    (7.0 - 114.0 * cosio2 + 395.0 * cosio4) +
                    temp3 * (3.0 - 36.0 * cosio2 + 49.0 * cosio4);
        xhdot1 = -temp1 * cosio;
        s.nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * cosio2) +
                    2.0 * temp3 * (3.0 - 7.0 * cosio2)) * cosio;
        s.omgcof = s.bstar * cc3 * cos(s.argpo);
        s.xmcof = 0.0;
        if (s.ecco > 1.0e-4) {
            s.xmcof = -x2o3 * coef * s.bstar / eeta;
        }
        s.nodecf = 3.5 * omeosq * xhdot1 * s.cc1;
        s.t2cof  = 1.5 * s.cc1;
        /* sgp4fix for divide by zero with xinco = 180 deg */
        if (fabs(cosio + 1.0) > 1.5e-12) {
            s.xlcof = -0.25 * j3oj2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
        } else {
            s.xlcof = -0.25 * j3oj2 * sinio * (3.0 + 5.0 * cosio) / temp4;
        }
        s.aycof = -0.5 * j3oj2 * sinio;
        /* sgp4fix use multiply for speed instead of pow */
        delmotemp = 1.0 + s.eta * cos(s.mo);
        s.delmo  = delmotemp * delmotemp * delmotemp;
        s.sinmao = sin(s.mo);
        s.x7thm1 = 7.0 * cosio2 - 1.0;

        if (s.isimp != 1) {
            cc1sq   = s.cc1 * s.cc1;
            s.d2    = 4.0 * ao * tsi * cc1sq;
            temp    = s.d2 * tsi * s.cc1 / 3.0;
            s.d3    = (17.0 * ao + sfour) * temp;
            s.d4    = 0.5 * temp * ao * tsi * (221.0 * ao + 31.0 * sfour) * s.cc1;
            s.t3cof = s.d2 + 2.0 * cc1sq;
            s.t4cof = 0.25 * (3.0 * s.d3 + s.cc1 * (12.0 * s.d2 + 10.0 * cc1sq));
            s.t5cof = 0.2 * (3.0 * s.d4 + 12.0 * s.cc1 * s.d3 +
                      6.0 * s.d2 * s.d2 + 15.0 * cc1sq * (2.0 * s.d2 + cc1sq));
        }
    }

    *rec = s;
    return ADCS_SGP4_OK;
}

int adcs_sgp4_propagate(const adcs_sgp4_t *rec, double tsince_min,
                        double r_km[3], double v_kms[3])
{
    const double twopi = 2.0 * M_PI;
    const double x2o3 = 2.0 / 3.0;
    const double xke = sgp4_xke();
    const double j2 = SGP4_J2;
    const double radiusearthkm = SGP4_RADIUS_KM;
    const double vkmpersec = radiusearthkm * xke / 60.0;

    double am, axnl, aynl, betal, cosim, cos2u, coseo1 = 0.0, cosi, cosip,
           cossu, cosu, delm, delomg, em, ecose, el2, eo1, ep, esine,
           argpm, argpp, argpdf, pl, mrt, mvt, rdotl, rl, rvdot, rvdotl,
           sinim, sin2u, sineo1 = 0.0, sini, sinip, sinsu, sinu, snod, cnod,
           su, t, t2, t3, t4, tem5, temp, temp1, temp2, tempa, tempe, templ,
           u, ux, uy, uz, vx, vy, vz, inclm, mm, nm, nodem, xinc, xincp, xl,
           xlm, mp, xmdf, xmx, xmy, nodedf, xnode, nodep, delmtemp;
    int ktr;

    if (rec == 0 || r_km == 0 || v_kms == 0) {
        return ADCS_SGP4_ERR_INVALID;
    }

    t = tsince_min;

    /* ---- update for secular gravity and atmospheric drag ---- */
    xmdf   = rec->mo + rec->mdot * t;
    argpdf = rec->argpo + rec->argpdot * t;
    nodedf = rec->nodeo + rec->nodedot * t;
    argpm  = argpdf;
    mm     = xmdf;
    t2     = t * t;
    nodem  = nodedf + rec->nodecf * t2;
    tempa  = 1.0 - rec->cc1 * t;
    tempe  = rec->bstar * rec->cc4 * t;
    templ  = rec->t2cof * t2;

    if (rec->isimp != 1) {
        delomg = rec->omgcof * t;
        /* sgp4fix use mutliply for speed instead of pow */
        delmtemp = 1.0 + rec->eta * cos(xmdf);
        delm  = rec->xmcof * (delmtemp * delmtemp * delmtemp - rec->delmo);
        temp  = delomg + delm;
        mm    = xmdf + temp;
        argpm = argpdf - temp;
        t3    = t2 * t;
        t4    = t3 * t;
        tempa = tempa - rec->d2 * t2 - rec->d3 * t3 - rec->d4 * t4;
        tempe = tempe + rec->bstar * rec->cc5 * (sin(mm) - rec->sinmao);
        templ = templ + rec->t3cof * t3 + t4 * (rec->t4cof + t * rec->t5cof);
    }

    nm    = rec->no_unkozai;
    em    = rec->ecco;
    inclm = rec->inclo;

    if (nm <= 0.0) {
        return ADCS_SGP4_ERR_MEAN_MOTION;
    }
    am = pow((xke / nm), x2o3) * tempa * tempa;
    nm = xke / pow(am, 1.5);
    em = em - tempe;

    /* fix tolerance for error recognition */
    if ((em >= 1.0) || (em < -0.001)) {
        return ADCS_SGP4_ERR_ECC;
    }
    /* sgp4fix fix tolerance to avoid a divide by zero */
    if (em < 1.0e-6) {
        em = 1.0e-6;
    }
    mm   = mm + rec->no_unkozai * templ;
    xlm  = mm + argpm + nodem;

    nodem = fmod(nodem, twopi);
    argpm = fmod(argpm, twopi);
    xlm   = fmod(xlm, twopi);
    mm    = fmod(xlm - argpm - nodem, twopi);

    /* ---- compute extra mean quantities ---- */
    sinim = sin(inclm);
    cosim = cos(inclm);

    /* ---- (no lunar-solar periodics for near-Earth) ---- */
    ep    = em;
    xincp = inclm;
    argpp = argpm;
    nodep = nodem;
    mp    = mm;
    sinip = sinim;
    cosip = cosim;

    /* ---- long period periodics ---- */
    axnl = ep * cos(argpp);
    temp = 1.0 / (am * (1.0 - ep * ep));
    aynl = ep * sin(argpp) + temp * rec->aycof;
    xl   = mp + argpp + nodep + temp * rec->xlcof * axnl;

    /* ---- solve kepler's equation ---- */
    u    = fmod(xl - nodep, twopi);
    eo1  = u;
    tem5 = 9999.9;
    ktr  = 1;
    /* sgp4fix for kepler iteration: the iteration needs better limits on corrections */
    while ((fabs(tem5) >= 1.0e-12) && (ktr <= 10)) {
        sineo1 = sin(eo1);
        coseo1 = cos(eo1);
        tem5   = 1.0 - coseo1 * axnl - sineo1 * aynl;
        tem5   = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
        if (fabs(tem5) >= 0.95) {
            tem5 = tem5 > 0.0 ? 0.95 : -0.95;
        }
        eo1 = eo1 + tem5;
        ktr = ktr + 1;
    }

    /* ---- short period preliminary quantities ---- */
    ecose = axnl * coseo1 + aynl * sineo1;
    esine = axnl * sineo1 - aynl * coseo1;
    el2   = axnl * axnl + aynl * aynl;
    pl    = am * (1.0 - el2);
    if (pl < 0.0) {
        return ADCS_SGP4_ERR_SEMILATUS;
    }

    rl     = am * (1.0 - ecose);
    rdotl  = sqrt(am) * esine / rl;
    rvdotl = sqrt(pl) / rl;
    betal  = sqrt(1.0 - el2);
    temp   = esine / (1.0 + betal);
    sinu   = am / rl * (sineo1 - aynl - axnl * temp);
    cosu   = am / rl * (coseo1 - axnl + aynl * temp);
    su     = atan2(sinu, cosu);
    sin2u  = (cosu + cosu) * sinu;
    cos2u  = 1.0 - 2.0 * sinu * sinu;
    temp   = 1.0 / pl;
    temp1  = 0.5 * j2 * temp;
    temp2  = temp1 * temp;

    /* ---- update for short period periodics ---- */
    mrt   = rl * (1.0 - 1.5 * temp2 * betal * rec->con41) +
            0.5 * temp1 * rec->x1mth2 * cos2u;
    su    = su - 0.25 * temp2 * rec->x7thm1 * sin2u;
    xnode = nodep + 1.5 * temp2 * cosip * sin2u;
    xinc  = xincp + 1.5 * temp2 * cosip * sinip * cos2u;
    mvt   = rdotl - nm * temp1 * rec->x1mth2 * sin2u / xke;
    rvdot = rvdotl + nm * temp1 * (rec->x1mth2 * cos2u + 1.5 * rec->con41) / xke;

    /* ---- orientation vectors ---- */
    sinsu = sin(su);
    cossu = cos(su);
    snod  = sin(xnode);
    cnod  = cos(xnode);
    sini  = sin(xinc);
    cosi  = cos(xinc);
    xmx   = -snod * cosi;
    xmy   =  cnod * cosi;
    ux    =  xmx * sinsu + cnod * cossu;
    uy    =  xmy * sinsu + snod * cossu;
    uz    =  sini * sinsu;
    vx    =  xmx * cossu - cnod * sinsu;
    vy    =  xmy * cossu - snod * sinsu;
    vz    =  sini * cossu;

    /* ---- position and velocity (in km and km/sec) ---- */
    r_km[0] = (mrt * ux) * radiusearthkm;
    r_km[1] = (mrt * uy) * radiusearthkm;
    r_km[2] = (mrt * uz) * radiusearthkm;
    v_kms[0] = (mvt * ux + rvdot * vx) * vkmpersec;
    v_kms[1] = (mvt * uy + rvdot * vy) * vkmpersec;
    v_kms[2] = (mvt * uz + rvdot * vz) * vkmpersec;

    /* sgp4fix for decaying satellites */
    if (mrt < 1.0) {
        return ADCS_SGP4_ERR_DECAYED;
    }
    return ADCS_SGP4_OK;
}

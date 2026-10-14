#!/usr/bin/env python3
"""Generate golden vectors for the ADCS orbit/environment model tests.

Writes into the output directory:
  igrf14_golden.csv  IGRF-14 field from ppigrf (independent implementation)
  sun_golden.csv     Geocentric apparent Sun direction in TEME from astropy

Not run by ctest. Needs a virtual environment with ppigrf (>= 2.1, ships
IGRF14.shc) and astropy:

    python3 -m venv venv && venv/bin/pip install ppigrf astropy
    venv/bin/python -I test/tools/gen_golden.py test/data

SGP4 golden data (SGP4-VER.TLE, tcppver.out) is not generated: it is copied
verbatim from Vallado's verification set (see test/data/README.md).
"""

import csv
import importlib.metadata
import os
import sys
from datetime import datetime, timedelta

import numpy as np
import ppigrf
from astropy import units as u
from astropy.coordinates import TEME, get_sun
from astropy.time import Time

IGRF_EPOCH = datetime(2025, 1, 1)
IGRF_END = datetime(2030, 1, 1)


def ppigrf_decimal_year(date):
    """Decimal year matching ppigrf's linear-in-time SV interpolation.

    ppigrf interpolates between the 2025.0 and 2030.0 (= 2025 + 5*SV)
    coefficient sets linearly in elapsed days, so the equivalent IGRF time
    offset is 5 yr * elapsed / (2030-01-01 - 2025-01-01).
    """
    frac = (date - IGRF_EPOCH) / (IGRF_END - IGRF_EPOCH)
    return 2025.0 + 5.0 * frac


def gen_igrf(path):
    dates = [datetime(2025, 1, 1), datetime(2027, 9, 15), datetime(2029, 12, 1)]
    radii = [6371.2, 6871.2]
    thetas = [0.001, 1.0] + list(np.arange(15.0, 166.0, 15.0)) + [179.0, 179.999]
    phis = list(np.arange(0.0, 360.0, 30.0))
    degrees = [13, 8, 1]

    rows = 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["decimal_year", "n_max", "r_km", "theta_deg", "phi_deg",
                    "br_nT", "btheta_nT", "bphi_nT"])
        for date in dates:
            dec = ppigrf_decimal_year(date)
            for n_max in degrees:
                for r in radii:
                    for th in thetas:
                        for ph in phis:
                            br, bt, bp = ppigrf.igrf_gc(r, th, ph, date,
                                                        max_degree=n_max)
                            w.writerow(["%.10f" % dec, n_max, "%.1f" % r,
                                        "%.3f" % th, "%.1f" % ph,
                                        "%.6f" % float(br[0]),
                                        "%.6f" % float(bt[0]),
                                        "%.6f" % float(bp[0])])
                            rows += 1
    return rows


def gen_sun(path):
    start = datetime(2025, 1, 1)
    times = [start + timedelta(days=10 * k, hours=(7 * k) % 24)
             for k in range(0, 183)]          # 2025-01-01 … 2029-12-xx
    t = Time(times, scale="utc")
    sun = get_sun(t).transform_to(TEME(obstime=t))
    xyz = sun.cartesian.xyz.to_value(u.km)
    unit = xyz / np.linalg.norm(xyz, axis=0)

    epoch = datetime(1970, 1, 1)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["unix_s", "x", "y", "z"])
        for k, when in enumerate(times):
            unix_s = (when - epoch).total_seconds()
            w.writerow(["%.1f" % unix_s, "%.12f" % unit[0, k],
                        "%.12f" % unit[1, k], "%.12f" % unit[2, k]])
    return len(times)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    n_igrf = gen_igrf(os.path.join(out, "igrf14_golden.csv"))
    n_sun = gen_sun(os.path.join(out, "sun_golden.csv"))
    ver = importlib.metadata.version
    print("igrf14_golden.csv: %d rows (ppigrf %s)" % (n_igrf, ver("ppigrf")))
    print("sun_golden.csv: %d rows (astropy %s, pyerfa %s)"
          % (n_sun, ver("astropy"), ver("pyerfa")))


if __name__ == "__main__":
    main()

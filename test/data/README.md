# ADCS model golden data

Reference data for `test_ephemeris` (ctest `ephemeris`). Do not edit these files by hand.

| File | Content | Source |
|------|---------|--------|
| `SGP4-VER.TLE` | Vallado SGP4 verification element sets, with start/stop/step (min) appended to line 2 | `sgp4/cpp/testsgp4/SGP4-VER.TLE` in <https://celestrak.org/publications/AIAA/2006-6753/AIAA-2006-6753.zip>; identical to the copy shipped with python-sgp4 2.25 |
| `sgp4_tcppver.txt` | Output of Vallado's C++ SGP4 for the file above (TEME km, km/s) | `sgp4/tcppver.out` in the python-sgp4 2.25 sdist (PyPI). Renamed because the repository `.gitignore` excludes `*.out` |
| `igrf14_golden.csv` | IGRF-14 geocentric field components (nT) at degrees 13 / 8 / 1, r = 6371.2 and 6871.2 km, 15 colatitudes (incl. 0.001° and 179.999°) × 12 longitudes, dates 2025-01-01, 2027-09-15, 2029-12-01 | `ppigrf` 2.1.0 (`IGRF14.shc`), an implementation independent of `adcs_igrf.c` |
| `sun_golden.csv` | Geocentric apparent Sun unit vector in TEME, every 10 days over 2025–2029 | astropy 8.0.1 / pyerfa 2.0.1.5 `get_sun()` transformed to `TEME` |

SHA-256 at the time of writing:

```
d246d1d9d768ace445a38a965713fa9ba52d80fd8a41a0502ff83d7acffe2881  SGP4-VER.TLE
687bf28dbe52df86e8e60ab5cb4a08d1aa3dbcaf4e63b1f7ab95f044fbe3833b  sgp4_tcppver.txt
2b2ff38fcd6e2c288409824ddd06f99777bd40eac1e9b27c2209c633b15eee9c  igrf14_golden.csv
8e7310b5f64aa3b6ac5ae35da8eb0f62648c22fa51909bceb1b336efb9842dfb  sun_golden.csv
```

## Notes

- `decimal_year` in `igrf14_golden.csv` is the time argument that reproduces ppigrf's interpolation, which is linear in elapsed days between the 2025.0 and 2030.0 coefficient sets: `2025 + 5 × (date − 2025-01-01) / (2030-01-01 − 2025-01-01)`. With that argument the two implementations should agree to rounding of the `float` coefficient table (< 0.001 nT).
- Three element sets in `SGP4-VER.TLE` (33333, 33334, 33335) have deliberately wrong checksums. They are deep-space sets, which the on-board propagator rejects anyway. The test recomputes column 69 before parsing every set and checks separately that the parser flags exactly these three.
- Regenerating the CSV files (not needed for the tests):

  ```bash
  python3 -m venv venv && venv/bin/pip install ppigrf astropy
  venv/bin/python -I test/tools/gen_golden.py test/data
  ```

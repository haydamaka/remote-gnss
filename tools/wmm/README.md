# WMM 2025 reference data

Straight from NOAA, unmodified, so the model in `main/wmm.c` can be regenerated
and rechecked without a download:

| File | What it is | Where it came from |
|---|---|---|
| `WMM.COF` | the coefficients, epoch 2025.0, issued 2024-11-13 | [WMM2025COF.zip](https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip) |
| `WMM2025_TestValues.txt` | 100 test points, 2025.0-2029.5, 0-100 km | the same zip |
| `WMM2025_TEST_VALUES.txt` | the 12 published test points | [WMM2025_TEST_VALUES.txt](https://www.ncei.noaa.gov/sites/default/files/2025-02/WMM2025_TEST_VALUES.txt) |

The model expires at the end of 2029. Replacing it means dropping in the new
`WMM.COF`, re-running `tools/wmm_gen.py`, and moving `WMM_EPOCH` and
`WMM_VALID_UNTIL` in `main/wmm.h`.

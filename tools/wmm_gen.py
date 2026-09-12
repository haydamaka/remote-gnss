#!/usr/bin/env python3
"""Turns NOAA's WMM.COF into the table embedded in main/wmm.c.

    tools/wmm_gen.py WMM.COF > table.txt

The coefficient file comes from
https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients
(WMM2025COF.zip). Paste the output over the table in main/wmm.c when the model
is replaced, and update WMM_EPOCH/WMM_VALID_UNTIL to match the new header line.
"""
import sys

def main():
    lines = [l for l in open(sys.argv[1]) if l.strip() and not l.strip().startswith('9999')]
    header, rows = lines[0].split(), [l.split() for l in lines[1:]]
    epoch, name = float(header[0]), header[1]
    # Triangular index: idx(n, m) = n(n+1)/2 + m - 1, so n=1..12, m=0..n packs into 90.
    table = {}
    for r in rows:
        n, m = int(r[0]), int(r[1])
        table[(n, m)] = tuple(float(v) for v in r[2:6])
    degree = max(n for n, _ in table)
    order = [table[(n, m)] for n in range(1, degree + 1) for m in range(n + 1)]

    print(f'// {name}, epoch {epoch}, {len(order)} coefficients to degree {degree}.')
    print(f'// Generated from WMM.COF by tools/wmm_gen.py; do not edit by hand.')
    for label, col in (('g', 0), ('h', 1), ('g_dot', 2), ('h_dot', 3)):
        print(f'static const float {label}[WMM_COEFFS] = {{')
        for i in range(0, len(order), 6):
            print('    ' + ' '.join(f'{order[j][col]:>10.1f}f,' for j in range(i, min(i + 6, len(order)))))
        print('};')
    print(f'// degree {degree}, epoch {epoch}')

if __name__ == '__main__':
    main()

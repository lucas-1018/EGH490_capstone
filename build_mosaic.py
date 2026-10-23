#!/usr/bin/env python3
"""
Usage:
    python3 build_mosaic.py /tmp/bluerov_survey_images/run_XXXX
    python3 build_mosaic.py RUN_DIR --coverage /tmp/bluerov_survey_images/coverage_XXXX.csv
    python3 build_mosaic.py RUN_DIR --step 0.3 --res 0.015 --out ~/map.png
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
from PIL import Image


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def load_rows(run_dir):
    rows = []
    with open(os.path.join(run_dir, 'poses.csv')) as f:
        for r in csv.DictReader(f):
            try:
                row = dict(file=r['file'], x=float(r['x']), y=float(r['y']),
                           yaw=float(r['yaw_rad']), alt=_f(r.get('altitude_m')),
                           L=_f(r.get('left_m')), R=_f(r.get('right_m')),
                           D=_f(r.get('down_m')))
            except (ValueError, KeyError):
                continue
            if all(math.isfinite(row[k]) for k in ('x', 'y', 'yaw')):
                rows.append(row)
    return rows


def attach_coverage(rows, cov_csv, max_dist=1.5):
    """Give each frame the left/right edge readings logged nearest to it
    (same heading) in a reactive_planner coverage CSV. Returns #frames hit."""
    cx, cy, hx, hy, cl, cr = [], [], [], [], [], []
    with open(cov_csv) as f:
        for r in csv.DictReader(f):
            try:
                if int(r['imaging']) != 1:
                    continue
                yaw = float(r['yaw'])
                cx.append(float(r['x']))
                cy.append(float(r['y']))
                hx.append(math.cos(yaw))
                hy.append(math.sin(yaw))
                cl.append(_f(r['left_lateral']))
                cr.append(_f(r['right_lateral']))
            except (ValueError, KeyError):
                continue
    if not cx:
        print('warning: coverage CSV has no imaging rows')
        return 0
    cx, cy, hx, hy = map(np.array, (cx, cy, hx, hy))
    cl, cr = np.array(cl), np.array(cr)
    hit = 0
    for r in rows:
        d2 = (cx - r['x']) ** 2 + (cy - r['y']) ** 2
        d2 = np.where(hx * math.cos(r['yaw']) + hy * math.sin(r['yaw']) > 0.5, d2, np.inf)
        i = int(np.argmin(d2))
        if d2[i] <= max_dist ** 2:
            r['L'], r['R'] = float(cl[i]), float(cr[i])
            hit += 1
    return hit


def clean_edges(rows, window):
    """Fill sonar misses from neighbouring frames of the same sweep and
    median-smooth the readings along the sweep."""
    n = len(rows)
    starts = [0]
    for i in range(1, n):
        if math.hypot(rows[i]['x'] - rows[i - 1]['x'], rows[i]['y'] - rows[i - 1]['y']) > 0.6:
            starts.append(i)
    starts.append(n)
    window = max(1, int(window)) | 1            # odd
    for key in ('L', 'R', 'D'):
        v = np.array([r[key] for r in rows], float)
        for a, b in zip(starts[:-1], starts[1:]):
            seg = v[a:b].copy()
            ok = np.isfinite(seg) & (seg > 0.05)
            if not ok.any():
                v[a:b] = np.nan
                continue
            idx = np.arange(b - a)
            seg[~ok] = np.interp(idx[~ok], idx[ok], seg[ok])
            if window > 1 and len(seg) > 1:
                k = window // 2
                pad = np.pad(seg, (k, k), mode='edge')
                seg = np.array([np.median(pad[i:i + window]) for i in range(len(seg))])
            v[a:b] = seg
        for r, val in zip(rows, v):
            r[key] = float(val)


def select(rows, step):
    """Keep a frame only after the drone has moved `step` m from the last
    kept one -- at 10 fps most frames are near duplicates."""
    kept, last = [], None
    for r in rows:
        if last is None or math.hypot(r['x'] - last['x'], r['y'] - last['y']) >= step:
            kept.append(r)
            last = r
    return kept


def image_left_is_vehicle_left(a):
    """Which sonar side is the image's left edge? None = cannot tell."""
    off = a.yaw_offset_deg % 360.0
    if abs(off) < 1e-6 or abs(off - 360.0) < 1e-6:
        left = True
    elif abs(off - 180.0) < 1e-6:
        left = False
    else:
        return None
    return (not left) if a.flip_x else left


class Placement:


    def __init__(self, row, w, h, a, swap, use_edges):
        alt = row['alt'] if math.isfinite(row['alt']) and row['alt'] > 0.2 else a.default_alt
        self.w, self.h = w, h
        tau = math.tan(math.radians(a.fov) / 2.0)
        self.f = (w / 2.0) / tau
        te = math.tan(math.radians(a.edge_angle))

        self.nadir_limited = False
        L = R = float('nan')
        if use_edges:
            L, R = (row['R'], row['L']) if swap else (row['L'], row['R'])
        nom = alt * te
        dL = dR = alt
        if math.isfinite(L) and L > 0.05:
            L = float(np.clip(L, 0.25 * nom, 4.0 * nom))
            dL = L / te
        else:
            L = nom
        if math.isfinite(R) and R > 0.05:
            R = float(np.clip(R, 0.25 * nom, 4.0 * nom))
            dR = R / te
        else:
            R = nom

        # optional third point: depth straight down
        d0 = None
        if use_edges:
            if math.isfinite(row['D']) and row['D'] > 0.2:
                d0 = row['D']
            elif (not a.no_nadir) and math.isfinite(row['alt']) and row['alt'] > 0.2:
                d0 = row['alt']
        if d0 is None:
            kc = [-L, R]
            kz = [dL, dR]
        else:
            # altitude_m is the NEAREST return anywhere in the fan, so on a
            # slope it reads low and is not the depth straight down. Only let
            # it bend the profile a limited amount away from the straight
            # line between the two edge hits.
            plane0 = dL + (dR - dL) * L / (L + R)
            lim = a.nadir_trust * plane0
            d0_raw = d0
            d0 = plane0 + float(np.clip(d0 - plane0, -lim, lim))
            self.nadir_limited = abs(d0 - d0_raw) > 1e-6
            kc = [-L, 0.0, R]
            kz = [dL, d0, dR]
        # Limit the slope of the ground profile. Near 1/tan(fov/2) (=1 for a
        # 90 deg camera) the pixel->ground map degenerates into smeared
        # streaks, so steeper faces are treated as a gentler ramp. Slopes are
        # re-integrated outward from the centre-most knot.
        smax = a.max_slope / tau
        raw = list(kz)
        k0 = int(np.argmin(np.abs(kc)))
        for i in range(k0 + 1, len(kc)):
            d = (kz[i] - kz[i - 1]) / (kc[i] - kc[i - 1])
            kz[i] = kz[i - 1] + float(np.clip(d, -smax, smax)) * (kc[i] - kc[i - 1])
        for i in range(k0 - 1, -1, -1):
            d = (kz[i + 1] - kz[i]) / (kc[i + 1] - kc[i])
            kz[i] = kz[i + 1] - float(np.clip(d, -smax, smax)) * (kc[i + 1] - kc[i])
        self.slope_limited = max(abs(x - y) for x, y in zip(raw, kz)) > 1e-6
        self.kc, self.kz = np.array(kc), np.maximum(np.array(kz), 0.3)
        self.s_lo = (self.kz[1] - self.kz[0]) / (self.kc[1] - self.kc[0])
        self.s_hi = (self.kz[-1] - self.kz[-2]) / (self.kc[-1] - self.kc[-2])

        # where the image's own left/right edge rays meet the ground
        cR, cL = R, -L
        for _ in range(40):
            cR = 0.5 * cR + 0.5 * tau * float(self.depth(np.array(cR)))
            cL = 0.5 * cL - 0.5 * tau * float(self.depth(np.array(cL)))
        self.cR, self.cL = cR, cL
        zmax = max(float(self.depth(np.array(x))) for x in (cL, 0.0, cR))
        self.A = zmax * (h / 2.0) / self.f

        th = row['yaw'] + math.radians(a.yaw_offset_deg)
        fwd = np.array([math.cos(th), math.sin(th)])        # image "up" in world
        right = np.array([math.sin(th), -math.cos(th)])     # image "right" in world
        if a.flip_x:
            right = -right
        if a.flip_y:
            fwd = -fwd
        self.fwd, self.right = fwd, right
        self.pos = np.array([row['x'], row['y']])

    def depth(self, c):
        """Ground depth below the camera at cross-track offset c (metres)."""
        z = np.interp(c, self.kc, self.kz)
        z = np.where(c < self.kc[0], self.kz[0] + self.s_lo * (c - self.kc[0]), z)
        z = np.where(c > self.kc[-1], self.kz[-1] + self.s_hi * (c - self.kc[-1]), z)
        return np.maximum(z, 0.3)

    def scale(self):
        """Average metres per source pixel across the frame."""
        return (self.cR - self.cL) / self.w

    def corners_world(self):
        return np.array([self.pos + self.right * c + self.fwd * a
                         for c in (self.cL, self.cR) for a in (-self.A, self.A)])

    def to_source(self, X, Y):
        """world (X, Y) arrays -> source pixel coordinates (u, v)."""
        dx, dy = X - self.pos[0], Y - self.pos[1]
        c = dx * self.right[0] + dy * self.right[1]
        a = dx * self.fwd[0] + dy * self.fwd[1]
        z = self.depth(c)
        return self.w / 2.0 + self.f * c / z, self.h / 2.0 - self.f * a / z


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', nargs='+',
                    help='run folder(s). Two drones: DIR1 DIR2@dx,dy (dx,dy = where drone 2 '
                         'spawned relative to drone 1, the launch-file offset)')
    ap.add_argument('--out', default=None, help='output PNG (default RUN_DIR/mosaic.png)')
    ap.add_argument('--coverage', default=None, nargs='+',
                    help='reactive_planner coverage CSV, for runs whose poses.csv has no '
                         'left_m/right_m columns')
    ap.add_argument('--no-edges', action='store_true',
                    help='ignore the sonar edges: flat floor at the sonar altitude')
    ap.add_argument('--fov', type=float, default=90.0, help='camera horizontal FOV, degrees')
    ap.add_argument('--edge-angle', type=float, default=45.0,
                    help='angle from straight down of the sonar edge beams (the '
                         "planner's camera_fov_deg / 2)")
    ap.add_argument('--no-nadir', action='store_true',
                    help="do not use the poses.csv altitude_m as a third (straight-down) "
                         "depth point -- fit one tilted plane through the two edge points "
                         "only. By default altitude_m is used: it is the sonar's nearest "
                         "return, which reads a little low on steep ground but makes the "
                         "ground model follow bumps between the two edges")
    ap.add_argument('--max-slope', type=float, default=0.85,
                    help='cap on ground slope in the stretch model, as a fraction of the '
                         'limit where the mapping breaks down (0-1, default 0.7)')
    ap.add_argument('--nadir-trust', type=float, default=0.5,
                    help='how far (fraction of depth) the altitude reading may pull the '
                         'centre of the profile off the edge-to-edge line (default 0.25)')
    ap.add_argument('--edge-smooth', type=int, default=7,
                    help='median window (frames) for the edge readings along a sweep')
    ap.add_argument('--step', type=float, default=0.5,
                    help='keep a frame every STEP metres of travel (default 0.5)')
    ap.add_argument('--res', type=float, default=None,
                    help='output metres per pixel (default: about the native image detail)')
    ap.add_argument('--default-alt', type=float, default=5.0,
                    help='altitude to use if a row has none')
    ap.add_argument('--yaw-offset-deg', type=float, default=0.0)
    ap.add_argument('--flip-x', action='store_true')
    ap.add_argument('--flip-y', action='store_true')
    ap.add_argument('--blend', choices=['best', 'feather'], default='best',
                    help="'best' (default): each pixel comes from the one frame that sees it "
                         "nearest its centre -- sharp, may show faint seams. 'feather': "
                         "average all overlapping frames -- smooth, but blurs wherever "
                         "frames are not perfectly aligned")
    ap.add_argument('--max-pixels', type=int, default=14000,
                    help='refuse canvases larger than this on a side')
    a = ap.parse_args()

    runs, rows = [], []
    for spec in a.run_dir:
        d, _, o = spec.partition('@')
        off = tuple(float(v) for v in o.split(',')) if o else (0.0, 0.0)
        rs = load_rows(d)
        for r in rs:
            r['dir'], r['off'] = d, off
        runs.append(rs)
        rows += rs
    if not rows:
        sys.exit('no usable rows in poses.csv')
    if a.coverage and len(a.coverage) != len(runs):
        sys.exit(f'give one --coverage CSV per run folder ({len(runs)} runs, '
                 f'{len(a.coverage)} CSVs)')

    use_edges = not a.no_edges
    swap = False
    if use_edges:
        side = image_left_is_vehicle_left(a)
        if side is None:
            print('note: --yaw-offset-deg is not a multiple of 180, so the image sides do not '
                  'line up with the sonar sides -> using the flat-floor assumption')
            use_edges = False
        else:
            swap = not side
    if use_edges and a.coverage:
        hit = sum(attach_coverage(rs, c) for rs, c in zip(runs, a.coverage))
        print(f'matched {hit}/{len(rows)} frames to coverage-log edge readings')
    if use_edges:
        have = sum(1 for r in rows if math.isfinite(r['L']) or math.isfinite(r['R']))
        if have == 0:
            print('no left/right edge data in poses.csv -> flat-floor assumption. '
                  'For a run recorded before those columns existed, add --coverage '
                  '/tmp/bluerov_survey_images/coverage_XXXX.csv')
            use_edges = False
        else:
            for rs in runs:
                clean_edges(rs, a.edge_smooth)
            print(f'slope-aware stretching ON ({have}/{len(rows)} frames have edge readings)')
    if not use_edges:
        print('slope-aware stretching OFF (flat floor at the sonar altitude)')

    kept = []
    for rs in runs:
        kept += select(rs, a.step)
    for r in kept:                       # put every drone in drone 1's frame
        r['x'] += r['off'][0]
        r['y'] += r['off'][1]
    print(f'{len(rows)} frames in poses.csv -> using {len(kept)} (every {a.step} m)')

    first = Image.open(os.path.join(kept[0]['dir'], kept[0]['file']))
    W, H = first.size

    pl = [Placement(r, W, H, a, swap, use_edges) for r in kept]
    if use_edges:
        ns = sum(p.slope_limited for p in pl)
        nn = sum(p.nadir_limited for p in pl)
        print(f'  guards: slope cap applied on {ns}/{len(pl)} frames, '
              f'altitude reading limited on {nn}/{len(pl)} frames')
    allc = np.vstack([p.corners_world() for p in pl])
    xmin, ymin = allc.min(axis=0)
    xmax, ymax = allc.max(axis=0)
    res = a.res or float(np.median([p.scale() for p in pl]))
    cw, ch = int(math.ceil((xmax - xmin) / res)) + 1, int(math.ceil((ymax - ymin) / res)) + 1
    print(f'area x[{xmin:.1f},{xmax:.1f}] y[{ymin:.1f},{ymax:.1f}] m, '
          f'{res * 100:.2f} cm/pixel, canvas {cw} x {ch}')
    if max(cw, ch) > a.max_pixels:
        sys.exit(f'canvas too big ({cw}x{ch}); use a larger --res, e.g. --res {res * 2:.3f}')

    acc = np.zeros((ch, cw, 3), np.float32)
    wsum = np.zeros((ch, cw), np.float32)       # feather: weight sum; best: best weight

    for n, (r, p) in enumerate(zip(kept, pl)):
        img = np.asarray(Image.open(os.path.join(r['dir'], r['file'])).convert('RGB'),
                         dtype=np.float32)
        if img.shape[:2] != (H, W):
            continue
        cs = p.corners_world()
        u0 = max(int(math.floor((cs[:, 0].min() - xmin) / res)), 0)
        u1 = min(int(math.ceil((cs[:, 0].max() - xmin) / res)) + 1, cw)
        v0 = max(int(math.floor((ymax - cs[:, 1].max()) / res)), 0)
        v1 = min(int(math.ceil((ymax - cs[:, 1].min()) / res)) + 1, ch)
        if u1 <= u0 or v1 <= v0:
            continue
        X = xmin + (np.arange(u0, u1)[None, :] + 0.5) * res
        Y = ymax - (np.arange(v0, v1)[:, None] + 0.5) * res
        su, sv = p.to_source(np.broadcast_to(X, (v1 - v0, u1 - u0)),
                             np.broadcast_to(Y, (v1 - v0, u1 - u0)))
        valid = (su >= 0) & (su < W) & (sv >= 0) & (sv < H)
        iy, ix = np.nonzero(valid)
        if len(iy) == 0:
            continue
        su, sv = su[iy, ix], sv[iy, ix]
        # feather weight: 1 at the frame centre, ~0 at its edges
        wgt = (np.clip(1 - np.abs(2 * su / W - 1), 0, 1) *
               np.clip(1 - np.abs(2 * sv / H - 1), 0, 1) + 1e-6).astype(np.float32)
        # bilinear sample
        fx, fy = su - 0.5, sv - 0.5
        x0 = np.clip(np.floor(fx).astype(int), 0, W - 1)
        y0 = np.clip(np.floor(fy).astype(int), 0, H - 1)
        x1, y1 = np.minimum(x0 + 1, W - 1), np.minimum(y0 + 1, H - 1)
        wx = np.clip(fx - x0, 0, 1)[:, None].astype(np.float32)
        wy = np.clip(fy - y0, 0, 1)[:, None].astype(np.float32)
        patch = (img[y0, x0] * (1 - wx) * (1 - wy) + img[y0, x1] * wx * (1 - wy) +
                 img[y1, x0] * (1 - wx) * wy + img[y1, x1] * wx * wy)
        cy_, cx_ = v0 + iy, u0 + ix
        if a.blend == 'feather':
            acc[cy_, cx_] += patch * wgt[:, None]
            wsum[cy_, cx_] += wgt
        else:
            win = wgt > wsum[cy_, cx_]
            cy_, cx_ = cy_[win], cx_[win]
            acc[cy_, cx_] = patch[win]
            wsum[cy_, cx_] = wgt[win]
        if (n + 1) % 50 == 0:
            print(f'  placed {n + 1}/{len(kept)}')

    out = np.zeros_like(acc)
    m = wsum > 1e-6
    if a.blend == 'feather':
        out[m] = acc[m] / wsum[m][..., None]
    else:
        out[m] = acc[m]
    path = a.out or os.path.join(a.run_dir[0].partition('@')[0], 'mosaic.png')
    Image.fromarray(np.clip(out, 0, 255).astype(np.uint8)).save(path)
    with open(os.path.splitext(path)[0] + '.json', 'w') as f:
        json.dump(dict(x_min=float(xmin), x_max=float(xmax), y_min=float(ymin),
                       y_max=float(ymax), metres_per_pixel=res, width=cw, height=ch,
                       frames_used=len(kept), slope_aware=bool(use_edges),
                       north_up=True), f, indent=2)
    print(f'covered {100.0 * m.mean():.1f}% of the canvas')
    print(f'saved {path}')


if __name__ == '__main__':
    main()

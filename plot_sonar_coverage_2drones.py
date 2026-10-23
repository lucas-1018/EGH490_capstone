#!/usr/bin/env python3
import argparse
import csv
import math

import numpy as np
from PIL import Image, ImageDraw
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.path import Path as MPath


def parse_poly(text):
    return np.array([tuple(float(v) for v in p.split(',')) for p in text.split()])


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append(dict(
                    x=float(r['x']), y=float(r['y']), yaw=float(r['yaw']),
                    L=float(r['left_lateral']), R=float(r['right_lateral']),
                    lane=int(r['lane']), img=int(r['imaging'])))
            except (ValueError, KeyError):
                pass
    return rows


def fill_missing(y, v):
    """Fill beams that saw nothing (NaN) by interpolating along the sweep."""
    v = np.array(v, float)
    ok = np.isfinite(v)
    if ok.all() or not ok.any():
        return v
    order = np.argsort(y)
    yy, vv, oo = y[order], v[order].copy(), ok[order]
    vv[~oo] = np.interp(yy[~oo], yy[oo], vv[oo])
    out = np.empty_like(v)
    out[order] = vv
    return out


def lane_edges(rows, off=(0.0, 0.0), tag=0):
    """-> {(tag, lane): (y, west_x, east_x)}; heading decides which side is west.
    off shifts this drone's frame into drone 1's frame."""
    lanes = {}
    for r in rows:
        if r['img'] and abs(math.sin(r['yaw'])) > 0.7:
            lanes.setdefault(r['lane'], []).append(r)
    out = {}
    for k, rs in lanes.items():
        y = np.array([r['y'] + off[1] for r in rs])
        x = np.array([r['x'] + off[0] for r in rs])
        s = np.array([math.sin(r['yaw']) for r in rs])
        L = fill_missing(y, [r['L'] for r in rs])
        R = fill_missing(y, [r['R'] for r in rs])
        ok = np.isfinite(L) & np.isfinite(R)
        if ok.sum() < 2:
            continue
        y, x, s, L, R = y[ok], x[ok], s[ok], L[ok], R[ok]
        xl, xr = x - s * L, x + s * R          # world x of left / right edge
        o = np.argsort(y)
        out[(tag, k)] = (y[o], np.minimum(xl, xr)[o], np.maximum(xl, xr)[o])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+',
                    help='coverage CSV(s); one per drone, optionally FILE@dx,dy')
    ap.add_argument('--polygon', default='0,0 40,0 40,30 0,30',
                    help='survey polygon "x,y x,y ..." (default: the launch-file rectangle)')
    ap.add_argument('--res', type=float, default=0.1, help='raster size, m per pixel')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    poly = parse_poly(a.polygon)
    lanes = {}
    for i, spec in enumerate(a.csv):
        path, _, o = spec.partition('@')
        off = tuple(float(v) for v in o.split(',')) if o else (0.0, 0.0)
        lanes.update(lane_edges(load(path), off, i))
    if not lanes:
        raise SystemExit('no usable imaging rows in the CSV (was the survey running?)')

    allx = np.concatenate([np.r_[w, e] for _, w, e in lanes.values()] + [poly[:, 0]])
    ally = np.concatenate([y for y, _, _ in lanes.values()] + [poly[:, 1]])
    x0, x1 = allx.min() - 2.0, allx.max() + 2.0
    y0, y1 = ally.min() - 2.0, ally.max() + 2.0
    W = int(math.ceil((x1 - x0) / a.res))
    H = int(math.ceil((y1 - y0) / a.res))

    def px(x, y):
        return ((x - x0) / a.res, (y1 - y) / a.res)

    count = np.zeros((H, W), np.int16)
    for y, west, east in lanes.values():
        m = Image.new('L', (W, H), 0)
        d = ImageDraw.Draw(m)
        for i in range(len(y) - 1):
            d.polygon([px(west[i], y[i]), px(east[i], y[i]),
                       px(east[i + 1], y[i + 1]), px(west[i + 1], y[i + 1])], fill=1)
        count += np.asarray(m, dtype=np.int16)

    img = np.ones((H, W, 3), np.float32)
    img[count == 1] = (0.55, 0.85, 0.55)
    img[count >= 2] = (0.9, 0.2, 0.2)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(img, extent=(x0, x1, y0, y1), origin='upper', interpolation='nearest')
    for y, west, east in lanes.values():
        ax.plot(west, y, color='#0b6623', lw=1.2)
        ax.plot(east, y, color='#0b6623', lw=1.2)
    closed = np.vstack([poly, poly[:1]])
    ax.plot(closed[:, 0], closed[:, 1], color='blue', lw=2.2)

    # score only the part inside the polygon
    gx = x0 + (np.arange(W) + 0.5) * a.res
    gy = y1 - (np.arange(H) + 0.5) * a.res
    GX, GY = np.meshgrid(gx, gy)
    inside = MPath(poly).contains_points(np.c_[GX.ravel(), GY.ravel()]).reshape(GX.shape)
    n_in = max(int(inside.sum()), 1)
    cov = 100.0 * (inside & (count >= 1)).sum() / n_in
    miss = 100.0 * (inside & (count == 0)).sum() / n_in
    over = 100.0 * (inside & (count >= 2)).sum() / n_in

    ax.set_title(f'Sonar edge-beam coverage, {len(lanes)} sweeps' + (f', {len(a.csv)} drones' if len(a.csv) > 1 else ''))
    ax.set_xlabel('x (m, east)')
    ax.set_ylabel('y (m, north)')
    ax.set_aspect('equal')
    ax.legend(handles=[Patch(color=(0.55, 0.85, 0.55), label='covered once'),
                       Patch(color=(0.9, 0.2, 0.2), label='overlap (2+ sweeps)'),
                       plt.Line2D([], [], color='#0b6623', label='footprint edges'),
                       plt.Line2D([], [], color='blue', lw=2.2, label='survey polygon')],
              loc='upper right')
    out = a.out or a.csv[0].partition('@')[0].rsplit('.', 1)[0] + '.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'saved {out}')
    print(f'inside the polygon: {cov:.1f}% covered, {miss:.1f}% missed, {over:.1f}% overlapped')


if __name__ == '__main__':
    main()

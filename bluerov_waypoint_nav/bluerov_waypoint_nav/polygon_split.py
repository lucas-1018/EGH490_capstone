import math


def polygon_area(xs, ys):
    n = len(xs)
    return 0.5 * abs(sum(xs[i] * ys[(i + 1) % n] - xs[(i + 1) % n] * ys[i]
                         for i in range(n)))


def _clip(points, x_cut, keep_left):
    """Sutherland-Hodgman clip of a polygon against the line x = x_cut."""
    inside = (lambda p: p[0] <= x_cut) if keep_left else (lambda p: p[0] >= x_cut)
    out = []
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        ia, ib = inside(a), inside(b)
        if ia != ib:
            t = (x_cut - a[0]) / (b[0] - a[0])
            out.append((x_cut, a[1] + t * (b[1] - a[1])))
        if ib:
            out.append(b)
    return out


def strip(xs, ys, x_lo, x_hi):
    pts = list(zip(xs, ys))
    pts = _clip(pts, x_hi, True)
    if pts:
        pts = _clip(pts, x_lo, False)
    return pts


def split_equal_area(xs, ys, n):
    """-> list of n (xs, ys) polygons, west to east, each 1/n of the area."""
    xs, ys = [float(v) for v in xs], [float(v) for v in ys]
    if n <= 1:
        return [(xs, ys)]
    xmin, xmax = min(xs), max(xs)
    total = polygon_area(xs, ys)
    cuts = [xmin]
    for k in range(1, n):
        lo, hi = cuts[-1], xmax
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            pts = strip(xs, ys, xmin, mid)
            a = polygon_area([p[0] for p in pts], [p[1] for p in pts]) if len(pts) >= 3 else 0.0
            lo, hi = (mid, hi) if a < total * k / n else (lo, mid)
        cuts.append(0.5 * (lo + hi))
    cuts.append(xmax)
    out = []
    for i in range(n):
        pts = strip(xs, ys, cuts[i], cuts[i + 1])
        out.append(([p[0] for p in pts], [p[1] for p in pts]))
    return out


def _y_extent(xs, ys, x):
    n = len(xs)
    hits = []
    for i in range(n):
        x1, y1, x2, y2 = xs[i], ys[i], xs[(i + 1) % n], ys[(i + 1) % n]
        if abs(x1 - x2) < 1e-12:
            if abs(x - x1) < 1e-9:
                hits += [y1, y2]
        elif min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9:
            hits.append(y1 + (x - x1) / (x2 - x1) * (y2 - y1))
    return (min(hits), max(hits)) if hits else (min(ys), max(ys))


def split_by_lanes(xs, ys, n, pitch):
    """Split into n strips that each hold a WHOLE number of sweeps.

    pitch = nominal distance between neighbouring sweeps (swath - overlap).
    A cut that lands mid-sweep makes the drone before it run an extra sweep
    that mostly overlaps its neighbour, so cuts are placed on sweep
    boundaries. Consecutive sweeps are grouped so the longest group (the
    drone that finishes last) is as short as possible. Falls back to equal
    area if there are fewer sweeps than drones."""
    xs, ys = [float(v) for v in xs], [float(v) for v in ys]
    xmin, xmax = min(xs), max(xs)
    m = int(math.ceil((xmax - xmin) / pitch - 1e-9))
    if n <= 1 or m < n:
        return split_equal_area(xs, ys, n)
    half = 0.5 * pitch
    cost = []                       # rough path length of each sweep
    for k in range(m):
        x0, x1 = xmin + k * pitch, xmin + (k + 1) * pitch
        xsamp = [min(max(x0 + t * (x1 - x0) / 4, xmin), xmax) for t in range(5)]
        lo = min(_y_extent(xs, ys, x)[0] for x in xsamp)
        hi = max(_y_extent(xs, ys, x)[1] for x in xsamp)
        cost.append(hi - lo + pitch)    # + cross-over to the next sweep
    pre = [0.0]
    for c in cost:
        pre.append(pre[-1] + c)
    INF = float('inf')
    best = [[INF] * (m + 1) for _ in range(n + 1)]
    cut = [[0] * (m + 1) for _ in range(n + 1)]
    best[0][0] = 0.0
    for d in range(1, n + 1):
        for j in range(d, m + 1):
            for i in range(d - 1, j):
                v = max(best[d - 1][i], pre[j] - pre[i])
                if v < best[d][j]:
                    best[d][j], cut[d][j] = v, i
    bounds, j = [m], m
    for d in range(n, 0, -1):
        j = cut[d][j]
        bounds.append(j)
    bounds = bounds[::-1]           # [0, k1, k2, ..., m]
    out = []
    for d in range(n):
        x_lo = xmin + bounds[d] * pitch if d else xmin
        x_hi = xmin + bounds[d + 1] * pitch if d < n - 1 else xmax
        pts = strip(xs, ys, x_lo, x_hi)
        out.append(([p[0] for p in pts], [p[1] for p in pts]))
    return out

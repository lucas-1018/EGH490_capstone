import math
from collections import namedtuple

import numpy as np

Waypoint = namedtuple('Waypoint', ['x', 'y', 'lane_dir'])

# returned by SurveyPlanner.next_waypoint() when a neighbouring drone's edge
# data is not available yet: hold position and call again a moment later
WAIT = object()

_AXIS = {'x': 0, 'y': 1, 'z': 2}


def lane_yaw(lane_dir):
    """Heading (rad, ENU) for travelling along +Y (lane_dir=+1) or -Y (-1)."""
    return math.pi / 2.0 if lane_dir > 0 else -math.pi / 2.0


# --------------------------------------------------------------------------
# point cloud helpers
# --------------------------------------------------------------------------
def cloud_to_xyz(data, point_step, offsets, count):

    if count <= 0:
        return np.empty((0, 3))
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': list(offsets), 'itemsize': int(point_step)})
    arr = np.frombuffer(data, dtype=dt, count=int(count))
    xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float64)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def _azimuth_deg(xyz, boresight_axis, lateral_axis, lateral_sign):
    b = xyz[:, _AXIS[boresight_axis]]
    lat = lateral_sign * xyz[:, _AXIS[lateral_axis]]
    return np.degrees(np.arctan2(lat, b))


def extract_edge(xyz, side, edge_angle_deg, tol_deg=2.0, boresight_axis='x',
                 lateral_axis='y', lateral_sign=1.0, min_points=3):
    """Footprint edge on one side, from the sonar beam(s) at edge_angle_deg.

    side: +1 = vehicle left, -1 = vehicle right.
    Returns (lateral_offset_m, down_distance_m, n_points) or None if the
    beam(s) at that angle saw nothing (everything +/-inf / out of range).

    For flat ground at altitude h and edge_angle 45 deg: lateral = h,
    down = h."""
    if xyz is None or len(xyz) == 0:
        return None
    az = _azimuth_deg(xyz, boresight_axis, lateral_axis, lateral_sign)
    mask = np.abs(az - side * edge_angle_deg) <= tol_deg
    if int(mask.sum()) < min_points:
        return None
    sel = xyz[mask]
    lateral = abs(float(np.median(lateral_sign * sel[:, _AXIS[lateral_axis]])))
    down = float(np.median(sel[:, _AXIS[boresight_axis]]))
    return lateral, down, int(mask.sum())


def describe_cloud(xyz, boresight_axis='x', lateral_axis='y', lateral_sign=1.0):
    """One-line diagnostic so the axis mapping can be checked against real
    data: for the real fan the azimuth span should be about +/-65 deg."""
    if xyz is None or len(xyz) == 0:
        return 'cloud has 0 finite points (sonar sees nothing in range)'
    az = _azimuth_deg(xyz, boresight_axis, lateral_axis, lateral_sign)
    ext = ', '.join(f'{n}:[{xyz[:, i].min():.2f},{xyz[:, i].max():.2f}]'
                    for n, i in _AXIS.items())
    return (f'{len(xyz)} finite points | extents {ext} | azimuth '
            f'[{az.min():.1f},{az.max():.1f}] deg using boresight='
            f'{boresight_axis} lateral={lateral_axis} sign={lateral_sign:+.0f}')


# --------------------------------------------------------------------------
# the planner
# --------------------------------------------------------------------------
class SurveyPlanner:
    def __init__(self, poly_x, poly_y, camera_fov_deg=90.0,
                 nominal_altitude=5.0, step_length=3.0, lane_overlap=0.5,
                 lateral_deadband=0.25, max_lateral_correction=3.0,
                 max_lanes=60, min_leg=0.5, missing_lateral=None,
                 fleet_size=1, fleet_index=0, provisional_jump=True):
        self.px = [float(v) for v in poly_x]
        self.py = [float(v) for v in poly_y]
        self.xmin, self.xmax = min(self.px), max(self.px)
        self.ymin, self.ymax = min(self.py), max(self.py)

        self.edge_angle = camera_fov_deg / 2.0
        self.nominal_half = nominal_altitude * math.tan(math.radians(self.edge_angle))
        self.step_length = step_length
        self.lane_overlap = lane_overlap
        self.deadband = lateral_deadband
        self.max_correction = max_lateral_correction
        self.max_lanes = max_lanes
        self.min_leg = min_leg
        # When an edge beam returns NOTHING (terrain falls away beyond the
        # sonar's range) the real footprint edge is at least this far out:
        # max_range * sin(edge_angle). Better than assuming the nominal width,
        # which under-estimates the swath and makes sweeps overlap too much.
        self.missing_lateral = missing_lateral

        self.fleet_size = max(int(fleet_size), 1)
        self.fleet_index = int(fleet_index)
        self.shared = self.fleet_size > 1
        self.provisional_jump = bool(provisional_jump)
        self.leader = self.fleet_index == 0
        self._ref_store = {}            # neighbour's lane -> dict(complete, y, east)
        self._ref_last_lane = None      # set once the neighbour has finished
        self._sent_complete = -1
        self.outbox = []                # (lane, complete, done, y[], east[]) to publish

        self.lane = 0
        self.lane_dir = +1
        self.lane_samples = []          # (y, west_x, east_x) for this lane
        self._lane_lo = None            # lane-wide y limits: only ever widen within a lane
        self._lane_hi = None
        # "previous lane" east edge. For lane 0 this is the survey boundary,
        # so the first lane's west edge gets aligned with xmin.
        self.prev_y = np.array([-1e9, 1e9])
        self.prev_east = np.array([self.xmin + lane_overlap] * 2)
        self.finished = False
        self.last_info = ''

    # -- geometry helpers --------------------------------------------------
    def _y_extent(self, x):
        """Survey polygon's y range along the vertical line at x."""
        x = min(max(x, self.xmin), self.xmax)
        ys = []
        n = len(self.px)
        for i in range(n):
            x1, y1 = self.px[i], self.py[i]
            x2, y2 = self.px[(i + 1) % n], self.py[(i + 1) % n]
            if abs(x1 - x2) < 1e-12:
                if abs(x - x1) < 1e-9:
                    ys += [y1, y2]
                continue
            if min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9:
                t = (x - x1) / (x2 - x1)
                ys.append(y1 + t * (y2 - y1))
        if not ys:
            return self.ymin, self.ymax
        return min(ys), max(ys)

    def _y_range(self, x_lo, x_hi):
        """Polygon's y range over a whole swath [x_lo, x_hi] (not just its
        centre line), so a sweep on a slanted edge still reaches the corner."""
        x_lo = min(max(x_lo, self.xmin), self.xmax)
        x_hi = min(max(x_hi, self.xmin), self.xmax)
        xs = list(np.linspace(x_lo, x_hi, 9)) + [v for v in self.px if x_lo <= v <= x_hi]
        los, his = zip(*(self._y_extent(x) for x in xs))
        return min(los), max(his)

    def _lane_clamp(self, py, want, y_lo, y_hi):
        """Next y along the lane. The polygon's y-limit depends on the measured
        swath, which jitters; letting the limit shrink again made the drone turn
        back (and never finish the lane). So the limit only widens within a lane,
        and the waypoint never moves against the lane direction."""
        self._lane_lo = y_lo if self._lane_lo is None else min(self._lane_lo, y_lo)
        self._lane_hi = y_hi if self._lane_hi is None else max(self._lane_hi, y_hi)
        ny = min(max(want, self._lane_lo), self._lane_hi)
        return max(ny, py) if self.lane_dir > 0 else min(ny, py)

    # -- fleet helpers -----------------------------------------------------
    def offer_ref(self, lane, complete, done, y, east):
        """A neighbour's lane profile (y, east edge x), in THIS drone's frame."""
        lane = int(lane)
        self._ref_store[lane] = dict(complete=bool(complete),
                                     y=np.asarray(y, float), east=np.asarray(east, float))
        for k in [k for k in self._ref_store if k < lane - 3]:
            del self._ref_store[k]
        if done:
            self._ref_last_lane = lane

    @staticmethod
    def _sorted(ref):
        o = np.argsort(ref['y'])
        return ref['y'][o], ref['east'][o]

    def _queue(self, complete, done):
        if not self.shared:
            return
        smp = sorted(self.lane_samples)
        self.outbox.append((self.lane, bool(complete), bool(done),
                            [a for a, _, _ in smp], [c for _, _, c in smp]))

    def _finish(self, info):
        self.finished = True
        self.last_info = info
        self._queue(True, True)
        return None

    def start_ready(self):
        """False while a following drone is still waiting for its neighbour's
        first reading (it needs it to place its first lane)."""
        return (not self.shared) or self.leader or (0 in self._ref_store)

    # -- public API --------------------------------------------------------
    def start(self):
        half = self.nominal_half
        x0 = self.xmin + half
        if x0 > self.xmax:
            x0 = 0.5 * (self.xmin + self.xmax)
        if self.shared and not self.leader:
            ry, re = self._sorted(self._ref_store[0])
            for _ in range(2):
                y_lo, _ = self._y_range(x0 - half, x0 + half)
                x0 = float(np.interp(y_lo, ry, re)) - self.lane_overlap + half
        y_lo, _ = self._y_range(x0 - half, x0 + half)
        self.last_info = (f'start lane 0 at ({x0:.2f}, {y_lo:.2f}), nominal '
                          f'half-swath {half:.2f} m')
        return Waypoint(x0, y_lo, self.lane_dir)

    def next_waypoint(self, px, py, yaw, left, right):
        """Called each time the vehicle has reached a waypoint and the sonar
        has been read.

        left / right: lateral offset (m) of the footprint edge on that side
        from extract_edge(), or None if that beam returned nothing.
        Returns the next Waypoint, None when the survey is finished, or (fleet
        mode only) WAIT when a neighbour's edge data is not available yet."""
        s = math.sin(yaw)
        aligned = abs(s) > 0.7           # heading ~ +/-Y, so left/right = W/E
        half = self.nominal_half
        miss = self.missing_lateral if self.missing_lateral else half
        # a beam that saw nothing counts as a measurement of "at least `miss`"
        l_meas = aligned
        r_meas = aligned
        L = (left if left is not None else miss) if aligned else half
        R = (right if right is not None else miss) if aligned else half

        # world x of each edge: body-left (0,+L) -> x = px - sin(yaw) * L
        x_left = px - s * L
        x_right = px + s * R
        if s > 0:           # heading +Y: left is west
            west, east, west_meas = x_left, x_right, l_meas
        else:               # heading -Y: left is east
            west, east, west_meas = x_right, x_left, r_meas
        self.lane_samples.append((py, west, east))

        # -- next position along the lane (clamped to the survey polygon) ------
        y_lo, y_hi = self._y_range(west, east)
        ny = self._lane_clamp(py, py + self.lane_dir * self.step_length, y_lo, y_hi)

        # which neighbour edge this lane lines up against
        if not self.shared:
            ref_y, ref_e, live = self.prev_y, self.prev_east, False
        elif self.leader and self.lane == 0:
            ref_y, ref_e, live = self.prev_y, self.prev_east, False    # survey boundary
        else:
            r = self._ref_store.get(self.lane - 1 if self.leader else self.lane)
            if r is None:
                self.lane_samples.pop()
                return WAIT
            ref_y, ref_e = self._sorted(r)
            live = not r['complete']
        if live and abs(ny - py) >= self.min_leg:
            # follower: the leader must have measured at least as far as the next y
            covered = ref_y[-1] >= ny if self.lane_dir > 0 else ref_y[0] <= ny
            if not covered:
                self.lane_samples.pop()
                return WAIT

        # -- lateral correction so this lane's west edge meets the previous
        #    lane's east edge (minus the stitching overlap). Uses the previous
        #    lane's east edge at the NEXT y (look-ahead), assuming the swath
        #    half-width stays about what we just measured, so the vehicle
        #    arrives roughly on target instead of lagging a waypoint behind.
        dx = 0.0
        gap_txt = 'no west-edge reading'
        if west_meas:
            west_ext_now = px - west
            prev_east_next = float(np.interp(ny, ref_y, ref_e))
            target_x = prev_east_next - self.lane_overlap + west_ext_now
            err = target_x - px
            gap_txt = (f'west edge {west:.2f}, prev east @next y '
                       f'{prev_east_next:.2f}')
            if self.shared and not live and prev_east_next - self.lane_overlap >= self.xmax:
                return self._finish('survey complete: the neighbouring lane already reaches xmax')
            if abs(err) >= self.deadband:
                dx = max(-self.max_correction, min(self.max_correction, err))

        # -- continue along the lane --------------------------------------
        nx = px + dx
        y_lo, y_hi = self._y_range(west + dx, east + dx)
        ny = self._lane_clamp(py, py + self.lane_dir * self.step_length, y_lo, y_hi)
        if abs(ny - py) >= self.min_leg:
            self.last_info = (f'lane {self.lane} | W {west:.2f} E {east:.2f} | '
                              f'{gap_txt} | shift {dx:+.2f} m')
            self._queue(False, False)
            return Waypoint(nx, ny, self.lane_dir)

        # -- lane finished: decide whether the survey is done -----------------
        lane_min_east = min(e for _, _, e in self.lane_samples)
        if lane_min_east >= self.xmax:
            return self._finish(f'survey complete: lane {self.lane} east edge '
                                f'(min {lane_min_east:.2f}) reached xmax {self.xmax:.2f}')
        if self.lane + 1 >= self.max_lanes:
            return self._finish(f'stopping: reached max_lanes={self.max_lanes}')

        west_ext = max(px - west, 0.1)
        east_ext = max(east - px, 0.1)

        if self.shared:
            # -- fleet: the next lane lines up against a neighbour's edge --------
            nl = self.lane + 1
            if self._sent_complete != self.lane:        # publish BEFORE waiting, or two drones
                self._queue(True, False)                # waiting on each other deadlock
                self._sent_complete = self.lane
            r = self._ref_store.get(self.lane if self.leader else nl)
            # leader: last drone's lane just finished; follower: previous drone's NEW lane
            ready = r is not None and (r['complete'] if self.leader else True)
            if not ready:
                if (not self.leader and r is None and self._ref_last_lane is not None
                        and self._ref_last_lane < nl):
                    return self._finish('survey complete: the drone ahead of me has finished')
                if (self.leader and r is None and self._ref_last_lane is not None
                        and self._ref_last_lane < self.lane):
                    return self._finish('survey complete: the last drone has finished')
                # Don't sit idle while the neighbour flies to its new lane: head for the
                # place the new lane will be (fleet_size lanes east); the live edge
                # alignment trims it as soon as the neighbour's readings arrive.
                # (never longer than the nominal swath: too short is trimmed by the live
                # alignment, too far east would leave a gap)
                est = px + self.fleet_size * (min(west_ext + east_ext, 2 * half) - self.lane_overlap) - 1.0
                if self.provisional_jump and est - west_ext < self.xmax - 1.0:
                    ry = re = None
                    new_x = est
                else:
                    self.lane_samples.pop()
                    return WAIT
            new_dir = -self.lane_dir
            if r is not None and ready:
                ry, re = self._sorted(r)
                new_x, new_y, tw = px, py, None
                for _ in range(3):
                    tw = float(np.interp(new_y, ry, re)) - self.lane_overlap
                    new_x = tw + west_ext
                    n_lo, n_hi = self._y_range(new_x - west_ext, new_x + east_ext)
                    new_y = n_hi if new_dir < 0 else n_lo
                if tw >= self.xmax:
                    return self._finish(f'survey complete: next lane would start at x={tw:.2f}, '
                                        f'beyond xmax {self.xmax:.2f}')
            new_x = min(new_x, self.xmax + half)
        else:
            new_x = px + east_ext + west_ext - self.lane_overlap
            new_x = min(new_x, self.xmax + half)
            if new_x - px < 0.5:
                return self._finish('stopping: next lane would not advance')
            ordered = sorted(self.lane_samples)
            self.prev_y = np.array([o[0] for o in ordered])
            self.prev_east = np.array([o[2] for o in ordered])

        self.lane_samples = []
        self._lane_lo = self._lane_hi = None
        self.lane += 1
        self.lane_dir = -self.lane_dir
        # start the next lane at the polygon edge over ITS swath, not at the
        # y where the last lane happened to stop
        n_lo, n_hi = self._y_range(new_x - west_ext, new_x + east_ext)
        new_y = n_hi if self.lane_dir < 0 else n_lo
        self.last_info = (f'lane {self.lane - 1} done (swath W {west_ext:.2f} + '
                          f'E {east_ext:.2f}); jumping to lane {self.lane} at '
                          f'x={new_x:.2f}, y={new_y:.2f}')
        return Waypoint(new_x, new_y, self.lane_dir)

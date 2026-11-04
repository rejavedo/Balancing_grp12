# TASKS 01, 05, 07: ROS-independent safety, search and localization algorithms.
"""ROS-independent geometry, coverage and temporal-consensus primitives."""
import math
from collections import deque
import numpy as np
from scipy.ndimage import distance_transform_edt, label


# TASK 07: Camera orientation transformation.
def rotation(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


# TASK 07: Optical axes (x right, y down, z forward) -> body-style camera_link (x forward, y left, z up).
OPTICAL_TO_LINK = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


# TASKS 03, 07: Compose two (R, t) transforms: a<-b then b<-c gives a<-c.
def compose(R_ab, t_ab, R_bc, t_bc):
    return R_ab @ R_bc, R_ab @ np.asarray(t_bc, dtype=float) + np.asarray(t_ab, dtype=float)


# TASK 07: Camera ray / ground-plane intersection.
def ground_hit(ray, origin, rot, ground_z=0., max_range=4., min_down=0.10):
    """Intersect a camera optical ray with the map horizontal ground plane."""
    ray = np.asarray(ray, dtype=float)
    d = rot @ (ray / np.linalg.norm(ray))
    origin = np.asarray(origin, dtype=float)
    if not np.isfinite(d).all() or d[2] >= -min_down:
        return None
    t = (ground_z-origin[2])/d[2]
    point = origin+t*d
    if t <= 0 or np.linalg.norm(point[:2]-origin[:2]) > max_range:
        return None
    return point


# TASK 05: Reachable map viewpoint generation.
def _start_seed(free, transit, local, resolution, clearance):
    """Connect the continuous start to a nearby transit cell without crossing obstacles."""
    height, width = free.shape
    extent = np.array([width, height]) * resolution
    # A one-cell neighbourhood plus the required clearance covers every candidate link.
    reach = clearance + 2 * resolution
    lo = np.maximum(0, np.floor((local - reach) / resolution).astype(int))
    hi = np.minimum([width, height], np.ceil((local + reach) / resolution).astype(int))
    if np.any(local < 0) or np.any(local >= extent):
        raise ValueError('Start is outside the saved map; check initial pose and map origin')
    rows, cols = np.where(~free[lo[1]:hi[1], lo[0]:hi[0]])
    obstacle_lo = (np.column_stack((cols, rows)) + lo) * resolution
    obstacle_hi = obstacle_lo + resolution

    def margin(end):
        # Bound the entire link by its axis-aligned rectangle in map-local coordinates.
        low, high = np.minimum(local, end), np.maximum(local, end)
        border = min(low.min(), (extent - high).min())
        if not len(obstacle_lo):
            return border
        gap = np.maximum(0, np.maximum(obstacle_lo - high, low - obstacle_hi))
        return min(border, np.linalg.norm(gap, axis=1).min())

    if margin(local) <= clearance:
        raise ValueError(f'Start violates physical transit clearance of {clearance:.3f} m; '
                         'check start pose against walls, unknown cells and map boundary')
    ix, iy = np.floor(local / resolution).astype(int)
    candidates = [(y, x) for y in range(max(0, iy-1), min(height, iy+2))
                  for x in range(max(0, ix-1), min(width, ix+2)) if transit[y, x]]
    def centre(cell):
        return (np.array(cell[::-1]) + .5) * resolution
    for cell in sorted(candidates, key=lambda cell: np.linalg.norm(centre(cell) - local)):
        if margin(centre(cell)) > clearance:
            return cell
    raise ValueError('Start has clearance but no safe connection to a neighbouring transit cell')


def coverage(data, resolution, origin, start, clearance, spacing,
             transit_clearance=None, max_seed_distance=1.0):
    """Erode free map, retain start component, greedily cover its cell centres.

    origin=(x,y,yaw). Unknown and occupied cells are both forbidden.
    Goals cover free cell centres to within spacing, not visibility behind objects.
    """
    if min(resolution, clearance, spacing) <= 0:
        raise ValueError('resolution/clearance/spacing must be positive')
    transit_clearance = clearance if transit_clearance is None else transit_clearance
    if not 0 < transit_clearance <= clearance:
        raise ValueError('transit_clearance must be positive and <= viewpoint clearance')
    free = np.asarray(data) == 0
    # Pad outside map as occupied; scipy's implicit boundary is not sufficient.
    distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1,1:-1]
    safe = free & (distance*resolution > clearance + resolution/math.sqrt(2))
    c, s = math.cos(origin[2]), math.sin(origin[2])
    R = np.array([[c,-s],[s,c]])
    local = R.T @ (np.asarray(start)-origin[:2])
    transit = free & (distance*resolution > transit_clearance + resolution/math.sqrt(2))
    iy, ix = _start_seed(free, transit, local, resolution, transit_clearance)
    labels, _ = label(transit)  # Never snap across walls or unknown cells.
    rows, cols = np.where(safe & (labels == labels[iy,ix]))
    if len(rows) == 0:
        raise ValueError('No reachable viewpoint with requested clearance')
    pts = np.column_stack((cols+.5, rows+.5))*resolution
    pts = pts @ R.T + origin[:2]
    if np.min(np.linalg.norm(pts-np.asarray(start), axis=1)) > max_seed_distance:
        raise ValueError('Nearest reachable viewpoint exceeds max_seed_distance')
    remaining = np.ones(len(pts), dtype=bool)
    current = np.asarray(start)
    goals = []
    while remaining.any():
        ids = np.flatnonzero(remaining)
        k = ids[np.argmin(np.linalg.norm(pts[ids]-current, axis=1))]
        current = pts[k]
        goals.append(tuple(current))
        remaining[np.linalg.norm(pts-current, axis=1) <= spacing] = False
    return goals


class Consensus:
    def __init__(self, count=4, radius=.20, window=6.):
        self.count, self.radius, self.window = count, radius, window
        self.samples = deque()
        self.last_stamp = -math.inf

    def add(self, stamp, point):
        if stamp <= self.last_stamp or not np.isfinite(point).all():
            return None
        self.last_stamp = stamp
        while self.samples and stamp-self.samples[0][0] > self.window:
            self.samples.popleft()
        if self.samples:
            centre = np.median([p for _, p in self.samples], axis=0)
            if np.linalg.norm(np.asarray(point)[:2]-centre[:2]) > self.radius:
                self.samples.clear()
        self.samples.append((stamp, np.asarray(point)))
        if len(self.samples) < self.count:
            return None
        result = np.median([p for _, p in self.samples], axis=0)
        return result if all(np.linalg.norm(p[:2]-result[:2]) <= self.radius
                             for _, p in self.samples) else None


# TASK 01: Velocity and acceleration limits.
def limited_twist(v, w, previous, dt, max_v=.15, max_w=.30, a=.15, alpha=.4):
    if not all(math.isfinite(x) for x in (v,w,dt)):
        return (0.,0.)
    v, w = max(0., min(max_v,v)), max(-max_w,min(max_w,w))
    return (float(np.clip(v, previous[0]-a*dt, previous[0]+a*dt)),
            float(np.clip(w, previous[1]-alpha*dt, previous[1]+alpha*dt)))


# TASK 05: Viewing headings from camera field of view.
def heading_count(minimum, width, fx, overlap=.15):
    if minimum < 4 or width <= 0 or fx <= 0 or not 0 <= overlap < 1:
        raise ValueError('Invalid camera FOV/heading parameters')
    fov = 2*math.atan(width/(2*fx))
    return max(minimum, math.ceil(2*math.pi/(fov*(1-overlap))))


# TASKS 01, 11: Sensor quality diagnostics.
def scan_quality(ranges, range_min, range_max, angle_increment):
    """Describe invalid data without reinterpreting missing returns as free space."""
    a=np.asarray(ranges, dtype=float)
    valid=np.isfinite(a) & (a > 0) & (a >= range_min) & (a <= range_max)
    no_return=np.isposinf(a)
    bad=~(valid | no_return)
    longest=run=0
    for x in np.concatenate([bad,bad]):
        run=run+1 if x else 0
        longest=max(longest,run)
    longest=min(longest,len(a))
    return dict(count=len(a),finite_count=int(valid.sum()),zeros=int((a==0).sum()),
                nan=int(np.isnan(a).sum()),positive_inf=int(no_return.sum()),
                invalid_fraction=float(bad.mean()) if len(a) else 1.,
                longest_invalid_gap_deg=math.degrees(longest*abs(angle_increment)),
                min_range=float(a[valid].min()) if valid.any() else None)


# TASK 01: Validate lidar before enabling movement.
def checked_scan(ranges, range_min, range_max, angle_increment,
                 max_invalid_fraction=0., max_invalid_gap_deg=0.):
    """Optionally bridge only small invalid gaps using nearest bracket ranges.

    Strict by default. Brackets must both be finite. Missing returns are NEVER
    treated as +inf. Even finite interpolation needs validation on the real lidar.
    """
    a=np.asarray(ranges,dtype=float).copy()
    q=scan_quality(a,range_min,range_max,angle_increment)
    if q['count'] < 10 or q['finite_count'] < 10:
        return None,'insufficient finite returns'
    if not 6.0 <= abs(angle_increment)*len(a) <= 6.5:
        return None,'expected full-circle scan'
    if q['invalid_fraction'] > max_invalid_fraction or q['longest_invalid_gap_deg'] > max_invalid_gap_deg:
        return None,'invalid scan fraction/gap exceeds configured limit'
    good=np.isfinite(a) & (a>0) & (a>=range_min) & (a<=range_max)
    bad=~(good | np.isposinf(a))
    for i in np.flatnonzero(bad):
        left=(i-1)%len(a); right=(i+1)%len(a)
        while bad[left]: left=(left-1)%len(a)
        while bad[right]: right=(right+1)%len(a)
        if not good[left] or not good[right]:
            return None,'invalid gap has no finite range brackets'
        a[i]=min(a[left],a[right])
    return a,'ok'

# TASKS 01, 05, 07: ROS-independent safety, search and localization algorithms.
"""ROS-independent geometry, coverage and temporal-consensus primitives."""
import math
from collections import deque
import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


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
    ix, iy = np.floor(local/resolution).astype(int)
    transit = free & (distance*resolution > transit_clearance + resolution/math.sqrt(2))
    if not (0 <= iy < transit.shape[0] and 0 <= ix < transit.shape[1] and transit[iy,ix]):
        raise ValueError('Start violates physical transit clearance; check footprint/pose')
    labels, _ = label(transit)  # Never snap across walls or unknown cells.
    rows, cols = np.where(safe & (labels == labels[iy,ix]))
    if len(rows) == 0:
        raise ValueError('No reachable viewpoint with requested clearance')
    pts = np.column_stack((cols+.5, rows+.5))*resolution
    pts = pts @ R.T + origin[:2]
    if np.min(np.linalg.norm(pts-np.asarray(start), axis=1)) > max_seed_distance:
        raise ValueError('Nearest reachable viewpoint exceeds max_seed_distance')
    # Path (geodesic) distances inside the reachable component: order and spacing follow the
    # corridors the robot drives, not straight lines through walls.
    region = labels == labels[iy, ix]
    graph, node = _grid_graph(region, resolution)
    cand = np.zeros(graph.shape[0], dtype=bool); cand[node[rows, cols]] = True
    room = np.zeros(graph.shape[0]); room[node[rows, cols]] = distance[rows, cols]
    xy = np.zeros((graph.shape[0], 2)); r_all, c_all = np.where(region)
    xy[node[r_all, c_all]] = (np.column_stack((c_all+.5, r_all+.5))*resolution) @ R.T + origin[:2]
    remaining = cand.copy()
    current = node[iy, ix]
    goals = []
    while remaining.any():
        d = dijkstra(graph, directed=False, indices=current)
        reach = remaining & np.isfinite(d)
        if not reach.any():
            break
        ids = np.flatnonzero(reach)
        k = ids[np.argmin(d[ids])]
        # Centre the viewpoint: most room among remaining candidates within spacing/2 (path) of k.
        dk = dijkstra(graph, directed=False, indices=k, limit=spacing/2)
        near = np.flatnonzero(remaining & (dk <= spacing/2))
        k = near[np.lexsort((dk[near], -room[near]))[0]]
        goals.append(tuple(xy[k]))
        remaining[dijkstra(graph, directed=False, indices=k, limit=spacing) <= spacing] = False
        current = k
    return goals


def _grid_graph(region, resolution):
    """Sparse graph of a cell mask: 4-neighbours, plus diagonals only when both sides are free."""
    node = -np.ones(region.shape, dtype=int)
    node[region] = np.arange(int(region.sum()))
    h, w = region.shape
    src, dst, cost = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = region[0:h-dr, max(0, -dc):w-max(0, dc)]
        b = region[dr:h, max(0, dc):w-max(0, -dc)]
        ok = a & b
        if dr and dc:  # no corner cutting
            ok &= region[0:h-dr, max(0, dc):w-max(0, -dc)] & region[dr:h, max(0, -dc):w-max(0, dc)]
        r, c = np.nonzero(ok)
        src.append(node[0:h-dr, max(0, -dc):w-max(0, dc)][r, c])
        dst.append(node[dr:h, max(0, dc):w-max(0, -dc)][r, c])
        cost.append(np.full(len(r), resolution*math.hypot(dr, dc)))
    src, dst, cost = map(np.concatenate, (src, dst, cost))
    n = int(region.sum())
    return csr_matrix((cost, (src, dst)), shape=(n, n)), node


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

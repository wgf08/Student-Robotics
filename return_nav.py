"""
return_nav.py — GPS-based navigation for return-to-zone.

Camera note
───────────
The camera is mounted UPSIDE DOWN. This flips the sign of horizontal_angle,
vertical_angle, and orientation.yaw from what you would read on a right-way-up
camera. All angle reads go through the helpers _ha(), _va(), _yaw() which apply
the correction in one place. If the camera is ever remounted right-way-up, set
CAMERA_UPSIDE_DOWN = False and nothing else needs changing.

Single-marker GPS derivation
─────────────────────────────
Each wall marker has a known world position (from COORDINATES × MARKER_INTERVAL)
and a known facing direction (MARKER_FACING[id]) — the inward normal of that wall.

Given one marker observation:
  ha  = corrected horizontal angle  (positive = marker is to the robot's right)
  yaw = corrected marker yaw        (how much the face is rotated from face-on)
  d   = horizontal distance to marker (distance × cos(vertical_angle))

When the robot looks at the marker perfectly face-on (yaw=0, ha=0) the robot's
heading is exactly opposite the marker normal:
  Robot_Bearing = MARKER_FACING[id] + π

With yaw ≠ 0 the marker face is rotated by `yaw` relative to the camera axis,
shifting the implied direction from robot → marker by the same amount.
With ha ≠ 0 the marker is off to one side; the world direction from robot to
marker is Robot_Bearing − ha.

Combining:
  world_dir_to_marker  =  MARKER_FACING[id] + π + yaw   (from geometry)
  world_dir_to_marker  =  Robot_Bearing − ha             (from camera)
  ∴ Robot_Bearing      =  MARKER_FACING[id] + π + yaw + ha

Robot position then follows directly:
  Robot_X = marker_world_x − d·cos(world_dir_to_marker)
  Robot_Y = marker_world_y − d·sin(world_dir_to_marker)

Two-marker GPS (more accurate when available)
─────────────────────────────────────────────
Uses the law of cosines to triangulate from two distance measurements, then
derives bearing from the direction to marker A plus its horizontal angle.
The single-marker method is used as the tie-breaker when both intersection
points appear to be inside the arena.
"""

from utility  import *          # sorted_boxes, find_marker, …
from movement import *          # halt, move_straight, spin, rotate_angle, move_angle
from info     import *          # ZONE_BOUNDARIES, PLATFORM_BOUNDARIES, MARKER_FACING, …
import math, heapq, time
import numpy as np              # type: ignore

# ─────────────────────────────────────────────────────────────────────────────
# Camera orientation flag
# ─────────────────────────────────────────────────────────────────────────────

CAMERA_UPSIDE_DOWN = True   # Set False if camera is ever remounted right-way-up


def _ha(m):
    """Horizontal angle, corrected for camera mount orientation."""
    return -m.position.horizontal_angle if CAMERA_UPSIDE_DOWN else m.position.horizontal_angle


def _va(m):
    """Vertical angle, corrected for camera mount orientation."""
    return -m.position.vertical_angle if CAMERA_UPSIDE_DOWN else m.position.vertical_angle


def _yaw(m):
    """Marker orientation yaw, corrected for camera mount orientation."""
    return -m.orientation.yaw if CAMERA_UPSIDE_DOWN else m.orientation.yaw


def _horiz_dist(m):
    """Ground-plane distance to marker (strips out vertical component)."""
    return m.position.distance * math.cos(_va(m))


# ─────────────────────────────────────────────────────────────────────────────
# Arena geometry
# ─────────────────────────────────────────────────────────────────────────────

MARKER_INTERVAL = 762.5         # mm — arena width / 6
ARENA_SIZE      = 4575          # mm — total arena side length
ROBOT_RADIUS    = 150           # mm — half robot width (obstacle padding)
MARKER_WIDTH    = 130           # mm — fiducial marker face width

# Marker grid positions. Index = marker id (0-19), value = (grid_col, grid_row).
# Origin is the BOTTOM-LEFT corner of the arena.
COORDINATES = [
    (1, 6), (2, 6), (3, 6), (4, 6), (5, 6),   # 0-4   top wall    (faces south  = 3pi/2)
    (6, 5), (6, 4), (6, 3), (6, 2), (6, 1),   # 5-9   right wall  (faces west   = pi  )
    (5, 0), (4, 0), (3, 0), (2, 0), (1, 0),   # 10-14 bottom wall (faces north  = pi/2)
    (0, 1), (0, 2), (0, 3), (0, 4), (0, 5),   # 15-19 left wall   (faces east   = 0   )
]

# NOTE: MARKER_FACING is imported from info.py. Check it maps:
#   ids  0- 4  ->  3*pi/2  top wall,   faces south
#   ids  5- 9  ->  pi      right wall, faces west    (info.py currently has 0.0 — likely wrong)
#   ids 10-14  ->  pi/2    bottom wall, faces north
#   ids 15-19  ->  0.0     left wall,   faces east   (info.py currently has pi/2 — likely wrong)

# ─────────────────────────────────────────────────────────────────────────────
# Tunable constants
# ─────────────────────────────────────────────────────────────────────────────

ERROR_MARGIN               = 100    # mm
ANGLE_CORRECTION_THRESHOLD = 0.05   # rad
DRIVE_BURST_TIME           = 0.4    # s
DRIVE_POWER                = 0.6
SPIN_SCAN_POWER            = 0.25
SPIN_SCAN_STEP             = 0.35   # s per scan tick
MAX_SCAN_STEPS             = 32
KINEM_SPEED_MM_S           = 600.0  # mm/s at power=1; tune on your robot
REPLAN_THRESHOLD           = 300    # mm
MAX_WAYPOINT_ITERATIONS    = 60
WAYPOINT_TOLERANCE         = 250    # mm
EPS                        = 1e-6
NUDGE                      = 2      # mm

# ─────────────────────────────────────────────────────────────────────────────
# Module-level state
# ─────────────────────────────────────────────────────────────────────────────

_robot        = None
Robot_X       = 0.0
Robot_Y       = 0.0
Robot_Bearing = 0.0     # radians, world frame

Sample_Data   = {}
Acid_Data     = {}
Base_Data     = {}


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

class AABB:
    __slots__ = ('MinX', 'MinY', 'MaxX', 'MaxY')

    def __init__(self, MinX, MinY, MaxX, MaxY):
        self.MinX = float(MinX); self.MinY = float(MinY)
        self.MaxX = float(MaxX); self.MaxY = float(MaxY)

    def __repr__(self):
        return f"AABB([{self.MinX:.0f},{self.MinY:.0f}]->[{self.MaxX:.0f},{self.MaxY:.0f}])"


def _dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def _marker_world_pos(marker_id):
    gx, gy = COORDINATES[marker_id]
    return gx * MARKER_INTERVAL, gy * MARKER_INTERVAL


def _inside_arena(x, y, margin=ERROR_MARGIN):
    return -margin <= x <= ARENA_SIZE+margin and -margin <= y <= ARENA_SIZE+margin


def _wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


# ─────────────────────────────────────────────────────────────────────────────
# Camera helpers
# ─────────────────────────────────────────────────────────────────────────────

def _get_markers():
    all_m  = _robot.camera.see()
    unique = list({m.id: m for m in all_m}.values())
    return sorted_boxes(unique)     # [samples, acids, bases, walls]


# ─────────────────────────────────────────────────────────────────────────────
# ScanForMarkers — rotate until >= 1 wall marker visible
# ─────────────────────────────────────────────────────────────────────────────

def ScanForMarkers(motors):
    """
    Rotate slowly until at least ONE wall marker is visible, then halt.
    One marker is sufficient for a full position + bearing fix.

    Returns list of visible wall markers (length >= 1).
    Raises RuntimeError if none found after MAX_SCAN_STEPS.
    """
    print("ScanForMarkers: searching for wall markers...")

    for step in range(MAX_SCAN_STEPS):
        _, _, _, walls = _get_markers()
        if walls:
            halt(motors)
            print(f"  {len(walls)} wall marker(s) found after {step} step(s)")
            return walls
        spin(motors, SPIN_SCAN_POWER, clockwise=True, duration=SPIN_SCAN_STEP)
        halt(motors)
        time.sleep(0.05)

    halt(motors)
    raise RuntimeError("ScanForMarkers: no wall markers found — view may be obstructed.")


# ─────────────────────────────────────────────────────────────────────────────
# GPS — single-marker  (always available with >= 1 wall marker)
# ─────────────────────────────────────────────────────────────────────────────

def _gps_single(marker):
    """
    Full position + bearing fix from ONE wall marker.

    world_dir_to_marker = MARKER_FACING[id] + pi + yaw
    Robot_Bearing       = world_dir_to_marker + ha
    Robot position      = marker_world_pos - d * (cos, sin)(world_dir_to_marker)
    """
    global Robot_X, Robot_Y, Robot_Bearing

    mid = marker.id
    if mid not in MARKER_FACING:
        return False

    ha   = _ha(marker)
    yaw  = _yaw(marker)
    d    = _horiz_dist(marker)

    world_dir = _wrap(MARKER_FACING[mid] + math.pi + yaw)
    Robot_Bearing = _wrap(world_dir + ha)

    mx, my = _marker_world_pos(mid)
    Robot_X = mx - d * math.cos(world_dir)
    Robot_Y = my - d * math.sin(world_dir)

    print(f"GPS(1 marker #{mid}) -> ({Robot_X:.0f}, {Robot_Y:.0f}) "
          f"bearing {math.degrees(Robot_Bearing):.1f}deg")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# GPS — two-marker triangulation  (more accurate when >= 2 markers visible)
# ─────────────────────────────────────────────────────────────────────────────

def _gps_two(ma, mb):
    """
    Triangulate from two wall markers using the law of cosines.
    Uses the single-marker estimate to resolve the two-solution ambiguity.
    """
    global Robot_X, Robot_Y, Robot_Bearing

    xa, ya  = _marker_world_pos(ma.id)
    xb, yb  = _marker_world_pos(mb.id)
    dist_ra = _horiz_dist(ma)
    dist_rb = _horiz_dist(mb)
    dist_ab = _dist((xa,ya), (xb,yb))
    if dist_ab < EPS:
        return False

    angle_ab = math.atan2(yb-ya, xb-xa)
    cos_rab  = max(-1.0, min(1.0,
        (dist_ra**2 + dist_ab**2 - dist_rb**2) / (2*dist_ra*dist_ab)))
    angle_rab = math.acos(cos_rab)

    p1 = (xa + dist_ra*math.cos(angle_ab+angle_rab),
          ya + dist_ra*math.sin(angle_ab+angle_rab))
    p2 = (xa + dist_ra*math.cos(angle_ab-angle_rab),
          ya + dist_ra*math.sin(angle_ab-angle_rab))

    p1_ok, p2_ok = _inside_arena(*p1), _inside_arena(*p2)

    if p1_ok and not p2_ok:
        rx, ry = p1
    elif p2_ok and not p1_ok:
        rx, ry = p2
    else:
        # Both/neither inside — use single-marker result as tiebreaker
        _gps_single(ma)
        sm_x, sm_y = Robot_X, Robot_Y
        rx, ry = p1 if _dist(p1,(sm_x,sm_y)) < _dist(p2,(sm_x,sm_y)) else p2

    Robot_X, Robot_Y = rx, ry
    bearing_to_a  = math.atan2(ya-ry, xa-rx)
    Robot_Bearing = _wrap(bearing_to_a + _ha(ma))

    print(f"GPS(2 markers #{ma.id}+#{mb.id}) -> ({Robot_X:.0f}, {Robot_Y:.0f}) "
          f"bearing {math.degrees(Robot_Bearing):.1f}deg")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# GPS — public dispatcher
# ─────────────────────────────────────────────────────────────────────────────

def GPS(wall_markers):
    """
    Best-available GPS fix. Two markers -> triangulation; one -> single-marker.
    Returns False if the list is empty.
    """
    if not wall_markers:
        return False
    if len(wall_markers) >= 2:
        return _gps_two(wall_markers[0], wall_markers[1])
    return _gps_single(wall_markers[0])


# ─────────────────────────────────────────────────────────────────────────────
# UpdatePosition  (GPS or dead-reckoning)
# ─────────────────────────────────────────────────────────────────────────────

def UpdatePosition(motors, last_bearing=None, elapsed_drive_s=0.0,
                   drive_power=DRIVE_POWER):
    """
    Try GPS from any visible wall markers (1 is enough).
    Falls back to dead reckoning if none are visible.
    Returns True if GPS succeeded.
    """
    global Robot_X, Robot_Y, Robot_Bearing

    _, _, _, walls = _get_markers()
    if walls:
        return GPS(walls)

    if last_bearing is not None and elapsed_drive_s > 0:
        d       = KINEM_SPEED_MM_S * drive_power * elapsed_drive_s
        Robot_X += d * math.cos(last_bearing)
        Robot_Y += d * math.sin(last_bearing)
        print(f"Dead reckon +{d:.0f} mm -> ({Robot_X:.0f}, {Robot_Y:.0f})")
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Box localisation
# ─────────────────────────────────────────────────────────────────────────────

def _get_corners(centre, width, angle):
    hw, hd = width/2, width/4
    local  = np.array([[-hw,-hd],[hw,-hd],[hw,hd],[-hw,hd]])
    c, s   = math.cos(angle), math.sin(angle)
    rot    = np.array([[c,-s],[s,c]])
    return np.dot(local, rot.T) + centre


def _in_zone(pos):
    for zid, bounds in ZONE_BOUNDARIES.items():
        xs = [b[0] for b in bounds]; ys = [b[1] for b in bounds]
        if min(xs) < pos[0] < max(xs) and min(ys) < pos[1] < max(ys):
            return zid
    return -1


def LocateBoxes(marker_list, existing_dict):
    for m in marker_list:
        d            = _horiz_dist(m)
        ha           = _ha(m)
        yaw          = _yaw(m)
        global_angle = Robot_Bearing - ha
        mx = Robot_X + d * math.cos(global_angle)
        my = Robot_Y + d * math.sin(global_angle)
        global_yaw   = _wrap((Robot_Bearing + math.pi) - yaw)
        corners      = _get_corners(np.array([mx,my]), MARKER_WIDTH, global_yaw)
        cmin, cmax   = corners.min(axis=0), corners.max(axis=0)
        aabb         = AABB(cmin[0], cmin[1], cmax[0], cmax[1])
        existing_dict[m.id] = (round(mx,1), round(my,1),
                               round(global_yaw,3), _in_zone([mx,my]), aabb)
    return existing_dict


# ─────────────────────────────────────────────────────────────────────────────
# Path planner  (Dijkstra visibility graph)
# ─────────────────────────────────────────────────────────────────────────────

def _get_aabbs(padding):
    boxes = []
    for data in Sample_Data.values():
        b = data[4]
        boxes.append(AABB(max(0,b.MinX-padding), max(0,b.MinY-padding),
                          min(ARENA_SIZE,b.MaxX+padding), min(ARENA_SIZE,b.MaxY+padding)))
    px0, py0, px1, py1 = PLATFORM_BOUNDARIES
    boxes.append(AABB(max(0,px0-padding), max(0,py0-padding),
                      min(ARENA_SIZE,px1+padding), min(ARENA_SIZE,py1+padding)))
    boxes += [
        AABB(0, 0, ARENA_SIZE, ROBOT_RADIUS),
        AABB(0, ARENA_SIZE-ROBOT_RADIUS, ARENA_SIZE, ARENA_SIZE),
        AABB(0, 0, ROBOT_RADIUS, ARENA_SIZE),
        AABB(ARENA_SIZE-ROBOT_RADIUS, 0, ARENA_SIZE, ARENA_SIZE),
    ]
    return boxes


def _seg_blocked(a, b, obs, skip=frozenset()):
    for i, box in enumerate(obs):
        if i in skip:
            continue
        dx, dy = b[0]-a[0], b[1]-a[1]
        tmin, tmax = 0.0, 1.0
        ok = True
        for d, lo, hi, ai in ((dx,box.MinX,box.MaxX,0),(dy,box.MinY,box.MaxY,1)):
            if abs(d) < EPS:
                if not (lo <= a[ai] <= hi): ok = False; break
            else:
                ta, tb = (lo-a[ai])/d, (hi-a[ai])/d
                tmin = max(tmin, min(ta,tb)); tmax = min(tmax, max(ta,tb))
        if ok and tmax >= tmin and tmax > 0 and tmin < 1:
            return True
    return False


def _on_boundary(p, box):
    on_x = abs(p[0]-box.MinX)<EPS or abs(p[0]-box.MaxX)<EPS
    on_y = abs(p[1]-box.MinY)<EPS or abs(p[1]-box.MaxY)<EPS
    in_x = box.MinX-EPS <= p[0] <= box.MaxX+EPS
    in_y = box.MinY-EPS <= p[1] <= box.MaxY+EPS
    return in_x and in_y and (on_x or on_y)


def _boxes_for(p, obs):
    return frozenset(i for i,b in enumerate(obs) if _on_boundary(p,b))


def _nudged_corners(box):
    n = NUDGE
    return [(box.MinX-n,box.MinY-n),(box.MaxX+n,box.MinY-n),
            (box.MaxX+n,box.MaxY+n),(box.MinX-n,box.MaxY+n)]


def _inside_any(p, obs):
    return any(b.MinX+EPS < p[0] < b.MaxX-EPS and
               b.MinY+EPS < p[1] < b.MaxY-EPS for b in obs)


def FindPath(start, goal):
    obs   = _get_aabbs(2*ROBOT_RADIUS)
    nodes = []
    for box in obs:
        for cx,cy in _nudged_corners(box):
            cx = max(0.0, min(float(ARENA_SIZE), cx))
            cy = max(0.0, min(float(ARENA_SIZE), cy))
            if not _inside_any((cx,cy), obs):
                nodes.append((cx,cy))

    si, gi = len(nodes), len(nodes)+1
    nodes += [start, goal]
    n      = len(nodes)
    nb     = [_boxes_for(p,obs) for p in nodes]
    dto    = [math.inf]*n; dto[si] = 0.0
    prev   = [-1]*n; visited = [False]*n
    heap   = [(0.0, si)]

    while heap:
        cd, u = heapq.heappop(heap)
        if visited[u]: continue
        visited[u] = True
        if u == gi: break
        for v in range(n):
            if visited[v]: continue
            if _seg_blocked(nodes[u], nodes[v], obs, nb[u]|nb[v]): continue
            nd = cd + _dist(nodes[u], nodes[v])
            if nd < dto[v]:
                dto[v] = nd; prev[v] = u
                heapq.heappush(heap, (nd,v))

    if dto[gi] == math.inf:
        print("FindPath: no path found"); return None

    path, cur = [], gi
    while cur != -1:
        path.insert(0, nodes[cur]); cur = prev[cur]

    print(f"FindPath: {len(path)} waypoints, {dto[gi]:.0f} mm")
    for p in path: print(f"  ({p[0]:.0f}, {p[1]:.0f})")
    return path


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _zone_centre(zone_id):
    bounds = ZONE_BOUNDARIES[zone_id]
    xs = [b[0] for b in bounds]; ys = [b[1] for b in bounds]
    return (min(xs)+max(xs))/2, (min(ys)+max(ys))/2


# ─────────────────────────────────────────────────────────────────────────────
# Drive to a single waypoint
# ─────────────────────────────────────────────────────────────────────────────

def _navigate_to_waypoint(motors, tx, ty):
    global Robot_X, Robot_Y, Robot_Bearing

    for _ in range(MAX_WAYPOINT_ITERATIONS):
        dx, dy = tx-Robot_X, ty-Robot_Y
        dist   = math.sqrt(dx**2+dy**2)
        if dist < WAYPOINT_TOLERANCE:
            halt(motors); return True

        target_bear = math.atan2(dy, dx)
        bear_err    = _wrap(target_bear - Robot_Bearing)
        if abs(bear_err) > ANGLE_CORRECTION_THRESHOLD:
            rotate_angle(motors, abs(bear_err), 0.5, clockwise=(bear_err>0))
            Robot_Bearing = _wrap(Robot_Bearing + bear_err)

        # --- NEW DYNAMIC BURST LOGIC ---
        MAX_BURST_TIME = 0.6
        
        # 1. Calculate estimated time to the waypoint
        estimated_speed_mm_s = KINEM_SPEED_MM_S * DRIVE_POWER
        time_to_wp = dist / estimated_speed_mm_s
        
        # 2. Add a 15% buffer so we don't fall just short of the waypoint
        padded_time = time_to_wp * 1.15 
        
        # 3. Take the smaller of your custom burst time or the padded calculated time
        dynamic_burst = min(MAX_BURST_TIME, padded_time)

        # 4. Drive!
        move_straight(motors, DRIVE_POWER, forwards=True)
        t0 = time.time()
        time.sleep(dynamic_burst)
        halt(motors)
        elapsed = time.time() - t0
        # -------------------------------

        UpdatePosition(motors, last_bearing=Robot_Bearing,
                       elapsed_drive_s=elapsed, drive_power=DRIVE_POWER)

    halt(motors)
    return False


# ─────────────────────────────────────────────────────────────────────────────
# NavigateToZone  (public API)
# ─────────────────────────────────────────────────────────────────────────────

def NavigateToZone(robot, motors, target_zone):
    """
    Navigate to target_zone.
      1. Scan until >= 1 wall marker visible (1 is enough for a full fix).
      2. GPS fix -> position + bearing.
      3. Plan obstacle-avoiding path with Dijkstra.
      4. Drive waypoints in bursts; dead-reckon between GPS fixes.
      5. Re-plan automatically if drift exceeds REPLAN_THRESHOLD.
    """
    global _robot, Robot_X, Robot_Y, Robot_Bearing, Sample_Data, Acid_Data, Base_Data

    _robot = robot
    Sample_Data = {}; Acid_Data = {}; Base_Data = {}

    ScanForMarkers(motors)

    _, _, _, walls = _get_markers()
    if not GPS(walls):
        raise RuntimeError("NavigateToZone: GPS fix failed after scan.")

    target = _zone_centre(target_zone)
    print(f"NavigateToZone: heading to zone {target_zone} at ({target[0]:.0f},{target[1]:.0f})")
    print(f"  from ({Robot_X:.0f},{Robot_Y:.0f}), bearing {math.degrees(Robot_Bearing):.1f}deg")

    path = FindPath((Robot_X, Robot_Y), target)
    if not path or len(path) < 2:
        raise Exception("NavigateToZone: no path found to target zone.")

    wp_idx = 1
    while wp_idx < len(path):
        wp = path[wp_idx]
        print(f"  waypoint {wp_idx}/{len(path)-1}: ({wp[0]:.0f},{wp[1]:.0f})")

        _navigate_to_waypoint(motors, wp[0], wp[1])

        _, _, _, walls = _get_markers()
        if GPS(walls):
            drift = _dist((Robot_X, Robot_Y), wp)
            print(f"  drift = {drift:.0f} mm")
            if drift > REPLAN_THRESHOLD:
                print("  re-planning route...")
                new_path = FindPath((Robot_X, Robot_Y), target)
                if new_path and len(new_path) >= 2:
                    path = new_path; wp_idx = 1; continue
                else:
                    print("  re-plan failed, continuing")
        else:
            print("  no GPS fix — dead reckoning")

        wp_idx += 1

    halt(motors)
    print(f"NavigateToZone: arrived at zone {target_zone}")

from PIL import Image, ImageTk
import warnings
import math
import heapq
import time
from tkinter import ttk
import tkinter as tk
import os

# Screen defaults (used for image sizing and any layout calculations)
screen_width = 1920
screen_height = 1080

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# NODES & EDGES
nodes = {
    "STAND1a": (835, 218), "STAND1b": (835, 240),
    "STAND2a": (886, 218), "STAND2b": (886, 240),
    "STAND3a": (937, 218), "STAND3b": (937, 240),
    "STAND4a": (987, 218), "STAND4b": (987, 240),
    "STAND5a": (1104, 218), "STAND5b": (1104, 240),
    "STAND6a": (1155, 218), "STAND6b": (1155, 240),
    "STAND7a": (1206, 218), "STAND7b": (1206, 240),
    "STAND8a": (1256, 218), "STAND8b": (1256, 240),
    "STAND1N": (835, 283), "STAND2N": (886, 283),
    "STAND3N": (937, 283), "STAND4N": (987, 283),
    "STAND5N": (1104, 283), "STAND6N": (1155, 283),
    "STAND7N": (1206, 283), "STAND8N": (1256, 283),
    "STAND8N_2": (784, 283), "STAND22N": (1320, 288),
    "STAND19N": (771,227), "STAND21N": (1320,227),
    "STAND20N": (771, 288), "STAND18N": (1307, 283),
    "STAND8A": (784, 348), "STAND8B": (784, 326),
    "STAND9A": (835, 348), "STAND9B": (835, 326),
    "STAND10A": (886, 348), "STAND10B": (886, 326),
    "STAND11A": (937, 348), "STAND11B": (937, 326),
    "STAND12A": (987, 348), "STAND12B": (987, 326),
    "STAND13A": (1045, 348), "STAND13B": (1045, 326),
    "STAND14A": (1104, 348), "STAND14B": (1104, 326),
    "STAND15A": (1155, 348), "STAND15B": (1155, 326),
    "STAND16A": (1206, 348), "STAND16B": (1206, 326),
    "STAND17A": (1256, 348), "STAND17B": (1256, 326),
    "STAND18A": (1307, 348), "STAND18B": (1307, 326),
    "STAND19A": (708, 227), "STAND19B": (730, 227),
    "STAND20A": (708, 288), "STAND20B": (730, 288),
    "STAND21A": (1385, 227), "STAND21B": (1363, 227),
    "STAND22A": (1385, 288), "STAND22B": (1363, 288),
    "AQ": (771, 158), "NQ": (771, 283),
    "AR": (1320, 158), "NR": (1320, 283),
    "AS": (1045, 158), "NS": (1045, 283),
    "TXY_A1": (1820, 158),"A1_HOLD": (1820, 120), "RWY27_A1": (1820, 70), "RWY27_A1_ALIGN": (1760, 70), "RWY09_AirBorne": (1920,70),
    "TXY_B1": (1558, 158),"B1_HOLD": (1558, 120), "RWY27_B1": (1558, 70), "RWY27_B1_EXIT": (1558, 92),
    "TXY_C1": (645, 158),"C1_HOLD": (645, 120), "RWY09_C1": (645, 70), "RWY09_C1_EXIT": (645, 92),
    "TXY_D1": (214, 158),"D1_HOLD": (214, 120), "RWY09_D1": (214, 70),
    "TXY_E1": (105, 158),"E1_HOLD": (105, 120), "RWY09_E1": (105, 70), "RWY09_E1_ALIGN": (165, 70), "RWY27_AirBorne": (0,70),
    "10m9": (20, 280),
    "9m9": (49.285714, 280),
    "8m9": (78.571429, 280),
    "7m9": (107.857143, 280),
    "6m9": (137.142857, 280),
    "5m9": (166.428571, 280),
    "4m9": (195.714286, 280),
    "3m9": (225.0, 280),
    "2m9": (254.285714, 280),
    "1m9": (283.571429, 280),
    "0m9": (312.857143, 280),
    "0m27": (342.142857, 280),
    "1m27": (371.428571, 280),
    "2m27": (400.714286, 280),
    "3m27": (430.0, 280),
    "4m27": (459.285714, 280),
    "5m27": (488.571429, 280),
    "6m27": (517.857143, 280),
    "7m27": (547.142857, 280),
    "8m27": (576.428571, 280),
    "9m27": (605.714286, 280),
    "10m27": (635, 280),
}

RUNWAY_TICK_NODES = [
    "10m9", "9m9", "8m9", "7m9", "6m9", "5m9", "4m9", "3m9", "2m9", "1m9", "0m9",
    "0m27", "1m27", "2m27", "3m27", "4m27", "5m27", "6m27", "7m27", "8m27", "9m27", "10m27",
]

DISABLED_STAND_NUMBERS = {19, 20, 21, 22}
DISABLED_STAND_NODES = {
    f"STAND{stand}{suffix}"
    for stand in DISABLED_STAND_NUMBERS
    for suffix in ("a", "b", "A", "B")
}

edges = {
    "RWY27_A1": ["A1_HOLD","RWY27_B1","RWY27_A1_ALIGN"],
    "RWY27_A1_ALIGN": ["RWY27_AirBorne"],
    "RWY27_B1": ["B1_HOLD","RWY09_C1","RWY27_B1_EXIT"],
    "RWY27_B1_EXIT": ["B1_HOLD"],
    "RWY09_C1": ["C1_HOLD","RWY09_D1","RWY09_C1_EXIT"],
    "RWY09_C1_EXIT": ["C1_HOLD"],
    "RWY09_D1": ["RWY09_E1","D1_HOLD"],
    "RWY09_E1": ["RWY09_E1_ALIGN"],
    "RWY09_E1_ALIGN": ["RWY09_AirBorne"],
    "E1_HOLD": ["RWY09_E1","TXY_E1"],
    "TXY_D1": ["TXY_E1","D1_HOLD"],
    "TXY_C1": ["TXY_D1","AQ","C1_HOLD"],
    "TXY_B1": ["TXY_A1","B1_HOLD"],
    "TXY_A1": ["AR","A1_HOLD"],
    "AQ": ["STAND19N","AS"],
    "STAND19B": ["STAND19A","STAND19N"],
    "NS": ["AS","NQ","NR","STAND1N","STAND2N","STAND3N","STAND4N","STAND5N","STAND6N","STAND7N","STAND8N"],
    "NQ": ["STAND1N","STAND2N","STAND3N","STAND4N","STAND8N_2","STAND20N","STAND19N"],
    "STAND20N": ["STAND20B"],
    "STAND20A": ["STAND20B"],
    "STAND8B": ["STAND8N_2","STAND8A"],
    "NR": ["STAND5N","STAND6N","STAND7N","STAND8N","STAND18N","STAND22N"],
    "AR": ["AS","NR","TXY_B1"],
    "STAND1b": ["STAND1N","STAND1a"],
    "STAND2b": ["STAND2N","STAND2a"],
    "STAND3b": ["STAND3N","STAND3a"],
    "STAND4b": ["STAND4N","STAND4a"],
    "STAND5b": ["STAND5N","STAND5a"],
    "STAND6b": ["STAND6N","STAND6a"],
    "STAND7b": ["STAND7N","STAND7a"],
    "STAND8b": ["STAND8N","STAND8a"],
    "STAND9B": ["STAND1N","STAND9A"],
    "STAND10B": ["STAND2N","STAND10A"],
    "STAND11B": ["STAND3N","STAND11A"],
    "STAND12B": ["STAND4N","STAND12A"],
    "STAND13B": ["NS","STAND13A"],
    "STAND14B": ["STAND5N","STAND14A"],
    "STAND15B": ["STAND6N","STAND15A"],
    "STAND16B": ["STAND7N","STAND16A"],
    "STAND17B": ["STAND8N","STAND17A"],
    "STAND18B": ["STAND18N","STAND18A"],
    "STAND21B": ["STAND21N","STAND21A"],
    "STAND22B": ["STAND22N","STAND22A"],
  
    "10m9": ["9m9"],"9m9": ["8m9"],"8m9": ["7m9"],
    "7m9": ["6m9"],"6m9": ["5m9"],"5m9": ["4m9"],
    "4m9": ["3m9"],"3m9": ["2m9"],"2m9": ["1m9"],
    "1m9": ["0m9"],"0m27": ["1m27"],
    "1m27": ["2m27"], "2m27": ["3m27"], "3m27": ["4m27"], 
    "4m27": ["5m27"], "5m27": ["6m27"], "6m27": ["7m27"], 
    "7m27": ["8m27"], "8m27": ["9m27"], "9m27": ["10m27"],
}

# Build bidirectional edges
for node, neighbors in list(edges.items()):
    for neighbor in neighbors:
        if neighbor not in edges:
            edges[neighbor] = []
        if node not in edges[neighbor]:
            edges[neighbor].append(node)

def is_disabled_stand_node(node_name):
    return node_name in DISABLED_STAND_NODES

def dijkstra_avoiding(start, goal, avoid_nodes=None):
    """Return shortest path avoiding specified nodes by filtering edges."""
    if avoid_nodes is None:
        avoid_nodes = set()
    if start not in nodes or goal not in nodes:
        return None
    if is_disabled_stand_node(start) or is_disabled_stand_node(goal):
        return None
    if start in avoid_nodes or goal in avoid_nodes:
        return None

    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]

    while pq:
        cur_cost, node = heapq.heappop(pq)
        if node == goal:
            path = []
            while node in prev:
                path.append(node)
                node = prev[node]
            path.append(start)
            return list(reversed(path))

        if cur_cost > dist.get(node, float('inf')):
            continue

        # Get neighbors for this node, filtering out avoided nodes
        neighbors = edges.get(node, [])
        filtered_neighbors = [n for n in neighbors if n not in avoid_nodes]
        
        for nxt in filtered_neighbors:
            if node not in nodes or nxt not in nodes:
                continue
            if is_disabled_stand_node(node) or is_disabled_stand_node(nxt):
                continue
            edge_cost = math.dist(nodes[node], nodes[nxt])
            new_cost = cur_cost + edge_cost
            if new_cost < dist.get(nxt, float('inf')):
                dist[nxt] = new_cost
                prev[nxt] = node
                heapq.heappush(pq, (new_cost, nxt))

    return None

# ==============================
# DIJKSTRA PATHFINDING
def dijkstra(start, goal):
    """Return shortest path as list of node names from start to goal using Euclidean edge costs."""
    if start not in nodes or goal not in nodes:
        return None
    if is_disabled_stand_node(start) or is_disabled_stand_node(goal):
        return None

    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]

    while pq:
        cur_cost, node = heapq.heappop(pq)
        if node == goal:
            path = []
            while node in prev:
                path.append(node)
                node = prev[node]
            path.append(start)
            return list(reversed(path))

        if cur_cost > dist.get(node, float('inf')):
            continue

        for nxt in edges.get(node, []):
            if node not in nodes or nxt not in nodes:
                continue
            if is_disabled_stand_node(node) or is_disabled_stand_node(nxt):
                continue
            edge_cost = math.dist(nodes[node], nodes[nxt])
            new_cost = cur_cost + edge_cost
            if new_cost < dist.get(nxt, float('inf')):
                dist[nxt] = new_cost
                prev[nxt] = node
                heapq.heappush(pq, (new_cost, nxt))

    return None

def build_route_via_nodes(start, goal, via_nodes=None, avoid_nodes=None):
    """Build a full route that must pass through via_nodes in order."""
    via_nodes = list(via_nodes or [])
    checkpoints = [start] + via_nodes + [goal]
    full_route = []

    for i in range(len(checkpoints) - 1):
        segment_start = checkpoints[i]
        segment_goal = checkpoints[i + 1]
        if avoid_nodes:
            segment = dijkstra_avoiding(segment_start, segment_goal, avoid_nodes)
        else:
            segment = dijkstra(segment_start, segment_goal)
        if not segment:
            return None

        if full_route:
            full_route.extend(segment[1:])
        else:
            full_route.extend(segment)

    return full_route

# ==============================
# AIRCRAFT CLASS
class Aircraft:
    def __init__(self, callsign, node):
        self.callsign = callsign
        self.node = node
        self.coords = nodes.get(node) if node in nodes else (0, 0)
        self.triangle_id = None
        self.label_id = None
        self.route = []
        self.spline_points = []
        self.spline_route_idx_map = []
        self.dist_along_path = 0.0
        self.waiting_for_stopbar = False
        self.status = "At Stand"


def catmull_rom_spline(P0, P1, P2, P3, n_points=20):
    """Generate Catmull-Rom spline points."""
    points = []
    for i in range(n_points):
        t = i / n_points
        t2 = t * t
        t3 = t2 * t
        x = 0.5 * ((2*P1[0]) + (-P0[0]+P2[0])*t + (2*P0[0]-5*P1[0]+4*P2[0]-P3[0])*t2 + (-P0[0]+3*P1[0]-3*P2[0]+P3[0])*t3)
        y = 0.5 * ((2*P1[1]) + (-P0[1]+P2[1])*t + (2*P0[1]-5*P1[1]+4*P2[1]-P3[1])*t2 + (-P0[1]+3*P1[1]-3*P2[1]+P3[1])*t3)
        points.append((x, y))
    return points

def build_spline_path(route, points_per_segment=20):
    """Build a Catmull-Rom spline for the given route.
    
    Returns (spline_points, spline_route_idx_map) where spline_route_idx_map[i]
    is the index j such that spline_points[i] corresponds to motion from route[j]
    towards route[j+1].
    """
    spline_points = []
    spline_route_idx_map = []
    if not route or len(route) == 0:
        return [], []
    if len(route) == 1:
        pt = nodes.get(route[0], (0, 0))
        return [pt], [0]

    n = len(route)
    for i in range(n-1):
        P0 = nodes[route[i-1]] if i > 0 else nodes[route[i]]
        P1 = nodes[route[i]]
        P2 = nodes[route[i+1]]
        P3 = nodes[route[i+2]] if i+2 < n else nodes[route[i+1]]
        seg = catmull_rom_spline(P0, P1, P2, P3, n_points=points_per_segment)
        spline_points.extend(seg)
        spline_route_idx_map.extend([i] * len(seg))

    return spline_points, spline_route_idx_map

def build_taxi_segment_points(route, route_idx, speed=0.24, min_points=12, start_override=None, corner_cut=10.0):
    """Build points for a taxi segment: straight edge with optional rounded corner near the end.

    Returns (points, next_start_override), where next_start_override is a point on the next
    segment if this segment rounds a corner (used to avoid snapping back to the node).
    """
    if route_idx < 0 or route_idx + 1 >= len(route):
        return [], None

    def _line_points(p0, p1, spacing):
        d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if d < 1e-6:
            return [p0]
        n = max(int(d / max(0.01, spacing)), 1)
        pts = []
        for i in range(n):
            t = i / n
            x = p0[0] + (p1[0] - p0[0]) * t
            y = p0[1] + (p1[1] - p0[1]) * t
            pts.append((x, y))
        pts.append(p1)
        return pts

    def _quad_points(p0, c, p1, spacing):
        approx_len = math.hypot(c[0] - p0[0], c[1] - p0[1]) + math.hypot(p1[0] - c[0], p1[1] - c[1])
        n = max(int(approx_len / max(0.01, spacing)), 12)
        pts = []
        for i in range(1, n + 1):
            t = i / n
            omt = 1.0 - t
            x = omt * omt * p0[0] + 2.0 * omt * t * c[0] + t * t * p1[0]
            y = omt * omt * p0[1] + 2.0 * omt * t * c[1] + t * t * p1[1]
            pts.append((x, y))
        return pts

    a = start_override if start_override is not None else nodes[route[route_idx]]
    b = nodes[route[route_idx + 1]]

    can_round = route_idx + 2 < len(route)
    if not can_round:
        pts = _line_points(a, b, speed)
        if len(pts) < min_points and len(pts) >= 2:
            pts = _line_points(a, b, max(0.01, speed * len(pts) / min_points))
        return pts, None

    c = nodes[route[route_idx + 2]]

    # Keep exact node crossing at hold points for procedural behavior.
    if route[route_idx + 1].endswith('_HOLD'):
        pts = _line_points(a, b, speed)
        if len(pts) < min_points and len(pts) >= 2:
            pts = _line_points(a, b, max(0.01, speed * len(pts) / min_points))
        return pts, None

    v1x, v1y = (b[0] - a[0], b[1] - a[1])
    v2x, v2y = (c[0] - b[0], c[1] - b[1])
    len1 = math.hypot(v1x, v1y)
    len2 = math.hypot(v2x, v2y)
    if len1 < 1e-6 or len2 < 1e-6:
        pts = _line_points(a, b, speed)
        if len(pts) < min_points and len(pts) >= 2:
            pts = _line_points(a, b, max(0.01, speed * len(pts) / min_points))
        return pts, None

    u1x, u1y = (v1x / len1, v1y / len1)
    u2x, u2y = (v2x / len2, v2y / len2)
    dot = max(-1.0, min(1.0, u1x * u2x + u1y * u2y))
    turn_angle_deg = math.degrees(math.acos(dot))

    # Keep straight segments straight unless there is a meaningful corner.
    if turn_angle_deg < 8.0:
        pts = _line_points(a, b, speed)
        if len(pts) < min_points and len(pts) >= 2:
            pts = _line_points(a, b, max(0.01, speed * len(pts) / min_points))
        return pts, None

    # Compute an inner fillet arc between the two segments so the nose "cuts" the corner.
    # Interpret `corner_cut` as a desired fillet radius (pixels). Compute tangent points
    # along each segment and build a circular arc between them. Fall back to the
    # previous quadratic-bezier approach if geometry fails.
    # Increase fillet radius (scale requested corner_cut) and allow larger
    # fraction of adjacent segment lengths so the nose cuts corners more aggressively.
    radius = min(corner_cut * 1.6, len1 * 0.5, len2 * 0.5)

    # Angle between incoming and outgoing vectors (in radians)
    dot = max(-1.0, min(1.0, u1x * u2x + u1y * u2y))
    beta = math.acos(dot)  # angle between u1 and u2

    # If nearly straight, don't round
    if beta < math.radians(8.0):
        pts = _line_points(a, b, speed)
        if len(pts) < min_points and len(pts) >= 2:
            pts = _line_points(a, b, max(0.01, speed * len(pts) / min_points))
        return pts, None

    # Distance from node along each segment to tangent points for a fillet of radius r
    try:
        d = radius * math.tan(beta / 2.0)
    except Exception:
        d = radius * 0.5

    # Ensure d isn't longer than available segment lengths
    d = min(d, len1 * 0.5, len2 * 0.5)

    in_pt = (b[0] - u1x * d, b[1] - u1y * d)
    out_pt = (b[0] + u2x * d, b[1] + u2y * d)

    # Compute circle center candidates: center must be at distance `radius` from both tangent points
    # and the vector from center to each tangent point must be perpendicular to the segment direction.
    def _norm(vx, vy):
        nl = math.hypot(vx, vy)
        if nl == 0:
            return (0.0, 0.0)
        return (vx / nl, vy / nl)

    # normals perpendicular to directions u1 and u2
    n1a = (-u1y, u1x)
    n1b = (u1y, -u1x)
    n2a = (-u2y, u2x)
    n2b = (u2y, -u2x)

    candidates = []
    for n1 in (n1a, n1b):
        for n2 in (n2a, n2b):
            c1 = (in_pt[0] + n1[0] * radius, in_pt[1] + n1[1] * radius)
            c2 = (out_pt[0] + n2[0] * radius, out_pt[1] + n2[1] * radius)
            # If these are close they represent the same center (signs chosen correctly)
            if math.hypot(c1[0] - c2[0], c1[1] - c2[1]) < 1.0:
                candidates.append(((c1[0] + c2[0]) / 2.0, (c1[1] + c2[1]) / 2.0))

    points = _line_points(a, in_pt, speed)

    if candidates:
        cx, cy = candidates[0]
        ang0 = math.atan2(in_pt[1] - cy, in_pt[0] - cx)
        ang1 = math.atan2(out_pt[1] - cy, out_pt[0] - cx)

        # Determine sweep direction that follows the path from in_pt -> out_pt
        # Compute cross product of u1 and u2 to choose direction
        cross = u1x * u2y - u1y * u2x
        # Normalize angles to [0, 2pi)
        def _norm_ang(a):
            while a < 0:
                a += 2 * math.pi
            while a >= 2 * math.pi:
                a -= 2 * math.pi
            return a

        a0 = _norm_ang(ang0)
        a1 = _norm_ang(ang1)

        if cross < 0:
            # clockwise rotation
            if a1 > a0:
                a1 -= 2 * math.pi
            arc_len = abs(a0 - a1) * radius
            n = max(int(arc_len / max(0.01, speed)), 12)
            for i in range(1, n + 1):
                t = i / n
                ang = a0 + (a1 - a0) * t
                x = cx + math.cos(ang) * radius
                y = cy + math.sin(ang) * radius
                points.append((x, y))
        else:
            # counter-clockwise rotation
            if a1 < a0:
                a1 += 2 * math.pi
            arc_len = abs(a1 - a0) * radius
            n = max(int(arc_len / max(0.01, speed)), 12)
            for i in range(1, n + 1):
                t = i / n
                ang = a0 + (a1 - a0) * t
                x = cx + math.cos(ang) * radius
                y = cy + math.sin(ang) * radius
                points.append((x, y))
    else:
        # Fallback: use previous quadratic approach if fillet center couldn't be computed
        points.extend(_quad_points(in_pt, b, out_pt, speed))

    if len(points) < min_points and len(points) >= 2:
        points = _line_points(a, in_pt, max(0.01, speed * len(points) / min_points))
        points.extend(_quad_points(in_pt, b, out_pt, max(0.01, speed * len(points) / min_points)))

    return points, out_pt

def nearest_node_to(x, y, max_dist=50):
    """Return the node name nearest to (x,y) within max_dist pixels, else None."""
    best = None
    best_d = float('inf')
    for name, (nx, ny) in nodes.items():
        d = math.hypot(nx - x, ny - y)
        if d < best_d:
            best_d = d
            best = name
    if best_d <= max_dist:
        return best
    return None

# ==============================
# CUSTOMTKINTER GUI SETUP
try:
    import customtkinter as ctk
    HAS_CTK = True
except Exception:
    ctk = None
    HAS_CTK = False
    print("customtkinter not available — GUI creation will be disabled when imported.")

active_aircraft = {}
aircraft_rows = {}
stop_bars = {}
stop_bar_draw_ids = {}
stop_bar_off_until = {}  # Track when each stop bar should turn back on (time-based)
next_departure_release_time = 0.0  # Track when the next departure may be released (time-based)
runway_protected = True  # Track if runway is protected (stop bars on)
simulation_speed = 1.0  # Speed multiplier for simulation (1x, 10x)
simulation_time_seconds = 0.0
last_sim_real_time = time.perf_counter()
main_canvas = None  # Global reference to the canvas
status_columns = {}  # Global reference to status board columns
aircraft_labels = {}  # Track labels in each status column
graph_element_ids = []  # Store all graph element IDs for hiding/showing
graph_visible = True  # Track whether graph is currently visible
simulation_running = False
activity_job_id = None
schedule_turnaround_cb = None

# Minimum gate turnaround before an arrival can push back (at 1x simulation speed).
MIN_TURNAROUND_DELAY_MS = 15 * 60 * 1000

# Pushback spacing between departures (at 1x simulation speed).
MIN_PUSHBACK_GAP_SECONDS = 30
MAX_PUSHBACK_GAP_SECONDS = 180
next_pushback_release_time = 0.0

# ==============================
# SPEED CONTROL HELPER
def adjust_delay(base_delay_ms):
    """Adjust a delay in milliseconds based on simulation speed multiplier."""
    return max(1, int(base_delay_ms / simulation_speed))

def get_simulation_time():
    """Return monotonically increasing simulation time in seconds.

    Simulation time advances as real elapsed time multiplied by current simulation speed,
    so changing the speed multiplier affects all clock-based timings immediately.
    """
    global simulation_time_seconds, last_sim_real_time
    now = time.perf_counter()
    elapsed_real = max(0.0, now - last_sim_real_time)
    simulation_time_seconds += elapsed_real * simulation_speed
    last_sim_real_time = now
    return simulation_time_seconds

def set_simulation_speed(new_speed):
    """Update simulation speed while preserving continuous simulation time."""
    global simulation_speed
    get_simulation_time()
    simulation_speed = new_speed

# ==============================
# GRAPH DRAW FUNCTION
def draw_graph(canvas):
    """Draw nodes and edges on the canvas."""
    global graph_element_ids, graph_visible
    
    # Clear existing graph element IDs
    graph_element_ids = []

    # Draw runway distance line (10m27 to 10m9)
    # Intentionally not tracked in graph_element_ids so it stays visible when toggling nodes.
    if "10m27" in nodes and "10m9" in nodes:
        x1, y1 = nodes["10m27"]
        x2, y2 = nodes["10m9"]
        canvas.create_line(x1, y1, x2, y2, fill="white", width=2)

    # Draw grey center segment between 0m9 and 0m27
    if "0m9" in nodes and "0m27" in nodes:
        x1, y1 = nodes["0m9"]
        x2, y2 = nodes["0m27"]
        canvas.create_line(x1, y1, x2, y2, fill="grey", width=6)

    # Draw small perpendicular ticks for runway distance nodes
    tick_half = 4
    for node_name in RUNWAY_TICK_NODES:
        if node_name not in nodes:
            continue
        x, y = nodes[node_name]
        canvas.create_line(x, y - tick_half, x, y + tick_half, fill="white", width=2)
    
    # Draw edges (connections) first so they appear under the nodes
    for node, neighbors in edges.items():
        x1, y1 = nodes[node]
        for neighbor in neighbors:
            x2, y2 = nodes[neighbor]
            line_id = canvas.create_line(x1, y1, x2, y2, fill="yellow", width=2, dash=(4,2))
            graph_element_ids.append(line_id)
    
    # Draw nodes (circles) on top
    for name, (x, y) in nodes.items():
        oval_id = canvas.create_oval(x-3, y-3, x+3, y+3, fill="red")
        # Position label below the node when graph visibility is ON, otherwise above
        label_y = y + 10 if graph_visible else y - 10
        text_id = canvas.create_text(x, label_y, text=name, fill="red", font=("Arial", 7, "bold"))
        graph_element_ids.append(oval_id)
        graph_element_ids.append(text_id)

def toggle_graph_visibility():
    """Toggle the visibility of nodes and edges."""
    global graph_visible
    
    if not main_canvas:
        return
    
    if graph_visible:
        # Hide all graph elements
        for element_id in graph_element_ids:
            main_canvas.itemconfig(element_id, state='hidden')
        graph_visible = False
    else:
        # Show all graph elements
        for element_id in graph_element_ids:
            main_canvas.itemconfig(element_id, state='normal')
        graph_visible = True

# ==============================
# STOP BARS
# Manual pixel coordinates for stop bar lights at each hold point
STOP_BAR_POSITIONS = {
    "A1_HOLD": {
        "red": [(1810, 120), (1815, 120), (1820, 120), (1825, 120), (1830, 120), (1835, 120)], 
    },
    "B1_HOLD": {
        "red": [(1548, 120), (1553, 120), (1558, 120), (1563, 120), (1568, 120)],
    },
    "C1_HOLD": {
        "red": [(635, 120), (640, 120), (645, 120), (650, 120), (655, 120)],
    },
    "D1_HOLD": {
        "red": [(204, 120), (209, 120), (214, 120), (219, 120), (224, 120)],
    },
    "E1_HOLD": {
        "red": [(90, 120), (95, 120), (100, 120), (105, 120), (110, 120), (115, 120)],
    }
}

def draw_stop_bars(canvas):
    """Draw stop bars at runway hold points using manual pixel coordinates.
    
    Red stop bar lights are always on, and turn off when a DEPARTING aircraft at that hold
    is cleared to cross (runway is clear). Landing aircraft do NOT affect stop bar state.
    Lights turn back on 2 seconds after turning off.
    """
    global stop_bar_draw_ids, stop_bar_off_until, next_departure_release_time
    
    # Clear existing stop bars
    for items in stop_bar_draw_ids.values():
        for item_id in items:
            canvas.delete(item_id)
    stop_bar_draw_ids.clear()
    
    current_time = get_simulation_time()
    
    # Check which hold points have DEPARTING aircraft that should turn off the stop bar
    # Landing aircraft (with ignore_stop_bars flag) should NOT affect stop bars
    for callsign, info in active_aircraft.items():
        node = info.get('node')
        # Only turn off stop bar if:
        # 1. Aircraft is at a hold point
        # 2. Runway is clear
        # 3. Aircraft is NOT a landing aircraft (doesn't have ignore_stop_bars flag)
        # 4. No arrival is within 3nm (3 miles) of the runway
        is_landing_aircraft = info.get('ignore_stop_bars', False)
        arrival_too_close = is_arrival_within_5nm()
        
        if node and node.endswith('_HOLD') and is_runway_clear() and not is_landing_aircraft and not arrival_too_close:
            # If this hold's stop bar isn't already off, turn it off now and set timer
            if (node not in stop_bar_off_until or current_time >= stop_bar_off_until[node]) and current_time >= next_departure_release_time:
                stop_bar_duration = 5.0
                stop_bar_off_until[node] = current_time + stop_bar_duration
    
    for hold_name, positions in STOP_BAR_POSITIONS.items():
        if not positions.get("red"):
            continue  # Skip if no red positions defined
            
        items = []
        
        # Red lights are OFF if we're within the timer period AND no arrival is within 3nm (3 miles)
        # Red lights turn back ON immediately if arrival comes within 3nm
        is_off = hold_name in stop_bar_off_until and current_time < stop_bar_off_until[hold_name] and not is_arrival_within_5nm()
        show_red_lights = not is_off
        
        # Draw red stop bar lights
        if show_red_lights:
            for x, y in positions.get("red", []):
                light_id = canvas.create_oval(
                    x-2, y-2, x+2, y+2,
                    fill="red", outline="red", width=1
                )
                canvas.tag_raise(light_id)  # Bring to front
                items.append(light_id)
        
        if items:
            stop_bar_draw_ids[hold_name] = items
            items.append(light_id)
        
        if items:
            stop_bar_draw_ids[hold_name] = items

def update_stop_bars(canvas):
    """Update stop bar lights based on runway status."""
    if not canvas:
        return
    
    # Redraw all stop bars with current runway status
    draw_stop_bars(canvas)

def continuous_stop_bar_update():
    """Continuously update stop bars every 100ms for smooth timer-based updates."""
    if main_canvas:
        update_stop_bars(main_canvas)
    # Schedule next update
    app.after(adjust_delay(100), continuous_stop_bar_update)

# ==============================
# DRAW AIRCRAFT
def draw_aircraft(canvas, x, y, callsign="TEST", direction="north", color="blue"):
    """Draw a blue triangle representing an aircraft at the given position.
    
    The front tip of the triangle is always at position (x, y).
    
    Args:
        direction: "north" (up), "south" (down), "right" (east), "left" (west)
    """
    size = 12
    
    if direction == "north":
        # Triangle pointing up (north) - tip at (x, y)
        triangle_id = canvas.create_polygon(
            x, y,               # Front point at top
            x-size, y+2*size,   # Bottom left
            x+size, y+2*size,   # Bottom right
            fill=color
        )
    elif direction == "south":
        # Triangle pointing down (south) - tip at (x, y)
        triangle_id = canvas.create_polygon(
            x, y,               # Front point at bottom
            x-size, y-2*size,   # Top left
            x+size, y-2*size,   # Top right
            fill=color
        )
    elif direction == "right":
        # Triangle pointing right (for runway 09) - tip at (x, y)
        triangle_id = canvas.create_polygon(
            x, y,               # Front point at right
            x-2*size, y-size,   # Top left
            x-2*size, y+size,   # Bottom left
            fill=color
        )
    else:  # direction == "left"
        # Triangle pointing left (for runway 27) - tip at (x, y)
        triangle_id = canvas.create_polygon(
            x, y,               # Front point at left
            x+2*size, y-size,   # Top right
            x+2*size, y+size,   # Bottom right
            fill=color
        )
    
    label_id = canvas.create_text(x, y-size-10, text=callsign, fill="white", font=("Arial", 10, "bold"))
    return triangle_id, label_id

# ==============================
# STATUS BOARD FUNCTIONS
def add_aircraft_to_status(callsign, column_name):
    """Add aircraft to a status column."""
    if column_name in status_columns:
        label = ctk.CTkLabel(
            status_columns[column_name],
            text=callsign,
            font=("Arial", 11, "bold"),
            text_color="black"
        )
        label.pack(anchor="w", padx=5, pady=2)
        
        # Store label reference
        if callsign not in aircraft_labels:
            aircraft_labels[callsign] = {}
        aircraft_labels[callsign]['label'] = label
        aircraft_labels[callsign]['column'] = column_name

def move_aircraft_status(callsign, new_column):
    """Move aircraft from current column to new column."""
    if callsign in aircraft_labels:
        # Remove from old column
        old_label = aircraft_labels[callsign].get('label')
        if old_label:
            old_label.destroy()
        
        # Add to new column
        add_aircraft_to_status(callsign, new_column)

def remove_aircraft_from_status(callsign):
    """Remove aircraft from status board."""
    if callsign in aircraft_labels:
        label = aircraft_labels[callsign].get('label')
        if label:
            label.destroy()
        del aircraft_labels[callsign]

# ==============================
# PUSHBACK AIRCRAFT
def pushback_aircraft(canvas, aircraft_info, target_node, final_direction="right", speed=1, runway_target=None):
    """Animate aircraft pushback from current position to target node.
    
    Aircraft keeps its stand-facing heading during most of pushback and
    gradually rotates to final_direction near the end.
    After pushback completes, starts taxi to runway if runway_target is provided.
    
    Args:
        speed: Pixels per step - controls smoothness of movement (default: 2)
    """
    import math

    pushback_duration_ms = 90000  # 1m30s at 1x speed
    post_pushback_wait_ms = 120000  # 2 minutes stationary at 1x speed
    
    start_x, start_y = aircraft_info['position']
    end_x, end_y = nodes[target_node]
    
    # Calculate distance and steps based on speed (speed controls smoothness)
    distance = math.hypot(end_x - start_x, end_y - start_y)
    steps = max(int(distance / speed), 1)
    
    # Calculate step delay to achieve target duration
    step_delay_ms = max(1, int(pushback_duration_ms / steps))
    
    step_x = (end_x - start_x) / steps
    step_y = (end_y - start_y) / steps
    rotation_start_step = int(steps * 0.7)  # Start rotation at 70% of journey
    rotation_steps = steps - rotation_start_step
    
    # Determine initial heading from stand orientation when available.
    # Optional override allows stand/runway-specific pushback behavior.
    start_direction = aircraft_info.pop('pushback_start_direction', None) or aircraft_info.get('direction')
    current_node = aircraft_info.get('node', '')
    if start_direction is None and current_node in nodes and current_node.startswith("STAND") and (current_node.endswith('a') or current_node.endswith('A')):
        start_direction = get_stand_direction(current_node)

    start_angle = direction_to_heading(start_direction or "north")
    end_angle = direction_to_heading(final_direction)

    # Use shortest angular path so turn-in is always the expected ~90 degrees.
    angle_delta = ((end_angle - start_angle + 180) % 360) - 180
    
    # Flag to track if we've moved to Taxiing status
    moved_to_taxiing = [False]
    
    def animate_step(current_step=0):
        # Move to taxiing column on first movement
        if current_step == 1 and not moved_to_taxiing[0]:
            move_aircraft_status(aircraft_info['callsign'], "Taxiing")
            moved_to_taxiing[0] = True
        
        if current_step >= steps:
            # Final position - update aircraft info
            aircraft_info['position'] = (end_x, end_y)
            aircraft_info['node'] = target_node
            aircraft_info['direction'] = final_direction
            # Set heading to match the final orientation to prevent glitch when taxi starts
            aircraft_info['heading'] = end_angle
            
            # Start taxi to runway if target provided
            if runway_target:
                app.after(adjust_delay(post_pushback_wait_ms), lambda: taxi_aircraft(canvas, aircraft_info, runway_target))
            return
        
        # Calculate current position
        curr_x = start_x + (step_x * current_step)
        curr_y = start_y + (step_y * current_step)
        
        # Calculate current angle with smooth rotation
        if current_step < rotation_start_step:
            current_angle = start_angle
        else:
            # Smooth interpolation during rotation
            rotation_progress = (current_step - rotation_start_step) / rotation_steps
            current_angle = start_angle + angle_delta * rotation_progress
        
        # Convert angle to radians
        angle_rad = math.radians(current_angle)
        
        # Update triangle position based on angle
        size = 12
        # Calculate triangle points rotated around the tip
        # Tip is always at (curr_x, curr_y)
        # Base points are offset from tip
        
        # For a triangle pointing right (0 degrees), the points would be:
        # tip: (0, 0), left back: (-2*size, -size), left bottom: (-2*size, size)
        # We rotate these relative positions
        
        # Define base triangle (pointing right at 0 degrees)
        base_points = [
            (0, 0),              # Tip
            (-2*size, -size),    # Upper back
            (-2*size, size)      # Lower back
        ]
        
        # Rotate and translate points
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        rotated_points = []
        for px, py in base_points:
            # Rotate
            rx = px * cos_a - py * sin_a
            ry = px * sin_a + py * cos_a
            # Translate to current position
            rotated_points.extend([curr_x + rx, curr_y + ry])

        if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
            app.after(adjust_delay(200), animate_step, current_step)
            return
        
        canvas.coords(aircraft_info['triangle_id'], *rotated_points)
        
        # Update label position
        canvas.coords(aircraft_info['label_id'], curr_x, curr_y-size-10)
        
        # Update stored position
        aircraft_info['position'] = (curr_x, curr_y)
        
        # Schedule next step
        app.after(adjust_delay(step_delay_ms), animate_step, current_step + 1)
    
    animate_step()

# ==============================
# TAXI AIRCRAFT ALONG PATH
def taxi_aircraft(canvas, aircraft_info, destination_node, speed=0.24):
    """Animate aircraft taxiing from current node to destination using pathfinding."""
    import math
    
    # Get current node from aircraft info
    current_node = aircraft_info.get('node', 'STAND1N')

    def _route_length(route_nodes):
        if not route_nodes or len(route_nodes) < 2:
            return 0.0
        return sum(
            math.dist(nodes[route_nodes[i]], nodes[route_nodes[i + 1]])
            for i in range(len(route_nodes) - 1)
        )
    
    # Find path using dijkstra
    route = dijkstra(current_node, destination_node)

    # Runway 27 rule: departures leaving stand area must flow out via AS or AR.
    runway_selector = globals().get('runway_var')
    runway_in_use = runway_selector.get() if runway_selector else None

    if runway_in_use == "27" and destination_node == "A1_HOLD" and (current_node.startswith("STAND") or current_node == "NS"):
        best_route = None
        best_route_length = None
        for stand_exit in ("AS", "AR"):
            to_exit = dijkstra(current_node, stand_exit)
            from_exit = dijkstra(stand_exit, destination_node)
            if not to_exit or not from_exit:
                continue
            candidate = to_exit + from_exit[1:]
            candidate_length = _route_length(candidate)
            if best_route is None or candidate_length < best_route_length:
                best_route = candidate
                best_route_length = candidate_length
        if best_route:
            route = best_route

    # Runway 09 rule: departures leaving stand area must flow out via AQ or AS.
    if runway_in_use == "09" and destination_node == "E1_HOLD" and (current_node.startswith("STAND") or current_node == "NS"):
        best_route = None
        best_route_length = None
        for stand_exit in ("AQ", "AS"):
            to_exit = dijkstra(current_node, stand_exit)
            from_exit = dijkstra(stand_exit, destination_node)
            if not to_exit or not from_exit:
                continue
            candidate = to_exit + from_exit[1:]
            candidate_length = _route_length(candidate)
            if best_route is None or candidate_length < best_route_length:
                best_route = candidate
                best_route_length = candidate_length
        if best_route:
            route = best_route
    
    if not route:
        print(f"No route found from {current_node} to {destination_node}")
        return
    
    aircraft_info['route'] = route
    aircraft_info['route_index'] = 0
    aircraft_info.pop('segment_start_override', None)
    aircraft_info.pop('segment_start_override_idx', None)
    # Clear heading and path history when starting taxi to ensure proper initialization from new segment
    aircraft_info.pop('heading', None)
    aircraft_info['path_history'] = [aircraft_info.get('position', nodes[current_node])]

    def _rear_from_history(history, wheelbase):
        if not history:
            return nodes[current_node]
        dist = 0.0
        for i in range(len(history) - 1, 0, -1):
            x1, y1 = history[i]
            x0, y0 = history[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if dist + seg >= wheelbase:
                t = (wheelbase - dist) / seg if seg > 1e-6 else 0.0
                rx = x1 + (x0 - x1) * t
                ry = y1 + (y0 - y1) * t
                return rx, ry
            dist += seg
        return history[0]
    
    def move_to_next_node():
        """Move aircraft to the next node in the route."""
        route_idx = aircraft_info.get('route_index', 0)
        
        if route_idx >= len(route) - 1:
            # Reached final destination
            aircraft_info['node'] = destination_node
            aircraft_info['position'] = nodes[destination_node]
            
            # Check if we've reached a hold point and should proceed to runway
            if destination_node in ["A1_HOLD", "E1_HOLD"]:
                # Determine runway entry point based on hold point
                if destination_node == "A1_HOLD":
                    runway_entry = "RWY27_A1_ALIGN"
                else:  # E1_HOLD
                    runway_entry = "RWY09_E1_ALIGN"
                
                # Continue to runway entry point after brief hold
                app.after(adjust_delay(500), lambda: taxi_aircraft(canvas, aircraft_info, runway_entry))
            # Check if we've reached a runway entry point and should start takeoff
            elif destination_node in ["RWY27_A1", "RWY09_E1", "RWY27_A1_ALIGN", "RWY09_E1_ALIGN"]:
                # Determine the airborne node based on runway
                if destination_node in ["RWY27_A1", "RWY27_A1_ALIGN"]:
                    airborne_node = "RWY27_AirBorne"
                else:  # RWY09_E1
                    airborne_node = "RWY09_AirBorne"
                
                # Start takeoff immediately
                takeoff_aircraft(canvas, aircraft_info, airborne_node)
            return
        
        # Get current and next node positions
        current_node = route[route_idx]
        next_node = route[route_idx + 1]

        # Merge priority rule at AR: traffic from AS has priority over traffic from NR.
        # If this aircraft is NR->AR, hold and retry until AS-priority conflict clears.
        if current_node == "NR" and next_node == "AR":
            if should_nr_yield_to_as_at_ar(aircraft_info['callsign']):
                aircraft_info['waiting_for_merge_priority'] = True
                app.after(adjust_delay(500), move_to_next_node)
                return
            aircraft_info['waiting_for_merge_priority'] = False
        elif current_node == "NQ" and next_node == "AQ":
            if should_nq_yield_to_as_at_aq(aircraft_info['callsign']):
                aircraft_info['waiting_for_merge_priority'] = True
                app.after(adjust_delay(500), move_to_next_node)
                return
            aircraft_info['waiting_for_merge_priority'] = False
        else:
            aircraft_info['waiting_for_merge_priority'] = False
        
        # Check if we're about to pass through a HOLD node (departing from it)
        # If so, check if runway is clear before proceeding
        if current_node.endswith('_HOLD'):
            # Check if stop bar at this hold point is illuminated (red lights on)
            stop_bar_illuminated = current_node in stop_bar_draw_ids and len(stop_bar_draw_ids.get(current_node, [])) > 0
            
            # Also check if any arrival is within 3nm (3 miles) of the runway
            arrival_too_close = is_arrival_within_5nm()
            
            if not is_runway_clear() or stop_bar_illuminated or arrival_too_close:
                # Runway is occupied, stop bar is on, or arrival is too close - wait and check again
                aircraft_info['waiting_at_hold'] = True
                app.after(adjust_delay(1000), move_to_next_node)  # Check again
                # Update stop bars in case this aircraft's presence changes the state
                if main_canvas:
                    update_stop_bars(main_canvas)
                return
            else:
                # Runway is clear, stop bar is off, and no arrivals too close - proceed and update status
                aircraft_info['waiting_at_hold'] = False
                move_aircraft_status(aircraft_info['callsign'], 'Runway')
                # Update stop bars to show runway is now occupied
                if main_canvas:
                    update_stop_bars(main_canvas)
        
        start_x, start_y = nodes[current_node]
        end_x, end_y = nodes[next_node]
        
        start_override = None
        if aircraft_info.get('segment_start_override_idx') == route_idx:
            start_override = aircraft_info.get('segment_start_override')
            aircraft_info.pop('segment_start_override_idx', None)
            aircraft_info.pop('segment_start_override', None)

        segment_points, next_start_override = build_taxi_segment_points(
            route,
            route_idx,
            speed=speed,
            start_override=start_override
        )
        if not segment_points:
            segment_points = [nodes[current_node]]

        if next_start_override is not None:
            aircraft_info['segment_start_override'] = next_start_override
            aircraft_info['segment_start_override_idx'] = route_idx + 1

        # Calculate distance and direction
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        
        # Calculate target angle for this segment
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Initialize heading if not set
        if 'heading' not in aircraft_info:
            aircraft_info['heading'] = target_angle
        
        # Detect if this segment involves runway nodes for special corner-cutting behavior
        runway_turn_nodes = {"RWY27_A1", "RWY09_E1", "RWY09_C1", "RWY27_B1"}
        is_runway_segment = current_node in runway_turn_nodes or next_node in runway_turn_nodes
        
        # Calculate look-ahead angle for smooth turning
        next_segment_angle = target_angle
        has_next_segment = False
        if route_idx + 2 < len(route):
            next_next_node = route[route_idx + 2]
            next_end_x, next_end_y = nodes[next_next_node]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        elif next_node == "RWY27_A1":
            next_end_x, next_end_y = nodes["RWY27_AirBorne"]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        elif next_node == "RWY09_E1":
            next_end_x, next_end_y = nodes["RWY09_AirBorne"]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        
        # Turn parameters
        turn_zone_distance = 20  # Start turning/cutting corner within this distance
        max_turn_per_step = 1.35
        corner_cut_radius = 10   # Pixels to cut corner
        
        # Use the same taxi turning logic for runway and taxiway segments.
        use_distance_stepping = False
        
        if use_distance_stepping:
            # For runway segments, use distance-based animation like landing_aircraft
            avg_speed = 0.24  # Taxi speed in pixels per frame
            steps = max(int(distance / avg_speed), 1)
            
            def animate_segment(step=0):
                if step == 0:
                    aircraft_info['segment_x'] = start_x
                    aircraft_info['segment_y'] = start_y
                    aircraft_info['segment_remaining_distance'] = distance
                
                # Check if we've reached the destination node
                current_x = aircraft_info.get('segment_x', start_x)
                current_y = aircraft_info.get('segment_y', start_y)
                dist_remaining = math.hypot(end_x - current_x, end_y - current_y)
                remaining = aircraft_info.get('segment_remaining_distance', dist_remaining)
                
                if remaining <= 1.0 or dist_remaining <= 1.0:
                    # Reached next node
                    aircraft_info['route_index'] = route_idx + 1
                    aircraft_info['node'] = next_node
                    aircraft_info['position'] = nodes[next_node]
                    aircraft_info['segment_x'] = end_x
                    aircraft_info['segment_y'] = end_y
                    move_to_next_node()
                    return
                
                # Move along segment by distance this frame
                if distance > 0:
                    ux = dx / distance
                    uy = dy / distance
                else:
                    ux, uy = 0.0, 0.0
                
                remaining = aircraft_info.get('segment_remaining_distance', distance)
                move_dist = min(speed, remaining)
                step_x = ux * move_dist
                step_y = uy * move_dist
                
                aircraft_info['segment_x'] += step_x
                aircraft_info['segment_y'] += step_y
                aircraft_info['segment_remaining_distance'] = max(0.0, remaining - move_dist)
                
                base_x = aircraft_info['segment_x']
                base_y = aircraft_info['segment_y']
                
                # Calculate distance to next node for turn zone detection
                dist_to_next_node = math.hypot(end_x - base_x, end_y - base_y)
                
                # Apply corner cutting if approaching a turn
                curr_x = base_x
                curr_y = base_y
                
                if has_next_segment and dist_to_next_node <= turn_zone_distance:
                    # Calculate angle difference to next segment
                    angle_diff_to_next = next_segment_angle - target_angle
                    while angle_diff_to_next > 180:
                        angle_diff_to_next -= 360
                    while angle_diff_to_next < -180:
                        angle_diff_to_next += 360
                    
                    if abs(angle_diff_to_next) > 5:
                        # Smooth corner cutting with cubic interpolation
                        linear_factor = 1.0 - (dist_to_next_node / turn_zone_distance)
                        cut_factor = linear_factor * linear_factor * linear_factor
                        
                        bisector_angle = target_angle + (angle_diff_to_next * 0.5)
                        bisector_rad = math.radians(bisector_angle)
                        
                        offset = corner_cut_radius * cut_factor
                        curr_x = base_x + offset * math.cos(bisector_rad)
                        curr_y = base_y + offset * math.sin(bisector_rad)
                
                # Determine which angle to turn toward
                if has_next_segment and dist_to_next_node <= turn_zone_distance:
                    angle_diff_to_next = next_segment_angle - target_angle
                    while angle_diff_to_next > 180:
                        angle_diff_to_next -= 360
                    while angle_diff_to_next < -180:
                        angle_diff_to_next += 360
                    
                    if abs(angle_diff_to_next) > 5:
                        turn_target = next_segment_angle
                    else:
                        turn_target = target_angle
                else:
                    turn_target = target_angle
                
                # Gradually adjust heading with limited turn rate
                heading = aircraft_info['heading']
                angle_diff = turn_target - heading
                
                while angle_diff > 180:
                    angle_diff -= 360
                while angle_diff < -180:
                    angle_diff += 360
                
                if abs(angle_diff) > max_turn_per_step:
                    heading += max_turn_per_step if angle_diff > 0 else -max_turn_per_step
                else:
                    heading = turn_target
                
                aircraft_info['heading'] = heading
                
                # Maintain path history for rear position calculation
                wheelbase = 24
                history = aircraft_info.get('nose_history')
                if history is None:
                    history = []
                    aircraft_info['nose_history'] = history
                
                if not history:
                    history.append((curr_x, curr_y, 0.0))
                else:
                    lx, ly, lc = history[-1]
                    dist = math.hypot(curr_x - lx, curr_y - ly)
                    history.append((curr_x, curr_y, lc + dist))
                
                # Trim history to reasonable length
                while len(history) > 2 and (history[-1][2] - history[0][2]) > 2000:
                    history.pop(0)
                
                # Find rear point from history
                curr_cum = history[-1][2]
                target_cum = max(0.0, curr_cum - wheelbase)
                
                rear_x = history[0][0]
                rear_y = history[0][1]
                for i in range(len(history)-1, 0, -1):
                    x1, y1, c1 = history[i-1]
                    x2, y2, c2 = history[i]
                    if c1 <= target_cum <= c2:
                        t = (target_cum - c1) / (c2 - c1) if c2 > c1 else 0.0
                        rear_x = x1 + (x2 - x1) * t
                        rear_y = y1 + (y2 - y1) * t
                        break
                
                # Compute heading from rear to nose
                heading = math.degrees(math.atan2(curr_y - rear_y, curr_x - rear_x))
                aircraft_info['heading'] = heading
                
                # Draw triangle
                angle_rad = math.radians(heading)
                size = 12
                base_points = [
                    (0, 0),
                    (-2*size, -size),
                    (-2*size, size)
                ]
                cos_a = math.cos(angle_rad)
                sin_a = math.sin(angle_rad)
                rotated_points = []
                for px, py in base_points:
                    rx = px * cos_a - py * sin_a
                    ry = px * sin_a + py * cos_a
                    rotated_points.extend([curr_x + rx, curr_y + ry])
                
                if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                    app.after(adjust_delay(200), animate_segment, step)
                    return
                
                canvas.coords(aircraft_info['triangle_id'], *rotated_points)
                canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)
                
                aircraft_info['position'] = (curr_x, curr_y)
                
                # Schedule next step
                app.after(adjust_delay(50), animate_segment, step + 1)
        
        else:
            # For non-runway segments, use original segment_points approach
            steps = len(segment_points)
            lookahead_points = 4

            if 'heading' not in aircraft_info:
                if steps > 1:
                    x0, y0 = segment_points[0]
                    x1, y1 = segment_points[min(lookahead_points, steps - 1)]
                    aircraft_info['heading'] = math.degrees(math.atan2(y1 - y0, x1 - x0))
                else:
                    aircraft_info['heading'] = math.degrees(math.atan2(end_y - start_y, end_x - start_x))
            
            def animate_segment(step=0):
                if step >= steps:
                    # Reached next node, move to the next segment immediately without delay
                    aircraft_info['route_index'] = route_idx + 1
                    aircraft_info['node'] = next_node
                    aircraft_info['position'] = segment_points[-1]
                    # Continue immediately to next segment for smooth transition
                    move_to_next_node()
                    return
                
                curr_x, curr_y = segment_points[step]

                if steps > 1:
                    look_idx = min(step + lookahead_points, steps - 1)
                    look_x, look_y = segment_points[look_idx]
                    turn_target = math.degrees(math.atan2(look_y - curr_y, look_x - curr_x))
                else:
                    turn_target = aircraft_info['heading']
                
                # Compute rear position from overall taxi history so turns stay smooth across segments.
                size = 12
                wheelbase = 2 * size
                history = aircraft_info.setdefault('path_history', [])
                if not history:
                    history.append(segment_points[0])
                rear_x, rear_y = _rear_from_history(history, wheelbase)

                aircraft_info['rear_pos'] = (rear_x, rear_y)

                # Compute heading from rear->nose so the whole aircraft aligns with path
                heading = math.degrees(math.atan2(curr_y - rear_y, curr_x - rear_x))
                aircraft_info['heading'] = heading

                # Update aircraft triangle rotated about the nose point
                angle_rad = math.radians(heading)
                base_points = [
                    (0, 0),              # Nose
                    (-wheelbase, -size), # Upper rear
                    (-wheelbase, size)   # Lower rear
                ]

                cos_a = math.cos(angle_rad)
                sin_a = math.sin(angle_rad)

                rotated_points = []
                for px, py in base_points:
                    rx = px * cos_a - py * sin_a
                    ry = px * sin_a + py * cos_a
                    rotated_points.extend([curr_x + rx, curr_y + ry])

                if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                    app.after(adjust_delay(200), animate_segment, step)
                    return
                
                canvas.coords(aircraft_info['triangle_id'], *rotated_points)
                canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)
                
                aircraft_info['position'] = (curr_x, curr_y)
                history.append((curr_x, curr_y))
                if len(history) > 400:
                    del history[:-300]
                
                # Schedule next step
                app.after(adjust_delay(50), animate_segment, step + 1)
        
        animate_segment()
    
    move_to_next_node()

# ==============================
# ARRIVAL SPACING HELPERS
def get_arrival_node_index(node_name):
    """Get the index of a node in the runway tick sequence, or None if not found."""
    if node_name in RUNWAY_TICK_NODES:
        return RUNWAY_TICK_NODES.index(node_name)
    return None

def get_closest_inbound_arrival():
    """Return the node index of the closest inbound arrival aircraft, or None."""
    closest_idx = None
    for callsign, info in active_aircraft.items():
        # Check if this is an inbound arrival (has radar_dot_id or is orange triangle on approach)
        node = info.get('node')
        if node and node in RUNWAY_TICK_NODES:
            idx = get_arrival_node_index(node)
            if idx is not None:
                if closest_idx is None or abs(idx - 10) < abs(closest_idx - 10):
                    closest_idx = idx
    return closest_idx

def has_departing_aircraft():
    """Check if there are any aircraft taxiing for departure or waiting at hold points."""
    for callsign, info in active_aircraft.items():
        # Check if aircraft is in Departures or Taxiing status
        if callsign in aircraft_labels:
            status = aircraft_labels[callsign].get('column')
            if status in ['Departures', 'Taxiing']:
                return True
        
        # Check if aircraft is at a hold point
        node = info.get('node')
        if node and node.endswith('_HOLD'):
            return True
    
    return False

def can_spawn_new_arrival(spawn_node_name):
    """Check if a new arrival can spawn at the given spawn node.
    
    Spacing requirement:
    - Always: >=7 nodes from any arrival on final approach
    
    Args:
        spawn_node_name: The node where the aircraft will spawn ("10m9" or "10m27")
    """
    if spawn_node_name not in RUNWAY_TICK_NODES:
        return False
    
    spawn_idx = RUNWAY_TICK_NODES.index(spawn_node_name)
    
    # Enforce 7-node separation between arrivals on final approach
    min_spacing = 7
    
    # Check all aircraft on the radar path
    for callsign, info in active_aircraft.items():
        node = info.get('node')
        if node and node in RUNWAY_TICK_NODES:
            idx = get_arrival_node_index(node)
            if idx is not None:
                # Check if this aircraft is within required spacing of the spawn point
                distance = abs(idx - spawn_idx)
                if distance < min_spacing:
                    return False
    
    return True

def is_arrival_within_5_5nm():
    """Check if any arrival is within 5.5nm (5.5 nodes) of the runway center (0m9/0m27)."""
    center_idx_09 = RUNWAY_TICK_NODES.index("0m9")
    center_idx_27 = RUNWAY_TICK_NODES.index("0m27")
    
    for callsign, info in active_aircraft.items():
        node = info.get('node')
        if node and node in RUNWAY_TICK_NODES:
            idx = get_arrival_node_index(node)
            if idx is not None:
                # Check if within 5 nodes of either center
                dist_09 = abs(idx - center_idx_09)
                dist_27 = abs(idx - center_idx_27)
                if min(dist_09, dist_27) <= 5:
                    return True
    return False

def is_arrival_within_5nm():
    """Check if any arrival is within 3nm (3 nodes) of the runway center (0m9/0m27)."""
    center_idx_09 = RUNWAY_TICK_NODES.index("0m9")
    center_idx_27 = RUNWAY_TICK_NODES.index("0m27")

    for callsign, info in active_aircraft.items():
        node = info.get('node')
        if node and node in RUNWAY_TICK_NODES:
            idx = get_arrival_node_index(node)
            if idx is not None:
                dist_09 = abs(idx - center_idx_09)
                dist_27 = abs(idx - center_idx_27)
                if min(dist_09, dist_27) <= 3:
                    return True
    return False

# ==============================
# RUNWAY OCCUPANCY CHECK
def is_runway_clear():
    """Check if the runway is clear (no aircraft in Runway status or at runway nodes)."""
    global runway_protected
    
    # Check if any aircraft is in the Runway status column
    # (Landing aircraft stay in this status until they pass the runway exit)
    for callsign, info in active_aircraft.items():
        if callsign in aircraft_labels:
            current_column = aircraft_labels[callsign].get('column')
            if current_column == 'Runway':
                runway_protected = False  # Runway occupied
                return False
    
    # Also check if any aircraft is at a runway node
    runway_nodes = ['RWY27_A1', 'RWY27_A1_ALIGN', 'RWY27_B1', 'RWY27_B1_EXIT', 'RWY09_C1', 'RWY09_C1_EXIT', 'RWY09_D1', 'RWY09_E1', 'RWY09_E1_ALIGN']
    for callsign, info in active_aircraft.items():
        if info.get('node') in runway_nodes:
            runway_protected = False  # Runway occupied
            return False
    
    # Runway is clear - reset protection for next aircraft
    runway_protected = True
    return True

# ==============================
# TAKEOFF AIRCRAFT
def takeoff_aircraft(canvas, aircraft_info, airborne_node):
    """Animate aircraft taking off from runway to airborne node.
    
    Aircraft gradually accelerates and moves towards the airborne node,
    then disappears and moves to the Airborne status column.
    """
    import math
    import random
    
    global next_departure_release_time

    # Get starting position (current runway position)
    start_x, start_y = aircraft_info['position']
    end_x, end_y = nodes[airborne_node]
    
    # Calculate direction and distance
    dx = end_x - start_x
    dy = end_y - start_y
    total_distance = math.hypot(dx, dy)
    
    # Calculate target heading for takeoff roll
    target_heading = math.degrees(math.atan2(dy, dx))
    
    # Preserve current heading from taxi (smooth transition)
    if 'heading' not in aircraft_info:
        aircraft_info['heading'] = target_heading
    
    # Enforce departure separation (1 or 2 minutes, scaled by sim speed)
    next_departure_release_time = get_simulation_time() + random.choice([60, 120])

    # Takeoff parameters - 45 second takeoff with gradual acceleration from taxi speed
    initial_speed = 0.22  # Starting speed - same as taxi speed (pixels per frame)
    final_speed = 2.21   # Speed at rotation/liftoff (pixels per frame) - targets 45sec for ~1820px runways
    acceleration_distance = total_distance * 0.8  # Accelerate for 80% of runway
    max_turn_per_step = 8.0  # Same as taxi turning rate for consistency
    
    # Total steps to achieve ~45 second takeoff duration (1500 steps * 30ms = 45 seconds)
    total_steps = 1500
    
    # Track if we've moved to airborne status
    moved_to_airborne = [False]
    
    def animate_takeoff(step=0):
        if step >= total_steps:
            # Aircraft has left the screen - remove it from canvas
            canvas.delete(aircraft_info['triangle_id'])
            canvas.delete(aircraft_info['label_id'])
            
            # Update aircraft info
            aircraft_info['position'] = (end_x, end_y)
            aircraft_info['node'] = airborne_node
            
            # Remove from status board after 10 seconds
            callsign = aircraft_info['callsign']
            app.after(adjust_delay(10000), lambda: remove_aircraft_from_status(callsign))
            
            return
        
        # Calculate progress (0.0 to 1.0)
        progress = step / total_steps
        
        # Move to Airborne status column at approximately 50% of takeoff roll
        if progress >= 0.5 and not moved_to_airborne[0]:
            move_aircraft_status(aircraft_info['callsign'], 'Airborne')
            moved_to_airborne[0] = True
            # Update stop bars - runway is now clear
            if main_canvas:
                update_stop_bars(main_canvas)
        
        # Calculate current speed with acceleration
        if progress < (acceleration_distance / total_distance):
            # Accelerating phase
            accel_progress = progress / (acceleration_distance / total_distance)
            current_speed = initial_speed + (final_speed - initial_speed) * accel_progress
        else:
            # Constant speed phase
            current_speed = final_speed
        
        # Calculate cumulative distance traveled
        if step == 0:
            aircraft_info['takeoff_distance'] = 0.0
        
        aircraft_info['takeoff_distance'] += current_speed
        distance_traveled = aircraft_info['takeoff_distance']
        
        # Calculate current position along the path
        if total_distance > 0:
            t = min(distance_traveled / total_distance, 1.0)
        else:
            t = 1.0
        
        curr_x = start_x + dx * t
        curr_y = start_y + dy * t
        
        # Gradually adjust heading toward target (same as taxi behavior)
        heading = aircraft_info['heading']
        angle_diff = target_heading - heading
        
        # Normalize angle difference to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # Apply limited turn rate (same as taxi)
        if abs(angle_diff) > max_turn_per_step:
            heading += max_turn_per_step if angle_diff > 0 else -max_turn_per_step
        else:
            heading = target_heading
        
        aircraft_info['heading'] = heading
        
        # Update aircraft triangle position
        angle_rad = math.radians(heading)
        size = 12
        
        # Define triangle with nose at origin
        base_points = [
            (0, 0),              # Nose
            (-2*size, -size),    # Upper rear
            (-2*size, size)      # Lower rear
        ]
        
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        rotated_points = []
        for px, py in base_points:
            rx = px * cos_a - py * sin_a
            ry = px * sin_a + py * cos_a
            rotated_points.extend([curr_x + rx, curr_y + ry])
        
        canvas.coords(aircraft_info['triangle_id'], *rotated_points)
        canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)
        
        aircraft_info['position'] = (curr_x, curr_y)
        
        # Schedule next step with shorter delay for faster animation
        app.after(adjust_delay(30), animate_takeoff, step + 1)
    
    animate_takeoff()

# ==============================
# LANDING AIRCRAFT
def landing_aircraft(canvas, aircraft_info, runway_exit_node):
    """Animate aircraft landing from airborne node to runway exit with smooth deceleration and turning.
    
    Aircraft starts fast, gradually slows to taxi speed after passing runway threshold, 
    and uses proper turning logic when exiting runway.
    After exiting, it will taxi to an available stand, ignoring the exit point stop bar.
    """
    import math
    
    # Determine the full landing route (airborne → runway → runway exit → taxiway)
    # Based on runway exit node, determine the taxiway node and full runway path
    if runway_exit_node == "RWY09_C1":
        # Landing on runway 27 (from east to west), exiting at C
        # Must follow the runway centerline through all intermediate nodes
        route = ["RWY09_AirBorne", "RWY27_A1", "RWY27_B1", "RWY09_C1", "RWY09_C1_EXIT", "C1_HOLD", "TXY_C1"]
        deceleration_start = "RWY27_A1"  # Start slowing after this node
    else:  # runway_exit_node == "RWY27_B1"
        # Landing on runway 09 (from west to east), exiting at B
        # Must follow the runway centerline through all intermediate nodes
        route = ["RWY27_AirBorne", "RWY09_E1", "RWY09_D1", "RWY09_C1", "RWY27_B1", "RWY27_B1_EXIT", "B1_HOLD", "TXY_B1"]
        deceleration_start = "RWY09_E1"  # Start slowing after this node
    
    aircraft_info['route'] = route
    aircraft_info['route_index'] = 0
    aircraft_info.pop('segment_start_override', None)
    aircraft_info.pop('segment_start_override_idx', None)
    aircraft_info['deceleration_start'] = deceleration_start
    
    landing_frame_seconds = 0.03  # Matches app.after(adjust_delay(30), ...)
    target_runway_landing_seconds = 50.0

    def _base_landing_speed_for_distance(distance_traveled, total_distance):
        """Base cubic landing speed profile in px/frame (before calibration scaling)."""
        if total_distance <= 0:
            return 0.22
        if distance_traveled < total_distance * 0.25:
            return 1.6
        adjusted_distance = distance_traveled - (total_distance * 0.25)
        adjusted_total = total_distance * 0.75
        decel_progress = min(1.0, adjusted_distance / max(adjusted_total * 0.9, 1))
        ease = decel_progress * decel_progress * decel_progress
        return 1.6 - (1.38 * ease)

    def _estimate_runway_landing_duration_seconds(total_distance):
        """Numerically estimate 1x runway-landing duration using the cubic profile."""
        if total_distance <= 0:
            return 0.0
        simulated_distance = 0.0
        frame_count = 0
        max_frames = 200000
        while simulated_distance < total_distance and frame_count < max_frames:
            simulated_distance += max(0.01, _base_landing_speed_for_distance(simulated_distance, total_distance))
            frame_count += 1
        return frame_count * landing_frame_seconds

    # Pre-calculate total deceleration distance for smooth global speed control
    try:
        # Decelerate from spawn (index 0) to the runway exit node
        deceleration_start_idx = 0
        runway_exit_idx = route.index(runway_exit_node)
        
        # Calculate total distance from deceleration start to runway exit
        decel_total_distance = 0
        for i in range(deceleration_start_idx, runway_exit_idx + 1):
            node_a = route[i]
            node_b = route[i + 1] if i + 1 < len(route) else route[i]
            decel_total_distance += math.hypot(nodes[node_b][0] - nodes[node_a][0], 
                                               nodes[node_b][1] - nodes[node_a][1])
        
        aircraft_info['decel_total_distance'] = decel_total_distance
        aircraft_info['decel_distance_traveled'] = 0.0
        aircraft_info['deceleration_start_idx'] = deceleration_start_idx
        aircraft_info['runway_exit_idx'] = runway_exit_idx

        base_duration_s = _estimate_runway_landing_duration_seconds(decel_total_distance)
        if base_duration_s > 0:
            aircraft_info['landing_speed_scale'] = max(0.05, min(5.0, base_duration_s / target_runway_landing_seconds))
        else:
            aircraft_info['landing_speed_scale'] = 1.0
    except (ValueError, KeyError):
        aircraft_info['decel_total_distance'] = 1000
        aircraft_info['decel_distance_traveled'] = 0.0
        aircraft_info['deceleration_start_idx'] = 0
        aircraft_info['runway_exit_idx'] = len(route) - 1
        aircraft_info['landing_speed_scale'] = 1.0
    
    # Track status changes
    moved_to_taxiing = [False]

    def _rear_from_history(history, wheelbase):
        if not history:
            return nodes[route[0]]
        dist = 0.0
        for i in range(len(history) - 1, 0, -1):
            x1, y1 = history[i]
            x0, y0 = history[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if dist + seg >= wheelbase:
                t = (wheelbase - dist) / seg if seg > 1e-6 else 0.0
                rx = x1 + (x0 - x1) * t
                ry = y1 + (y0 - y1) * t
                return rx, ry
            dist += seg
        return history[0]

    def _animate_landing_taxi_style_segment(route_idx, current_node, next_node):
        start_override = None
        if aircraft_info.get('segment_start_override_idx') == route_idx:
            start_override = aircraft_info.get('segment_start_override')
            aircraft_info.pop('segment_start_override_idx', None)
            aircraft_info.pop('segment_start_override', None)

        segment_points, next_start_override = build_taxi_segment_points(
            route,
            route_idx,
            speed=0.24,
            start_override=start_override
        )
        if not segment_points:
            segment_points = [nodes[current_node]]

        if next_start_override is not None:
            aircraft_info['segment_start_override'] = next_start_override
            aircraft_info['segment_start_override_idx'] = route_idx + 1

        steps = len(segment_points)
        lookahead_points = 4
        point_spacing = 0.24

        if 'heading' not in aircraft_info:
            if steps > 1:
                x0, y0 = segment_points[0]
                x1, y1 = segment_points[min(lookahead_points, steps - 1)]
                aircraft_info['heading'] = math.degrees(math.atan2(y1 - y0, x1 - x0))
            else:
                sx, sy = nodes[current_node]
                ex, ey = nodes[next_node]
                aircraft_info['heading'] = math.degrees(math.atan2(ey - sy, ex - sx))

        def _point_at_cursor(cursor):
            if steps <= 1:
                return segment_points[0]
            idx = int(cursor)
            if idx >= steps - 1:
                return segment_points[-1]
            t = cursor - idx
            x0, y0 = segment_points[idx]
            x1, y1 = segment_points[idx + 1]
            return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

        def animate_segment(cursor=0.0):
            if cursor >= max(steps - 1, 0):
                aircraft_info['route_index'] = route_idx + 1
                aircraft_info['node'] = next_node
                aircraft_info['position'] = segment_points[-1]
                move_through_route()
                return

            curr_x, curr_y = _point_at_cursor(cursor)

            runway_exit_idx = aircraft_info.get('runway_exit_idx', len(route) - 1)
            if route_idx > runway_exit_idx and not moved_to_taxiing[0]:
                move_aircraft_status(aircraft_info['callsign'], 'Taxiing')
                moved_to_taxiing[0] = True

            size = 12
            wheelbase = 2 * size
            history = aircraft_info.setdefault('path_history', [])
            if not history:
                history.append(segment_points[0])
            rear_x, rear_y = _rear_from_history(history, wheelbase)
            aircraft_info['rear_pos'] = (rear_x, rear_y)

            heading = math.degrees(math.atan2(curr_y - rear_y, curr_x - rear_x))
            aircraft_info['heading'] = heading

            angle_rad = math.radians(heading)
            base_points = [
                (0, 0),
                (-wheelbase, -size),
                (-wheelbase, size)
            ]
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            rotated_points = []
            for px, py in base_points:
                rx = px * cos_a - py * sin_a
                ry = px * sin_a + py * cos_a
                rotated_points.extend([curr_x + rx, curr_y + ry])

            if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                app.after(adjust_delay(200), animate_segment, cursor)
                return

            canvas.coords(aircraft_info['triangle_id'], *rotated_points)
            canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)

            prev_x, prev_y = aircraft_info.get('position', (curr_x, curr_y))
            aircraft_info['position'] = (curr_x, curr_y)
            history.append((curr_x, curr_y))
            if len(history) > 400:
                del history[:-300]

            if route_idx <= runway_exit_idx:
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)
                decel_distance_traveled += math.hypot(curr_x - prev_x, curr_y - prev_y)
                aircraft_info['decel_distance_traveled'] = decel_distance_traveled

            # Keep cubic landing deceleration before runway exit, then taxi speed.
            if route_idx <= runway_exit_idx:
                decel_total_distance = aircraft_info.get('decel_total_distance', 1000)
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)
                if decel_distance_traveled < decel_total_distance * 0.25:
                    current_speed = 1.6
                else:
                    adjusted_distance = decel_distance_traveled - (decel_total_distance * 0.25)
                    adjusted_total = decel_total_distance * 0.75
                    decel_progress = min(1.0, adjusted_distance / max(adjusted_total * 0.9, 1))
                    ease = decel_progress * decel_progress * decel_progress
                    current_speed = 1.6 - (1.38 * ease)
                current_speed *= aircraft_info.get('landing_speed_scale', 1.0)
            else:
                current_speed = 0.22

            cursor_step = max(0.05, current_speed / max(point_spacing, 0.01))
            app.after(adjust_delay(30), animate_segment, cursor + cursor_step)

        animate_segment(0.0)
        return True
    
    def move_through_route():
        """Move aircraft through each segment of the landing route."""
        route_idx = aircraft_info.get('route_index', 0)
        
        if route_idx >= len(route) - 1:
            # Completed landing sequence, now taxi to stand
            aircraft_info['node'] = route[-1]
            aircraft_info['position'] = nodes[route[-1]]
            
            # Move to Taxiing status
            move_aircraft_status(aircraft_info['callsign'], 'Taxiing')
            
            # Use the reserved target stand (set when aircraft spawned)
            target_stand = aircraft_info.get('target_stand')
            if target_stand:
                # Taxi to the reserved stand, ignoring stop bars on exit
                aircraft_info['ignore_stop_bars'] = True
                app.after(adjust_delay(0), lambda: taxi_to_stand_after_landing(canvas, aircraft_info, target_stand))
            else:
                # Fallback: find any available stand (shouldn't happen but just in case)
                available_stand = find_available_stand()
                if available_stand:
                    aircraft_info['ignore_stop_bars'] = True
                    app.after(adjust_delay(0), lambda: taxi_to_stand_after_landing(canvas, aircraft_info, available_stand))
            
            return
        
        # Get current and next node
        current_node = route[route_idx]
        next_node = route[route_idx + 1]

        # Check if this is a post-runway-exit segment that should use taxi-style segment animation
        # Use build_taxi_segment_points to create smooth curved paths
        post_runway_exit_segments = {
            ("RWY27_B1", "RWY09_C1"),
            ("RWY09_C1", "RWY09_C1_EXIT"),
            ("RWY09_C1_EXIT", "C1_HOLD"),
            ("C1_HOLD", "TXY_C1"),
            ("RWY09_C1", "RWY27_B1"),
            ("RWY27_B1", "RWY27_B1_EXIT"),
            ("RWY27_B1_EXIT", "B1_HOLD"),
            ("B1_HOLD", "TXY_B1"),
        }
        
        is_post_runway_exit = (current_node, next_node) in post_runway_exit_segments
        
        if is_post_runway_exit:
            # Use taxi-style segment animation with path-based movement
            return _animate_landing_taxi_style_segment(route_idx, current_node, next_node)

        # Keep landing movement on the cubic distance-based profile so runway
        
        start_x, start_y = nodes[current_node]
        end_x, end_y = nodes[next_node]
        
        # Calculate distance and direction
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        
        # Calculate target angle for this segment
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Initialize heading if not set
        if 'heading' not in aircraft_info:
            aircraft_info['heading'] = target_angle
        
        # Determine speed based on position in landing sequence
        # Decelerate smoothly from spawn to runway exit: 20 → 5 px/frame
        deceleration_start_idx = aircraft_info.get('deceleration_start_idx', 0)
        runway_exit_idx = aircraft_info.get('runway_exit_idx', len(route) - 1)
        
        # Decelerate until runway exit; after exit maintain taxi speed
        is_decelerating = route_idx <= runway_exit_idx
        
        # Calculate number of steps
        avg_speed = (1.6 + 0.22) / 2  # Average between landing and taxi speed (50 second landing)
        steps = max(int(distance / avg_speed), 1)
        
        # Calculate look-ahead angle for smooth turning
        next_segment_angle = target_angle
        has_next_segment = False
        if route_idx + 2 < len(route):
            next_next_node = route[route_idx + 2]
            next_end_x, next_end_y = nodes[next_next_node]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        elif next_node == "RWY27_A1":
            next_end_x, next_end_y = nodes["RWY27_AirBorne"]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        elif next_node == "RWY09_E1":
            next_end_x, next_end_y = nodes["RWY09_AirBorne"]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        
        # Turn parameters (same as taxi)
        turn_zone_distance = 20  # Start turning within this distance of node
        max_turn_per_step = 1.35  # Degrees per step
        corner_cut_radius = 10   # Pixels to cut corner
        
        def animate_segment(step=0):
            # Initialize segment position and remaining distance on first step
            if step == 0:
                aircraft_info['segment_x'] = start_x
                aircraft_info['segment_y'] = start_y
                aircraft_info['segment_remaining_distance'] = distance
            
            # Check if we've reached the destination node (distance-based)
            current_x = aircraft_info.get('segment_x', start_x)
            current_y = aircraft_info.get('segment_y', start_y)
            dist_remaining = math.hypot(end_x - current_x, end_y - current_y)
            remaining = aircraft_info.get('segment_remaining_distance', dist_remaining)
            
            if remaining <= 1.0 or dist_remaining <= 1.0:
                # Reached next node - snap to exact position
                aircraft_info['route_index'] = route_idx + 1
                aircraft_info['node'] = next_node
                aircraft_info['position'] = nodes[next_node]
                aircraft_info['segment_x'] = end_x
                aircraft_info['segment_y'] = end_y
                # Continue to next segment
                move_through_route()
                return
            
            # Calculate progress through this segment
            progress = step / steps
            
            # Move to Taxiing status once aircraft has passed the runway exit node
            runway_exit_idx = aircraft_info.get('runway_exit_idx', len(route) - 1)
            if route_idx > runway_exit_idx and not moved_to_taxiing[0]:
                move_aircraft_status(aircraft_info['callsign'], 'Taxiing')
                moved_to_taxiing[0] = True
            
            # Calculate current speed with smooth cubic deceleration calibrated to ~50s runway landing at 1x
            if is_decelerating:
                # Use pre-calculated total deceleration distance
                decel_total_distance = aircraft_info.get('decel_total_distance', 1000)
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)
                
                # Maintain constant speed for first 25% of runway, then decelerate
                if decel_distance_traveled < decel_total_distance * 0.25:
                    # Constant landing speed for first 25%
                    current_speed = 1.6
                else:
                    # Calculate deceleration progress starting after 25% point
                    adjusted_distance = decel_distance_traveled - (decel_total_distance * 0.25)
                    adjusted_total = decel_total_distance * 0.75  # Remaining distance for deceleration
                    
                    # Complete deceleration at 90% of remaining distance
                    decel_progress = min(1.0, adjusted_distance / max(adjusted_total * 0.9, 1))
                    
                    # Cubic easing for smooth deceleration: 1.6 → 0.22 px/frame
                    ease = decel_progress * decel_progress * decel_progress
                    current_speed = 1.6 - (1.38 * ease)

                current_speed *= aircraft_info.get('landing_speed_scale', 1.0)
            else:
                # Constant taxi speed after runway exit
                current_speed = 0.22
            
            # Move along segment by actual distance this frame
            if distance > 0:
                ux = dx / distance
                uy = dy / distance
            else:
                ux, uy = 0.0, 0.0
            
            remaining = aircraft_info.get('segment_remaining_distance', distance)
            move_dist = min(current_speed, remaining)
            step_x = ux * move_dist
            step_y = uy * move_dist
            
            # Update deceleration tracker by actual movement
            if is_decelerating:
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)
                decel_distance_traveled += move_dist
                aircraft_info['decel_distance_traveled'] = decel_distance_traveled
            
            aircraft_info['segment_x'] += step_x
            aircraft_info['segment_y'] += step_y
            aircraft_info['segment_remaining_distance'] = max(0.0, remaining - move_dist)
            
            base_x = aircraft_info['segment_x']
            base_y = aircraft_info['segment_y']
            
            # Calculate distance to next node for turn zone detection
            dist_to_next_node = math.hypot(end_x - base_x, end_y - base_y)
            
            # Apply corner cutting if approaching a turn
            curr_x = base_x
            curr_y = base_y
            
            if has_next_segment and dist_to_next_node <= turn_zone_distance:
                # Calculate angle difference to next segment
                angle_diff_to_next = next_segment_angle - target_angle
                while angle_diff_to_next > 180:
                    angle_diff_to_next -= 360
                while angle_diff_to_next < -180:
                    angle_diff_to_next += 360
                
                if abs(angle_diff_to_next) > 5:
                    # Smooth corner cutting
                    linear_factor = 1.0 - (dist_to_next_node / turn_zone_distance)
                    cut_factor = linear_factor * linear_factor * linear_factor
                    
                    bisector_angle = target_angle + (angle_diff_to_next * 0.5)
                    bisector_rad = math.radians(bisector_angle)
                    
                    offset = corner_cut_radius * cut_factor
                    curr_x = base_x + offset * math.cos(bisector_rad)
                    curr_y = base_y + offset * math.sin(bisector_rad)
            
            # Determine which angle to turn toward
            if has_next_segment and dist_to_next_node <= turn_zone_distance:
                angle_diff_to_next = next_segment_angle - target_angle
                while angle_diff_to_next > 180:
                    angle_diff_to_next -= 360
                while angle_diff_to_next < -180:
                    angle_diff_to_next += 360
                
                if abs(angle_diff_to_next) > 5:
                    turn_target = next_segment_angle
                else:
                    turn_target = target_angle
            else:
                turn_target = target_angle
            
            # Gradually adjust heading with limited turn rate
            heading = aircraft_info['heading']
            angle_diff = turn_target - heading
            
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            if abs(angle_diff) > max_turn_per_step:
                heading += max_turn_per_step if angle_diff > 0 else -max_turn_per_step
            else:
                heading = turn_target
            
            aircraft_info['heading'] = heading
            
            # Maintain a short history of nose positions so we can sample the exact
            # path the nose took and place the rear a fixed `wheelbase` distance
            # behind it. This makes the rear follow the nose path precisely.
            wheelbase = 24  # pixels (2 * 12)
            history = aircraft_info.get('nose_history')
            if history is None:
                history = []
                aircraft_info['nose_history'] = history

            # Append current nose point with cumulative distance
            if not history:
                history.append((curr_x, curr_y, 0.0))
            else:
                lx, ly, lc = history[-1]
                dist = math.hypot(curr_x - lx, curr_y - ly)
                history.append((curr_x, curr_y, lc + dist))

            # Trim history to reasonable length (keep last 2000 px of path)
            while len(history) > 2 and (history[-1][2] - history[0][2]) > 2000:
                history.pop(0)

            # Desired cumulative distance for rear point
            curr_cum = history[-1][2]
            target_cum = max(0.0, curr_cum - wheelbase)

            # Find sample in history
            rear_x = history[0][0]
            rear_y = history[0][1]
            for i in range(len(history)-1, 0, -1):
                x1, y1, c1 = history[i-1]
                x2, y2, c2 = history[i]
                if c1 <= target_cum <= c2:
                    t = (target_cum - c1) / (c2 - c1) if c2 > c1 else 0.0
                    rear_x = x1 + (x2 - x1) * t
                    rear_y = y1 + (y2 - y1) * t
                    break

            # Smooth rear position slightly to avoid jitter at segment boundaries
            prev_rear = aircraft_info.get('rear_pos', (rear_x, rear_y))
            smooth_factor = 0.7
            rear_x = prev_rear[0] + (rear_x - prev_rear[0]) * smooth_factor
            rear_y = prev_rear[1] + (rear_y - prev_rear[1]) * smooth_factor
            aircraft_info['rear_pos'] = (rear_x, rear_y)

            # Compute heading from rear->nose so the whole aircraft aligns with path
            heading = math.degrees(math.atan2(curr_y - rear_y, curr_x - rear_x))
            aircraft_info['heading'] = heading

            # Draw triangle using computed heading
            angle_rad = math.radians(heading)
            size = 12
            base_points = [
                (0, 0),
                (-2*size, -size),
                (-2*size, size)
            ]
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            rotated_points = []
            for px, py in base_points:
                rx = px * cos_a - py * sin_a
                ry = px * sin_a + py * cos_a
                rotated_points.extend([curr_x + rx, curr_y + ry])

            if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                app.after(adjust_delay(200), animate_segment, step)
                return

            canvas.coords(aircraft_info['triangle_id'], *rotated_points)
            canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)

            aircraft_info['position'] = (curr_x, curr_y)
            
            # Schedule next step
            app.after(adjust_delay(30), animate_segment, step + 1)
        
        animate_segment()
    
    # Start moving through the route
    move_through_route()

def is_stand_available(stand_name):
    """Check if a specific stand is available (not occupied or reserved by another aircraft)."""
    if is_disabled_stand_node(stand_name):
        return False
    for callsign, info in active_aircraft.items():
        # Check if aircraft is physically at the stand
        if info.get('node', '') == stand_name:
            return False
        # Check if stand is reserved as target for a landing aircraft
        if info.get('target_stand', '') == stand_name:
            return False
    return True

def find_available_stand():
    """Find a random available stand (not occupied or reserved by another aircraft)."""
    import random
    available_stands = find_available_stands()
    if not available_stands:
        return None
    return random.choice(available_stands)

def find_available_stands():
    """Return a list of available stands (not occupied or reserved)."""
    stands = [
        "STAND1a", "STAND2a", "STAND3a", "STAND4a", "STAND5a", "STAND6a", "STAND7a", "STAND8a",
        "STAND9A", "STAND10A", "STAND11A", "STAND12A", "STAND13A", "STAND14A", "STAND15A", "STAND16A", "STAND17A", "STAND18A",
        "STAND19A", "STAND20A", "STAND21A", "STAND22A"
    ]
    stands = [stand for stand in stands if not is_disabled_stand_node(stand)]
    occupied_stands = set()
    for callsign, info in active_aircraft.items():
        node = info.get('node', '')
        if node in stands:
            occupied_stands.add(node)
        target = info.get('target_stand', '')
        if target in stands:
            occupied_stands.add(target)
    return [stand for stand in stands if stand not in occupied_stands]

def is_safe_to_move(callsign, next_x, next_y, min_distance=28):
    """Check that the next position keeps a minimum separation from other aircraft."""
    for other_callsign, info in active_aircraft.items():
        if other_callsign == callsign:
            continue
        other_pos = info.get('position')
        if not other_pos:
            continue
        if math.hypot(other_pos[0] - next_x, other_pos[1] - next_y) < min_distance:
            return False
    return True

def should_nr_yield_to_as_at_ar(callsign, merge_clearance_px=55):
    """Return True when NR->AR movement must yield to AS-priority traffic at AR merge."""
    ar_pos = nodes.get("AR")
    if not ar_pos:
        return False

    for other_callsign, info in active_aircraft.items():
        if other_callsign == callsign:
            continue

        # Focus this rule on departures/taxiing aircraft.
        other_status = aircraft_labels.get(other_callsign, {}).get('column')
        if other_status not in {'Departures', 'Taxiing'}:
            continue

        other_node = info.get('node')
        other_pos = info.get('position')
        route = info.get('route') or []
        route_idx = info.get('route_index', 0)
        route_idx = max(0, min(route_idx, len(route) - 1)) if route else 0

        # Immediate AS priority: aircraft at AS or actively on AS->AR segment.
        if other_node == "AS":
            return True
        if route and route_idx < len(route) - 1 and route[route_idx] == "AS" and route[route_idx + 1] == "AR":
            return True

        # Keep NR waiting while merge area around AR is still occupied by aircraft
        # coming from AS side.
        if other_pos and math.hypot(other_pos[0] - ar_pos[0], other_pos[1] - ar_pos[1]) <= merge_clearance_px:
            if other_node == "AR":
                prev_node = route[route_idx - 1] if route and route_idx > 0 else None
                if prev_node == "AS":
                    return True
            elif route:
                upcoming = route[route_idx:route_idx + 3]
                for i in range(len(upcoming) - 1):
                    if upcoming[i] == "AS" and upcoming[i + 1] == "AR":
                        return True

    return False

def should_nq_yield_to_as_at_aq(callsign, merge_clearance_px=55):
    """Return True when NQ->AQ movement must yield to AS-priority traffic at AQ merge."""
    aq_pos = nodes.get("AQ")
    if not aq_pos:
        return False

    for other_callsign, info in active_aircraft.items():
        if other_callsign == callsign:
            continue

        other_status = aircraft_labels.get(other_callsign, {}).get('column')
        if other_status not in {'Departures', 'Taxiing'}:
            continue

        other_node = info.get('node')
        other_pos = info.get('position')
        route = info.get('route') or []
        route_idx = info.get('route_index', 0)
        route_idx = max(0, min(route_idx, len(route) - 1)) if route else 0

        if other_node == "AS":
            return True
        if route and route_idx < len(route) - 1 and route[route_idx] == "AS" and route[route_idx + 1] == "AQ":
            return True

        if other_pos and math.hypot(other_pos[0] - aq_pos[0], other_pos[1] - aq_pos[1]) <= merge_clearance_px:
            if other_node == "AQ":
                prev_node = route[route_idx - 1] if route and route_idx > 0 else None
                if prev_node == "AS":
                    return True
            elif route:
                upcoming = route[route_idx:route_idx + 3]
                for i in range(len(upcoming) - 1):
                    if upcoming[i] == "AS" and upcoming[i + 1] == "AQ":
                        return True

    return False

def _distance_point_to_segment(px, py, ax, ay, bx, by):
    """Return (distance, t) from point P to line segment AB where t is projection factor on AB."""
    abx = bx - ax
    aby = by - ay
    ab_len2 = abx * abx + aby * aby
    if ab_len2 <= 1e-9:
        return math.hypot(px - ax, py - ay), 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / ab_len2
    t_clamped = max(0.0, min(1.0, t))
    proj_x = ax + abx * t_clamped
    proj_y = ay + aby * t_clamped
    return math.hypot(px - proj_x, py - proj_y), t

def is_pushback_path_clear(callsign, stand_node, target_node, corridor_half_width=30, node_clearance=40, route_lookahead=5):
    """Return True if no aircraft is blocking or converging on the pushback path.

    Checks:
    - Aircraft physically behind/in the pushback corridor (stand -> standN)
    - Aircraft already at/near standN where pushback exits
    - Aircraft with upcoming taxi route steps that enter the same stand pushback nodes
    - Full planned routes of aircraft in Taxiing/Departures status to prevent route obstructions
    """
    if stand_node not in nodes or target_node not in nodes:
        return True

    sx, sy = nodes[stand_node]
    tx, ty = nodes[target_node]

    stand_b_node = f"{stand_node[:-1]}b" if stand_node.endswith("a") else None
    conflict_nodes = {stand_node, target_node}
    if stand_b_node and stand_b_node in nodes:
        conflict_nodes.add(stand_b_node)
    conflict_nodes.update(edges.get(target_node, []))

    for other_callsign, info in active_aircraft.items():
        if other_callsign == callsign:
            continue

        other_node = info.get('node')
        other_pos = info.get('position')

        if other_node in conflict_nodes:
            return False

        if other_pos:
            dist_to_path, projection_t = _distance_point_to_segment(other_pos[0], other_pos[1], sx, sy, tx, ty)
            if -0.35 <= projection_t <= 1.4 and dist_to_path <= corridor_half_width:
                return False
            if math.hypot(other_pos[0] - tx, other_pos[1] - ty) <= node_clearance:
                return False

        route = info.get('route') or []
        if route:
            route_idx = info.get('route_index', 0)
            route_idx = max(0, min(route_idx, len(route) - 1))
            # For Taxiing/Departing aircraft, check entire remaining route to prevent obstruction.
            other_status = aircraft_labels.get(other_callsign, {}).get('column')
            if other_status in {'Taxiing', 'Departures'}:
                # Check full route ahead for departing aircraft to maximize lookahead
                remaining_route = route[route_idx:]
            else:
                # For other aircraft, use standard lookahead window
                remaining_route = route[route_idx:route_idx + route_lookahead]
            if any(node in conflict_nodes for node in remaining_route):
                return False
        elif info.get('route'):
            # If aircraft is queued with a route, also check it
            other_status = aircraft_labels.get(other_callsign, {}).get('column')
            if other_status in {'Taxiing', 'Departures', 'Interrupting'}:
                # Be conservative: block pushback if a departure/taxi aircraft is active
                return False

    return True

def get_stand_direction(stand_node):
    """Determine the direction aircraft should face at a given stand (nose toward a/A, tail toward b/B)."""
    # Extract the base stand name without the a/b or A/B suffix
    if stand_node.endswith('a') or stand_node.endswith('b'):
        # STAND1-8: lowercase a/b - vertical stands
        base = stand_node[:-1]
        a_node = base + 'a'
        b_node = base + 'b'
        if a_node in nodes and b_node in nodes:
            ax, ay = nodes[a_node]
            bx, by = nodes[b_node]
            # If a is above b (lower y), face north; if a is below b (higher y), face south
            if ay < by:
                return "north"
            else:
                return "south"
    elif stand_node.endswith('A') or stand_node.endswith('B'):
        # STAND9-22: uppercase A/B
        base = stand_node[:-1]
        a_node = base + 'A'
        b_node = base + 'B'
        if a_node in nodes and b_node in nodes:
            ax, ay = nodes[a_node]
            bx, by = nodes[b_node]
            # Check if vertical (same x) or horizontal (same y)
            if ax == bx:
                # Vertical: if A is below B (higher y), face south; if A is above B (lower y), face north
                if ay > by:
                    return "south"
                else:
                    return "north"
            else:
                # Horizontal: if A is left of B (lower x), face left; if A is right of B (higher x), face right
                if ax < bx:
                    return "left"
                else:
                    return "right"
    return "north"  # Default fallback

def direction_to_heading(direction):
    """Convert direction string to heading angle in degrees."""
    direction_map = {
        "north": -90,
        "south": 90,
        "east": 0,
        "right": 0,
        "west": -180,
        "left": -180
    }
    return direction_map.get(direction, -90)

def taxi_to_stand_after_landing(canvas, aircraft_info, destination_stand):
    """Special taxi function for landing aircraft that ignores stop bars.
    
    This is a modified version of taxi_aircraft that doesn't check stop bars
    when exiting the runway.
    """
    import math
    
    # Get current node from aircraft info
    current_node = aircraft_info.get('node', '')
    
    # Get active runway
    runway_selector = globals().get('runway_var')
    runway_in_use = runway_selector.get() if runway_selector else None

    # Determine mandatory runway-specific nodes for arrival taxi routing.
    required_nodes = []
    if runway_in_use == "27":
        required_nodes = ["AQ", "STAND19N"]
    elif runway_in_use == "09":
        required_nodes = ["AR", "STAND21N"]

    # Build a route that always includes the mandatory runway-dependent nodes.
    route = build_route_via_nodes(current_node, destination_stand, via_nodes=required_nodes)
    
    if not route:
        print(f"No route found from {current_node} to {destination_stand} via {required_nodes}")
        return
    
    aircraft_info['route'] = route
    aircraft_info['route_index'] = 0
    if not aircraft_info.get('path_history'):
        aircraft_info['path_history'] = [aircraft_info.get('position', nodes[current_node])]

    def _rear_from_history(history, wheelbase):
        if not history:
            return nodes[current_node]
        dist = 0.0
        for i in range(len(history) - 1, 0, -1):
            x1, y1 = history[i]
            x0, y0 = history[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if dist + seg >= wheelbase:
                t = (wheelbase - dist) / seg if seg > 1e-6 else 0.0
                rx = x1 + (x0 - x1) * t
                ry = y1 + (y0 - y1) * t
                return rx, ry
            dist += seg
        return history[0]
    
    def move_to_next_node():
        """Move aircraft to the next node in the route."""
        route_idx = aircraft_info.get('route_index', 0)
        
        if route_idx >= len(route) - 1:
            # Reached final destination (stand)
            aircraft_info['node'] = destination_stand
            aircraft_info['position'] = nodes[destination_stand]
            
            # Reorient aircraft to face correct direction at stand
            stand_direction = get_stand_direction(destination_stand)
            aircraft_info['direction'] = stand_direction
            aircraft_info['heading'] = direction_to_heading(stand_direction)
            
            # Update triangle to face the correct direction
            x, y = nodes[destination_stand]
            size = 12
            angle_rad = math.radians(aircraft_info['heading'])
            
            base_points = [
                (0, 0),
                (-2*size, -size),
                (-2*size, size)
            ]
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            rotated_points = []
            for px, py in base_points:
                rx = px * cos_a - py * sin_a
                ry = px * sin_a + py * cos_a
                rotated_points.extend([x + rx, y + ry])
            
            canvas.coords(aircraft_info['triangle_id'], *rotated_points)
            canvas.coords(aircraft_info['label_id'], x, y - size - 10)
            
            # Move to Arrivals status
            move_aircraft_status(aircraft_info['callsign'], 'Arrivals')

            if 'triangle_id' in aircraft_info and aircraft_info['triangle_id']:
                canvas.itemconfig(aircraft_info['triangle_id'], fill="blue")
            
            # Clear the ignore_stop_bars flag
            aircraft_info['ignore_stop_bars'] = False
            
            # Clear the target stand reservation now that we've arrived
            if 'target_stand' in aircraft_info:
                del aircraft_info['target_stand']

            if schedule_turnaround_cb:
                schedule_turnaround_cb(aircraft_info, destination_stand)
            
            return
        
        # Get current and next node positions
        current_node = route[route_idx]
        next_node = route[route_idx + 1]
        
        # For landing aircraft, ignore stop bars for the first few nodes after runway exit
        # This allows them to taxi straight through the exit point stop bar
        should_ignore_stopbar = aircraft_info.get('ignore_stop_bars', False) and route_idx < 3
        
        # Check if we're about to pass through a HOLD node (departing from it)
        # Only check if we're not ignoring stop bars
        if current_node.endswith('_HOLD') and not should_ignore_stopbar:
            # Check if stop bar at this hold point is illuminated (red lights on)
            stop_bar_illuminated = current_node in stop_bar_draw_ids and len(stop_bar_draw_ids.get(current_node, [])) > 0
            
            if not is_runway_clear() or stop_bar_illuminated:
                # Runway is occupied or stop bar is on, wait and check again
                aircraft_info['waiting_at_hold'] = True
                app.after(adjust_delay(1000), move_to_next_node)  # Check again
                return
        
        start_x, start_y = nodes[current_node]
        end_x, end_y = nodes[next_node]
        
        start_override = None
        if aircraft_info.get('segment_start_override_idx') == route_idx:
            start_override = aircraft_info.get('segment_start_override')
            aircraft_info.pop('segment_start_override_idx', None)
            aircraft_info.pop('segment_start_override', None)

        segment_points, next_start_override = build_taxi_segment_points(
            route,
            route_idx,
            speed=0.24,
            start_override=start_override
        )
        if not segment_points:
            segment_points = [nodes[current_node]]

        if next_start_override is not None:
            aircraft_info['segment_start_override'] = next_start_override
            aircraft_info['segment_start_override_idx'] = route_idx + 1

        # Calculate distance and direction
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        
        # Calculate target angle for this segment
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Initialize heading if not set
        if 'heading' not in aircraft_info:
            aircraft_info['heading'] = target_angle
        
        # Detect if this segment involves runway nodes for special corner-cutting behavior
        runway_turn_nodes = {"RWY27_A1", "RWY09_E1", "RWY09_C1", "RWY27_B1"}
        is_runway_segment = current_node in runway_turn_nodes or next_node in runway_turn_nodes
        
        # Calculate look-ahead angle for smooth turning
        next_segment_angle = target_angle
        has_next_segment = False
        if route_idx + 2 < len(route):
            next_next_node = route[route_idx + 2]
            next_end_x, next_end_y = nodes[next_next_node]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        
        # Turn parameters
        turn_zone_distance = 20  # Start turning/cutting corner within this distance
        max_turn_per_step = 1.35
        corner_cut_radius = 10   # Pixels to cut corner
        
        # Use the same taxi turning logic for runway and taxiway segments.
        use_distance_stepping = False
        
        if use_distance_stepping:
            # For runway segments, use distance-based animation like landing_aircraft
            avg_speed = 0.24  # Taxi speed in pixels per frame
            steps = max(int(distance / avg_speed), 1)
            
            def animate_segment(step=0):
                if step == 0:
                    aircraft_info['segment_x'] = start_x
                    aircraft_info['segment_y'] = start_y
                    aircraft_info['segment_remaining_distance'] = distance
                
                # Check if we've reached the destination node
                current_x = aircraft_info.get('segment_x', start_x)
                current_y = aircraft_info.get('segment_y', start_y)
                dist_remaining = math.hypot(end_x - current_x, end_y - current_y)
                remaining = aircraft_info.get('segment_remaining_distance', dist_remaining)
                
                if remaining <= 1.0 or dist_remaining <= 1.0:
                    # Reached next node
                    aircraft_info['route_index'] = route_idx + 1
                    aircraft_info['node'] = next_node
                    aircraft_info['position'] = nodes[next_node]
                    aircraft_info['segment_x'] = end_x
                    aircraft_info['segment_y'] = end_y
                    move_to_next_node()
                    return
                
                # Move along segment by distance this frame
                if distance > 0:
                    ux = dx / distance
                    uy = dy / distance
                else:
                    ux, uy = 0.0, 0.0
                
                remaining = aircraft_info.get('segment_remaining_distance', distance)
                move_dist = min(0.24, remaining)
                step_x = ux * move_dist
                step_y = uy * move_dist
                
                aircraft_info['segment_x'] += step_x
                aircraft_info['segment_y'] += step_y
                aircraft_info['segment_remaining_distance'] = max(0.0, remaining - move_dist)
                
                base_x = aircraft_info['segment_x']
                base_y = aircraft_info['segment_y']
                
                # Calculate distance to next node for turn zone detection
                dist_to_next_node = math.hypot(end_x - base_x, end_y - base_y)
                
                # Apply corner cutting if approaching a turn
                curr_x = base_x
                curr_y = base_y
                
                if has_next_segment and dist_to_next_node <= turn_zone_distance:
                    # Calculate angle difference to next segment
                    angle_diff_to_next = next_segment_angle - target_angle
                    while angle_diff_to_next > 180:
                        angle_diff_to_next -= 360
                    while angle_diff_to_next < -180:
                        angle_diff_to_next += 360
                    
                    if abs(angle_diff_to_next) > 5:
                        # Smooth corner cutting with cubic interpolation
                        linear_factor = 1.0 - (dist_to_next_node / turn_zone_distance)
                        cut_factor = linear_factor * linear_factor * linear_factor
                        
                        bisector_angle = target_angle + (angle_diff_to_next * 0.5)
                        bisector_rad = math.radians(bisector_angle)
                        
                        offset = corner_cut_radius * cut_factor
                        curr_x = base_x + offset * math.cos(bisector_rad)
                        curr_y = base_y + offset * math.sin(bisector_rad)
                
                # Determine which angle to turn toward
                if has_next_segment and dist_to_next_node <= turn_zone_distance:
                    angle_diff_to_next = next_segment_angle - target_angle
                    while angle_diff_to_next > 180:
                        angle_diff_to_next -= 360
                    while angle_diff_to_next < -180:
                        angle_diff_to_next += 360
                    
                    if abs(angle_diff_to_next) > 5:
                        turn_target = next_segment_angle
                    else:
                        turn_target = target_angle
                else:
                    turn_target = target_angle
                
                # Gradually adjust heading with limited turn rate
                heading = aircraft_info['heading']
                angle_diff = turn_target - heading
                
                while angle_diff > 180:
                    angle_diff -= 360
                while angle_diff < -180:
                    angle_diff += 360
                
                if abs(angle_diff) > max_turn_per_step:
                    heading += max_turn_per_step if angle_diff > 0 else -max_turn_per_step
                else:
                    heading = turn_target
                
                aircraft_info['heading'] = heading
                
                # Maintain path history for rear position calculation
                wheelbase = 24
                history = aircraft_info.get('nose_history')
                if history is None:
                    history = []
                    aircraft_info['nose_history'] = history
                
                if not history:
                    history.append((curr_x, curr_y, 0.0))
                else:
                    lx, ly, lc = history[-1]
                    dist = math.hypot(curr_x - lx, curr_y - ly)
                    history.append((curr_x, curr_y, lc + dist))
                
                # Trim history to reasonable length
                while len(history) > 2 and (history[-1][2] - history[0][2]) > 2000:
                    history.pop(0)
                
                # Find rear point from history
                curr_cum = history[-1][2]
                target_cum = max(0.0, curr_cum - wheelbase)
                
                rear_x = history[0][0]
                rear_y = history[0][1]
                for i in range(len(history)-1, 0, -1):
                    x1, y1, c1 = history[i-1]
                    x2, y2, c2 = history[i]
                    if c1 <= target_cum <= c2:
                        t = (target_cum - c1) / (c2 - c1) if c2 > c1 else 0.0
                        rear_x = x1 + (x2 - x1) * t
                        rear_y = y1 + (y2 - y1) * t
                        break
                
                # Compute heading from rear to nose
                heading = math.degrees(math.atan2(curr_y - rear_y, curr_x - rear_x))
                aircraft_info['heading'] = heading
                
                # Draw triangle
                angle_rad = math.radians(heading)
                size = 12
                base_points = [
                    (0, 0),
                    (-2*size, -size),
                    (-2*size, size)
                ]
                cos_a = math.cos(angle_rad)
                sin_a = math.sin(angle_rad)
                rotated_points = []
                for px, py in base_points:
                    rx = px * cos_a - py * sin_a
                    ry = px * sin_a + py * cos_a
                    rotated_points.extend([curr_x + rx, curr_y + ry])
                
                if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                    app.after(adjust_delay(200), animate_segment, step)
                    return
                
                canvas.coords(aircraft_info['triangle_id'], *rotated_points)
                canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)
                
                aircraft_info['position'] = (curr_x, curr_y)
                
                # Schedule next step
                app.after(adjust_delay(50), animate_segment, step + 1)
        
        else:
            # For non-runway segments, use original segment_points approach
            steps = len(segment_points)
            lookahead_points = 4

            if 'heading' not in aircraft_info:
                if steps > 1:
                    x0, y0 = segment_points[0]
                    x1, y1 = segment_points[min(lookahead_points, steps - 1)]
                    aircraft_info['heading'] = math.degrees(math.atan2(y1 - y0, x1 - x0))
                else:
                    aircraft_info['heading'] = math.degrees(math.atan2(end_y - start_y, end_x - start_x))
            
            def animate_segment(step=0):
                if step >= steps:
                    # Reached next node, move to the next segment
                    aircraft_info['route_index'] = route_idx + 1
                    aircraft_info['node'] = next_node
                    aircraft_info['position'] = segment_points[-1]
                    move_to_next_node()
                    return
                
                curr_x, curr_y = segment_points[step]

                if steps > 1:
                    look_idx = min(step + lookahead_points, steps - 1)
                    look_x, look_y = segment_points[look_idx]
                    turn_target = math.degrees(math.atan2(look_y - curr_y, look_x - curr_x))
                else:
                    turn_target = aircraft_info['heading']
                
                # Gradually adjust heading toward turn target
                heading = aircraft_info['heading']
                angle_diff = turn_target - heading
                
                while angle_diff > 180:
                    angle_diff -= 360
                while angle_diff < -180:
                    angle_diff += 360
                
                if abs(angle_diff) > max_turn_per_step:
                    heading += max_turn_per_step if angle_diff > 0 else -max_turn_per_step
                else:
                    heading = turn_target
                
                aircraft_info['heading'] = heading
                
                # Compute rear position from overall taxi history so turns stay consistent.
                size = 12
                wheelbase = 2 * size
                history = aircraft_info.setdefault('path_history', [])
                if not history:
                    history.append(segment_points[0])
                rear_x, rear_y = _rear_from_history(history, wheelbase)
                aircraft_info['rear_pos'] = (rear_x, rear_y)

                # Compute heading from rear->nose
                heading = math.degrees(math.atan2(curr_y - rear_y, curr_x - rear_x))
                aircraft_info['heading'] = heading

                # Draw triangle
                angle_rad = math.radians(heading)
                base_points = [
                    (0, 0),
                    (-2*size, -size),
                    (-2*size, size)
                ]
                cos_a = math.cos(angle_rad)
                sin_a = math.sin(angle_rad)
                rotated_points = []
                for px, py in base_points:
                    rx = px * cos_a - py * sin_a
                    ry = px * sin_a + py * cos_a
                    rotated_points.extend([curr_x + rx, curr_y + ry])

                if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                    app.after(adjust_delay(200), animate_segment, step)
                    return

                canvas.coords(aircraft_info['triangle_id'], *rotated_points)
                canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)

                aircraft_info['position'] = (curr_x, curr_y)
                history.append((curr_x, curr_y))
                if len(history) > 400:
                    del history[:-300]
                
                # Schedule next step
                app.after(adjust_delay(50), animate_segment, step + 1)
        
        animate_segment()
    
    move_to_next_node()

# ==============================
# HOME SCREEN DISPLAY
def build_home_screen():
    """Build the ATC home screen UI matching the provided PNG layout."""
    global main_canvas  # Make canvas accessible globally
    
    # Clear any existing widgets
    for widget in app.winfo_children():
        widget.destroy()

    # ===== TOP SECTION: Airport Map in green banner =====
    try:
        map_path = os.path.join(script_dir, "EGNX_Map_Zoom.tif")
        top_img = Image.open(map_path)

        # Scale proportionally to fit screen width
        original_width, original_height = top_img.size  # 2234x464
        scale_factor = screen_width / original_width
        new_width = screen_width
        new_height = int(original_height * scale_factor)
        top_img = top_img.resize((new_width, new_height), Image.Resampling.LANCZOS)
        
        # Create a canvas to draw on top of the image
        canvas_frame = ctk.CTkFrame(app)
        canvas_frame.pack(fill="x", expand=False, padx=0, pady=0)
        
        main_canvas = tk.Canvas(canvas_frame, width=new_width, height=new_height, highlightthickness=0)
        main_canvas.pack()
        
        # Draw the map image on the canvas
        top_photo = ImageTk.PhotoImage(top_img)
        main_canvas.create_image(0, 0, anchor="nw", image=top_photo)
        main_canvas.image = top_photo  # Keep a reference to prevent garbage collection

        # Radar display frame (placeholder)
        main_canvas.create_rectangle(10, 210, 645, 350, fill="black", outline="white", width=2)
        
        # Draw nodes and edges on top of the map
        draw_graph(main_canvas)
        
        # Draw stop bars (always enabled)
        draw_stop_bars(main_canvas)
        
    except Exception as e:
        print(f"Error loading map image: {e}")
        placeholder = ctk.CTkLabel(app, text="[Map image not available]", font=("Arial", 16, "bold"), text_color="white")
        placeholder.pack(pady=10)

    # ===== MIDDLE SECTION: Controls Row =====
    controls_main = ctk.CTkFrame(app)
    controls_main.pack(fill="x", padx=10, pady=10)

    # Left panel: Operations
    ops_frame = ctk.CTkFrame(controls_main)
    ops_frame.pack(side="left", padx=20, pady=10)
    
    ctk.CTkLabel(ops_frame, text="Operations", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 10))
    ops_var = tk.StringVar(value="Normal Ops")
    ctk.CTkRadioButton(ops_frame, text="Normal Ops", variable=ops_var, value="Normal Ops").pack(anchor="w", pady=5)
    ctk.CTkRadioButton(ops_frame, text="Low Visibility Ops", variable=ops_var, value="Low Visibility Ops").pack(anchor="w", pady=5)

    # Middle-left: Traffic Flow Rate
    tfr_frame = ctk.CTkFrame(controls_main)
    tfr_frame.pack(side="left", padx=20, pady=10)
    
    ctk.CTkLabel(tfr_frame, text="Traffic Flow Rate", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 10))
    tfr_var = tk.StringVar(value="Low")
    ctk.CTkRadioButton(tfr_frame, text="Low", variable=tfr_var, value="Low").pack(anchor="w", pady=5)
    ctk.CTkRadioButton(tfr_frame, text="Medium", variable=tfr_var, value="Medium").pack(anchor="w", pady=5)
    ctk.CTkRadioButton(tfr_frame, text="High", variable=tfr_var, value="High").pack(anchor="w", pady=5)

    # Middle-left-2: Runway in use
    runway_frame = ctk.CTkFrame(controls_main)
    runway_frame.pack(side="left", padx=20, pady=10)
    
    ctk.CTkLabel(runway_frame, text="Runway in use", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 10))
    runway_var = tk.StringVar(value="27")
    ctk.CTkRadioButton(runway_frame, text="09", variable=runway_var, value="09").pack(anchor="w", pady=5)
    ctk.CTkRadioButton(runway_frame, text="27", variable=runway_var, value="27").pack(anchor="w", pady=5)

    # Middle: LVP Improvement checkboxes
    lvp_frame = ctk.CTkFrame(controls_main)
    lvp_frame.pack(side="left", padx=20, pady=10)
    
    ctk.CTkLabel(lvp_frame, text="LVP Improvement", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 10))
    lvp_reduced_sep_var = tk.BooleanVar(value=False)
    lvp_adaptive_seq_var = tk.BooleanVar(value=False)
    lvp_StopBar_sep_var = tk.BooleanVar(value=False)
    
    # Stop bars are always enabled independently, checkbox is non-functional
    ctk.CTkCheckBox(lvp_frame, text="Stop bars", variable=lvp_StopBar_sep_var).pack(anchor="w", pady=5)
    ctk.CTkCheckBox(lvp_frame, text="Reduced separation", variable=lvp_reduced_sep_var).pack(anchor="w", pady=5)
    ctk.CTkCheckBox(lvp_frame, text="Adaptive sequencing", variable=lvp_adaptive_seq_var).pack(anchor="w", pady=5)
    
    # Toggle Graph Button
    ctk.CTkButton(lvp_frame, text="Toggle Nodes/Edges", command=toggle_graph_visibility).pack(anchor="w", pady=(10, 5))

    # Center: Movements per hour / Delays in columnar format (matches status board style)
    data_frame = ctk.CTkFrame(controls_main)
    data_frame.pack(side="left", padx=20, pady=0, fill="both", expand=True)

    # Metric variables - edit these values to update the display
    movements_per_hour_var = tk.StringVar(value="45")
    delays_var = tk.StringVar(value="15")
    avg_taxi_time_var = tk.StringVar(value="15min")
    runway_util_var = tk.StringVar(value="80secs")

    metrics = [
        ("Movements Per Hour", movements_per_hour_var),
        ("Delays", delays_var),
        ("Average Taxi Time", avg_taxi_time_var),
        ("Runway Utilisation", runway_util_var),
    ]

    for title, var in metrics:
        col_frame = ctk.CTkFrame(data_frame)
        col_frame.pack(side="left", fill="both", expand=True, padx=2, pady=2)

        header = ctk.CTkLabel(
            col_frame,
            text=title,
            font=("Arial", 12, "bold"),
            fg_color="#99ccff",
            text_color="black",
            height=32,
        )
        header.pack(fill="x")
        
        content = ctk.CTkFrame(col_frame, fg_color="#e6f5ff")
        content.pack(fill="both", expand=True)

        ctk.CTkLabel(
            content,
            textvariable=var,
            font=("Arial", 20, "bold"),
            text_color="black",
            anchor="center",
        ).pack(fill="both", expand=True, padx=4, pady=4)

    # Right: Play button for general activity
    def clear_existing_aircraft():
        global next_pushback_release_time
        if not main_canvas:
            return
        for callsign, ac_info in list(active_aircraft.items()):
            turnaround_job_id = ac_info.get('turnaround_job_id')
            if turnaround_job_id:
                try:
                    app.after_cancel(turnaround_job_id)
                except Exception:
                    pass
            pushback_wait_job_id = ac_info.get('pushback_wait_job_id')
            if pushback_wait_job_id:
                try:
                    app.after_cancel(pushback_wait_job_id)
                except Exception:
                    pass
            if 'radar_dot_id' in ac_info and ac_info['radar_dot_id']:
                main_canvas.delete(ac_info['radar_dot_id'])
            if 'radar_label_id' in ac_info and ac_info['radar_label_id']:
                main_canvas.delete(ac_info['radar_label_id'])
            if 'triangle_id' in ac_info and ac_info['triangle_id']:
                main_canvas.delete(ac_info['triangle_id'])
            if 'label_id' in ac_info and ac_info['label_id']:
                main_canvas.delete(ac_info['label_id'])
            remove_aircraft_from_status(callsign)
        active_aircraft.clear()
        next_pushback_release_time = 0.0

    def generate_unique_callsign(prefix=None):
        import random

        def make_suffix():
            pattern = random.choice(["NNN", "NLL", "NNLL"])
            if pattern == "NNN":
                return f"{random.randint(100, 999)}"
            if pattern == "NLL":
                return f"{random.randint(1, 9)}{random.choice('ABCDEFGHIJKLMNOPQRSTUVWXYZ')}{random.choice('ABCDEFGHIJKLMNOPQRSTUVWXYZ')}"
            return f"{random.randint(10, 99)}{random.choice('ABCDEFGHIJKLMNOPQRSTUVWXYZ')}{random.choice('ABCDEFGHIJKLMNOPQRSTUVWXYZ')}"

        allowed_prefixes = ["RYR", "EZY", "EXS", "WIZZ"]
        chosen_prefix = random.choice(allowed_prefixes)

        for _ in range(50):
            callsign = f"{chosen_prefix}{make_suffix()}"
            if callsign not in active_aircraft:
                return callsign
        return f"{chosen_prefix}{make_suffix()}"

    def get_turnaround_delay_ms():
        import random
        rate = tfr_var.get()
        if rate == "High":
            return random.randint(15 * 60 * 1000, 30 * 60 * 1000)
        if rate == "Medium":
            return random.randint(20 * 60 * 1000, 40 * 60 * 1000)
        return random.randint(30 * 60 * 1000, 60 * 60 * 1000)

    def get_seeded_departure_delay_seconds():
        import random
        rate = tfr_var.get()
        if rate == "High":
            return random.randint(15, 60)
        if rate == "Medium":
            return random.randint(20, 90)
        return random.randint(30, 120)

    def start_departure_from_stand(aircraft_info, stand_node):
        """Convert a parked aircraft into a departure and start pushback."""
        if not main_canvas:
            return
        if is_disabled_stand_node(stand_node):
            return

        callsign = aircraft_info.get('callsign')
        if not callsign or callsign not in active_aircraft:
            return

        if aircraft_info.get('node') != stand_node:
            return

        # Pushback targets should be stand-facing N-style nodes so aircraft reverse out then
        # complete the 90-degree turn ready to taxi toward the active runway.
        stand_pushback_node_map = {
            "STAND8A": "STAND8N_2",
            "STAND9A": "STAND1N",
            "STAND10A": "STAND2N",
            "STAND11A": "STAND3N",
            "STAND12A": "STAND4N",
            "STAND13A": "NS",
            "STAND14A": "STAND5N",
            "STAND15A": "STAND6N",
            "STAND16A": "STAND7N",
            "STAND17A": "STAND8N",
            "STAND18A": "STAND18N",
            "STAND19A": "STAND19N",
            "STAND20A": "STAND20N",
            "STAND21A": "STAND21N",
            "STAND22A": "STAND22N",
        }

        stand_pushback_node = stand_pushback_node_map.get(stand_node, f"{stand_node[:-1]}N")
        if stand_pushback_node not in nodes:
            print(f"No valid pushback node found for stand {stand_node}: {stand_pushback_node}")
            return

        runway = runway_var.get()
        # Runway 09 departures taxi toward E1 (west/left), runway 27 departures taxi toward A1 (east/right)
        final_direction = "left" if runway == "09" else "right"
        runway_target = "A1_HOLD" if runway == "27" else "E1_HOLD"

        # Special pushback heading handling for side stands.
        # For runway 27, stands 19/20 should remain south-facing during most of pushback,
        # then complete the 90-degree turn near the stand-area exit.
        if runway == "27" and stand_node in {"STAND19A", "STAND20A"}:
            aircraft_info['pushback_start_direction'] = "south"
        else:
            aircraft_info.pop('pushback_start_direction', None)

        if callsign in aircraft_labels:
            move_aircraft_status(callsign, 'Departures')
        else:
            add_aircraft_to_status(callsign, 'Departures')

        def attempt_pushback():
            global next_pushback_release_time
            import random

            if callsign not in active_aircraft:
                return
            if aircraft_info.get('node') != stand_node:
                return

            now_sim = get_simulation_time()
            if now_sim < next_pushback_release_time:
                aircraft_info['waiting_for_pushback_clearance'] = True
                aircraft_info['pushback_wait_job_id'] = app.after(adjust_delay(1000), attempt_pushback)
                return

            if not is_pushback_path_clear(callsign, stand_node, stand_pushback_node):
                aircraft_info['waiting_for_pushback_clearance'] = True
                aircraft_info['pushback_wait_job_id'] = app.after(adjust_delay(1000), attempt_pushback)
                return

            aircraft_info['waiting_for_pushback_clearance'] = False
            aircraft_info.pop('pushback_wait_job_id', None)

            # Once this pushback starts, reserve the next pushback release at a random
            # interval between 30 seconds and 3 minutes.
            next_pushback_release_time = now_sim + random.randint(MIN_PUSHBACK_GAP_SECONDS, MAX_PUSHBACK_GAP_SECONDS)
            pushback_aircraft(main_canvas, aircraft_info, stand_pushback_node, final_direction, runway_target=runway_target)

        aircraft_info['pushback_wait_job_id'] = app.after(adjust_delay(1000), attempt_pushback)

    def schedule_turnaround(aircraft_info, stand_node):
        """Schedule a turnaround so an arriving aircraft later departs."""
        delay_ms = max(get_turnaround_delay_ms(), MIN_TURNAROUND_DELAY_MS)
        callsign = aircraft_info.get('callsign')

        def begin_departure():
            if callsign not in active_aircraft:
                return
            if aircraft_info.get('node') != stand_node:
                return
            start_departure_from_stand(aircraft_info, stand_node)

        aircraft_info['turnaround_job_id'] = app.after(adjust_delay(delay_ms), begin_departure)

    def schedule_seeded_departure(aircraft_info, stand_node):
        """Schedule initial stand departures with random timing."""

        callsign = aircraft_info.get('callsign')
        delay_ms = get_seeded_departure_delay_seconds() * 1000

        def begin_departure():
            if callsign not in active_aircraft:
                return
            if aircraft_info.get('node') != stand_node:
                return
            start_departure_from_stand(aircraft_info, stand_node)

        aircraft_info['turnaround_job_id'] = app.after(adjust_delay(delay_ms), begin_departure)

    global schedule_turnaround_cb
    schedule_turnaround_cb = schedule_turnaround

    def seed_initial_aircraft():
        """Seed 5-10 parked departures on random stands and stagger pushbacks."""
        if not main_canvas:
            return
        import random
        available_stands = find_available_stands()
        if not available_stands:
            return
        seed_count = min(len(available_stands), random.randint(5, 10))
        seed_stands = random.sample(available_stands, seed_count)

        for stand_node in seed_stands:
            x, y = nodes[stand_node]
            callsign = generate_unique_callsign("DEP")
            direction = get_stand_direction(stand_node)
            triangle_id, label_id = draw_aircraft(main_canvas, x, y, callsign, direction, color="blue")
            aircraft_info = {
                'callsign': callsign,
                'position': (x, y),
                'node': stand_node,
                'triangle_id': triangle_id,
                'label_id': label_id,
                'direction': direction
            }
            active_aircraft[callsign] = aircraft_info
            add_aircraft_to_status(callsign, 'Departures')
            schedule_seeded_departure(aircraft_info, stand_node)

    def spawn_landing_aircraft():
        """Spawn a landing aircraft at the appropriate airborne node based on runway direction."""
        if not main_canvas:
            return

        if not is_runway_clear():
            return

        available_stand = find_available_stand()
        if available_stand is None:
            print("Cannot spawn landing aircraft - no available stands")
            return

        runway = runway_var.get()
        
        # Determine spawn point based on runway
        if runway == "27":
            radar_spawn_node = "10m27"
        else:
            radar_spawn_node = "10m9"
        
        # Check arrival spacing - must be >3.5nm from any arrival at this specific spawn point
        if not can_spawn_new_arrival(radar_spawn_node):
            return

        callsign = generate_unique_callsign("ARR")

        if runway == "27":
            radar_start_node = "10m27"
            radar_end_node = "0m27"
            spawn_node = "RWY09_AirBorne"
            runway_exit = "RWY09_C1"
            direction = "left"
        else:
            radar_start_node = "10m9"
            radar_end_node = "0m9"
            spawn_node = "RWY27_AirBorne"
            runway_exit = "RWY27_B1"
            direction = "right"

        def animate_radar_inbound():
            if radar_start_node not in nodes or radar_end_node not in nodes:
                return

            if radar_start_node not in RUNWAY_TICK_NODES or radar_end_node not in RUNWAY_TICK_NODES:
                return

            start_idx = RUNWAY_TICK_NODES.index(radar_start_node)
            end_idx = RUNWAY_TICK_NODES.index(radar_end_node)
            if start_idx <= end_idx:
                path_nodes = RUNWAY_TICK_NODES[start_idx:end_idx + 1]
            else:
                path_nodes = list(reversed(RUNWAY_TICK_NODES[end_idx:start_idx + 1]))

            if len(path_nodes) < 2:
                return

            segment_duration_ms = 25000
            frame_interval_ms = 50
            steps_per_segment = max(1, int(segment_duration_ms / frame_interval_ms))
            dot_radius = 3

            start_x, start_y = nodes[path_nodes[0]]

            radar_dot_id = main_canvas.create_oval(
                start_x - dot_radius, start_y - dot_radius,
                start_x + dot_radius, start_y + dot_radius,
                fill="orange", outline="orange"
            )
            
            radar_label_id = main_canvas.create_text(
                start_x, start_y - 12,
                text=callsign,
                fill="white",
                font=("Arial", 8, "bold")
            )

            active_aircraft[callsign] = {
                'callsign': callsign,
                'position': (start_x, start_y),
                'node': radar_start_node,
                'radar_dot_id': radar_dot_id,
                'radar_label_id': radar_label_id,
                'direction': direction,
                'target_stand': available_stand
            }

            def move_segment(segment_idx=0, step_idx=0):
                if callsign not in active_aircraft:
                    main_canvas.delete(radar_dot_id)
                    if 'radar_label_id' in active_aircraft.get(callsign, {}):
                        main_canvas.delete(active_aircraft[callsign]['radar_label_id'])
                    return

                if segment_idx >= len(path_nodes) - 1:
                    main_canvas.delete(radar_dot_id)
                    if 'radar_label_id' in active_aircraft[callsign]:
                        main_canvas.delete(active_aircraft[callsign]['radar_label_id'])
                    spawn_x, spawn_y = nodes[spawn_node]
                    triangle_id, label_id = draw_aircraft(
                        main_canvas, spawn_x, spawn_y, callsign, direction, color="orange"
                    )
                    aircraft_info = {
                        'callsign': callsign,
                        'position': (spawn_x, spawn_y),
                        'node': spawn_node,
                        'triangle_id': triangle_id,
                        'label_id': label_id,
                        'direction': direction,
                        'target_stand': available_stand
                    }
                    active_aircraft[callsign] = aircraft_info
                    # Move to Runway status immediately to prevent departures
                    move_aircraft_status(callsign, 'Runway')
                    app.after(adjust_delay(500), lambda: landing_aircraft(main_canvas, aircraft_info, runway_exit))
                    return

                seg_start = nodes[path_nodes[segment_idx]]
                seg_end = nodes[path_nodes[segment_idx + 1]]
                dx = seg_end[0] - seg_start[0]
                dy = seg_end[1] - seg_start[1]
                t = min(1.0, step_idx / steps_per_segment)
                x = seg_start[0] + dx * t
                y = seg_start[1] + dy * t
                main_canvas.coords(
                    radar_dot_id,
                    x - dot_radius, y - dot_radius,
                    x + dot_radius, y + dot_radius
                )
                if 'radar_label_id' in active_aircraft[callsign]:
                    main_canvas.coords(active_aircraft[callsign]['radar_label_id'], x, y - 12)
                active_aircraft[callsign]['position'] = (x, y)

                if step_idx >= steps_per_segment:
                    active_aircraft[callsign]['node'] = path_nodes[segment_idx + 1]
                    app.after(adjust_delay(frame_interval_ms), lambda: move_segment(segment_idx + 1, 0))
                    return

                app.after(adjust_delay(frame_interval_ms), lambda: move_segment(segment_idx, step_idx + 1))

            move_segment()

        add_aircraft_to_status(callsign, 'Arrivals')
        animate_radar_inbound()

    def get_activity_delay_ms():
        import random
        rate = tfr_var.get()
        if rate == "High":
            return random.randint(6000, 12000)
        if rate == "Medium":
            return random.randint(10000, 20000)
        return random.randint(16000, 30000)

    def schedule_next_activity():
        global simulation_running, activity_job_id
        if not simulation_running:
            return

        available_stands = find_available_stands()
        can_arrive = len(available_stands) > 0 and is_runway_clear()

        if can_arrive:
            spawn_landing_aircraft()

        delay_ms = get_activity_delay_ms()
        activity_job_id = app.after(adjust_delay(delay_ms), schedule_next_activity)

    def start_activity():
        global simulation_running, activity_job_id
        if simulation_running:
            return
        clear_existing_aircraft()
        simulation_running = True
        seed_initial_aircraft()
        schedule_next_activity()
    
    # Button panel for run and speed control
    button_panel = ctk.CTkFrame(controls_main)
    button_panel.pack(side="right", padx=20, pady=10)
    
    run_btn = ctk.CTkButton(
        button_panel,
        text="▶ Play",
        font=("Arial", 14, "bold"),
        fg_color="#1e88e5",
        hover_color="#1565c0",
        height=60,
        width=200,
        command=start_activity
    )
    run_btn.pack(pady=(0, 10))
    
    # Speed control - toggle button that cycles through speeds
    speed_options = [1.0, 10.0, 50.0]
    speed_index = [0]  # Use list to allow modification in nested function
    speed_btn_ref = []  # Store reference to button for text updates
    
    def cycle_speed():
        speed_index[0] = (speed_index[0] + 1) % len(speed_options)
        set_simulation_speed(speed_options[speed_index[0]])
        speed_btn_ref[0].configure(text=f"Speed: {speed_options[speed_index[0]]}x")
    
    speed_btn = ctk.CTkButton(
        button_panel,
        text="Speed: 1x",
        font=("Arial", 12, "bold"),
        fg_color="#4CAF50",
        hover_color="#45a049",
        height=40,
        width=150,
        command=cycle_speed
    )
    speed_btn.pack(pady=(0, 10))
    speed_btn_ref.append(speed_btn)

    # ===== BOTTOM SECTION: Status Board =====
    status_frame = ctk.CTkFrame(app)
    status_frame.pack(fill="both", expand=True, padx=10, pady=10)

    # Header row with 5 columns
    columns = ["Departures", "Taxiing", "Runway", "Airborne", "Arrivals"]
    global status_columns
    status_columns = {}
    
    for col_name in columns:
        col_frame = ctk.CTkFrame(status_frame, corner_radius=10)
        col_frame.pack(side="left", fill="both", expand=True, padx=2, pady=2)
        col_frame.pack_propagate(False)
        
        header = ctk.CTkLabel(
            col_frame, 
            text=col_name, 
            font=("Arial", 12, "bold"),
            fg_color="#99ccff",
            text_color="black",
            height=40
        )
        header.pack(fill="x")
        
        content = ctk.CTkFrame(col_frame, fg_color="#cce5ff")
        content.pack(fill="both", expand=True)
        content.pack_propagate(False)
        content.configure(height=80)
        
        # Store reference to content frame
        status_columns[col_name] = content


if __name__ == "__main__":
    if not HAS_CTK:
        print("customtkinter is not installed. To run the GUI locally, install customtkinter and run this file directly.")
    else:
        app = ctk.CTk()
        app.title("Airport Control Panel")
        app.geometry(f"{screen_width}x{screen_height}+100+100")
        app.minsize(800, 600)
        app.state("zoomed")
        # Build the home screen and start the GUI
        build_home_screen()
        # Start continuous stop bar updates
        continuous_stop_bar_update()
        app.mainloop()
from PIL import Image, ImageTk
import warnings
import math
import heapq
import time
import ctypes
import sys
from tkinter import ttk
import tkinter as tk
import os
import csv
from datetime import datetime

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
    "STAND19N": (771,200), "STAND21N": (1320,200),
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
    "QUEBEC": (771, 260), "SIERRA": (1045, 260), "ROMEO": (1320, 260),
    "N_1": (1022, 283), "N_2": (1068, 283),
    "AQ": (771, 158), "NQ": (771, 283),
    "AR": (1320, 158), "NR": (1320, 283),
    "AS": (1045, 158), "NS": (1045, 283),
    "TXY_A1": (1820, 158),"A1_HOLD": (1820, 120), "RWY27_A1": (1820, 70), "RWY27_A1_ALIGN": (1760, 70), "RWY09_AirBorne": (1920,70),
    "A2_HOLD": (1790,158),"E2_HOLD":(135,158),"D2_HOLD":(214,137),"B2_HOLD":(1558,137),"C2_HOLD":(645,137),
    "A3": (1510, 158), "A4": (1180, 158), "A5": (886, 158), "A6": (560, 158), "A7": (315, 158),
    "S1": (1730, 158), "S2": (1670, 158), "S3": (1610, 158), "S4": (1450, 158), "S5": (1390, 158), "S6": (1250, 158),"S7": (1120, 158), 
    "S8": (965, 158), "S9": (829, 158), "S10": (710, 158), "S11": (499, 158), "S12": (438, 158), "S13": (377, 158), "S14": (265, 158), "S15": (174, 158),
    "Stand_Stop_Bar": (1045, 200),
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
    "D1_HOLD": ["D2_HOLD"],
    "TXY_D1": ["S14","D2_HOLD","S15"],
    "E2_HOLD": ["TXY_E1","S15"],
    "TXY_C1": ["C2_HOLD","S10","A6"], "C2_HOLD": ["C1_HOLD"],
    "TXY_B1": ["B2_HOLD","A3","S3"],
    "B2_HOLD": ["B1_HOLD",],
    "TXY_A1": ["A2_HOLD","A1_HOLD"],
    "AQ": ["STAND19N","S10","S9"],
    "AS": ["S8","S7","Stand_Stop_Bar"],
    "A7": ["S14", "S13"], "S12": ["S13", "S11"], "A6": ["S11", "TXY_C1"],
    "A5": ["S9", "S8"], "A4": ["S6", "S7"], "S4": ["S5", "A3"],
    "S2": ["S3", "S1"], "S1": ["A2_HOLD"],
    "NS": ["N_1","N_2","SIERRA"],   
    "STAND5N":["N_2", "STAND6N"],
    "STAND7N": ["STAND6N","STAND8N"],
    "STAND18N": ["STAND8N"],
    "SIERRA": ["Stand_Stop_Bar"],
    "NQ": ["STAND8N_2","QUEBEC"],
    "STAND1N": ["STAND1b","STAND9B","STAND8N_2","STAND2N"],
    "STAND3N": ["STAND2N","STAND4N"],
    "STAND4N": ["N_1"],
    "QUEBEC": ["STAND19N"],
    "STAND8B": ["STAND8N_2","STAND8A"],
    "NR": ["STAND18N","ROMEO"],
    "STAND21N": ["ROMEO"],
    "AR": ["STAND21N","S6","S5"],
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
    # Check permanent disabled stands
    if node_name in DISABLED_STAND_NODES:
        return True
    
    # Check runway-specific disabled stands
    runway_selector = globals().get('runway_var')
    if runway_selector:
        runway_in_use = runway_selector.get() if runway_selector else None
        if runway_in_use == "09" and node_name in {"STAND18A", "STAND18B"}:
            return True
        if runway_in_use == "27" and node_name in {"STAND8A", "STAND8B"}:
            return True
    
    return False

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

def build_taxi_segment_points(route, route_idx, speed=None, min_points=12, start_override=None, corner_cut=10.0):
    """Build points for a taxi segment: straight edge with optional rounded corner near the end.

    Returns (points, next_start_override), where next_start_override is a point on the next
    segment if this segment rounds a corner (used to avoid snapping back to the node).
    """
    if route_idx < 0 or route_idx + 1 >= len(route):
        return [], None

    if speed is None:
        speed = get_current_taxi_speed()

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

    # Keep exact node crossing at stop bars for procedural behavior.
    if is_stop_bar_node(route[route_idx + 1]):
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
current_operations_mode = "Normal Ops"
NORMAL_OPS_TAXI_SPEED = 3.6
LOW_VIS_TAXI_SPEED = 0.6
simulation_speed = 1.0  # Speed multiplier for simulation (1x, 10x)
simulation_time_seconds = 0.0
last_sim_real_time = time.perf_counter()
main_canvas = None  # Global reference to the canvas
show_template_s_node_stop_bars = False  # Toggle S-node template stop bars via checkbox
runtime_clock_var = None  # Elapsed simulation runtime display
sim_clock_var = None  # Simulated wall-clock display
status_columns = {}  # Global reference to status board columns
aircraft_labels = {}  # Track labels in each status column
graph_element_ids = []  # Store all graph element IDs for hiding/showing
graph_visible = True  # Track whether graph is currently visible
simulation_running = False
activity_job_id = None
schedule_turnaround_cb = None
current_low_visibility_blocks = None  # Runway-specific low-vis blocks, set after play button
current_low_visibility_runway = "27"  # Locked-in runway used for low-vis block selection
windows_timer_period_active = False
runway_var = None
arrival_spawning_paused = False

INBOUND_RADAR_SEGMENT_DURATION_SECONDS = 25.0
INBOUND_RADAR_UPDATE_INTERVAL_MS = 16

def build_movement_log_csv_path():
    """Create a timestamped movement log filename, e.g. Movement_log_13_1041.csv."""
    timestamp = datetime.now().strftime("%d_%H%M")
    base_name = f"Movement_log_{timestamp}"
    candidate = os.path.join(script_dir, f"{base_name}.csv")
    if not os.path.exists(candidate):
        return candidate

    suffix = 1
    while True:
        candidate = os.path.join(script_dir, f"{base_name}_{suffix}.csv")
        if not os.path.exists(candidate):
            return candidate
        suffix += 1


MOVEMENTS_LOG_CSV_PATH = build_movement_log_csv_path()

# Minimum gate turnaround before an arrival can push back (at 1x simulation speed).
MIN_TURNAROUND_DELAY_MS = 15 * 60 * 1000

# Pushback spacing between departures (at 1x simulation speed).
MIN_PUSHBACK_GAP_SECONDS = 30
MAX_PUSHBACK_GAP_SECONDS = 180
next_pushback_release_time = 0.0
manual_pushback_waiting = {}
pushback_method_var = None

# ==============================
# SPEED CONTROL HELPER
SIMULATION_START_SECONDS = 9 * 60 * 60

def adjust_delay(base_delay_ms):
    """Adjust a delay in milliseconds based on simulation speed multiplier."""
    return max(1, int(base_delay_ms / simulation_speed))


def configure_windows_timer_resolution():
    """Request 1ms timer resolution on Windows to stabilize tkinter after() timing."""
    global windows_timer_period_active
    if sys.platform != "win32" or windows_timer_period_active:
        return
    try:
        result = ctypes.windll.winmm.timeBeginPeriod(1)
        if result == 0:
            windows_timer_period_active = True
    except Exception:
        windows_timer_period_active = False


def restore_windows_timer_resolution():
    """Release the Windows timer resolution request if it was enabled."""
    global windows_timer_period_active
    if sys.platform != "win32" or not windows_timer_period_active:
        return
    try:
        ctypes.windll.winmm.timeEndPeriod(1)
    except Exception:
        pass
    windows_timer_period_active = False

def get_frame_timing_correction(base_delay_ms):
    """Return a multiplier to compensate for tkinter after() minimum-delay flooring.

    Example: base 30ms at 50x ideally runs every 0.6ms, but after() floors to 1ms.
    This returns 1.666... so per-frame movement is increased accordingly.
    """
    if simulation_speed <= 0:
        return 1.0
    desired_delay_ms = base_delay_ms / simulation_speed
    actual_delay_ms = adjust_delay(base_delay_ms)
    if desired_delay_ms <= 0:
        return 1.0
    return actual_delay_ms / desired_delay_ms

def get_simulation_time():
    """Return monotonically increasing simulation time in seconds.

    Simulation time advances as real elapsed time multiplied by current simulation speed,
    so changing the speed multiplier affects all clock-based timings immediately.
    """
    global simulation_time_seconds, last_sim_real_time
    now = time.perf_counter()
    if not simulation_running:
        last_sim_real_time = now
        return simulation_time_seconds
    elapsed_real = max(0.0, now - last_sim_real_time)
    simulation_time_seconds += elapsed_real * simulation_speed
    last_sim_real_time = now
    return simulation_time_seconds

def format_clock_time(total_seconds, wrap_24h=False):
    """Format a time value in seconds as HH:MM:SS."""
    total_seconds = max(0, int(total_seconds))
    if wrap_24h:
        total_seconds %= 24 * 60 * 60
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

def update_simulation_clock_display(current_sim_time=None):
    """Update runtime and simulated wall-clock labels."""
    if runtime_clock_var is None or sim_clock_var is None:
        return
    if current_sim_time is None:
        current_sim_time = get_simulation_time()
    runtime_clock_var.set(format_clock_time(current_sim_time))
    sim_clock_var.set(format_clock_time(SIMULATION_START_SECONDS + current_sim_time, wrap_24h=True))


def get_logging_operations_mode():
    return "low visibility" if current_operations_mode == "Low Visibility Ops" else "normal"


def get_logging_runway_in_use():
    runway_selector = globals().get('runway_var')
    if runway_selector is not None:
        try:
            return runway_selector.get()
        except Exception:
            pass
    return current_low_visibility_runway


def get_logging_lvp_stop_bars_enabled():
    return "yes" if show_template_s_node_stop_bars else "no"


def append_movement_log(callsign, movement_type):
    """Append a takeoff/landing movement row to the CSV movement log."""
    sim_clock = format_clock_time(SIMULATION_START_SECONDS + get_simulation_time(), wrap_24h=True)
    row = {
        "callsign": callsign,
        "movement_type": movement_type,
        "event_time": sim_clock,
        "operations_mode": get_logging_operations_mode(),
        "runway_in_use": get_logging_runway_in_use(),
        "lvp_improvement_stop_bars": get_logging_lvp_stop_bars_enabled(),
    }
    header = list(row.keys())

    try:
        needs_header = (not os.path.exists(MOVEMENTS_LOG_CSV_PATH)) or os.path.getsize(MOVEMENTS_LOG_CSV_PATH) == 0
        with open(MOVEMENTS_LOG_CSV_PATH, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=header)
            if needs_header:
                writer.writeheader()
            writer.writerow(row)
    except Exception as exc:
        print(f"Movement logging failed for {callsign}: {exc}")

def set_simulation_speed(new_speed):
    """Update simulation speed while preserving continuous simulation time."""
    global simulation_speed
    get_simulation_time()
    simulation_speed = new_speed


def set_arrival_spawning_paused(paused):
    """Enable or disable automatic arrival spawning."""
    global arrival_spawning_paused
    arrival_spawning_paused = bool(paused)

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
# Manual pixel coordinates for stop bar lights.
NORMAL_STOP_BAR_POSITIONS = {
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

LOW_VISIBILITY_ONLY_STOP_BAR_POSITIONS = {
    "A2_HOLD": {"red": [(1790, 148), (1790, 153), (1790, 158), (1790, 163), (1790, 168)]},
    "B2_HOLD": {"red": [(1548, 137), (1553, 137), (1558,137), (1563, 137), (1568, 137)]},
    "C2_HOLD": {"red": [(635, 137), (640, 137), (645,137), (650, 137), (655, 137)]},
    "D2_HOLD": {"red": [(204, 137), (209, 137), (214,137), (219, 137), (224, 137)]},
    "E2_HOLD": {"red": [(135, 148), (135, 153), (135, 158), (135, 163), (135, 168)]}, 
}

# Draw-only stop bars for Low Visibility Ops. These remain visual and do not
# participate in hold/runway-occupancy gating logic.
LOW_VISIBILITY_VISUAL_ONLY_STOP_BAR_POSITIONS = {
    "A3": {"red": [(1510, 148),(1510, 153),(1510, 158),(1510, 163),(1510, 168)]},
    "A4": {"red": [(1180, 148), (1180, 153), (1180, 158), (1180, 163), (1180, 168)]}, 
    "A5": {"red": [(886, 148), (886, 153), (886, 158), (886, 163), (886, 168)]}, 
    "A6": {"red": [(560, 148), (560, 153), (560, 158), (560, 163), (560, 168)]}, 
    "A7": {"red": [(315, 148), (315, 153), (315, 158), (315, 163), (315, 168)]},
    "Stand_Stop_Bar": {"red": [(1035, 200), (1040, 200), (1045, 200), (1050, 200), (1055, 200)]},
    "STAND19N": {"red": [(761, 200), (766, 200), (771, 200), (776, 200), (781, 200)]},
    "STAND21N": {"red": [(1310, 200), (1315, 200), (1320, 200), (1325, 200), (1330, 200)]},
    "N_1": {"red": [(1022, 273),(1022, 278),(1022, 283),(1022, 288),(1022, 293),]},
    "N_2": {"red": [(1068, 273),(1068, 278),(1068, 283),(1068, 288),(1068, 293),]},
}

# Template: fill in red-light pixel coordinates for S-node stop bars.
# When the "Stop bars" checkbox is enabled, any populated entries here are drawn.

S_NODE_STOP_BAR_TEMPLATE = {
    "S1": {"red": [(1730, 148), (1730, 153), (1730, 158), (1730, 163), (1730, 168)]},
    "S2": {"red": [(1670, 148), (1670, 153), (1670, 158), (1670, 163), (1670, 168)]},
    "S3": {"red": [(1610, 148), (1610, 153), (1610, 158), (1610, 163), (1610, 168)]},
    "S4": {"red": [(1450, 148), (1450, 153), (1450, 158), (1450, 163), (1450, 168)]},
    "S5": {"red": [(1390, 148), (1390, 153), (1390, 158), (1390, 163), (1390, 168)]},
    "S6": {"red": [(1250, 148), (1250, 153), (1250, 158), (1250, 163), (1250, 168)]},
    "S7": {"red": [(1120, 148), (1120, 153), (1120, 158), (1120, 163), (1120, 168)]},
    "S8": {"red": [(965, 148), (965, 153), (965, 158), (965, 163), (965, 168)]},
    "S9": {"red": [(829, 148), (829, 153), (829, 158), (829, 163), (829, 168)]},
    "S10": {"red": [(710, 148), (710, 153), (710, 158), (710, 163), (710, 168)]},
    "S11": {"red": [(499, 148), (499, 153), (499, 158), (499, 163), (499, 168)]},
    "S12": {"red": [(438, 148), (438, 153), (438, 158), (438, 163), (438, 168)]},
    "S13": {"red": [(377, 148), (377, 153), (377, 158), (377, 163), (377, 168)]},
    "S14": {"red": [(265, 148), (265, 153), (265, 158), (265, 163), (265, 168)]},
    "S15": {"red": [(174, 148), (174, 153), (174, 158), (174, 163), (174, 168)]},
    "SIERRA": {"red": [(1035, 260), (1040, 260), (1045, 260), (1050, 260), (1055, 260)]},
    "QUEBEC": {"red": [(761, 260), (766, 260), (771, 260), (776, 260), (781, 260)]}, 
    "ROMEO": {"red": [(1310, 260), (1315, 260), (1320, 260), (1325, 260), (1330, 260)]},
}

S_NODE_STOP_BAR_BLOCKS_RWY27 = {
    "S1": {"A2_HOLD"},
    "S2": {"S1"},
    "S3": {"S2"},
    "A3": {"S3"},
    "S4": {"A3"},
    "S5": {"S4"},
    "STAND21N": {"S5", "AR",},
    "S6": {"STAND21N", "AR", "S5", "ROMEO"},\
    "A4": {"S6"},
    "S7": {"A4"},
    "Stand_Stop_Bar": {"AS", "S7", "S8"},
    "S10": {"S9","AQ","STAND19N"},
    "SIERRA": {"Stand_Stop_Bar"},
    "STAND19N": {"QUEBEC"},
    "QUEBEC": {"N_1","NQ", "STAND8N_2", "STAND1N", "STAND2N", "STAND3N", "STAND4N"},
    "N_1": {"N_2","NS","SIERRA"},
    "N_2": {"STAND5N","STAND6N", "STAND7N","STAND8N","STAND18N","NR","ROMEO"},
    "ROMEO": {"STAND21N"},
}

S_NODE_STOP_BAR_BLOCKS_RWY09 = {
    "A3": {"S4"},
    "S4": {"S5"},
    "S5": {"AR", "STAND21N","S6"},
    "STAND21N": {"ROMEO"},
    "ROMEO": {"NR", "STAND18N", "STAND8N", "STAND7N", "STAND6N", "STAND5N","N_2"},
    "N_2": {"NS", "SIERRA", "N_1"},
    "N_1": {"STAND4N", "STAND3N", "STAND2N", "STAND1N", "STAND8N_2", "QUEBEC","NQ"},
    "QUEBEC": {"STAND19N"},
    "STAND19N": {"AQ", "S10"},
    "SIERRA": {"Stand_Stop_Bar"},
    "Stand_Stop_Bar": {"AS", "S7", "S8"},
    "S8": {"A5"},
    "A5": {"S9"},
    "S9": {"AQ", "S10", "STAND19N","QUEBEC"},
    "S10": {"A6"},
    "A6": {"S11"},
    "S11": {"S12"},
    "S12": {"S13"},
    "S13": {"A7"},
    "A7": {"S14"},
    "S14": {"S15"},
    "S15": {"E2_HOLD"},
}

LOW_VISIBILITY_STOP_BAR_POSITIONS = {
    **LOW_VISIBILITY_ONLY_STOP_BAR_POSITIONS,
    **LOW_VISIBILITY_VISUAL_ONLY_STOP_BAR_POSITIONS,
}

STOP_BAR_POSITIONS_BY_MODE = {
    "Normal Ops": NORMAL_STOP_BAR_POSITIONS,
    "Low Visibility Ops": LOW_VISIBILITY_STOP_BAR_POSITIONS,
}

STOP_BAR_CONTROLLED_NODES_BY_MODE = {
    "Normal Ops": set(NORMAL_STOP_BAR_POSITIONS.keys()),
    "Low Visibility Ops": set(LOW_VISIBILITY_ONLY_STOP_BAR_POSITIONS.keys()),
}

# Low-visibility sections with one-aircraft occupancy.
LOW_VISIBILITY_SINGLE_AIRCRAFT_SECTIONS = {
    "A2_HOLD",
    "A3",
    "A4",
    "STAND21N",
    "Stand_Stop_Bar",
    "A5",
    "STAND19N",
    "A6",
    "A7",
    "E2_HOLD",
    "N_1",
    "N_2",
}

# Low-visibility taxi blocks: the upstream stop bar can only deluminate when
# all nodes in the next block are clear.
# Base low visibility blocks (used as template)
# Runway 27 specific low visibility blocks
# Priority: STAND21N > A4; STAND19N does not interact with A5
LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY27 = {
    "A3": {"TXY_B1", "S3", "S2", "S1","A2_HOLD"},
    "A4": {"S4", "S5", "AR", "S6","A3", "STAND21N"},
    "STAND21N": {"AR","S5","S4", "A3","S6"},
    "A5": {"AQ", "S9","S10", "TXY_C1"},
    "A6": {"S11", "S12", "S13", "A7"},
    "A7": {"S14","TXY_D1", "S15","E2_HOLD"},
    "Stand_Stop_Bar": {"AS", "S7", "S8","A4","A5"},
    "STAND19N": {"QUEBEC", "NQ", "STAND8N_2", "STAND1N", "STAND2N", "STAND3N", "STAND4N", "N_1","NS","STAND1b","STAND2b","STAND3b","STAND4b","STAND9B","STAND10B","STAND11B","STAND12B"},
    "N_1": {"NS", "SIERRA", "Stand_Stop_Bar", "N_2"},
    "N_2": {"STAND5N","STAND6N", "STAND7N","STAND8N","STAND18N","NR", "ROMEO", "STAND21N"},
}

# Runway 09 specific low visibility blocks
# Priority: STAND19N > A5; STAND21N does not interact with A4
LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY09 = {
    "A3": {"S4" , "S5", "AR", "S6","STAND21N", "A4"},
    "A4": {"S4", "S5", "AR", "S6","A3"},
    "STAND21N": {"STAND5N","STAND6N", "STAND7N","STAND8N","STAND18N","NR", "ROMEO", "N_2"},
    "A5": {"AQ", "S9","S10", "TXY_C1", "STAND19N"},
    "A6": {"S11", "S12", "S13", "A7"},
    "A7": {"S14","TXY_D1", "S15","E2_HOLD"},
    "Stand_Stop_Bar": {"AS", "S7", "S8","A4","A5"},
    "STAND19N": {"AQ", "S10","TXY_C1", "S9", "A6"},
    "N_1": {"QUEBEC", "NQ", "STAND8N_2", "STAND1N", "STAND2N", "STAND3N", "STAND4N","STAND19N"},
    "N_2": {"NS", "SIERRA", "Stand_Stop_Bar", "N_1"},
}

def initialize_low_visibility_blocks(runway_in_use=None):
    """Initialize runway-specific low visibility blocks after play button is pressed.
    
    Called from start_activity() to lock in the runway selection.
    On Runway 27: STAND21N has priority over A4; STAND19N doesn't interact with A5
    On Runway 09: STAND19N has priority over A5; STAND21N doesn't interact with A4
    """
    global current_low_visibility_blocks, current_low_visibility_runway

    # Resolve runway from UI selection if caller did not pass it explicitly.
    if runway_in_use is None:
        runway_selector = globals().get('runway_var')
        runway_in_use = runway_selector.get() if runway_selector else None

    runway_in_use = str(runway_in_use).strip() if runway_in_use is not None else ""
    
    if runway_in_use == "09":
        current_low_visibility_blocks = dict(LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY09)
        current_low_visibility_runway = "09"
        print("RWY 09")
    elif runway_in_use == "27":
        current_low_visibility_blocks = dict(LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY27)
        current_low_visibility_runway = "27"
        print("RWY 27")
    else:
        # This should not happen if runway is always set before play button
        current_low_visibility_blocks = dict(LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY27)
        current_low_visibility_runway = "27"
        print("ELSE")


def get_template_stop_bar_blocks_for_runway():
    runway_in_use = str(current_low_visibility_runway).strip()
    if runway_in_use == "09":
        return S_NODE_STOP_BAR_BLOCKS_RWY09
    return S_NODE_STOP_BAR_BLOCKS_RWY27


def get_low_visibility_stop_bar_blocks():
    """Return the runway-locked low visibility stop bar blocks.
    
    Must be initialized by calling initialize_low_visibility_blocks() after the
    play button is pressed. Returns the blocks for the currently selected runway.
    """
    # When template stop bars are enabled, always use only the runway-specific
    # template block map and ignore LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY27/09.
    if show_template_s_node_stop_bars:
        template_blocks = get_template_stop_bar_blocks_for_runway()
        return {stop_bar_node: set(block_nodes) for stop_bar_node, block_nodes in template_blocks.items()}

    if current_low_visibility_blocks is None:
        # Should not happen in normal operation; use Runway 27 as fallback
        return dict(LOW_VISIBILITY_STOP_BAR_BLOCKS_RWY27)

    return dict(current_low_visibility_blocks)


def get_active_stop_bar_positions():
    active = dict(STOP_BAR_POSITIONS_BY_MODE.get(current_operations_mode, NORMAL_STOP_BAR_POSITIONS))
    if show_template_s_node_stop_bars:
        active.update(S_NODE_STOP_BAR_TEMPLATE)
    return active


def get_active_controlled_stop_bar_nodes():
    controlled_nodes = set(STOP_BAR_CONTROLLED_NODES_BY_MODE.get(
        current_operations_mode,
        set(NORMAL_STOP_BAR_POSITIONS.keys())
    ))

    return controlled_nodes


def get_active_visual_only_stop_bar_nodes():
    active_nodes = set(get_active_stop_bar_positions().keys())
    return active_nodes - get_active_controlled_stop_bar_nodes()


def get_active_stop_bar_nodes():
    return set(get_active_stop_bar_positions().keys())


def is_aircraft_approaching_node(node_name, proximity_px=10.0, require_proximity=False):
    """Return True when an aircraft is near or routing into the specified node."""
    if node_name not in nodes:
        return False

    target_x, target_y = nodes[node_name]

    for info in active_aircraft.values():
        current_node = info.get('node')
        route = info.get('route') or []
        route_idx = info.get('route_index', 0)

        # For proximity-based checks (visual stop bars), only consider aircraft
        # that are actively taxiing on a route to avoid pre-clearing bars while
        # aircraft are parked at a node after pushback.
        if require_proximity and not route:
            continue

        if current_node == node_name:
            if require_proximity:
                if route and 0 <= route_idx < len(route) - 1:
                    return True
            else:
                return True

        if route and 0 <= route_idx < len(route) - 1:
            if route[route_idx + 1] == node_name:
                if require_proximity:
                    pos = info.get('position')
                    if pos and math.hypot(pos[0] - target_x, pos[1] - target_y) <= proximity_px:
                        return True
                    continue
                return True

        pos = info.get('position')
        if pos and math.hypot(pos[0] - target_x, pos[1] - target_y) <= proximity_px:
            return True

    return False


def is_low_visibility_block_clear(stop_bar_node):
    """Return True when the configured downstream low-vis block is unoccupied."""
    if current_operations_mode != "Low Visibility Ops":
        return True

    blocks = get_low_visibility_stop_bar_blocks()
    block_nodes = blocks.get(stop_bar_node)
    if not block_nodes:
        return True

    for block_node in block_nodes:
        if is_low_visibility_section_occupied(block_node):
            return False

    return True


def is_low_visibility_section_occupied(section_node, exclude_callsign=None, proximity_px=18.0):
    """Return True when another aircraft is at or entering a mapped low-vis block node."""
    if current_operations_mode != "Low Visibility Ops":
        return False
    if section_node not in nodes:
        return False

    section_pos = nodes.get(section_node)
    for other_callsign, info in active_aircraft.items():
        if other_callsign == exclude_callsign:
            continue

        node = info.get('node')
        if node == section_node:
            return True

        route = info.get('route') or []
        route_idx = info.get('route_index', 0)
        if route and 0 <= route_idx < len(route) - 1:
            if route[route_idx + 1] == section_node:
                pos = info.get('position')
                if not section_pos or not pos or math.hypot(pos[0] - section_pos[0], pos[1] - section_pos[1]) <= proximity_px:
                    return True

    return False


def is_low_visibility_section_entry_allowed(callsign, current_node, next_node):
    """Return True when low-vis section capacity and downstream block checks pass."""
    if current_operations_mode != "Low Visibility Ops":
        return True

    if next_node in LOW_VISIBILITY_SINGLE_AIRCRAFT_SECTIONS:
        if is_low_visibility_section_occupied(next_node, exclude_callsign=callsign):
            return False

    blocks = get_low_visibility_stop_bar_blocks()
    block_nodes = blocks.get(current_node, set())
    for block_node in block_nodes:
        if is_low_visibility_section_occupied(block_node, exclude_callsign=callsign):
            return False

    return True


def get_current_taxi_speed():
    if current_operations_mode == "Low Visibility Ops":
        return LOW_VIS_TAXI_SPEED
    # Keep taxi speed independent of simulation speed.
    # Simulation acceleration is already handled by sim-time progression and adjusted delays.
    return NORMAL_OPS_TAXI_SPEED


def get_departure_gap_min_nodes():
    if current_operations_mode == "Low Visibility Ops":
        return 7
    return 3


def get_stop_bar_off_duration_seconds():
    if current_operations_mode == "Low Visibility Ops":
        return 30.0
    return 5.0


def is_stop_bar_node(node_name):
    return node_name in get_active_controlled_stop_bar_nodes()


def set_operations_mode(mode):
    global current_operations_mode

    current_operations_mode = mode
    stop_bar_off_until.clear()
    if main_canvas:
        update_stop_bars(main_canvas)


def set_template_s_node_stop_bars_enabled(enabled):
    """Toggle optional S-node template stop bars from the UI checkbox."""
    global show_template_s_node_stop_bars
    show_template_s_node_stop_bars = bool(enabled)
    if main_canvas:
        update_stop_bars(main_canvas)

def draw_stop_bars(canvas, current_time=None):
    """Draw stop bars for the currently selected operations mode.
    
    Red stop bar lights are always on, and turn off when a DEPARTING aircraft at that hold
    is cleared to cross (runway is clear). Landing aircraft do NOT affect stop bar state.
    Lights turn back on after a mode-specific off period.
    """
    global stop_bar_draw_ids, stop_bar_off_until, next_departure_release_time
    active_stop_bar_positions = get_active_stop_bar_positions()
    controlled_stop_bar_nodes = get_active_controlled_stop_bar_nodes()
    visual_only_stop_bar_nodes = get_active_visual_only_stop_bar_nodes()
    
    # Clear existing stop bars
    for items in stop_bar_draw_ids.values():
        for item_id in items:
            canvas.delete(item_id)
    stop_bar_draw_ids.clear()
    
    if current_time is None:
        current_time = get_simulation_time()
    
    # Check which hold points have DEPARTING aircraft that should turn off the stop bar
    # Landing aircraft (with ignore_stop_bars flag) should NOT affect stop bars
    for callsign, info in active_aircraft.items():
        node = info.get('node')
        # Only turn off stop bar if:
        # 1. Aircraft is at a hold point
        # 2. Runway is clear
        # 3. Aircraft is NOT a landing aircraft (doesn't have ignore_stop_bars flag)
        # 4. No arrival is within the current departure-gap threshold of the runway
        is_landing_aircraft = info.get('ignore_stop_bars', False)
        arrival_too_close = is_arrival_within_5nm()
        
        if node and is_stop_bar_node(node) and is_runway_clear() and not is_landing_aircraft and not arrival_too_close and is_low_visibility_block_clear(node):
            # If this hold's stop bar isn't already off, turn it off now and set timer
            if (node not in stop_bar_off_until or current_time >= stop_bar_off_until[node]) and current_time >= next_departure_release_time:
                stop_bar_duration = get_stop_bar_off_duration_seconds()
                stop_bar_off_until[node] = current_time + stop_bar_duration

    # Visual-only stop bars deluminate when traffic approaches those nodes.
    stop_bar_duration = get_stop_bar_off_duration_seconds()
    for hold_name in visual_only_stop_bar_nodes:
        if is_aircraft_approaching_node(hold_name, proximity_px=6.0, require_proximity=True) and is_low_visibility_block_clear(hold_name):
            stop_bar_off_until[hold_name] = current_time + stop_bar_duration
    
    for hold_name, positions in active_stop_bar_positions.items():
        if not positions.get("red"):
            continue  # Skip if no red positions defined
            
        items = []
        
        if hold_name in controlled_stop_bar_nodes:
            # Controlled stop bars are tied to runway protection logic.
            is_off = hold_name in stop_bar_off_until and current_time < stop_bar_off_until[hold_name] and not is_arrival_within_5nm()
        else:
            # Visual-only stop bars use local approach timer and optional
            # low-visibility downstream block occupancy checks.
            is_off = hold_name in stop_bar_off_until and current_time < stop_bar_off_until[hold_name]
            if not is_low_visibility_block_clear(hold_name):
                is_off = False
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

def update_stop_bars(canvas, current_time=None):
    """Update stop bar lights based on runway status."""
    if not canvas:
        return
    
    # Redraw all stop bars with current runway status
    draw_stop_bars(canvas, current_time)

def continuous_stop_bar_update():
    """Continuously update stop bars every 100ms for smooth timer-based updates."""
    current_time = get_simulation_time()
    update_simulation_clock_display(current_time)
    if main_canvas:
        update_stop_bars(main_canvas, current_time)
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
def taxi_aircraft(canvas, aircraft_info, destination_node, speed=None):
    """Animate aircraft taxiing from current node to destination using pathfinding."""
    import math

    if speed is None:
        speed = get_current_taxi_speed()
    
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

    runway_27_hold_target = "A2_HOLD" if current_operations_mode == "Low Visibility Ops" else "A1_HOLD"

    if runway_in_use == "27" and destination_node == runway_27_hold_target and (current_node.startswith("STAND") or current_node == "NS"):
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

    base_segment_speed = max(speed, 0.01)

    def _get_segment_speed(route_idx):
        segment_speed = speed
        runway_27_boost = (
            current_operations_mode == "Low Visibility Ops"
            and route
            and route[0] == "A2_HOLD"
            and destination_node == "RWY27_A1_ALIGN"
            and "RWY27_A1_ALIGN" in route
            and route.index("RWY27_A1_ALIGN") > route_idx
        )
        runway_09_boost = (
            current_operations_mode == "Low Visibility Ops"
            and route
            and route[0] == "E2_HOLD"
            and destination_node == "RWY09_E1_ALIGN"
            and "RWY09_E1_ALIGN" in route
            and route.index("RWY09_E1_ALIGN") > route_idx
        )
        if runway_27_boost or runway_09_boost:
            segment_speed *= 2.5
        return segment_speed

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
            if destination_node in ["A1_HOLD", "A2_HOLD", "E1_HOLD", "E2_HOLD"]:
                # Determine runway entry point based on hold point
                if destination_node in ["A1_HOLD", "A2_HOLD"]:
                    runway_entry = "RWY27_A1_ALIGN"
                else:  # E1_HOLD / E2_HOLD
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

        if not is_low_visibility_section_entry_allowed(aircraft_info['callsign'], current_node, next_node):
            aircraft_info['waiting_at_hold'] = True
            app.after(adjust_delay(500), move_to_next_node)
            if main_canvas:
                update_stop_bars(main_canvas)
            return

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
        
        # Stop-bar handling.
        # All active stop bars can hold taxiing traffic when illuminated.
        # Controlled stop bars additionally require runway and arrival-gap checks.
        active_stop_bar_nodes = get_active_stop_bar_nodes()
        controlled_stop_bar_nodes = get_active_controlled_stop_bar_nodes()
        if current_node in active_stop_bar_nodes:
            stop_bar_illuminated = current_node in stop_bar_draw_ids and len(stop_bar_draw_ids.get(current_node, [])) > 0

            if stop_bar_illuminated:
                aircraft_info['waiting_at_hold'] = True
                app.after(adjust_delay(1000), move_to_next_node)
                if main_canvas:
                    update_stop_bars(main_canvas)
                return

            if current_node in controlled_stop_bar_nodes:
                arrival_too_close = is_arrival_within_5nm()

                if not is_runway_clear() or arrival_too_close:
                    aircraft_info['waiting_at_hold'] = True
                    app.after(adjust_delay(1000), move_to_next_node)
                    if main_canvas:
                        update_stop_bars(main_canvas)
                    return

                # Controlled hold crossed: aircraft is entering runway flow.
                aircraft_info['waiting_at_hold'] = False
                move_aircraft_status(aircraft_info['callsign'], 'Runway')
                if main_canvas:
                    update_stop_bars(main_canvas)
            else:
                aircraft_info['waiting_at_hold'] = False
        
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

        segment_speed = _get_segment_speed(route_idx)
        segment_speed_scale = segment_speed / base_segment_speed

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
            avg_speed = segment_speed
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
                move_dist = min(segment_speed, remaining)
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
            point_spacing = base_segment_speed

            if 'heading' not in aircraft_info:
                if steps > 1:
                    x0, y0 = segment_points[0]
                    x1, y1 = segment_points[min(lookahead_points, steps - 1)]
                    aircraft_info['heading'] = math.degrees(math.atan2(y1 - y0, x1 - x0))
                else:
                    aircraft_info['heading'] = math.degrees(math.atan2(end_y - start_y, end_x - start_x))

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

            aircraft_info['taxi_last_sim_time'] = get_simulation_time()

            def animate_segment(cursor=0.0):
                if cursor >= max(steps - 1, 0):
                    # Reached next node, move to the next segment immediately without delay
                    aircraft_info['route_index'] = route_idx + 1
                    aircraft_info['node'] = next_node
                    aircraft_info['position'] = segment_points[-1]
                    # Continue immediately to next segment for smooth transition
                    move_to_next_node()
                    return

                curr_x, curr_y = _point_at_cursor(cursor)

                if steps > 1:
                    look_cursor = min(cursor + lookahead_points, max(steps - 1, 0))
                    look_x, look_y = _point_at_cursor(look_cursor)
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
                    aircraft_info['taxi_last_sim_time'] = get_simulation_time()
                    app.after(adjust_delay(200), animate_segment, cursor)
                    return
                
                canvas.coords(aircraft_info['triangle_id'], *rotated_points)
                canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)
                
                aircraft_info['position'] = (curr_x, curr_y)
                history.append((curr_x, curr_y))
                if len(history) > 400:
                    del history[:-300]

                now_sim = get_simulation_time()
                last_sim = aircraft_info.get('taxi_last_sim_time', now_sim)
                sim_dt = max(0.0, now_sim - last_sim)
                aircraft_info['taxi_last_sim_time'] = now_sim

                cursor_step = max(0.05, (base_segment_speed * sim_dt) / point_spacing) * segment_speed_scale

                # Schedule next step
                app.after(adjust_delay(50), animate_segment, cursor + cursor_step)
        
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
        
        # Check if aircraft is at a stop bar
        node = info.get('node')
        if node and is_stop_bar_node(node):
            return True
    
    return False

def can_spawn_new_arrival(spawn_node_name):
    """Check if a new arrival can spawn at the given spawn node.
    
    Spacing requirement:
    - Normal Ops: >=5.5 nodes from any arrival on final approach
    - Low Visibility Ops: >=8 nodes from any arrival on final approach
    
    Args:
        spawn_node_name: The node where the aircraft will spawn ("10m9" or "10m27")
    """
    if spawn_node_name not in RUNWAY_TICK_NODES:
        return False
    
    spawn_idx = RUNWAY_TICK_NODES.index(spawn_node_name)
    
    # Enforce arrival spacing minima based on operations mode.
    min_spacing = 10 if current_operations_mode == "Low Visibility Ops" else 5.5
    
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
    """Check if any arrival is within the departure-gap threshold of runway center.

    Normal Ops threshold is 3 nodes and Low Visibility threshold is 7 nodes.
    """
    center_idx_09 = RUNWAY_TICK_NODES.index("0m9")
    center_idx_27 = RUNWAY_TICK_NODES.index("0m27")
    min_nodes = get_departure_gap_min_nodes()

    for callsign, info in active_aircraft.items():
        node = info.get('node')
        if node and node in RUNWAY_TICK_NODES:
            idx = get_arrival_node_index(node)
            if idx is not None:
                dist_09 = abs(idx - center_idx_09)
                dist_27 = abs(idx - center_idx_27)
                if min(dist_09, dist_27) <= min_nodes:
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

    # Takeoff parameters as hard speeds (px per 30ms baseline frame)
    initial_speed = 0.22  # Starting speed - same as taxi speed (pixels per frame)
    final_speed = 2.21   # Speed at rotation/liftoff (pixels per frame)
    acceleration_distance = total_distance * 0.8  # Accelerate for 80% of runway
    max_turn_per_step = 8.0  # Same as taxi turning rate for consistency

    # Use simulation-time deltas for movement so speed multipliers are exact.
    base_frame_seconds = 0.03
    aircraft_info['takeoff_distance'] = 0.0
    aircraft_info['takeoff_last_sim_time'] = get_simulation_time()
    
    # Track if we've moved to airborne status
    moved_to_airborne = [False]
    
    def animate_takeoff():
        distance_traveled = aircraft_info.get('takeoff_distance', 0.0)
        if distance_traveled >= total_distance:
            append_movement_log(aircraft_info['callsign'], "takeoff")
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
        
        # Calculate progress (0.0 to 1.0) from physical distance travelled.
        if total_distance > 0:
            progress = min(1.0, distance_traveled / total_distance)
        else:
            progress = 1.0
        
        # Move to Airborne status column at approximately 50% of takeoff roll
        if progress >= 0.5 and not moved_to_airborne[0]:
            move_aircraft_status(aircraft_info['callsign'], 'Airborne')
            moved_to_airborne[0] = True
            # Update stop bars - runway is now clear
            if main_canvas:
                update_stop_bars(main_canvas)
        
        # Calculate current speed with acceleration
        if total_distance > 0 and progress < (acceleration_distance / total_distance):
            # Accelerating phase
            accel_progress = progress / (acceleration_distance / total_distance)
            current_speed = initial_speed + (final_speed - initial_speed) * accel_progress
        else:
            # Constant speed phase
            current_speed = final_speed

        # Scale movement by elapsed simulation time since last update.
        now_sim = get_simulation_time()
        last_sim = aircraft_info.get('takeoff_last_sim_time', now_sim)
        sim_dt = max(0.0, now_sim - last_sim)
        aircraft_info['takeoff_last_sim_time'] = now_sim

        move_dist = current_speed * (sim_dt / base_frame_seconds)
        distance_traveled = min(total_distance, distance_traveled + move_dist)
        aircraft_info['takeoff_distance'] = distance_traveled
        
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
        
        app.after(adjust_delay(30), animate_takeoff)
    
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

    # Hard-coded landing speed profile by runway progress. These values are per update,
    # and the speed multiplier is still handled by the existing af#ter() delay scaling.
    landing_speed_profiles = {
        1.0: [(0.00, 0.95), (0.75, 0.95), (0.82, 0.80), (0.90, 0.62), (0.96, 0.42), (1.00, 0.28)],
        10.0: [(0.00, 0.95), (0.75, 0.95), (0.82, 0.80), (0.90, 0.62), (0.96, 0.42), (1.00, 0.28)],
        50.0: [(0.00, 0.95), (0.75, 0.95), (0.82, 0.80), (0.90, 0.62), (0.96, 0.42), (1.00, 0.28)],
    }

    def get_landing_roll_speed(progress):
        """Return a hard-coded landing speed for the current runway progress."""
        profile = landing_speed_profiles.get(simulation_speed, landing_speed_profiles[1.0])
        progress = max(0.0, min(1.0, progress))
        if progress <= profile[0][0]:
            return profile[0][1]
        for index in range(1, len(profile)):
            p0, s0 = profile[index - 1]
            p1, s1 = profile[index]
            if progress <= p1:
                if p1 <= p0:
                    return s1
                blend = (progress - p0) / (p1 - p0)
                return s0 + (s1 - s0) * blend
        return profile[-1][1]

    # Pre-calculate total distance from airborne spawn to the configured runway
    # exit node (e.g. RWY09_AirBorne -> RWY09_C1).
    try:
        deceleration_start_idx = 0
        runway_exit_idx = route.index(runway_exit_node)
        
        decel_total_distance = 0
        for i in range(deceleration_start_idx, runway_exit_idx):
            node_a = route[i]
            node_b = route[i + 1]
            decel_total_distance += math.hypot(nodes[node_b][0] - nodes[node_a][0], 
                                               nodes[node_b][1] - nodes[node_a][1])
        
        aircraft_info['decel_total_distance'] = decel_total_distance
        aircraft_info['decel_distance_traveled'] = 0.0
        aircraft_info['deceleration_start_idx'] = deceleration_start_idx
        aircraft_info['runway_exit_idx'] = runway_exit_idx
    except (ValueError, KeyError):
        aircraft_info['decel_total_distance'] = 1000
        aircraft_info['decel_distance_traveled'] = 0.0
        aircraft_info['deceleration_start_idx'] = 0
        aircraft_info['runway_exit_idx'] = len(route) - 1
    
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
            speed=get_current_taxi_speed(),
            start_override=start_override
        )
        if not segment_points:
            segment_points = [nodes[current_node]]

        if next_start_override is not None:
            aircraft_info['segment_start_override'] = next_start_override
            aircraft_info['segment_start_override_idx'] = route_idx + 1

        steps = len(segment_points)
        lookahead_points = 4
        point_spacing = get_current_taxi_speed()

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

        aircraft_info['landing_last_sim_time'] = get_simulation_time()

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

            # Avoid taxi conflict hold while still on runway rollout.
            if route_idx > runway_exit_idx and not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
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

            # Keep landing movement on the hard-coded progress table until runway exit.
            if route_idx <= runway_exit_idx:
                decel_total_distance = aircraft_info.get('decel_total_distance', 1000)
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)
                if decel_total_distance > 0:
                    runway_progress = min(1.0, decel_distance_traveled / decel_total_distance)
                else:
                    runway_progress = 1.0
                current_speed = get_landing_roll_speed(runway_progress)
            else:
                current_speed = get_current_taxi_speed()

            now_sim = get_simulation_time()
            last_sim = aircraft_info.get('landing_last_sim_time', now_sim)
            sim_dt = max(0.0, now_sim - last_sim)
            aircraft_info['landing_last_sim_time'] = now_sim

            dt_factor = sim_dt / 0.03
            cursor_step = max(0.05, current_speed / max(point_spacing, 0.01)) * dt_factor
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
            # Check if target stand is still available (not disabled or occupied)
            if target_stand and (is_disabled_stand_node(target_stand) or target_stand not in find_available_stands()):
                # Target stand is no longer available, find a new one
                available_stand = find_available_stand()
                if available_stand:
                    target_stand = available_stand
                    aircraft_info['target_stand'] = available_stand
                else:
                    print(f"{aircraft_info['callsign']}: No available stands after landing")
                    return
            
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
                aircraft_info['landing_last_sim_time'] = get_simulation_time()
            
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
            
            # Move to Taxiing status once aircraft has passed the runway exit node
            runway_exit_idx = aircraft_info.get('runway_exit_idx', len(route) - 1)
            if route_idx > runway_exit_idx and not moved_to_taxiing[0]:
                move_aircraft_status(aircraft_info['callsign'], 'Taxiing')
                moved_to_taxiing[0] = True
            
            # Calculate current speed with hard-coded progress-based landing profile.
            if is_decelerating:
                decel_total_distance = aircraft_info.get('decel_total_distance', 1000)
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)

                if decel_total_distance > 0:
                    runway_progress = min(1.0, decel_distance_traveled / decel_total_distance)
                else:
                    runway_progress = 1.0

                current_speed = get_landing_roll_speed(runway_progress)
            else:
                # Constant taxi speed after runway exit
                current_speed = get_current_taxi_speed()
            
            # Move along segment by actual distance this frame
            if distance > 0:
                ux = dx / distance
                uy = dy / distance
            else:
                ux, uy = 0.0, 0.0
            
            now_sim = get_simulation_time()
            last_sim = aircraft_info.get('landing_last_sim_time', now_sim)
            sim_dt = max(0.0, now_sim - last_sim)
            aircraft_info['landing_last_sim_time'] = now_sim

            dt_factor = sim_dt / 0.03
            remaining = aircraft_info.get('segment_remaining_distance', distance)
            move_dist = min(current_speed * dt_factor, remaining)
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

            # Do not apply generic taxi conflict holds during runway landing rollout.
            # Runway occupancy is already protected at spawn logic; this avoids the
            # 200ms retry loop making landing look unnaturally slow from AirBorne.
            if not is_decelerating and not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
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

def is_safe_to_move(callsign, next_x, next_y, min_distance=40):
    """Check that the next position keeps safe separation from other aircraft.

    Nearby aircraft behind the movement direction should not block forward progress,
    but very-close rear overlap is still prevented.
    """
    own_info = active_aircraft.get(callsign, {})
    own_pos = own_info.get('position')

    move_dx = move_dy = 0.0
    move_len = 0.0
    if own_pos:
        move_dx = next_x - own_pos[0]
        move_dy = next_y - own_pos[1]
        move_len = math.hypot(move_dx, move_dy)

    for other_callsign, info in active_aircraft.items():
        if other_callsign == callsign:
            continue

        other_pos = info.get('position')
        if not other_pos:
            continue

        rel_dx = other_pos[0] - next_x
        rel_dy = other_pos[1] - next_y
        distance = math.hypot(rel_dx, rel_dy)
        if distance >= min_distance:
            continue

        if move_len > 1e-6:
            unit_dx = move_dx / move_len
            unit_dy = move_dy / move_len
            forward_projection = rel_dx * unit_dx + rel_dy * unit_dy

            # If nearby traffic is clearly behind us, allow movement unless it is
            # very close (tail overlap / clipping risk).
            if forward_projection < -6.0 and distance > 16.0:
                continue

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
    
    # Safety check: ensure destination stand is not disabled
    if is_disabled_stand_node(destination_stand):
        print(f"Warning: {aircraft_info.get('callsign')} cannot taxi to disabled stand {destination_stand}")
        return
    
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

        if not is_low_visibility_section_entry_allowed(aircraft_info['callsign'], current_node, next_node):
            aircraft_info['waiting_at_hold'] = True
            app.after(adjust_delay(500), move_to_next_node)
            if main_canvas:
                update_stop_bars(main_canvas)
            return
        
        # For landing aircraft, ignore stop bars for the first few nodes after runway exit
        # This allows them to taxi straight through the exit point stop bar
        should_ignore_stopbar = aircraft_info.get('ignore_stop_bars', False) and route_idx < 3
        
        # Check if we're about to pass through a stop-bar node
        # Only check if we're not ignoring stop bars
        if is_stop_bar_node(current_node) and not should_ignore_stopbar:
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
            speed=get_current_taxi_speed(),
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
            avg_speed = get_current_taxi_speed()
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
                move_dist = min(get_current_taxi_speed(), remaining)
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
            point_spacing = max(get_current_taxi_speed(), 0.01)

            if 'heading' not in aircraft_info:
                if steps > 1:
                    x0, y0 = segment_points[0]
                    x1, y1 = segment_points[min(lookahead_points, steps - 1)]
                    aircraft_info['heading'] = math.degrees(math.atan2(y1 - y0, x1 - x0))
                else:
                    aircraft_info['heading'] = math.degrees(math.atan2(end_y - start_y, end_x - start_x))

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

            aircraft_info['arrival_taxi_last_sim_time'] = get_simulation_time()

            def animate_segment(cursor=0.0):
                if cursor >= max(steps - 1, 0):
                    # Reached next node, move to the next segment
                    aircraft_info['route_index'] = route_idx + 1
                    aircraft_info['node'] = next_node
                    aircraft_info['position'] = segment_points[-1]
                    move_to_next_node()
                    return

                curr_x, curr_y = _point_at_cursor(cursor)

                if steps > 1:
                    look_cursor = min(cursor + lookahead_points, max(steps - 1, 0))
                    look_x, look_y = _point_at_cursor(look_cursor)
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
                    aircraft_info['arrival_taxi_last_sim_time'] = get_simulation_time()
                    app.after(adjust_delay(200), animate_segment, cursor)
                    return

                canvas.coords(aircraft_info['triangle_id'], *rotated_points)
                canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)

                aircraft_info['position'] = (curr_x, curr_y)
                history.append((curr_x, curr_y))
                if len(history) > 400:
                    del history[:-300]

                now_sim = get_simulation_time()
                last_sim = aircraft_info.get('arrival_taxi_last_sim_time', now_sim)
                sim_dt = max(0.0, now_sim - last_sim)
                aircraft_info['arrival_taxi_last_sim_time'] = now_sim

                cursor_step = max(0.05, (get_current_taxi_speed() * sim_dt) / point_spacing)

                # Schedule next step
                app.after(adjust_delay(50), animate_segment, cursor + cursor_step)
        
        animate_segment()
    
    move_to_next_node()

# ==============================
# HOME SCREEN DISPLAY
def build_home_screen():
    """Build the ATC home screen UI matching the provided PNG layout."""
    global main_canvas, runtime_clock_var, sim_clock_var, pushback_method_var, runway_var  # Make canvas, clocks, and pushback mode accessible globally
    
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

        def find_clicked_stand_node(click_x, click_y, threshold=18):
            nearest_stand = None
            nearest_distance = None
            for node_name, (sx, sy) in nodes.items():
                if not (node_name.startswith("STAND") and node_name[-1] in {"A", "a"}):
                    continue
                dx = click_x - sx
                dy = click_y - sy
                dist_sq = dx * dx + dy * dy
                if dist_sq <= threshold * threshold and (nearest_distance is None or dist_sq < nearest_distance):
                    nearest_distance = dist_sq
                    nearest_stand = node_name
            return nearest_stand

        def trigger_manual_pushback_for_aircraft(aircraft_info):
            callsign = aircraft_info.get('callsign')
            if not callsign or callsign not in active_aircraft:
                return False
            stand_node = aircraft_info.get('node')
            if not stand_node or not (stand_node.startswith("STAND") and stand_node[-1] in {"A", "a"}):
                return False
            turnaround_job_id = aircraft_info.pop('turnaround_job_id', None)
            if turnaround_job_id:
                try:
                    app.after_cancel(turnaround_job_id)
                except Exception:
                    pass
            pushback_wait_job_id = aircraft_info.pop('pushback_wait_job_id', None)
            if pushback_wait_job_id:
                try:
                    app.after_cancel(pushback_wait_job_id)
                except Exception:
                    pass
            manual_pushback_waiting.pop(callsign, None)
            start_departure_from_stand(aircraft_info, stand_node, manual_authorized=True)
            return True

        def on_canvas_click(event):
            if pushback_method_var is None or pushback_method_var.get() != "manual":
                return
            if not main_canvas:
                return

            click_x, click_y = event.x, event.y
            overlapping = main_canvas.find_overlapping(click_x - 20, click_y - 20, click_x + 20, click_y + 20)

            # 1) Try direct click on aircraft triangle/label first.
            for callsign, ac in list(active_aircraft.items()):
                triangle_id = ac.get('triangle_id')
                label_id = ac.get('label_id')
                if (triangle_id and triangle_id in overlapping) or (label_id and label_id in overlapping):
                    if trigger_manual_pushback_for_aircraft(ac):
                        return

            # 2) If direct hit misses, pick nearest parked stand aircraft within a click radius.
            nearest_aircraft = None
            nearest_dist_sq = None
            nearest_radius = 45
            for callsign, ac in list(active_aircraft.items()):
                stand_node = ac.get('node')
                if not stand_node or not (stand_node.startswith("STAND") and stand_node[-1] in {"A", "a"}):
                    continue
                pos = ac.get('position')
                if not pos:
                    continue
                dx = click_x - pos[0]
                dy = click_y - pos[1]
                dist_sq = dx * dx + dy * dy
                if dist_sq <= nearest_radius * nearest_radius and (nearest_dist_sq is None or dist_sq < nearest_dist_sq):
                    nearest_dist_sq = dist_sq
                    nearest_aircraft = ac
            if nearest_aircraft and trigger_manual_pushback_for_aircraft(nearest_aircraft):
                return

            # 3) Final fallback: stand-click pushback.
            clicked_stand = find_clicked_stand_node(click_x, click_y)
            if not clicked_stand:
                return
            for callsign, ac in list(active_aircraft.items()):
                if ac.get('node') == clicked_stand:
                    trigger_manual_pushback_for_aircraft(ac)
                    return
        
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

        # Allow manual pushback control by clicking aircraft or occupied stand.
        main_canvas.bind("<Button-1>", on_canvas_click)
        
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
    ops_var = tk.StringVar(value=current_operations_mode)
    ctk.CTkRadioButton(
        ops_frame,
        text="Normal Ops",
        variable=ops_var,
        value="Normal Ops",
        command=lambda: set_operations_mode(ops_var.get()),
    ).pack(anchor="w", pady=5)
    ctk.CTkRadioButton(
        ops_frame,
        text="Low Visibility Ops",
        variable=ops_var,
        value="Low Visibility Ops",
        command=lambda: set_operations_mode(ops_var.get()),
    ).pack(anchor="w", pady=5)

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
    
    ctk.CTkCheckBox(
        lvp_frame,
        text="Stop bars",
        variable=lvp_StopBar_sep_var,
        command=lambda: set_template_s_node_stop_bars_enabled(lvp_StopBar_sep_var.get()),
    ).pack(anchor="w", pady=5)
    ctk.CTkCheckBox(lvp_frame, text="Reduced separation", variable=lvp_reduced_sep_var).pack(anchor="w", pady=5)
    ctk.CTkCheckBox(lvp_frame, text="Adaptive sequencing", variable=lvp_adaptive_seq_var).pack(anchor="w", pady=5)
    
    # Toggle Graph Button
    ctk.CTkButton(lvp_frame, text="Toggle Nodes/Edges", command=toggle_graph_visibility).pack(anchor="w", pady=(10, 5))

    # Pushback method selector
    pushback_frame = ctk.CTkFrame(controls_main)
    pushback_frame.pack(side="left", padx=20, pady=10)

    ctk.CTkLabel(pushback_frame, text="Pushback method", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 10))
    pushback_method_var = tk.StringVar(value="automatic")
    ctk.CTkRadioButton(pushback_frame, text="Manual", variable=pushback_method_var, value="manual").pack(anchor="w", pady=5)
    ctk.CTkRadioButton(pushback_frame, text="Automatic", variable=pushback_method_var, value="automatic").pack(anchor="w", pady=5)

    # Center: Movements per hour / Delays in columnar format (matches status board style)
    data_frame = ctk.CTkFrame(controls_main)
    data_frame.pack(side="left", padx=20, pady=0, fill="both", expand=True)

    # Metric variables - edit these values to update the display
    movements_per_hour_var = tk.StringVar(value="45")
    delays_var = tk.StringVar(value="15")
    avg_taxi_time_var = tk.StringVar(value="15min")
    runway_util_var = tk.StringVar(value="80secs")
    runtime_clock_var = tk.StringVar(value="00:00:00")
    sim_clock_var = tk.StringVar(value="09:00:00")

    metrics = [
        #("Movements Per Hour", movements_per_hour_var),
        #("Delays", delays_var),
        #("Average Taxi Time", avg_taxi_time_var),
        #("Runway Utilisation", runway_util_var),
        ("Runtime Clock", runtime_clock_var),
        ("Sim Clock", sim_clock_var),
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
        manual_pushback_waiting.clear()
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

    def start_departure_from_stand(aircraft_info, stand_node, manual_authorized=False):
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
        # Runway 09 departures taxi toward E1/E2 (west/left), runway 27 departures taxi toward A1/A2 (east/right)
        final_direction = "left" if runway == "09" else "right"
        if runway == "27":
            runway_target = "A2_HOLD" if current_operations_mode == "Low Visibility Ops" else "A1_HOLD"
        else:
            runway_target = "E2_HOLD" if current_operations_mode == "Low Visibility Ops" else "E1_HOLD"

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

            if pushback_method_var is not None and pushback_method_var.get() == "manual" and not manual_authorized:
                aircraft_info['waiting_for_pushback_clearance'] = True
                manual_pushback_waiting[callsign] = {
                    'aircraft_info': aircraft_info,
                    'stand_node': stand_node,
                }
                return

            now_sim = get_simulation_time()
            if not manual_authorized and now_sim < next_pushback_release_time:
                aircraft_info['waiting_for_pushback_clearance'] = True
                aircraft_info['pushback_wait_job_id'] = app.after(adjust_delay(1000), attempt_pushback)
                return

            if not manual_authorized and not is_pushback_path_clear(callsign, stand_node, stand_pushback_node):
                aircraft_info['waiting_for_pushback_clearance'] = True
                aircraft_info['pushback_wait_job_id'] = app.after(adjust_delay(1000), attempt_pushback)
                return

            aircraft_info['waiting_for_pushback_clearance'] = False
            aircraft_info.pop('pushback_wait_job_id', None)
            manual_pushback_waiting.pop(callsign, None)

            # Once this pushback starts, reserve the next pushback release at a random
            # interval between 30 seconds and 3 minutes.
            if not manual_authorized:
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
        """Seed parked departures by selected traffic flow rate and stagger pushbacks."""
        if not main_canvas:
            return
        import random
        available_stands = find_available_stands()
        if not available_stands:
            return

        rate = tfr_var.get()
        if rate == "High":
            min_seed, max_seed = 13, 16
        elif rate == "Medium":
            min_seed, max_seed = 7, 12
        else:
            min_seed, max_seed = 3, 7

        seed_count = min(len(available_stands), random.randint(min_seed, max_seed))
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

            segment_duration_sim_seconds = max(0.001, INBOUND_RADAR_SEGMENT_DURATION_SECONDS)
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

            def move_segment(segment_idx=0, segment_start_sim_time=None):
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
                    append_movement_log(callsign, "landing")
                    # Move to Runway status immediately to prevent departures
                    move_aircraft_status(callsign, 'Runway')
                    app.after(adjust_delay(500), lambda: landing_aircraft(main_canvas, aircraft_info, runway_exit))
                    return

                seg_start = nodes[path_nodes[segment_idx]]
                seg_end = nodes[path_nodes[segment_idx + 1]]
                dx = seg_end[0] - seg_start[0]
                dy = seg_end[1] - seg_start[1]

                if segment_start_sim_time is None:
                    segment_start_sim_time = get_simulation_time()
                elapsed_sim = max(0.0, get_simulation_time() - segment_start_sim_time)
                t = min(1.0, elapsed_sim / segment_duration_sim_seconds)
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

                if t >= 1.0:
                    active_aircraft[callsign]['node'] = path_nodes[segment_idx + 1]
                    app.after(INBOUND_RADAR_UPDATE_INTERVAL_MS, lambda: move_segment(segment_idx + 1, None))
                    return

                app.after(INBOUND_RADAR_UPDATE_INTERVAL_MS, lambda: move_segment(segment_idx, segment_start_sim_time))

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

        if can_arrive and not arrival_spawning_paused:
            spawn_landing_aircraft()

        delay_ms = get_activity_delay_ms()
        activity_job_id = app.after(adjust_delay(delay_ms), schedule_next_activity)

    def start_activity():
        global simulation_running, activity_job_id, simulation_time_seconds, last_sim_real_time, next_departure_release_time, next_pushback_release_time, arrival_spawning_paused
        if simulation_running:
            return
        clear_existing_aircraft()
        runway_in_use = runway_var.get()
        initialize_low_visibility_blocks(runway_in_use)  # Lock in runway before starting
        simulation_time_seconds = 0.0
        last_sim_real_time = time.perf_counter()
        next_departure_release_time = 0.0
        next_pushback_release_time = 0.0
        arrival_spawning_paused = False
        simulation_running = True
        update_simulation_clock_display(0.0)
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

    arrival_pause_btn_ref = []

    def toggle_arrival_pause():
        new_state = not arrival_spawning_paused
        set_arrival_spawning_paused(new_state)
        arrival_pause_btn_ref[0].configure(text="Resume Arrivals" if new_state else "Pause Arrivals")

    arrival_pause_btn = ctk.CTkButton(
        button_panel,
        text="Pause Arrivals",
        font=("Arial", 12, "bold"),
        fg_color="#f39c12",
        hover_color="#d68910",
        height=40,
        width=150,
        command=toggle_arrival_pause
    )
    arrival_pause_btn.pack(pady=(0, 10))
    arrival_pause_btn_ref.append(arrival_pause_btn)

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
        configure_windows_timer_resolution()
        app = ctk.CTk()
        app.title("Airport Control Panel")
        app.geometry(f"{screen_width}x{screen_height}+100+100")
        app.minsize(800, 600)
        app.state("zoomed")

        def on_app_close():
            restore_windows_timer_resolution()
            app.destroy()

        app.protocol("WM_DELETE_WINDOW", on_app_close)
        # Build the home screen and start the GUI
        build_home_screen()
        # Start continuous stop bar updates
        continuous_stop_bar_update()
        try:
            app.mainloop()
        finally:
            restore_windows_timer_resolution()
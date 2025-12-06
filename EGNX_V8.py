from PIL import Image, ImageTk
import warnings
import math
import heapq
from tkinter import ttk
import tkinter as tk
import os

# Screen defaults (used for image sizing and any layout calculations)
screen_width = 1920
screen_height = 1080

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# ==============================
# NODES & EDGES
nodes = {
    "STAND1a": (846, 200), "STAND1b": (846, 215),
    "STAND2a": (892, 200), "STAND2b": (892, 215),
    "STAND3a": (938, 200), "STAND3b": (938, 215),
    "STAND4a": (984, 200), "STAND4b": (984, 215),
    "STAND5a": (1089, 200), "STAND5b": (1089, 215),
    "STAND6a": (1135, 200), "STAND6b": (1135, 215),
    "STAND7a": (1181, 200), "STAND7b": (1181, 215),
    "STAND8a": (1227, 200), "STAND8b": (1227, 215),
    "STAND1N": (846, 241), "STAND2N": (892, 241),
    "STAND3N": (938, 241), "STAND4N": (984, 241),
    "STAND5N": (1089, 241), "STAND6N": (1135, 241),
    "STAND7N": (1181, 241), "STAND8N": (1227, 241),
    "AQ": (790, 152), "NQ": (790, 241),
    "AR": (1282, 152), "NR": (1282, 241),
    "AS": (1036, 152), "NS": (1036, 241),
    "TXY_A1": (1730, 152), "RWY27_A1": (1730, 90),
    "TXY_B1": (1498, 152), "RWY27_B1": (1498, 90),
    "TXY_C1": (675, 152), "RWY09_C1": (675, 90),
    "TXY_D1": (287, 152), "RWY09_D1": (287, 90),
    "TXY_E1": (190, 152), "RWY09_E1": (190, 90),
}

edges = {
    "RWY27_A1": ["TXY_A1","RWY27_B1"],
    "RWY27_B1": ["TXY_B1","RWY09_C1"],
    "RWY09_C1": ["TXY_C1","RWY09_D1"],
    "RWY09_D1": ["RWY09_E1","TXY_D1"],
    "TXY_E1": ["RWY09_E1","TXY_D1"],
    "TXY_C1": ["TXY_D1","AQ"],
    "AQ": ["NQ","AS"],
    "NS": ["AS","NQ","NR","STAND1N","STAND2N","STAND3N","STAND4N","STAND5N","STAND6N","STAND7N","STAND8N"],
    "NQ": ["STAND1N","STAND2N","STAND3N","STAND4N"],
    "NR": ["STAND5N","STAND6N","STAND7N","STAND8N"],
    "AR": ["AS","NR","TXY_B1"],
    "TXY_B1": ["TXY_A1"],
    "STAND1b": ["STAND1N","STAND1a"],
    "STAND2b": ["STAND2N","STAND2a"],
    "STAND3b": ["STAND3N","STAND3a"],
    "STAND4b": ["STAND4N","STAND4a"],
    "STAND5b": ["STAND5N","STAND5a"],
    "STAND6b": ["STAND6N","STAND6a"],
    "STAND7b": ["STAND7N","STAND7a"],
    "STAND8b": ["STAND8N","STAND8a"],
}

# Build bidirectional edges
for node, neighbors in list(edges.items()):
    for neighbor in neighbors:
        if neighbor not in edges:
            edges[neighbor] = []
        if node not in edges[neighbor]:
            edges[neighbor].append(node)

# ==============================
# DIJKSTRA PATHFINDING
def dijkstra(start, goal):
    """Return shortest path as list of node names from start to goal using Euclidean edge costs."""
    if start not in nodes or goal not in nodes:
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
            edge_cost = math.dist(nodes[node], nodes[nxt])
            new_cost = cur_cost + edge_cost
            if new_cost < dist.get(nxt, float('inf')):
                dist[nxt] = new_cost
                prev[nxt] = node
                heapq.heappush(pq, (new_cost, nxt))

    return None

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

# ==============================
# CATMULL-ROM SPLINE
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

# ==============================
# HOME SCREEN DISPLAY
def build_home_screen():
    """Build the ATC home screen UI matching the provided PNG layout."""
    
    # Clear any existing widgets
    for widget in app.winfo_children():
        widget.destroy()

    # ===== TOP SECTION: Airport Map in green banner =====
    try:
        map_path = os.path.join(script_dir, "EGNX_Map_Zoom.tif")
        top_img = Image.open(map_path)

        # Resize to fit screen width while maintaining aspect ratio
        # Image is 2234x464, so aspect ratio is ~4.81:1
        # Target height of ~300px, calculate width to maintain aspect ratio
        target_height = 310
        original_width, original_height = top_img.size
        aspect_ratio = original_width / original_height  # width:height ratio
        new_width = int(target_height * aspect_ratio)
        top_img = top_img.resize((new_width, target_height), Image.Resampling.LANCZOS)
        # Convert PIL Image to CTkImage so customtkinter can handle HighDPI scaling
        try:
            top_ctk_img = ctk.CTkImage(light_image=top_img, size=(new_width, target_height))
            img_label = ctk.CTkLabel(app, image=top_ctk_img, text="")
            img_label.image = top_ctk_img
        except Exception:
            top_photo = ImageTk.PhotoImage(top_img)
            img_label = ctk.CTkLabel(app, image=top_photo, text="")
            img_label.image = top_photo
        img_label.pack(fill="x", expand=False, padx=0, pady=0)
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

    # Middle: LVP Improvement checkboxes
    lvp_frame = ctk.CTkFrame(controls_main)
    lvp_frame.pack(side="left", padx=20, pady=10)
    
    ctk.CTkLabel(lvp_frame, text="LVP Improvement", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 10))
    lvp_stop_bars_var = tk.BooleanVar(value=False)
    lvp_reduced_sep_var = tk.BooleanVar(value=False)
    lvp_adaptive_seq_var = tk.BooleanVar(value=False)
    ctk.CTkCheckBox(lvp_frame, text="Stop bars", variable=lvp_stop_bars_var).pack(anchor="w", pady=5)
    ctk.CTkCheckBox(lvp_frame, text="Reduced separation", variable=lvp_reduced_sep_var).pack(anchor="w", pady=5)
    ctk.CTkCheckBox(lvp_frame, text="Adaptive sequencing", variable=lvp_adaptive_seq_var).pack(anchor="w", pady=5)

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

    # Right: Run Simulation button
    run_btn = ctk.CTkButton(
        controls_main, 
        text="▶ Run Simulation", 
        font=("Arial", 14, "bold"),
        fg_color="#1e88e5",
        hover_color="#1565c0",
        height=80,
        width=200,
        command=lambda: None
    )
    run_btn.pack(side="right", padx=20, pady=10)

    # ===== BOTTOM SECTION: Status Board =====
    status_frame = ctk.CTkFrame(app)
    status_frame.pack(fill="both", expand=True, padx=10, pady=10)

    # Header row with 5 columns
    columns = ["Departures", "Taxiing", "Runway", "Airborne", "Arrivals"]
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
        app.mainloop()
import customtkinter as ctk
from PIL import Image, ImageTk
import warnings
import math
import heapq
from tkinter import ttk
import tkinter as tk
import os

# ==============================
# MAIN WINDOW
app = ctk.CTk()
app.title("Airport Control Panel")
screen_width = 1920
screen_height = 1080
app.geometry(f"{screen_width}x{screen_height}+100+100")
app.minsize(800, 600)

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# ==============================
# BACKGROUND MAP
bg_photo = None
try:
    bg_image = Image.open("EGNX_Map_zoom.tif")
    bg_image = bg_image.resize((screen_width, screen_height), Image.Resampling.LANCZOS)
    bg_photo = ImageTk.PhotoImage(bg_image)
except FileNotFoundError:
    print("Warning: EGNX_Map_zoom.tif not found — continuing with placeholder background")
except Exception as e:
    print(f"Warning: failed to load map image: {e}")

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
        
        # Calculate height to maintain aspect ratio when width = screen_width
        original_width, original_height = top_img.size
        aspect_ratio = original_height / original_width
        new_height = int(screen_width * aspect_ratio)
        
        top_img = top_img.resize((screen_width, new_height), Image.Resampling.LANCZOS)
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

    # Center: Movements per hour / Delays
    data_frame = ctk.CTkFrame(controls_main, fg_color="#cce5ff")
    data_frame.pack(side="left", padx=20, pady=10, fill="both", expand=True)
    data_frame.pack_propagate(False)
    data_frame.configure(height=120, width=300)
    
    header_frame = ctk.CTkFrame(data_frame, fg_color="#99ccff")
    header_frame.pack(fill="x", side="top")
    ctk.CTkLabel(header_frame, text="Movements Per Hour", font=("Arial", 12, "bold")).pack(side="left", padx=10, pady=8, fill="x", expand=True)
    ctk.CTkLabel(header_frame, text="Delays", font=("Arial", 12, "bold")).pack(side="left", padx=10, pady=8, fill="x", expand=True)
    
    content_frame = ctk.CTkFrame(data_frame, fg_color="#e6f5ff")
    content_frame.pack(fill="both", expand=True, side="bottom")

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
        col_frame = ctk.CTkFrame(status_frame)
        col_frame.pack(side="left", fill="both", expand=True, padx=2, pady=2)
        
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


# Build the home screen on startup
build_home_screen()

# ==============================
# MAIN LOOP
app.mainloop()
import customtkinter as ctk
from PIL import Image, ImageTk
import warnings
import math
import heapq
from tkinter import ttk
import tkinter as tk

# ==============================
# MAIN WINDOW
app = ctk.CTk()
app.title("Airport Control Panel")
screen_width = 1920
screen_height = 1080
app.geometry(f"{screen_width}x{screen_height}+100+100")
app.minsize(800, 600)

# ==============================
# BACKGROUND MAP
bg_photo = None
try:
    bg_image = Image.open("EGNX_Map.tif")
    bg_image = bg_image.resize((screen_width, screen_height), Image.Resampling.LANCZOS)
    bg_photo = ImageTk.PhotoImage(bg_image)
    # warn if not CTkImage (CTkImage supports HighDPI scaling)
    try:
        if not isinstance(bg_photo, ctk.CTkImage):
            warnings.warn(f"EGNX_V7 Warning: Given image is not CTkImage but {type(bg_photo)}. Image cannot be scaled on HighDPI displays; use CTkImage instead.", stacklevel=2)
    except Exception:
        warnings.warn(f"EGNX_V7 Warning: Given image is of type {type(bg_photo)}; expected CTkImage. Use CTkImage for HighDPI scaling.", stacklevel=2)
except FileNotFoundError:
    print("Warning: EGNX_Map.tif not found — continuing with placeholder background")
except Exception as e:
    print(f"Warning: failed to load map image: {e}")

# Create canvas regardless; if image loaded we'll place it, otherwise keep blank
canvas = ctk.CTkCanvas(app, width=screen_width, height=screen_height)
canvas.pack(fill="both", expand=True)
if bg_photo:
    canvas_bg = canvas.create_image(0, -100, anchor="nw", image=bg_photo)

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
# GRAPH DRAW
def draw_graph():
    for node, neighbors in edges.items():
        x1, y1 = nodes[node]
        for neighbor in neighbors:
            x2, y2 = nodes[neighbor]
            canvas.create_line(x1, y1, x2, y2, fill="yellow", width=2, dash=(4,2))
    for name, (x, y) in nodes.items():
        canvas.create_oval(x-3, y-3, x+3, y+3, fill="red")
        canvas.create_text(x, y-10, text=name, fill="white", font=("Arial", 7, "bold"))

draw_graph()

# ==============================
# DIJKSTRA PATHFINDING
def dijkstra(start, goal):
    """Return shortest path as list of node names from start to goal using Euclidean edge costs.

    Both start and goal must be node keys (strings) present in the `nodes` dict.
    Returns None if no path found or inputs invalid.
    """
    if start not in nodes or goal not in nodes:
        return None

    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]

    while pq:
        cur_cost, node = heapq.heappop(pq)
        if node == goal:
            # reconstruct path
            path = []
            while node in prev:
                path.append(node)
                node = prev[node]
            path.append(start)
            return list(reversed(path))

        if cur_cost > dist.get(node, float('inf')):
            continue

        for nxt in edges.get(node, []):
            # ensure node coordinates exist
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
        # logical node name where aircraft currently is (string)
        self.callsign = callsign
        self.node = node
        # current coordinates (x,y) for drawing and movement
        self.coords = nodes.get(node) if node in nodes else (0, 0)
        self.triangle_id = None
        self.label_id = None
        self.route = []
        # spline_points: list of (x,y) tuples
        self.spline_points = []
        # mapping from spline index -> route index (index into self.route)
        self.spline_route_idx_map = []
        self.dist_along_path = 0.0
        self.waiting_for_stopbar = False

active_aircraft = {}
aircraft_rows = {}
stop_bars = {}
stop_bar_draw_ids = {}

# ==============================
# DRAW AIRCRAFT
def draw_aircraft(ac):
    """Draw an aircraft at its current coords and store canvas ids on the object."""
    x, y = ac.coords
    size = 12
    ac.triangle_id = canvas.create_polygon(
        x, y-size,
        x-size, y+size,
        x+size, y+size,
        fill="blue"
    )
    ac.label_id = canvas.create_text(x, y-size-10, text=ac.callsign, fill="white", font=("Arial", 10, "bold"))

# ==============================
# CATMULL-ROM SPLINE
def catmull_rom_spline(P0,P1,P2,P3,n_points=20):
    points = []
    for i in range(n_points):
        t = i/n_points
        t2 = t*t
        t3 = t2*t
        x = 0.5*((2*P1[0]) + (-P0[0]+P2[0])*t + (2*P0[0]-5*P1[0]+4*P2[0]-P3[0])*t2 + (-P0[0]+3*P1[0]-3*P2[0]+P3[0])*t3)
        y = 0.5*((2*P1[1]) + (-P0[1]+P2[1])*t + (2*P0[1]-5*P1[1]+4*P2[1]-P3[1])*t2 + (-P0[1]+3*P1[1]-3*P2[1]+P3[1])*t3)
        points.append((x,y))
    return points

def build_spline_path(route, points_per_segment=20):
    """Build a Catmull-Rom spline for the given route (list of node names).

    Returns (spline_points, spline_route_idx_map) where spline_route_idx_map[i]
    is the index j such that spline_points[i] corresponds to motion from route[j]
    towards route[j+1].
    """
    spline_points = []
    spline_route_idx_map = []
    if not route or len(route) == 0:
        return [], []
    if len(route) == 1:
        # single point
        pt = nodes.get(route[0], (0, 0))
        return [pt], [0]

    n = len(route)
    for i in range(n-1):
        P0 = nodes[route[i-1]] if i>0 else nodes[route[i]]
        P1 = nodes[route[i]]
        P2 = nodes[route[i+1]]
        P3 = nodes[route[i+2]] if i+2<n else nodes[route[i+1]]
        seg = catmull_rom_spline(P0, P1, P2, P3, n_points=points_per_segment)
        spline_points.extend(seg)
        # for each point in seg, map to route index i
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
# MOVE AIRCRAFT ALONG SPLINE WITH CONTROLLED SPEED
def move_aircraft(ac, speed=2):
    if not ac.spline_points or ac.dist_along_path >= len(ac.spline_points)-1:
        # arrived or nothing to follow
        if ac.route:
            ac.node = ac.route[-1]
            ac.coords = nodes.get(ac.node, ac.coords)
        row_id = aircraft_rows.get(ac.callsign)
        if row_id:
            aircraft_table.item(row_id, values=(ac.callsign, "", "", "ARRIVED", ac.node))
        return

    idx = int(ac.dist_along_path)
    next_idx = min(idx+1, len(ac.spline_points)-1)
    curr_x, curr_y = ac.spline_points[idx]
    next_x, next_y = ac.spline_points[next_idx]

    # stop bar check using precomputed spline->route map
    route_idx = None
    if ac.spline_route_idx_map and idx < len(ac.spline_route_idx_map):
        route_idx = ac.spline_route_idx_map[idx]
    if route_idx is not None and route_idx + 1 < len(ac.route):
        next_node = ac.route[route_idx + 1]
        if stop_bars.get(next_node):
            ac.waiting_for_stopbar = True
            row_id = aircraft_rows.get(ac.callsign)
            if row_id:
                aircraft_table.item(row_id, values=("", ac.callsign + " (HOLD)", "", "", ac.node))
            return
        else:
            ac.waiting_for_stopbar = False

    # rotation
    angle = math.atan2(next_y-curr_y, next_x-curr_x)
    size = 12
    points = [
        (curr_x + size*math.cos(angle), curr_y + size*math.sin(angle)),
        (curr_x - size*math.cos(angle+0.5), curr_y - size*math.sin(angle+0.5)),
        (curr_x - size*math.cos(angle-0.5), curr_y - size*math.sin(angle-0.5)),
    ]
    coords = [c for p in points for c in p]
    canvas.coords(ac.triangle_id, *coords)
    canvas.coords(ac.label_id, curr_x, curr_y-size-10)

    ac.coords = (curr_x, curr_y)
    row_id = aircraft_rows.get(ac.callsign)
    if row_id:
        aircraft_table.item(row_id, values=(ac.callsign, "", "", "", f"{ac.coords}"))

    # move along spline by distance
    ac.dist_along_path += speed
    app.after(30, move_aircraft, ac, speed)

# ==============================
# TABLE
table_frame = ctk.CTkFrame(app)
table_frame.place(relx=0.5, rely=0.35, anchor="n")
aircraft_table = ttk.Treeview(table_frame, columns=("stand","taxi","runway","inbound","pos"), show="headings", height=6)
for col in ("stand","taxi","runway","inbound","pos"):
    aircraft_table.heading(col, text=col.capitalize())
    aircraft_table.column(col, width=100)
aircraft_table.pack()

# ==============================
# ADD AIRCRAFT
def open_add_aircraft():
    popup = ctk.CTkToplevel(app)
    popup.title("Add Aircraft")
    popup.geometry("300x200")
    popup.grab_set()
    ctk.CTkLabel(popup, text="Select Stand:").pack(pady=5)
    stand_menu = ctk.CTkOptionMenu(popup, values=[s for s in nodes if s.endswith("a")])
    stand_menu.pack(pady=5)
    ctk.CTkLabel(popup, text="Enter Callsign:").pack(pady=5)
    callsign_entry = ctk.CTkEntry(popup)
    callsign_entry.pack(pady=5)
    callsign_entry.focus_set()

    def confirm(event=None):
        stand = stand_menu.get()
        callsign = callsign_entry.get().strip().upper()
        if not callsign: return
        ac = Aircraft(callsign, stand)
        # ensure coords are set from stand
        ac.coords = nodes.get(stand, ac.coords)
        draw_aircraft(ac)
        row_id = aircraft_table.insert("", "end", values=(callsign, "", "", "", stand))
        active_aircraft[callsign] = ac
        aircraft_rows[callsign] = row_id
        popup.destroy()

    confirm_btn = ctk.CTkButton(popup, text="Add", command=confirm)
    confirm_btn.pack(pady=10)
    callsign_entry.bind("<Return>", confirm)

add_btn = ctk.CTkButton(app, text="Add Aircraft", command=open_add_aircraft)
add_btn.place(relx=0.5, rely=0.25, anchor="center")

# ==============================
# TAXI BUTTON
controls_frame = ctk.CTkFrame(app)
controls_frame.place(relx=0.5, rely=0.42, anchor="n")
runway_selector = ctk.CTkOptionMenu(
    controls_frame,
    values=["RWY27_A1","RWY27_B1","RWY09_C1","RWY09_D1","RWY09_E1"]
)
runway_selector.pack(side="left", padx=10, pady=10)

def taxi_selected_aircraft():
    sel = aircraft_table.selection()
    if not sel: return
    row_id = sel[0]
    values = aircraft_table.item(row_id, "values")
    callsign = next((v for v in values if v and v != values[-1]), None)
    if not callsign: return
    ac = active_aircraft[callsign]
    destination = runway_selector.get()
    route = dijkstra(ac.node, destination)
    if route and len(route) >= 1:
        ac.route = route
        spline_points, spline_map = build_spline_path(route, points_per_segment=15)
        ac.spline_points = spline_points
        ac.spline_route_idx_map = spline_map
        ac.dist_along_path = 0.0
        # set starting coords to first spline point
        if spline_points:
            ac.coords = spline_points[0]
            canvas.coords(ac.triangle_id, *([c for p in [
                (spline_points[0][0] + 12*math.cos(0), spline_points[0][1] + 12*math.sin(0)),
                (spline_points[0][0] - 12*math.cos(0.5), spline_points[0][1] - 12*math.sin(0.5)),
                (spline_points[0][0] - 12*math.cos(-0.5), spline_points[0][1] - 12*math.sin(-0.5)),
            ] for c in p]) )
        move_aircraft(ac, speed=2)

taxi_button = ctk.CTkButton(controls_frame, text="Taxi to Runway", command=taxi_selected_aircraft)
taxi_button.pack(side="left", padx=10, pady=10)

# ==============================
# STOP-BAR
stopbar_frame = ctk.CTkFrame(app)
stopbar_frame.place(relx=0.85, rely=0.15, anchor="n")
placing_stopbars = {"on": False}

def enter_stopbar_mode():
    placing_stopbars["on"] = not placing_stopbars["on"]
    if placing_stopbars["on"]:
        place_btn.configure(text="Stop-Bar Mode: ON (click map)")
        canvas.bind("<Button-1>", stopbar_place_click)
    else:
        place_btn.configure(text="Stop-Bar Mode: OFF")
        canvas.unbind("<Button-1>")

def stopbar_place_click(event):
    node = nearest_node_to(event.x, event.y, max_dist=50)
    if node:
        toggle_stop_bar(node)

def draw_stop_bar_at(node):
    x, y = nodes[node]
    length = 24
    draw_id = canvas.create_line(x - length//2, y - 6, x + length//2, y - 6, width=6, fill="red")
    stop_bar_draw_ids[node] = draw_id

def remove_stop_bar_draw(node):
    cid = stop_bar_draw_ids.get(node)
    if cid:
        canvas.delete(cid)
        stop_bar_draw_ids.pop(node, None)

def toggle_stop_bar(node):
    if stop_bars.get(node):
        stop_bars[node] = False
        remove_stop_bar_draw(node)
    else:
        stop_bars[node] = True
        draw_stop_bar_at(node)
    refresh_stopbar_list()

def clear_stop_bar(node):
    if stop_bars.get(node):
        stop_bars[node] = False
        remove_stop_bar_draw(node)
        refresh_stopbar_list()
        resume_aircraft_waiting_at(node)

def refresh_stopbar_list():
    for i in stopbar_list.get_children():
        stopbar_list.delete(i)
    for node, active in sorted(stop_bars.items()):
        if active:
            stopbar_list.insert("", "end", values=(node, "ON"))

def clear_selected_stopbar():
    sel = stopbar_list.selection()
    if not sel: return
    node = stopbar_list.item(sel[0], "values")[0]
    clear_stop_bar(node)

def resume_aircraft_waiting_at(node):
    for callsign, ac in active_aircraft.items():
        if ac.waiting_for_stopbar and ac.route:
            ac.waiting_for_stopbar = False
            move_aircraft(ac, speed=2)

place_btn = ctk.CTkButton(stopbar_frame, text="Stop-Bar Mode: OFF", command=enter_stopbar_mode)
place_btn.pack(pady=6)
stopbar_list = ttk.Treeview(stopbar_frame, columns=("node","status"), show="headings", height=6)
stopbar_list.heading("node", text="Node")
stopbar_list.heading("status", text="Status")
stopbar_list.column("node", width=100)
stopbar_list.column("status", width=80)
stopbar_list.pack(pady=6)

for n in nodes:
    stop_bars[n] = False


def show_home_screen():
    """Replace the interactive canvas with a static 'home screen' layout inspired by the provided PNG.

    This function intentionally leaves interactive behaviour disabled — it only builds the visual layout.
    """
    # remove the existing canvas (we'll build the home-screen with frames)
    try:
        canvas.destroy()
    except Exception:
        pass

    # Top green banner with airport map image
    top_frame = ctk.CTkFrame(app, fg_color="#00c853", width=int(screen_width*0.96), height=260)
    top_frame.place(relx=0.5, rely=0.03, anchor="n")
    try:
        top_img = Image.open("EGNX_Map.tif")
        # scale image to fit inside top_frame while keeping aspect
        top_img = top_img.resize((1400, 200), Image.Resampling.LANCZOS)
        top_photo = ImageTk.PhotoImage(top_img)
        # warn if not CTkImage (CTkImage supports HighDPI scaling)
        try:
            if not isinstance(top_photo, ctk.CTkImage):
                warnings.warn(f"EGNX_V7 Warning: Given image is not CTkImage but {type(top_photo)}. Image cannot be scaled on HighDPI displays; use CTkImage instead.", stacklevel=2)
        except Exception:
            warnings.warn(f"EGNX_V7 Warning: Given image is of type {type(top_photo)}; expected CTkImage. Use CTkImage for HighDPI scaling.", stacklevel=2)
        img_label = ctk.CTkLabel(top_frame, image=top_photo, text="")
        img_label.image = top_photo
        img_label.place(relx=0.5, rely=0.5, anchor="center")
    except Exception:
        # show placeholder rectangle if image missing
        placeholder = ctk.CTkLabel(top_frame, text="[Map image not available]", font=("Arial", 18, "bold"))
        placeholder.place(relx=0.5, rely=0.5, anchor="center")

    # Left: Operations
    ops_frame = ctk.CTkFrame(app, width=300, height=180)
    ops_frame.place(relx=0.03, rely=0.32, anchor="nw")
    ctk.CTkLabel(ops_frame, text="Operations", font=("Arial", 16, "bold")).place(x=10, y=6)
    ops_var = tk.StringVar(value="Normal Ops")
    ctk.CTkRadioButton(ops_frame, text="Normal Ops", variable=ops_var, value="Normal Ops").place(x=10, y=40)
    ctk.CTkRadioButton(ops_frame, text="Low Visibility Ops", variable=ops_var, value="Low Visibility Ops").place(x=10, y=75)

    # Middle-left: Traffic Flow Rate
    tfr_frame = ctk.CTkFrame(app, width=300, height=180)
    tfr_frame.place(relx=0.22, rely=0.32, anchor="nw")
    ctk.CTkLabel(tfr_frame, text="Traffic Flow Rate", font=("Arial", 16, "bold")).place(x=10, y=6)
    tfr_var = tk.StringVar(value="Low")
    ctk.CTkRadioButton(tfr_frame, text="Low", variable=tfr_var, value="Low").place(x=10, y=40)
    ctk.CTkRadioButton(tfr_frame, text="Medium", variable=tfr_var, value="Medium").place(x=10, y=75)
    ctk.CTkRadioButton(tfr_frame, text="High", variable=tfr_var, value="High").place(x=10, y=110)

    # Center: Movements per hour / Delays box
    mid_frame = ctk.CTkFrame(app, fg_color="#e6f7ff", width=260, height=140)
    mid_frame.place(relx=0.48, rely=0.32, anchor="n")
    ctk.CTkLabel(mid_frame, text="Movements\nPer Hour", font=("Arial", 14, "bold")).place(relx=0.03, rely=0.06)
    ctk.CTkLabel(mid_frame, text="Delays", font=("Arial", 14, "bold")).place(relx=0.55, rely=0.06)

    # Right: Run Simulation big button
    run_frame = ctk.CTkFrame(app, width=360, height=160)
    run_frame.place(relx=0.82, rely=0.26, anchor="n")
    run_btn = ctk.CTkButton(run_frame, text="Run Simulation", width=240, height=120, fg_color="#1976d2", hover_color="#125ea8", command=lambda: None)
    run_btn.place(relx=0.5, rely=0.5, anchor="center")

    # Bottom: large multi-column status board
    bottom_frame = ctk.CTkFrame(app)
    bottom_frame.place(relx=0.5, rely=0.62, anchor="n", relwidth=0.92, relheight=0.66)

    cols = ["Departures", "Taxiing", "Runway", "Airborne", "Arrivals"]
    for i, col in enumerate(cols):
        hdr = ctk.CTkLabel(bottom_frame, text=col, font=("Arial", 14, "bold"), fg_color="#dff2ff")
        hdr.place(relx=i/5, rely=0.0, relwidth=1/5, relheight=0.08)
        # content area
        content = ctk.CTkFrame(bottom_frame, fg_color="#eef9ff")
        content.place(relx=i/5, rely=0.08, relwidth=1/5, relheight=0.92)


# build the static home screen UI
show_home_screen()

# ==============================
# FULLSCREEN SNAP
def check_snap(event):
    if app.winfo_y() <= 0:
        app.state("zoomed")
    else:
        if app.state() == "zoomed":
            app.state("normal")

app.bind("<Configure>", check_snap)
app.mainloop()

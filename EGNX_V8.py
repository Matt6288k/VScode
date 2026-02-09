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
    "AQ": (771, 158), "NQ": (771, 283),
    "AR": (1320, 158), "NR": (1320, 283),
    "AS": (1045, 158), "NS": (1045, 283),
    "TXY_A1": (1820, 158),"A1_HOLD": (1820, 120), "RWY27_A1": (1820, 70), "RWY09_AirBorne": (1920,70),
    "TXY_B1": (1558, 158),"B1_HOLD": (1558, 120), "RWY27_B1": (1558, 70),
    "TXY_C1": (645, 158),"C1_HOLD": (645, 120), "RWY09_C1": (645, 70),
    "TXY_D1": (214, 158),"D1_HOLD": (214, 120), "RWY09_D1": (214, 70),
    "TXY_E1": (105, 158),"E1_HOLD": (105, 120), "RWY09_E1": (105, 70), "RWY27_AirBorne": (0,70),
}

edges = {
    "RWY27_A1": ["A1_HOLD","RWY27_B1","RWY27_AirBorne"],
    "RWY27_B1": ["B1_HOLD","RWY09_C1"],
    "RWY09_C1": ["C1_HOLD","RWY09_D1"],
    "RWY09_D1": ["RWY09_E1","D1_HOLD"],
    "RWY09_E1": ["RWY09_AirBorne"],
    "E1_HOLD": ["RWY09_E1","TXY_E1"],
    "TXY_D1": ["TXY_E1","D1_HOLD"],
    "TXY_C1": ["TXY_D1","AQ","C1_HOLD"],
    "TXY_B1": ["TXY_A1","B1_HOLD"],
    "TXY_A1": ["AR","A1_HOLD"],
    "AQ": ["NQ","AS"],
    "NS": ["AS","NQ","NR","STAND1N","STAND2N","STAND3N","STAND4N","STAND5N","STAND6N","STAND7N","STAND8N"],
    "NQ": ["STAND1N","STAND2N","STAND3N","STAND4N"],
    "NR": ["STAND5N","STAND6N","STAND7N","STAND8N"],
    "AR": ["AS","NR","TXY_B1"],
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
stop_bar_off_until = {}  # Track when each stop bar should turn back on (time-based)
runway_protected = True  # Track if runway is protected (stop bars on)
simulation_speed = 1.0  # Speed multiplier for simulation (1x, 2x, 5x, 10x)
simulation_speed = 1.0  # Speed multiplier for simulation (1x, 2x, 5x, 10x)
main_canvas = None  # Global reference to the canvas
status_columns = {}  # Global reference to status board columns
aircraft_labels = {}  # Track labels in each status column
graph_element_ids = []  # Store all graph element IDs for hiding/showing
graph_visible = True  # Track whether graph is currently visible
simulation_running = False
activity_job_id = None
schedule_turnaround_cb = None

# ==============================
# SPEED CONTROL HELPER
def adjust_delay(base_delay_ms):
    """Adjust a delay in milliseconds based on simulation speed multiplier."""
    return max(1, int(base_delay_ms / simulation_speed))

# ==============================
# SPEED CONTROL HELPER
def adjust_delay(base_delay_ms):
    """Adjust a delay in milliseconds based on simulation speed multiplier."""
    return max(1, int(base_delay_ms / simulation_speed))

# ==============================
# GRAPH DRAW FUNCTION
def draw_graph(canvas):
    """Draw nodes and edges on the canvas."""
    global graph_element_ids
    
    # Clear existing graph element IDs
    graph_element_ids = []
    
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
        text_id = canvas.create_text(x, y-10, text=name, fill="white", font=("Arial", 7, "bold"))
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
# Format: "HOLD_NAME": {"red": [(x1,y1), (x2,y2), ...], "green": [(x1,y1), (x2,y2), ...]}
STOP_BAR_POSITIONS = {
    "A1_HOLD": {
        "red": [(1810, 120), (1815, 120), (1820, 120), (1825, 120), (1830, 120), (1835, 120)],  # Add 6 red light positions here: [(x1,y1), (x2,y2), (x3,y3), (x4,y4), (x5,y5), (x6,y6)] 
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
    global stop_bar_draw_ids, stop_bar_off_until
    import time
    
    # Clear existing stop bars
    for items in stop_bar_draw_ids.values():
        for item_id in items:
            canvas.delete(item_id)
    stop_bar_draw_ids.clear()
    
    current_time = time.time()
    
    # Check which hold points have DEPARTING aircraft that should turn off the stop bar
    # Landing aircraft (with ignore_stop_bars flag) should NOT affect stop bars
    for callsign, info in active_aircraft.items():
        node = info.get('node')
        # Only turn off stop bar if:
        # 1. Aircraft is at a hold point
        # 2. Runway is clear
        # 3. Aircraft is NOT a landing aircraft (doesn't have ignore_stop_bars flag)
        is_landing_aircraft = info.get('ignore_stop_bars', False)
        
        if node and node.endswith('_HOLD') and is_runway_clear() and not is_landing_aircraft:
            # If this hold's stop bar isn't already off, turn it off now and set timer
            if node not in stop_bar_off_until or current_time >= stop_bar_off_until[node]:
                stop_bar_duration = 1.2 / simulation_speed  # Adjust duration based on simulation speed
                stop_bar_off_until[node] = current_time + stop_bar_duration
    
    for hold_name, positions in STOP_BAR_POSITIONS.items():
        if not positions.get("red"):
            continue  # Skip if no red positions defined
            
        items = []
        
        # Red lights are OFF if we're within the 2-second timer period
        # Red lights turn back ON after 2 seconds
        is_off = hold_name in stop_bar_off_until and current_time < stop_bar_off_until[hold_name]
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
    app.after(100, continuous_stop_bar_update)

# ==============================
# DRAW AIRCRAFT
def draw_aircraft(canvas, x, y, callsign="TEST", direction="north", color="blue"):
    """Draw a blue triangle representing an aircraft at the given position.
    
    The front tip of the triangle is always at position (x, y).
    
    Args:
        direction: "north" (up), "right" for runway 09 (East), "left" for runway 27 (West)
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
def pushback_aircraft(canvas, aircraft_info, target_node, final_direction="right", steps=30, runway_target=None):
    """Animate aircraft pushback from current position to target node.
    
    Aircraft starts facing north and gradually rotates to final_direction near the end.
    After pushback completes, starts taxi to runway if runway_target is provided.
    """
    import math
    
    start_x, start_y = aircraft_info['position']
    end_x, end_y = nodes[target_node]
    
    step_x = (end_x - start_x) / steps
    step_y = (end_y - start_y) / steps
    rotation_start_step = int(steps * 0.7)  # Start rotation at 70% of journey
    rotation_steps = steps - rotation_start_step
    
    # Define angles: north = -90 degrees (pointing up), right = 0 degrees, left = 180 degrees
    start_angle = -90  # North
    if final_direction == "left":
        end_angle = 0  # Turn right (clockwise) to east - 90 degree turn
    else:  # left
        end_angle = -180  # Turn left (counterclockwise) to west - 90 degree turn
    
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
            
            # Start taxi to runway if target provided
            if runway_target:
                app.after(adjust_delay(500), lambda: taxi_aircraft(canvas, aircraft_info, runway_target))
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
            current_angle = start_angle + (end_angle - start_angle) * rotation_progress
        
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
        app.after(adjust_delay(50), animate_step, current_step + 1)
    
    animate_step()

# ==============================
# TAXI AIRCRAFT ALONG PATH
def taxi_aircraft(canvas, aircraft_info, destination_node, speed=5):
    """Animate aircraft taxiing from current node to destination using pathfinding."""
    import math
    
    # Get current node from aircraft info
    current_node = aircraft_info.get('node', 'STAND1N')
    
    # Find path using dijkstra
    route = dijkstra(current_node, destination_node)
    
    if not route:
        print(f"No route found from {current_node} to {destination_node}")
        return
    
    aircraft_info['route'] = route
    aircraft_info['route_index'] = 0
    
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
                    runway_entry = "RWY27_A1"
                else:  # E1_HOLD
                    runway_entry = "RWY09_E1"
                
                # Continue to runway entry point after brief hold
                app.after(adjust_delay(500), lambda: taxi_aircraft(canvas, aircraft_info, runway_entry))
            # Check if we've reached a runway entry point and should start takeoff
            elif destination_node in ["RWY27_A1", "RWY09_E1"]:
                # Determine the airborne node based on runway
                if destination_node == "RWY27_A1":
                    airborne_node = "RWY27_AirBorne"
                else:  # RWY09_E1
                    airborne_node = "RWY09_AirBorne"
                
                # Start takeoff immediately
                takeoff_aircraft(canvas, aircraft_info, airborne_node)
            return
        
        # Get current and next node positions
        current_node = route[route_idx]
        next_node = route[route_idx + 1]
        
        # Check if we're about to pass through a HOLD node (departing from it)
        # If so, check if runway is clear before proceeding
        if current_node.endswith('_HOLD'):
            # Check if stop bar at this hold point is illuminated (red lights on)
            stop_bar_illuminated = current_node in stop_bar_draw_ids and len(stop_bar_draw_ids.get(current_node, [])) > 0
            
            if not is_runway_clear() or stop_bar_illuminated:
                # Runway is occupied or stop bar is on, wait and check again
                aircraft_info['waiting_at_hold'] = True
                app.after(adjust_delay(1000), move_to_next_node)  # Check again
                # Update stop bars in case this aircraft's presence changes the state
                if main_canvas:
                    update_stop_bars(main_canvas)
                return
            else:
                # Runway is clear and stop bar is off, proceed and update status
                aircraft_info['waiting_at_hold'] = False
                move_aircraft_status(aircraft_info['callsign'], 'Runway')
                # Update stop bars to show runway is now occupied
                if main_canvas:
                    update_stop_bars(main_canvas)
        
        start_x, start_y = nodes[current_node]
        end_x, end_y = nodes[next_node]
        
        # Calculate distance and steps
        distance = math.hypot(end_x - start_x, end_y - start_y)
        steps = max(int(distance / speed), 1)
        step_x = (end_x - start_x) / steps
        step_y = (end_y - start_y) / steps
        
        # Calculate target angle for this segment
        target_angle = math.degrees(math.atan2(end_y - start_y, end_x - start_x))
        
        # Get current heading from aircraft info (or initialize to target angle)
        if 'heading' not in aircraft_info:
            aircraft_info['heading'] = target_angle
        
        current_heading = aircraft_info['heading']
        
        # Calculate look-ahead angle for next segment if it exists
        next_segment_angle = target_angle
        has_next_segment = False
        if route_idx + 2 < len(route):
            next_next_node = route[route_idx + 2]
            next_end_x, next_end_y = nodes[next_next_node]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        
        # Turn parameters
        turn_zone_distance = 10  # Start turning within 15 pixels of node
        max_turn_per_step = 8.0  # Moderate turn rate for realistic steering (degrees per step)
        wheelbase = 40  # Distance from nose to main gear (affects turning behavior)
        corner_cut_radius = 10  # How much to cut the corner (pixels)
        
        def animate_segment(step=0):
            if step >= steps:
                # Reached next node, move to the next segment immediately without delay
                aircraft_info['route_index'] = route_idx + 1
                aircraft_info['node'] = next_node
                aircraft_info['position'] = nodes[next_node]
                # Continue immediately to next segment for smooth transition
                move_to_next_node()
                return
                return
            
            # Calculate base position on straight line (nose/front of aircraft)
            base_x = start_x + (step_x * step)
            base_y = start_y + (step_y * step)
            
            # Calculate distance to next node
            dist_to_next_node = math.hypot(end_x - base_x, end_y - base_y)
            
            # Calculate angle difference to next segment (if it exists)
            angle_diff_to_next = 0
            if has_next_segment:
                angle_diff_to_next = next_segment_angle - target_angle
                while angle_diff_to_next > 180:
                    angle_diff_to_next -= 360
                while angle_diff_to_next < -180:
                    angle_diff_to_next += 360
            
            # Apply smooth corner cutting if we have a next segment, we're in the turn zone, and there's a significant angle change
            curr_x = base_x
            curr_y = base_y
            
            # Only apply corner cutting if the angle change is significant (more than 5 degrees)
            if has_next_segment and dist_to_next_node <= turn_zone_distance and abs(angle_diff_to_next) > 5:
                # Calculate how much to cut the corner based on distance to node
                # Use a smooth easing function (cubic) for even smoother transition
                linear_factor = 1.0 - (dist_to_next_node / turn_zone_distance)  # 0 at edge, 1 at node
                cut_factor = linear_factor * linear_factor * linear_factor  # Cubic easing for smoothness
                
                # Calculate bisector angle (halfway between current and next segment)
                bisector_angle = target_angle + (angle_diff_to_next * 0.5)
                bisector_rad = math.radians(bisector_angle)
                
                # Offset position toward the bisector to smoothly cut the corner
                offset = corner_cut_radius * cut_factor
                curr_x = base_x + offset * math.cos(bisector_rad)
                curr_y = base_y + offset * math.sin(bisector_rad)
            
            # Determine which angle to turn toward - only if angle difference is significant
            if dist_to_next_node <= turn_zone_distance and has_next_segment and abs(angle_diff_to_next) > 5:
                # Within turn zone - start turning toward next segment
                turn_target = next_segment_angle
            else:
                # Outside turn zone - face current segment direction
                turn_target = target_angle
            
            # Gradually adjust heading toward turn target (tricycle steering)
            heading = aircraft_info['heading']
            angle_diff = turn_target - heading
            
            # Normalize angle difference to [-180, 180]
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            # Apply limited turn rate (tricycle steering)
            if abs(angle_diff) > max_turn_per_step:
                heading += max_turn_per_step if angle_diff > 0 else -max_turn_per_step
            else:
                heading = turn_target
            
            aircraft_info['heading'] = heading
            
            # Calculate position of main landing gear (rear wheels)
            # The main gear is behind the nose by the wheelbase distance
            angle_rad = math.radians(heading)
            rear_x = curr_x - wheelbase * math.cos(angle_rad)
            rear_y = curr_y - wheelbase * math.sin(angle_rad)
            
            # For tricycle steering, the triangle should pivot around the rear position
            # The nose is at curr_x, curr_y and the body extends back from there
            size = 12
            
            # Define triangle with nose at origin, body extending backward
            base_points = [
                (0, 0),              # Nose (front wheel at path position)
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

            if not is_safe_to_move(aircraft_info['callsign'], curr_x, curr_y):
                app.after(adjust_delay(200), animate_segment, step)
                return
            
            canvas.coords(aircraft_info['triangle_id'], *rotated_points)
            canvas.coords(aircraft_info['label_id'], curr_x, curr_y - size - 10)
            
            aircraft_info['position'] = (curr_x, curr_y)
            aircraft_info['rear_position'] = (rear_x, rear_y)  # Track rear for reference
            
            # Schedule next step
            app.after(adjust_delay(50), animate_segment, step + 1)
        
        animate_segment()
    
    move_to_next_node()

# ==============================
# RUNWAY OCCUPANCY CHECK
def is_runway_clear():
    """Check if the runway is clear (no aircraft in Runway status or at runway nodes)."""
    global runway_protected
    
    # Check if any aircraft is in the Runway status column
    for callsign, info in active_aircraft.items():
        if callsign in aircraft_labels:
            current_column = aircraft_labels[callsign].get('column')
            if current_column == 'Runway':
                runway_protected = False  # Runway occupied
                return False
    
    # Also check if any aircraft is at a runway node
    runway_nodes = ['RWY27_A1', 'RWY27_B1', 'RWY09_C1', 'RWY09_D1', 'RWY09_E1']
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
    
    # Takeoff parameters
    initial_speed = 5  # Starting speed (pixels per frame)
    final_speed = 20   # Speed at rotation/liftoff (pixels per frame)
    acceleration_distance = total_distance * 0.8  # Accelerate for 80% of runway
    max_turn_per_step = 8.0  # Same as taxi turning rate for consistency
    
    # Total steps based on average speed
    avg_speed = (initial_speed + final_speed) / 2
    total_steps = int(total_distance / avg_speed)
    
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
        route = ["RWY09_AirBorne", "RWY27_A1", "RWY27_B1", "RWY09_C1", "C1_HOLD", "TXY_C1"]
        deceleration_start = "RWY27_A1"  # Start slowing after this node
    else:  # runway_exit_node == "RWY27_B1"
        # Landing on runway 09 (from west to east), exiting at B
        # Must follow the runway centerline through all intermediate nodes
        route = ["RWY27_AirBorne", "RWY09_E1", "RWY09_D1", "RWY09_C1", "RWY27_B1", "B1_HOLD", "TXY_B1"]
        deceleration_start = "RWY09_E1"  # Start slowing after this node
    
    aircraft_info['route'] = route
    aircraft_info['route_index'] = 0
    aircraft_info['deceleration_start'] = deceleration_start
    
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
    except (ValueError, KeyError):
        aircraft_info['decel_total_distance'] = 1000
        aircraft_info['decel_distance_traveled'] = 0.0
        aircraft_info['deceleration_start_idx'] = 0
        aircraft_info['runway_exit_idx'] = len(route) - 1
    
    # Track status changes
    moved_to_runway = [False]
    
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
        avg_speed = (20 + 5) / 2  # Average between landing and taxi speed
        steps = max(int(distance / avg_speed), 1)
        
        # Calculate look-ahead angle for smooth turning
        next_segment_angle = target_angle
        has_next_segment = False
        if route_idx + 2 < len(route):
            next_next_node = route[route_idx + 2]
            next_end_x, next_end_y = nodes[next_next_node]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        
        # Turn parameters (same as taxi)
        turn_zone_distance = 20  # Start turning within this distance of node
        max_turn_per_step = 8.0  # Degrees per step
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
            
            # Move to Runway status column at start of landing (first segment)
            if route_idx == 0 and progress >= 0.1 and not moved_to_runway[0]:
                move_aircraft_status(aircraft_info['callsign'], 'Runway')
                moved_to_runway[0] = True
            
            # Calculate current speed with smooth deceleration
            if is_decelerating:
                # Use pre-calculated total deceleration distance
                decel_total_distance = aircraft_info.get('decel_total_distance', 1000)
                decel_distance_traveled = aircraft_info.get('decel_distance_traveled', 0.0)
                
                # Complete deceleration at 70% of the distance so aircraft reaches taxi speed before exit
                # Progress across decel zone (0 → 1), but reaches 1.0 at 70% of total distance
                decel_progress = min(1.0, decel_distance_traveled / max(decel_total_distance * 0.7, 1))
                
                # Cubic easing for smooth deceleration: 20 → 5 px/frame
                ease = decel_progress * decel_progress * decel_progress
                current_speed = 20 - (15 * ease)
            else:
                # Constant taxi speed after runway exit
                current_speed = 5
            
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
            
            # Update aircraft triangle
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
    for callsign, info in active_aircraft.items():
        # Check if aircraft is physically at the stand
        if info.get('node', '') == stand_name:
            return False
        # Check if stand is reserved as target for a landing aircraft
        if info.get('target_stand', '') == stand_name:
            return False
    return True

def find_available_stand():
    """Find the first available stand (not occupied or reserved by another aircraft)."""
    stands = ["STAND1a", "STAND2a", "STAND3a", "STAND4a", "STAND5a", "STAND6a", "STAND7a", "STAND8a"]
    
    # Check which stands are currently occupied or reserved
    occupied_stands = set()
    for callsign, info in active_aircraft.items():
        # Check physical occupancy
        node = info.get('node', '')
        if node in stands:
            occupied_stands.add(node)
        # Check reservations (target stands for landing aircraft)
        target = info.get('target_stand', '')
        if target in stands:
            occupied_stands.add(target)
    
    # Find first available stand
    for stand in stands:
        if stand not in occupied_stands:
            return stand
    
    # Return None if all stands occupied
    return None

def find_available_stands():
    """Return a list of available stands (not occupied or reserved)."""
    stands = ["STAND1a", "STAND2a", "STAND3a", "STAND4a", "STAND5a", "STAND6a", "STAND7a", "STAND8a"]
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

def taxi_to_stand_after_landing(canvas, aircraft_info, destination_stand):
    """Special taxi function for landing aircraft that ignores stop bars.
    
    This is a modified version of taxi_aircraft that doesn't check stop bars
    when exiting the runway.
    """
    import math
    
    # Get current node from aircraft info
    current_node = aircraft_info.get('node', '')
    
    # Find path using dijkstra
    route = dijkstra(current_node, destination_stand)
    
    if not route:
        print(f"No route found from {current_node} to {destination_stand}")
        return
    
    aircraft_info['route'] = route
    aircraft_info['route_index'] = 0
    
    def move_to_next_node():
        """Move aircraft to the next node in the route."""
        route_idx = aircraft_info.get('route_index', 0)
        
        if route_idx >= len(route) - 1:
            # Reached final destination (stand)
            aircraft_info['node'] = destination_stand
            aircraft_info['position'] = nodes[destination_stand]
            
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
        
        # Calculate distance and steps
        distance = math.hypot(end_x - start_x, end_y - start_y)
        steps = max(int(distance / 5), 1)  # speed = 5
        step_x = (end_x - start_x) / steps
        step_y = (end_y - start_y) / steps
        
        # Calculate target angle for this segment
        target_angle = math.degrees(math.atan2(end_y - start_y, end_x - start_x))
        
        # Get current heading from aircraft info (or initialize to target angle)
        if 'heading' not in aircraft_info:
            aircraft_info['heading'] = target_angle
        
        current_heading = aircraft_info['heading']
        
        # Calculate look-ahead angle for next segment if it exists
        next_segment_angle = target_angle
        has_next_segment = False
        if route_idx + 2 < len(route):
            next_next_node = route[route_idx + 2]
            next_end_x, next_end_y = nodes[next_next_node]
            next_segment_angle = math.degrees(math.atan2(next_end_y - end_y, next_end_x - end_x))
            has_next_segment = True
        
        # Turn parameters
        turn_zone_distance = 10
        max_turn_per_step = 8.0
        wheelbase = 40
        corner_cut_radius = 10
        
        def animate_segment(step=0):
            if step >= steps:
                # Reached next node, move to the next segment
                aircraft_info['route_index'] = route_idx + 1
                aircraft_info['node'] = next_node
                aircraft_info['position'] = nodes[next_node]
                move_to_next_node()
                return
            
            # Calculate base position on straight line
            base_x = start_x + (step_x * step)
            base_y = start_y + (step_y * step)
            
            # Calculate distance to next node
            dist_to_next_node = math.hypot(end_x - base_x, end_y - base_y)
            
            # Calculate angle difference to next segment
            angle_diff_to_next = 0
            if has_next_segment:
                angle_diff_to_next = next_segment_angle - target_angle
                while angle_diff_to_next > 180:
                    angle_diff_to_next -= 360
                while angle_diff_to_next < -180:
                    angle_diff_to_next += 360
            
            # Apply smooth corner cutting
            curr_x = base_x
            curr_y = base_y
            
            if has_next_segment and dist_to_next_node <= turn_zone_distance and abs(angle_diff_to_next) > 5:
                linear_factor = 1.0 - (dist_to_next_node / turn_zone_distance)
                cut_factor = linear_factor * linear_factor * linear_factor
                
                bisector_angle = target_angle + (angle_diff_to_next * 0.5)
                bisector_rad = math.radians(bisector_angle)
                
                offset = corner_cut_radius * cut_factor
                curr_x = base_x + offset * math.cos(bisector_rad)
                curr_y = base_y + offset * math.sin(bisector_rad)
            
            # Determine turn target
            if dist_to_next_node <= turn_zone_distance and has_next_segment and abs(angle_diff_to_next) > 5:
                turn_target = next_segment_angle
            else:
                turn_target = target_angle
            
            # Adjust heading
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
            
            # Update aircraft triangle
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
        if not main_canvas:
            return
        for callsign, ac_info in list(active_aircraft.items()):
            turnaround_job_id = ac_info.get('turnaround_job_id')
            if turnaround_job_id:
                try:
                    app.after_cancel(turnaround_job_id)
                except Exception:
                    pass
            if 'triangle_id' in ac_info and ac_info['triangle_id']:
                main_canvas.delete(ac_info['triangle_id'])
            if 'label_id' in ac_info and ac_info['label_id']:
                main_canvas.delete(ac_info['label_id'])
            remove_aircraft_from_status(callsign)
        active_aircraft.clear()

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
            return random.randint(15000, 30000)
        if rate == "Medium":
            return random.randint(20000, 40000)
        return random.randint(30000, 60000)

    def start_departure_from_stand(aircraft_info, stand_node):
        """Convert a parked aircraft into a departure and start pushback."""
        if not main_canvas:
            return

        callsign = aircraft_info.get('callsign')
        if not callsign or callsign not in active_aircraft:
            return

        if aircraft_info.get('node') != stand_node:
            return

        stand_n_node = f"{stand_node[:-1]}N"

        runway = runway_var.get()
        final_direction = "right" if runway == "09" else "left"
        runway_target = "A1_HOLD" if runway == "27" else "E1_HOLD"

        if callsign in aircraft_labels:
            move_aircraft_status(callsign, 'Departures')
        else:
            add_aircraft_to_status(callsign, 'Departures')

        app.after(adjust_delay(1000), lambda: pushback_aircraft(main_canvas, aircraft_info, stand_n_node, final_direction, runway_target=runway_target))

    def schedule_turnaround(aircraft_info, stand_node):
        """Schedule a turnaround so an arriving aircraft later departs."""
        delay_ms = get_turnaround_delay_ms()
        callsign = aircraft_info.get('callsign')

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
        """Seed 2-5 parked aircraft on random stands and schedule turnarounds."""
        if not main_canvas:
            return
        import random
        available_stands = find_available_stands()
        if not available_stands:
            return
        seed_count = min(len(available_stands), random.randint(2, 5))
        seed_stands = random.sample(available_stands, seed_count)

        for stand_node in seed_stands:
            x, y = nodes[stand_node]
            callsign = generate_unique_callsign("ARR")
            triangle_id, label_id = draw_aircraft(main_canvas, x, y, callsign, "north", color="blue")
            aircraft_info = {
                'callsign': callsign,
                'position': (x, y),
                'node': stand_node,
                'triangle_id': triangle_id,
                'label_id': label_id,
                'direction': 'north'
            }
            active_aircraft[callsign] = aircraft_info
            add_aircraft_to_status(callsign, 'Arrivals')
            schedule_turnaround(aircraft_info, stand_node)

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
        callsign = generate_unique_callsign("ARR")

        if runway == "27":
            spawn_node = "RWY09_AirBorne"
            runway_exit = "RWY09_C1"
            direction = "left"
        else:
            spawn_node = "RWY27_AirBorne"
            runway_exit = "RWY27_B1"
            direction = "right"

        x, y = nodes[spawn_node]
        triangle_id, label_id = draw_aircraft(main_canvas, x, y, callsign, direction, color="orange")

        aircraft_info = {
            'callsign': callsign,
            'position': (x, y),
            'node': spawn_node,
            'triangle_id': triangle_id,
            'label_id': label_id,
            'direction': direction,
            'target_stand': available_stand
        }
        active_aircraft[callsign] = aircraft_info
        add_aircraft_to_status(callsign, 'Arrivals')
        app.after(adjust_delay(500), lambda: landing_aircraft(main_canvas, aircraft_info, runway_exit))

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
    speed_options = [1.0, 2.0, 5.0, 10.0]
    speed_index = [0]  # Use list to allow modification in nested function
    speed_btn_ref = []  # Store reference to button for text updates
    
    def cycle_speed():
        global simulation_speed
        speed_index[0] = (speed_index[0] + 1) % len(speed_options)
        simulation_speed = speed_options[speed_index[0]]
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
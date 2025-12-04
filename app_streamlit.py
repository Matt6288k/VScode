import streamlit as st
from PIL import Image
import os
import math
from sim_core import (
    nodes, dijkstra, Aircraft, build_spline_path, 
    catmull_rom_spline
)

# ==============================
# STREAMLIT CONFIG
st.set_page_config(layout="wide", page_title="EGNX ATC Control Panel")

# Custom CSS for styling
st.markdown("""
<style>
    /* Hide streamlit header/footer */
    header {display: none;}
    footer {display: none;}
    
    /* Full width container */
    .main {padding: 0; max-width: 100%; margin: 0;}
    .block-container {padding: 0; max-width: 100%;}
    
    /* Control panel styling */
    .control-label {
        font-size: 18px;
        font-weight: bold;
        color: #1a5490;
        margin-bottom: 10px;
    }
    
    .status-header {
        background-color: #99ccff;
        padding: 10px;
        font-weight: bold;
        text-align: center;
        border: 1px solid #0066cc;
    }
    
    .status-content {
        background-color: #cce5ff;
        padding: 10px;
        border: 1px solid #0066cc;
        min-height: 300px;
    }
</style>
""", unsafe_allow_html=True)

# ==============================
# SESSION STATE
if "aircraft" not in st.session_state:
    st.session_state.aircraft = {}
if "current_aircraft" not in st.session_state:
    st.session_state.current_aircraft = None
if "operations" not in st.session_state:
    st.session_state.operations = "Normal Ops"
if "flow_rate" not in st.session_state:
    st.session_state.flow_rate = "Low"

# ==============================
# LAYOUT

# TOP SECTION: Airport Map Image
script_dir = os.path.dirname(os.path.abspath(__file__))
map_path = os.path.join(script_dir, "EGNX_Map_Zoom.tif")

if os.path.exists(map_path):
    img = Image.open(map_path)
    st.image(img, use_column_width=True)
else:
    st.error(f"Map image not found at: {map_path}")

# ==============================
# MIDDLE SECTION: Controls Row
st.markdown("---")

col1, col2, col3, col4 = st.columns([1, 1.5, 1.5, 1.5])

# Left: Operations
with col1:
    st.markdown("<p class='control-label'>Operations</p>", unsafe_allow_html=True)
    st.session_state.operations = st.radio(
        "ops_select",
        ("Normal Ops", "Low Visibility Ops"),
        label_visibility="collapsed"
    )

# Middle-left: Traffic Flow Rate
with col2:
    st.markdown("<p class='control-label'>Traffic Flow Rate</p>", unsafe_allow_html=True)
    st.session_state.flow_rate = st.radio(
        "flow_select",
        ("Low", "Medium", "High"),
        label_visibility="collapsed"
    )

# Middle-right: Movements Per Hour / Delays
with col3:
    st.markdown("<div class='status-header' style='background-color: #99ccff; padding: 15px;'><b>Movements Per Hour</b> | <b>Delays</b></div>", unsafe_allow_html=True)
    st.markdown("<div style='background-color: #cce5ff; height: 80px;'></div>", unsafe_allow_html=True)

# Right: Run Simulation Button
with col4:
    st.markdown("")
    st.markdown("")
    if st.button("▶ Run Simulation", use_container_width=True, key="run_sim_btn"):
        st.session_state["simulation_running"] = not st.session_state.get("simulation_running", False)

# ==============================
# AIRCRAFT MANAGEMENT SECTION
st.markdown("---")
st.subheader("Aircraft Management")

mgmt_col1, mgmt_col2, mgmt_col3 = st.columns([1, 1, 2])

with mgmt_col1:
    stand_list = [s for s in nodes if s.startswith("STAND") and s.endswith("a")]
    selected_stand = st.selectbox("Select Stand:", stand_list, key="stand_select")

with mgmt_col2:
    callsign = st.text_input("Callsign:", value="BA123", key="callsign_input")

with mgmt_col3:
    if st.button("Add Aircraft", use_container_width=True):
        if callsign.strip():
            ac = Aircraft(callsign.strip().upper(), selected_stand)
            ac.coords = nodes.get(selected_stand, ac.coords)
            st.session_state.aircraft[callsign.upper()] = ac
            st.session_state.current_aircraft = callsign.upper()
            st.success(f"Aircraft {callsign.upper()} added at {selected_stand}")

# ==============================
# TAXI & MOVEMENT
if st.session_state.aircraft:
    st.markdown("---")
    st.subheader("Taxi to Runway")
    
    col_ac, col_rwy, col_action = st.columns([1, 1, 1])
    
    with col_ac:
        ac_select = st.selectbox(
            "Select Aircraft:",
            list(st.session_state.aircraft.keys()),
            key="ac_select"
        )
    
    with col_rwy:
        rwy_options = ["RWY27_A1", "RWY27_B1", "RWY09_C1", "RWY09_D1", "RWY09_E1"]
        rwy_select = st.selectbox("Destination Runway:", rwy_options, key="rwy_select")
    
    with col_action:
        if st.button("Taxi", use_container_width=True, key="taxi_btn"):
            ac = st.session_state.aircraft[ac_select]
            route = dijkstra(ac.node, rwy_select)
            if route:
                ac.route = route
                spline_points, spline_map = build_spline_path(route, points_per_segment=15)
                ac.spline_points = spline_points
                ac.spline_route_idx_map = spline_map
                ac.dist_along_path = 0.0
                ac.status = f"Taxiing to {rwy_select}"
                st.success(f"Route planned: {' → '.join(route)}")
            else:
                st.error("No path found!")

# ==============================
# BOTTOM SECTION: Status Board
st.markdown("---")
st.markdown("## Aircraft Status")

# Display active aircraft
if st.session_state.aircraft:
    status_data = []
    for callsign, ac in st.session_state.aircraft.items():
        status_data.append({
            "Callsign": callsign,
            "Current Node": ac.node,
            "Status": ac.status,
            "Position": f"({ac.coords[0]:.0f}, {ac.coords[1]:.0f})"
        })
    
    st.dataframe(status_data, use_container_width=True)
else:
    st.info("No aircraft added yet.")

# 5-column status summary
st.markdown("---")
st.subheader("Movement Summary")
col_dep, col_tax, col_rwy, col_air, col_arr = st.columns(5)

with col_dep:
    st.markdown("<div class='status-header'>Departures</div>", unsafe_allow_html=True)
    st.markdown("<div class='status-content'></div>", unsafe_allow_html=True)

with col_tax:
    st.markdown("<div class='status-header'>Taxiing</div>", unsafe_allow_html=True)
    st.markdown("<div class='status-content'></div>", unsafe_allow_html=True)

with col_rwy:
    st.markdown("<div class='status-header'>Runway</div>", unsafe_allow_html=True)
    st.markdown("<div class='status-content'></div>", unsafe_allow_html=True)

with col_air:
    st.markdown("<div class='status-header'>Airborne</div>", unsafe_allow_html=True)
    st.markdown("<div class='status-content'></div>", unsafe_allow_html=True)

with col_arr:
    st.markdown("<div class='status-header'>Arrivals</div>", unsafe_allow_html=True)
    st.markdown("<div class='status-content'></div>", unsafe_allow_html=True)

st.markdown("---")
st.markdown("""
**Instructions:**
1. Select a stand and enter an aircraft callsign, then click "Add Aircraft"
2. Select the aircraft and a destination runway, then click "Taxi"
3. The aircraft will be assigned a route using Dijkstra pathfinding
4. Monitor the status below for real-time updates

This is a Streamlit-based preview of the EGNX ATC Control Panel. The underlying simulation logic (nodes, edges, pathfinding, splines) is shared with the desktop GUI.
""")

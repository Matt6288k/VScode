import streamlit as st
from PIL import Image
import os

st.set_page_config(layout="wide", page_title="EGNX ATC Panel")

# locate image relative to this script
script_dir = os.path.dirname(os.path.abspath(__file__))
map_path = os.path.join(script_dir, "EGNX_Map_Zoom.tif")

st.title("EGNX — ATC Control Panel (Streamlit Preview)")

# Top: image banner
if os.path.exists(map_path):
    img = Image.open(map_path)
    # show full width; Streamlit will scale while keeping aspect ratio
    st.image(img, use_column_width=True)
else:
    st.warning(f"Map image not found at: {map_path}")

# Middle controls: three columns for layout
col1, col2, col3 = st.columns([1,1,1])

with col1:
    st.subheader("Operations")
    ops = st.radio("", ("Normal Ops", "Low Visibility Ops"))
    st.write("Selected:", ops)

with col2:
    st.subheader("Traffic Flow Rate")
    tfr = st.radio("", ("Low", "Medium", "High"))
    st.write("Selected:", tfr)

with col3:
    st.subheader("")
    if st.button("▶ Run Simulation"):
        st.info("Run Simulation pressed — simulation logic not yet ported.")

# Bottom: status board with five columns
st.markdown("---")
cols = st.columns(5)
headers = ["Departures", "Taxiing", "Runway", "Airborne", "Arrivals"]
for c, h in zip(cols, headers):
    with c:
        st.subheader(h)
        # placeholder table; later we can populate with simulation data
        st.write("---")
        for i in range(6):
            st.text(" ")

st.sidebar.title("Notes")
st.sidebar.info("This is a Streamlit preview of the GUI. Interactive simulation code (CustomTkinter -> Streamlit) must be ported: Dijkstra, Aircraft movement and state should be reused, but UI code (customtkinter widgets) will need Streamlit equivalents.")

st.sidebar.markdown("**Run locally:**\n1. Install dependencies: `pip install -r requirements.txt`\n2. Run: `streamlit run app_streamlit.py`")

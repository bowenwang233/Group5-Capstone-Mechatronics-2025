# dashboard_app.py
"""
Digital Twin Dashboard (UI-only)

What this file does:
- Provides the Dash layout and lightweight callbacks to render:
  * 3 small line charts (velocity, acceleration, wheel angle)
  * Camera preview (top-right)
  * LiDAR point cloud (bottom-right)
  * Large Simulation panel (bottom-left)
  * Tiny floating Status card
- Reads *current* data from a shared state_store (threadsafe buffers),
  which you will implement in a separate module (e.g., ZeroMQ subscribers).

What this file does NOT do:
- It does NOT open sockets, read ZeroMQ, or write data.
- It does NOT manage threads for producers.
- It only *reads* from state_store.

Expected state_store API (you will implement later):
------------------------------------------------------------------
get_timeseries(window_sec: float) -> dict:
    returns {"t": list[float], "v": list[float], "a": list[float], "steer": list[float]}
    - - t is relative seconds in [-window_sec, 0] (past → 0)
    - v in m/s, a in m/s^2, steer in degrees
get_lidar() -> tuple[list[float], list[float], list[float]]:
    returns (xs, ys, zs) point cloud arrays (already downsampled)
get_camera_b64() -> str | None:
    returns a data-URI string like "data:image/jpeg;base64,...." or None
get_status() -> dict:
    returns {"battery": "85%", "conn": "OK" | "LOST", "mode": "AUTO" | "MANUAL"}
------------------------------------------------------------------

You can test this module before building state_store; it will show placeholders.
"""

from __future__ import annotations
import os
from typing import Optional

import dash
from dash import html, dcc, Output, Input, no_update
import plotly.graph_objects as go

from camera_stream import CameraStream
from LidarReceiver import LidarReceiver


CAR_IP = "192.168.68.103"   # ← set JetArcker IP
VIDEO_PORT = 5557

cam_stream = CameraStream(CAR_IP, VIDEO_PORT)
cam_stream.start()

lidar_receiver = LidarReceiver()   
lidar_receiver.start()

# ---------------------------------------------------------------------
# Try to import your future state_store. If not present, use fallbacks.
# ---------------------------------------------------------------------
try:
    import state_store  # you will create this file later
except Exception:
    state_store = None  # safe fallback


# ---------------------- UI CONSTANTS / STYLES ------------------------
# Realistic toy-car ranges (feel free to tune later)
VEL_RANGE   = [0, 4]     # m/s  (≈ 0–14.4 km/h)
ACC_RANGE   = [-4, 4]    # m/s^2
STEER_RANGE = [-35, 35]  # deg
HISTORY_SEC = 20.0       # window for plots
REFRESH_MS  = 200        # UI refresh rate (~5 Hz)

CARD_STYLE = {
    "backgroundColor": "#11151a",
    "border": "1px solid #1f2a36",
    "borderRadius": "16px",
    "padding": "12px",
    "boxShadow": "0 2px 10px rgba(0,0,0,0.25)",
}
GRID_STYLE = {
    "position": "relative",
    "display": "grid",
    # 3 columns for charts/sim (smaller each), 1 column for camera/lidar stack
    "gridTemplateColumns": "0.9fr 0.9fr 0.9fr 1.1fr",
    # taller second row to prioritize Simulation pane
    "gridTemplateRows": "auto 1fr",
    "gridTemplateAreas": (
        "'vel acc steer camera' "
        "'sim sim sim lidar'"
    ),
    "gap": "14px",
    "padding": "16px",
    "backgroundColor": "#0a0e14",
    "minHeight": "100vh",
    "color": "#dbe2ea",
    "fontFamily": "-apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif",
}


# -------------------------- FIGURE HELPERS ---------------------------
def mk_line(title, x, y, yrange, ytitle, height=None):
    fig = go.Figure()
    # Add the data trace (this was missing)
    fig.add_trace(go.Scatter(
        x=x or [],
        y=y or [],
        mode="lines",
        name=title,
        line=dict(width=2),
    ))
    fig.update_layout(
        title=title,
        template="plotly_dark",
        margin=dict(l=20, r=10, t=30, b=20),
        xaxis_title="time (s, past → 0)",
        yaxis_title=ytitle,
        autosize=True,
        height=height,
    )
    # Use your configured history window
    fig.update_xaxes(range=[-HISTORY_SEC, 0], showgrid=True, zeroline=False)
    # Either keep autoscale (nice for live demo)...
    fig.update_yaxes(autorange=True, showgrid=True, zeroline=True)
    # ...or lock to yrange:
    # fig.update_yaxes(range=yrange, showgrid=True, zeroline=True)
    return fig

def lidar_fig(xs, ys, zs):
    fig = go.Figure(
        data=[go.Scatter3d(
            x=xs, y=ys, z=zs, mode="markers",
            marker=dict(size=2, opacity=0.8),
            name="LiDAR"
        )]
    )
    fig.update_layout(
        title="LiDAR (Point Cloud)",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        template="plotly_dark",
        margin=dict(l=0, r=0, t=35, b=0),
        autosize=True,  # <- let it fill the card
    )
    return fig


# ----------------------------- APP -----------------------------------
app = dash.Dash(__name__, title="Digital Twin Dashboard")

app.layout = html.Div(
    [
        # Interval drives screen refresh; data is read from state_store each tick.
        dcc.Interval(id="tick", interval=REFRESH_MS, n_intervals=0),

        html.Div(
            [
                # --- Small charts (top-left row) ---
                html.Div(dcc.Graph(id="graph-velocity"), style=CARD_STYLE | {"gridArea": "vel"}),
                html.Div(dcc.Graph(id="graph-accel"),    style=CARD_STYLE | {"gridArea": "acc"}),
                html.Div(dcc.Graph(id="graph-steer"),    style=CARD_STYLE | {"gridArea": "steer"}),

                # --- Camera (top-right) ---
                html.Div(
                    [
                        html.Div("Camera", style={"marginBottom": "8px", "fontWeight": 600}),
                        html.Img(
                            id="camera-feed",
                            src="",  # will be updated by callback
                            style={
                                "width": "100%", "height": "240px",
                                "objectFit": "cover",
                                "background": "#0b0f14", "borderRadius": "10px",
                            },
                        ),
                        # trigger regular updates (~12.5 FPS)
                        dcc.Interval(id="camera-tick", interval=80, n_intervals=0),
                    ],
                    style=CARD_STYLE | {"gridArea": "camera"},
                ),

                # --- Simulation (dominant area bottom-left across 3 columns) ---
                html.Div(
                    [
                        html.Div("Simulation", style={"marginBottom": "8px", "fontWeight": 600}),
                        # This is just a visual placeholder; your sim adapter will own it.
                        html.Div(
                            id="simulation-viewport",
                            style={
                                "width": "100%",
                                "height": "100%",
                                "background": "repeating-linear-gradient(45deg, #0b0f14, #0b0f14 12px, #0e1319 12px, #0e1319 24px)",
                                "borderRadius": "10px",
                                "minHeight": "0",   # prevent overflow
                            },
                        ),
                    ],
                    style=CARD_STYLE | {"gridArea": "sim", "height": "100%"},
                ),

                # --- LiDAR (bottom-right) ---
                html.Div(
                [
                dcc.Graph(
                    id="graph-lidar",
                    style={"height": "100%"},
                    config={"responsive": True},
                ),
                dcc.Interval(id="lidar-update", interval=200, n_intervals=0),
                ],
                style=CARD_STYLE | {"gridArea": "lidar", "height": "100%", "minHeight": 0},
                ),

                # --- Tiny floating Status (corner) ---
                html.Div(
                    [
                        html.Div("Status", style={"marginBottom": "6px", "fontWeight": 600}),
                        html.Div(["Batt: ", html.Span(id="status-battery", children="—")]),
                        html.Div(["Conn: ", html.Span(id="status-conn", children="—")]),
                        html.Div(["Mode: ", html.Span(id="status-mode", children="—")]),
                    ],
                    style=CARD_STYLE | {
                        "position": "absolute",
                        "right": "22px",
                        "bottom": "22px",
                        "width": "160px",
                        "height": "120px",
                        "padding": "10px",
                        "zIndex": 5,
                    },
                ),
            ],
            id="grid",
            style=GRID_STYLE,
        ),
    ]
)


# ----------------------------- CALLBACKS -----------------------------
#Lidar callback
@app.callback(
    Output("graph-lidar", "figure"),
    Input("lidar-update", "n_intervals"),
)
def update_lidar_plot(_):
    x, y, r = lidar_receiver.latest_xy(default_radius=8.0, downsample=1)
    fig = go.Figure()
    fig.add_trace(go.Scattergl(x=x, y=y, mode="markers", marker=dict(size=3), name="LiDAR"))
    fig.add_trace(go.Scattergl(x=[0], y=[0], mode="markers", marker=dict(size=10), name="robot"))
    fig.add_trace(go.Scattergl(x=[0, r], y=[0, 0], mode="lines", name="heading"))
    fig.update_layout(
        xaxis=dict(scaleanchor="y", scaleratio=1, range=[-r, r], zeroline=False),
        yaxis=dict(range=[-r, r], zeroline=False),
        height=700,
        margin=dict(l=10, r=10, t=30, b=10),
        template="plotly_dark",
        title="LiDAR Point Cloud",
    )
    return fig
    
# Camera: its own tiny callback, driven by camera-tick
@app.callback(Output("camera-feed", "src"), Input("camera-tick", "n_intervals"))
def update_camera(_):
    b64 = cam_stream.latest_base64()
    if not b64:
        raise dash.exceptions.PreventUpdate
    return f"data:image/jpeg;base64,{b64}"

# UI: graphs, lidar, status — driven by tick (no camera output here)
@app.callback(
    Output("graph-velocity", "figure"),
    Output("graph-accel", "figure"),
    Output("graph-steer", "figure"),
    Output("status-battery", "children"),
    Output("status-conn", "children"),
    Output("status-mode", "children"),
    Input("tick", "n_intervals"),
    prevent_initial_call=False,
)
def update_ui(_n):
    # ---------- Timeseries ----------
    if state_store and hasattr(state_store, "get_timeseries"):
        try:
            ts = state_store.get_timeseries(HISTORY_SEC) or {}
        except Exception:
            ts = {}
    else:
        ts = {}

    t = ts.get("t", [])
    v = ts.get("v", [])
    a = ts.get("a", [])
    steer = ts.get("steer", [])

    fig_v = mk_line("Velocity", t, v, VEL_RANGE, "m/s")
    fig_a = mk_line("Acceleration", t, a, ACC_RANGE, "m/s²")
    fig_s = mk_line("Wheel Angle", t, steer, STEER_RANGE, "deg")

    # ---------- Status ----------
    if state_store and hasattr(state_store, "get_status"):
        try:
            st = state_store.get_status() or {}
        except Exception:
            st = {}
    else:
        st = {}
    batt = st.get("battery", "—")
    conn = st.get("conn", "—")
    mode = st.get("mode", "—")

    return fig_v, fig_a, fig_s, batt, conn, mode

# ----------------------------- ENTRY POINT ---------------------------
def _open_browser():
    """Open one browser tab when running with debug reloader."""
    import webbrowser, threading
    webbrowser.open_new("http://127.0.0.1:8050/")

if __name__ == "__main__":
    # Open a single tab even when the Werkzeug reloader forks.
    if os.environ.get("WERKZEUG_RUN_MAIN") == "true":
        import threading
        threading.Timer(1.0, _open_browser).start()

    # Dash 3.x uses app.run()
    try:
        app.run(host="0.0.0.0", port=8050, debug=False)
    finally:
        cam_stream.stop() 
        lidar_receiver.stop()   




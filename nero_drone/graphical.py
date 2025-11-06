#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import os

# -------------------------------------------------------------------
# File selection (ROS 2: default path = ~/ros2_ws/src/nero_drone/data)
# -------------------------------------------------------------------
Tk().withdraw()

default_dir = os.path.expanduser("~/ros2_ws/src/nero_drone/data")
os.makedirs(default_dir, exist_ok=True)

filename = askopenfilename(
    title="Select the Bebop CSV file",
    initialdir=default_dir,
    filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
)

if not filename:
    print("No file selected. Exiting...")
    exit()

print(f"Selected file: {filename}")

# -------------------------------------------------------------------
# Load data
# -------------------------------------------------------------------
df = pd.read_csv(filename)
df["time"] = pd.to_numeric(df["time"], errors="coerce")
df["t_rel"] = df["time"] - df["time"].iloc[0]

print("Data loaded:", df.shape)
print(df.head())

# -------------------------------------------------------------------
# Transform reference velocities from world to body frame
# -------------------------------------------------------------------
if all(col in df.columns for col in ["vxd", "vyd", "vzd", "yaw"]):
    print("Transforming reference velocities from world to body frame...")
    psi = df["yaw"]
    df["vxd_b"] = np.cos(psi) * df["vxd"] + np.sin(psi) * df["vyd"]
    df["vyd_b"] = -np.sin(psi) * df["vxd"] + np.cos(psi) * df["vyd"]
    df["vzd_b"] = df["vzd"]
else:
    print("Skipping velocity frame transformation (missing columns).")

# -------------------------------------------------------------------
# Unified plotting: positions + velocities in one figure
# -------------------------------------------------------------------
def plot_combined(df, title_suffix=""):
    t = df["t_rel"]
    t_min, t_max = t.min(), t.max()

    fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Bebop Flight Data {title_suffix}", fontsize=14, fontweight="bold")

    # POSITIONS (left column)
    ax = axes[0, 0]
    ax.plot(t, df["x"], label="x measured [m]")
    if "xd" in df.columns:
        ax.plot(t, df["xd"], "--", label="x ref [m]")
    ax.set_ylabel("X [m]"); ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    ax = axes[1, 0]
    ax.plot(t, df["y"], label="y measured [m]")
    if "yd" in df.columns:
        ax.plot(t, df["yd"], "--", label="y ref [m]")
    ax.set_ylabel("Y [m]"); ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    ax = axes[2, 0]
    ax.plot(t, df["z"], label="z measured [m]")
    if "zd" in df.columns:
        ax.plot(t, df["zd"], "--", label="z ref [m]")
    ax.set_ylabel("Z [m]"); ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    ax = axes[3, 0]
    ax.plot(t, df["yaw"], label="yaw measured [rad]")
    if "yawd" in df.columns:
        ax.plot(t, df["yawd"], "--", label="yaw ref [rad]")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Yaw [rad]")
    ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    # VELOCITIES (right column)
    ax = axes[0, 1]
    ax.plot(t, df["cmd_linx"], ":", label="Vx control [m/s]", alpha=0.5)
    ax.plot(t, df["linx"], label="Vx measured [m/s]")
    if "vxd_b" in df.columns:
        ax.plot(t, df["vxd_b"], "--", label="Vx ref (body) [m/s]")
    ax.set_ylabel("Vx [m/s]"); ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    ax = axes[1, 1]
    ax.plot(t, df["cmd_liny"], ":", label="Vy control [m/s]", alpha=0.5)
    ax.plot(t, df["liny"], label="Vy measured [m/s]")
    if "vyd_b" in df.columns:
        ax.plot(t, df["vyd_b"], "--", label="Vy ref (body) [m/s]")
    ax.set_ylabel("Vy [m/s]"); ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    ax = axes[2, 1]
    ax.plot(t, df["cmd_linz"], ":", label="Vz control [m/s]", alpha=0.5)
    ax.plot(t, df["linz"], label="Vz measured [m/s]")
    if "vzd_b" in df.columns:
        ax.plot(t, df["vzd_b"], "--", label="Vz ref (body) [m/s]")
    ax.set_ylabel("Vz [m/s]"); ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    ax = axes[3, 1]
    ax.plot(t, df["cmd_angz"], ":", label="Yaw rate control [rad/s]", alpha=0.5)
    ax.plot(t, df["yaw_rate"], label="Yaw rate measured [rad/s]")
    if "wyawd" in df.columns:
        ax.plot(t, df["wyawd"], "--", label="Yaw rate ref [rad/s]")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Yaw rate [rad/s]")
    ax.grid(True); ax.legend()
    ax.set_xlim(t_min, t_max)

    # Ajustar para eliminar márgenes innecesarios
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.subplots_adjust(left=0.05, right=0.98, top=0.93, bottom=0.07, wspace=0.25, hspace=0.25)
    plt.show()

# -------------------------------------------------------------------
# Plot only once
# -------------------------------------------------------------------
plot_combined(df, "(Full)")

#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# -------------------------------------------------------------------
# File selection
# -------------------------------------------------------------------
Tk().withdraw()
filename = askopenfilename(
    title="Select the Bebop CSV file",
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
t = df["time"] - df["time"].iloc[0]
df["t_rel"] = t

print("Data loaded:", df.shape)
print(df.head())


def wrap_to_pi(angle_rad):
    """Wrap angles to range [-π, π]."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def plot_data(dataframe):
    """Plot position and velocity data with consistent limits."""
    t = dataframe["t_rel"]

    # -------------------------------------------------------------------
    # FIGURE 1: POSITIONS (measured vs reference)
    # -------------------------------------------------------------------
    plt.figure("Positions (measured vs reference)", figsize=(12, 10))

    # X
    plt.subplot(4, 1, 1)
    plt.plot(t, dataframe["x"], label="x measured [m]")
    if "xd" in dataframe.columns:
        plt.plot(t, dataframe["xd"], "--", label="x ref [m]")
    plt.title("X Position")
    plt.ylabel("[m]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # Y
    plt.subplot(4, 1, 2)
    plt.plot(t, dataframe["y"], label="y measured [m]")
    if "yd" in dataframe.columns:
        plt.plot(t, dataframe["yd"], "--", label="y ref [m]")
    plt.title("Y Position")
    plt.ylabel("[m]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # Z
    plt.subplot(4, 1, 3)
    plt.plot(t, dataframe["z"], label="z measured [m]")
    if "zd" in dataframe.columns:
        plt.plot(t, dataframe["zd"], "--", label="z ref [m]")
    plt.title("Z Position")
    plt.ylabel("[m]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])
    plt.ylim(bottom=0)

    # YAW
    plt.subplot(4, 1, 4)
    yaw = wrap_to_pi(dataframe["yaw_pos"])
    plt.plot(t, yaw, label="Yaw measured [rad]")
    if "yawd" in dataframe.columns:
        plt.plot(t, wrap_to_pi(dataframe["yawd"]), "--", label="Yaw ref [rad]")
    plt.title("Yaw Angle")
    plt.xlabel("Time [s]")
    plt.ylabel("[rad]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])
    plt.ylim(-np.pi, np.pi)

    plt.tight_layout()

    # -------------------------------------------------------------------
    # FIGURE 2: VELOCITIES (measured vs reference vs control)
    # -------------------------------------------------------------------
    plt.figure("Velocities (measured vs reference vs control)", figsize=(12, 10))

    # VX
    plt.subplot(4, 1, 1)
    plt.plot(t, dataframe["cmd_linx"], ":", label="Vx control [m/s]", alpha=0.4)
    plt.plot(t, dataframe["linx"], label="Vx measured [m/s]")
    if "vxd" in dataframe.columns:
        plt.plot(t, dataframe["vxd"], "--", label="Vx ref [m/s]")
    plt.title("X Velocity")
    plt.ylabel("[m/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # VY
    plt.subplot(4, 1, 2)
    plt.plot(t, dataframe["cmd_liny"], ":", label="Vy control [m/s]", alpha=0.4)
    plt.plot(t, dataframe["liny"], label="Vy measured [m/s]")
    if "vyd" in dataframe.columns:
        plt.plot(t, dataframe["vyd"], "--", label="Vy ref [m/s]")
    plt.title("Y Velocity")
    plt.ylabel("[m/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # VZ
    plt.subplot(4, 1, 3)
    plt.plot(t, dataframe["cmd_linz"], ":", label="Vz control [m/s]", alpha=0.4)
    plt.plot(t, dataframe["linz"], label="Vz measured [m/s]")
    if "vzd" in dataframe.columns:
        plt.plot(t, dataframe["vzd"], "--", label="Vz ref [m/s]")
    plt.title("Z Velocity")
    plt.ylabel("[m/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # WYAW
    plt.subplot(4, 1, 4)
    plt.plot(t, dataframe["cmd_angz"], ":", label="Yaw rate control [rad/s]", alpha=0.4)
    plt.plot(t, dataframe["yaw_rate"], label="Yaw rate measured [rad/s]")
    if "wyawd" in dataframe.columns:
        plt.plot(t, dataframe["wyawd"], "--", label="Yaw rate ref [rad/s]")
    plt.title("Yaw Angular Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("[rad/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    plt.tight_layout()
    plt.show()


# -------------------------------------------------------------------
# Plot complete data
# -------------------------------------------------------------------
plot_data(df)

# -------------------------------------------------------------------
# Request time crop
# -------------------------------------------------------------------
try:
    t_min = float(input("\nEnter start time (s): "))
    t_max = float(input("Enter end time (s): "))

    if t_min >= t_max:
        print("Start time must be less than end time.")
        exit()

    # Filter and re-align time
    df_trimmed = df[(df["t_rel"] >= t_min) & (df["t_rel"] <= t_max)].copy()
    df_trimmed["t_rel"] = df_trimmed["t_rel"] - df_trimmed["t_rel"].iloc[0]

    print(f"Trimmed data: {df_trimmed.shape}")
    plot_data(df_trimmed)

except Exception as e:
    print(f"Error selecting interval: {e}")

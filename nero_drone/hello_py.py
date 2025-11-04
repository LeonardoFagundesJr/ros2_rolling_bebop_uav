#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# -------------------------------------------------------------------
# Selección de archivo
# -------------------------------------------------------------------
Tk().withdraw()
filename = askopenfilename(
    title="Selecciona el archivo CSV del Bebop",
    filetypes=[("Archivos CSV", "*.csv"), ("Todos los archivos", "*.*")]
)
if not filename:
    print("No se seleccionó ningún archivo. Saliendo...")
    exit()

print(f"Archivo seleccionado: {filename}")

# -------------------------------------------------------------------
# Cargar datos
# -------------------------------------------------------------------
df = pd.read_csv(filename)
df["time"] = pd.to_numeric(df["time"], errors="coerce")
t = df["time"] - df["time"].iloc[0]
df["t_rel"] = t

print("Datos cargados:", df.shape)
print(df.head())


def wrap_to_pi(angle_rad):
    """Ajusta ángulos a rango [-π, π]."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def plot_data(dataframe):
    """Función que grafica posiciones y velocidades con límites consistentes."""
    t = dataframe["t_rel"]

    # -------------------------------------------------------------------
    # FIGURA 1: POSICIONES (x, y, z, yaw)
    # -------------------------------------------------------------------
    plt.figure("Posiciones (medida vs referencia)", figsize=(12, 10))

    # X
    plt.subplot(4, 1, 1)
    plt.plot(t, dataframe["x"], label="x medida [m]")
    if "xd" in dataframe.columns:
        plt.plot(t, dataframe["xd"], "--", label="x ref [m]")
    plt.title("Posición X")
    plt.ylabel("[m]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])  # recorte exacto

    # Y
    plt.subplot(4, 1, 2)
    plt.plot(t, dataframe["y"], label="y medida [m]")
    if "yd" in dataframe.columns:
        plt.plot(t, dataframe["yd"], "--", label="y ref [m]")
    plt.title("Posición Y")
    plt.ylabel("[m]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # Z
    plt.subplot(4, 1, 3)
    plt.plot(t, dataframe["z"], label="z medida [m]")
    if "zd" in dataframe.columns:
        plt.plot(t, dataframe["zd"], "--", label="z ref [m]")
    plt.title("Posición Z")
    plt.ylabel("[m]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])
    plt.ylim(bottom=0)  # siempre desde cero

    # YAW
    plt.subplot(4, 1, 4)
    yaw = wrap_to_pi(dataframe["yaw_pos"])
    plt.plot(t, yaw, label="Yaw medida [rad]")
    if "yawd" in dataframe.columns:
        plt.plot(t, wrap_to_pi(dataframe["yawd"]), "--", label="Yaw ref [rad]")
    plt.title("Posición Yaw")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("[rad]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])
    plt.ylim(-np.pi, np.pi)  # siempre en rango [-pi, pi]

    plt.tight_layout()

    # -------------------------------------------------------------------
    # FIGURA 2: VELOCIDADES (medida vs referencia vs control)
    # -------------------------------------------------------------------
    plt.figure("Velocidades (medida vs referencia vs control)", figsize=(12, 10))

    # VX
    plt.subplot(4, 1, 1)
    plt.plot(t, dataframe["cmd_linx"], ":", label="Vx control [m/s]", alpha=0.4)
    plt.plot(t, dataframe["linx"], label="Vx medida [m/s]")
    if "vxd" in dataframe.columns:
        plt.plot(t, dataframe["vxd"], "--", label="Vx ref [m/s]")
    plt.title("Velocidad X")
    plt.ylabel("[m/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # VY
    plt.subplot(4, 1, 2)
    plt.plot(t, dataframe["cmd_liny"], ":", label="Vy control [m/s]", alpha=0.4)
    plt.plot(t, dataframe["liny"], label="Vy medida [m/s]")
    if "vyd" in dataframe.columns:
        plt.plot(t, dataframe["vyd"], "--", label="Vy ref [m/s]")
    plt.title("Velocidad Y")
    plt.ylabel("[m/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # VZ
    plt.subplot(4, 1, 3)
    plt.plot(t, dataframe["cmd_linz"], ":", label="Vz control [m/s]", alpha=0.4)
    plt.plot(t, dataframe["linz"], label="Vz medida [m/s]")
    if "vzd" in dataframe.columns:
        plt.plot(t, dataframe["vzd"], "--", label="Vz ref [m/s]")
    plt.title("Velocidad Z")
    plt.ylabel("[m/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    # WYAW
    plt.subplot(4, 1, 4)
    plt.plot(t, dataframe["cmd_angz"], ":", label="Yaw rate control [rad/s]", alpha=0.4)
    plt.plot(t, dataframe["yaw_rate"], label="Yaw rate medida [rad/s]")
    if "wyawd" in dataframe.columns:
        plt.plot(t, dataframe["wyawd"], "--", label="Yaw rate ref [rad/s]")
    plt.title("Velocidad angular Yaw")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("[rad/s]")
    plt.grid(True)
    plt.legend()
    plt.xlim(t.iloc[0], t.iloc[-1])

    plt.tight_layout()
    plt.show()


# -------------------------------------------------------------------
# Graficar datos completos
# -------------------------------------------------------------------
plot_data(df)

# -------------------------------------------------------------------
# Solicitar recorte temporal
# -------------------------------------------------------------------
try:
    t_min = float(input("\nIntroduce el tiempo inicial (s): "))
    t_max = float(input("Introduce el tiempo final (s): "))

    if t_min >= t_max:
        print("El tiempo inicial debe ser menor que el final.")
        exit()

    # Filtrar y reajustar tiempo
    df_recortado = df[(df["t_rel"] >= t_min) & (df["t_rel"] <= t_max)].copy()
    df_recortado["t_rel"] = df_recortado["t_rel"] - df_recortado["t_rel"].iloc[0]

    print(f"Datos recortados: {df_recortado.shape}")
    plot_data(df_recortado)

except Exception as e:
    print(f"Error en la selección del intervalo: {e}")

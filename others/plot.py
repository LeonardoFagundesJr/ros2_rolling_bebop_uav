#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog

def main():
    # Ocultar la ventana principal de Tkinter
    root = Tk()
    root.withdraw()

    # Abrir diálogo para seleccionar archivo
    file_path = filedialog.askopenfilename(
        title="Selecciona archivo de log CSV",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )

    if not file_path:
        print("No se seleccionó ningún archivo.")
        return

    print(f"Leyendo archivo: {file_path}")

    # Leer el archivo CSV
    df = pd.read_csv(file_path)

    # Verificar columnas esperadas
    expected_cols = [
        "time", "ref_x", "ref_y", "ref_z", "ref_yaw",
        "real_x", "real_y", "real_z", "real_yaw"
    ]
    for c in expected_cols:
        if c not in df.columns:
            raise ValueError(f"Columna faltante en el CSV: {c}")

    # Normalizar tiempo
    df["time"] = df["time"] - df["time"].iloc[0]

    # Crear figura
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    labels = ['X', 'Y', 'Z', 'Yaw']
    ref_cols = ['ref_x', 'ref_y', 'ref_z', 'ref_yaw']
    real_cols = ['real_x', 'real_y', 'real_z', 'real_yaw']

    for i in range(4):
        axs[i].plot(df["time"], df[ref_cols[i]], 'r--', label=f"Ref {labels[i]}")
        axs[i].plot(df["time"], df[real_cols[i]], 'b', label=f"Real {labels[i]}")
        axs[i].set_ylabel(f"{labels[i]} velocity")
        axs[i].grid(True)
        axs[i].legend()

    axs[-1].set_xlabel("Time [s]")
    plt.suptitle("Velocity Reference vs Real Velocity - Bebop 2")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

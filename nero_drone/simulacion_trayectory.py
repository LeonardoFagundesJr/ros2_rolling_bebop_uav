#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

# Parámetros de la trayectoria
dt = 1.0 / 10.0   # 10 Hz
T_total = 40.0
omega = 2 * np.pi / T_total
t_vec = np.arange(0, T_total, dt)

# Trayectoria tipo 8 (Lemniscata de Gerono)
x = 1.5 * np.sin(omega * t_vec)
y = 1.5 * np.sin(omega * t_vec) * np.cos(omega * t_vec)
z = 1.0 + 0.3 * np.sin(0.5 * omega * t_vec) - 0.5

# Derivadas (velocidades)
dx = np.gradient(x, dt)
dy = np.gradient(y, dt)
dz = np.gradient(z, dt)
yaw = np.unwrap(np.arctan2(dy, dx))

# Configuración de la figura
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Simulación de trayectoria tipo 8 (10 Hz, 40 s)")

# Dibujar la trayectoria completa
ax.plot(x, y, '--', color='gray', alpha=0.5, label="Trayectoria prevista")
ax.legend()

# Crear el triángulo (representa el dron)
triangle, = ax.plot([], [], 'b-', lw=2)

# Función para construir el triángulo en la orientación dada
def get_triangle_coords(xc, yc, yaw, size=0.15):
    pts = np.array([
        [size, 0],
        [-size/2, size/2],
        [-size/2, -size/2],
        [size, 0]
    ])
    R = np.array([[np.cos(yaw), -np.sin(yaw)],
                  [np.sin(yaw),  np.cos(yaw)]])
    rotated = (R @ pts.T).T
    rotated[:, 0] += xc
    rotated[:, 1] += yc
    return rotated

# Inicialización para la animación
def init():
    triangle.set_data([], [])
    return triangle,

# Actualización de cada frame
def update(frame):
    tri = get_triangle_coords(x[frame], y[frame], yaw[frame])
    triangle.set_data(tri[:, 0], tri[:, 1])
    return triangle,

# Crear la animación
ani = animation.FuncAnimation(
    fig, update,
    frames=len(t_vec),
    init_func=init,
    blit=True,
    interval=10 * dt,
    repeat=False  # <-- Detiene la animación al finalizar
)

# Mostrar la animación
plt.show()

import numpy as np
import pandas as pd

# Parámetros de la curva
x_points = np.linspace(0, 2 * np.pi, 15)  # 15 puntos para ida
y_points_up = 30 * np.sin(x_points - np.pi/2)  # Desfase para iniciar en -30 y terminar en 30
y_points = np.concatenate((y_points_up, y_points_up[::-1]))  # Ida y vuelta

# Guardar solo los valores en formato CSV
np.savetxt("puntos_sinusoidales.csv", y_points, delimiter=",", fmt="%.2f")

print("Archivo CSV generado: puntos_sinusoidales.csv")

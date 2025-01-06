import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Trajectoire optimale trouvée (simulée pour cet exemple)
q_learning_path = [
    (0.5, 0.5), (1.5, 1.5), (2.5, 1.5), (3.5, 2.5), (4.5, 3.5)
]  # Coordonnées du centre des cases

# Obstacles
obstacle = [(2, 2), (3, 2), (2, 3), (3, 3)]

def is_segment_crossing_obstacle(p1, p2, obstacles):
    """Vérifie si un segment entre p1 et p2 traverse un obstacle."""
    for obs in obstacles:
        obs_center = np.array([obs[0], obs[1]])
        obs_radius = 0.5
        segment_vector = np.array(p2) - np.array(p1)
        obs_vector = obs_center - np.array(p1)
        projection_length = np.dot(obs_vector, segment_vector) / np.linalg.norm(segment_vector)**2
        projection_point = np.array(p1) + projection_length * segment_vector
        distance_to_obs = np.linalg.norm(projection_point - obs_center)
        if 0 <= projection_length <= 1 and distance_to_obs < obs_radius:
            return True
    return False

# Générer une trajectoire ajustée pour contourner les obstacles
adjusted_path = [q_learning_path[0]]  # Commencer au premier point
for i in range(1, len(q_learning_path)):
    p1 = adjusted_path[-1]
    p2 = q_learning_path[i]
    if is_segment_crossing_obstacle(p1, p2, obstacle):
        # Ajouter un point intermédiaire pour contourner l'obstacle
        mid_point = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        adjusted_path.append(mid_point)
    adjusted_path.append(p2)

# Convertir en array pour les splines
adjusted_path = np.array(adjusted_path)

# Extraire les coordonnées x et y
x = adjusted_path[:, 0]
y = adjusted_path[:, 1]

# Génération des splines pour rendre la trajectoire fluide
t = np.arange(len(adjusted_path))  # Indices des points comme "temps"
cs_x = CubicSpline(t, x)    # Spline pour x
cs_y = CubicSpline(t, y)    # Spline pour y

# Interpolation pour un chemin lisse
t_fine = np.linspace(0, len(adjusted_path) - 1, 200)  # Plus de points
x_smooth = cs_x(t_fine)
y_smooth = cs_y(t_fine)

# Visualisation
plt.figure(figsize=(6, 6))
plt.plot(x, y, 'o--', label="Trajectoire ajustée", color="blue")
plt.plot(x_smooth, y_smooth, '-', label="Trajectoire fluide (spline)", color="red")

# Ajouter obstacles
for obs in obstacle:
    plt.gca().add_patch(plt.Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color="black"))

# Configurations du graphique
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.title("Trajectoire ajustée vs Trajectoire fluide")
plt.show()

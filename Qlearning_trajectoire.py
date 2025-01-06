import numpy as np
import matplotlib.pyplot as plt
import random

# Paramètres de la grille et du Q-learning
original_grid_size = 5
subdivision_factor = 10  # Nombre de subdivisions pour lisser la trajectoire
alpha = 0.1  # Taux d'apprentissage
gamma = 0.9  # Facteur de discount
epsilon = 0.1  # Taux d'exploration

# Définition de l'environnement : récompenses, obstacles, départ, objectif
goal_position = (4, 4)
start_position = (0, 0)
obstacles = [
    (1, 1),
    (1, 2),
    (2, 2),
    (3, 3)
]

# Initialisation de la Q-table
q_table = np.zeros((original_grid_size, original_grid_size, 8))  # 8 directions possibles

# Actions possibles : haut, bas, gauche, droite, et diagonales
actions = {
    0: (-1, 0),   # Haut
    1: (1, 0),    # Bas
    2: (0, -1),   # Gauche
    3: (0, 1),    # Droite
    4: (-1, -1),  # Diagonale haut-gauche
    5: (-1, 1),   # Diagonale haut-droite
    6: (1, -1),   # Diagonale bas-gauche
    7: (1, 1)     # Diagonale bas-droite
}

def get_next_position(state, action):
    row, col = state
    row += actions[action][0]
    col += actions[action][1]
    if row < 0 or row >= original_grid_size or col < 0 or col >= original_grid_size:
        return state
    return (row, col)

def get_reward(state):
    if state == goal_position:
        return 100
    elif state in obstacles:
        return -100
    else:
        return -1

def choose_action(state):
    if random.uniform(0, 1) < epsilon:
        return random.choice(list(actions.keys()))
    else:
        row, col = state
        return np.argmax(q_table[row, col])

def train_agent(episodes):
    for episode in range(episodes):
        state = start_position
        while state != goal_position:
            action = choose_action(state)
            next_state = get_next_position(state, action)
            reward = get_reward(next_state)
            row, col = state
            next_row, next_col = next_state
            q_table[row, col, action] = q_table[row, col, action] + alpha * (
                reward + gamma * np.max(q_table[next_row, next_col]) - q_table[row, col, action]
            )
            state = next_state

def find_optimal_path():
    current_position = start_position
    path = [current_position]
    while current_position != goal_position:
        row, col = current_position
        best_action = np.argmax(q_table[row, col])
        next_position = get_next_position(current_position, best_action)
        if next_position == current_position:
            break  # Évite les boucles infinies
        path.append(next_position)
        current_position = next_position
    return path

def smooth_trajectory(path):
    """Ajoute des subdivisions entre les points du chemin pour lisser la trajectoire."""
    smoothed_path = []
    for i in range(len(path) - 1):
        start = np.array(path[i])
        end = np.array(path[i + 1])
        for t in np.linspace(0, 1, subdivision_factor):
            interpolated_point = (1 - t) * start + t * end
            smoothed_path.append(interpolated_point)
    smoothed_path.append(path[-1])  # Ajoute le dernier point
    return smoothed_path

# Entraîner l'agent
train_agent(episodes=1000)
optimal_path = find_optimal_path()
smoothed_path = smooth_trajectory(optimal_path)

# Extraction des coordonnées pour visualisation
x, y = zip(*smoothed_path)

# Visualisation
plt.figure(figsize=(6, 6))
plt.plot(x, y, '-', label="Trajectoire lissée", color="blue")

# Ajouter les obstacles
for obs in obstacles:
    plt.gca().add_patch(plt.Rectangle((obs[1], obs[0]), 1, 1, color="black"))

plt.scatter(*zip(*optimal_path), color="red", label="Points originaux")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.title("Trajectoire optimale avec lissage")
plt.show()

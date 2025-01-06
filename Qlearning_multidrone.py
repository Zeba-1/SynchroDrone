import numpy as np
import random
import matplotlib.pyplot as plt
import time

# Paramètres de la grille et du Q-learning
grid_size = 11
alpha = 0.1  # Taux d'apprentissage
gamma = 0.9  # Facteur de discount
epsilon = 0.1  # Taux d'exploration

grid_length, grid_width = 11, 11
start_position = (0, 0)  # Départ en haut à gauche de la grille
goal_position = (8, 1)  # Cible en bas à droite de la grille
obstacles = [(0, 4), (0, 9), (0, 10),
            (1, 4), (1, 9), (1, 10),
            (2, 0), (2, 4), (2, 5), (2, 9), (2, 10),
            (3, 5), (3, 9), (3, 10),
            (4, 9), (4, 10),
            (5, 5),
            (6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5),
            (8, 0), (8, 2)]  # Ensemble d'obstacles
"""
# Définition de l'environnement
goal_position = (9, 9)
obstacles = [(3, 3), (3, 4), (4, 3), (7, 7), (7, 8), (8, 7)]
"""
# Drones
num_drones = 3
start_positions = [(0, 0), (0, 1), (1, 0)]  # Positions initiales des drones
q_tables = [np.zeros((grid_size, grid_size, 4)) for _ in range(num_drones)]  # Q-tables pour chaque drone

# Actions possibles (haut, bas, gauche, droite)
actions = {
    0: (-1, 0),  # Haut
    1: (1, 0),   # Bas
    2: (0, -1),  # Gauche
    3: (0, 1)    # Droite
}
"""
# Visualisation dynamique
def display_grid(states):
    """"""Affiche la grille avec les positions actuelles des drones.
    grid = np.full((grid_size, grid_size), '.', dtype=str)
    for obs in obstacles:
        grid[obs] = 'X'
    grid[goal_position] = 'G'
    for i, state in enumerate(states):
        grid[state] = str(i + 1)

    print("\n".join("".join(row) for row in grid))
    print("\n" + "-" * (grid_size * 2))

    # Matplotlib pour visualisation graphique
    plt.imshow(grid == '.', cmap="gray", origin="upper")
    plt.scatter(*goal_position[::-1], color='green', label="Goal")
    for obs in obstacles:
        plt.scatter(*obs[::-1], color='red', label="Obstacle" if obs == obstacles[0] else "")
    for i, state in enumerate(states):
        plt.scatter(state[1], state[0], label=f"Drone {i + 1}", s=100)

    plt.legend(loc="upper right")
    plt.pause(0.5)
    plt.clf()
"""
def get_next_position(state, action):
    """Calcul de la prochaine position en fonction de l'action choisie."""
    row, col = state
    row += actions[action][0]
    col += actions[action][1]
    # Vérifier les limites de la grille
    if row < 0 or row >= grid_size or col < 0 or col >= grid_size:
        return state  # Si hors grille, rester à la même position
    return (row, col)

def get_reward(state, drone_id, positions):
    """Fonction de récompense spécifique pour chaque drone."""
    if state == goal_position:
        return 100  # Récompense pour atteindre l'objectif
    elif state in obstacles:
        return -100  # Pénalité pour collision avec un obstacle
    elif state in positions and positions[state] != drone_id:
        return -150  # Pénalité pour collision avec un autre drone
    else:
        return -1  # Pénalité par défaut pour chaque mouvement

def choose_action(state, q_table):
    """Choisit l'action basée sur epsilon-greedy : exploration ou exploitation."""
    if random.uniform(0, 1) < epsilon:
        return random.choice(list(actions.keys()))  # Exploration
    else:
        row, col = state
        return np.argmax(q_table[row, col])  # Exploitation

def train_multi_agent(episodes):
    """Entraîne plusieurs drones sur un certain nombre d'épisodes."""
    for episode in range(episodes):
        states = start_positions[:]  # Positions initiales des drones
        total_rewards = [0] * num_drones
        positions = {pos: i for i, pos in enumerate(states)}  # Suivi des positions des drones

        steps = 0
        while any(state != goal_position for state in states):
            for i in range(num_drones):
                state = states[i]

                if state == goal_position:
                    continue  # Si le drone est déjà arrivé, il ne bouge plus

                # Choisir une action et obtenir le nouvel état
                action = choose_action(state, q_tables[i])
                next_state = get_next_position(state, action)

                # Calculer la récompense pour ce drone
                reward = get_reward(next_state, i, positions)

                # Mettre à jour la Q-table avec la formule du Q-learning
                row, col = state
                next_row, next_col = next_state
                q_tables[i][row, col, action] = q_tables[i][row, col, action] + alpha * (
                    reward + gamma * np.max(q_tables[i][next_row, next_col]) - q_tables[i][row, col, action]
                )

                # Mettre à jour l'état du drone
                if state in positions:
                    positions.pop(state)  # Retirer l'ancienne position
                if next_state not in positions:
                    positions[next_state] = i  # Ajouter la nouvelle position
                states[i] = next_state
                total_rewards[i] += reward

            steps += 1

        # Affichage des récompenses totales pour observer l'apprentissage
        print(f"Épisode {episode + 1}: Récompenses totales = {total_rewards}")

# Lancer l'apprentissage multi-agent sur 1000 épisodes
#plt.figure(figsize=(8, 8))
train_multi_agent(episodes=2000)
#plt.close()

# Trajectoire optimale pour chaque drone
def find_optimal_paths():
    """Trouve et retourne les trajectoires optimales pour tous les drones."""
    paths = []
    for i in range(num_drones):
        print("HERE")
        current_position = start_positions[i]
        path = [current_position]
        steps = 0  # Compteur de pas
        while current_position != goal_position:
            row, col = current_position
            best_action = np.argmax(q_tables[i][row, col])
            next_position = get_next_position(current_position, best_action)
            print(q_tables[1][row, col], best_action, next_position)
            path.append(next_position)
            current_position = next_position
            steps += 1
        paths.append(path)
    return paths


# Affichage des trajectoires optimales pour chaque drone
optimal_paths = find_optimal_paths()
for i, path in enumerate(optimal_paths):
    print(f"Trajectoire optimale du drone {i + 1} :", path)

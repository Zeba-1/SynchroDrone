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
starts_position = [(3, 4), (5, 4), (1, 5)]  # Positions initiales des drones
goals_position = [(8, 1), (0,5), (5, 6)]  # Cible en bas à droite de la grille
q_tables = [np.zeros((grid_size, grid_size, 5)) for _ in range(num_drones)]  # Q-tables pour chaque drone

# Actions possibles (haut, bas, gauche, droite)
actions = {
    0: (-1, 0),  # Haut
    1: (1, 0),   # Bas
    2: (0, -1),  # Gauche
    3: (0, 1),   # Droite
    4: (0, 0)    # Reste sur place
}

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
    if state == goals_position[drone_id]:
        return 100  # Récompense pour atteindre l'objectif
    elif state in obstacles:
        return -80 # Pénalité pour collision avec un obstacle
    elif state in positions and positions[state] != drone_id:
        return -30 # Pénalité pour collision avec un autre drone
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
        states = starts_position[:]  # Positions initiales des drones
        total_rewards = [0] * num_drones
        positions = {pos: i for i, pos in enumerate(states)}  # Suivi des positions des drones

        for step in range(200): # Nombre maximum de pas par épisode
            for i in range(num_drones):
                state = states[i]

                if state == goals_position[i]:
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

        # Affichage des récompenses totales pour observer l'apprentissage
        print(f"Épisode {episode + 1}: Récompenses totales = {total_rewards}")

# Lancer l'apprentissage multi-agent sur 1000 épisodes
train_multi_agent(episodes=5000)

# Trajectoire optimale pour chaque drone
def find_optimal_paths():
    """Trouve et retourne les trajectoires optimales pour tous les drones."""
    paths = []
    for i in range(num_drones):
        current_position = starts_position[i]
        path = [current_position]
        steps = 0  # Compteur de pas
        while current_position != goals_position[i] and steps < 100:
            row, col = current_position
            best_action = np.argmax(q_tables[i][row, col])
            next_position = get_next_position(current_position, best_action)
            print(q_tables[i][row, col], best_action, next_position)
            path.append(next_position)
            current_position = next_position
            steps += 1
        paths.append(path)
    return paths


def display_grid(states):
    grid = np.full((grid_size, grid_size), '.', dtype=str)
    
    # Matplotlib pour visualisation graphique
    plt.imshow(grid == '.', cmap="gray", origin="upper")
    
    for goal in goals_position:
        plt.scatter(*goal[::-1], color='green', label="Objectif" if goal == goals_position[0] else "")
    for obs in obstacles:
        plt.scatter(*obs[::-1], color='red', label="Obstacle" if obs == obstacles[0] else "")
    for i, state in enumerate(states):
        plt.scatter(state[1], state[0], label=f"Drone {i + 1}", s=100)

    plt.legend(loc="upper right")
    plt.pause(0.5)
    plt.clf()

# Affichage des trajectoires optimales pour chaque drone
optimal_paths = find_optimal_paths()
# Uniformiser la taille des listes dans optimal_paths
max_length = max(len(path) for path in optimal_paths)
for path in optimal_paths:
    while len(path) < max_length:
        path.append(path[-1])  # Ajouter la dernière position jusqu'à atteindre la longueur maximale

for path in optimal_paths:
    print(path)

# Affichage des positions des drones à chaque étape
for step in range(max_length):
    current_states = [path[step] for path in optimal_paths]
    display_grid(current_states)

import numpy as np
import random

# Paramètres de la grille et du Q-learning
grid_size = 5
alpha = 0.1  # Taux d'apprentissage
gamma = 0.9  # Facteur de discount
epsilon = 0.1  # Taux d'exploration

# Définition de l'environnement : récompenses, obstacles, départ, objectif
goal_position = (4, 4)
start_position = (0, 0)
obstacles = [(1, 1), (1, 2), (2, 2), (3, 3)]

# Initialisation de la Q-table (table des valeurs Q)
q_table = np.zeros((grid_size, grid_size, 4))
print(q_table)
# Actions possibles (haut, bas, gauche, droite)
actions = {
    0: (-1, 0),  # Haut
    1: (1, 0),   # Bas
    2: (0, -1),  # Gauche
    3: (0, 1)    # Droite
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

def get_reward(state):
    """Fonction de récompense : +100 pour l'objectif, -100 pour un obstacle, -1 sinon."""
    if state == goal_position:
        return 100
    elif state in obstacles:
        return -100
    else:
        return -1

def choose_action(state):
    """Choisit l'action basée sur epsilon-greedy : exploration ou exploitation."""
    if random.uniform(0, 1) < epsilon:
        return random.choice(list(actions.keys()))  # Exploration
    else:
        row, col = state
        return np.argmax(q_table[row, col])  # Exploitation

def train_agent(episodes):
    """Entraîne l'agent sur un certain nombre d'épisodes."""
    for episode in range(episodes):
        state = start_position
        total_reward = 0

        while state != goal_position:
            # Choisir une action et obtenir le nouvel état
            action = choose_action(state)
            next_state = get_next_position(state, action)
            reward = get_reward(next_state)
            
            # Mettre à jour la Q-table avec la formule du Q-learning
            row, col = state
            next_row, next_col = next_state
            q_table[row, col, action] = q_table[row, col, action] + alpha * (
                reward + gamma * np.max(q_table[next_row, next_col]) - q_table[row, col, action]
            )
            
            # Passer à l'état suivant
            state = next_state
            total_reward += reward

        # Affichage de la récompense totale pour observer l'apprentissage
        print(f"Épisode {episode + 1}: Récompense totale = {total_reward}")

# Lancer l'apprentissage sur 1000 épisodes
train_agent(episodes=1000)

# Affichage de la Q-table après apprentissage
print("\nQ-table après apprentissage:")
print(q_table)

# Fonction pour suivre la trajectoire optimale
def find_optimal_path():
    """Trouve et retourne la trajectoire optimale en suivant la Q-table."""
    current_position = start_position
    path = [current_position]  # Liste pour stocker le chemin emprunté

    while current_position != goal_position:
        row, col = current_position
        # Choisir l'action ayant la plus haute valeur Q dans la Q-table
        best_action = np.argmax(q_table[row, col])
        # Obtenir la prochaine position en fonction de la meilleure action
        next_position = get_next_position(current_position, best_action)
        # Ajouter la nouvelle position au chemin
        path.append(next_position)
        # Mettre à jour la position actuelle
        current_position = next_position

    return path

# Affichage de la trajectoire optimale
optimal_path = find_optimal_path()
print("Trajectoire optimale pour atteindre la cible :", optimal_path)
"""
grid_length, grid_width = 10, 10
start_position = (0, 0)  # Départ en haut à gauche de la grille
goal_position = (6, 1)  # Cible en bas à droite de la grille
obstacles = {(0, 4), (0, 9), (0, 10),
            (1, 4), (1, 9), (1, 10),
            (2, 0), (2, 4), (2, 5), (2, 9), (2, 10),
            (3, 5), (3, 9), (3, 10),
            (4, 9), (4, 10),
            (5, 5),
            (6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5),
            (8, 0), (8, 2)}  # Ensemble d'obstacles
"""
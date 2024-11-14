import numpy as np
import random

# Paramètres de l'environnement 3D et du Q-learning
grid_length, grid_width, grid_height = 5, 5, 5
alpha = 0.1  # Taux d'apprentissage
gamma = 0.9  # Facteur de discount
epsilon = 0.1  # Taux d'exploration

# Définition de l'environnement : récompenses, obstacles, départ, objectif
goal_position = (4, 4, 4)  # Cible au coin opposé
start_position = (0, 0, 0)  # Départ à l'origine
obstacles = [(1, 1, 1), (2, 2, 2), (3, 3, 3)]  # Obstacles placés dans la grille

# Initialisation de la Q-table (table des valeurs Q)
q_table = np.zeros((grid_length, grid_width, grid_height, 6))  # 6 actions : avant, arrière, gauche, droite, haut, bas

# Actions possibles (avant, arrière, gauche, droite, haut, bas)
actions = {
    0: (0, 0, -1),  # Avant
    1: (0, 0, 1),   # Arrière
    2: (0, -1, 0),  # Gauche
    3: (0, 1, 0),   # Droite
    4: (-1, 0, 0),  # Haut
    5: (1, 0, 0)    # Bas
}

def get_next_position(state, action):
    """Calcul de la prochaine position en fonction de l'action choisie."""
    x, y, z = state
    dx, dy, dz = actions[action]
    x += dx
    y += dy
    z += dz
    # Vérifier les limites de la grille
    if x < 0 or x >= grid_length or y < 0 or y >= grid_width or z < 0 or z >= grid_height:
        return state  # Si hors grille, rester à la même position
    return (x, y, z)

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
        x, y, z = state
        return np.argmax(q_table[x, y, z])  # Exploitation

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
            x, y, z = state
            nx, ny, nz = next_state
            q_table[x, y, z, action] = q_table[x, y, z, action] + alpha * (
                reward + gamma * np.max(q_table[nx, ny, nz]) - q_table[x, y, z, action]
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
        x, y, z = current_position
        # Choisir l'action ayant la plus haute valeur Q dans la Q-table
        best_action = np.argmax(q_table[x, y, z])
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

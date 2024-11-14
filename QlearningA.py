import numpy as np
import random
import heapq

# Paramètres de l'environnement 2D et du Q-learning
grid_length, grid_width = 10, 10
start_position = (0, 0)  # Départ en haut à gauche de la grille
goal_position = (7, 1)  # Cible en bas à droite de la grille
obstacles = {(0, 4), (0, 8), (0, 9),
            (1, 4), (1, 8), (1, 9),
            (2, 0), (2, 4), (2, 5), (2, 8), (2, 9),
            (3, 5), (3, 8), (3, 9),
            (4, 8), (4, 9),
            (5, 5),
            (6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5),
            (8, 0), (8, 2)}  # Ensemble d'obstacles
# Q-learning parameters
alpha = 0.1 # Taux d'apprentissage
gamma = 0.9  # Facteur de discount
epsilon = 0.1  # Taux d'exploration
actions = {
    0: (-1, 0),  # Haut
    1: (1, 0),   # Bas
    2: (0, -1),  # Gauche
    3: (0, 1)    # Droite
}

# Initialisation de la Q-table (table des valeurs Q)
q_table = np.zeros((grid_length, grid_width, len(actions)))

def is_within_bounds(position):
    """Vérifie si la position est dans les limites de la grille."""
    x, y = position
    return 0 <= x < grid_length and 0 <= y < grid_width

def get_next_position(state, action):
    """Calcul de la prochaine position en fonction de l'action choisie."""
    x, y = state
    dx, dy = actions[action]
    next_state = (x + dx, y + dy)
    if is_within_bounds(next_state) and next_state not in obstacles:
        return next_state
    return state  # Reste à la même position si hors grille ou obstacle

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
        x, y = state
        return np.argmax(q_table[x, y])  # Exploitation

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
            x, y = state
            nx, ny = next_state
            q_table[x, y, action] = q_table[x, y, action] + alpha * (
                reward + gamma * np.max(q_table[nx, ny]) - q_table[x, y, action]
            )
            
            # Passer à l'état suivant
            state = next_state
            total_reward += reward

        # Affichage de la récompense totale pour observer l'apprentissage
        print(f"Épisode {episode + 1}: Récompense totale = {total_reward}")

# Entraînement avec Q-learning
train_agent(episodes=10000)

# Affichage de la Q-table après apprentissage
print("\nQ-table après apprentissage:")
print(q_table)

# Fonction pour suivre la trajectoire optimale
def find_optimal_path():
    """Trouve et retourne la trajectoire optimale en suivant la Q-table."""
    current_position = start_position
    path = [current_position]  # Liste pour stocker le chemin emprunté

    while current_position != goal_position:
        x, y = current_position
        # Choisir l'action ayant la plus haute valeur Q dans la Q-table
        best_action = np.argmax(q_table[x, y])
        # Obtenir la prochaine position en fonction de la meilleure action
        next_position = get_next_position(current_position, best_action)
        # Ajouter la nouvelle position au chemin
        path.append(next_position)
        # Mettre à jour la position actuelle
        current_position = next_position

    return path

# Calculer le chemin optimal avec Q-learning
optimal_path_q_learning = find_optimal_path()
print("Trajectoire optimale trouvée par Q-learning :", optimal_path_q_learning)


# Implémentation de l'algorithme A* pour comparaison
def heuristic(a, b):
    """Fonction heuristique utilisant la distance de Manhattan en 2D."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(position):
    """Retourne les voisins valides (dans la grille et sans obstacles) d'une position donnée."""
    neighbors = []
    for action in actions.values():
        x, y = position[0] + action[0], position[1] + action[1]
        new_position = (x, y)
        if is_within_bounds(new_position) and new_position not in obstacles:
            neighbors.append(new_position)
    return neighbors

def a_star(start, goal):
    """Algorithme A* pour trouver le chemin optimal de start à goal dans une grille 2D."""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None

# Calculer le chemin optimal avec A*
optimal_path_a_star = a_star(start_position, goal_position)
print("Trajectoire optimale trouvée par A* :", optimal_path_a_star)
print("isEqual ? ", optimal_path_q_learning == optimal_path_a_star)
print(optimal_path_q_learning)
print(optimal_path_a_star)
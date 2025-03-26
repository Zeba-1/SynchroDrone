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

actions = {
    0: (-1, 0),  # Haut
    1: (1, 0),   # Bas
    2: (0, -1),  # Gauche
    3: (0, 1)    # Droite
}

def is_within_bounds(position):
    """Vérifie si une position est dans la grille."""
    x, y = position
    return 0 <= x < grid_length and 0 <= y < grid_width

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
print(optimal_path_a_star)
import numpy as np
import random
import heapq

class A_star:
    def __init__(self):
        # Paramètres de l'environnement 2D et du Q-learning
        self.grid_length, self.grid_width = 10, 10
        self.start_position = (1, 1)  # Départ en haut à gauche de la grille
        self.goal_position = (3, 3)  # Cible en bas à droite de la grille
        self.obstacles = {(0, 0), (0, 1), (0, 2),
                          (1, 0), (1, 2),
                          (2, 0), (2, 2), (2, 3),
                          (3, 0),
                          (4, 0), (4, 1), (4, 2), (4, 3)}  # Ensemble d'obstacles
        # Q-learning parameters
        self.alpha = 0.1 # Taux d'apprentissage
        self.gamma = 0.9  # Facteur de discount
        self.epsilon = 0.1  # Taux d'exploration
        self.actions = {
            0: (1, 0),  # Haut
            1: (-1, 0),   # Bas
            2: (0, -1),  # Gauche
            3: (0, 1)    # Droite
        }

    def is_within_bounds(self, position):
        """Vérifie si la position est dans les limites de la grille."""
        x, y = position
        return 0 <= x < self.grid_length and 0 <= y < self.grid_width

    def heuristic(self, a, b):
        """Fonction heuristique utilisant la distance de Manhattan en 2D."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, position):
        """Retourne les voisins valides (dans la grille et sans obstacles) d'une position donnée."""
        neighbors = []
        for action in self.actions.values():
            x, y = position[0] + action[0], position[1] + action[1]
            new_position = (x, y)
            if self.is_within_bounds(new_position) and new_position not in self.obstacles:
                neighbors.append(new_position)
        return neighbors

    def find_optimal_path(self):
        """Algorithme A* pour trouver le chemin optimal de start à goal dans une grille 2D."""
        start = self.start_position
        goal = self.goal_position
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
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
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None
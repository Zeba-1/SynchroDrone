from pathlib import Path
import numpy as np
import random
import heapq

class A_star:
    def __init__(self):
        # Paramètres de l'environnement 2D et du Q-learning
        self.grid_length, self.grid_width = 10, 10
        self.start_position = []  # Départ en haut à gauche de la grille
        self.goal_position = [] # Cible en bas à droite de la grille
        self.obstacles = []  # Ensemble d'obstacles

        script_dir = Path(__file__).resolve().parent
        file_path = script_dir / "setup.txt"
        with open(file_path, 'r') as f:
            start_line = f.readline().strip()
            start_positions = start_line.split('-')
            for pos in start_positions:
                if pos == '': continue
                x, y = pos.split('.')
                self.start_position.append((int(x), int(y)))

            end_line = f.readline().strip()
            end_positions = end_line.split('-')
            for pos in end_positions:
                if pos == '': continue
                x, y = pos.split('.')
                self.goal_position.append((int(x), int(y)))

            wall_line = f.readline().strip()
            wall_positions = wall_line.split('-')
            for pos in wall_positions:
                if pos == '': continue
                x, y = pos.split('.')
                self.obstacles.append((int(x), int(y)))

        print(self.start_position)
        print(self.goal_position)
        print(self.obstacles)

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

    def find_optimal_paths(self):
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
                return [path]
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None
    
if __name__ == "__main__":
    astart = A_star()
from pathlib import Path
import numpy as np
import random
import matplotlib.pyplot as plt
import time

class QLrearning():
    def __init__(self, num_drones):
        # Paramètres de la grille et du Q-learning
        self.num_drones = num_drones
        self.grid_size = 14
        self.alpha = 0.1  # Taux d'apprentissage
        self.gamma = 0.9  # Facteur de discount
        self.epsilon = 0.1  # Taux d'exploration

        self.grid_length, self.grid_width = 14, 14
        self.obstacles = []
        self.starts_position = []  # Positions initiales des drones
        self.goals_position = []  # Cible en bas à droite de la grille

        script_dir = Path(__file__).resolve().parent
        file_path = "/home/user/crazyflie_mapping_demo/ros2_ws/src/SynchroDrone/ros_package/crazyflie_ros2_multiranger/crazyflie_ros2_path_finder_controller/crazyflie_ros2_path_finder_controller/algorithms/setup.txt"
        with open(file_path, 'r') as f:
            start_line = f.readline().strip()
            start_positions = start_line.split('-')
            for pos in start_positions:
                if pos == '': continue
                x, y = pos.split('.')
                self.starts_position.append((int(x), int(y)))

            end_line = f.readline().strip()
            end_positions = end_line.split('-')
            for pos in end_positions:
                if pos == '': continue
                x, y = pos.split('.')
                self.goals_position.append((int(x), int(y)))

            wall_line = f.readline().strip()
            wall_positions = wall_line.split('-')
            for pos in wall_positions:
                if pos == '': continue
                x, y = pos.split('.')
                self.obstacles.append((int(x), int(y)))

        print(self.starts_position)
        print(self.goals_position)
        print(self.obstacles)

        ## Grid division by two
        # grid_size = grid_size * 2
        # goals_position = [(goal[0] * 2, goal[1] * 2) for goal in goals_position]
        # starts_position = [(start[0] * 2, start[1] * 2) for start in starts_position]
        # new_obstacles = []
        # for obs in obstacles:
        #     new_obstacles.append((obs[0] * 2, obs[1] * 2))
        #     new_obstacles.append((obs[0] * 2 + 1, obs[1] * 2))
        #     new_obstacles.append((obs[0] * 2, obs[1] * 2 + 1))
        #     new_obstacles.append((obs[0] * 2 + 1, obs[1] * 2 + 1))
        # obstacles = new_obstacles

        # Drones
        self.q_tables = [np.zeros((self.grid_size, self.grid_size, 9)) for _ in range(self.num_drones)]  # Q-tables pour chaque drone

        # Actions possibles (haut, bas, gauche, droite)
        self.actions = {
            0: (-1, 0),   # Haut
            1: (1, 0),    # Bas
            2: (0, -1),   # Gauche
            3: (0, 1),    # Droite
            4: (-1, -1),  # Diagonale haut-gauche
            5: (-1, 1),   # Diagonale haut-droite
            6: (1, -1),   # Diagonale bas-gauche
            7: (1, 1),    # Diagonale bas-droite
            8: (0, 0)     # Reste sur place
        }

    def get_next_position(self, state, action):
        """Calcul de la prochaine position en fonction de l'action choisie."""
        row, col = state
        row += self.actions[action][0]
        col += self.actions[action][1]
        # Vérifier les limites de la grille
        if row < 0 or row >= self.grid_size or col < 0 or col >= self.grid_size:
            return state  # Si hors grille, rester à la même position
        return (row, col)

    def get_reward(self, state, drone_id, positions, action):
        """Fonction de récompense spécifique pour chaque drone."""
        if action in [4, 5, 6, 7]:
            gauche = (state[0], state[1] - 1)
            droite = (state[0], state[1] + 1)
            haut = (state[0] - 1, state[1])
            bas = (state[0] + 1, state[1])

            if action == 4 and (droite in self.obstacles or bas in self.obstacles):
                    return -80
            elif action == 5 and (gauche in self.obstacles or bas in self.obstacles):
                    return -80
            elif action == 6 and (droite in self.obstacles or haut in self.obstacles):
                    return -80
            elif action == 7 and (gauche in self.obstacles or haut in self.obstacles):
                    return -80
            
        if state == self.goals_position[drone_id]:
            return 100  # Récompense pour atteindre l'objectif
        elif state in self.obstacles:
            return -80 # Pénalité pour collision avec un obstacle
        elif state in positions and positions[state] != drone_id:
            return -30 # Pénalité pour collision avec un autre drone
        elif action in [4, 5, 6, 7]:
            return -1.5
        else:
            return -1  # Pénalité par défaut pour chaque mouvement

    def choose_action(self, state, q_table):
        """Choisit l'action basée sur epsilon-greedy : exploration ou exploitation."""
        if random.uniform(0, 1) < self.epsilon:
            return random.choice(list(self.actions.keys()))  # Exploration
        else:
            row, col = state
            return np.argmax(q_table[row, col])  # Exploitation

    def train_multi_agent(self, episodes):
        """Entraîne plusieurs drones sur un certain nombre d'épisodes."""
        for episode in range(episodes):
            states = self.starts_position[:]  # Positions initiales des drones
            total_rewards = [0] * self.num_drones
            positions = {pos: i for i, pos in enumerate(states)}  # Suivi des positions des drones

            for step in range(200): # Nombre maximum de pas par épisode
                for i in range(self.num_drones):
                    state = states[i]

                    if state == self.goals_position[i]:
                        continue  # Si le drone est déjà arrivé, il ne bouge plus

                    # Choisir une action et obtenir le nouvel état
                    action = self.choose_action(state, self.q_tables[i])
                    next_state = self.get_next_position(state, action)

                    # Calculer la récompense pour ce drone
                    reward = self.get_reward(next_state, i, positions, action)

                    # Mettre à jour la Q-table avec la formule du Q-learning
                    row, col = state
                    next_row, next_col = next_state
                    self.q_tables[i][row, col, action] = self.q_tables[i][row, col, action] + self.alpha * (
                        reward + self.gamma * np.max(self.q_tables[i][next_row, next_col]) - self.q_tables[i][row, col, action]
                    )

                    # Mettre à jour l'état du drone
                    if state in positions:
                        positions.pop(state)  # Retirer l'ancienne position
                    if next_state not in positions:
                        positions[next_state] = i  # Ajouter la nouvelle position
                    states[i] = next_state
                    total_rewards[i] += reward

            # Affichage des récompenses totales pour observer l'apprentissage
            # print(f"Épisode {episode + 1}: Récompenses totales = {total_rewards}")

    # Trajectoire optimale pour chaque drone
    def find_optimal_paths(self):
        """Trouve et retourne les trajectoires optimales pour tous les drones."""
        self.train_multi_agent(episodes=10000)
        paths = []
        for i in range(self.num_drones):
            current_position = self.starts_position[i]
            path = [current_position]
            steps = 0  # Compteur de pas
            while current_position != self.goals_position[i] and steps < 100:
                row, col = current_position
                best_action = np.argmax(self.q_tables[i][row, col])
                next_position = self.get_next_position(current_position, best_action)
                print(self.q_tables[i][row, col], best_action, next_position)
                path.append(next_position)
                current_position = next_position
                steps += 1
            paths.append(path)
        return paths
    

if __name__ == "__main__":
     ql = QLrearning(num_drones=2)

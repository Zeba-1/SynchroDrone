import math
from enum import Enum
from .astar import A_star
from .qlearning_multi_agent import QLrearning


class PathFinder():
    def __init__(self, nb_drones, logger):
        self.nb_drones = nb_drones
        self.logger = logger
        # if nb_drones == 1:
        #     self.algo = A_star()
        # elif nb_drones >= 2:
        self.algo = QLrearning(nb_drones)

        self.path_goal = [0] * nb_drones

        self.finished = [False] * nb_drones

        self.path = self.algo.find_optimal_paths()
        print("=== debug ===> optimal path:" + str(self.path))

    # Helper function
    def grid_to_world(self, grid):
        return grid[0] - 0.5, grid[1] + 0.5

    def get_heading(self, x_pos, y_pos, x_goal, y_goal):
        return math.atan2(y_goal - y_pos, x_goal - x_pos)
    
    def correct_heading(self, current_heading, desired_heading):
        if desired_heading - current_heading > 0:
            return 0.5
        else:
            return -0.5
    
    def have_reached_goal(self, x_pos, y_pos, x_goal, y_goal):
        return math.sqrt((x_goal - x_pos)**2 + (y_goal - y_pos)**2) < 0.1

    # Wall following State machine
    def path_finder(self, positions):
        results = [(0., 0., 0.)] * self.nb_drones
        for i in range(self.nb_drones):
            x_pos, y_pos, current_heading = positions[i]

            if self.finished[i]:
                results[i] = (0., 0., 0.)
                break
            
            x_goal, y_goal = self.grid_to_world(self.path[i][self.path_goal[i]])
            desired_heading = self.get_heading(x_pos, y_pos, x_goal, y_goal)

            # if i == 0:
            #     self.logger.info(f"== DEBUG ==> Drone {i} - x_pos: {x_pos}, y_pos: {y_pos}, x_goal: {x_goal}, y_goal: {y_goal}, current_heading: {current_heading}, desired_heading: {desired_heading}")

            if self.have_reached_goal(x_pos, y_pos, x_goal, y_goal):
                self.path_goal[i] += 1
                if self.path_goal[i] >= len(self.path[i]):
                    self.finished[i] = True
                    print(f"== DEBUG ==> {i} FINISHED")
                    results[i] = (0., 0., 0.)
                    break
                print(f"== DEBUG ==> Nex goal for {i}: {self.path_goal[i]} = {self.path[i][self.path_goal[i]]}")
                

            correct_heading = self.correct_heading(current_heading, desired_heading)

            if abs(desired_heading - current_heading) < 0.1:
                results[i] = (0.5, 0., correct_heading)
            else:
                results[i] = (0., 0., correct_heading)

        return results

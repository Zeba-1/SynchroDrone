import math
from enum import Enum
from .astar import A_star


class PathFinder():
    def __init__(self, algo="astar"):
        if algo == "astar":
            self.algo = A_star()

        self.path_goal = 0

        self.finished = False

        self.path = self.algo.find_optimal_path()
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
    def path_finder(self, x_pos, y_pos, current_heading):
        if self.finished:
            return 0., 0., 0.
        
        x_goal, y_goal = self.grid_to_world(self.path[self.path_goal])
        desired_heading = self.get_heading(x_pos, y_pos, x_goal, y_goal)

        if self.have_reached_goal(x_pos, y_pos, x_goal, y_goal):
            self.path_goal += 1
            if self.path_goal >= len(self.path):
                self.finished = True
                print(f"== DEBUG ==> FINISHED")
                return 0., 0., 0.
            print(f"== DEBUG ==> Nex goal: {self.path_goal} = {self.path[self.path_goal]}")
            

        correct_heading = self.correct_heading(current_heading, desired_heading)

        if abs(desired_heading - current_heading) < 0.1:
            return 1., 0., correct_heading
        else:
            return 0., 0., correct_heading

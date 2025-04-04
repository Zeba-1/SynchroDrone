import math
from enum import Enum
import time
from .astar import A_star
from .qlearning_multi_agent import QLrearning
from .report import Report

TIME_LIMIT = 300 # 5 minutes = max time for the simulation

class PathFinder():
    def __init__(self, nb_drones, logger):
        self.nb_drones = nb_drones
        self.logger = logger
        self.algo = A_star()
        #self.algo = QLrearning(nb_drones)
        self.path_goal = [0] * nb_drones
        self.finished = [False] * nb_drones

        self.report = Report(nb_drones, self.algo.name())
        self.report.start()

        start_thinking = time.time()
        self.path = self.algo.find_optimal_paths()
        self.report.set_algo_calculation(time.time() - start_thinking)
        self.logger.info("=== DEBUG ===> optimal path:" + str(self.path) + "in " + str(time.time() - start_thinking) + "s")

        for i in range(nb_drones):
            if self.path[i] is not None:
                self.report.have_found_path(i)
            else:
                self.finished[i] = True

        self.last_time_logged = 0

    # Helper function
    def grid_to_world(self, grid):
        return grid[0] - 0.5, grid[1] + 0.5
    
    def world_to_grid(self, x, y):
        return int(x + 0.5), int(y - 0.5)

    def get_heading(self, x_pos, y_pos, x_goal, y_goal):
        return math.atan2(y_goal - y_pos, x_goal - x_pos)
    
    def normalize_angle(self, angle):
        """Normalize the angle to be within the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def correct_heading(self, current_heading, desired_heading):
        current_heading = self.normalize_angle(current_heading)
        desired_heading = self.normalize_angle(desired_heading)
        heading_diff = self.normalize_angle(desired_heading - current_heading)
        
        if heading_diff > 0:
            return 0.5
        else:
            return -0.5
    
    def have_reached_goal(self, x_pos, y_pos, x_goal, y_goal):
        return math.sqrt((x_goal - x_pos)**2 + (y_goal - y_pos)**2) < 0.1

    # Wall following State machine
    def path_finder(self, positions):
        results = [(0., 0., 0.)] * self.nb_drones

        self.isColliding(positions)

        for i in range(self.nb_drones):
            x_pos, y_pos, current_heading = positions[i]

            if self.finished[i] or self.path[i] is None:
                results[i] = (0., 0., 0.)
                continue
            
            x_goal, y_goal = self.grid_to_world(self.path[i][self.path_goal[i]])
            desired_heading = self.get_heading(x_pos, y_pos, x_goal, y_goal)

            if self.have_reached_goal(x_pos, y_pos, x_goal, y_goal):
                self.report.add_steps(i)
                self.path_goal[i] += 1
                if self.path_goal[i] >= len(self.path[i]):
                    self.report.add_reached_goal(i)
                    self.finished[i] = True
                    self.logger.info(f"== DEBUG ==> {i} FINISHED")
                    results[i] = (0., 0., 0.)

                    if all(self.finished):
                        self.report.end()
                        self.report.generate_report()

                    continue
                self.logger.info(f"== DEBUG ==> Nex goal for {i}: {self.path_goal[i]} = {self.path[i][self.path_goal[i]]}")
                

            correct_heading = self.correct_heading(current_heading, desired_heading)

            if abs(desired_heading - current_heading) < 0.2:
                results[i] = (0.5, 0., correct_heading)
            else:
                results[i] = (0., 0., correct_heading)

            if self. last_time_logged + 1 < time.time():
                self.logger.info(f"== DEBUG ==> Drone {i} - x: {x_pos} - y: {y_pos} - heading: {current_heading} => {desired_heading} - goal: {x_goal}, {y_goal} - result: {results[i]}")
                self.last_time_logged = time.time()

            if (self.report.start_time + TIME_LIMIT) < time.time():
                self.report.end()
                self.report.generate_report()
                self.finished = [True] * self.nb_drones
                self.logger.info("== DEBUG ==> TIME LIMIT REACHED - END OF SIMULATION")

        return results
    
    def isColliding(self, positions):
        for i in range(self.nb_drones):
            x_pos, y_pos, current_heading = positions[i]

            for j in range(self.nb_drones):
                if i == j:
                    continue
                x_pos_j, y_pos_j, current_heading_j = positions[j]
                if math.sqrt((x_pos - x_pos_j)**2 + (y_pos - y_pos_j)**2) < 0.3:
                    self.report.add_collision(i, self.world_to_grid(x_pos, y_pos))
                    return True
                
            for j in range(len(self.algo.obstacles)):
                x_obs, y_obs = self.algo.obstacles[j]
                if math.sqrt((x_pos - x_obs)**2 + (y_pos - y_obs)**2) < 0.25:
                    self.report.add_collision(i, self.world_to_grid(x_pos, y_pos))
                    return True
        return False

import time

class Report():
    def __init__(self, nbDrone, algo):
        self.nbDrone = nbDrone
        self.algo = algo

        self.collisions = 0
        self.path_steps = [0] * self.nbDrone
        self.reached_goals = [False] * self.nbDrone

    def add_steps(self, drone):
        self.path_steps[drone] += 1

    def add_collision(self):
        self.collisions += 1

    def add_reached_goal(self, drone):
        self.reached_goals[drone] = True

    def start(self):
        self.start_time = time.time()

    def end(self):
        self.end_time = time.time()

    def generate_report(self):
        print(f"=== Report ===")
        print(f"Total time: {self.end_time - self.start_time}")
        print(f"Collisions: {self.collisions}")
        for i in range(self.nbDrone):
            print(f"Drone {i} - Steps: {self.path_steps[i]} - Reached goal: {self.reached_goals[i]}")
        print(f"==============")
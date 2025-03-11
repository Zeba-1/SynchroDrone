import json
import time

class Report():
    def __init__(self, nbDrone, algo):
        self.nbDrone = nbDrone
        self.algo = algo

        self.collisions = 0
        self.path_steps = [0] * self.nbDrone
        self.path_found = [False] * self.nbDrone
        self.reached_goals = [False] * self.nbDrone

    def add_steps(self, drone):
        self.path_steps[drone] += 1

    def add_collision(self):
        self.collisions += 1

    def have_found_path(self, drone):
        self.path_found[drone] = True

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
            if self.path_found[i]:
                print(f"Drone {i} - Steps: {self.path_steps[i]} - Reached goal: {self.reached_goals[i]}")
            else :
                print(f"Drone {i} - No path found")
        print(f"==============")

        report_data = {
            "total_time": self.end_time - self.start_time,
            "collisions": self.collisions,
            "drones": []
        }

        for i in range(self.nbDrone):
            drone_data = {
            "drone_id": i,
            "steps": self.path_steps[i],
            "reached_goal": self.reached_goals[i],
            "path_found": self.path_found[i]
            }
            report_data["drones"].append(drone_data)

        file = f"report_{self.algo}_{self.nbDrone}_{time.time()}.json"
        with open(file, "w") as report_file:
            json.dump(report_data, report_file, indent=4)
        
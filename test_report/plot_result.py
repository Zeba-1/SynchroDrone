import os
import pandas as pd
import json

import matplotlib.pyplot as plt

# Chemins des dossiers contenant les rapports
BASE_DIR = "/home/user/crazyflie_mapping_demo/ros2_ws/src/SynchroDrone/test_report"
FOLDERS = ["one", "two", "three"]
ALGORITHMS = ["Q-Learning", "A*"]
def load_reports():
    """Charge les rapports de vol depuis les dossiers."""
    data = []
    for folder in FOLDERS:
        folder_path = os.path.join(BASE_DIR, folder)
        for algo in ALGORITHMS:
            file_path = os.path.join(folder_path, f"report_{algo}_*.json")
            for file in os.listdir(folder_path):
                if file.startswith(f"report_{algo}_") and file.endswith(".json"):
                    full_path = os.path.join(folder_path, file)
                    with open(full_path, 'r') as f:
                        report = json.load(f)
                        for drone in report["drones"]:
                            data.append({
                                "Algorithm": report["algo"],
                                "Scenario": folder,
                                "Difficulty": file.split("_")[-1].replace(".json", ""),
                                "FlightTime": report["total_time"],
                                "DroneID": drone["drone_id"],
                                "Steps": drone["steps"],
                                "ReachedGoal": drone["reached_goal"],
                                "PathFound": drone["path_found"],
                                "Collisions": drone["nb_collisions"],
                                "AlgoTime": report["algo_time"]
                            })
    return pd.DataFrame(data)

def plot_results(data):
    scenario_order = ["one", "two", "three"]
    difficulty_order = ["simple", "mid", "hard"]

    # graphique en barres pour les temps de vol moyens par difficulté et algorithme
    avg_flight_time = data.groupby(["Difficulty", "Algorithm"])["FlightTime"].mean().unstack()

    avg_flight_time = avg_flight_time.reindex(difficulty_order)

    avg_flight_time.plot(kind="bar", figsize=(10, 6))
    plt.xlabel("Difficulté")
    plt.ylabel("Temps de vol moyen (s)")
    plt.title("Temps de vol moyen en fonction de l'algorithme pour chaque difficulté")
    plt.legend(title="Algorithme")
    plt.grid(axis="y")
    plt.show()

    # graphique en barres pour le nombre moyen de collisions par difficulté et algorithme
    avg_collisions = data.groupby(["Scenario", "Algorithm"])["Collisions"].mean().unstack()

    avg_collisions = avg_collisions.reindex(scenario_order)

    avg_collisions.plot(kind="bar", figsize=(10, 6))
    plt.xlabel("Scenario")
    plt.ylabel("Nombre moyen de collisions")
    plt.title("Nombre moyen de collisions en fonction de l'algorithme pour les différents nombre de drones")
    plt.legend(title="Algorithme")
    plt.grid(axis="y")
    plt.show()

    # graphique en barres pour le nombre moyen de pas par drone par difficulté et algorithme
    avg_steps = data.groupby(["Difficulty", "Algorithm"])["Steps"].mean().unstack()

    avg_steps = avg_steps.reindex(difficulty_order)

    avg_steps.plot(kind="bar", figsize=(10, 6))
    plt.xlabel("Difficulté")
    plt.ylabel("Nombre moyen de pas par drone")
    plt.title("Nombre moyen de pas par drone en fonction de l'algorithme pour chaque difficulté")
    plt.legend(title="Algorithme")
    plt.grid(axis="y")
    plt.show()

    # graphique en barres pour l'évolution du nombre de pas en fonction du scénario et de l'algorithme
    steps_evolution = data.groupby(["Scenario", "Algorithm"])["Steps"].mean().unstack()

    steps_evolution = steps_evolution.reindex(scenario_order)

    steps_evolution.plot(kind="bar", figsize=(10, 6))
    plt.xlabel("Scenario")
    plt.ylabel("Nombre moyen de pas")
    plt.title("Évolution du nombre de pas en fonction du scénario et de l'algorithme")
    plt.legend(title="Algorithme")
    plt.grid(axis="y")
    plt.show()

    # Tableau pour le temps moyen pris par l'algorithme pour trouver une réponse
    avg_algo_time = data.groupby(["Difficulty", "Algorithm"])["AlgoTime"].mean().unstack()
    avg_algo_time = avg_algo_time.reindex(difficulty_order)
    print("Temps moyen pour trouver une réponse (par difficulté et algorithme):")
    print(avg_algo_time)
    print("\n")

    # Tableau pour comparer les algorithmes en fonction du nombre de drones
    avg_algo_time_by_drones = data.groupby(["Scenario", "Algorithm"])["AlgoTime"].mean().unstack()
    avg_algo_time_by_drones = avg_algo_time_by_drones.reindex(scenario_order)
    print("Comparaison des algorithmes en fonction du nombre de drones (par scénario):")
    print(avg_algo_time_by_drones)
    print("\n")

def main():
    # Charger les rapports
    data = load_reports()

    # Vérifier les colonnes attendues
    if not data.empty and 'FlightTime' in data.columns:
        plot_results(data)
    else:
        print("Les rapports ne contiennent pas les colonnes attendues.")

if __name__ == "__main__":
    main()
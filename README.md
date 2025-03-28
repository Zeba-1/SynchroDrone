# Documentation d'Installation pour le Projet Crazyflie Mapping Demo

## Prérequis

- Une machine avec Ubuntu 22.04 en architecture x86.
- Des Crazyflies 2.1.
- Une Crazyradio PA.

## Étapes d'Installation

### Installer ROS 2 Humble

Suivez les instructions d'installation de ROS 2 Humble disponibles sur le site officiel : [ROS 2 Humble Installation](https://docs.ros.org/en/humble/index.html).

### Installer Gazebo Harmonic

Suivez les instructions d'installation de Gazebo Harmonic disponibles sur le site officiel : [Gazebo Harmonic Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/).

### Créer les Répertoires de Travail

Ouvrez un terminal et exécutez les commandes suivantes pour créer les répertoires de travail :

```bash
mkdir ~/crazyflie_mapping_demo
cd crazyflie_mapping_demo
mkdir simulation_ws
mkdir ros2_ws
cd ros2_ws
mkdir src
```

### Cloner les Dépôts Git

#### Cloner le dépôt `crazyflie-simulation`

```bash
cd ~/crazyflie_mapping_demo/simulation_ws
git clone https://github.com/bitcraze/crazyflie-simulation.git
```

#### Cloner les dépôts `SynchroDrone` et `crazyswarm2`

```bash
cd ~/crazyflie_mapping_demo/ros2_ws/src
git clone https://github.com/Zeba-1/SynchroDrone.git
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
```

### Installer les Dépendances

Installez les dépendances nécessaires en exécutant les commandes suivantes :

```bash
sudo apt-get install libboost-program-options-dev libusb-1.0-0-dev python3-colcon-common-extensions
sudo apt-get install ros-humble-tf-transformations
sudo apt-get install ros-humble-ros-gzharmonic
pip3 install cflib transform3D
```

### Construire le Workspace ROS 2

```bash
cd ~/crazyflie_mapping_demo/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
```

### Configurer la Crazyradio

Suivez les instructions de configuration de la Crazyradio disponibles sur le site officiel : [Crazyradio Configuration](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/).

### Configurer le Système Lighthouse

Suivez les instructions de configuration du système Lighthouse disponibles sur le site officiel : [Lighthouse Configuration](https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/).

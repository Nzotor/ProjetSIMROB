# 🚁 Tello ROS2 Simulation – Projet SIMROB

## 📌 Description

Ce projet a été réalisé dans le cadre du module **SIMROB**.

Il propose une simulation complète d’un drone **Tello** sous **ROS 2**, utilisant **Gazebo** comme environnement physique et visuel.

Le système permet :

- ✅ Le pilotage manuel (clavier / joystick)
- ✅ L’exécution de trajectoires prédéfinies
- ✅ Un mode pilote automatique
- ✅ L’interaction avec un environnement simulé (anneaux, arène)
- ✅ Une architecture compatible multi-drones

---

## 🏗️ Architecture

Le projet repose sur une architecture ROS2 modulaire composée de :

- `tello_description` → Description URDF du drone  
- `tello_gazebo` → Plugin Gazebo et mondes  
- `tello_driver` → Nœuds de contrôle (clavier, joystick, autopilot)  
- `tello_msgs` → Messages et services personnalisés  

### 🔄 Flux principal

tello_keyboard / tello_joy
↓
cmd_vel
↓
TelloPlugin (Gazebo)
↓
Simulation physique
↓
model_states


Cette architecture permet une séparation claire entre :

- 🔹 La commande utilisateur  
- 🔹 La dynamique du drone  
- 🔹 La simulation physique  
- 🔹 Le retour d’état  

---

## 🛠️ Technologies utilisées

- ROS 2
- Gazebo
- Python (rclpy)
- C++ (rclcpp)
- URDF
- geometry_msgs / sensor_msgs
- gazebo_ros
- tello_msgs (services personnalisés)

---

## 📦 Dépendances

### ROS 2

- rclpy  
- rclcpp  
- geometry_msgs  
- sensor_msgs  
- std_msgs  
- gazebo_ros  
- joy  

### Système

```bash
sudo apt install gazebo11 libgazebo11 libgazebo11-dev
sudo apt install libasio-dev
sudo apt install ros-<distro>-gazebo-ros-pkgs
sudo apt install ros-<distro>-cv-bridge
pip3 install transformations
Remplacer <distro> par votre distribution ROS2 (ex: galactic, humble).

🚀 Installation
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/Nzotor/ProjetSIMROB.git
cd ..
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
▶️ Lancement de la simulation
ros2 launch tello_gazebo simple_launch.py
Si nécessaire :

export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh
🎮 Commandes clavier
Touche	Action
↑ ↓ ← →	Déplacement
t	Takeoff
l	Land
a	Trajectoire carré
c	Trajectoire cercle
h	Trajectoire huit
k	Activer anneau
g	Activer autopilot
space	Stop
q	Quitter
🧠 Services utilisés
/drone1/tello_action
Type : tello_msgs/TelloAction

Permet :

takeoff

land

commandes rc

Exemple :

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"

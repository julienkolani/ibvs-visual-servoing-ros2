# IBVS Visual Servoing ROS2

Implémentation de l'asservissement visuel basé image (IBVS) sous ROS2.

## Présentation

L'IBVS est une technique de commande robotique qui calcule les vitesses de la caméra directement depuis l'erreur sur les primitives visuelles dans l'image, sans reconstruction 3D explicite. Configuration eye-in-hand (EIH).

## Principe

- Extraction des primitives visuelles dans l'image courante
- Calcul de l'erreur par rapport à l'image de référence
- Loi de commande via la matrice d'interaction (Jacobien image)
- Publication des vitesses sur le topic ROS2

## Stack technique

- Python, ROS2, OpenCV, NumPy

## Installation

```bash
source /opt/ros/humble/setup.bash
colcon build && source install/setup.bash
ros2 run ibvs_package ibvs_node
```

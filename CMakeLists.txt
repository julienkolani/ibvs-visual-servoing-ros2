cmake_minimum_required(VERSION 3.5)
project(turtlebot3_bird_eye_view_control)

# Obligatoire pour les packages Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installe le package Python
ament_python_install_package(${PROJECT_NAME})

# Installe les scripts exécutables
install(
  PROGRAMS
    turtlebot3_bird_eye_view_control/turtlebot3_bird_eye_view_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(lorandi_assignament_2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installe les scripts Python comme exécutables ROS 2
# → accessible via `ros2 run lorandi_assignament_2 <nom_node>`
install(PROGRAMS
  scripts/tag_detector_node.py
  scripts/cube_swapper_node.py
  scripts/color_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installe les launch files
# → accessible via `ros2 launch lorandi_assignament_2 <fichier>`
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Installe les fichiers de config (yaml)
# → accessible via FindPackageShare("lorandi_assignament_2")
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

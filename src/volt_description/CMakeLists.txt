cmake_minimum_required(VERSION 3.8)
project(volt_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config description launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sit_pose.py
    scripts/stand_up.py
    scripts/volt_control_gui.py
    scripts/volt_gait_controller.py
    scripts/volt_kinematics.py
    scripts/volt_motion_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

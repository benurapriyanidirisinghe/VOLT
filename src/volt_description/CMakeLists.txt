cmake_minimum_required(VERSION 3.8)
project(volt_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

enable_testing()

install(
  DIRECTORY config description emotes launch meshes moveit2 rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES EMOTES.md SERVO_CALIBRATION.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sit_pose.py
    scripts/stand_up.py
    scripts/volt_control_gui.py
    scripts/volt_emote_player.py
    scripts/volt_gait_controller.py
    scripts/volt_joint_command_router.py
    scripts/volt_joint_test_gui.py
    scripts/volt_kinematics.py
    scripts/volt_joint_mapping_test.py
    scripts/volt_motion_controller.py
    scripts/volt_run_all.py
    scripts/volt_servo_calibration.py
    scripts/volt_servo_calibration_gui.py
    scripts/volt_serial_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

add_test(
  NAME test_servo_calibration
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_servo_calibration.py
)

ament_package()

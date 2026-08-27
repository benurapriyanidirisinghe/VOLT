cmake_minimum_required(VERSION 3.8)
project(volt_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

enable_testing()

install(
  DIRECTORY config description emotes launch meshes moveit2 rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES CONTROL_GUI.md EMOTES.md PHYSICAL_TESTS.md SERVO_CALIBRATION.md THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES scripts/volt_arm_workflow.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/volt_emote_engine.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/volt_real_profiles.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/volt_face.py
  DESTINATION lib/${PROJECT_NAME}
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
    scripts/volt_body_balance_test.py
    scripts/volt_measure_gui.py
    scripts/volt_motion_controller.py
    scripts/volt_pca_channel_gui.py
    scripts/volt_physical_tests.py
    scripts/volt_run_all.py
    scripts/volt_servo_calibration.py
    scripts/volt_servo_calibration_gui.py
    scripts/volt_serial_protocol.py
    scripts/volt_serial_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

add_test(
  NAME test_servo_calibration
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_servo_calibration.py
)

add_test(
  NAME test_serial_protocol
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_serial_protocol.py
)

add_test(
  NAME test_gait_controller
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gait_controller.py
)

add_test(
  NAME test_kinematics
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_kinematics.py
)

add_test(
  NAME test_emote_engine
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_emote_engine.py
)

add_test(
  NAME test_emote_player
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_emote_player.py
)

add_test(
  NAME test_joint_command_router
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_command_router.py
)

add_test(
  NAME test_launch_configuration
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_configuration.py
)

add_test(
  NAME test_motion_controller
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motion_controller.py
)

add_test(
  NAME test_serial_bridge
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_serial_bridge.py
)

add_test(
  NAME test_run_all
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_run_all.py
)

add_test(
  NAME test_arm_workflow
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_arm_workflow.py
)

add_test(
  NAME test_control_gui_arm_guards
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_control_gui_arm_guards.py
)

add_test(
  NAME test_face
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_face.py
)

add_test(
  NAME test_control_gui_face
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_control_gui_face.py
)

add_test(
  NAME test_face_firmware
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_face_firmware.py
)

add_test(
  NAME test_physical_tests
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_physical_tests.py
)

add_test(
  NAME test_real_robot_profiles
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_robot_profiles.py
)

add_test(
  NAME test_motion_emotes
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motion_emotes.py
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(nextage_tutorials)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nextage_description QUIET)
find_package(roseus QUIET)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_program(IRTEUSGL_EXECUTABLE irteusgl
  HINTS "$ENV{EUSDIR}/Linux64/bin")

set(NEXTAGE_LISP "${CMAKE_CURRENT_SOURCE_DIR}/nextage.l")
install(FILES "${NEXTAGE_LISP}" DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/ft_gravity_calib.py
  scripts/ft_gravity_compensate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config euslisp model sample udev urdf
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)


ament_package()

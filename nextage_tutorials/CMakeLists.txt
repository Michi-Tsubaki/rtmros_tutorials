cmake_minimum_required(VERSION 3.8)
project(nextage_tutorials)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nextage_description QUIET)
find_package(rclcpp REQUIRED)
find_package(roseus QUIET)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_program(IRTEUSGL_EXECUTABLE irteusgl
  HINTS "$ENV{EUSDIR}/Linux64/bin")

set(NEXTAGE_LISP_FILES)
foreach(lisp_file nextage.l jsk-nextage.l jsk-nextage-grippers.l)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${lisp_file}")
    list(APPEND NEXTAGE_LISP_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${lisp_file}")
  endif()
endforeach()

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

if(NEXTAGE_LISP_FILES)
  install(FILES ${NEXTAGE_LISP_FILES} DESTINATION share/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/ft_gravity_calib.py
  scripts/ft_gravity_compensate.py
  scripts/gripper_joint_state_publisher.py
  scripts/viser_teleope.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config euslisp meshes model sample udev urdf
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
    PATTERN "*.launch.xml"
)

ament_package()

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
if(NOT EXISTS "${NEXTAGE_LISP}")
  message(FATAL_ERROR "Pre-generated NEXTAGE JSKEus model is missing")
endif()
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

if(BUILD_TESTING AND IRTEUSGL_EXECUTABLE)
  find_package(ament_cmake_test REQUIRED)
  configure_file(test/test-nextage-ik.l.in
    "${CMAKE_CURRENT_BINARY_DIR}/test-nextage-ik.l" @ONLY)
  ament_add_test(nextage_model_ik
    COMMAND "${IRTEUSGL_EXECUTABLE}"
            "${CMAKE_CURRENT_BINARY_DIR}/test-nextage-ik.l"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60)
  get_filename_component(EUS_BIN_DIR "${IRTEUSGL_EXECUTABLE}" DIRECTORY)
  get_filename_component(EUS_ARCH_DIR "${EUS_BIN_DIR}" DIRECTORY)
  set_tests_properties(nextage_model_ik PROPERTIES
    ENVIRONMENT
      "LD_LIBRARY_PATH=${EUS_ARCH_DIR}/lib:$ENV{LD_LIBRARY_PATH}")
endif()

ament_package()

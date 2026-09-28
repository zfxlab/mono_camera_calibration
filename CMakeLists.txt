cmake_minimum_required(VERSION 3.8)
project(mono_camera_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(mono_calibration_core src/calibrator.cpp)
set_target_properties(mono_calibration_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mono_calibration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mono_calibration_core PUBLIC ${OpenCV_LIBS})

add_library(mono_calibration_component SHARED src/calibrator_node.cpp)
target_include_directories(mono_calibration_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mono_calibration_component mono_calibration_core ${OpenCV_LIBS})
ament_target_dependencies(mono_calibration_component
  cv_bridge diagnostic_msgs rclcpp rclcpp_components sensor_msgs std_srvs)
rclcpp_components_register_node(mono_calibration_component
  PLUGIN "mono_camera_calibration::MonoCalibrationNode"
  EXECUTABLE mono_calibrator_node)

add_executable(mono_calibration_gui src/gui.cpp)
target_link_libraries(mono_calibration_gui ${OpenCV_LIBS})
ament_target_dependencies(mono_calibration_gui
  diagnostic_msgs rclcpp sensor_msgs std_srvs)

install(TARGETS mono_calibration_core mono_calibration_component mono_calibration_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_calibrator test/test_calibrator.cpp)
  target_link_libraries(test_calibrator mono_calibration_core ${OpenCV_LIBS})
endif()

ament_export_include_directories(include)
ament_export_libraries(mono_calibration_core)
ament_package()

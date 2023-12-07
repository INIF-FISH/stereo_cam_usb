cmake_minimum_required(VERSION 3.8)
project(stereo_cam_usb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp
  rclcpp_components
  OpenCV
  image_transport
  sensor_msgs
  cv_bridge
)

add_library(${PROJECT_NAME} SHARED
  src/stereo_cam_usb_node.cpp
  src/stereo_cam_usb/stereo_cam_usb.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}  
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "stereo_cam_usb::StereoCamUsbNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

add_executable(${PROJECT_NAME}_node
  src/stereo_cam_usb_node.cpp
  src/stereo_cam_usb/stereo_cam_usb.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}_node
  # testing 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

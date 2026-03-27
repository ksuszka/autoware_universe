cmake_minimum_required(VERSION 3.14)
project(autoware_detection_by_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find PCL Dependencies
find_package(PCL REQUIRED)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detection_by_tracker_node.cpp
  src/object_splitter/object_splitter.cpp
  src/tracker/tracker_handler.cpp
  src/utils/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::detection_by_tracker::DetectionByTracker"
  EXECUTABLE detection_by_tracker_node
)

# Add tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Add object_splitter_test
  ament_add_gtest(object_splitter_test
    test/object_splitter_test.cpp
  )

  target_include_directories(object_splitter_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_libraries(object_splitter_test
    ${PROJECT_NAME}
    gtest_main
  )

  ament_target_dependencies(object_splitter_test
    autoware_perception_msgs
    tier4_perception_msgs
    pcl_conversions
    rclcpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

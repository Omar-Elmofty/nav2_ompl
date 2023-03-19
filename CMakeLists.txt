cmake_minimum_required(VERSION 3.8)
project(nav2_ompl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ompl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

set(library_name ompl_planner)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  ompl
  tf2_geometry_msgs
)

add_library(${library_name} SHARED
  src/ompl_planner.cpp
)
target_link_libraries(${library_name} ${OMPL_LIBRARIES})

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core nav2_ompl_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()

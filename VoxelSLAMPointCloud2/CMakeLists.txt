cmake_minimum_required(VERSION 3.10)
project(voxelslam_pointcloud2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(SRC_FILES
  src/voxelslam_pc2.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  rviz_ogre_vendor::OgreMain
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  sensor_msgs
  rviz_common
  rviz_default_plugins
  rviz_rendering
)

# Prevent pluginlib from stripping the plugin class (needed since the class is
# only referenced through the pluginlib export macro).
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Registers the plugin in the ament index under rviz_common's pluginlib
# resource type, which is how rviz2 discovers available display plugins.
# This also installs plugin_description.xml into share/${PROJECT_NAME}.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/media")
  install(DIRECTORY media/
    DESTINATION share/${PROJECT_NAME}/media)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

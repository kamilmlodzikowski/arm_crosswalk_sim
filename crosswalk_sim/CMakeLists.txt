cmake_minimum_required(VERSION 3.8)
project(crosswalk_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Gazebo Harmonic through the ROS 2 Jazzy vendor packages
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin_VERSION_MAJOR})
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport_VERSION_MAJOR})
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
set(GZ_MSGS_VER ${gz-msgs_VERSION_MAJOR})

# ---- Gazebo system plugin
add_library(CrosswalkScenario SHARED src/CrosswalkScenario.cc)
target_link_libraries(CrosswalkScenario
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::register
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-msgs${GZ_MSGS_VER}::core)
install(TARGETS CrosswalkScenario LIBRARY DESTINATION lib)

# ---- Python (world generator + monitor node)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/crosswalk_monitor scripts/fetch_assets.sh DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

# make the plugin discoverable by Gazebo after sourcing the workspace
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()

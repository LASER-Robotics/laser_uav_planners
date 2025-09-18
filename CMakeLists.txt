cmake_minimum_required(VERSION 3.8)
project(laser_uav_planners)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
Eigen3
nav_msgs
laser_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(laser_uav_planners SHARED 
  src/pmm_trajectory3d.cpp 
  src/pmm_mg_trajectory3d.cpp 
  src/common.cpp
  src/pmm_trajectory.cpp
  src/agile_planner.cpp
)

target_include_directories(laser_uav_planners PRIVATE src/include)

ament_target_dependencies(laser_uav_planners
  ${dependencies}
)

install(TARGETS laser_uav_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    params
    DESTINATION include/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()

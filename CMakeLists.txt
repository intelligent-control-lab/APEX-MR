cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(apex_mr VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */
add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)
# Check if profiling is needed
option(ENABLE_PROFILING "Enable profiling with gprof" OFF)
option(BUILD_MOTOPLUS "Build MotoPlus" OFF)

if (ENABLE_PROFILING)
    # Add the -pg flag to the compiler and linker flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
endif()


# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
  tf2_geometry_msgs
  message_generation
  actionlib
)

find_package(yk_tasks QUIET)


if (BUILD_MOTOPLUS)
  # Attempt to find the hardware_interface library
  find_library(HARDWARE_INTERFACE_LIB hardware_interface)
  find_path(HARDWARE_INTERFACE_INCLUDE_DIR NAMES hardware_interface.h
            HINTS /usr/local/include/hardware_interface)

  if(HARDWARE_INTERFACE_LIB AND HARDWARE_INTERFACE_INCLUDE_DIR)
    message(STATUS "Found hardware_interface library: ${HARDWARE_INTERFACE_LIB}")
    message(STATUS "Found hardware_interface include directory: ${HARDWARE_INTERFACE_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "Could not find hardware_interface library and headers")
  endif()

  
endif()

add_service_files(
  FILES
  StabilityScore.srv
  TaskAssignment.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# /* -------------------------------------------------------------------------- */
# /*                               Catkin Package                               */
# /* -------------------------------------------------------------------------- */
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs 
    message_runtime
  DEPENDS
    EIGEN3
)

# /* -------------------------------------------------------------------------- */
# /*                               Include Directories                          */
# /* -------------------------------------------------------------------------- */

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */
file(GLOB src_files src/instance.cpp src/planner.cpp
                    src/logger.cpp src/tpg.cpp src/adg.cpp src/task.cpp src/policy.cpp
                    src/shortcutter.cpp
                    )

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
  jsoncpp
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)

file(GLOB lego_files src/lego/*.cpp src/lego/Utils/*.cpp src/lego_policy.cpp)

add_library(lego SHARED ${lego_files})

# Conditionally define a macro if yk_tasks is found
if(yk_tasks_FOUND)
  message(STATUS "yk_tasks found, defining HAVE_YK_TASKS")
  target_compile_definitions(lego PRIVATE HAVE_YK_TASKS)
else()
  message(WARNING "yk_tasks not found, excluding related code")
endif()

target_link_libraries(lego
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  jsoncpp
)

target_include_directories(lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */
add_executable(lego_node
  src/exe/lego_node.cpp
)

# Conditionally define a macro if yk_tasks is found
if(yk_tasks_FOUND)
  target_compile_definitions(lego_node PRIVATE HAVE_YK_TASKS)
endif()

target_link_libraries(lego_node
  ${PROJECT_NAME}
  lego
  ${catkin_LIBRARIES}
)

install(TARGETS lego_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(lego_assign
  src/exe/lego_assignment.cpp
)

target_link_libraries(lego_assign
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  lego
)
add_dependencies(lego_assign ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS lego_assign
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sllidar_ros2)

# Set CMake policies to avoid warnings
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(sllidar_node	src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})
# target_link_libraries(sllidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(sllidar_client src/sllidar_client.cpp)
ament_target_dependencies(sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(pointcloudfilter src/pointcloudfilter.cpp)
# Link libraries
target_link_libraries(pointcloudfilter
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pointcloudfilter
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# OpenCVのリンクを追加
target_link_libraries(pointcloudfilter ${OpenCV_LIBS})

################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS sllidar_node sllidar_client pointcloudfilter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()


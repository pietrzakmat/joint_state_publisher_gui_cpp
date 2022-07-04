cmake_minimum_required(VERSION 3.8)
project(joint_state_publisher_gui_cpp)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(orocos_kdl REQUIRED)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(PROJECT_SOURCES
    src/main.cpp
    src/joint_state_widget.cpp
    src/joint_state_widget.h
    src/jntinfo.h
    src/qrosnode.cpp
    src/qrosnode.h
)
include_directories(
src/
)


add_executable(${PROJECT_NAME}
	       ${PROJECT_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs kdl_parser urdf)
target_link_libraries(${PROJECT_NAME}
Qt5::Core
Qt5::Widgets
rclcpp::rclcpp
${std_msgs_TARGETS}
${sensor_msgs_TARGETS}
#rmw_implementation
${orocos_kdl_LIBRARIES})

ament_export_libraries(${PROJECT_NAME})

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

install(
  TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  
  LIBRARY DESTINATION lib
  #INCLUDES DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

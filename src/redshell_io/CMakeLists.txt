cmake_minimum_required(VERSION 3.8)
project(redshell_io)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
set(DEPENDENICES rclcpp geometry_msgs)

add_executable(redshell_interface src/redshell_interface.cpp)
target_include_directories(redshell_interface PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/include/redshell_messages/include)
ament_target_dependencies(redshell_interface ${DEPENDENICES})
install(TARGETS redshell_interface
    DESTINATION lib/${PROJECT_NAME})

ament_package()

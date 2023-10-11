cmake_minimum_required(VERSION 3.3)
#set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#set(CATKIN_ENABLE_TESTING OFF)
project(waterlinked_a50)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(nlohmann_json)


rosidl_generate_interfaces( waterlinked_a50
        "msg/Config.msg"
        "msg/PositionReport.msg"
        "msg/PositionReportStamped.msg"
        "msg/Transducer.msg"
        "msg/TransducerReport.msg"
        "msg/TransducerReportStamped.msg"
        "msg/TransducerStamped.msg"
        DEPENDENCIES builtin_interfaces
        DEPENDENCIES std_msgs
        DEPENDENCIES geometry_msgs
)

include_directories(include)

add_executable(dvl_node src/dvl_publisher_node_main.cpp src/dvl_publisher_node.cpp )
ament_target_dependencies(dvl_node rclcpp std_msgs)
#rosidl_get_typesupport_target(dvl_node "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(dvl_node nlohmann_json::nlohmann_json )
rosidl_target_interfaces(dvl_node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")


#target_include_directories(dvl_node
#        PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)





ament_export_dependencies(rosidl_default_runtime)

ament_package()


#add_executable(dvl_node src/dvl_publisher_node_main.cpp src/dvl_publisher_node)
#ament_target_dependencies(dvl_node rclcpp std_msgs )






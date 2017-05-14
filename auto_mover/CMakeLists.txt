cmake_minimum_required(VERSION 2.8.3)

project(auto_mover)

find_package(catkin REQUIRED COMPONENTS 
        message_generation
        std_msgs
        joy
	roscpp
        geometry_msgs  
        ar_sys    
)

add_service_files( FILES robot_state.srv)

generate_messages(
    DEPENDENCIES 
    std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime joy std_msgs geometry_msgs roscpp tf 
)

add_executable(auto_mover src/auto_mover.cpp)
add_dependencies(auto_mover  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_mover ${catkin_LIBRARIES})






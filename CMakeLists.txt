cmake_minimum_required(VERSION 2.8.3)
project(ros_iiwa_fri)

find_package(
	catkin REQUIRED COMPONENTS
	roscpp  
	std_msgs
	sensor_msgs
	genmsg
	message_generation
)

add_message_files(FILES ExternalTorque.msg JointCommandPosition.msg)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	/home/bernard/FRI-Client-SDK_Cpp/include
)

add_executable(fri_talker src/fri_talker.cpp src/MojoClient.cpp src/MojoClient.h)

target_link_libraries(fri_talker ${catkin_LIBRARIES} /home/bernard/FRI-Client-SDK_Cpp/lib/libFRIClient.a)
add_dependencies(fri_talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

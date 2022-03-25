cmake_minimum_required(VERSION 2.8.3)
project(px4_sitl_duck)
set (CMAKE_CXX_STANDARD 11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
#  px4
)

add_service_files(
  FILES
  start_trig.srv
  take_off.srv
  return_home.srv
  request_mode_info.srv
  landing.srv
  next_mission.srv
  previous_mission.srv
  pause_mission.srv
  emergency_landing.srv
  send_mission_info.srv
  request_gps_mission.srv
  record_start.srv
  record_stop.srv
  Waypoint.srv
  Srv_key.srv
  monitor_key.srv
  move_control.srv
  drone2monitor.srv
  drone2monitor_list.srv
  json_drone2monitor.srv
  json_drone2monitor_list.srv
  wp_idx.srv
  rotate_control.srv
  linenumin_waypointout.srv
  automatic_mission_start.srv
  ui_mission_request.srv
  manual_control.srv
  test_srv.srv
)

add_message_files(
        FILES
        estimated_gps.msg
        battery_info.msg
        flight_data.msg
        uav_status.msg
        log_data.msg
        sensor_data.msg
        video_data.msg
        mission_struct.msg
        is_connected.msg
        WaypointInform.msg
        WaypointList.msg
        rpyThrust.msg
        json.msg
        jsonList.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)



find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)

include(FindCURL)
if(NOT CURL_FOUND)
  message("CURL not found!  Aborting...")
  fail()
endif(NOT CURL_FOUND)
include_directories(${CURL_INCLUDE_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
  DEPENDS system-lib
)


#########################
##      WP reader      ##
#########################
add_executable(waypoint_reader waypoint_reader/src/wp_reader_integration.cpp)
add_dependencies(waypoint_reader px4_sitl_duck_generate_messages_cpp)
target_link_libraries(waypoint_reader
                ${catkin_LIBRARIES}
                )

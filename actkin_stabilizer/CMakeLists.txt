cmake_minimum_required(VERSION 3.0.2)
project(actkin_stabilizer)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
  geometry_msgs
  )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EEPoint.msg
  EEState.msg
  PrimitiveState.msg
 )

# catkin_python_setup()
# initialize rtmbuild (generate_messages is called)
rtmbuild2_init(rtmbuild2 geometry_msgs)  # ここにidlとmsgが依存する外部パッケージを書く

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ActKinStabilizerServiceSkel ActKinStabilizerServiceStub PrimitiveStateSkel PrimitiveStateStub
  CATKIN_DEPENDS rtmbuild2
  DEPENDS openrtm_aist
)

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

###########
## Build ##
###########

#add_subdirectory(rtc)

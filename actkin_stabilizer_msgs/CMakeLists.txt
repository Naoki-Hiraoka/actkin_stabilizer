cmake_minimum_required(VERSION 3.0.2)
project(actkin_stabilizer_msgs)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

rtmbuild2_init(rtmbuild2 geometry_msgs) # ここにidlとmsgが依存する外部パッケージを書く

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES ActKinStabilizerSkel ActKinStabilizerStub
#  CATKIN_DEPENDS rtmbuild2
   DEPENDS openrtm_aist
)

###########
## Build ##
###########

rtmbuild2_genidl()

add_custom_target(genidl All DEPENDS RTMBUILD2_${PROJECT_NAME}_genrpc)

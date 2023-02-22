add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint
  prioritized_qp_osqp
  prioritized_inverse_kinematics_solver
  cpp_filters
  ik_constraint_joint_limit_table
  hrpsys_ros_bridge
  joint_limit_table
  collision_checker_msgs
  auto_stabilizer_msgs
  aik_constraint
  aik_constraint_joint_limit_table
  prioritized_acc_inverse_kinematics_solver
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )
link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(ActKinStabilizer SHARED
  ActKinStabilizer.cpp
  ActKinStabilizerService_impl.cpp
  ActToGenFrameConverter.cpp
  ResolvedAccelerationController.cpp
  WrenchDistributor.cpp
  MathUtil.cpp
  )
target_link_libraries(ActKinStabilizer
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  RobotHardwareServiceSkel RobotHardwareServiceStub
  )
set_target_properties(ActKinStabilizer PROPERTIES PREFIX "")
add_dependencies(ActKinStabilizer RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild2_genidl

add_executable(ActKinStabilizerComp ActKinStabilizerComp.cpp)
target_link_libraries(ActKinStabilizerComp ActKinStabilizer)

install(TARGETS ActKinStabilizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

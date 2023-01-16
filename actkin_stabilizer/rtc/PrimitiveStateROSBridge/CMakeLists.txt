add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS})

add_library(PrimitiveStateROSBridge SHARED PrimitiveStateROSBridge.cpp)
target_link_libraries(PrimitiveStateROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS} # generated from idl in this package
  )
set_target_properties(PrimitiveStateROSBridge PROPERTIES PREFIX "")
add_dependencies(PrimitiveStateROSBridge RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild2_genidl

add_executable(PrimitiveStateROSBridgeComp PrimitiveStateROSBridgeComp.cpp)
target_link_libraries(PrimitiveStateROSBridgeComp PrimitiveStateROSBridge)

install(TARGETS PrimitiveStateROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS PrimitiveStateROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(actkin_stabilizer_choreonoid_sample)

find_package(catkin REQUIRED COMPONENTS
  jvrc_models
  hrpsys_choreonoid_tutorials)

catkin_python_setup()
catkin_package()

configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_JVRC.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_JVRC.cnoid @ONLY)

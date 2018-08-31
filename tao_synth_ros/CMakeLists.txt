cmake_minimum_required(VERSION 2.8.3)
project(tao_synth_ros)

find_package(catkin REQUIRED COMPONENTS
  # dynamic_reconfigure
  # nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

roslint_cpp()

# find_library(TAO_SYNTH_LIBRARY NAMES tao_synth)
find_package(tao_synth REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/Delay.cfg
# )

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES tao_synth_ros
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${tao_synth_INCLUDE_DIRES}
)

add_library(tao_synth_ros
  src/tao_synth.cpp
)
# add_dependencies(tao_synth_ros ${PROJECT_NAME}_gencfg)
target_link_libraries(tao_synth_ros ${catkin_LIBRARIES} ${tao_synth_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(tao_synth_ros ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS tao_synth_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   nodelet_tao_synth_ros.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

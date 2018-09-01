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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${tao_synth_INCLUDE_DIRES}
)

add_executable(${PROJECT_NAME}
  src/tao_synth.cpp
)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${tao_synth_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   nodelet_${PROJECT_NAME}.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

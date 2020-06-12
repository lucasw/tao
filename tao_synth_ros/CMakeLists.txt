cmake_minimum_required(VERSION 3.0)
project(tao_synth_ros)

find_package(catkin REQUIRED COMPONENTS
  # dynamic_reconfigure
  # nodelet
  geometry_msgs
  message_generation
  roscpp
  roslint
  sensor_msgs
  std_msgs
  visualization_msgs
)

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)
roslint_cpp()
roslint_python()

# find_library(TAO_SYNTH_LIBRARY NAMES tao_synth)
# find_package(tao_synth REQUIRED)
add_subdirectory(.. tao_synth)

# generate_dynamic_reconfigure_options(
#   cfg/Delay.cfg
# )

add_message_files(
  FILES
  Force.msg
  Instrument.msg
  Output.msg
  Stop.msg
)

add_service_files(
  FILES
  AddAssembly.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

# TODO(lucasw) fix add_subdirectory(..) to provide ../include and tao_synth lib
# message(INFO ${tao_synth_INCLUDE_DIRS})
# message(INFO ${tao_synth_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${tao_synth_INCLUDE_DIRS}
  ../include
)

add_executable(${PROJECT_NAME}
  src/tao_synth.cpp
)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  # ${tao_synth_LIBRARIES}
  tao_synth
)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

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

cmake_minimum_required(VERSION 2.8)
project(examples)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

# set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list(APPEND example_files
  strand
)


list(APPEND example_files_tmp_disable
  40strings
  accesspoint1
  accesspoint2
  bow
  circle2
  circle
  connector
  damp
  diffraction
  ellipse2
  ellipse
  hammer
  lock
  outputarray
  outputs
  pitches
  rectangle2
  rectangle
  stop
  strand
  testt
)

foreach(example ${example_files})
add_executable(${example} ${example}.cpp)
target_link_libraries(${example} tao_synth)
endforeach()


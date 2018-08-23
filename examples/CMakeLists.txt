cmake_minimum_required(VERSION 2.8)
project(examples)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

list(APPEND example_files
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
  stringt
  testt
)

foreach(example ${example_files})
add_executable(${example} ${example}.cpp)
target_link_libraries(${example} tao_synth)
endforeach()


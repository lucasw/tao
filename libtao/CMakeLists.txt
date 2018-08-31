cmake_minimum_required(VERSION 2.8)
project(tao_synth)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(${PROJECT_NAME} SHARED
  access_point.cpp
  bow.cpp
  tao.cpp
  cell.cpp
  circle.cpp
  connector.cpp
  device.cpp
  ellipse.cpp
  # TODO(lucasw) move the graphics engine out
  graphics_engine.cpp
  hammer.cpp
  instrument.cpp
  output.cpp
  pitch.cpp
  rectangle.cpp
  stop.cpp
  strand.cpp
  synth_engine.cpp
  triangle.cpp
)
# -lX11 -lXmu -lXi -lGL -lGLU -lpthread -lglut -laudiofile
target_link_libraries(${PROJECT_NAME} GL GLU glfw)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../include
  DESTINATION ${CMAKE_INSTALL_PREFIX}/
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
  PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

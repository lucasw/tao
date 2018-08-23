cmake_minimum_required(VERSION 2.8)
project(tao_synth)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

add_library(${PROJECT_NAME} SHARED
  TaoAccessPoint.cc
  TaoBow.cc
  Tao.cc
  TaoCell.cc
  TaoCircle.cc
  TaoConnector.cc
  TaoDevice.cc
  TaoEllipse.cc
  # TODO(lucasw) move the graphics engine out
  TaoGraphicsEngine.cc
  TaoHammer.cc
  TaoInstrument.cc
  TaoOutput.cc
  TaoPitch.cc
  TaoRectangle.cc
  TaoStop.cc
  TaoString.cc
  TaoSynthEngine.cc
  TaoTriangle.cc
)
# -lX11 -lXmu -lXi -lGL -lGLU -lpthread -lglut -laudiofile
target_link_libraries(${PROJECT_NAME} GL GLU glut)

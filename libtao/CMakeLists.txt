cmake_minimum_required(VERSION 2.8)
project(libtao_synth)

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

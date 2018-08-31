/* Tao - A software package for sound synthesis with physical models
 * Copyright (C) 1993-1999 Mark Pearson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

extern "C" {
#include <unistd.h>
}

#include <GL/glu.h>
#include <iostream>
#include <tao/tao.h>
#include <tao/access_point.h>
#include <tao/cell.h>
#include <tao/device.h>
#include <tao/graphics_engine.h>
#include <tao/instrument.h>
#include <stdio.h>
#include <string>

static std::shared_ptr<Tao> tao;

// These variables are declared extern by the getopt() function, which
// is used to parse the command line arguments sent to the graphics
// engine. They therefore have to be defined here.

extern char *optarg;
extern int optind, opterr, optopt;

// The following global functions are registered callbacks for OpenGL.

#if 0
void tao_visibility(int state) {
  if (!tao)
    return;
  if (!tao->graphics_engine_)
    return;

  if (state == GLUT_NOT_VISIBLE)
    tao->graphics_engine_->active = FALSE;
  if (state == GLUT_VISIBLE) {
    glutIdleFunc(tao_master_tick);
    tao->graphics_engine_->active = TRUE;
  }
}

void tao_display() {
  if (!tao)
    return;
  if (!tao->graphics_engine_)
    return;
  tao->graphics_engine_->display();
}

void tao_reshape(int w, int h) {
  if (!tao)
    return;
  if (!tao->graphics_engine_)
    return;
  tao->graphics_engine_->reshape(w, h);
}

void tao_special(int key, int x, int y) {
  if (!tao)
    return;
  if (!tao->graphics_engine_)
    return;
  x, y; // referenced to get rid of compiler warning

  switch (key) {
}
#endif

void tao_mouse(GLFWwindow* window, int button, int action, int mods) {
  // std::cout << "mouse " << button << " " << action << " " << mods << "\n";
  TaoGraphicsEngine* tge = static_cast<TaoGraphicsEngine*>(glfwGetWindowUserPointer(window));
  if (!tge)
  {
    throw std::runtime_error("glfw get graphics engine user pointer");
  }
  tge->mouse(button, action, mods);
}

void tao_motion(GLFWwindow* window, double x, double y) {
  // std::cout << "motion " << x << " " << y << "\n";
  TaoGraphicsEngine* tge = static_cast<TaoGraphicsEngine*>(glfwGetWindowUserPointer(window));
  if (!tge)
  {
    throw std::runtime_error("glfw get graphics engine user pointer");
  }
  tge->motion(x, y);
}

void tao_keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
  // std::cout << "key " << key << " " << scancode << " " << action << " " << mods << "\n";
  TaoGraphicsEngine* tge = static_cast<TaoGraphicsEngine*>(glfwGetWindowUserPointer(window));
  if (!tge)
  {
    throw std::runtime_error("glfw get graphics engine user pointer");
  }

  switch (key) {
  case 27:
    exit(0);

  case 'i':
    tge->displayInstrumentNames =
        tge->displayInstrumentNames ? 0 : 1;
    std::cout << "display device names: " << tge->displayDeviceNames << "\n";
    break;

  case 'd':
    tge->displayDeviceNames =
        tge->displayDeviceNames ? 0 : 1;
    std::cout << "display device names: " << tge->displayDeviceNames << "\n";
    break;

  case GLFW_KEY_UP:
    tge->globalMagnification *= 1.1f;
    std::cout << "magnification: " << tge->globalMagnification << "\n";
    break;

  case GLFW_KEY_DOWN:
    tge->globalMagnification /= 1.1f;
    std::cout << "magnification: " << tge->globalMagnification << "\n";
    break;

  case GLFW_KEY_RIGHT:
    if (tge->refreshRate == 1 &&
        !tao->synthesisEngine.isActive()) {
      tao->synthesisEngine.unpause();
      // glutIdleFunc(tao_master_tick);
    } else {
      if (tge->refreshRate < 65536) {
        tge->refreshRate *= 2;
      }
    }
    std::cout << "refresh rate: " << tge->refreshRate << " "
        << tao->synthesisEngine.isActive() << "\n";
    break;

  case GLFW_KEY_LEFT:
    if (tge->refreshRate != 1)
      tge->refreshRate /= 2;
    else {
      if (tao->synthesisEngine.isActive()) {
        tao->synthesisEngine.pause();
      }
    }
    std::cout << "refresh rate: " << tge->refreshRate << " "
        << tao->synthesisEngine.isActive() << "\n";
    break;
  }
}

TaoGraphicsEngine::TaoGraphicsEngine(std::shared_ptr<Tao> ptao) :
    tao_(ptao),
    active(FALSE),
    viewportWidth(1280),
    viewportHeight(720),
    xOffset(-1.0),
    yOffset(-9.0),
    zOffset(-40.0),
    xAngle(-251.0),
    yAngle(0.0),
    zAngle(-31.0),
    globalMagnification(1.0),
    refreshRate(1),  // refresh graphics window on every time step of synthesis engine
    drag(FALSE),
    dolly(FALSE),
    rotate(FALSE),
    displayInstrumentNames(1),
    displayDeviceNames(1) {
  tao = ptao;
  setInstrDisplayResolution();
  std::cout << "Using graphics " << viewportWidth << " " << viewportHeight << "\n";
}

void TaoGraphicsEngine::activate() { active = 1; };

void TaoGraphicsEngine::deactivate() { active = 0; }

static void glfw_error(int error, const char* text)
{
  std::cerr << error << " " << text << std::endl;
}

TaoGraphicsEngine::~TaoGraphicsEngine()
{
  glfwTerminate();
}

void TaoGraphicsEngine::init(const std::string win_name, int lineMode) {
  glfwSetErrorCallback(glfw_error);
  if (!glfwInit())
  {
    throw std::runtime_error("glfw couldn't init");
  }

  window_.reset(glfwCreateWindow(1280, 720, win_name.c_str(), NULL, NULL),
      [](GLFWwindow* w) { glfwDestroyWindow(w); } );
  if (!window_)
  {
    throw std::runtime_error("glfw couldn't make window");
  }

  glfwMakeContextCurrent(window_.get());

  glfwSetWindowUserPointer(window_.get(), this);
  glfwSetKeyCallback(window_.get(), tao_keyboard);
  glfwSetMouseButtonCallback(window_.get(), tao_mouse);
  glfwSetCursorPosCallback(window_.get(), tao_motion);

  // std::cout << "OpenGL shader language version: "
  //     << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

  glClearColor(0.7, 0.7, 0.7, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  swapBuffers();
  glClear(GL_COLOR_BUFFER_BIT);

  if (lineMode == TAO_ANTIALIAS) {
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
  }

#if 1
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
#endif

  /*
      static float fog_color[] = {0.7, 0.75, 0.78, 1.0};
      static float fogDensity = 1.0;

      glEnable(GL_FOG);
      glFogi(GL_FOG_MODE, GL_LINEAR);
      glFogf(GL_FOG_START, 30.0);
      glFogf(GL_FOG_END, 1000.0);
      glFogf(GL_FOG_DENSITY, fogDensity);
      glFogfv(GL_FOG_COLOR, fog_color);
  */

  active = TRUE;

  flushGraphics();

  std::cout << "initialized graphics\n";
}

void TaoGraphicsEngine::reshape(int w, int h) {
  viewportWidth = w;
  viewportHeight = h;
  setInstrDisplayResolution();
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  if (projectionMode == TAO_PERSPECTIVE) {
    gluPerspective(15.0, (GLdouble)w / (GLdouble)h, 10.0, 2000.0);
  } else {
    glOrtho(-80.0, 80.0, -80.0 * (GLdouble)h / (GLdouble)w,
            80.0 * (GLdouble)h / (GLdouble)w, -200.0, 1000.0);
  }
}

void TaoGraphicsEngine::clearBackBuffer() {
  glColor3f(1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void TaoGraphicsEngine::pushModelViewMatrix() {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glPushMatrix();
}

void TaoGraphicsEngine::popModelViewMatrix() { glPopMatrix(); }

void TaoGraphicsEngine::rotateAndTranslate() {
  glTranslatef(xOffset / 5.0, yOffset / 5.0, zOffset);
  glRotatef(xAngle / 4.0, 1.0, 0.0, 0.0);
  glRotatef(yAngle / 4.0, 0.0, 1.0, 0.0);
  glRotatef(zAngle / 4.0, 0.0, 0.0, 1.0);
  glTranslatef(-translateX, -translateY, 0.0);
}

void TaoGraphicsEngine::display() {
  timestream << std::setw(0) << std::setprecision(4)
             << std::setiosflags(std::ios::fixed);
  timestream << "Time=" << tao_->synthesisEngine.time << " seconds";

  glPushMatrix();
  float ratio;
  int width, height;
  glfwGetFramebufferSize(window_.get(), &width, &height);
  ratio = width / (float) height;
  glViewport(0, 0, width, height);
  // glfw test
  // glClear(GL_COLOR_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  const float sc = 100.0;
  glOrtho(-ratio * sc, ratio * sc, -sc , sc, sc, -sc);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  #if 1
  glPushMatrix();
  glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
  glBegin(GL_TRIANGLES);
  glColor3f(1.f * sc, 0.f, 0.f);
  glVertex3f(-0.6f * sc, -0.4f * sc, 0.f);
  glColor3f(0.f, 1.f * sc, 0.f);
  glVertex3f(0.6f * sc, -0.4f * sc, 0.f);
  glColor3f(0.f, 0.f, 1.f * sc);
  glVertex3f(0.f, 0.6f * sc, 0.f);
  glEnd();
  glPopMatrix();
  glPopMatrix();
  #endif

  displayInstruments();
  displayDevices();

  swapBuffers();
  flushGraphics();
  glfwPollEvents();

  /*  THIS DOESN'T WORK AND NEVER HAS!!

      glColor3f(0.0,0.0,0.0);
      glRasterPos2f(9.0, 9.0);

      string=timestream.str();
      len = (int) strlen(string);
      for (i = 0; i < len; i++)
          {
          glutBitmapCharacter(GLUT_BITMAP_8_BY_13, string[i]);
          }

      timestream.seekp(0, ostream::beg);
  */
}

void TaoGraphicsEngine::flushGraphics() { glFlush(); }

void TaoGraphicsEngine::swapBuffers() {
  glfwSwapBuffers(window_.get());
}

void TaoGraphicsEngine::mouse(int button, int action, int mods) {
  if (action == GLFW_PRESS) {
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      drag = TRUE;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      dolly = TRUE;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      rotate = TRUE;
      break;
    }

    // refreshRateToRestore=refreshRate;
    // refreshRate=1;
    glfwGetCursorPos(window_.get(), &lastMouseX, &lastMouseY);
  } else {
    drag = FALSE;
    dolly = FALSE;
    rotate = FALSE;
    // refreshRate=refreshRateToRestore;
  }
}

void TaoGraphicsEngine::setInstrDisplayResolution() {
  if (zOffset <= -viewportWidth * 3)
    jstep = 8;
  else if (zOffset <= -viewportWidth * 3 / 2)
    jstep = 4;
  else if (zOffset <= -viewportWidth * 3 / 4)
    jstep = 2;
  else
    jstep = 1;
}

void TaoGraphicsEngine::motion(double x, double y) {
  if (drag == TRUE) {
    xOffset += x - lastMouseX;
    yOffset -= y - lastMouseY;
    lastMouseX = x;
    lastMouseY = y;

    std::cout << xOffset << ", " << yOffset << ", " << zOffset
        << ", angles " << xAngle << ", " << zAngle << "\n";
  }

  if (dolly == TRUE) {
    zOffset += y - lastMouseY;
    setInstrDisplayResolution();
    lastMouseX = x;
    lastMouseY = y;
  }

  if (rotate == TRUE) {
    xAngle += y - lastMouseY;
    zAngle += x - lastMouseX;
    lastMouseX = x;
    lastMouseY = y;
  }
}

void TaoGraphicsEngine::calculateOriginForRotations() {
  for (TaoInstrument *i = tao_->synthesisEngine.instrumentList; i; i = i->next) {
    if (i->worldx < minWorldX)
      minWorldX = i->worldx;
    if (i->worldy < minWorldY)
      minWorldY = i->worldy;
    if (i->worldx + i->xmax > maxWorldX)
      maxWorldX = i->worldx + i->xmax;
    if (i->worldy + i->ymax > maxWorldY)
      maxWorldY = i->worldy + i->ymax;
  };

  translateX = minWorldX + (maxWorldX - minWorldX) / 2.0;
  translateY = minWorldY + (maxWorldY - minWorldY) / 2.0;
  scaleBy = 20.0 / (maxWorldX - minWorldX);
}

void TaoGraphicsEngine::displayCharString(GLfloat x, GLfloat y, GLfloat z,
                                          const std::string text) {
  if (tao_->synthesisEngine.tick % refreshRate != 0)
    return;
  if (!this->active)
    return;

  glColor3f(0.0, 0.0, 0.0);
  glRasterPos3f(x, y, z);
  for (size_t i = 0; i < text.size(); i++) {
    // glutBitmapCharacter(GLUT_BITMAP_8_BY_13, text[i]);
  }
}

void TaoGraphicsEngine::displayCharString(GLfloat x, GLfloat y, GLfloat z,
                                          const std::string text, GLfloat r, GLfloat g,
                                          GLfloat b) {
  int len, i;

  if (tao_->synthesisEngine.tick % refreshRate != 0)
    return;
  if (!this->active)
    return;

  glColor3f(r, g, b);
  glRasterPos3f(x, y, z);
  for (i = 0; i < text.size(); i++) {
    // glutBitmapCharacter(GLUT_BITMAP_8_BY_13, text[i]);
  }
}

void TaoGraphicsEngine::displayPoint(GLfloat x, GLfloat y, int colour) {
  if (tao_->synthesisEngine.tick % refreshRate != 0)
    return;
  if (!this->active)
    return;
  setDrawColour(colour);
  glPointSize(5.0);
  glBegin(GL_POINTS);
  glVertex2f(x, y);
  glEnd();
}

void TaoGraphicsEngine::displayInstruments() {
  if (!this->active)
    return;
  for (TaoInstrument *i = tao_->synthesisEngine.instrumentList; i; i = i->next)
    displayInstrument(*i);
}

void TaoGraphicsEngine::displayDevices() {
  if (!this->active)
    return;
  for (TaoDevice *d = tao_->synthesisEngine.deviceList; d; d = d->next)
    d->display();
}

void TaoGraphicsEngine::displayInstrument(TaoInstrument &instr) {
  static GLdouble textClipPlane[] = {0.0, 0.0, 1.0, 50.0};
  register short i, j;
  TaoCell *c;
  float cellPosition;
  float magnification = globalMagnification * instr.getMagnification();
  GLfloat x, y, z;
  int step;

  if (!this->active)
    return;

  glColor3f(0.0, 0.0, 0.0);
  glLineWidth(1.0);

  for (j = instr.ymax; j >= 0;
       j -= jstep) // draw horizontal lines through rows of cells
  {
    glBegin(GL_LINE_STRIP);
    for (i = 0, c = instr.rows[j].cells; i <= instr.rows[j].xmax; i++, c++) {
      cellPosition = c->position;
      if (c->velocityMultiplier < instr.defaultVelocityMultiplier)
        glColor3f(0.2, 0.2, 0.2);
      else
        glColor3f(0.0, 0.0, 0.0);
      x = instr.worldx + instr.rows[j].offset + i;
      z = cellPosition * magnification;
      y = j + instr.worldy;
      glVertex3f(x, y, z);
    }
    glEnd();
  }

  glColor3f(0.0, 0.0, 0.0);

  if (instr.ymax > 0) // if instrument is 2D, draw line round perimeter
  {                   // if perimeter is locked make line thicker
    if (instr.perimeterLocked)
      glLineWidth(2.0);
    else
      glLineWidth(1.0);

    glBegin(GL_LINE_STRIP);

    j = 0;

    for (i = 0, c = instr.rows[0].cells; i <= instr.rows[0].xmax; i++, c++)
    // across bottom
    {
      cellPosition = c->position;
      x = instr.worldx + instr.rows[j].offset + i;
      z = cellPosition * magnification;
      y = j + instr.worldy;

      glVertex3f(x, y, z);
    }

    for (j = 0; j <= instr.ymax; j++) // up right
    {
      c = &instr.rows[j].cells[instr.rows[j].xmax];
      cellPosition = c->position;
      x = instr.worldx + instr.rows[j].offset + instr.rows[j].xmax;
      z = cellPosition * magnification;
      y = j + instr.worldy;

      glVertex3f(x, y, z);
    }

    j = instr.ymax;

    for (i = instr.rows[instr.ymax].xmax; i >= 0; i--) // across top
    {
      c = &instr.rows[instr.ymax].cells[i];
      cellPosition = c->position;
      x = instr.worldx + instr.rows[j].offset + i;
      z = cellPosition * magnification;
      y = j + instr.worldy;

      glVertex3f(x, y, z);
    }

    for (j = instr.ymax; j >= 0; j--) // down left
    {
      c = &instr.rows[j].cells[0];
      cellPosition = c->position;
      x = instr.worldx + instr.rows[j].offset;
      z = cellPosition * magnification;
      y = j + instr.worldy;

      glVertex3f(x, y, z);
    }

    glEnd();
  }

  glPointSize(3.0);
  glBegin(GL_POINTS);

  for (j = 0; j <= instr.ymax; j++) // scan cells again to mark any
  {                                 // locked or glued ones

    for (i = 0, c = instr.rows[j].cells; i <= instr.rows[j].xmax; i++, c++) {
      cellPosition = c->position;
      if (c->mode & TAO_CELL_LOCK_MODE) {
        if ((i == 0 || i == instr.rows[j].xmax || j == 0 || j == instr.ymax) &&
            instr.perimeterLocked) // if we're at the instrument's
        {                          // perimeter and it is locked then
          continue;                // don't mark individual locked
        }                          // points as the locked perimeter
                                   // has already been displayed as a
                                   // thicker line.
        glColor3f(0.0f, 0.0f, 0.0f);
        x = instr.worldx + instr.rows[j].offset + i;
        z = cellPosition * magnification;
        y = j + instr.worldy;
        glVertex3f(x, y, z);
      }
    }
  }

  glEnd();

  j = instr.ymax / 2;
  c = &instr.rows[j].cells[instr.xmax];
  cellPosition = c->position;
  x = (GLfloat)(instr.worldx + instr.xmax + 3.0);
  z = (GLfloat)(cellPosition * magnification);
  y = (GLfloat)(j + instr.worldy);

  // std::cout << "x=" << x << " y=" << y << " z=" << z
  //     << " name=" << instr.name << std::endl;

  if (displayInstrumentNames)
    displayCharString(x, y, z, instr.name, 0.0, 0.0, 0.0);
}

void TaoGraphicsEngine::displayAccessPoint(TaoInstrument &instr, int i, int j) {
  TaoCell *c;
  float cellPosition;
  GLfloat x, y, z;
  GLfloat screenx, screeny;

  if (!this->active)
    return;

  glPointSize(4.0);
  glColor3f(1.0, 0.0, 0.0);
  if (tao_->synthesisEngine.tick % this->refreshRate == 0) {
    c = &instr.rows[j].cells[0];
    cellPosition = c->position;
    x = instr.worldx + instr.rows[j].offset + i;
    z = cellPosition * instr.amplification * globalMagnification;
    y = j + instr.worldy;
    glBegin(GL_POINTS);
    glVertex3f(x, y, z);
    glEnd();
  }
}

class TaoAccessPoint;

void TaoGraphicsEngine::displayAccessPoint(TaoAccessPoint &p) {
  int i, j;
  TaoCell *c;
  GLfloat x, y, z;
  GLfloat screenx, screeny;

  if (!this->active)
    return;
  if (p.instrument == NULL)
    return;
  TaoInstrument &instr = *(p.instrument);

  if (tao_->synthesisEngine.tick % this->refreshRate == 0) {
    j = (int)p.celly;
    x = instr.worldx + instr.rows[j].offset + p.cellx;
    z = (GLfloat)(p.getPosition() * instr.amplification * globalMagnification);
    y = (GLfloat)(p.celly + instr.worldy);

    glColor3f(1.0, 0.0, 0.0);
    glPointSize(4.0);
    glBegin(GL_POINTS);
    glVertex3f(x, y, z);
    glEnd();
  }
}

float TaoGraphicsEngine::screenX(TaoInstrument &instr, float x, float y) {
  return 0.0;
}

float TaoGraphicsEngine::screenY(TaoInstrument &instr, float x, float y) {
  return 0.0;
}

float TaoGraphicsEngine::screenY(TaoInstrument &instr, float x, float y,
                                 float z) {
  return 0.0;
}

void TaoGraphicsEngine::displayPointInInstrumentSpace(TaoInstrument &instr,
                                                      float instrx,
                                                      float instry,
                                                      float instrz) {
  GLfloat x, y, z;

  if (!this->active)
    return;

  TaoAccessPoint p = instr.point(instrx, instry);

  x = (GLfloat)(instr.getWorldX() + p.cellx);
  z = (GLfloat)(instrz * instr.getMagnification() * globalMagnification);
  y = (GLfloat)(instr.getWorldY() + p.celly);

  glPointSize(4.0);
  glBegin(GL_POINTS);
  glVertex3f(x, y, z);
  glEnd();
}

void TaoGraphicsEngine::label(TaoInstrument &instr, float x, float y,
                              float labelXOffset, float labelYOffset,
                              const std::string caption, GLfloat r, GLfloat g, GLfloat b) {
  GLfloat worldx, worldy, worldz;
  TaoAccessPoint &p = instr.point(x, y);

  if (tao_->synthesisEngine.tick % refreshRate != 0)
    return;

  if (active) {
    worldx = (GLfloat)(instr.getWorldX() + p.cellx);
    worldz = (GLfloat)(p.getPosition() * instr.getMagnification() *
                       globalMagnification);
    worldy = (GLfloat)(instr.getWorldY() + p.celly);
    displayCharString(worldx, worldy, worldz, caption, r, g, b);
  }
}

void TaoGraphicsEngine::label(TaoInstrument &instr, float x, float labelXOffset,
                              float labelYOffset, const std::string caption, GLfloat r,
                              GLfloat g, GLfloat b) {
  GLfloat worldx, worldy, worldz;
  TaoAccessPoint &p = instr.point(x);

  if (tao_->synthesisEngine.tick % refreshRate != 0)
    return;

  if (active) {
    worldx = (GLfloat)(instr.getWorldX() + p.cellx);
    worldz = (GLfloat)(p.getPosition() * instr.getMagnification() *
                       globalMagnification);
    worldy = (GLfloat)(instr.getWorldY() + p.celly);
    displayCharString(worldx, worldy, worldz, caption, r, g, b);
  }
}

void TaoGraphicsEngine::label(TaoInstrument &instr, float x, float y, float z,
                              float labelXOffset, float labelYOffset,
                              const std::string caption, GLfloat r, GLfloat g, GLfloat b) {
  GLfloat worldx, worldy, worldz;
  TaoAccessPoint &p = instr.point(x, y);

  if (tao_->synthesisEngine.tick % refreshRate != 0)
    return;

  if (active) {
    worldx = (GLfloat)(instr.getWorldX() + p.cellx);
    worldz = (GLfloat)(z * instr.getMagnification() * globalMagnification);
    worldy = (GLfloat)(instr.getWorldY() + p.celly);
    displayCharString(worldx, worldy, worldz, caption, r, g, b);
  }
}

void TaoGraphicsEngine::setDrawColour(int c) {
  if (!this->active)
    return;
  switch (c) {
  case 0:
    glColor3f(0.0, 0.0, 0.0);
    break; // BLACK
  case 1:
    glColor3f(0.0, 0.0, 1.0);
    break; // BLUE
  case 2:
    glColor3f(0.0, 1.0, 0.0);
    break; // GREEN
  case 3:
    glColor3f(0.0, 1.0, 1.0);
    break; // CYAN
  case 4:
    glColor3f(1.0, 0.0, 0.0);
    break; // RED
  case 5:
    glColor3f(1.0, 0.0, 1.0);
    break; // MAGENTA
  case 6:
    glColor3f(1.0, 1.0, 0.0);
    break; // YELLOW
  case 7:
    glColor3f(1.0, 1.0, 1.0);
    break; // WHITE
  }
}

void TaoGraphicsEngine::setClearColour(int c) {
  if (!this->active)
    return;
  switch (c) {
  case 0:
    glClearColor(0.0, 0.0, 0.0, 0.0);
    break; // BLACK
  case 1:
    glClearColor(0.0, 0.0, 1.0, 1.0);
    break; // BLUE
  case 2:
    glClearColor(0.0, 1.0, 0.0, 1.0);
    break; // GREEN
  case 3:
    glClearColor(0.0, 1.0, 1.0, 1.0);
    break; //
  case 4:
    glClearColor(1.0, 0.0, 0.0, 1.0);
    break; // RED
  case 5:
    glClearColor(1.0, 0.0, 1.0, 1.0);
    break; //
  case 6:
    glClearColor(1.0, 1.0, 0.0, 1.0);
    break; //
  case 7:
    glClearColor(1.0, 1.0, 1.0, 1.0);
    break; // WHITE
  default:
    break;
  }
}

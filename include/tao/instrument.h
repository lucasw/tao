/* TaoSynth - A software package for sound synthesis with physical models
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

#ifndef INSTRUMENT_H
#define INSTRUMENT_H

extern "C" {
#include <stdlib.h>
}

#include <tao/access_point.h>
#include <tao/pitch.h>
#include <vector>

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

namespace tao {
struct DLLEXPORT Row {
  int xmax;
  int offset;
  std::vector<Cell> cells;
};

class Cell;
class SynthEngine;
class GraphicsEngine;

class DLLEXPORT Instrument {
  friend class SynthEngine;
  friend class GraphicsEngine;
  // TODO(lucasw) is this needed?
  friend int main(int argc, char *argv[]);

public:
  Instrument(std::shared_ptr<Manager> manager);
  ~Instrument();
  Instrument(std::shared_ptr<Manager> manager, const Pitch &xpitch, const Pitch &ypitch,
                float decay);
  Instrument(std::shared_ptr<Manager> manager, const std::string name, const Pitch &xpitch,
                const Pitch &ypitch, float decay);
  void calculateForces(int startRow, int endRow);
  void calculatePositions(int startRow, int endRow);
  std::string getName() { return name; }
  float getMagnification();
  Instrument &setMagnification(float m);
  Instrument &setDecay(float x1, float x2, float y1, float y2, float decay);
  Instrument &setDecay(float left, float right, float decay);
  Instrument &setDecay(float decay);
  Instrument &resetDecay(float x1, float x2, float y1, float y2);
  Instrument &resetDecay(float left, float right);
  Instrument &resetDecay();
  Instrument &setDamping(float x1, float x2, float y1, float y2,
                            float damping);
  Instrument &setDamping(float left, float right, float damping);
  Instrument &setDamping(float position, float damping);
  Instrument &setDamping(float damping);
  Instrument &resetDamping(float x1, float x2, float y1, float y2);
  Instrument &resetDamping(float left, float right);
  Instrument &resetDamping(float position);
  Instrument &resetDamping();
  Instrument &lock(float x1, float x2, float y1, float y2);
  Instrument &lock(float x, float y);
  Instrument &lockLeft();
  Instrument &lockRight();
  Instrument &lockTop();
  Instrument &lockBottom();
  Instrument &lockPerimeter();
  Instrument &lockCorners();
  Instrument &lockEnds();
  AccessPoint &operator()(float x, float y);
  AccessPoint &operator()(float x);
  Cell &at(float x, float y);
  AccessPoint &point(float x, float y); // The only difference between these
  AccessPoint &point(float x);          // two func's and the operator() ones
  // above is that these two don't
  // affect the graphics display whereas
  // the operator() ones mark the point
  // accessed with a blue point.

  void displayAt(int x, int y) {
    graphx = x;
    graphy = y;
  }
  void placeAt(int x, int y) {
    worldx = x;
    worldy = y;
  }
  void placeAbove(Instrument &ref);
  void placeBelow(Instrument &ref);
  void placeRightOf(Instrument &ref);
  void placeLeftOf(Instrument &ref);
  void placeAbove(Instrument &ref, int distanceInWorldCoords);
  void placeBelow(Instrument &ref, int distanceInWorldCoords);
  void placeRightOf(Instrument &ref, int distanceInWorldCoords);
  void placeLeftOf(Instrument &ref, int distanceInWorldCoords);
  void copyWorldPosition(Instrument &instr);

  inline int getWorldX() { return worldx; }
  inline int getWorldY() { return worldy; }
  inline int getXMax() { return xmax; }
  inline int getYMax() { return ymax; }
  inline float getXFrequency() { return xfrequency; }
  inline float getYFrequency() { return yfrequency; }

  float decay2velocityMultiplier(float decay);
  int hertz2cells(float freq);

  static void glue(Instrument &i1, float x1, float y1, Instrument &i2,
                   float x2, float y2);
  static void glue(Instrument &i1, float x1, float y1, Instrument &i2,
                   float x2);
  static void glue(Instrument &i1, float x1, Instrument &i2, float x2,
                   float y2);
  static void glue(Instrument &i1, float x1, Instrument &i2, float x2);

  static void join(AccessPoint &a1, AccessPoint &a2);

  // old prototype
  //
  //    static void join(Instrument &i1, float x1, float y1,
  //		     Instrument &i2, float x2, float y2);

  Pitch xpitch, ypitch;

  // TODO(lucasw) ought to be protected but need to be able to get
  // position externally
  std::vector<Row> rows;
protected:
  std::shared_ptr<Manager> manager_;
  const std::string name;
  float defaultDecay, defaultVelocityMultiplier;
  float amplification;
  Instrument *next;

  int xmax, ymax;
  int graphx, graphy;
  int worldx, worldy;
  float xfrequency, yfrequency;
  int perimeterLocked;
  AccessPoint currentAccess;

  // these variables are used to implement access points on an instrument with
  // real x and y coordinates.

  virtual void createTheMaterial() = 0;
  void initialiseCells();
  void linkCells();

  static float defaultMass;
  static void glueCells(Cell *c1, Cell *c2);
  static void joinLeftToLeft(Cell &cell1, Cell &cell2);
  static void joinLeftToRight(Cell &cell1, Cell &cell2);
  static void joinRightToLeft(Cell &cell1, Cell &cell2);
  static void joinRightToRight(Cell &cell1, Cell &cell2);
  static void joinBottomToBottom(Cell &cell1, Cell &cell2);
  static void joinBottomToTop(Cell &cell1, Cell &cell2);
  static void joinTopToBottom(Cell &cell1, Cell &cell2);
  static void joinTopToTop(Cell &cell1, Cell &cell2);
};
}
#endif

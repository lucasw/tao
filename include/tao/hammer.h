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

#ifndef HAMMER_H
#define HAMMER_H

#include <tao/device.h>

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

namespace tao {
class DLLEXPORT Hammer : public Device {
public:
  Hammer(std::shared_ptr<Manager> manager);
  Hammer(std::shared_ptr<Manager> manager, const std::string hammerName);
  void update();
  void display();
  void reset();
  void drop();
  void operator()(AccessPoint &a);
  void operator()(Instrument &instr, float x);
  void operator()(Instrument &instr, float x, float y);
  Hammer &setMass(float m);
  Hammer &setPosition(float p);
  Hammer &setInitVelocity(float v);
  Hammer &setGravity(float g);
  Hammer &setHeight(float h);
  Hammer &setDamping(float d);
  Hammer &setHardness(float h);
  Hammer &setMaxImpacts(int maxImpacts);
  float getMass();
  float getPosition();
  float getVelocity();
  float getInitVelocity();
  float getGravity();
  float getHeight();
  float getDamping();
  float getHardness();
  int numberOfImpacts();
  int getMaxImpacts();

private:
  enum HammerMode { contact, nocontact };
  HammerMode mode;
  float height;
  float position;
  float initVelocity;
  float velocity;
  float mass;
  float force;
  float damping;
  float size;
  float gravity;
  float hardness;
  float collisionForce;
  int numImpacts;
  int maxImpacts;
};
}
#endif

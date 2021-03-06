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

#include <tao/hammer.h>
#include <tao/manager.h>
#include <tao/access_point.h>
#include <tao/cell.h>
#include <tao/instrument.h>

using namespace tao;
Hammer::Hammer(std::shared_ptr<Manager> manager) : Device(manager, "") {
  deviceType = Device::HAMMER;
  mode = nocontact;
  mass = 10.0;
  height = 20.0;
  gravity = 1.0;
  damping = 1.0;

  position = height;
  velocity = 0.0;
  initVelocity = 0.0;
  force = 0.0;
  numImpacts = 0;

  addToSynthesisEngine();
}

Hammer::Hammer(std::shared_ptr<Manager> manager, const std::string hammerName) :
    Device(manager, hammerName) {
  deviceType = Device::HAMMER;
  mode = nocontact;
  mass = 10.0;
  height = 20.0;
  gravity = 1.0;
  damping = 1.0;
  hardness = 1.0;

  position = height;
  velocity = 0.0;
  initVelocity = 0.0;
  force = 0.0;
  numImpacts = 0;
  maxImpacts = 9999999;

  addToSynthesisEngine();
}

void Hammer::reset() {
  position = height;
  velocity = initVelocity;
  force = 0.0;
  numImpacts = 0;
}

void Hammer::drop() {
  reset();
  activate();
}

void Hammer::operator()(AccessPoint &a) { apply(a); }

void Hammer::operator()(Instrument &instr, float x) { apply(instr(x)); }

void Hammer::operator()(Instrument &instr, float x, float y) {
  apply(instr(x, y));
}

void Hammer::update() {
  if (!active)
    return;
  if (!targetInstrument)
    return;

  force = -mass * gravity;

  if (mode == nocontact && position < interfacePoint.getPosition()) {
    mode = contact;
  }

  if (mode == contact && position > interfacePoint.getPosition()) {
    mode = nocontact;
    numImpacts++;
    if (numImpacts >= maxImpacts) {
      deactivate();
    }
  }

  if (mode == contact) {
    collisionForce = (this->position - interfacePoint.getPosition()) * hardness;
    interfacePoint.applyForce(collisionForce);
    this->force -= collisionForce;
  }

  velocity += force / mass;
  velocity *= damping;
  position += velocity;
}

Hammer &Hammer::setHeight(float h) {
  height = h;
  return *this;
}
Hammer &Hammer::setMass(float m) {
  mass = m;
  return *this;
}
Hammer &Hammer::setPosition(float p) {
  position = p;
  return *this;
}
Hammer &Hammer::setInitVelocity(float v) {
  initVelocity = v;
  return *this;
}
Hammer &Hammer::setGravity(float g) {
  gravity = g;
  return *this;
}
Hammer &Hammer::setDamping(float d) {
  damping = d;
  return *this;
}
Hammer &Hammer::setHardness(float h) {
  hardness = h;
  return *this;
}
Hammer &Hammer::setMaxImpacts(int m) {
  maxImpacts = m;
  return *this;
}

float Hammer::getHeight() { return height; }
float Hammer::getMass() { return mass; }
float Hammer::getPosition() { return position; }
float Hammer::getVelocity() { return velocity; }
float Hammer::getInitVelocity() { return initVelocity; }
float Hammer::getGravity() { return gravity; }
float Hammer::getDamping() { return damping; }
float Hammer::getHardness() { return hardness; }
int Hammer::numberOfImpacts() { return numImpacts; }
int Hammer::getMaxImpacts() { return maxImpacts; }

void Hammer::display() {
  if (!manager_->graphics_engine_)
    return;
  if (!manager_->graphics_engine_->active || !active || !targetInstrument)
    return;
  if (manager_->synthesisEngine.tick % manager_->graphics_engine_->refreshRate != 0)
    return;

  Instrument &instr = interfacePoint.getInstrument();
  GLfloat x;
  GLfloat y;
  GLfloat z;

  manager_->graphics_engine_->displayAccessPoint(interfacePoint);
  manager_->graphics_engine_->displayPointInInstrumentSpace(
      *targetInstrument, interfacePoint.x, interfacePoint.y, this->position);

  if (manager_->graphics_engine_->displayDeviceNames) {
    x = (GLfloat)(instr.getWorldX() + interfacePoint.cellx);
    z = (GLfloat)(this->position * instr.getMagnification() *
                      manager_->graphics_engine_->globalMagnification +
                  2.0);
    y = (GLfloat)(instr.getWorldY() + interfacePoint.celly);

    manager_->graphics_engine_->displayCharString(x, y, z, this->name, 1.0, 1.0, 1.0);
  }
}

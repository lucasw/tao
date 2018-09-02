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

#include <tao/bow.h>
#include <tao/manager.h>
#include <tao/access_point.h>
#include <tao/instrument.h>
#include <cmath>

using namespace tao;
Bow::Bow(std::shared_ptr<Manager> manager) : Device(manager, "") {
  deviceType = Device::BOW;
  mode = stick;
  bowVelocity = 0.0;
  downwardForce = 1.0;
  bowPointPosition = 0.0;

  addToSynthesisEngine();
}

Bow::Bow(std::shared_ptr<Manager> manager, const std::string bowName) : Device(manager, bowName) {
  deviceType = Device::BOW;
  mode = stick;
  bowVelocity = 0.0;
  downwardForce = 1.0;
  bowPointPosition = 0.0;

  addToSynthesisEngine();
}

Bow &Bow::setForce(float force) {
  downwardForce = force;
  return *this;
}

Bow &Bow::setVelocity(float velocity) {
  bowVelocity = velocity;
  return *this;
}

float Bow::getForce() { return downwardForce; }
float Bow::getVelocity() { return bowVelocity; }

void Bow::operator()(AccessPoint &a) { apply(a); }

void Bow::operator()(Instrument &instr, float x) { apply(instr(x)); }

void Bow::operator()(Instrument &instr, float x, float y) {
  apply(instr(x, y));
}

void Bow::update() {
  if (!active)
    return;
  if (!targetInstrument)
    return;

  bowPointPosition += bowVelocity;
  instrVelocity = interfacePoint.getVelocity();
  instrForce = interfacePoint.getForce();
  relativeVelocity = instrAcceleration = bowVelocity - instrVelocity;
  // a=dv/dt but dt=1 so a=dv.
  if (mode == stick) // if in `stick' mode.
  {
    stickingForce = bowPointPosition - interfacePoint.getPosition();
    if (stickingForce > downwardForce)
      mode = slip;
    else
      forceExerted = stickingForce; // if static frictional
  }                                 // force required is too
                                    // great, change to
                                    // `slip' mode.

  else // else in `slip' mode.
  {
    slippingForce = downwardForce / (1.0 + fabs(relativeVelocity));
    if (instrVelocity >= 0.0) {
      mode = stick;                                    // if the cell starts
      bowPointPosition = interfacePoint.getPosition(); // travelling in the same
    }                                                  // (positive) direction
                                                       // as the bow, change to
    else
      forceExerted = slippingForce; // `stick' mode.
  }

  interfacePoint.applyForce(forceExerted); // apply the appropriate
} // frictional force.

void Bow::display() {
  if (!manager_->graphics_engine_)
    return;
  if (!active || !targetInstrument || !manager_->graphics_engine_->active)
    return;
  if (manager_->synthesisEngine.tick % manager_->graphics_engine_->refreshRate != 0)
    return;

  Instrument &instr = interfacePoint.getInstrument();
  GLfloat x;
  GLfloat y;
  GLfloat z;

  manager_->graphics_engine_->displayAccessPoint(interfacePoint);

  if (manager_->graphics_engine_->displayDeviceNames) {
    x = (GLfloat)(instr.getWorldX() + interfacePoint.cellx);
    z = (GLfloat)(interfacePoint.getPosition() * instr.getMagnification() *
                      manager_->graphics_engine_->globalMagnification +
                  2.0);
    y = (GLfloat)(instr.getWorldY() + interfacePoint.celly);

    manager_->graphics_engine_->displayCharString(x, y, z, this->name.c_str(), 1.0, 1.0, 1.0);
  }
}

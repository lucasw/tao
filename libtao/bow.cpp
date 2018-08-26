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

#include <tao/bow.h>
#include <tao/tao.h>
#include <tao/access_point.h>
#include <tao/instrument.h>
#include <cmath>

TaoBow::TaoBow(std::shared_ptr<Tao> tao) : TaoDevice(tao, "") {
  deviceType = TaoDevice::BOW;
  mode = stick;
  bowVelocity = 0.0;
  downwardForce = 1.0;
  bowPointPosition = 0.0;

  addToSynthesisEngine();
}

TaoBow::TaoBow(std::shared_ptr<Tao> tao, const std::string bowName) : TaoDevice(tao, bowName) {
  deviceType = TaoDevice::BOW;
  mode = stick;
  bowVelocity = 0.0;
  downwardForce = 1.0;
  bowPointPosition = 0.0;

  addToSynthesisEngine();
}

TaoBow &TaoBow::setForce(float force) {
  downwardForce = force;
  return *this;
}

TaoBow &TaoBow::setVelocity(float velocity) {
  bowVelocity = velocity;
  return *this;
}

float TaoBow::getForce() { return downwardForce; }
float TaoBow::getVelocity() { return bowVelocity; }

void TaoBow::operator()(TaoAccessPoint &a) { apply(a); }

void TaoBow::operator()(TaoInstrument &instr, float x) { apply(instr(x)); }

void TaoBow::operator()(TaoInstrument &instr, float x, float y) {
  apply(instr(x, y));
}

void TaoBow::update() {
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

void TaoBow::display() {
  if (!tao_->graphics_engine_)
    return;
  if (!active || !targetInstrument || !tao_->graphics_engine_->active)
    return;
  if (tao_->synthesisEngine.tick % tao_->graphics_engine_->refreshRate != 0)
    return;

  TaoInstrument &instr = interfacePoint.getInstrument();
  GLfloat x;
  GLfloat y;
  GLfloat z;

  tao_->graphics_engine_->displayAccessPoint(interfacePoint);

  if (tao_->graphics_engine_->displayDeviceNames) {
    x = (GLfloat)(instr.getWorldX() + interfacePoint.cellx);
    z = (GLfloat)(interfacePoint.getPosition() * instr.getMagnification() *
                      tao_->graphics_engine_->globalMagnification +
                  2.0);
    y = (GLfloat)(instr.getWorldY() + interfacePoint.celly);

    tao_->graphics_engine_->displayCharString(x, y, z, this->name.c_str(), 1.0, 1.0, 1.0);
  }
}

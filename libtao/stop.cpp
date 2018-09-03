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

#include <tao/stop.h>
#include <tao/manager.h>
#include <tao/instrument.h>

using namespace tao;
Stop::Stop(std::shared_ptr<Manager> manager, const std::string stopName) :
    Device(manager, stopName),
    last_x(0.0) {
  deviceType = Device::STOP;
  maxDampingCoefficient = 0.95f;
  currentDampingCoefficient = 0.0f;
  dampMode = 0;
  amount = 1.0f;

  addToSynthesisEngine();
}

Stop::~Stop() {}

Stop &Stop::dampModeOn() {
  dampMode = 1;
  return *this;
}
Stop &Stop::dampModeOff() {
  dampMode = 0;
  return *this;
}

Stop &Stop::setAmount(float amount) {
  this->amount = amount;
  setDamping(1.0 - (1.0 - maxDampingCoefficient) * amount);
  return *this;
}

Stop &Stop::setDamping(float damping) {
  currentDampingCoefficient = damping;
  return *this;
}

void Stop::operator()(AccessPoint &a) { apply(a); }

void Stop::operator()(Instrument &instr, float x) { apply(instr(x)); }

void Stop::operator()(Instrument &instr, float x, float y) {
  apply(instr(x, y));
}

void Stop::operator()(String &string, Pitch &stoppedPitch) {
  float fundamentalFreq, stoppedFreq, effectiveStringLength, stopPosition;

  fundamentalFreq = string.xpitch.asFrequency();
  stoppedFreq = stoppedPitch.asFrequency();
  effectiveStringLength = fundamentalFreq / stoppedFreq;
  stopPosition = 1.0 - effectiveStringLength;
  apply(string(stopPosition));
}

void Stop::update() {
  if (!active)
    return;
  if (!targetInstrument)
    return;

  if (dampMode == 1 && manager_->synthesisEngine.tick % 100 == 0) {
    targetInstrument->resetDamping(0, last_x);
    targetInstrument->setDamping(0, interfacePoint.x,
                                 currentDampingCoefficient);
  }

  last_x = interfacePoint.x;

  AccessPoint::ground(interfacePoint, 2.0 * amount);
}

void Stop::display() {
  if (!manager_->graphics_engine_)
    return;
  if (!manager_->graphics_engine_->active || !active || !targetInstrument)
    return;
  if (manager_->synthesisEngine.tick % manager_->graphics_engine_->refreshRate != 0)
    return;

  Instrument &instr = interfacePoint.getInstrument();
  GLfloat x, y, z;

  manager_->graphics_engine_->displayAccessPoint(interfacePoint);

  if (manager_->graphics_engine_->displayDeviceNames) {
    x = (GLfloat)(instr.getWorldX() + interfacePoint.cellx);
    z = (GLfloat)(interfacePoint.getPosition() * instr.getMagnification() *
                      manager_->graphics_engine_->globalMagnification +
                  2.0);
    y = (GLfloat)(instr.getWorldY() + interfacePoint.celly);

    manager_->graphics_engine_->displayCharString(x, y, z, this->name, 1.0, 1.0, 1.0);
  }
}

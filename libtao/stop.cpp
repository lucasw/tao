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

#include <tao/stop.h>
#include <tao/tao.h>
#include <tao/instrument.h>

TaoStop::TaoStop(std::shared_ptr<Tao> tao, const std::string stopName) : TaoDevice(tao, stopName) {
  deviceType = TaoDevice::STOP;
  maxDampingCoefficient = 0.95f;
  currentDampingCoefficient = 0.0f;
  dampMode = 0;
  amount = 1.0f;

  addToSynthesisEngine();
}

TaoStop::~TaoStop() {}

TaoStop &TaoStop::dampModeOn() {
  dampMode = 1;
  return *this;
}
TaoStop &TaoStop::dampModeOff() {
  dampMode = 0;
  return *this;
}

TaoStop &TaoStop::setAmount(float amount) {
  this->amount = amount;
  setDamping(1.0 - (1.0 - maxDampingCoefficient) * amount);
  return *this;
}

TaoStop &TaoStop::setDamping(float damping) {
  currentDampingCoefficient = damping;
  return *this;
}

void TaoStop::operator()(TaoAccessPoint &a) { apply(a); }

void TaoStop::operator()(TaoInstrument &instr, float x) { apply(instr(x)); }

void TaoStop::operator()(TaoInstrument &instr, float x, float y) {
  apply(instr(x, y));
}

void TaoStop::operator()(TaoString &string, TaoPitch &stoppedPitch) {
  float fundamentalFreq, stoppedFreq, effectiveStringLength, stopPosition;

  fundamentalFreq = string.xpitch.asFrequency();
  stoppedFreq = stoppedPitch.asFrequency();
  effectiveStringLength = fundamentalFreq / stoppedFreq;
  stopPosition = 1.0 - effectiveStringLength;
  apply(string(stopPosition));
}

void TaoStop::update() {
  static float last_x = 0.0;

  if (!active)
    return;
  if (!targetInstrument)
    return;

  if (dampMode == 1 && tao_->synthesisEngine.tick % 100 == 0) {
    targetInstrument->resetDamping(0, last_x);
    targetInstrument->setDamping(0, interfacePoint.x,
                                 currentDampingCoefficient);
  }

  last_x = interfacePoint.x;

  TaoAccessPoint::ground(interfacePoint, 2.0 * amount);
}

void TaoStop::display() {
  if (!tao_->graphics_engine_)
    return;
  if (!tao_->graphics_engine_->active || !active || !targetInstrument)
    return;
  if (tao_->synthesisEngine.tick % tao_->graphics_engine_->refreshRate != 0)
    return;

  TaoInstrument &instr = interfacePoint.getInstrument();
  GLfloat x, y, z;

  tao_->graphics_engine_->displayAccessPoint(interfacePoint);

  if (tao_->graphics_engine_->displayDeviceNames) {
    x = (GLfloat)(instr.getWorldX() + interfacePoint.cellx);
    z = (GLfloat)(interfacePoint.getPosition() * instr.getMagnification() *
                      tao_->graphics_engine_->globalMagnification +
                  2.0);
    y = (GLfloat)(instr.getWorldY() + interfacePoint.celly);

    tao_->graphics_engine_->displayCharString(x, y, z, this->name, 1.0, 1.0, 1.0);
  }
}

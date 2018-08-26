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

#include <tao/device.h>
#include <tao/tao.h>
#include <tao/access_point.h>
#include <tao/instrument.h>
#include <string.h>

TaoDevice::TaoDevice(std::shared_ptr<Tao> tao) :
    tao_(tao),
    interfacePoint(tao),
    name("anon") {
  targetInstrument = NULL;
  interfacePoint.clear();
  active = 0;
  next = NULL;
  x = 0.0;
  y = 0.0;
}

TaoDevice::~TaoDevice() {}

TaoDevice::TaoDevice(std::shared_ptr<Tao> tao, const std::string deviceName) :
    tao_(tao), interfacePoint(tao), name(deviceName) {
  targetInstrument = NULL;
  active = 0;
  next = NULL;
}

std::string TaoDevice::getName() { return name; }

float TaoDevice::getX() { return interfacePoint.x; }

float TaoDevice::getY() { return interfacePoint.y; }

void TaoDevice::apply(TaoAccessPoint &point) {
  targetInstrument = &point.getInstrument();
  interfacePoint = point;
  activate();
}

void TaoDevice::remove() {
  targetInstrument = NULL;
  interfacePoint.clear();
  deactivate();
}

void TaoDevice::activate() { active = TRUE; }

void TaoDevice::deactivate() { active = FALSE; }

void TaoDevice::addToSynthesisEngine() { tao_->synthesisEngine.addDevice(this); }

void TaoDevice::removeFromSynthesisEngine() {
  tao_->synthesisEngine.removeDevice(this);
}

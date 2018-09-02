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

#include <tao/device.h>
#include <tao/manager.h>
#include <tao/access_point.h>
#include <tao/instrument.h>
#include <string.h>

using namespace tao;
Device::Device(std::shared_ptr<Manager> manager) :
    manager_(manager),
    interfacePoint(manager),
    name("anon") {
  targetInstrument = NULL;
  interfacePoint.clear();
  active = 0;
  next = NULL;
  x = 0.0;
  y = 0.0;
}

Device::~Device() {}

Device::Device(std::shared_ptr<Manager> manager, const std::string deviceName) :
    manager_(manager), interfacePoint(manager), name(deviceName) {
  targetInstrument = NULL;
  active = 0;
  next = NULL;
}

std::string Device::getName() { return name; }

float Device::getX() { return interfacePoint.x; }

float Device::getY() { return interfacePoint.y; }

void Device::apply(AccessPoint &point) {
  targetInstrument = &point.getInstrument();
  interfacePoint = point;
  activate();
}

void Device::remove() {
  targetInstrument = NULL;
  interfacePoint.clear();
  deactivate();
}

void Device::activate() { active = TRUE; }

void Device::deactivate() { active = FALSE; }

void Device::addToSynthesisEngine() { manager_->synthesisEngine.addDevice(this); }

void Device::removeFromSynthesisEngine() {
  manager_->synthesisEngine.removeDevice(this);
}

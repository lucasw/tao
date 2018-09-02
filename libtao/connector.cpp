/* Manager - A software package for sound synthesis with physical models
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

#include <tao/connector.h>
#include <tao/manager.h>
#include <tao/access_point.h>
#include <tao/cell.h>
#include <tao/instrument.h>

using namespace tao;
Connector::Connector(std::shared_ptr<Manager> manager) :
    Device(manager),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  anchorPoint1 = 0.0;
  anchorPoint2 = 0.0;
  strength = 1.0;

  addToSynthesisEngine();
}

Connector::~Connector() {}

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  anchorPoint1 = 0.0;
  anchorPoint2 = 0.0;
  strength = 1.0;

  addToSynthesisEngine();
}

// Constructors for case where both ends are connected to access points on
// instruments.

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName, AccessPoint &ap1,
                           AccessPoint &ap2) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  accessPoint1 = ap1;
  accessPoint2 = ap2;
  strength = 1.0;

  addToSynthesisEngine();
}

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName, AccessPoint &ap1,
                           AccessPoint &ap2, float connectionStrength) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  accessPoint1 = ap1;
  accessPoint2 = ap2;
  strength = connectionStrength;

  addToSynthesisEngine();
}

// Constructors for case where first end is connected to an access point on an
// instrument but the opposite end is connected to an anchor point, i.e. just a
// numerical value representing a position along the axis of vibration. This
// anchor point might be a constant or might be some time varying signal derived
// elsewhere in the script.

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName,
                     AccessPoint &ap,
                     float anchor) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  accessPoint1 = ap;
  anchorPoint2 = anchor;
  strength = 1.0;

  addToSynthesisEngine();
}

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName,
                     AccessPoint &ap,
                     float anchor, float connectionStrength) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  accessPoint1 = ap;
  anchorPoint2 = anchor;
  strength = connectionStrength;

  addToSynthesisEngine();
}

// Constructors for case where second end is connected to an access point on an
// instrument but the first end is connected to an anchor point, i.e. just a
// numerical value representing a position along the axis of vibration. This
// anchor point might be a constant or might be some time varying signal derived
// elsewhere in the script.

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName, float anchor,
                           AccessPoint &ap) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  anchorPoint1 = anchor;
  accessPoint2 = ap;
  strength = 1.0;

  addToSynthesisEngine();
}

Connector::Connector(std::shared_ptr<Manager> manager, const std::string connectorName, float anchor,
                           AccessPoint &ap, float connectionStrength) :
    Device(manager, connectorName),
    accessPoint1(manager),
    accessPoint2(manager) {
  deviceType = Device::CONNECTOR;
  anchorPoint1 = anchor;
  accessPoint2 = ap;
  strength = connectionStrength;

  addToSynthesisEngine();
}

// The Connector::operator() functions are used to adjust the access/anchor
// points of a connector during a performance. The arguments represent the newly
// calculate access points or anchor values. The first two functions represent
// the case where both ends of the connector are connected to access points on
// instruments.

void Connector::operator()(AccessPoint &a1, AccessPoint &a2) {
  accessPoint1 = a1;
  accessPoint2 = a2;
  strength = 1.0;
}

void Connector::operator()(AccessPoint &a1, AccessPoint &a2,
                              float connectionStrength) {
  accessPoint1 = a1;
  accessPoint2 = a2;
  strength = connectionStrength;
}

// The next two functions represent the case where the first end of the
// connector
// is connected to an access point but the second end is connected to an anchor
// point (a numerical value rather than a position on an instrument).

void Connector::operator()(AccessPoint &ap, float anchor) {
  accessPoint1 = ap;
  accessPoint2.clear();
  anchorPoint2 = anchor;
  strength = 1.0;
}

void Connector::operator()(AccessPoint &ap, float anchor,
                              float connectionStrength) {
  accessPoint1 = ap;
  accessPoint2.clear();
  anchorPoint2 = anchor;
  strength = connectionStrength;
}

// The next two functions represent the case where the first end of the
// connector
// is connected to an anchor point but the second end is connected to an access
// point.

void Connector::operator()(float anchor, AccessPoint &ap) {
  accessPoint1.clear();
  anchorPoint1 = anchor;
  accessPoint2 = ap;
  strength = 1.0;
}

void Connector::operator()(float anchor, AccessPoint &ap,
                              float connectionStrength) {
  accessPoint1.clear();
  anchorPoint1 = anchor;
  accessPoint2 = ap;
  strength = connectionStrength;
}

void Connector::update() {
  if (accessPoint1.instrument && accessPoint2.instrument)
    this->updateAccessToAccess();
  else if (accessPoint1.instrument && !accessPoint2.instrument)
    this->updateAccessToAnchor();
  else if (!accessPoint1.instrument && accessPoint2.instrument)
    this->updateAnchorToAccess();
}

void Connector::updateAccessToAccess() {
  static float eaa, eab, eac, ead;
  static float eba, ebb, ebc, ebd;
  static float eca, ecb, ecc, ecd;
  static float eda, edb, edc, edd;

  static float faa, fab, fac, fad;
  static float fba, fbb, fbc, fbd;
  static float fca, fcb, fcc, fcd;
  static float fda, fdb, fdc, fdd;

  eaa = accessPoint1.X * accessPoint1.Y * accessPoint2.X * accessPoint2.Y;
  eab = accessPoint1.X * accessPoint1.Y * accessPoint2.X_ * accessPoint2.Y;
  eac = accessPoint1.X * accessPoint1.Y * accessPoint2.X * accessPoint2.Y_;
  ead = accessPoint1.X * accessPoint1.Y * accessPoint2.X_ * accessPoint2.Y_;

  eba = accessPoint1.X_ * accessPoint1.Y * accessPoint2.X * accessPoint2.Y;
  ebb = accessPoint1.X_ * accessPoint1.Y * accessPoint2.X_ * accessPoint2.Y;
  ebc = accessPoint1.X_ * accessPoint1.Y * accessPoint2.X * accessPoint2.Y_;
  ebd = accessPoint1.X_ * accessPoint1.Y * accessPoint2.Y_ * accessPoint2.X_;

  eca = accessPoint1.X * accessPoint1.Y_ * accessPoint2.X * accessPoint2.Y;
  ecb = accessPoint1.X * accessPoint1.Y_ * accessPoint2.X_ * accessPoint2.Y;
  ecc = accessPoint1.X * accessPoint1.Y_ * accessPoint2.X * accessPoint2.Y_;
  ecd = accessPoint1.X * accessPoint1.Y_ * accessPoint2.X_ * accessPoint2.Y_;

  eda = accessPoint1.X_ * accessPoint1.Y_ * accessPoint2.X * accessPoint2.Y;
  edb = accessPoint1.X_ * accessPoint1.Y_ * accessPoint2.X_ * accessPoint2.Y;
  edc = accessPoint1.X_ * accessPoint1.Y_ * accessPoint2.X * accessPoint2.Y_;
  edd = accessPoint1.X_ * accessPoint1.Y_ * accessPoint2.X_ * accessPoint2.Y_;

  if (accessPoint1.cella) {
    if (accessPoint2.cella)
      faa = (accessPoint2.cella->position - accessPoint1.cella->position) * eaa;
    if (accessPoint2.cellb)
      fab = (accessPoint2.cellb->position - accessPoint1.cella->position) * eab;
    if (accessPoint2.cellc)
      fac = (accessPoint2.cellc->position - accessPoint1.cella->position) * eac;
    if (accessPoint2.celld)
      fad = (accessPoint2.celld->position - accessPoint1.cella->position) * ead;
  }

  if (accessPoint1.cellb) {
    if (accessPoint2.cella)
      fba = (accessPoint2.cella->position - accessPoint1.cellb->position) * eba;
    if (accessPoint2.cellb)
      fbb = (accessPoint2.cellb->position - accessPoint1.cellb->position) * ebb;
    if (accessPoint2.cellc)
      fbc = (accessPoint2.cellc->position - accessPoint1.cellb->position) * ebc;
    if (accessPoint2.celld)
      fbd = (accessPoint2.celld->position - accessPoint1.cellb->position) * ebd;
  }

  if (accessPoint1.cellc) {
    if (accessPoint2.cella)
      fca = (accessPoint2.cella->position - accessPoint1.cellc->position) * eca;
    if (accessPoint2.cellb)
      fcb = (accessPoint2.cellb->position - accessPoint1.cellc->position) * ecb;
    if (accessPoint2.cellc)
      fcc = (accessPoint2.cellc->position - accessPoint1.cellc->position) * ecc;
    if (accessPoint2.celld)
      fcd = (accessPoint2.celld->position - accessPoint1.cellc->position) * ecd;
  }

  if (accessPoint1.celld) {
    if (accessPoint2.cella)
      fda = (accessPoint2.cella->position - accessPoint1.celld->position) * eda;
    if (accessPoint2.cellb)
      fdb = (accessPoint2.cellb->position - accessPoint1.celld->position) * edb;
    if (accessPoint2.cellc)
      fdc = (accessPoint2.cellc->position - accessPoint1.celld->position) * edc;
    if (accessPoint2.celld)
      fdd = (accessPoint2.celld->position - accessPoint1.celld->position) * edd;
  }

  if (accessPoint1.cella)
    accessPoint1.cella->force += (faa + fab + fac + fad) * strength;
  if (accessPoint1.cellb)
    accessPoint1.cellb->force += (fba + fbb + fbc + fbd) * strength;
  if (accessPoint1.cellc)
    accessPoint1.cellc->force += (fca + fcb + fcc + fcd) * strength;
  if (accessPoint1.celld)
    accessPoint1.celld->force += (fda + fdb + fdc + fdd) * strength;

  if (accessPoint2.cella)
    accessPoint2.cella->force += (-faa - fba - fca - fda) * strength;
  if (accessPoint2.cellb)
    accessPoint2.cellb->force += (-fab - fbb - fcb - fdb) * strength;
  if (accessPoint2.cellc)
    accessPoint2.cellc->force += (-fac - fbc - fcc - fdc) * strength;
  if (accessPoint2.celld)
    accessPoint2.celld->force += (-fad - fbd - fcd - fdd) * strength;
}

void Connector::updateAccessToAnchor() {
  static float elasticities[2][2];

  elasticities[0][0] = accessPoint1.X * accessPoint1.Y;
  elasticities[0][1] = accessPoint1.X_ * accessPoint1.Y;
  elasticities[1][0] = accessPoint1.X * accessPoint1.Y_;
  elasticities[1][1] = accessPoint1.X_ * accessPoint1.Y_;

  if (accessPoint1.cella)
    accessPoint1.cella->force -= (accessPoint1.cella->position - anchorPoint2) *
                                 elasticities[0][0] * strength;
  if (accessPoint1.cellb)
    accessPoint1.cellb->force -= (accessPoint1.cellb->position - anchorPoint2) *
                                 elasticities[0][1] * strength;
  if (accessPoint1.cellc)
    accessPoint1.cellc->force -= (accessPoint1.cellc->position - anchorPoint2) *
                                 elasticities[1][0] * strength;
  if (accessPoint1.celld)
    accessPoint1.celld->force -= (accessPoint1.celld->position - anchorPoint2) *
                                 elasticities[1][1] * strength;
}

void Connector::updateAnchorToAccess() {
  static float elasticities[2][2];

  elasticities[0][0] = accessPoint2.X * accessPoint2.Y;
  elasticities[0][1] = accessPoint2.X_ * accessPoint2.Y;
  elasticities[1][0] = accessPoint2.X * accessPoint2.Y_;
  elasticities[1][1] = accessPoint2.X_ * accessPoint2.Y_;

  if (accessPoint2.cella)
    accessPoint2.cella->force -= (accessPoint2.cella->position - anchorPoint1) *
                                 elasticities[0][0] * strength;
  if (accessPoint2.cellb)
    accessPoint2.cellb->force -= (accessPoint2.cellb->position - anchorPoint1) *
                                 elasticities[0][1] * strength;
  if (accessPoint2.cellc)
    accessPoint2.cellc->force -= (accessPoint2.cellc->position - anchorPoint1) *
                                 elasticities[1][0] * strength;
  if (accessPoint2.celld)
    accessPoint2.celld->force -= (accessPoint2.celld->position - anchorPoint1) *
                                 elasticities[1][1] * strength;
}

void Connector::display() {
  if (!manager_->graphics_engine_)
    return;
  if (!manager_->graphics_engine_->active)
    return;
  if (manager_->synthesisEngine.tick % manager_->graphics_engine_->refreshRate != 0)
    return;

  Instrument &instr1 = accessPoint1.getInstrument();
  GLfloat x1;
  GLfloat y1;
  GLfloat z1;
  Instrument &instr2 = accessPoint2.getInstrument();
  GLfloat x2;
  GLfloat y2;
  GLfloat z2;

  if (accessPoint1.instrument) {
    if (manager_->graphics_engine_->displayDeviceNames) {
      x1 = (GLfloat)(instr1.getWorldX() + accessPoint1.cellx);
      z1 = (GLfloat)(accessPoint1.getPosition() * instr1.getMagnification() *
                         manager_->graphics_engine_->globalMagnification +
                     2.0);
      y1 = (GLfloat)(instr1.getWorldY() + accessPoint1.celly);

      manager_->graphics_engine_->displayCharString(x1, y1, z1, this->name, 1.0, 1.0,
                                           1.0);
    }
    manager_->graphics_engine_->displayAccessPoint(accessPoint1);
  }

  if (accessPoint2.instrument) {
    if (manager_->graphics_engine_->displayDeviceNames) {
      x2 = (GLfloat)(instr2.getWorldX() + accessPoint2.cellx);
      z2 = (GLfloat)(accessPoint2.getPosition() * instr2.getMagnification() *
                         manager_->graphics_engine_->globalMagnification +
                     2.0);
      y2 = (GLfloat)(instr2.getWorldY() + accessPoint2.celly);

      manager_->graphics_engine_->displayCharString(x2, y2, z2, this->name, 1.0, 1.0,
                                           1.0);
    }
    manager_->graphics_engine_->displayAccessPoint(accessPoint2);
  }
}

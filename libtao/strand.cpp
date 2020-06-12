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

#include <tao/strand.h>
#include <tao/instrument.h>
// #include <cassert>

using namespace tao;
String::String(std::shared_ptr<Manager> manager, const Pitch &pitch, float decay)
    : Instrument(manager, pitch, Pitch(0.0), decay) {
  createTheMaterial();
}

String::String(std::shared_ptr<Manager> manager, const std::string name, const Pitch &pitch,
    float decay)
    : Instrument(manager, name, pitch, Pitch(0.0), decay) {
  createTheMaterial();
}

void String::createTheMaterial() {
  int xsize = xmax + 1, ysize = ymax + 1;

  rows.resize(ysize);

  rows[0].xmax = xsize - 1;
  rows[0].offset = 0;
  rows[0].cells.resize(xsize);

  // assert(rows[0].cells != 0);

  initialiseCells();
  linkCells();
}

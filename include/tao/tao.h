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

#ifndef TAO_H
#define TAO_H

extern "C" {
#include <stdlib.h>
#include <unistd.h>
}

#include <tao/graphics_engine.h>
#include <tao/synth_engine.h>

class Tao { // : std::enable_shared_from_this<Tao> {
  friend class TaoOutput; // necessary because a TaoOutput object must
                          // be able to find out the audio sample rate
public:
  Tao();
  void seedRandomNumGen();
  void audioRateFunc(int (*functionPtr)(void));
  void initFunc(void (*functionPtr)(void));
  void scoreDurationFunc(float (*functionPtr)(void));
  void scoreFunc(void (*functionPtr)(void));
  void setAudioSampleRate();
  void setAudioSampleRate(int sr);
  void initInstrumentsAndDevices();
  void setScoreDuration();
  void setScoreDuration(float duration);
  void executeScore();
  void masterTick();
  void run();

  TaoSynthEngine synthesisEngine;
  std::shared_ptr<TaoGraphicsEngine> graphics_engine_;

private:
  void (*scoreFunctionPtr)(void); // User supplied functions
  void (*initFunctionPtr)(void);
  float (*durationFunctionPtr)(void);
  int (*audioRateFunctionPtr)(void);
  int audioRate;
  float scoreDuration;

  // using std::enable_shared_from_this<Tao>::shared_from_this;

};

void taoMasterTick();

#endif

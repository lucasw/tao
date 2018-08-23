//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "stop.tao".
//
// It contains automatically generated definitions for the following functions
// which are required by the Tao library in order to produce a complete
// executable:
//
//   int    taoAudioRate()	- returns the audio sampling rate in Hz.
//   float  taoScoreDuration()	- returns the duration of the score
//				  in seconds.
//   void   taoInit()		- this function is called just before execution
//				  of the score and contains user specified code
//				  for initialising variable values, devices,
//				  instruments and connections.
//   void   taoScore()		- this function is called once on every tick of
//				  the synthesis engine and contains all the code
//				  representing the user's time-domain inter-
//				  actions with the instruments and devices.
//
// The `main()' function defined at the end of this generated file registers
// the functions described above with the top level object `tao' (an instance
// of class `Tao'), and then invokes the member function `tao.main()'. This
// function enters the main synthesis engine loop which calculates the number
// of ticks specified by the score duration, and updates the graphics window
// (if graphics mode is on). It only exits if the graphics window is closed, if
// the ESC key is pressed whilst the graphics window has the mouse focus, if
// CTRL-C is pressed in the shell window from which Tao was invoked, or the
// `performance' reaches its natural conclusion.
//////////////////////////////////////////////////////////////////////////////////

#include "taodefs.h"
#include <cmath>
#include <iostream>

Tao tao;

// Audio rate: <sample_rate> ;

int taoAudioRate() { return 44100; }

// Declarations

TaoString string("string", TaoPitch(200.000f, TaoPitch::frq), 40.0000f);

TaoStop stop("stop");
float position, amount = 0.00000f;

// Init: <statements> ...

void taoInit() {
  string.lockEnds();
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 0.300000f; }

void taoScore() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = 0.000100000) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.00000) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    string(0.900000f).applyForce(10.0000f);
    tao.popStartAndEnd();
  }

  position =
      ((Time - tao.start) / (tao.end - tao.start) * (0.900000f - 0.100000f) +
       0.100000f);
  stop(string(position));
  if (Tick <= (long)((tao.newEnd = 0.100000f) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.0500000f) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    amount =
        ((Time - tao.start) / (tao.end - tao.start) * (1.00000f - 0.00000f) +
         0.00000f);
    stop.setAmount(
        ((Time - tao.start) / (tao.end - tao.start) * (1.00000f - 0.00000f) +
         0.00000f));
    tao.popStartAndEnd();
  }

  if (Tick <= (long)((tao.newEnd = 0.250000f) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.200000f) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    amount =
        ((Time - tao.start) / (tao.end - tao.start) * (0.00000f - 1.00000f) +
         1.00000f);
    stop.setAmount(
        ((Time - tao.start) / (tao.end - tao.start) * (0.00000f - 1.00000f) +
         1.00000f));
    tao.popStartAndEnd();
  }

  if (Tick % (long)(0.00500000f * tao.synthesisEngine.modelSampleRate) == 0) {
    tao.pushStartAndEnd2();

    std::cout << tao.synthesisEngine.time << " " << amount << std::endl;

    tao.popStartAndEnd();
  }

  tao.popStartAndEnd();
}

main(int argc, char *argv[]) {
  tao.initStartAndEnd();
  tao.audioRateFunc(taoAudioRate);
  tao.initFunc(taoInit);
  tao.scoreDurationFunc(taoScoreDuration);
  tao.scoreFunc(taoScore);
  tao.main(argc, argv);
}

// End of C++ program generated from script "stop.tao"

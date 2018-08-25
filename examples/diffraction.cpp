//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "diffraction.tao".
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

#include <tao/taodefs.h>
#include <cmath>
#include <iostream>

Tao tao;

// Audio rate: <sample_rate> ;

int taoAudioRate() { return 44100; }

// Declarations

TaoRectangle source("source", TaoPitch(150.000f, TaoPitch::frq),
                    TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoRectangle dest("dest", TaoPitch(150.000f, TaoPitch::frq),
                  TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

// Init: <statements> ...

void taoInit() {
  source.lockCorners();
  dest.lockCorners();
  source.lock(0.00000f, 0.0500000f, 1.00000f, 1.00000f);
  source.lock(0.0700000f, 0.120000f, 1.00000f, 1.00000f);
  source.lock(0.140000f, 0.190000f, 1.00000f, 1.00000f);
  source.lock(0.210000f, 0.260000f, 1.00000f, 1.00000f);
  source.lock(0.280000f, 0.330000f, 1.00000f, 1.00000f);
  source.lock(0.350000f, 0.400000f, 1.00000f, 1.00000f);
  source.lock(0.420000f, 0.470000f, 1.00000f, 1.00000f);
  source.lock(0.490000f, 0.540000f, 1.00000f, 1.00000f);
  source.lock(0.560000f, 0.610000f, 1.00000f, 1.00000f);
  source.lock(0.630000f, 0.680000f, 1.00000f, 1.00000f);
  source.lock(0.700000f, 0.750000f, 1.00000f, 1.00000f);
  source.lock(0.770000f, 0.820000f, 1.00000f, 1.00000f);
  source.lock(0.840000f, 0.890000f, 1.00000f, 1.00000f);
  source.lock(0.910000f, 0.960000f, 1.00000f, 1.00000f);
  source.lock(0.980000f, 1.00000f, 1.00000f, 1.00000f);
  TaoInstrument::join(source(0.500000f, 1.00000f), dest(0.500000f, 0.00000f));
  dest.setMagnification(5.00000f);
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 5.00000f; }

void taoScore() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = 0.000100000) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.00000) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    source(0.500000f, 0.00000f).applyForce(50);
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

// End of C++ program generated from script "diffraction.tao"

//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "lock.tao".
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

TaoRectangle rect1("rect1", TaoPitch(500.000f, TaoPitch::frq),
                   TaoPitch(600.000f, TaoPitch::frq), 20.0000f);

TaoRectangle rect2("rect2", TaoPitch(500.000f, TaoPitch::frq),
                   TaoPitch(600.000f, TaoPitch::frq), 20.0000f);

TaoRectangle rect3("rect3", TaoPitch(500.000f, TaoPitch::frq),
                   TaoPitch(600.000f, TaoPitch::frq), 20.0000f);

TaoRectangle rect4("rect4", TaoPitch(500.000f, TaoPitch::frq),
                   TaoPitch(600.000f, TaoPitch::frq), 20.0000f);

TaoRectangle rect5("rect5", TaoPitch(500.000f, TaoPitch::frq),
                   TaoPitch(600.000f, TaoPitch::frq), 20.0000f);

TaoRectangle rect6("rect6", TaoPitch(500.000f, TaoPitch::frq),
                   TaoPitch(600.000f, TaoPitch::frq), 20.0000f);

// Init: <statements> ...

void taoInit() {
  rect1.lockCorners();
  rect2.lockLeft().lockRight();
  rect3.lockTop().lockBottom();
  rect4.lockPerimeter();
  rect5.lock(0.200000f, 0.400000f);
  rect6.lock(0.700000f, 1.00000f, 0.700000f, 1.00000f);
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 5.00000f; }

void taoScore() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = 0.00100000) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.00000) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    rect1(0.100000f, 0.100000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 30.0000f) +
                     30.0000f));
    rect2(0.100000f, 0.100000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 50.0000f) +
                     50.0000f));
    rect3(0.100000f, 0.100000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 50.0000f) +
                     50.0000f));
    rect4(0.100000f, 0.100000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 50.0000f) +
                     50.0000f));
    rect5(0.100000f, 0.100000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 20.0000f) +
                     20.0000f));
    rect6(0.100000f, 0.100000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 30.0000f) +
                     30.0000f));
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

// End of C++ program generated from script "lock.tao"

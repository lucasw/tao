//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "damp.tao".
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

TaoString string1("string1", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string2("string2", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string3("string3", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string4("string4", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string5("string5", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string6("string6", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string7("string7", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string8("string8", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string9("string9", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

TaoString string10("string10", TaoPitch(300.000f, TaoPitch::frq), 20.0000f);

// Init: <statements> ...

void taoInit() {
  string1.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.00000f);
  string2.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.100000f);
  string3.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.200000f);
  string4.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.300000f);
  string5.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.400000f);
  string6.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.500000f);
  string7.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.600000f);
  string8.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.700000f);
  string9.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.800000f);
  string10.lockEnds().setDamping(0.00000f, 1 / 20.0, 0.800000f);
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
    string1(0.100000f).applyForce(10);
    string2(0.100000f).applyForce(10);
    string3(0.100000f).applyForce(10);
    string4(0.100000f).applyForce(10);
    string5(0.100000f).applyForce(10);
    string6(0.100000f).applyForce(10);
    string7(0.100000f).applyForce(10);
    string8(0.100000f).applyForce(10);
    string9(0.100000f).applyForce(10);
    string10(0.100000f).applyForce(10);
    tao.popStartAndEnd();
  }

  if (Tick % (long)(0.100000f * tao.synthesisEngine.modelSampleRate) == 0) {
    tao.pushStartAndEnd2();

    std::cout << tao.synthesisEngine.time << std::endl;

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

// End of C++ program generated from script "damp.tao"

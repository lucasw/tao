//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "ellipse2.tao".
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
#include <functional>
#include <iostream>

// this has to be global and named tao or undefined reference?
Tao tao;

class EllipseExample
{
public:
  EllipseExample()
  {
    ellipse2 = new TaoEllipse("ellipse2", TaoPitch(200.000f, TaoPitch::frq),
                              TaoPitch(400.000f, TaoPitch::frq), 20.0000f);
  }

int taoAudioRate() { return 44100; }

TaoEllipse* ellipse2;

void taoInit() {
  ellipse2->lock(0.00000f, 0.800000f, 0.500000f, 0.500000f);
}

float taoScoreDuration() { return 5.00000f; }

void update() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = 0.000500000) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.00000) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    (*ellipse2)(0.300000f, 0.200000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 30.0000f) +
                     30.0000f));
    tao.popStartAndEnd();
  }

  tao.popStartAndEnd();
}

  void run(int argc, char *argv[]) {
    #if 0
    tao.initStartAndEnd();
    // tao.audioRateFunc(std::bind(&EllipseExample::taoAudioRate, this));
    // auto f = [this](void*) {return taoAudioRate();};
    tao.audioRateFunc(f);
    tao.initFunc(taoInit);
    tao.scoreDurationFunc(taoScoreDuration);
    tao.scoreFunc(taoScore);
    tao.main(argc, argv);
    #endif
  }
};

int main(int argc, char *argv[]) {
  EllipseExample ellipse_example;
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "40strings.tao".
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

TaoString string[] = {
    TaoString("string[0]", TaoPitch(7.00000f, TaoPitch::pch), 60.0000f),
    TaoString("string[1]", TaoPitch(7.01333f, TaoPitch::pch), 60.0000f),
    TaoString("string[2]", TaoPitch(7.02666f, TaoPitch::pch), 60.0000f),
    TaoString("string[3]", TaoPitch(7.04000f, TaoPitch::pch), 60.0000f),
    TaoString("string[4]", TaoPitch(7.05333f, TaoPitch::pch), 60.0000f),
    TaoString("string[5]", TaoPitch(7.06666f, TaoPitch::pch), 60.0000f),
    TaoString("string[6]", TaoPitch(7.08000f, TaoPitch::pch), 60.0000f),
    TaoString("string[7]", TaoPitch(7.09333f, TaoPitch::pch), 60.0000f),
    TaoString("string[8]", TaoPitch(7.10666f, TaoPitch::pch), 60.0000f),
    TaoString("string[9]", TaoPitch(8.00000f, TaoPitch::pch), 60.0000f),
    TaoString("string[10]", TaoPitch(8.01333f, TaoPitch::pch), 60.0000f),
    TaoString("string[11]", TaoPitch(8.02666f, TaoPitch::pch), 60.0000f),
    TaoString("string[12]", TaoPitch(8.04000f, TaoPitch::pch), 60.0000f),
    TaoString("string[13]", TaoPitch(8.05333f, TaoPitch::pch), 60.0000f),
    TaoString("string[14]", TaoPitch(8.06666f, TaoPitch::pch), 60.0000f),
    TaoString("string[15]", TaoPitch(8.08000f, TaoPitch::pch), 60.0000f),
    TaoString("string[16]", TaoPitch(8.09333f, TaoPitch::pch), 60.0000f),
    TaoString("string[17]", TaoPitch(8.10666f, TaoPitch::pch), 60.0000f),
    TaoString("string[18]", TaoPitch(9.00000f, TaoPitch::pch), 60.0000f),
    TaoString("string[19]", TaoPitch(9.01333f, TaoPitch::pch), 60.0000f),
    TaoString("string[20]", TaoPitch(9.02666f, TaoPitch::pch), 60.0000f),
    TaoString("string[21]", TaoPitch(9.04000f, TaoPitch::pch), 60.0000f),
    TaoString("string[22]", TaoPitch(9.05333f, TaoPitch::pch), 60.0000f),
    TaoString("string[23]", TaoPitch(9.06666f, TaoPitch::pch), 60.0000f),
    TaoString("string[24]", TaoPitch(9.08000f, TaoPitch::pch), 60.0000f),
    TaoString("string[25]", TaoPitch(9.09333f, TaoPitch::pch), 60.0000f),
    TaoString("string[26]", TaoPitch(9.10666f, TaoPitch::pch), 60.0000f),
    TaoString("string[27]", TaoPitch(10.0000f, TaoPitch::pch), 60.0000f),
    TaoString("string[28]", TaoPitch(10.0133f, TaoPitch::pch), 60.0000f),
    TaoString("string[29]", TaoPitch(10.0267f, TaoPitch::pch), 60.0000f),
    TaoString("string[30]", TaoPitch(10.0400f, TaoPitch::pch), 60.0000f),
    TaoString("string[31]", TaoPitch(10.0533f, TaoPitch::pch), 60.0000f),
    TaoString("string[32]", TaoPitch(10.0667f, TaoPitch::pch), 60.0000f),
    TaoString("string[33]", TaoPitch(10.0800f, TaoPitch::pch), 60.0000f),
    TaoString("string[34]", TaoPitch(10.0933f, TaoPitch::pch), 60.0000f),
    TaoString("string[35]", TaoPitch(10.1067f, TaoPitch::pch), 60.0000f),
    TaoString("string[36]", TaoPitch(11.0000f, TaoPitch::pch), 60.0000f),
    TaoString("string[37]", TaoPitch(11.0133f, TaoPitch::pch), 60.0000f),
    TaoString("string[38]", TaoPitch(11.0267f, TaoPitch::pch), 60.0000f),
    TaoString("string[39]", TaoPitch(11.0400f, TaoPitch::pch), 60.0000f)};

TaoString resonator1("resonator1", TaoPitch(200.000f, TaoPitch::frq), 5.00000f);

TaoString resonator2("resonator2", TaoPitch(200.000f, TaoPitch::frq), 5.00000f);
int stringConnectionOrder[40] = {39, 37, 35, 33, 31, 29, 27, 25, 23, 21,
                                 19, 17, 15, 13, 11, 9,  7,  5,  3,  1,
                                 0,  2,  4,  6,  8,  10, 12, 14, 16, 18,
                                 20, 22, 24, 26, 28, 30, 32, 34, 36, 38};
int s, stringToPluck;

TaoConnector c[40] = {
    TaoConnector("c0"),  TaoConnector("c1"),  TaoConnector("c2"),
    TaoConnector("c3"),  TaoConnector("c4"),  TaoConnector("c5"),
    TaoConnector("c6"),  TaoConnector("c7"),  TaoConnector("c8"),
    TaoConnector("c9"),  TaoConnector("c10"), TaoConnector("c11"),
    TaoConnector("c12"), TaoConnector("c13"), TaoConnector("c14"),
    TaoConnector("c15"), TaoConnector("c16"), TaoConnector("c17"),
    TaoConnector("c18"), TaoConnector("c19"), TaoConnector("c20"),
    TaoConnector("c21"), TaoConnector("c22"), TaoConnector("c23"),
    TaoConnector("c24"), TaoConnector("c25"), TaoConnector("c26"),
    TaoConnector("c27"), TaoConnector("c28"), TaoConnector("c29"),
    TaoConnector("c30"), TaoConnector("c31"), TaoConnector("c32"),
    TaoConnector("c33"), TaoConnector("c34"), TaoConnector("c35"),
    TaoConnector("c36"), TaoConnector("c37"), TaoConnector("c38"),
    TaoConnector("c39")};

TaoConnector r2r[5] = {TaoConnector("r2r0"), TaoConnector("r2r1"),
                       TaoConnector("r2r2"), TaoConnector("r2r3"),
                       TaoConnector("r2r4")};

TaoOutput out("out", "40strings_out", 2);
float interval[] = {1,       1 / 2.0, 1 / 3.0, 1 / 4.0, 1 / 5.0,
                    1 / 6.0, 1 / 7.0, 1 / 8.0, 1 / 9.0, 1 / 10.0};
float now = 0.00000f;
float position, force;
float volume = 1.00000f;

// Init: <statements> ...

void taoInit() {
  resonator1.setDamping(0.00000f, 0.100000f, 0.200000f);
  resonator1.setDamping(1.00000f, 0.900000f, 0.200000f);
  resonator1.setMagnification(5.00000f);
  for (s = 0; s <= 39; s++) {
    string[s].lockEnds();
  }

  r2r[0](resonator1(0.100000f), resonator2(0.100000f));
  r2r[1](resonator1(0.300000f), resonator2(0.300000f));
  r2r[2](resonator1(0.500000f), resonator2(0.500000f));
  r2r[3](resonator1(0.700000f), resonator2(0.700000f));
  r2r[4](resonator1(0.900000f), resonator2(0.900000f));
  resonator1.placeRightOf(string[32], 40);
  resonator2.placeAbove(resonator1, 30);
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 30.0000f; }

void taoScore() {
  tao.initStartAndEnd();

  if (Tick <
      (long)(tao.newStart = tao.start,
             (tao.newEnd = 21.0000f) * tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();

    if (Tick <= (long)((tao.newEnd = now * 1.00000 + 0.000500000f) *
                       tao.synthesisEngine.modelSampleRate) &&
        Tick >= (long)((tao.newStart = now) *
                       tao.synthesisEngine.modelSampleRate)) {
      tao.pushStartAndEnd1();
      if (Tick ==
          (long)(tao.start * 1.00000f * tao.synthesisEngine.modelSampleRate)) {
        tao.pushStartAndEnd2();
        stringToPluck = randomi(0, 39);
        position = randomf(0.00000f, 1.00000f);
        force = randomf(5.00000f, 10.0000f);
        std::cout << "plucking string " << string[s].getName();

        std::cout << " with force " << force;

        std::cout << " at position " << position;

        std::cout << " at time " << tao.synthesisEngine.time << std::endl;

        tao.popStartAndEnd();
      }

      string[stringToPluck](position).applyForce(force);
      if (Tick ==
          (long)(tao.end * 1.00000f * tao.synthesisEngine.modelSampleRate)) {
        tao.pushStartAndEnd2();
        now += interval[randomi(0, 9)];
        tao.popStartAndEnd();
      }

      tao.popStartAndEnd();
    }

    tao.popStartAndEnd();
  }

  if (Tick % (long)(0.0100000f * tao.synthesisEngine.modelSampleRate) == 0) {
    tao.pushStartAndEnd2();

    std::cout << "Time=" << tao.synthesisEngine.time << std::endl;

    tao.popStartAndEnd();
  }

  if (Tick % 1000L == 0) {
    tao.pushStartAndEnd2();
    for (s = 0; s <= 39; s++) {
      c[s](string[stringConnectionOrder[s]](((Time - tao.start) /
                                                 (tao.end - tao.start) *
                                                 (0.500000f - 0.0500000f) +
                                             0.0500000f)),
           resonator1((s + 10) / 60.0000f), 0.200000f);
    }

    tao.popStartAndEnd();
  }

  if (Tick == (long)(0.00000f * tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd2();
    volume = 1.00000f;
    tao.popStartAndEnd();
  }

  if (Tick <= (long)((tao.newEnd = 30.0000f) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 25.0000f) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    volume =
        ((Time - tao.start) / (tao.end - tao.start) * (0.00000f - 1.00000f) +
         1.00000f);
    tao.popStartAndEnd();
  }

  out.chL(resonator2(0.0500000f) * volume);
  out.chR(resonator2(0.950000f) * volume);

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

// End of C++ program generated from script "40strings.tao"

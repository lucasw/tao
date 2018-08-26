		
The Tao Physical Modelling Sound Synthesis Program
--------------------------------------------------

Introduction
------------

Tao is a software package for sound synthesis using physical models. It
provides a virtual acoustic material based on point masses connected
together with springs from which a wide variety of virtual musical
instruments can be constructed. Tao can be used either as a stand-alone
tool or as a C++ library for those who wish to incorporate its
functionality into their own C++ programs or don't want to use the user
interface provided.

Tao provides various objects such as bows, hammers, connectors and
outputs for exciting the instruments, coupling them together and
generating sound output. One of the main features of Tao is its 3-D
graphics visualisations of the instrument constructed, showing how the
acoustic waves propagate through the instruments.

Features
--------

The main features of Tao are:

- A physically modelled elastic material from which a wide variety
of virtual musical instruments can be constructed.

- The ability to produce very high quality `organic' sounds, i.e.
sounds which are much more `acoustic' than those usually produced
by digital synthesis.

- A set of virtual `devices' such as Bows, Hammers, Connectors and
Outputs for coupling together, exciting and generating output from
instruments.

- Real-time visualisations of the instruments showing in detail how
the waves propagate through the various componnents.

- A well documented C++ API for those who want to develop their own
programs making use of Tao's functionality (actually it is not
documented yet but that part is the next priority).

Requirements
------------

Tao has been built and tested on Ubuntu 16.04.
It requires X windows and OpenGL
compatible libraries and headers to be installed, either true OpenGL
or a clone such as Brian Paul's Mesa3D, and Mark Kilgard's GLUT (GL
Utility Toolkit) library and headers. It also requires Michael Pruett's
port of the SGI audiofile library in order to write WAV format soundfiles.

Finally it requires the following tools in order to build it from source.

    A C++ compiler      (preferably gnu g++ but the code should
                         compile perfectly well under others)
    CMake               (build tool)
    Doxygen             (a tool for generating documentation from
                         C++ sources)
    Hyperlatex          (a tool for generating multi-format
                         documentation from LaTeX sources)

You only need Doxygen and Hyperlatex if you intend to build the
documentation from the sources.

Tao works with both OpenGL and Mesa3D since the rendering used does
not rely upon any advanced or esoteric features.

Steps for installation
----------------------

Get the latest source with git:

```
git clone https://github.com/lucasw/tao_synth.git
mkdir build_tao_synth
cd build_tao_synth
cmake ../tao_synth -DCMAKE_INSTALL_PREFIX=<your path>
make
# make install TBD
```

Make your path somewhere on your system that your user has write
permissions to and that is on your PATHs, this is an example of
a bashrc configured to have a user install space in ~/other/install:

```
export DEST_DIR=$HOME/other/install
export CPATH=$CPATH:$DEST_DIR/include
export PYTHONPATH=$PYTHONPATH:$DEST_DIR/lib/python/
export LD_LIBRARY_PATH=$DEST_DIR/lib:$DEST_DIR/usr/lib:$DEST_DIR/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH
export PATH=$DEST_DIR/bin:$PATH
export PKG_CONFIG_PATH=$DEST_DIR/lib/pkgconfig:$DEST_DIR/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$DEST_DIR/lib/cmake
```

Leaving out the prefix will require root permissions for
make install, but you shouldn't really install random software as
root on your system.

The `cmake' part checks to see if you have the necessary programs
headers and libraries installed. If you do not the configuration will
abort with a message telling you what is missing. See the earlier
part of this README for details on where to get the components you
need in order to use Tao.


Troubleshooting the configuration process
-----------------------------------------

TBD

Continuing with the build process
---------------------------------

Assuming the `make' works OK you should now
have libtao.so and a directory full of example executables.

Testing Tao
-----------

Run an example to see if tao is working, first with visualization then without:

```
examples/strand -g
```

You should then see Tao's instrument visualisation window open. When
this window opens initially Tao is in 'pause' mode. This gives you
time to move, rotate and zoom the image before setting the synthesis
in motion. To unpause Tao press the [right-arrow] cursor key. This
should set the instrument in motion. For more information on what to
do next refer to the User Manual.

```
examples/strand
```

This should run through the visualization rapidly then output one or more .dat
files that can be converted to wav files for listening.

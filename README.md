This repository contains preliminary implementations of the Sketch Algorithm. In this simulation, we don't include any physics or control laws.

Below are descriptions of what various files represent:

render.py - Renders the output from the output file (sketch_plot.txt) of one of the C++ code into a visual showing the path of the two robots. Currently overlayed on the contour plots generated from the data in the data folder. 

sketchalgorithmautogaussian.cpp - The Sketch Algorithm for Gaussians, to run this code just compile using any C++ compiler, we recommend g++ without any flags.

This should be compiled using:
clang++ -std=c++17 -g -O0 -I<path-to-eigen3> sketchalgorithmautogaussian.cpp


Details on input format:

The variable num in line 1417 indicates the number of gaussians to be included in the plume. Then gaussianVar and gaussianCenter are two vectors that
will contain the variances.

The polygon is given as a sequence of successive vertices in the file sketchinput.txt and then it outputs the result (the path of the two robots) in sketch_plot.txt 


Other Render Files:
renderfinal.py  and renderinput.py have not been modified for the general
sketch algorithm.

renderinput.py - renders a good image of a polygon and outputs the set of its vertices in successive sequence. Copy this in a file named "sketchinput.txt" after

For debugging purposes:
render.py renders gaussians to visualize the input


Summary:

To run gaussian test cases - compile and run sketchalgorithmautogaussian.cpp and then feed the output to render.py

Finally, one can tweak with the epsilon values in the main function however they wish upto the algorithm specifications (see the paper).


New Changes
src/ contains src files
compile main.cpp for executable equivalent to sketckalgorithmautogaussian.cpp

compile pysketch.cpp to use pybind11 to create a python shared object
use this python module to control the simulation

/examples
  contains pybind11 example code and howto guide

environment setup
load your virtual environment if you use one
pip install pybind11

environment setup with spack
(macos system python is not compatible with pybind11)
spack env create sketch
spack env load sketch
spack env add python py-pip python-venv py-pybind11 eigen
spack concretize
spack install
spack env deactivate
spack env activate sketch

use python virtual environment to install additional dependencies such as matplotlib
source otherenv/bin/activate
pip ...

then in the source directory compile with
c++ -O3 -Wall -shared -std=c++11 -undefined dynamic_lookup $(python3 -m pybind11 --includes) pysketch.cpp -o pysketch$(python3-config --extension-suffix)

python
import pysketch
help(pysketch)

if ctrl+c doesn't kill pysketch use ctrl+z and kill to stop the process

General notes on environment
if eigen is not found add the argument -Ipath/to/eigen to the compile statement

if pybind11 is not found modify the compile statement with
(python3-config --includes) -Iextern/pybind11/include instead of $(python3 -m pybind11 --includes)

If you have to specify the path to both eigen and pybind11 try making a directory and linking both locations to that directory then specify the path to said directory in the compile statement


What's next:
    we need to close file pointers before the simulation exits
    figure out why the simulation doesn't terminate even with one gaussian
    ^^it looks like the y coordinate of the drones may not be getting updated appropriately

    create a makefile!


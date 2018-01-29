# PID Controller
Self-Driving Car Engineer Nanodegree

This project aims to implement a PID controller to set a car to drive itself around a race track.

## Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build and Run Instructions
To build the program, run the following sequence of command line instructions.

```bash
$ cd /path/to/cloned/repo
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To see the program in action, make sure to download the [Term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases), run it, and choose the third simulation option. Then, execute the program by executing the `pid` program in the `build` folder.

```bash
$ ./pid
```

## Project Information
The PID controller allows a car to drive itself by identifying ways to correct the car's steering. It comes in 3 parts.

The P part of the PID controller is short for proportional. This part identifies the cross-track error (abbr. CTE, def. the perpendicular distance away from the target trajectory). This term will tell the car to adjust its steering toward the opposite direction to get closer to the trajectory. Although this update works, it poses a problem once the car gets close enough that it overshoots the trajectory. Once this happens, the car will continously undesirably oscillate around the trajectory.

The D part of the PID controller is short for differential. This part identifies the change in the CTE and helps the car adjust its steering to avoid the oscillation problem.

The I part of the PID controller is short for integral. This part looks at the entire history of the car's CTE to adjust for systematic bias. One potential systematic bias is if the car's steering isn't properly aligned, causing it to drift towards a location away from the target trajectory.

Each part of the PID controller also takes a coefficient to adjust how much each part affects the vehicle's driving behavior. This will have to be tuned manually or by using an algorithm that automatically tunes it while the car is driving.

## Project Implementation Details
The `src/pid.cpp` and `src/main.cpp` files were modified for this project.

The `src/pid.cpp` file implements the PID controller as is specified in the course.

The `src/main.cpp` file adds code that sets the PID controller's coefficients. It also provides code manually test different coefficients without having to compile the program with new coefficient values each time a new set of values are decided.

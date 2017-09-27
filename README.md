# Model Predictive Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## The Project

In this project the goal is to implement a Model Predictive Control to drive the car around the track. The cross track error isn't provided by the simulator and there's a 100 millisecond latency between actuations commands on top of the connection latency.

### The goals / steps of this project are the following:

* Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.
* Implement the MPC calculation, including setting variables and constraints.
* Calculate actuator values from the MPC calculation based on current state.
* Account for latency.
* Calculate steering angle & throttle/brake based on the actuator values.
* Adjust Timesteps and Timesteps Duration values after testing on Udacity simulator.


## Implementation

### The Model

#### Websocket Data

This document describes the JSON object send back from the simulator command server.

Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.

`psi` and `psi_unity` representations

`psi`

```
//            90
//
//  180                   0/360
//
//            270
```


`psi_unity`

```
//            0/360
//
//  270                   90
//
//            180
```


### Fitting based on road waypoints and evaluating the current state

First, I transformed the points from the simulator's global coordinates into the vehicle's coordinates using standard 2d vector transformation.
Using the `polyfit()` function, a third-degree polynomial line is fit to these transformed waypoints, drawing the target path.
From the car perspective we can assume that it is always at the center of the coordinate system and always pointing to a zero orientation. The cross-track error can be calculated by evaluating the polynomial function (`polyeval()`) at px, which is now zero.
Calculating epsi from the derivative of polynomial fit line is a lot easier since polynomials above the first order are all eliminated through multiplication by zero.


### Model Predictive Control

MPC::Solve
The variable (state) is the initial state [x,y,ψ,v,cte,eψ], coeffs are the coefficients of the fitting polynomial. The bulk of this method is setting up the vehicle model constraints (constraints) and variables (vars) for Ipopt.

FG_eval()
First it creates cost functions for each of the variables and sets the weights to be able to influence the importance of each.
For example, lower weights related to cte and epsi lead to the model not focusing enough on staying near the center of the road and turning correctly, the (velocity) cost is there to prevent the car from stoping and the costs related to (delta) and (a), and especially the costs related to the changes of those values, are important as well. Putting more weight to (delta_change) helps the ride to be much smoother.
Finally, the updated cost constraints are calculated by first calculating the states at time t and time + 1. These states are then put through the update equations, such as the given y cost constraint being equal to y1 - (y0 + v0 * sin(psi) * dt). This, along with the variables and constraints calculated earlier, can be fed to the `ipopt` solver. This solver takes in all the information and will calculate the future predicted states, which also includes updated (delta) and (a) values that I use for my actuator values.


### Latency

The simulator added 100ms latency between the actuator calculation and the time the simulator performs that action.
The best way to account for this issue was to add a step to predict where the vehicle would be after 100ms, in order to take the action that needed to actually be taken at that time, instead of the one in reaction to the data from the simulator.


### Tuning Timesteps (N) and Timestep Duration (dt)

Finally, after visualizing the model in the simulator, it was time to adjust (N) and (dt). I started with values of 5 for (N) and 0.2 for (dt). The car wasn't able to stay on the track until I set (N) to 10 and the car was able to complete the track. I tried increasing even more the (N) value but the car was slowing down so I decided to leave it at 10. After toying with (dt) I noticed that 0.2 was too slow to react so I set (dt) to 0.1. Considering that after setting desired speed to 130 the car was able to complete the lap I decided to leave those variables like that.


## Results
After increasing the desired speed to 130 and pushing the car to the limit I was very pleased to see how smooth the handling was. This was one of the most rewarding projects of this second term of the NanoDegree.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

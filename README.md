# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Mpc optimizer
 The main goal of the project is to implement in C++ Model Predictive Control to drive the car around the track using a simple Global Kinematic Model that ignore tire forces, gravity, and mass and consider state(posiions, speed and orientatation) and actuator(throttle and steering) vector inputs.  

![Kinematic model ](/images/global_kinematic_model.png)

This simplification reduces the accuracy of the model making it more tractable. The model developed equations determine the *next state (state at t+1)* from our *state vector at t* and our actuator values. It's added a variable to our state called *L​f*​​ which measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle , the slower the turn rate.  
So Model Predictive Control involves simulating different actuator inputs predicting the resulting trajectory and selecting the one with minimum cost. Known the current state and the reference trajectory to follow, at each time step we optimize different actuator inputs in order to minimize the cost of our predicted trajectory. Once it is found the lowest cost trajectory and implemented the set of actuation commands this brings the vehicle in the new state used to calculate a new optimal trajectory, then in the same way the process is repeated and new inputs are calculated over new horizons(The Receding Horizon Control Principle)


## The Model

![MPC Setup ](/images/model_predictive_control_setup.png)

 First set up the MPC loop:
* Define the duration *T* of the trajectory by chosing the number of timesteps in the horizon *N* and time elapses between actuations *dt*;
* Define the vehicle model and constraints on the actuators *[δ,a]*;
* Define the cost function;

![MPC Loop ](/images/model_predictive_control_loop_12.png)

Then is called the optimizarion solver:
* The solver uses the initial state, model constraints and cost function returning a vector of control inputs that minimize the cost function;
* We apply the first control input *[δ1,a1]* to the vehicle and the repeat the loop;  

The Solver used in this project is called IPOPT, short for "Interior Point OPTimizer", a software library for large scale nonlinear optimization of continuous systems

## Timestep Length and Elapsed Duration (N & dt)
Have a large N would be nice but is computationally critical for the the system because the solver will take more time to compute the solution, experimentally a good choice for N is 10, with N = 20 the cost taking longer to compute and at there is a high probability that the car at some point will come out of the track. Same computional problem with dt, but this time if it is too small, good choice for dt is 0.1

## Polynomial Fitting and MPC Preprocessing
The reference trajectory is passed to the control block as a 3rd order polynomial. In the project code polynomial is fitted with waypoints (x, y) in C++ using Eigen. Have been implemented to functions:
* *polyfit* (line 38 in file main.cpp) to fit a 3rd order polynomial to the given x and y coordinates representing waypoints.
* *polyeval* (line 49) to evaluate y values of given x coordinates.

## Model Predictive Control with Latency
To simulate the 100 ms delay you can set in the main the same state transition function used in file mpc.cpp. In main.cpp I defined a function *globalKinematicLatency* at line 75 and called every time at line 173 as first state implementation. Using dt = 0.1 it's like forwarding the state in time by 100 ms

## Final video
Video of the MPC solution tested on the simulator
https://www.youtube.com/watch?v=KeM_CI4eRAA

---

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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Reflection

PID control is very useful control method in case the engineer does not have a mathematical model of the system for more detailed techniques such as optimal control or pole placement. The technique creates an actuation signal based on the Proportional, Integral and Derivatives of the tracking error :

1. The proportional gain acts on the instantaneous error of the system. If the gain is sufficiently positive the actuator tend to stabilize the system.
2. The integral component is used to eliminate the steady-state error, or bias. In the case of the driving car, this component helped speed up the recovery during curves, as the tracking error accumulates.
3. The derivative component acts as a "predictive" component that reacts faster than the proportional component in case of a sudden increase in error. This component makes the system converge to the reference trajectory faster and also dampens the oscillations.

The classical tuning methods are based on the following steps:

1. Determine the Kp coefficient so that the system has a fast response. Usually this is accomplished by increasing the proportional gain until the system is marginally stable, i.e., oscillates. 
2. Determine the Ki coefficient so that the steady state error vanishes. Usually the Ki coefficient is fairly small.
3. Finally, increase the Kd coefficient until the rise time and overshoot are acceptable. Caution is needed to avoid destabilization of the system with too high Kd.

In this project, I first set the throttle to 0.3 and searched for all the parameters using the Twiddle algorithm (Kp = 0.035, Ki = 0.0020 and Kd = 2.8). To speed up the calibration phase, I restarted the simulation whenever the car left the road.  Once this was set I tuned the throttle controller so that the car would seek a speed of 92mph, but once the car hit the target speed it no longer completed the track, failing the first turn after the bridge. The car was too slow at the increased speed.

So I ran the twiddle algorithm once again and both the proportional and integral gains started increasing. After a large number of runs the simulator application had communication problems with the controller, so I fine tuned the parameters by hand, noticing that I had to actually increase the first two gains.

I made the car faster to respond by increasing the proportional and derivative gain, but I also made the car oscillate heavily when trying to center the trajectory at the bridge. To dampen the oscillation, I increased the integral gain until the car was able to complete the first turn and it also was able to complete the full track many times. The final parameters are presented in the code.

During the tuning process it became clear that the best parameters for the steering control changed according to the target velocity. The final parameters in the code are very well suited to speeds of 92 mph, but the car is marginally stable during the first acceleration phase. One solution to this problem is to use a lookup table for the PID parameters and automatically swtich them according to the car's speed.

---
# Compilation and run instructions

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Optimizer mode

This project implements the "Twiddle algorithm" [https://www.youtube.com/watch?v=2uQ2BSzDvXs] for tuning the controller parameters. To run the optimizer, create a twiddler object setting the first argument as true.
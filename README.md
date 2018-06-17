# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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

## Reflection
* Describe the effect each of the P, I, D components had in your implementation.

The P, proportional, is the component that causes the car to steer proportional to the error(CTE). This controller will bring large overshoot and cause frequently oscillation. 

The D, differential, is the component that reduce the overshotting and oscillation in the prportional part. It can make the car approch the center lane more smoothly.

The I, integral, is the component that avoid the bias of the system which make car cannot stay on the center lane with only PD controller. This component accumulate the history error and let the controller respond by a stronger action, so the car can approach the center lane faster.

*  Describe how the final hyperparameters were chosen.

In the project I chose the parameters in two steps, 

1. Mannully adjust the parameters to let the car can be on the lane without runing out.

Here is the steps I did in mannul adjust.

   1) Set Kp, Ki, Kd to be 0.
   
   2) Increase Kp to see the car is osillating.
   
   3) Increase Ki to agaist some offset.
   
   4) Increase Kd to let the system be stable.
   
2. Use Twiddle to further optimize the parameters

   1) Use Twiddle function in the source code
   
   2) Start simulator to let parameters adjusting start
   
In the end I got the Kp, Ki, Kd as ``[0.128674, 8.92023e-6, 3.5673]``

##Simulation

Here is the simulation of the PID controller, PD controller, ID controller and PI controller.


[PID controller](https://github.com/x327397818/UDC-Term2-project4/blob/master/output/PID.mp4)

[PD controller](https://github.com/x327397818/UDC-Term2-project4/blob/master/output/PD.mp4)

[ID controller](https://github.com/x327397818/UDC-Term2-project4/blob/master/output/ID.mp4)

[PI controller](https://github.com/x327397818/UDC-Term2-project4/blob/master/output/PI.mp4)

As you can see from the videos, only PID and PD controllers can run the whole loop of the track. So P and D component are essencial to make the system stable. And I component is more related to bias eliminating.


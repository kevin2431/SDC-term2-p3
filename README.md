# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Introduction
This project is to implement a PID controller in C++ to maneuver the vehicle around the track. The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle. 

## PID Algorithm
### Describes the effect of the P, I, D component
* Term **P** is proportional to the current value CTE. For example, if the error is large and positive, the control output will be proportionately large and positive.
* Term **I** accounts for past values of the CTE and integrates them over time to produce the I term. For example, if there is a residual CTE after the application of proportional control, the integral term seeks to eliminate the residual error by adding a control effect due to the historic cumulative value of the error. 
* Term **D** is a best estimate of the future trend of the CTE, based on its current rate of change. The more rapid the change, the greater the controlling or dampening effect.

### How to chose the final hyperparameters
In order to have a good performence on the track, we should choose proper P, I, D coefficients. However, it is a trick problem. Instead of manaul tuning, I choose **twiddle** algorithm to automatically fine tune the parameters.

Here is the pseudo code of twiddle, and I implement C++ code in `main.cpp`.
```
# Choose an initialization parameter vector
p = [0, 0, 0]
# Define potential changes
dp = [1, 1, 1]
# Calculate the error
best_err = A(p)

threshold = 0.001

while sum(dp) > threshold:
    for i in range(len(p)):
        p[i] += dp[i]
        err = A(p)

        if err < best_err:  # There was some improvement
            best_err = err
            dp[i] *= 1.1
        else:  # There was no improvement
            p[i] -= 2*dp[i]  # Go into the other direction
            err = A(p)

            if err < best_err:  # There was an improvement
                best_err = err
                dp[i] *= 1.05
            else  # There was no improvement
                p[i] += dp[i]
                # As there was no improvement, the step size in either
                # direction, the step size might simply be too big.
                dp[i] *= 0.95
```
### Final hyperparameters
At first, set `p =[ 1,0.01,0.5]`, `dp = [0.5,0.001,0.5]`, and `threshhold = 0.02`.

In this projection, the throttle is a constant `0.4`. To get a better performce, it maybe use another PID controller to control the speed!

After iteration, it converge to the result below
```
Kp = 0.552 
Ki = 0.00001
Kd = 8.1577
```

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


# CarND-Controls-MPC
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
* In addition, for Mac user, I noticed an error related to 'coin/IpIpoptApplication.hpp' during building. I found the solution from Slack, which is as following: 

Enter `export HOMEBREW_NO_AUTO_UPDATE=0` to reset homebrew to allow for updates. Then,  `brew update`.
Then, run `brew uninstall ipopt` in case it's somewhere on your computer.
Next, run `brew install ruby`. If it's already there, run `brew upgrade ruby`.
`cd /usr/local/Homebrew/Library/Taps/homebrew/homebrew-science` ​ (or your own related directory)
`git checkout 19f75951641d3a5e70ea105f76a6a77bc0553d07`
`export HOMEBREW_NO_AUTO_UPDATE=1`
`cd ..` (You should now be back into the main homebrew directory instead of homebrew-science)
`git checkout 93a2e9fc25407b049d594ad2da112a5cb8bdf5c3`
`brew tap homebrew/science`
`brew install ipopt --with-openblas` 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## MPC Controller

MPC stands for Model Predictive Controller. This project is implemented by using a simple Global Kinematic model, which considers position (x, y), heading direction (ψ), and velocity (v), but ignores gravity, tire forces, etc. In addition, two actuators, stearing angle (δ) and throttle (a) are used in this model. The stearting angle is in a range of [-25, 25] degree. The negative value of throttle means braking, and the positive values indicates accelerating. 

The model uses follow equations to predict the vehicle current state and actuators based on their previous values in the last timestep.

x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = cte[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = epsi[t-1] + v[t-1] * delta[t-1] / Lf * dt

Lf stands for the distance between the front of the vehicle and its gravity center. CTE is the short form of cross track error, and epsi is the error of heading direction (ψ).

The values of N and dt are set to 10 and 0.1, respectively (These values are from the Udacity Q&A video). The meaning of these values is that the optimizer is taking account 1 second of duration into determing trajectory. Moreover, faster speed usually requires longer duration, or greater N value. 

A third order polynomial fitting is used for waypoints, which are transformed to the vehicle's coordinates. The parameters of the cost function is determined by trying different values, using those values from the Udacity Q&A video as refereced. The final video is [here](https://youtu.be/YRigWV7o3Hw).

A interesting finding is when trying to record my screen via Quicktime, I notice that it could affect the MPC controller performance. Therefore, I have to use my phone to record the video. One possible reason for that is due to the limited hardware of me laptop. 

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

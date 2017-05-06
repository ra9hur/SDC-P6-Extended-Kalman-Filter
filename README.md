# Extended Kalman Filter Project

This project is about implementing Extended Kalman Filter (EKF) to locate objects (i.e. a bicycle) in real-time, using a data inputs from multiple sensors such as Laser and Radar. 

This is a 3-step process-

1. To predict position and velocity with some uncertainty. Here, we are predicting information based on current belief.
2. Then measure position and velocity with some uncertainty using sensors.
3. Finally, increase the certainty of our prediction by combining our prediction with the measurement information. Here belief is being updated with new information.

Simulated lidar and radar measurements are provided to detect a bicycle that travels around the vehicle. While laser sensors represent measurements in cartesian coordinate, the radar sensors represent in polar coordinate. A direct conversion of polar coordinate to cartesian coordinate gives nonlinearity and so Kalman filter is no more useful. Hence, we have to be able to linearly map from the polar coordinate to the cartesian coordinate. 

EKF uses the method called first order Taylor expansion to obtain linear approximation of the polar coordinate measurements. A Jacobian matrix is used, which represents linear mapping from polar to cartesian coordinate.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Extended Kalman Filter - Visualization

![ekf_visualization](https://cloud.githubusercontent.com/assets/17127066/25347974/4afe592a-293a-11e7-844d-c8854c8a3aa3.png)

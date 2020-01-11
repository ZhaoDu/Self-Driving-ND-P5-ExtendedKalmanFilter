# Extended Kalman Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

In this project, a simple extended kalman filter(EKF) was built to fusion data collected from radar and laser sensors to estimate position and velocity for self driving cars. A demo tested by [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases) was shown as follows:

|Demo                 |
|:-------------------:|
![left][image1]       |

## Dependencies
- Ubuntu
- [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases)
- [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
- [cmake >= 3.5](https://cmake.org/install/)
- make >= 4.1 
- gcc/g++ >= 5.4

*`intsall-linux.sh` file can be run to garther all the dependencies.

## Basic Build Instructions
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./ExtendedKF `

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Results
The following demo videos show the test results. In this demo, the RMSE of px,py,vx,vy was examined to check the performance of the implementation. The RMSE values are stated in the right of each video.
|Dataset 1            |Dataset 2            |
|:-------------------:|:-------------------:|
|![left][image1]      |![left][image2]      |

## Tips

### 1. How does the Extended Kalman Filter Work

![ekf_flow][image3]
### 2. Extended Kalman Filter V.S. Standard Kalman Filter
![ekf_vs_kf][image4]
* _x_ is the mean state vector.
* _F_ is the state transition function.
* _P_ is the state covariance matrix, indicating the uncertainty of the object's state.
* _u_ is the process noise, which is a Gaussian with zero mean and covariance as Q.
* _Q_ is the covariance matrix of the process noise.
---------------------------------------------------------
* _y_ is the innovation term, i.e. the difference between the measurement and the prediction. In order to compute the innovation term, we transform the state to measurement space by measurement function, so that we can compare the measurement and prediction directly.
* _H_ is the measurement function.
* _z_ is the measurement.
* _R_ is the covariance matrix of the measurement noise.
* _I_ is the identity matrix.
* _K_ is the Kalman filter gain.
* _Hj_ and _Fj_ are the jacobian matrix.

**All Kalman filters have the same three steps:**

1. Initialization
2. Prediction
3. Update
![ekf_vs_kf][image5]

A standard Kalman filter can only handle linear equations. Both the Extended Kalman Filter (EKF) and the Unscented Kalman Filter (UKF will be disuccsed in the next project) allow you to use non-linear equations; the difference between EKF and UKF is how they handle non-linear equations: Extended Kalman Filter uses the Jacobian matrix to linearize non-linear functions; Unscented Kalman Filter, on the other hand, does not need to linearize non-linear functions, insteadly, the unscented Kalman filter takes representative points from a Gaussian distribution.

[//]: # (Image References)
[image1]: ./img/test.gif "test"
[image2]: ./img/test-2.gif "test"
[image3]: ./img/ekf_flow.jpg "ekf_flow"
[image4]: ./img/ekf_vs_kf.jpg "ekf_vs_kf"
[image5]: ./img/kf_flow.png "kf_flow"




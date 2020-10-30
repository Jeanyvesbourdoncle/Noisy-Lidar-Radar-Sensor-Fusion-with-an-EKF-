
### Position and Velocity Estimation of a bicycle using the Extended Kalman Filter with noisy lidar and radar data measurements.
The target is to obtain acceptable RMSE values.
The RMSE Values are the difference values for the position  px,py and the velocity vx,vy between :
- the ground truth,
and 
- the sensor data (radar+lidar) fusion.

--- 

The use of an extended kalman filter is useful, because the bicycle is moving along a circular path. In this case, the regular Kalman Filter will underestimate the truth state.

The Lidar uses carthesian coordinates and the radar ues polar coordinates. Consequently, the use of a regular KF for the lidar application is acceptable. In the second case (radar), the regular KF is not acceptable because we don't have a linear estimation (the fonction h'(x) for the polar coordinates transformation is not linear) and we cannot work with a gaussian coordinates. The Jacobian matrix must be implemented for the use of the EFK for the radar estimation. 

You can find here the sensor fusion overview with the differents steps of the pipeline :
![Sensor Fusion Overview](https://github.com/Jeanyvesbourdoncle/Noisy-Lidar-Radar-Sensor-Fusion-with-an-EKF-/blob/master/Sensor_Fusion_Overview.png)

---

### SW Preparation/Installation:
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The files are in ./src :
- FusionEKF.cpp, 
- FusionEKF.h, 
- kalman_filter.cpp, 
- kalman_filter.h, 
- tools.cpp, 
- tools.h.

Every C++ file are very good commented. The basic math knowledge :
- the jacobian matrix for the partial derivative for the radar estimation,
- the RMSE calculation
are very good detailled.

The program main.cpp has already been filled out. The main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

---

#### Input: values provided by the simulator to the c++ program
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

#### Code : 
C++ Code with the regular and the Extended Kalman Filter Implementation (update and prediction implementation)
C++ Code for the sensor fusion algorithm to track the bicycle'position and velocity

#### Output: values provided by the c++ program to the simulator
["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

### Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

---

### SW architecture
4 important .cpp files :

1. tool.cpp : 
the implemented functionnalities in this module are :
- the RMSE calculation to measure the accuracy between the predict state and the real state for px,py, vx,vy
- the Jocabian matrix for the EKF (radar) : it's a matrix of partial derivative with the parameters rho,phi,rho_dot and px,py, vx,vy
	
2. Kalman_filter.cpp:
the implemented functionnalities in this module are :
- predict() for the radar and the lidar (same equation)
- update() for the lidar : you can find here the equation for a standard FK
- updateEFK() for the radar : you can find here the equation for an extanded FK
        
3. FusionEKF.cpp :
the implemented functionnalities in this module are :
- the initialization of all the matrix (for the prediction step ans the measurement step)
- the initialization of the KF with the first radar measures (polar coordinates) or the first lidar measures (carthesian 		coordinates)

3a : predict step :
- F_ (state transition matrix) modification with the dt (time between 2 measures)
- Q_ (process covariance matrix,acceleration/velocity noise )modification with the dt between 2 measures (time between 2 measures)

3b : measurement step :
- Radar measurement : Call the Jacobian Matrix (partial derivative calculation), the R_ radar instantiation, and update()
- Lidar measurement : H_laser and R_laser instantioation and update()

4. main.cpp :
No implementattion here useful, the code is already delivered
The most inmportant functionnality is the communication with the simulator.


You can see a screenshot of the results :
- in green : the estimation state
- in red : the lidar cells
- in blue : the radar cells
- the car is the real object, which we want track	
- in the right of the window : the RMSE data with the difference for the position  px,py and the velocity vx,vy between the ground truth and the sensor data.

<<<<<<< HEAD
![Sensor Fusion Picture](https://github.com/Jeanyvesbourdoncle/Noisy-Lidar-Radar-Sensor-Fusion-with-an-EKF-/blob/master/Bicycle_Tracking_Sensor_Fusion.png)
=======
![Sensor Fusion Overview](https://github.com/Jeanyvesbourdoncle/Noisy-Lidar-Radar-Sensor-Fusion-with-an-EKF-/blob/master/Bicycle_Tracking_Sensor_Fusion.png)
>>>>>>> Update README.md

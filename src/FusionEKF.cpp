//=================================================================================================================================
// Name        : FusionEKF.cpp
// Author      : Jean-Yves Bourdoncle
// Version     : v1.0
// Date		   : 26/01/2019
// Description : the implemented functionnalities in this module are :
//				- the initialization of all the matrix (for the prediction step ans the measurement step)
//				- the initialization of the KF with the first radar measures (polar coordinates) or the first lidar mesaures (carthesian coordinates)
//				- predict step :
//					- F_ (state transition matrix) modification with the dt (time between 2 measures)
//					- Q_ (process covariance matrix,acceleration/velocity noise )modification with the dt between 2 measures (time between 2 measures)
//				- measurement step :
//					- Radar measurement : Call the Jacobian Matrix (partial derivative calculation), the R_ radar instantiation, and update()
//					- Lidar measurement : H_laser and R_laser instantioation and update()
//====================================================================================================================================


#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::cout;
using std::endl;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //initialize measurement covariance matrix (Noise) - laser
  // (R= E[wwT] : diag matrix : Tpx^2,Tpy^2)
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // initialize measurement covariance matrix (Noise) -
  // already calculated, same calculation as R_laser with 3*3 matrix
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Initialize measurement matrix - laser (only the position : px=py=1, vx=vy=0)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
/*****************************************************************************
*  Prediction
****************************************************************************/
  if (!is_initialized_) {

    // first measurement
	//cout << "Sensor Fusion Initialization" << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1; // initial state (px=py=vx=vy=1)


    /*Initialize transition matrix F with 1
    px' = px
    py' = py
    vx' = vx
    vy' = vy
    */
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 0, 0,
    		0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;


     /*initial state Covariance Matrix
     the initial velocities vx et vy are initialized with a large covariance : 1000, because vx and vy are unknown
     the initial poitions px and y are intialized witht the standard value 1
     */
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0, 0, 0,
    		0, 1, 0, 0,
			0, 0, 1000, 0,
			0, 0, 0, 1000;

    // RADAR measurement process
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      cout << "First measurement RADAR" << endl;
      // Convert radar from polar to cartesian coordinates and initialize state.
      float rho = measurement_pack.raw_measurements_[0]; // range
      float phi = measurement_pack.raw_measurements_[1]; // bearing
      float rhodot = measurement_pack.raw_measurements_[2]; // range rate

      // polar to cartesian - r * cos(angle) for x and r * sin(angle) for y
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rhodot * cos(phi);
      float vy = rhodot * sin(phi);
      ekf_.x_ << px,
	  	  	  	 py,
				 vx,
				 vy;
      cout << "Radar initialization done" << endl;
      }

      // LIDAR measurement process
      else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //lidar runs with standard cartesian coordinates
      //we use only px and py, that why vx=vy=0
      cout << "EKF : First measurement LIDAR" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0],
    		  measurement_pack.raw_measurements_[1],
			  0,
			  0;
      cout << "Lidar initialization done" << endl;
      }

      // Saving first timestamp in seconds for calculating interval
      previous_timestamp_ = measurement_pack.timestamp_;
      // done initializing, no need to predict or update
      is_initialized_ = true;
      return;
  }

/*****************************************************************************
*  Prediction
****************************************************************************/
  // compute the time elapsed between the current and previous measurements
  //dt definition expressed in ms, useful for the F matrix and the Q matrix update
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transition matrix update with the integration of dt in the 2 dedicated place
  // row 1, column 3 ==> (1-1 =0, 3-1=2)
  ekf_.F_(0, 2) = dt;
  // row 2, column 4 ==> (2-1=1, 4-1=3)
  ekf_.F_(1, 3) = dt;

  // preliminary for the Q matrix calculation
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  // set the acceleration noise : acceleration vector for the Q calculation
  // this value was recommanended and not calculated
  float noise_ax=9.0;
  float noise_ay=9.0;


  // Process covariance matrix Q calculation update with dt (acceleration/velocity as random noise)
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (dt4 * noise_ax) / 4, 0, (dt3 * noise_ax) / 2, 0,
      		0, (dt4 * noise_ay) / 4, 0, (dt3 * noise_ay) / 2,
      		(dt3 * noise_ax) / 2, 0, dt2 * noise_ax, 0,
      		0, (dt3 * noise_ay) / 2, 0, dt2 * noise_ay;

  // Call the function predict
  //cout << "start predict step " << endl;
  ekf_.Predict();



/*****************************************************************************
*  Update
****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  //Radar updates
	  cout << " RADAR measurement update step - EKF " << endl;
	  // H_Radar from the jacobian matrix for the partial derivative for rho,phi et roho_dot in consideration with px,py,vx,vy
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  //R_radar given by the radar supplier
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  	  }
  else {
	  //Laser updates
	  cout << "LIDAR measurement update step - KF " << endl;
	  //standard update for H and R
	  // H_laser : only for the positions px and py
	  ekf_.H_ = H_laser_;
	  //R_laser given by the lidar supplier
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  	  }

}



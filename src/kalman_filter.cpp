//============================================================================
// Name        : kalman_filter.cpp
// Author      : Jean-Yves Bourdoncle
// Version     : v1.0
// Date		   : 26/01/2019
// Description : the implemented functionnalities in this module are :
//				- predict() for the radar and the lidar
//				- update() for the lidar
//				- updateEFK() for the radar
//============================================================================


#include "kalman_filter.h"
#define PI 3.141592

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}



void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {

	// x is the state vector
	x_ = x_in; // State vector : px,py,vx,vy

	// P,F, Q are matrix for the prediction step
	P_ = P_in; // Covariance uncertainty
	F_ = F_in; // State function matrix
	Q_ = Q_in; // Noise matrix (from the velocity uncertainty)

	// H and R are matrix for the measurement update step
	H_ = H_in; // Measure function matrix
	R_ = R_in; // Noise matrix (Measure error from the sensor)

}


// The functionnality prediction is the same for the lidar and the radar
void KalmanFilter::Predict() {
	// new localisation = state matrix*old localisation
	// Bu the control matrix is ignored, not internal information about the tracking object
	x_ = F_ * x_;

	// Tranpose the matrix F
	MatrixXd Ft = F_.transpose();
	// Calculation of the new covariance uncertainty , Q is the noise matrix (velocity/acceleration uncertainly)
	P_ = F_ * P_ * Ft + Q_;
}


// The functionnality "measurement update" is for the LIDAR : linear function
// Standard Kalman Filter equation

void KalmanFilter::Update(const VectorXd &z) {
	// y = z - Hx' : difference between the mesure update (z) and and the prediction (H_ * x_)
	VectorXd y = z - H_ * x_;
	// Transpose the matrix H
	MatrixXd Ht = H_.transpose();
	// Error Matrix calculation
	// with the covariance matrix uncertainly P and the noise matrix R (sensor error measurement)
	MatrixXd S = H_ * P_ * Ht + R_;
	// Inverse the matrix S
	MatrixXd Si = S.inverse();
	// Gain of the Kalman filter K
	// P' is the incertitude where we think to be (comes from Q : error velocity/acceleration )
	// R is the incertitude of the measure of uncertainly (comes from R : error sensor measurement)
	MatrixXd K = P_ *Ht * Si;

	//new state estimation for x and P
	// for the new localisation : we take in consideration the kalman filter gain K * Comporaison error y
	x_ = x_ + (K * y);
	float x_size = x_.size();
	// I Identity matrix
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	// for the new incertainty : we take in consideration the measure fonction H * the kalman filter gain K
	P_ = (I - K * H_) * P_;
}



// The functionnality "measurement update EKF" is for the RADAR : no-linear function
// We use here the jacobienne matrix implemented in tools.cpp to be linear
void KalmanFilter::UpdateEKF(const VectorXd &z) {

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	// The 3 parameters for the radar are rho, phi, rhot_dot
	// they are defined :
	float rho = sqrt(px*px + py * py);
	float phi = atan2(py, px);
	float rho_dot = (px*vx + py * vy) / rho;

	// Check division by 0
	if (fabs(rho)<0.0001){
		//cout << "parameters rho error - division by zero" <<endl;
		return;
		}

	// the non linear function h(x') is built with the 3 parameters named here h
	VectorXd h = VectorXd(3);
	h << rho, phi, rho_dot;
	// the radar measure is : y= z- h(x')
	VectorXd y = z - h;

	// angle normalization for the phi ajustement between [-pi, pi]
	// y(1) = z(1) - h(1) where h(1) is the phi pre-calculated
	while (y(1) > PI || y(1) < -PI) {
		if (y(1) > PI) {
			y(1) -= 2 * PI;
		}
		else {
			y(1) += 2* PI;
		}
	}

	//Transpose the matrix H
	MatrixXd Ht = H_.transpose();
	// Error Matrix calculation
	// with the covariance matrix uncertainly P and the noise matrix R (sensor error measurement)
	MatrixXd S = H_ * P_ * Ht + R_;
	// Inverse the matrix S
	MatrixXd Si = S.inverse();
	// Gain of the Kalman filter K
	// P' is the incertitude where we think to be (comes from Q : error velocity/acceleration )
	// R is the incertitude of the measure of uncertainly (comes from R : error sensor measurement)
	MatrixXd K = P_ * Ht * Si;


	//new state estimation for x and P
	// for the new localisation : we take in consideration the kalman filter gain K * Comporaison error y
	x_ = x_ + (K * y);
	float x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	//new state estimation for x and P
	// for the new localisation : we take in consideration the kalman filter gain K * Comporaison error y
	P_ = (I - K * H_) * P_;
}

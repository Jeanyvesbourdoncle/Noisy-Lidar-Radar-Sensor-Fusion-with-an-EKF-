//============================================================================
// Name        : tools.cpp
// Author      : Jean-Yves Bourdoncle
// Version     : v1.0
// Date		   : 26/01/2019
// Description : the implemented functionnalities in this module are :
//				- the RMSE calculation to measure the accuracy between the predict state and the real state for px,py, vx,vy
//				- the Jocabian matrix for the EKF (radar) : it's a matrix of partial derivative with the parameters rho,phi,rho_dot and px,py, vx,vy
//				- The Hessienne matrix is not implemented (second partial derivative)
//============================================================================


#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	/*
	RMSE : Root Mean Squared Error
	difference between the estimation and the ground truth for the 4 states :
	px,py for the position
	vx,vy for the velocity
	*/
	VectorXd rmse (4);
	rmse << 0,0,0,0;

	/* preliminary
	estimation vector size must be the same as the ground truth vector size
	estimation vector size must be different as 0
	*/
	if (estimations.size() != ground_truth.size() || estimations.size()==0) {
    return rmse;
	}

	// accumulate squared residuals
	for (unsigned int i=0; i< estimations.size();i++){
		VectorXd residual = estimations[i] - ground_truth[i];

		// coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse +=residual;
	}
  
	// calculate the mean and the square
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}
  
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/* 3 rows for the 3 polar coordinate (meadurement function):
	partial derivate from :
	ro : range,
	phi : bearing,
	ro dot : range rate.

	4 columns for the 4 x-state :
	partial derivative from :
	px, py, vx, vy
  	*/

	// only the jacobian matrix is mplemented, the hessienne (second partial derivative are not implemented)

	MatrixXd  Hj(3,4);

	//state parameters
	float px = x_state(0); // column 0
	float py = x_state(1); // column 1
	float vx = x_state(2); // column 2
	float vy = x_state(3); // column 3

	// Jocobienne Matrix Hj  preliminary step
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = c1*c2;

	// Check division by 0
	if (fabs(c1)<0.0001){
		//cout << "Jacobian Matrix Error - division by zero" <<endl;
		return Hj;
	}

	// Compute the jacobian matrix with c1,c2, c3, px,py,vx,vy
	Hj << (px/c2),(py/c2),0,0,
		  -(py/c1),(px/c1),0,0,
		  py*(vx*py-vx*px)/c3, px*(px*vy-py*vx)/c3,px/c2,py/c2;

	return Hj;

}

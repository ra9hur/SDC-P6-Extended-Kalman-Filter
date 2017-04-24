#include <iostream>
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here

  if (estimations.size() == 0) {
	  cout << "Size of estimation vector is zero" << endl;
  } 
  else if (estimations.size() != ground_truth.size()) {
	  cout << "Estimation vector size is not equal to ground truth vector size" << endl;
  }	
  else {

	  VectorXd error(4), error_sq(4), mean(4);

	  size_t n = estimations.size();

	  for(int i=0; i < estimations.size(); ++i){
      // residual error
		  error = estimations[i]-ground_truth[i];
		  error = error.array().abs();
      error_sq = error.array()*error.array();

		  //accumulate squared residuals
		  mean += error_sq;
	  }
	  //calculate the mean
	  mean = (1./n)*mean;

	  //calculate the squared root
    mean = abs(mean.array());
	  rmse = mean.array().sqrt();
  }

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj = MatrixXd::Zero(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float sq = (px*px + py*py);

  //check division by zero
  if (fabs(sq) < 0.0001) {
	  cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	  return Hj;
  }
  
  //compute the Jacobian matrix
  float sq_1_2 = pow(sq,0.5);
  float sq_3_2 = pow(sq,1.5);
   
  float a = px / sq_1_2;
  float b = py / sq_1_2;
  float c = -py / sq;
  float d = px / sq;
  float e = py*(vx*py - vy*px)/sq_3_2;
  float f = px*(vy*px - vx*py)/sq_3_2;

  Hj << a,b,0,0,
	    c,d,0,0,
	    e,f,a,b;

  return Hj;
}

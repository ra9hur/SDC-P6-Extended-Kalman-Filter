#include "kalman_filter.h"
#include <iostream>
//#include <math.h>       /* atan2 */

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
	float sq = (px*px + py*py);

  if (fabs(sq) < 0.0001) {
		cout << "Update() - Error - Division by Zero" << endl;
  } else {
  	VectorXd z_pred = H_ * x_;
		VectorXd y = z - z_pred;
		MatrixXd Ht = H_.transpose();
		MatrixXd S = H_ * P_ * Ht + R_;
		MatrixXd Si = S.inverse();
		MatrixXd PHt = P_ * Ht;
		MatrixXd K = PHt * Si;

		//new estimate
		x_ = x_ + (K * y);
		long x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size, x_size);
		P_ = (I - K * H_) * P_;
	}

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

	float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
	float sq = (px*px + py*py);

  if (fabs(sq) < 0.0001) {
		cout << "UpdateEKF() - Error - Division by Zero" << endl;
  } else {
  	float sq_1_2 = pow(sq,0.5);
		float atan_val = atan2(py,px);

		VectorXd hx(3);
		hx << sq_1_2, atan_val, (px*vx + py*vy)/sq_1_2;
	
		VectorXd z_pred = hx;
		VectorXd y = z - z_pred;

		if (y[1] > M_PI)
			y[1] = fmod(y[1] - M_PI, 2*M_PI) - M_PI;
		if (y[1] < -M_PI)
			y[1] = fmod(y[1] + M_PI, 2*M_PI) + M_PI;

		MatrixXd Ht = H_.transpose();
		MatrixXd S = H_ * P_ * Ht + R_;
		MatrixXd Si = S.inverse();
		MatrixXd PHt = P_ * Ht;
		MatrixXd K = PHt * Si;

		//new estimate
		x_ = x_ + (K * y);
		long x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size, x_size);
		P_ = (I - K * H_) * P_;
	}
}

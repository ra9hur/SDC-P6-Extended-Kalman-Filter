#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			       0, 1, 0, 1,
			       0, 0, 1, 0,
			       0, 0, 0, 1;

	//measurement matrix
	H_laser_ << 1, 0, 0, 0,
			        0, 1, 0, 0;

	//measurement matrix
	Hj_ << 1, 0, 0, 0,
			   0, 1, 0, 0,
         0, 0, 1, 0;

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1000, 0, 0, 0,
			       0, 1000, 0, 0,
			       0, 0, 1000, 0,
			       0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  float rho, phi, rho_dot;
  float px, py, vx, vy;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      rho = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];
      rho_dot = measurement_pack.raw_measurements_[2];

      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);
      ekf_.x_ << px,py,vx,vy;

      // previous_timestamp_ already initialized to zero, not initialized here
      previous_timestamp_ = measurement_pack.timestamp_;
      
      // Check if initial measurements are zeros
      if ((rho==0) && (phi==0) && (rho_dot==0)) {
        is_initialized_ = false;
      } 
      // done initializing, no need to predict or update
      else {
        is_initialized_ = true;
      }

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

      ekf_.x_ << px, py, 0, 0;
 
      // previous_timestamp_ already initialized to zero, not initialized here
      previous_timestamp_ = measurement_pack.timestamp_;
      // Check if initial measurements are zeros
      if ((px==0) && (py==0)) {
        is_initialized_ = false;
      } 
      // done initializing, no need to predict or update
      else {
        is_initialized_ = true;
      }
    }

    // done initializing, no need to predict or update
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	
	// Modify the F matrix so that the time is integrated
	ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;

	// Set the process covariance matrix Q
  float dt2 = pow(dt, 2);
  float dt3 = pow(dt, 3);
  float dt4 = pow(dt, 4);

  //acceleration noise components
  float noise_ax = 9.;
  float noise_ay = 9.;

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << (dt4/4)*noise_ax, 0, (dt3/2)*noise_ax, 0,
			      0, (dt4/4)*noise_ay, 0, (dt3/2)*noise_ay,
			      (dt3/2)*noise_ax, 0, (dt2)*noise_ax, 0,
			      0, (dt3/2)*noise_ay, 0, (dt2)*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  
  else {
    // Laser updates

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

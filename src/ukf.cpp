#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#define PI 3.1415926

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7;
 
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer. 
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;
  time_us_ = 0;

  // initial covariance matrix
  P_ <<  1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_ , 0,
              0, std_laspy_ * std_laspy_;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_ , 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  //predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); 

  //weights for update
  weights_ = VectorXd(2 * n_aug_ + 1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  //if(use_radar != true){

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_ != true){

    std::cout << "RADAR disabled!" << std::endl;
    return;

  }


  //if(use_ladar != true){
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_ != true){

    std::cout << "LASER disabled!" << std::endl;
    return;

  }


  /*****************************************************************************
   *  Initialization x_ and weights_
   ****************************************************************************/
  if (!is_initialized_) {


    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */


      float ro = meas_package.raw_measurements_(0);
      float theta = PI/ 2 - meas_package.raw_measurements_(1);

      float px = ro * cos(theta);
      float py = ro * sin(theta);

      x_(0) = ro * cos(theta);
      x_(1) = ro * sin(theta);
      x_(2) = 0;
      x_(3) = atan2(py, px);
      x_(4) = 0;

      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      float px = meas_package.raw_measurements_(0);
      float py = meas_package.raw_measurements_(1);

      x_(0) = px;
      x_(1) = py;
      x_(2) = 0;
      x_(3) = atan2(py, px);
      x_(4) = 0;

      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;

    }

  }


  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0 ;

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (x_(2) == 0) ) {

    float ro = meas_package.raw_measurements_(0);
    float theta = PI/2 - meas_package.raw_measurements_(1);

    float px = ro * cos(theta);
    float py = ro * sin(theta);
    float vx = (px - x_(0));
    float vy = (py - x_(1));

    //x_(0) = px;
    //x_(1) = py;
    x_(2) = sqrt(vx * vx + vy * vy);
    x_(4) = (theta - x_(3)) / delta_t;
    x_(3) = theta;

  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (x_(2) == 0)) {

    float px = meas_package.raw_measurements_(0);
    float py = meas_package.raw_measurements_(1);
    float vx = (px - x_(0)) / delta_t;
    float vy = (py - x_(1)) / delta_t;

    //x_(0) = px;
    //x_(1) = py;
    x_(2) = sqrt(vx * vx + vy * vy);
    x_(4) = (atan2(vy, vx) - x_(3)) / delta_t;
    x_(3) = atan2(vy, vx);

  }

  //weights for update
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      if (i == 0){
          weights_(i) = lambda_ / (lambda_ + n_aug_);
      } else{
          weights_(i) = 1 / (2.0 * (lambda_ + n_aug_));
      }

  }


  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

     UpdateRadar(meas_package);

  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

     UpdateLidar(meas_package);

  }

  time_us_ = meas_package.timestamp_;  

}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  /*****************************************************************************
   *  Generate Xsig_aug
   ****************************************************************************/
   
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  x_aug.setZero();

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.setZero();

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.setZero();

  //create augmented mean state
  x_aug<<x_, 0, 0;

  //create augmented covariance matrix
  MatrixXd Q = MatrixXd(2, 2);

  float lambda = 3 - n_x_;
  VectorXd weights = VectorXd(2*n_aug_+1);

  Q <<  std_a_ * std_a_ , 0.,
        0., std_yawdd_ * std_yawdd_;

  P_aug.topLeftCorner(P_.rows(),P_.cols()) = P_;
  P_aug.bottomRightCorner(Q.rows(), Q.cols()) = Q;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd P_aug_sqrt = MatrixXd(n_aug_, n_aug_);
  P_aug_sqrt = sqrt(lambda + n_aug_) * A;

  MatrixXd x_n_aug = MatrixXd(n_aug_, n_aug_);
  x_n_aug = x_aug.replicate(1, n_aug_);

  Xsig_aug << x_aug, x_n_aug + P_aug_sqrt, x_n_aug - P_aug_sqrt;



  /*****************************************************************************
   *  Generate Xsig_pred_
   ****************************************************************************/

  MatrixXd X_dot = MatrixXd(n_x_, 1);
  X_dot.setZero();

  MatrixXd nu = MatrixXd(n_x_, 1);
  nu.setZero();

  //create vector for predicted state
  VectorXd x_pred = VectorXd(n_x_);
  x_pred.setZero();

  //create covariance matrix for prediction
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);
  P_pred.setZero();

  float v_k = 0;
  float psi = 0;
  float psi_dot = 0;
  float nu_a_k = 0;
  float nu_psi_dd =0;

  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  for (int i = 0; i< 2 * n_aug_ + 1; i++)
  {
    v_k = Xsig_aug.col(i)(2);
    psi = Xsig_aug.col(i)(3);
    psi_dot = Xsig_aug.col(i)(4);
    nu_a_k = Xsig_aug.col(i)(5);
    nu_psi_dd= Xsig_aug.col(i)(6);


    if (psi_dot == 0){

        X_dot <<    v_k * cos(psi) * delta_t,
                    v_k * sin(psi) * delta_t,
                    0,
                    0,
                    0;
    } else{
        X_dot <<    v_k / psi_dot * (sin(psi + psi_dot * delta_t) - sin(psi)),
                    v_k / psi_dot * (-cos(psi + psi_dot * delta_t) + cos(psi)),
                    0,
                    psi_dot * delta_t,
                    0;
    }

    nu <<   1.0/2.0 * delta_t * delta_t * cos(psi) * nu_a_k,
            1.0/2.0 * delta_t * delta_t * sin(psi) * nu_a_k,
            delta_t * nu_a_k,
            1.0/2.0 * delta_t * delta_t * nu_psi_dd,
            delta_t * nu_psi_dd;


    Xsig_pred_.col(i) = Xsig_aug.col(i).segment(0, 5) + X_dot + nu;

  }



  /*****************************************************************************
   *  Generate x_pred and P_pred
   ****************************************************************************/
   
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      if (i == 0){
          weights(i) = lambda / (lambda + n_aug_);
      } else{
          weights(i) = 1 / (2.0 * (lambda + n_aug_));
      }

  }

  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      x_pred = x_pred + (weights(i) * Xsig_pred_.col(i));

  }


  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
    
    //angle normalization
    while (x_diff(3)> PI) x_diff(3)-=2.*PI;
    while (x_diff(3)<-PI) x_diff(3)+=2.*PI;

    P_pred = P_pred + weights(i) * x_diff * x_diff.transpose() ;

  }

  x_ = x_pred;
  P_ = P_pred;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */



  /*****************************************************************************
   *  Generate Zsig and z_pred 
   ****************************************************************************/

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.setZero();

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.setZero();

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.setZero();

  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

    Zsig.col(i) << 
               Xsig_pred_.col(i)(0), 
               Xsig_pred_.col(i)(1);

  }

  //calculate mean predicted measurement
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      z_pred += weights_(i) * Zsig.col(i);
      
  }


  /*****************************************************************************
   *  Generate z , K 
   ****************************************************************************/
      
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      S += weights_(i) * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose() ;
      
  }

  S += R_laser_;

  VectorXd z = VectorXd(n_z);
  z <<
      meas_package.raw_measurements_(0),   //px 
      meas_package.raw_measurements_(1);   //py 

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero(); 

  //NIS
  MatrixXd NIS = MatrixXd(n_x_, n_z);
  NIS.setZero(); 

  //diff for z
  VectorXd z_diff =  VectorXd(n_z);

  //calculate cross correlation matrix
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

    z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff(3)> PI) x_diff(3)-=2.*PI;
    while (x_diff(3)<-PI) x_diff(3)+=2.*PI;
    while (x_diff(4)> PI) x_diff(4)-=2.*PI;
    while (x_diff(4)<-PI) x_diff(4)+=2.*PI;
  
    Tc += weights_(i) * x_diff * z_diff.transpose();
   
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_z,n_z);
  K = Tc * S.inverse();

  //update state mean and covariance matrix
  z_diff = z - z_pred;
  x_ = x_ + K * z_diff;
  P_ = P_ -  K * S * K.transpose();

  NIS = z_diff.transpose() * S.inverse() * z_diff;
  std::cout << "Lidar NIS = "  << NIS << std::endl;
 
}





/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */


  /*****************************************************************************
   *  Generate Zsig and z_pred 
   ****************************************************************************/

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.setZero();

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.setZero();

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.setZero();

  //transform sigma points into measurement space
  float px = 0;
  float py = 0;
  float v = 0;
  float psi = 0;

  float ro = 0;
  float theta = 0;
  float ro_dot = 0;

  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

    px = Xsig_pred_.col(i)(0);
    py = Xsig_pred_.col(i)(1);
    v = Xsig_pred_.col(i)(2);
    psi = Xsig_pred_.col(i)(3);

    ro = 0;
    theta = 0;
    ro_dot = 0;

    if (px != 0){

        ro = sqrt(px * px + py * py);
        theta = atan2(py , px);
        ro_dot = (px * cos(psi) * v + py * sin(psi) * v) / ro;
    }

    Zsig.col(i) << ro, theta, ro_dot;

  }

  //calculate mean predicted measurement
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      z_pred += weights_(i) * Zsig.col(i);
      
  }




  /*****************************************************************************
   *  Generate z , K 
   ****************************************************************************/
   
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

      S += weights_(i) * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose() ;
  }

  S += R_radar_;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
      meas_package.raw_measurements_(0),   //rho in m
      meas_package.raw_measurements_(1),   //phi in rad
      meas_package.raw_measurements_(2);   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();

  //NIS
  MatrixXd NIS = MatrixXd(n_x_, n_z);
  NIS.setZero();

  VectorXd z_diff =  VectorXd(n_z); 

  //calculate cross correlation matrix
  for(int i = 0; i < 2 * n_aug_ + 1; i++ ){

    z_diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    while (z_diff(1)> PI) z_diff(1)-=2.* PI;
    while (z_diff(1)<-PI) z_diff(1)+=2.* PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //angle normalization
    while (x_diff(3)> PI) x_diff(3)-=2.* PI;
    while (x_diff(3)<-PI) x_diff(3)+=2.* PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();

  }


  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_z,n_z);
  K = Tc * S.inverse();

  //update state mean and covariance matrix
  z_diff = z - z_pred;
  x_ = x_ + K * z_diff;
  P_ = P_ -  K * S * K.transpose();

  NIS = z_diff.transpose() * S.inverse() * z_diff;
  std::cout << "Radar NIS = "  << NIS << std::endl;

}

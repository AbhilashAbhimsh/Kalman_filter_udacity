#include "kalman_filter.h"
#include<iostream>
#include"tools.h"
#include<cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
//   std::cout << "KalmanFilter::Predict() Entered" << std::endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = (F_ * P_ * Ft) + Q_;
//   std::cout << "P_\n" << P_ << std::endl;
//   std::cout << "Q_\n" << Q_ << std::endl;
//   std::cout << "x_\n" << x_ << std::endl;
//   std::cout << "F_\n" << F_ <<std::endl;
//   std::cout << "KalmanFilter::Predict() Exited" << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
//   std::cout << "KalmanFilter::update() Entered" << std::endl;
//   std::cout << z << H_ << std::endl << x_ <<std::endl;
//   std::cout << H_ << std::endl << std::endl << x_ <<std::endl;
  
  VectorXd y = z - (H_ * x_);
  MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
//   std::cout << "KalmanFilter::update() S: \n" << S << std::endl;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_)*P_;
//   std::cout << "KalmanFilter::update() Exited\n";
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
//   std::cout << "\n\nKalmanFilter::updateEKF() Entered" << std::endl;
  
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float add_px_py = pow(px, 2) + pow(py, 2);
  
  float rho = pow(add_px_py , 0.5);
  float theta = atan2(py, px);
  float rho_prime = (vx*px + vy*py)/rho;
  
  VectorXd H_x(3), y(3);
  H_x << rho, theta, rho_prime;
//   std::cout << "H_x\n" << H_x << std::endl;
//   std::cout << "z\n" << z << std::endl;
  y = z - H_x;
//   std::cout << "y\n" << y << std::endl;
  MatrixXd S = (H_*P_*H_.transpose()) + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
//   std::cout << "H_\n" << H_ << std::endl;
//   std::cout << "S\n" << S << std::endl;
//   std::cout << "K\n" << K << std::endl;
  
  x_ = x_ + (K * y);
//   std::cout << "After Update\n" << x_ << std::endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
//    std::cout << "KalmanFilter::updateEKF() Exited\n\n" << std::endl;
}

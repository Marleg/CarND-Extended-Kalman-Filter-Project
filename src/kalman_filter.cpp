#include "kalman_filter.h"
#include <iostream>
#include <math.h>

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
  P_ = F_ * P_ * F_.transpose() + Q_;
  // std::cout << "KalmanFilter::Predict Done" << std::endl;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
  * update the state by using Kalman Filter equations
  */

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  // std::cout << "KalmanFilter::Update Done" << std::endl;
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

  float ro = 0.0;
  float phi = 0.0;
  float ro_dot = 0.0;

  VectorXd z_pred(3);

  if (x_(0) > 0.0001 && x_(1) > 0.0001) {
  //recover state parameters

    //TODO: YOUR CODE HERE
    ro = sqrt(px * px + py * py);
    phi = atan2(py, px);
    ro_dot = (px * vx + py * vy) / ro;

  }

  z_pred << ro, phi, ro_dot;
  VectorXd y = z - z_pred;

  // while (y[1] > M_PI || y[1] < - M_PI) {
  //   if (y[1] > M_PI)
  //     y[1] = y[1] - M_PI;
  //   if (y[1] < - M_PI)
  //     y[1] = y[1] + M_PI;
  // }

  while (y[1] > M_PI || y[1] < - M_PI) {
    if (y[1] > M_PI)
      y[1] = y[1] - M_PI;
    else y[1] = y[1] + M_PI;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  // std::cout << "KalmanFilter::UpdateEKF Done" << std::endl;
}

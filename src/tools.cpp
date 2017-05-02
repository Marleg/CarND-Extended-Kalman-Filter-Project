#include <iostream>
#include "tools.h"

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
  rmse << 0, 0, 0, 0;

  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
    // std::cout << "Invalid estimation or ground truth data" << std::endl;
    return rmse;
  }

  for(int unsigned i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculating the mean
  rmse = rmse/estimations.size();

  //calculating the squared root
  rmse = rmse.array().sqrt();

  // std::cout << "tools:CalculateRMSE executed" << std::endl;
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
  * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
  //recover state parameters
  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);

  //TODO: YOUR CODE HERE
  // float c1 = px * px + py * py;
  const double c1 = std::max(0.0001, px * px + py * py);
  const double c2 = sqrt(c1);
  const double c3 = (c1 * c2);

  // //check division by zero
  // if (fabs(c1) < 0.0001) {
  //   // std::cout << "Division by Zero" << std::endl;
  //   return Hj;
  //   // c1 = 0.0001;
  // }

  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  // std::cout << "tools:CalculateJacobian executed" << std::endl;
  return Hj;
}

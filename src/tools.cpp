#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0,0,0,0;

  if( estimations.size() != ground_truth.size() && estimations.size() != 0 ) {
    std::cout << "Error in calculating RMSE" << std::endl;
    return rmse;
  }

  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  * TODO:(Completed)
  * Calculate a Jacobian here.
  */
  MatrixXd Hj = MatrixXd(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float p_2 = px*px + py*py;
  float p_2_sqrt = sqrt(p_2);
  float p_3by2 = p_2*p_2_sqrt;

  if(fabs(p_2) < 0.0001) {
    std::cout << "Divide by zero error" << std::endl;
    return Hj;
  }

  Hj << px / p_2_sqrt, py / p_2_sqrt, 0, 0,
                -py / p_2, px / p_2, 0, 0,
                py * (vx*py - vy*px) / p_3by2, px * (vy*px - vx*py) / p_3by2, px / p_2_sqrt, py / p_2_sqrt;

  return Hj;
}

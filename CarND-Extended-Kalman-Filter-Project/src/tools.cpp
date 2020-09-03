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
  
  if ((estimations.size() != ground_truth.size()) || (estimations.size() == 0))
  {
    std::cout << "Estimation and ground truth size is not matching" << std::endl;
    return rmse;
  }

  for(unsigned int i = 0; i < estimations.size(); i++)
  {
    VectorXd res = estimations[i] - ground_truth[i];
    res = res.array() * res.array();
    rmse += res;
    
  }
  
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  
  float px,py,vx,vy;
  px = x_state(0);
  py = x_state(1);
  vx = x_state(2);
  vy = x_state(3);
  
  if (px==0 && py==0)
  {
    std::cout << "Both Posistions are zero" << std::endl;
    return Hj;
  }
  
  float px_py, root_px_py, cube_root;
  px_py = pow(px, 2) + pow(py,2);
  root_px_py = pow(px_py, 0.5);
  cube_root = pow(px_py, 1.5);
  
  Hj << px/root_px_py, py/root_px_py, 0, 0,
        -py/px_py, px/px_py, 0, 0,
  		py*(vx*py - vy*px)/cube_root, px*(vy*px - vx*py)/cube_root, px/root_px_py, py/root_px_py;
  
  return Hj;
  
}

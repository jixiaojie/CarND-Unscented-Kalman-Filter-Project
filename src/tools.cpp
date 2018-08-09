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


  int estimations_size = estimations.size();
  int ground_truth_size = ground_truth.size();

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if((estimations_size == 0) or (ground_truth_size == 0))
  {
     return rmse;
  }

  if(estimations_size != ground_truth_size)
  {
     return rmse;
  }

  for(int k = 0; k < estimations_size; k++)
  {
    VectorXd residual = estimations[k] - ground_truth[k];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;

  }

  rmse = rmse.array() / estimations_size;
  rmse = rmse.array().sqrt();

  return rmse;


}

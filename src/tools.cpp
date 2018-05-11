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
	rmse << 0,0,0,0;
	int le = estimations.size();
	int lg = ground_truth.size();
	if (!le){
	    cout<<"Error: The size of the estimation vector is zero"<<endl;
	}
	if (le!=lg){
	    cout<<"Error: The size of the estimation vector should be the same as the ground truth"<<endl;
	}
	for(int i=0; i < le; ++i){
        rmse = rmse + (estimations[i]-ground_truth[i]).array().square().matrix();
	}
	rmse/=le;
	rmse = rmse.array().sqrt().matrix();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	if (px==0 && py==0){
	    cout<<"Error: Division by zero"<<endl;
	}
	float norm = pow(px, 2)+pow(py,2);
	Hj << px/sqrt(norm), py/sqrt(norm), 0, 0,
	        -py/norm, px/norm, 0, 0,
	        py*(vx*py-vy*px)/pow(norm,1.5), px*(vy*px-vx*py)/pow(norm,1.5), px/sqrt(norm), py/sqrt(norm);

	return Hj;
}

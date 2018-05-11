#include <math.h>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

static void adjust_angle(double &angle){
  while(angle>M_PI){
    angle-=2*M_PI;
  }
  while(angle<-M_PI){
    angle+=2*M_PI;
  }
}

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
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z-H_*x_;
  MatrixXd S = H_*P_*H_.transpose()+R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  VectorXd hx = VectorXd(3);
  float rho = sqrt(pow(x_[0],2) + pow(x_[1],2));
  hx<<rho, atan2(x_[1], x_[0]), (x_[0]*x_[2]+x_[1]*x_[3])/rho;
  VectorXd y = z-hx;
  adjust_angle(y[1]);
  MatrixXd S = H_*P_*H_.transpose()+R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I-K*H_)*P_;
}

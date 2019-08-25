#include "../include/ExtendedKalmanFilter/ExtendedKalmanFilter.hpp"

#include <cmath>

using namespace std;

ExtendedKalmanFilter::ExtendedKalmanFilter(double init_x, double init_y, double init_theta, double dt){
    dt = dt;

    x << init_x, init_y, init_theta;

    u << dt*cos(init_theta), dt*sin(init_theta), dt;

    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    B << 0.5, 0, 0,
         0, 0.5, 0,
         0, 0, 0.5;

    C << 1, 0, 0,
         0, 1, 0;

    P << 0.5, 0, 0,
         0, 0.5, 0,
         0, 0, 0.5;

    Q << 10, 0, 0,
         0, 10, 0,
         0, 0, 10;

    R << 3, 0,
         0, 3;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

MatrixXd ExtendedKalmanFilter::prior_state_estimate(){
}

MatrixXd ExtendedKalmanFilter::prior_error_convariance_matrix(){
}

MatrixXd ExtendedKalmanFilter::kalman_gain(MatrixXd _P){
}

void ExtendedKalmanFilter::jacobian_matrix(double v, double theta){
}

MatrixXd ExtendedKalmanFilter::state_estimate(MatrixXd y, MatrixXd x, MatrixXd kalman_gain){
}

MatrixXd ExtendedKalmanFilter::posteriori_error_convariance_matrix(MatrixXd kalman_gain, MatrixXd _P){
}

MatrixXd ExtendedKalmanFilter::kalman_filter(double x, double y, double v){
}

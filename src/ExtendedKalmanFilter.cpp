#include "../include/ExtendedKalmanFilter/ExtendedKalmanFilter.hpp"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/LU>

using namespace std;
using namespace Eigen;

ExtendedKalmanFilter::ExtendedKalmanFilter(double init_x, double init_y, double init_theta, double dt){
    dt = dt;

    x << init_x,
         init_y,
         init_theta;

    u << dt*cos(init_theta),
         dt*sin(init_theta),
         dt;

    A << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    B << 0.5, 0.0, 0.0,
         0.0, 0.5, 0.0,
         0.0, 0.0, 0.5;

    C << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;

    P << 0.5, 0.0, 0.0,
         0.0, 0.5, 0.0,
         0.0, 0.0, 0.5;

    Q << 10.0, 0.0, 0.0,
         0.0, 10.0, 0.0,
         0.0, 0.0, 10.0;

    R << 3.0, 0.0,
         0.0, 3.0;
}


ExtendedKalmanFilter::~ExtendedKalmanFilter(){
}


Matrix<double, 3, 1> ExtendedKalmanFilter::prior_state_estimate(){
    return A*x + B*u;
}


Matrix<double, 3, 3> ExtendedKalmanFilter::prior_error_covariance_matrix(){
    return A*P*A.transpose() + Q;
}


Matrix<double, 3, 2> ExtendedKalmanFilter::kalman_gain(Matrix<double, 3, 3> _P){
    return _P*C.transpose()*((C*_P*C.transpose())+R).inverse();
}


void ExtendedKalmanFilter::jacobian_matrix(double v, double theta){
    A << 1.0, 0.0, v*dt*sin(theta),
         0.0, 1.0, -v*dt*cos(theta),
         0.0, 0.0, 1.0;
}


Matrix<double, 3, 1> ExtendedKalmanFilter::state_estimate(Matrix<double, 2, 1> y, Matrix<double, 3, 1> x, Matrix<double, 3, 2> kalman_gain){
    return x + kalman_gain*(y - C*x);
}


Matrix<double, 3, 3> ExtendedKalmanFilter::posteriori_error_covariance_matrix(Matrix<double, 3, 2> kalman_gain, Matrix<double, 3, 3> _P){
    return (MatrixXd::Identity(3, 3) - kalman_gain*C)*P;
}

Matrix<double, 2, 1> ExtendedKalmanFilter::kalman_filter(double _x, double _y, double _v){
    Matrix<double, 3, 1> value_of_prior_state_estimate = prior_state_estimate();
    jacobian_matrix(_v, value_of_prior_state_estimate(2));
    Matrix<double, 3, 3> value_of_prior_error_covariance_matrix = prior_error_covariance_matrix();
    Matrix<double, 3, 2> kalman_gain_matrix = kalman_gain(value_of_prior_error_covariance_matrix);

    Matrix<double, 2, 1> observation_matrix;
    observation_matrix << _x, _y;

    Matrix<double, 3, 1> value_state_estimate = state_estimate(observation_matrix, value_of_prior_state_estimate, kalman_gain_matrix);
    Matrix<double, 3, 3> value_posteriori_error_covariance_matrix = posteriori_error_covariance_matrix(kalman_gain_matrix, value_of_prior_error_covariance_matrix);
    double est_x = value_state_estimate(0, 0);
    double est_y = value_state_estimate(1, 0);

    x = value_state_estimate;
    P = value_posteriori_error_covariance_matrix;

    Matrix<double, 2, 1> est;
    est << est_x, est_y;
    return est;
}

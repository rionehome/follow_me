#include "../include/ExtendedKalmanFilter/ExtendedKalmanFilter.hpp"

using namespace std;
using namespace Eigen;

ExtendedKalmanFilter::ExtendedKalmanFilter(double init_x, double init_y, double dt){
    dt = dt;

    x << init_x,
         init_y;

    u << 0.0,
         0.0;

    A << 1.0, 0.0,
         0.0, 1.0;

    B << 1.0, 0.0,
         0.0, 1.0;

    C << 1.0, 0.0,
         0.0, 1.0;

    P << 0.1, 0.0,
         0.0, 0.1;

    Q << 1.0, 0.0,
         0.0, 1.0;

    R << 1.0, 0.0,
         0.0, 1.0;
}


ExtendedKalmanFilter::~ExtendedKalmanFilter(){
}


Matrix<double, 2, 1> ExtendedKalmanFilter::prior_state_estimate(){
    return A*x + B*u;
}


Matrix<double, 2, 2> ExtendedKalmanFilter::prior_error_covariance_matrix(){
    return A*P*A.transpose() + Q;
}


Matrix<double, 2, 2> ExtendedKalmanFilter::kalman_gain(Matrix<double, 2, 2> _P){
    return _P*C.transpose()*((C*_P*C.transpose())+R).inverse();
}


void ExtendedKalmanFilter::jacobian_matrix(double dx, double dy){
    A << dx, 0.0,
         0.0, dy;
}


Matrix<double, 2, 1> ExtendedKalmanFilter::state_estimate(Matrix<double, 2, 1> y, Matrix<double, 2, 1> x, Matrix<double, 2, 2> kalman_gain){
    return x + kalman_gain*(y - C*x);
}


Matrix<double, 2, 2> ExtendedKalmanFilter::posteriori_error_covariance_matrix(Matrix<double, 2, 2> kalman_gain, Matrix<double, 2, 2> _P){
    return (MatrixXd::Identity(2, 2) - kalman_gain*C)*_P;
}

tuple<double, double> ExtendedKalmanFilter::kalman_filter(double _px, double _py, double _dx, double _dy){
    Matrix<double, 2, 1> value_of_prior_state_estimate = prior_state_estimate();
    jacobian_matrix(_dx, _dy);
    Matrix<double, 2, 2> value_of_prior_error_covariance_matrix = prior_error_covariance_matrix();
    Matrix<double, 2, 2> kalman_gain_matrix = kalman_gain(value_of_prior_error_covariance_matrix);

    Matrix<double, 2, 1> observation_matrix;
    observation_matrix << _px, _py;

    Matrix<double, 2, 1> value_state_estimate = state_estimate(observation_matrix, value_of_prior_state_estimate, kalman_gain_matrix);
    Matrix<double, 2, 2> value_posteriori_error_covariance_matrix = posteriori_error_covariance_matrix(kalman_gain_matrix, value_of_prior_error_covariance_matrix);

    x = value_state_estimate;
    P = value_posteriori_error_covariance_matrix;

    return make_tuple(value_state_estimate(0, 0), value_state_estimate(1, 0));
}

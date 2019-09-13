#include <cmath>
#include <tuple>

#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class ExtendedKalmanFilter {
    public :

        ExtendedKalmanFilter(double init_x, double init_y, double dt);
        ~ExtendedKalmanFilter();

        Matrix<double, 2, 1> prior_state_estimate();
        Matrix<double, 2, 2> prior_error_covariance_matrix();
        Matrix<double, 2, 2> kalman_gain(Matrix<double, 2, 2> _P);
        void jacobian_matrix(double dx, double dy);
        Matrix<double, 2, 1> state_estimate(Matrix<double, 2, 1> y, Matrix<double, 2, 1> x, Matrix<double, 2, 2> kalman_gain);
        Matrix<double, 2, 2> posteriori_error_covariance_matrix(Matrix<double, 2, 2> kalman_gain, Matrix<double, 2, 2> _P);
        tuple<double, double> kalman_filter(double _px, double _py, double _dx, double _dy);

    private :

        double dt;
        Matrix<double, 2, 2> A;
        Matrix<double, 2, 2> B;
        Matrix<double, 2, 2> P;
        Matrix<double, 2, 2> Q;
        Matrix<double, 2, 2> R;

        Matrix<double, 2, 1> x;
        Matrix<double, 2, 1> u;
        Matrix<double, 2, 2> C;

};

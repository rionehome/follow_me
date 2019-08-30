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

        ExtendedKalmanFilter(double init_x, double init_y, double init_theta, double dt);
        ~ExtendedKalmanFilter();

        Matrix<double, 3, 1> prior_state_estimate();
        Matrix<double, 3, 3> prior_error_covariance_matrix();
        Matrix<double, 3, 2> kalman_gain(Matrix<double, 3, 3> _P);
        void jacobian_matrix(double v, double theta);
        Matrix<double, 3, 1> state_estimate(Matrix<double, 2, 1> y, Matrix<double, 3, 1> x, Matrix<double, 3, 2> kalman_gain);
        Matrix<double, 3, 3> posteriori_error_covariance_matrix(Matrix<double, 3, 2> kalman_gain, Matrix<double, 3, 3> _P);
        tuple<double, double> kalman_filter(double _x, double _y, double _v);

    private :

        double dt;
        Matrix<double, 3, 3> A;
        Matrix<double, 3, 3> B;
        Matrix<double, 3, 3> P;
        Matrix<double, 3, 3> Q;

        Matrix<double, 2, 2> R;

        Matrix<double, 3, 1> x;
        Matrix<double, 3, 1> u;
        Matrix<double, 2, 3> C;

};

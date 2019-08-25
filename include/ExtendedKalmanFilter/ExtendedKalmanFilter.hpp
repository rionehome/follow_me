#include <cmath>

#include </usr/include/eigen3/Eigen/Core>
#include </usr/include/eigen3/Eigen/Geometry>
#include </usr/include/eigen3/Eigen/Dense>

using namespace Eigen;

class ExtendedKalmanFilter {
	public :
		ExtendedKalmanFilter(double init_x, double init_y, double init_theta, double dt);
		~ExtendedKalmanFilter();

		MatrixXd prior_state_estimate();
	    MatrixXd prior_error_convariance_matrix();
		MatrixXd kalman_gain(MatrixXd _P);
		void jacobian_matrix(double v, double theta);
		MatrixXd state_estimate(MatrixXd y, MatrixXd x, MatrixXd kalman_gain);
		MatrixXd posteriori_error_convariance_matrix(MatrixXd kalman_gain, MatrixXd _P);
		MatrixXd kalman_filter(double x, double y, double v);
		
	private:
        double dt;
        Matrix3d A, B, P, Q;
        MatrixXd x, u, C, R;
};

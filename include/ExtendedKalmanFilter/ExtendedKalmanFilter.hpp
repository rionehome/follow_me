#include <cmath>
#include <vector>

using namespace std;

class ExtendedKalmanFilter {
	public :
		ExtendedKalmanFilter(double init_x, double init_y, double init_theta, double dt);
		~ExtendedKalmanFilter();

		vector<double> prior_state_estimate();
		vector<double> prior_error_convariance_matrix();
		vector<double> kalman_gain(vector<double> _P);
		void jacobian_matrix(double v, double theta);
		vector<double> state_estimate(vector<double> y, vector<double> x, vector<double> kalman_gain);
		vector<double> posteriori_error_convariance_matrix(vector<double> kalman_gain, vector<double> _P);
		vector<double> kalman_filter(double x, double y, double v);
		
	private:
};

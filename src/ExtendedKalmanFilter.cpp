#include "../include/ExtendedKalmanFilter/ExtendedKalmanFilter.hpp"

#include <cmath>

using namespace std;

ExtendedKalmanFilter::ExtendedKalmanFilter(double init_x, double init_y, double init_theta, double dt){
	dt = dt;

	vector<double> x{init_x, init_y, init_theta};

	vector<double> u{dt*cos(init_theta), dt*sin(init_theta), dt};

	vector<vector<double>> A{{1, 0, 0},
		                     {0, 1, 0},
				             {0, 0, 1}};

	vector<vector<double>> B{{0.5, 0, 0},
		                     {0, 0.5, 0},
					         {0, 0, 0.5}};

	vector<vector<double>> C{{1, 0, 0},
		                     {0, 1, 0}};

	vector<vector<double>> P{{0.5, 0, 0},
		                     {0, 0.5, 0},
					         {0, 0, 0.5}};

	vector<vector<double>> Q{{10, 0, 0},
		                     {0, 10, 0},
					         {0, 0, 10}};

	vector<vector<double>> R{{3, 0},
	    	                 {0, 3}};
}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

vector<double> ExtendedKalmanFilter::prior_state_estimate(){
}

vector<double> ExtendedKalmanFilter::prior_error_convariance_matrix(){
}

vector<double> ExtendedKalmanFilter::kalman_gain(vector<double> _P){
}

void ExtendedKalmanFilter::jacobian_matrix(double v, double theta){
}

vector<double> ExtendedKalmanFilter::state_estimate(vector<double> y, vector<double> x, vector<double> kalman_gain){
}

vector<double> ExtendedKalmanFilter::posteriori_error_convariance_matrix(vector<double> kalman_gain, vector<double> _P){
}

vector<double> ExtendedKalmanFilter::kalman_filter(double x, double y, double v){
}

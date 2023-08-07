#include <bb_tracker/kalmanFilter.h>
#include <Eigen/Cholesky>
#include <iostream>

namespace byte_kalman
{
	const double KalmanFilter::chi2inv95[10] = {
	0,
	3.8415,
	5.9915,
	7.8147,
	9.4877,
	11.070,
	12.592,
	14.067,
	15.507,
	16.919
	};
	
	KalmanFilter::KalmanFilter()
	{
		double dt = 1.;

		// Linear motion with dt = 1 for the first 6 values using the others as velocity
		_motion_mat = Eigen::MatrixXf::Identity(state_dim, state_dim);
		for (int i = 0; i < detection_dim; i++) {
			_motion_mat(i, detection_dim + i) = dt;
		}

		// Update matrix just update with one the first 6 values, doesn't touch the others
		_observation_mat = Eigen::MatrixXf::Identity(detection_dim, state_dim);

		this->_std_weight_position = 1. / 20;
		this->_std_weight_velocity = 1. / 160;
	}

	KAL_DATA KalmanFilter::initiate(const DETECTBOX &measurement)
	{
		DETECTBOX mean_pos = measurement;
		DETECTBOX mean_vel;
		for (int i = 0; i < detection_dim; i++) mean_vel(i) = 0;

		KAL_MEAN mean;
		for (int i = 0; i < state_dim; i++) {
			if (i < detection_dim) mean(i) = mean_pos(i);
			else mean(i) = mean_vel(i - detection_dim);
		}

		KAL_MEAN std;
		float dimensional_factor = measurement[3];
		std(0) = 2 * _std_weight_position * dimensional_factor; 	// x
		std(1) = 2 * _std_weight_position * dimensional_factor;		// y
		std(2) = 1e-2;												// z
		std(3) = 1e-2;												// a_w
		std(4) = 1e-2;												// a_d
		std(5) = 2 * _std_weight_position *  dimensional_factor;	// h
		std(6) = 10 * _std_weight_velocity * dimensional_factor;	// v x	
		std(7) = 10 * _std_weight_velocity * dimensional_factor;	// v y
		std(8) = 1e-5;												// v z
		std(9) = 1e-5;												// v a_w
		std(10) = 1e-5;												// v a_d
		std(11) = 10 * _std_weight_velocity * dimensional_factor;	// v h

		KAL_MEAN tmp = std.array().square();
		KAL_COVA var = tmp.asDiagonal();
		return std::make_pair(mean, var);
	}

	void KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance)
	{
		// mean(3) in xyah representation is the only value representing the real size of the object -> big object = big height
		// With the same meaning mean(5) in xyzwadah is the only real size
		// A cov is given to pos that changes in real time in proportion to the object size
		float size_regularization = mean(5);
		DETECTBOX std_pos;
		std_pos << _std_weight_position * size_regularization,
			_std_weight_position * size_regularization,
			1e-2,
			1e-2,
			1e-2,
			_std_weight_position * size_regularization;
		DETECTBOX std_vel;
		std_vel << _std_weight_velocity * size_regularization,
			_std_weight_velocity * size_regularization,
			1e-5,
			1e-5,
			1e-5,
			_std_weight_velocity * size_regularization;
		KAL_MEAN tmp;
		tmp.block<1, 6>(0, 0) = std_pos;
		tmp.block<1, 6>(0, 6) = std_vel;
		tmp = tmp.array().square();
		// This is V1
		KAL_COVA motion_cov = tmp.asDiagonal();
		// State Prediction
		// x(t+1|t) = F*x(t|t-1)   	[without the update part K(t) * e(t)]
		KAL_MEAN mean1 = this->_motion_mat * mean.transpose();
		// Error Covariance Prediction
		// P_x(t+1) = F*P_x(t)*F^T + V1		[without the update part (F*P(t)*H^T)(H*P(t)*H^T+V2)^-1(F*P(t)*H^T)^T]
		KAL_COVA covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
		covariance1 += motion_cov;

		mean = mean1;
		covariance = covariance1;
	}

	KAL_HDATA KalmanFilter::project(const KAL_MEAN &mean, const KAL_COVA &covariance)
	{
		DETECTBOX std;
		float size_regularization = mean(5);
		std << _std_weight_position * size_regularization,
			_std_weight_position * size_regularization,
			1e-1, 
			1e-1, 
			1e-1, 
			_std_weight_position * size_regularization;
		// Perform state projection in the measurement space
		// y~(t|t-1) = H * x(t|t-1)
		KAL_HMEAN mean1 = _observation_mat * mean.transpose();
		// Perform covariance projection in measurement space
		// P_y(t+1) = H*P_x(t)*H^T + V2
		KAL_HCOVA covariance1 = _observation_mat * covariance * (_observation_mat.transpose());
		Eigen::Matrix<float, 6, 6> diag = std.asDiagonal();
		// square each element to obtain variance from standard deviation
		diag = diag.array().square().matrix();
		covariance1 += diag;

		return std::make_pair(mean1, covariance1);
	}

	KAL_DATA KalmanFilter::update(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const DETECTBOX &measurement)
	{
		// Project to measurement space
		KAL_HDATA pa = project(mean, covariance);
		KAL_HMEAN projected_mean = pa.first;
		KAL_HCOVA projected_cov = pa.second;

		// Compute the KALMAN GAIN solving efficiently the linear system:
		// (H*P(t)*H^T + V2) * K = P(t)*H^T
		// In other words:
		// K = (P(t)*H^T)(H*P(t)*H^T + V2)^-1
		Eigen::Matrix<float, 6, 12> B = (covariance * (_observation_mat.transpose())).transpose();
		// Computes the Cholesky decomposition and performs a triangular solve to find the Kalman gain.
		// The Cholesky decomposition is a method to factorize a symmetric positive-definite matrix into 
		// the product of a lower triangular matrix and its transpose. 
		// The resulting lower triangular matrix is used to solve a linear system efficiently.
		Eigen::Matrix<float, 12, 6> kalman_gain = (projected_cov.llt().solve(B)).transpose();

		// Calculate the Innovation or measurement ERROR
		// e(t) = y(t) - y~(t|t-1)
		Eigen::Matrix<float, 1, 6> innovation = measurement - projected_mean;

		// Compute new STATE estimate
		// x(t|t) = x(t+1|t) + K(t) * e(t)
		auto tmp = innovation * (kalman_gain.transpose());
		KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();

		// Compute new COVARIANCE
		// if we consider the old covariance as state covariance (F*P*F^T + V1) we obtain 
		// P(t+1|t) = (F*P*F^T + V1) - (F*P*H^T)*(H*P*H^T + V2)^-1 *(F*P*H^T)^T
		KAL_COVA new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());

		return std::make_pair(new_mean, new_covariance);
	}

	Eigen::Matrix<float, 1, -1> KalmanFilter::gating_distance(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const std::vector<DETECTBOX> &measurements,
			bool only_position)
	{
		KAL_HDATA pa = this->project(mean, covariance);
		if (only_position) {
			printf("not implement!");
			exit(0);
		}

		KAL_HMEAN mean1 = pa.first;
		KAL_HCOVA covariance1 = pa.second;

		//    Eigen::Matrix<float, -1, 6, Eigen::RowMajor> d(size, 6);
		DETECTBOXSS d(measurements.size(), 6);
		int pos = 0;
		for (DETECTBOX box : measurements) {
			d.row(pos++) = box - mean1;
		}

		// Calculates the Cholesky decomposition of the covariance1 matrix and extracts the lower triangular matrix factor.
		// The Cholesky decomposition is used to decompose a positive-definite matrix into a product of
		// a lower triangular matrix and its transpose.
		Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
		// z is calculated by solving the linear system 'factor * z = d' for z.
		Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
		// Element-wise squaring of the z matrix
		auto zz = ((z.array())*(z.array())).matrix();
		auto square_maha = zz.colwise().sum();
		return square_maha;
	}
}
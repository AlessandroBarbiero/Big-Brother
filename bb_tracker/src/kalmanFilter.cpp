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

		_motion_mat = Eigen::MatrixXf::Identity(state_dim, state_dim);

		_observation_mat3D = Eigen::MatrixXf::Identity(detection3D_dim, state_dim);
		_observation_mat3D(2,2)=0;
		_observation_mat3D(2,3)=1;
		_observation_mat3D(3,3)=0;
		_observation_mat3D(4,4)=0;
		_observation_mat3D(3,4)=1;
		_observation_mat3D(4,5)=1;

		_observation_mat2D = Eigen::MatrixXf::Identity(detection2D_dim, state_dim);
		_observation_mat3D(2,2)=0;
		_observation_mat3D(2,3)=1;
		_observation_mat3D(3,3)=0;
		_observation_mat3D(3,5)=1;

		this->_std_weight_position = 1. / 20;
		this->_std_weight_velocity = 1. / 160;
	}

	// %%%%%%%%%%%%%%%%
	// %%% INITIATE %%%
	// %%%%%%%%%%%%%%%%

	KAL_DATA KalmanFilter::initiate(const DETECTBOX3D &measurement)
	{
		KAL_MEAN mean;
		mean(0) = measurement(0); 	// x
		mean(1) = measurement(1); 	// y
		mean(2) = 0;				// theta
		mean(3) = measurement(2); 	// l_ratio
		mean(4) = measurement(3);	// d_ratio
		mean(5) = measurement(4);	// h
		mean(6) = 0;				// v
		mean(7) = 0;				// w

		// TODO: implement gaussian noise
		KAL_MEAN std;
		std(0) = 2 * _std_weight_position; 	// x
		std(1) = 2 * _std_weight_position;	// y
		std(2) = 10 * _std_weight_position; // theta
		std(3) = _std_weight_position;		// l_ratio
		std(4) = _std_weight_position;		// d_ratio
		std(5) = 2 * _std_weight_position;	// h
		std(6) = 10 * _std_weight_velocity;	// v
		std(7) = 10 * _std_weight_velocity;	// w

		KAL_MEAN tmp = std.array().square();
		KAL_COVA var = tmp.asDiagonal();
		return std::make_pair(mean, var);
	}

	
	KAL_DATA KalmanFilter::initiate(const DETECTBOX2D &measurement)
	{
		KAL_MEAN mean;
		mean(0) = measurement(0); 	// x
		mean(1) = measurement(1); 	// y
		mean(2) = 0;				// theta
		mean(3) = measurement(2); 	// l_ratio
		mean(4) = 0.5;				// d_ratio
		mean(5) = measurement(3);	// h
		mean(6) = 0;				// v
		mean(7) = 0;				// w

		// TODO: implement gaussian noise
		KAL_MEAN std;
		std(0) = 2 * _std_weight_position; 	// x
		std(1) = 2 * _std_weight_position;	// y
		std(2) = 10 * _std_weight_position; // theta
		std(3) = _std_weight_position;		// l_ratio
		std(4) = 10 * _std_weight_position;		// d_ratio
		std(5) = 2 * _std_weight_position;	// h
		std(6) = 10 * _std_weight_velocity;	// v
		std(7) = 10 * _std_weight_velocity;	// w

		KAL_MEAN tmp = std.array().square();
		KAL_COVA var = tmp.asDiagonal();
		return std::make_pair(mean, var);
	}

	void KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance, double dt)
	{
		// TODO: implement gaussian noise
		KAL_MEAN std;
		std(0) = _std_weight_position; 	// x
		std(1) = _std_weight_position;	// y
		std(2) = _std_weight_position; 	// theta
		std(3) = _std_weight_position;	// l_ratio
		std(4) = _std_weight_position;	// d_ratio
		std(5) = _std_weight_position;	// h
		std(6) = _std_weight_velocity;	// v
		std(7) = _std_weight_velocity;	// w

		KAL_MEAN tmp = std.array().square();
		// This is V1
		KAL_COVA motion_cov = tmp.asDiagonal();

		// State Prediction
		// x(t+1|t) = F*x(t|t-1)
		KAL_MEAN mean1 = predictState(mean, dt);
		// Error Covariance Prediction
		// P(t+1) = F*P(t)*F^T + V1
		KAL_COVA covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
		covariance1 += motion_cov;

		mean = mean1;
		covariance = covariance1;
	}

	KAL_MEAN KalmanFilter::predictState(KAL_MEAN &mean, double dt){
		return this->_motion_mat * mean.transpose();
	}

	// %%%%%%%%%%%%%%%
	// %%% PROJECT %%%
	// %%%%%%%%%%%%%%%

	KAL_HDATA3D KalmanFilter::project3D(const KAL_MEAN &mean, const KAL_COVA &covariance)
	{
		// TODO: gaussian noise
		// Compute V2
		DETECTBOX3D std;
		std << _std_weight_position,
			_std_weight_position,
			_std_weight_position, 
			_std_weight_position, 
			_std_weight_position;
		DETECTBOX3D tmp = std.array().square();
		KAL_HCOVA3D measure_var = tmp.asDiagonal();

		// Perform state projection in the measurement space
		// y~(t|t-1) = H * x(t|t-1)
		KAL_HMEAN3D mean1 = projectState3D(mean);
		// Perform covariance projection in measurement space
		// P_y(t+1) = H*P_x(t)*H^T + V2
		KAL_HCOVA3D covariance1 = _observation_mat3D * covariance * (_observation_mat3D.transpose());
		
		covariance1 += measure_var;

		return std::make_pair(mean1, covariance1);
	}

	KAL_HMEAN3D KalmanFilter::projectState3D(const KAL_MEAN &mean){
		return _observation_mat3D * mean.transpose();
	}

	KAL_HDATA2D KalmanFilter::project2D(const KAL_MEAN &mean, const KAL_COVA &covariance)
	{
		// TODO: gaussian noise
		// Compute V2
		DETECTBOX2D std;
		std << _std_weight_position,
			_std_weight_position,
			_std_weight_position, 
			_std_weight_position;
		DETECTBOX2D tmp = std.array().square();
		KAL_HCOVA2D measure_var = tmp.asDiagonal();

		// Perform state projection in the measurement space
		// y~(t|t-1) = H * x(t|t-1)
		KAL_HMEAN2D mean1 = projectState2D(mean);
		// Perform covariance projection in measurement space
		// P_y(t+1) = H*P_x(t)*H^T + V2
		KAL_HCOVA2D covariance1 = _observation_mat2D * covariance * (_observation_mat2D.transpose());
		
		covariance1 += measure_var;

		return std::make_pair(mean1, covariance1);
	}

	KAL_HMEAN2D KalmanFilter::projectState2D(const KAL_MEAN &mean){
		return _observation_mat2D * mean.transpose();
	}

	// %%%%%%%%%%%%%%
	// %%% UPDATE %%%
	// %%%%%%%%%%%%%%

	KAL_DATA KalmanFilter::update(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const DETECTBOX3D &measurement,
			double dt)
	{
		// Project to measurement space
		KAL_HDATA3D pa = project3D(mean, covariance);
		KAL_HMEAN3D projected_mean = pa.first;
		KAL_HCOVA3D projected_cov = pa.second;

		// Compute the KALMAN GAIN solving efficiently the linear system:
		// (H*P(t)*H^T + V2) * K = P(t)*H^T
		// In other words:
		// K = (P(t)*H^T)(H*P(t)*H^T + V2)^-1
		Eigen::Matrix<float, 5, 8> B = (covariance * (_observation_mat3D.transpose())).transpose();
		// Computes the Cholesky decomposition and performs a triangular solve to find the Kalman gain.
		// The Cholesky decomposition is a method to factorize a symmetric positive-definite matrix into 
		// the product of a lower triangular matrix and its transpose. 
		// The resulting lower triangular matrix is used to solve a linear system efficiently.
		Eigen::Matrix<float, 8, 5> kalman_gain = (projected_cov.llt().solve(B)).transpose();

		// Calculate the Innovation or measurement ERROR
		// e(t) = y(t) - y~(t|t-1)
		Eigen::Matrix<float, 1, 5> innovation = measurement - projected_mean;

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

	KAL_DATA KalmanFilter::update(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const DETECTBOX2D &measurement,
			double dt)
	{
		// Project to measurement space
		KAL_HDATA2D pa = project2D(mean, covariance);
		KAL_HMEAN2D projected_mean = pa.first;
		KAL_HCOVA2D projected_cov = pa.second;

		// K = (P(t)*H^T)(H*P(t)*H^T + V2)^-1
		Eigen::Matrix<float, 4, 8> B = (covariance * (_observation_mat2D.transpose())).transpose();
		// Computes the Cholesky decomposition and performs a triangular solve to find the Kalman gain.
		Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose();

		// Calculate the Innovation or measurement ERROR
		// e(t) = y(t) - y~(t|t-1)
		Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean;

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


	Eigen::Matrix<float, 1, Eigen::Dynamic> KalmanFilter::gating_distance(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const std::vector<DETECTBOX3D> &measurements,
			bool only_position)
	{
		KAL_HDATA3D pa = this->project3D(mean, covariance);
		if (only_position) {
			printf("not implement!");
			exit(0);
		}

		KAL_HMEAN3D mean1 = pa.first;
		KAL_HCOVA3D covariance1 = pa.second;

		//    Eigen::Matrix<float, -1, 5, Eigen::RowMajor> d(size, 5);
		DETECTBOX3DSS d(measurements.size(), 5);
		int pos = 0;
		for (DETECTBOX3D box : measurements) {
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
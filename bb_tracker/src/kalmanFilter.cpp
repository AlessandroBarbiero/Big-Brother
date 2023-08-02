#include <bb_tracker/kalmanFilter.h>
#include <Eigen/Cholesky>

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
		// int ndim = 6; detection dim
		double dt = 1.;

		_motion_mat = Eigen::MatrixXf::Identity(state_dim, state_dim);
		for (int i = 0; i < detection_dim; i++) {
			_motion_mat(i, detection_dim + i) = dt;
		}
		_update_mat = Eigen::MatrixXf::Identity(detection_dim, state_dim);

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
		// revise the data;
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
		KAL_COVA motion_cov = tmp.asDiagonal();
		KAL_MEAN mean1 = this->_motion_mat * mean.transpose();
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
		KAL_HMEAN mean1 = _update_mat * mean.transpose();
		KAL_HCOVA covariance1 = _update_mat * covariance * (_update_mat.transpose());
		Eigen::Matrix<float, 6, 6> diag = std.asDiagonal();
		diag = diag.array().square().matrix();
		covariance1 += diag;
		//    covariance1.diagonal() << diag;
		return std::make_pair(mean1, covariance1);
	}

	KAL_DATA
		KalmanFilter::update(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const DETECTBOX &measurement)
	{
		KAL_HDATA pa = project(mean, covariance);
		KAL_HMEAN projected_mean = pa.first;
		KAL_HCOVA projected_cov = pa.second;

		//chol_factor, lower =
		//scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
		//kalmain_gain =
		//scipy.linalg.cho_solve((cho_factor, lower),
		//np.dot(covariance, self._upadte_mat.T).T,
		//check_finite=False).T
		Eigen::Matrix<float, 6, 12> B = (covariance * (_update_mat.transpose())).transpose();
		// Computes the Cholesky decomposition and performs a triangular solve to find the Kalman gain.
		Eigen::Matrix<float, 12, 6> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.12x6
		Eigen::Matrix<float, 1, 6> innovation = measurement - projected_mean; //eg.1x6
		auto tmp = innovation * (kalman_gain.transpose());
		KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();
		KAL_COVA new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());
		return std::make_pair(new_mean, new_covariance);
	}

	Eigen::Matrix<float, 1, -1>
		KalmanFilter::gating_distance(
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
#pragma once

#include "dataType.h"

namespace byte_kalman
{
	class KalmanFilter
	{
	public:
		static const double chi2inv95[10];
		KalmanFilter();
		KAL_DATA initiate(const DETECTBOX& measurement);
		void predict(KAL_MEAN& mean, KAL_COVA& covariance);
		KAL_HDATA project(const KAL_MEAN& mean, const KAL_COVA& covariance);
		KAL_DATA update(const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const DETECTBOX& measurement);

		/**
		 * Gating distance sets a threshold that defines a region around the predicted state within 
		 * which measurements are likely to be consistent with the prediction.
		 * Here it is calculated with the squared Mahalanobis distance for multiple measurements.
		 * The Mahalanobis distance plays a crucial role in the Kalman filter's update step,
		 * where it is used to assess the consistency of a measurement with the predicted state and decide how much weight 
		 * should be given to the measurement during the state update.
		 * This allows the Kalman filter to effectively incorporate noisy measurements while maintaining a balance between
		 * the prediction and measurement information.
		 * @param only_position: A boolean flag that determines whether only the position (coordinates)
		 * should be considered for the Mahalanobis distance calculation. (Not implemented yet)
		*/
		Eigen::Matrix<float, 1, -1> gating_distance(
			const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const std::vector<DETECTBOX>& measurements,
			bool only_position = false);

	private:
		Eigen::Matrix<float, 12, 12, Eigen::RowMajor> _motion_mat;
		Eigen::Matrix<float, 6, 12, Eigen::RowMajor> _update_mat;
		float _std_weight_position;
		float _std_weight_velocity;
		int detection_dim = 6;
		int state_dim = 12;
	};
}
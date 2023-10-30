#pragma once

#include "dataType.h"
#include <bb_tracker/ellipsoid_ellipse.hpp>
//TODO: add when finished to speed up
// #define DEIGEN_NO_DEBUG

namespace byte_kalman
{
	class KalmanFilter
	{
	public:
		static const double chi2inv95[10];
		KalmanFilter();
		KAL_DATA initiate3D(const DETECTBOX3D& measurement);
		KAL_DATA initiate2D(const DETECTBOX2D& measurement, TRANSFORMATION &V, PROJ_MATRIX &P);

		/**
		 * Predicts the Kalman filter state and covariance at the next step and returns them throught the input parameters
		 * The prediction is made using the relative motion model and std_weights.
		 * Equations:
		 * x(t+1|t) = F*x(t|t-1)
		 * P_x(t+1) = F*P_x(t)*F^T + V1
		 * @param mean The current state x(t) of the Kalman Filter - return x~(t+1)
		 * @param covariance The current covariance P_x(t) of the Kalman Filter state - return P_x~(t+1)
		 * @param dt Time passed from the last measurement in seconds
		*/
		void predict(KAL_MEAN& mean, KAL_COVA& covariance, double dt);

		// %%%%%%%%%%%%%%%
		// %%% PROJECT %%%
		// %%%%%%%%%%%%%%%
		/**
		 * Project the current state of the Kalman Filter into measurement space 3D.
		 * Equations:
		 * y~(t|t-1) = H*x(t|t-1)
		 * P_y(t+1) = H*P_x(t)*H^T + V2
		 * @param mean The current state x(t) of the Kalman Filter
		 * @param covariance The current covariance P_x(t) of the Kalman Filter state
		 * @return The projected output y(t|t-1) mean and covariance P_y(t) in 3 dimensions
		*/
		virtual KAL_HDATA3D project3D(const KAL_MEAN& mean, const KAL_COVA& covariance);
		/**
		 * Project the current state of the Kalman Filter into measurement space 2D.
		 * Equations:
		 * y~(t|t-1) = H*x(t|t-1)
		 * P_y(t+1) = H*P_x(t)*H^T + V2
		 * @param mean The current state x(t) of the Kalman Filter
		 * @param covariance The current covariance P_x(t) of the Kalman Filter state
		 * @return The projected output y(t|t-1) mean and covariance P_y(t) in 2 dimensions
		*/
		virtual KAL_HDATA2D project2D(const KAL_MEAN& mean, const KAL_COVA& covariance);

		// %%%%%%%%%%%%%%
		// %%% UPDATE %%%
		// %%%%%%%%%%%%%%
		/**
		 * Update the state estimate based on the actual measurement received at time t.
		 * 
		 * Equations:
		 * e(t) = y(t) - y~(t|t-1)
		 * K(t) = (P*H^T) * (H*P*H^T + V2)^-1
		 * x(t|t) = x(t+1|t) + K(t) * e(t)
		 * P(t+1|t) = (F*P*F^T + V1) - (F*P*H^T)*(H*P*H^T + V2)^-1 *(F*P*H^T)^T
		 * @param mean        The current state mean estimate (x(t|t-1))
		 * @param covariance  The current state covariance estimate (P(t|t-1))
		 * @param measurement The measurement 3D obtained at time t
		 * 
		 * @return A pair containing the updated state mean estimate (x(t|t)) and covariance (P(t|t))
		*/
		KAL_DATA update3D(const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const DETECTBOX3D& measurement
			);
		/**
		 * Update the state estimate based on the actual measurement received at time t.
		 * 
		 * Equations:
		 * e(t) = y(t) - y~(t|t-1)
		 * K(t) = (P*H^T) * (H*P*H^T + V2)^-1
		 * x(t|t) = x(t+1|t) + K(t) * e(t)
		 * P(t+1|t) = (F*P*F^T + V1) - (F*P*H^T)*(H*P*H^T + V2)^-1 *(F*P*H^T)^T
		 * @param mean        The current state mean estimate (x(t|t-1))
		 * @param covariance  The current state covariance estimate (P(t|t-1))
		 * @param measurement The measurement 2D obtained at time t
		 * 
		 * @return A pair containing the updated state mean estimate (x(t|t)) and covariance (P(t|t))
		*/
		KAL_DATA update2D(const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const DETECTBOX2D& measurement
			);


		/**
		 * Gating distance sets a threshold that defines a region around the predicted state within 
		 * which measurements are likely to be consistent with the prediction.
		 * Here it is calculated with the squared Mahalanobis distance for multiple measurements.
		 * The Mahalanobis distance plays a crucial role in the Kalman filter's update step,
		 * where it is used to assess the consistency of a measurement with the predicted state and decide how much weight 
		 * should be given to the measurement during the state update.
		 * This allows the Kalman filter to effectively incorporate noisy measurements while maintaining a balance between
		 * the prediction and measurement information.
		 * @param mean: The predicted state mean (a vector)
		 * @param covariance: The predicted state covariance matrix
		 * @param measurements: A vector of measurements (each represented as a DETECTBOX vector)
		 * @param only_position: A boolean flag that determines whether only the position (coordinates)
		 * should be considered for the Mahalanobis distance calculation. (Not implemented yet)
		 * @return The squared Mahalanobis distance for each measurement with respect to the predicted state.
		 * The result is represented as a row vector of floating-point values (Eigen::Matrix<float, 1, -1>)
		*/
		Eigen::Matrix<float, 1, Eigen::Dynamic> gating_distance(
			const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const std::vector<DETECTBOX3D>& measurements,
			bool only_position = false);

	protected: 
		virtual KAL_MEAN predictState(KAL_MEAN &mean, double dt);
		virtual KAL_HMEAN3D projectState3D(const KAL_MEAN &mean);
		virtual KAL_HMEAN2D projectState2D(const KAL_MEAN &mean);

	protected:
		Eigen::Matrix<float, 8, 8, Eigen::RowMajor> _motion_mat;			// F
		Eigen::Matrix<float, 5, 8, Eigen::RowMajor> _observation_mat3D; 	// H_3D
		Eigen::Matrix<float, 5, 8, Eigen::RowMajor> _observation_mat2D; 	// H_2D
		float _std_weight_position;
		float _std_weight_velocity;
		int detection3D_dim = 5;
		int detection2D_dim = 5;
		int state_dim = 8;
	};
}
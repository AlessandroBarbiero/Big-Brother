#pragma once

#include <memory>
#include "dataType.h"
#include <bb_tracker/kalmanFilter.h>
#include <bb_tracker/ellipsoid_ellipse.hpp>

namespace byte_kalman
{
	class EKF : public KalmanFilter
	{
	public:

		void predict(KAL_MEAN& mean, KAL_COVA& covariance, double dt);

		KAL_HDATA2D project2D(const KAL_MEAN& mean, const KAL_COVA& covariance);
		KAL_HDATA3D project3D(const KAL_MEAN& mean, const KAL_COVA& covariance);

		KAL_DATA initiate2D(const DETECTBOX2D& measurement, ClassLabel class_label);

		std::shared_ptr<TRANSFORMATION> V; 	// View Matrix
		std::shared_ptr<PROJ_MATRIX> P; 	// Projection Matrix

	protected: 
		// Call this before the update, the update will use the updated jacobians
        void computeJacobianMotion(const KAL_MEAN &mean, double dt);
		void computeJacobianObservation2D(const KAL_MEAN &mean);
		// void computeJacobianObservation3D(const KAL_MEAN &mean);
		KAL_MEAN predictState(KAL_MEAN &mean, double dt);
		KAL_HMEAN3D projectState3D(const KAL_MEAN &mean);
		KAL_HMEAN2D projectState2D(const KAL_MEAN &mean);
	};
}
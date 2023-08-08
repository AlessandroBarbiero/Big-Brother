#pragma once

#include "dataType.h"
#include <bb_tracker/kalmanFilter.h>

namespace byte_kalman
{
	class EKF : public KalmanFilter
	{
	public:

		void predict(KAL_MEAN& mean, KAL_COVA& covariance, double dt);

		KAL_HDATA2D project2D(const KAL_MEAN& mean, const KAL_COVA& covariance);
		KAL_HDATA3D project3D(const KAL_MEAN& mean, const KAL_COVA& covariance);

	protected: 
		// Call this before the update, the update will use the updated jacobians
        void computeJacobians(const KAL_MEAN &mean);
		KAL_MEAN predictState(KAL_MEAN &mean, double dt);
		KAL_HMEAN3D projectState3D(const KAL_MEAN &mean);
		KAL_HMEAN2D projectState2D(const KAL_MEAN &mean);
	};
}
#include <bb_tracker/EKF.hpp>
#include <iostream>

namespace byte_kalman
{
	KAL_MEAN EKF::predictState(KAL_MEAN &mean, double dt){
    	KAL_MEAN predict_mean;
    	predict_mean << mean(0) + mean(6)*cos(mean(2))*dt,
						mean(1) + mean(6)*sin(mean(2))*dt,
						mean(2) + mean(7)*dt,
						mean(3),
						mean(4),
						mean(5),
						mean(6),
						mean(7);
  
    	return predict_mean;
	}

  	KAL_HMEAN3D EKF::projectState3D(const KAL_MEAN &mean){
		return _observation_mat3D * mean.transpose();
	}

  	KAL_HMEAN2D EKF::projectState2D(const KAL_MEAN &mean){
		// TODO: add convertion from world to camera
		return _observation_mat2D * mean.transpose();
	}

	void EKF::computeJacobians(const KAL_MEAN &mean){
      	_motion_mat = Eigen::MatrixXf::Identity(state_dim, state_dim);
		_motion_mat(0,6) = cos(mean(2));
		_motion_mat(0,2) = - mean(0)*sin(mean(2));
		_motion_mat(1,6) = sin(mean(2));
		_motion_mat(1,2) = mean(1)*cos(mean(2));
		_motion_mat(2,7) = 1.0;
		
      	// TODO: compute jacobian and save it into _observation_mat2D
  	}

	void EKF::predict(KAL_MEAN& mean, KAL_COVA& covariance, double dt){
		computeJacobians(mean);
		KalmanFilter::predict(mean, covariance, dt);
	}

	KAL_HDATA2D EKF::project2D(const KAL_MEAN& mean, const KAL_COVA& covariance){
		computeJacobians(mean);
		return KalmanFilter::project2D(mean, covariance);
	}

	KAL_HDATA3D EKF::project3D(const KAL_MEAN& mean, const KAL_COVA& covariance){
		computeJacobians(mean);
		return KalmanFilter::project3D(mean, covariance);
	}

}
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
		// To project the 3d mean into a 2d world i need 
		// Transform the point from world tf to camera tf multiplying by a transform matrix
		// (Optional check if behind the camera (z>0))
		// camera projection matix P[4x4] (PinholeCameraModel.projectionMatrix()), 
		// convert point in homogeneous coordinates [x,y,z,1]
		// X_projected = P * X_homogeneous
		// X_normalized = X_projected / X_projected[3]
		// x,y are u,v 
		// check if it is into the view of the camera (0<x<width, 0<y<height)
		
		// TODO: add convertion from world to camera
		return _observation_mat2D * mean.transpose();
	}

	void EKF::computeJacobians(const KAL_MEAN &mean){
      	_motion_mat = Eigen::MatrixXf::Identity(state_dim, state_dim);
		_motion_mat(0,6) = cos(mean(2));
		_motion_mat(0,2) = - mean(6)*sin(mean(2));
		_motion_mat(1,6) = sin(mean(2));
		_motion_mat(1,2) = mean(6)*cos(mean(2));
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
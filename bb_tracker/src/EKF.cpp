#include <bb_tracker/EKF.hpp>
#include <iostream>

namespace byte_kalman
{
	KAL_DATA EKF::initiate2D(const DETECTBOX2D& measurement, ClassLabel class_label){
		return KalmanFilter::initiate2D(measurement, class_label, *V, *P);
	}

	KAL_MEAN EKF::predictState(KAL_MEAN &mean, double dt){
    	KAL_MEAN predict_mean;
		// Euler integration
		// predict_mean << mean(0) + mean(6)*cos(mean(2))*dt,
		// 				mean(1) + mean(6)*sin(mean(2))*dt,
		// Runge-Kutta integration
    	predict_mean << mean(0) + mean(6)*dt*cos(mean(2)+(mean(7)*dt)/2),
						mean(1) + mean(6)*dt*sin(mean(2)+(mean(7)*dt)/2),
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
		return ellipseFromEllipsoidv2(mean, *V, *P);
	}

	void EKF::computeJacobianMotion(const KAL_MEAN &mean, double dt){
      	_motion_mat = Eigen::MatrixXf::Identity(state_dim, state_dim);
		// Euler integration
		// _motion_mat(0,2) = - mean(6)*sin(mean(2))*dt;
		// _motion_mat(0,6) = cos(mean(2))*dt;
		// _motion_mat(1,2) = mean(6)*cos(mean(2))*dt;
		// _motion_mat(1,6) = sin(mean(2))*dt;
		// _motion_mat(2,7) = 1.0*dt;

		// Runge-Kutta integration
		_motion_mat(0,2) = -dt*mean(6)*sin(dt*mean(7)/2 + mean(2));
		_motion_mat(0,6) = dt*cos(dt*mean(7)/2 + mean(2));
		_motion_mat(0,7) = -dt*dt*mean(6)*sin(dt*mean(7)/2 + mean(2))/2;
		_motion_mat(1,2) = dt*mean(6)*cos(dt*mean(7)/2 + mean(2));
		_motion_mat(1,6) = dt*sin(dt*mean(7)/2 + mean(2));
		_motion_mat(1,7) = dt*dt*mean(6)*cos(dt*mean(7)/2 + mean(2))/2;
		_motion_mat(2,7) = 1.0*dt;
  	}

	#include "computeJacobianEllipse.cpp"

	void EKF::computeJacobianObservation2D(const KAL_MEAN &mean){
		// State
		float x = mean(0),
		y 		= mean(1),
		theta 	= mean(2),
		l_ratio = mean(3),
		d_ratio = mean(4),
		h =       mean(5),
		// v =    mean(6),
		// w =    mean(7),
		// View
		vr00 	= (*V)(0,0),
		vr01 	= (*V)(0,1),
		vr02 	= (*V)(0,2),
		vr10 	= (*V)(1,0),
		vr11 	= (*V)(1,1),
		vr12 	= (*V)(1,2),
		vr20 	= (*V)(2,0),
		vr21 	= (*V)(2,1),
		vr22 	= (*V)(2,2),
		vtx 	= (*V)(0,3),
		vty 	= (*V)(1,3),
		vtz 	= (*V)(2,3),
		// Projection
		fx 		= (*P)(0,0),
		fy 		= (*P)(1,1),
		cx 		= (*P)(0,2),
		cy 		= (*P)(1,2);
		_observation_mat2D = computeJacobianEllipse(x,y,theta,l_ratio,d_ratio,h,vr00,vr01,vr02,vr10,vr11,vr12,vr20,vr21,vr22,vtx,vty,vtz,fx,fy,cx,cy);

	}

	void EKF::predict(KAL_MEAN& mean, KAL_COVA& covariance, double dt){
		computeJacobianMotion(mean, dt);
		KalmanFilter::predict(mean, covariance, dt);
	}

	KAL_HDATA2D EKF::project2D(const KAL_MEAN& mean, const KAL_COVA& covariance){
		computeJacobianObservation2D(mean);
		return KalmanFilter::project2D(mean, covariance);
	}

	KAL_HDATA3D EKF::project3D(const KAL_MEAN& mean, const KAL_COVA& covariance){
		// computeJacobianObservation3D(mean);  // It is not necessary, it is linear, H is definite
		return KalmanFilter::project3D(mean, covariance);
	}

}
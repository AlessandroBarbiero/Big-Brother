#include <bb_tracker/STrack.h>

#define ACCEPT_OLD_DETECTION // Add this define to accept older detections

size_t STrack::last_points_capacity = 10;

std::unordered_map<int, std::string> STrack::trackStateToString{
	{0,"New"},
	{1,"Tracked"},
	{2,"Lost"},
	{3,"Removed"}
};

STrack::STrack(Object2D *obj, ClassLabel class_label)
{
	this->vis2D_tlbr = obj->tlbr;

	is_activated = false;
	track_id = 0;
	state = TrackState::New;

	minwdh.resize(6);
	minmax.resize(6);
	
	frame_id = 0;
	tracklet_len = 0;
	this->state_current.confidence = obj->prob;
	this->state_current.label = class_label;
	start_frame = 0;
	state_current.time_ms = obj->time_ms;

	this->last_points.set_capacity(STrack::last_points_capacity);
}

STrack::STrack(vector<float> minwdh_, float score, ClassLabel class_label, int64_t time_ms)
{
	_minwdh.resize(6);
	_minwdh.assign(minwdh_.begin(), minwdh_.end());

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	minwdh.resize(6);
	minmax.resize(6);

	static_minwdh();
	static_minmax();
	frame_id = 0;
	tracklet_len = 0;
	this->state_current.confidence = score;
	this->state_current.label = class_label;
	start_frame = 0;
	state_current.time_ms = time_ms;

	this->last_points.set_capacity(STrack::last_points_capacity);
}

STrack::~STrack()
{
	// std::cout<<"track deleted, id: "<< track_id << " size: " << path_marker.points.size() << std::endl;
}

void STrack::activate3D(byte_kalman::EKF &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id();

	vector<float> _minwdh_tmp(6);
	_minwdh_tmp[0] = this->_minwdh[0];
	_minwdh_tmp[1] = this->_minwdh[1];
	_minwdh_tmp[2] = this->_minwdh[2];
	_minwdh_tmp[3] = this->_minwdh[3];
	_minwdh_tmp[4] = this->_minwdh[4];
	_minwdh_tmp[5] = this->_minwdh[5];
	vector<float> xyzaah = minwdh_to_xyzaah(_minwdh_tmp);

	DETECTBOX3D xy_yaw_aah_box;
	xy_yaw_aah_box[0] = xyzaah[0];	// x
	xy_yaw_aah_box[1] = xyzaah[1];	// y
	xy_yaw_aah_box[2] = this->theta;
	xy_yaw_aah_box[3] = xyzaah[3];	// w/h
	xy_yaw_aah_box[4] = xyzaah[4];	// d/h
	xy_yaw_aah_box[5] = xyzaah[5];	// h
	auto mc = this->kalman_filter.initiate3D(xy_yaw_aah_box);

	this->state_current.mean = mc.first;
	this->state_current.covariance = mc.second;
	this->theta = state_current.mean[2];
	this->state_predicted = this->state_current;

	this->last_detection = {xy_yaw_aah_box, state_current.label, state_current.confidence};

	static_minwdh();
	static_minmax();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}

	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

void STrack::activate2D(byte_kalman::EKF &kalman_filter, TRANSFORMATION &V, PROJ_MATRIX &P, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id();

	DETECTBOX2D xyabt_box;
	xyabt_box[0] = (vis2D_tlbr[0] + vis2D_tlbr[2])/2.0;
	xyabt_box[1] = (vis2D_tlbr[1] + vis2D_tlbr[3])/2.0;
	xyabt_box[2] = (vis2D_tlbr[2] - vis2D_tlbr[0])/2.0;
	xyabt_box[3] = (vis2D_tlbr[3] - vis2D_tlbr[1])/2.0;
	xyabt_box[4] = 0;
	if(xyabt_box[2]<xyabt_box[3]){
		// a is the longest semi-axis, b the shortest
		std::swap(xyabt_box[2], xyabt_box[3]);
		xyabt_box[4] = M_PI / 2.0;  // 90°
	}

	auto mc = static_cast<byte_kalman::KalmanFilter>(this->kalman_filter).initiate2D(xyabt_box, state_current.label, V, P);

	this->state_current.mean = mc.first;
	this->state_current.covariance = mc.second;
	this->theta = state_current.mean[2];
	this->state_predicted = this->state_current;

	auto sh_P = std::make_shared<PROJ_MATRIX>(P);
	auto sh_V = std::make_shared<TRANSFORMATION>(V);
	DETECTION2D det2d = {xyabt_box, sh_V, sh_P};
	last_detection = {det2d, state_current.label, state_current.confidence};

	// The state has to be updated before the static minwdh in 2D
	this->state = TrackState::Tracked;
	static_minwdh();
	static_minmax();

	this->tracklet_len = 0;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

void STrack::re_activate(Object2D &new_track, int frame_id, bool new_id)
{
	vector<float> tlbr = new_track.tlbr;

	DETECTBOX2D xyabt_box;
	xyabt_box[0] = (tlbr[0]+tlbr[2])/2.0;	// x
	xyabt_box[1] = (tlbr[1]+tlbr[3])/2.0;	// y
	xyabt_box[2] = (tlbr[2]-tlbr[0])/2.0;	// a
	xyabt_box[3] = (tlbr[3]-tlbr[1])/2.0;	// b
	xyabt_box[4] = 0;						// theta
	if(xyabt_box[2]<xyabt_box[3]){
		std::swap(xyabt_box[2], xyabt_box[3]);
		xyabt_box[4] = M_PI / 2.0;  // 90°
	}

	if(new_track.time_ms > this->state_current.time_ms){
		DETECTION2D det2d = {xyabt_box, kalman_filter.V, kalman_filter.P};
		last_detection = {det2d, new_track.label, new_track.prob};
	}

	auto mc = this->kalman_filter.update2D(this->state_predicted.mean, this->state_predicted.covariance, xyabt_box);

	updateTrackState(mc, new_track.time_ms, new_track.prob, new_track.label, frame_id, true);

	if (new_id)
		this->track_id = next_id();
}

void STrack::re_activate(STrack &new_track, int frame_id, bool new_id)
{
	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);

	DETECTBOX3D xy_yaw_aah_box;
	xy_yaw_aah_box[0] = xyzaah[0];	// x
	xy_yaw_aah_box[1] = xyzaah[1];	// y
	xy_yaw_aah_box[2] = new_track.theta;
	xy_yaw_aah_box[3] = xyzaah[3];	// w/h
	xy_yaw_aah_box[4] = xyzaah[4];	// d/h
	xy_yaw_aah_box[5] = xyzaah[5];	// h

	if(new_track.state_current.time_ms > this->state_current.time_ms){
		last_detection = {xy_yaw_aah_box, new_track.state_current.label, new_track.state_current.confidence};
	}

	auto mc = this->kalman_filter.update3D(this->state_predicted.mean, this->state_predicted.covariance, xy_yaw_aah_box);

	updateTrackState(mc, new_track.state_current.time_ms, new_track.state_current.confidence, new_track.state_current.label, frame_id, true);

	if (new_id)
		this->track_id = next_id();
}

bool STrack::checkOldDetection(int64_t detection_time_ms){
	if(this->state_current.time_ms > detection_time_ms){
		// Saved state is more updated than detection, just delete the projection
		this->state_predicted = this->state_current;
		static_minwdh();
		static_minmax();
		return true;
	}
	return false;
}

void STrack::updateClassLabel(KAL_STATE& state, ClassLabel new_label, float new_score){
	// This is the maximum value in meters that the dimensions can differ from prior
	static float margin = 1.0; 

	if(state.label == new_label){
		// The new score is the average between the old*2 and new
		state.confidence = (state.confidence*2 + new_score)/3;
		auto priorDim = priorDimensions.at(state.label);
		// Respect the priors
		if(		state.mean(5) > (priorDim[2]+margin)  ||
				state.mean(3) > (priorDim[0]+margin)/priorDim[2]  ||
				state.mean(4) > (priorDim[1]+margin)/priorDim[2])
		{
			state.mean(5) = priorDim[2];
			state.mean(3) = priorDim[0]/priorDim[2];
			state.mean(4) = priorDim[1]/priorDim[2];
		}
	}
	else{
		// The score is decreased and new priors are applied if object change class
		state.confidence = state.confidence - new_score/3;
		if(state.confidence < 0.5){
			state.label = new_label;
			// Apply new priors
			auto priorDim = priorDimensions.at(state.label);
			state.mean(3) = priorDim[0]/priorDim[2]; //l_ratio
			state.mean(4) = priorDim[1]/priorDim[2]; //d_ratio
			state.mean(5) = priorDim[2];			   //h
		}
	}
}

void STrack::updateTrackState(KAL_DATA& updated_values, int64_t detection_time_ms, float new_score, ClassLabel new_label, int frame_id, bool reset_tracklet_len){

	if(reset_tracklet_len)
		this->tracklet_len = 0;
	else
		this->tracklet_len++;
	this->frame_id = frame_id;

	if(detection_time_ms >= state_current.time_ms){ // Det in Future or second detection
		state_past = state_current;
		state_current.time_ms = detection_time_ms;
		state_current.mean = updated_values.first;
		state_current.covariance = updated_values.second;
		updateClassLabel(state_current, new_label, new_score);
		state_predicted = state_current;
		theta = state_current.mean(2);
	}
	else{	//Det in Past, update with last detection

		// cout<<"\n-----> Update track n " << this->track_id << " with a detection in the past: " << 
		// 	"\ncurrent\t\t" << state_current.time_ms <<
		// 	"\npast\t\t" << state_past.time_ms <<
		// 	"\ndetection\t" << detection_time_ms <<
		// 	"\ntracklet len: " << tracklet_len <<
		// 	"\nstate current:\n" << state_current.mean <<
		// 	"\nstate past:\n" << state_past.mean <<
		// 	"\nstate predict:\n"<< state_predicted.mean <<
		// 	"\nupdated value (new past value):\n"<< updated_values.first <<
		// 	"\nlast detection:\n"<< (last_detection.detection.index()==1 ? "2D" : "3D") <<
		// 	endl;

		assert(is_activated);
		// 1. Finalize first update with detection in the past
		state_past.time_ms = detection_time_ms;
		state_past.mean = updated_values.first;
		state_past.covariance = updated_values.second;
		updateClassLabel(state_past, new_label, new_score);
		// 2. Predict and update with last detection saved
		// 2.1 Predict updated past state at time of current
		double dt = static_cast<double>(state_current.time_ms - state_past.time_ms)/MILLIS_IN_SECONDS;
		state_predicted = state_past;
		kalman_filter.predict(state_predicted.mean, state_predicted.covariance, dt);
		// 2.2 Update with the last detection based on type (2D/3D)
		KAL_DATA mc;
		switch(last_detection.detection.index()){
			case 1: { //2D
				DETECTION2D last_det2d = std::get<DETECTION2D>(last_detection.detection);
				setViewProjection(last_det2d.V, last_det2d.P);
				mc = this->kalman_filter.update2D(this->state_predicted.mean, this->state_predicted.covariance, last_det2d.detectbox);
				break;
			}
			case 2: //3D
				mc = this->kalman_filter.update3D(this->state_predicted.mean, this->state_predicted.covariance, std::get<DETECTBOX3D>(last_detection.detection));
				break;
		}
		state_current.mean = mc.first;
		state_current.covariance = mc.second;
		theta = state_current.mean(2);
		// 2.3 Work on class labels
		state_current.label = state_past.label;
		state_current.confidence = state_past.confidence;
		updateClassLabel(state_current, last_detection.label, last_detection.confidence);
		state_predicted = state_current;

		// cout<<"state current after updating the updated with the last detection:" << 
		// 	"\nstate current:\n" << state_current.mean << 
		// 	"\n" << endl;
	}

	static_minwdh();
	static_minmax();

	this->state = TrackState::Tracked;
	this->is_activated = true;
}

KAL_DATA STrack::correctFirstOrientation2D(DETECTBOX2D second_detection){
	double yaw;
	Eigen::Vector3f second_point;
	double dx,dy;
	second_point = projectPoint2D({second_detection[0], second_detection[1]}, state_current.mean(5)/2.0, *this->kalman_filter.V, *this->kalman_filter.P);
	dx = second_point(0) - state_current.mean(0);
	dy = second_point(1) - state_current.mean(1);

	yaw = std::atan2(dy, dx);
	this->state_predicted.mean(2) = yaw;

	// Update the state, this is useful to find the starting velocity
	auto mc = this->kalman_filter.update2D(this->state_predicted.mean, this->state_predicted.covariance, second_detection);

	mc.first(2) = yaw;
	mc.first(0) = second_point(0);
	mc.first(1) = second_point(1);

	return mc;
}

void STrack::update(Object2D &new_track, int frame_id)
{
	auto detection_time_ms = new_track.time_ms;
	bool det_in_future = detection_time_ms >= state_current.time_ms;

	#ifndef ACCEPT_OLD_DETECTION
	if(checkOldDetection(detection_time))
		return; // Saved state is more updated than detection, do not update
	#endif

	vector<float> tlbr = new_track.tlbr;
	DETECTBOX2D xyabt_box;
	xyabt_box[0] = (tlbr[0]+tlbr[2])/2.0;	// x
	xyabt_box[1] = (tlbr[1]+tlbr[3])/2.0;	// y
	xyabt_box[2] = (tlbr[2]-tlbr[0])/2.0;	// a
	xyabt_box[3] = (tlbr[3]-tlbr[1])/2.0;	// b
	xyabt_box[4] = 0;						// theta
	if(xyabt_box[2]<xyabt_box[3]){
		std::swap(xyabt_box[2], xyabt_box[3]);
		xyabt_box[4] = M_PI / 2.0;  // 90°
	}

	if(det_in_future){
		DETECTION2D det2d = {xyabt_box, kalman_filter.V, kalman_filter.P};
		last_detection = {det2d, new_track.label, new_track.prob};
	}

	KAL_DATA mc;
	if(!this->is_activated){
		if(det_in_future)
			mc = correctFirstOrientation2D(xyabt_box);
		else {
			// First two detections are t1 > t2, second detection in the past
			// 1. Initiate a new track with values from the first detection in time
			auto mc = this->kalman_filter.initiate2D(xyabt_box, new_track.label);
			this->state_current.confidence = new_track.prob;
			this->state_current.mean = mc.first;
			this->state_current.covariance = mc.second;
			this->theta = state_current.mean[2];
			
			// 2. Predict the state at the time of the second detection in time, first received
			double dt = static_cast<double>(state_current.time_ms - detection_time_ms)/MILLIS_IN_SECONDS;
			this->state_predicted = this->state_current;
			kalman_filter.predict(state_predicted.mean, state_predicted.covariance, dt);

			// 3. Update the object exploiting the saved detection
			switch(last_detection.detection.index()){
				case 1: { //2D
					DETECTION2D last_det2d = std::get<DETECTION2D>(last_detection.detection);
					setViewProjection(last_det2d.V, last_det2d.P);
					mc = correctFirstOrientation2D(last_det2d.detectbox);
					break;
				}
				case 2: //3D
					mc = this->kalman_filter.update3D(this->state_predicted.mean, this->state_predicted.covariance, std::get<DETECTBOX3D>(last_detection.detection));
					break;
			}

			updateTrackState(mc, state_current.time_ms, last_detection.confidence, last_detection.label, frame_id);
			return;
		}
	} else {
		// Update the state, it can be done on the prediction of the past or the current state, previously predicted by multi_predict
		mc = this->kalman_filter.update2D(this->state_predicted.mean, this->state_predicted.covariance, xyabt_box);
	}

	updateTrackState(mc, detection_time_ms, new_track.prob, new_track.label, frame_id);
}

void STrack::update(STrack &new_track, int frame_id)
{
	auto detection_time_ms = new_track.state_current.time_ms;

	#ifndef ACCEPT_OLD_DETECTION
	if(checkOldDetection(detection_time_ms))
		return; // Saved state is more updated than detection, do not update
	#endif

	// Handle detection and update
	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);

	DETECTBOX3D xy_yaw_aah_box;
	xy_yaw_aah_box[0] = xyzaah[0];	// x
	xy_yaw_aah_box[1] = xyzaah[1];	// y
	xy_yaw_aah_box[2] = new_track.theta;
	xy_yaw_aah_box[3] = xyzaah[3];	// w/h
	xy_yaw_aah_box[4] = xyzaah[4];	// d/h
	xy_yaw_aah_box[5] = xyzaah[5];	// h

	if(detection_time_ms >= state_current.time_ms){
		last_detection = {xy_yaw_aah_box, new_track.state_current.label, new_track.state_current.confidence};
	}

	auto mc = this->kalman_filter.update3D(this->state_predicted.mean, this->state_predicted.covariance, xy_yaw_aah_box);

	auto new_score = new_track.state_current.confidence;
	updateTrackState(mc, detection_time_ms, new_score, new_track.state_current.label, frame_id);
}

void STrack::static_minwdh()
{
	if (this->state == TrackState::New)
	{
		minwdh[0] = _minwdh[0];
		minwdh[1] = _minwdh[1];
		minwdh[2] = _minwdh[2];
		minwdh[3] = _minwdh[3];
		minwdh[4] = _minwdh[4];
		minwdh[5] = _minwdh[5];
		return;
	}

	minwdh[0] = state_current.mean[0];	// x center
	minwdh[1] = state_current.mean[1];	// y center
	minwdh[2] = 0.0;		// z min
	minwdh[3] = state_current.mean[3];	// w/h
	minwdh[4] = state_current.mean[4];	// d/h
	minwdh[5] = state_current.mean[5];	// h 


	minwdh[3] *= minwdh[5];			// w
	minwdh[4] *= minwdh[5];			// d
	minwdh[0] -= minwdh[3] / 2;		// x min
	minwdh[1] -= minwdh[4] / 2;		// y min

}

void STrack::static_minwdh_predicted()
{
	minwdh[0] = state_predicted.mean[0];	// x center
	minwdh[1] = state_predicted.mean[1];	// y center
	minwdh[2] = 0.0;				// z min
	minwdh[3] = state_predicted.mean[3];	// w/h
	minwdh[4] = state_predicted.mean[4];	// d/h
	minwdh[5] = state_predicted.mean[5];	// h 

	minwdh[3] *= minwdh[5];			// w
	minwdh[4] *= minwdh[5];			// d
	minwdh[0] -= minwdh[3] / 2;		// x min
	minwdh[1] -= minwdh[4] / 2;		// y min
}

void STrack::static_minmax()
{
	minmax.clear();
	minmax.assign(minwdh.begin(), minwdh.end());

	minmax[3] += minmax[0];		// x max
	minmax[4] += minmax[1];		// y max
	minmax[5] += minmax[2];		// z max
}

vector<float> STrack::minwdh_to_xyzaah(vector<float> minwdh_tmp)
{
	vector<float> minwdh_output = minwdh_tmp;
	minwdh_output[0] += minwdh_output[3] / 2;	// x center
	minwdh_output[1] += minwdh_output[4] / 2;	// y center
	minwdh_output[2] += minwdh_output[5] / 2;	// z center
	minwdh_output[3] /= minwdh_output[5];		// w/h
	minwdh_output[4] /= minwdh_output[5];		// d/h
	return minwdh_output;
}

vector<float> STrack::to_xyzaah()
{
	return minwdh_to_xyzaah(minwdh);
}

vector<float> STrack::minmax_to_minwdh(vector<float> &minmax)
{
	minmax[3] -= minmax[0];		// w
	minmax[4] -= minmax[1]; 	// d
	minmax[5] -= minmax[2]; 	// h
	return minmax;
}

void STrack::mark_lost()
{
	state = TrackState::Lost;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id()
{
	static int _count = 0;
	_count++;
	return _count;
}

int STrack::end_frame()
{
	return this->frame_id;
}

// Predict all the stracks respect to the detection time, if the detection happens before the current time, predict respect to the state saved before the current
void STrack::multi_predict(vector<STrack*> &stracks, byte_kalman::EKF &kalman_filter, int64_t detection_time_ms)
{
	for (unsigned int i = 0; i < stracks.size(); i++)
	{
		double dt;
		if(!stracks[i]->is_activated || detection_time_ms >= stracks[i]->state_current.time_ms){
			// Detection in the future or there is not a past state saved
			dt = static_cast<double>(detection_time_ms - stracks[i]->state_current.time_ms)/MILLIS_IN_SECONDS;
			stracks[i]->state_predicted = stracks[i]->state_current;
		}
		else{
			// Detection in the past, project respect to old detection
			dt = static_cast<double>(detection_time_ms - stracks[i]->state_past.time_ms)/MILLIS_IN_SECONDS;
			stracks[i]->state_predicted = stracks[i]->state_past;
			//cout << "id: "<< stracks[i]->track_id << " - delta t = " << dt << endl;
		}

		kalman_filter.predict(stracks[i]->state_predicted.mean, stracks[i]->state_predicted.covariance, dt);

		stracks[i]->static_minwdh_predicted();
		stracks[i]->static_minmax();
	}
}



// Project the predicted mean into the image, separate the objects that are outside the image in another vector
void STrack::multi_project(vector<STrack*> &stracks, vector<STrack*> &outside_image, PROJ_MATRIX& P, TRANSFORMATION& V, uint32_t width, uint32_t height){
	// 1. filter out objects behind the camera
	auto isBehindCamera = [&](STrack* x){ return objectBehindCamera(x->state_predicted.mean[0], x->state_predicted.mean[1], x->state_predicted.mean[5]/2.0, V); };
	auto moveIt = std::remove_if(stracks.begin(), stracks.end(), isBehindCamera);
	outside_image.insert(outside_image.end(), std::make_move_iterator(moveIt), std::make_move_iterator(stracks.end()));
	stracks.erase(moveIt, stracks.end());

	// 2. project the ellipsoid
	for (unsigned int i = 0; i < stracks.size(); i++){
		ELLIPSE_STATE e_state = ellipseFromEllipsoidv2(stracks[i]->state_predicted.mean, V, P);
		// Define an Axis Aligned Bounding Box from the ellipse (it is used for the IoU) 
		stracks[i]->vis2D_tlbr = tlbrFromEllipse(e_state);
	}

	// 3. filter out objects outside image
	// auto isOutsideImage = [&](STrack* x){ return !boxInImage(x->vis2D_tlbr[0], x->vis2D_tlbr[1], x->vis2D_tlbr[2], x->vis2D_tlbr[3], width, height); };
	
	// Remove only objects with the center outside the image (Keep it if rectangle goes outside)
	auto isOutsideImage = [&](STrack* x){ 
		float cx, cy;
		cx = (x->vis2D_tlbr[0]+x->vis2D_tlbr[2])/2.0;
		cy = (x->vis2D_tlbr[1]+x->vis2D_tlbr[3])/2.0;

		return (cx<-10 || cx>width+10 || cy<-10 || cy>height+10); 
		};

	moveIt = std::remove_if(stracks.begin(), stracks.end(), isOutsideImage);
	outside_image.insert(outside_image.end(), std::make_move_iterator(moveIt), std::make_move_iterator(stracks.end()));
	stracks.erase(moveIt, stracks.end());

	auto sh_P = std::make_shared<PROJ_MATRIX>(P);
	auto sh_V = std::make_shared<TRANSFORMATION>(V);
	for (auto track: stracks){
		track->setViewProjection(sh_V, sh_P);
	}
}

void STrack::setViewProjection(std::shared_ptr<TRANSFORMATION> V, std::shared_ptr<PROJ_MATRIX> P){
	kalman_filter.P = P;
	kalman_filter.V = V;
}

bb_interfaces::msg::STrack STrack::toMessage(){
	bb_interfaces::msg::STrack message;
	message.is_activated = 	is_activated;
	message.track_id =		track_id;
	message.state =			trackStateToString[state];
	message.mean = 			vector<float>(state_current.mean.data(), state_current.mean.data() + state_current.mean.rows() * state_current.mean.cols());
	message.covariance = 	vector<float>(state_current.covariance.data(), state_current.covariance.data() + state_current.covariance.rows() * state_current.covariance.cols());
	message.class_name = 	classLabelString[(int)state_current.label];
	message.score = 		state_current.confidence;
	return message;
}
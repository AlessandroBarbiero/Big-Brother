#include <bb_tracker/STrack.h>

// #define ACCEPT_OLD_DETECTION // Add this define to accept older detections

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
	this->score = obj->prob;
	this->class_label = class_label;
	start_frame = 0;
	last_filter_update_ms = obj->time_ms;

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
	this->score = score;
	this->class_label = class_label;
	start_frame = 0;
	last_filter_update_ms = time_ms;

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
	this->mean = mc.first;
	this->covariance = mc.second;
	this->mean_predicted = this->mean;
	this->covariance_predicted = this->covariance;
	theta = mean[2];

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

	// Remove bacause not used by initiate, pass as parameters
	// auto sh_P = std::make_shared<PROJ_MATRIX>(P);
	// auto sh_V = std::make_shared<TRANSFORMATION>(V);
	// setViewProjection(sh_V, sh_P);

	auto mc = this->kalman_filter.initiate2D(xyabt_box, class_label, V, P);
	this->mean = mc.first;
	this->covariance = mc.second;
	this->mean_predicted = this->mean;
	this->covariance_predicted = this->covariance;
	theta = mean[2];

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

	auto mc = this->kalman_filter.update2D(this->mean_predicted, this->covariance_predicted, xyabt_box);

	auto current_time_ms = new_track.time_ms;
	updateTrackState(mc, current_time_ms, new_track.prob, new_track.label, frame_id, true);

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

	auto mc = this->kalman_filter.update3D(this->mean_predicted, this->covariance_predicted, xy_yaw_aah_box);

	auto current_time_ms = new_track.last_filter_update_ms;
	updateTrackState(mc, current_time_ms, new_track.score, new_track.class_label, frame_id, true);

	if (new_id)
		this->track_id = next_id();
}

bool STrack::checkOldDetection(int64_t detection_time_ms){
	if(this->last_filter_update_ms > detection_time_ms){
		// Saved state is more updated than detection, just delete the projection
		this->mean_predicted = this->mean;
		this->covariance_predicted = this->covariance;
		static_minwdh();
		static_minmax();
		return true;
	}
	return false;
}

void STrack::updateClassLabel(ClassLabel new_label, float new_score){
	// this is the maximum value in meters that the dimensions can differ from prior
	static float margin = 1.0; 

	if(this->class_label == new_label){
		// The new score is the average between the old*2 and new
		this->score = (this->score*2 + new_score)/3;
		auto priorDim = priorDimensions.at(this->class_label);
		// Respect the priors
		if(		this->mean(5) > (priorDim[2]+margin)  ||
				this->mean(3) > (priorDim[0]+margin)/priorDim[2]  ||
				this->mean(4) > (priorDim[1]+margin)/priorDim[2])
		{
			this->mean(5) = priorDim[2];
			this->mean(3) = priorDim[0]/priorDim[2];
			this->mean(4) = priorDim[1]/priorDim[2];
		}
	}
	else{
		// The score is decreased and new priors are applied if object change class
		this->score = (this->score*2 - new_score)/3;
		if(this->score < 0.5){
			this->class_label = new_label;
			// Apply new priors
			auto priorDim = priorDimensions.at(this->class_label);
			this->mean(3) = priorDim[0]/priorDim[2]; //l_ratio
			this->mean(4) = priorDim[1]/priorDim[2]; //d_ratio
			this->mean(5) = priorDim[2];			 //h
		}
	}
}

void STrack::updateTrackState(KAL_DATA& updated_values, int64_t detection_time_ms, float new_score, ClassLabel new_label, int frame_id, bool reset_tracklet_len){

	if(reset_tracklet_len)
		this->tracklet_len = 0;
	else
		this->tracklet_len++;
	this->frame_id = frame_id;

	last_filter_update_ms = detection_time_ms;
	this->mean = updated_values.first;
	updateClassLabel(new_label, new_score);
	this->covariance = updated_values.second;
	this->mean_predicted = this->mean;
	this->covariance_predicted = this->covariance;
	theta = this->mean[2];

	static_minwdh();
	static_minmax();

	this->state = TrackState::Tracked;
	this->is_activated = true;
}

void STrack::update(Object2D &new_track, int frame_id)
{
	#ifndef ACCEPT_OLD_DETECTION
	if(checkOldDetection(new_track.time_ms))
		return; // Saved state is more updated than detection, do not update
	#endif

	// Handle detection and update
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

	double yaw = 0;
	Eigen::Vector3f second_point;
	// If the object is new compute the orientation projecting the new position and computing the vector from the old to the new position
	if(!this->is_activated){
		second_point = projectPoint2D({xyabt_box[0], xyabt_box[1]}, mean(5)/2.0, *this->kalman_filter.V, *this->kalman_filter.P);
		double dx = second_point(0) - mean(0);
		double dy = second_point(1) - mean(1);

		// Calculate the yaw angle using atan2
		yaw = std::atan2(dy, dx);
		this->mean_predicted(2) = yaw;
	}

	auto mc = this->kalman_filter.update2D(this->mean_predicted, this->covariance_predicted, xyabt_box);

	if(!this->is_activated){
		mc.first(2) = yaw;
		mc.first(0) = second_point(0);
		mc.first(1) = second_point(1);
	}

	// TODO: possibility to remove time update
	updateTrackState(mc, new_track.time_ms, new_track.prob, new_track.label, frame_id);
	//updateTrackState(mc, this->last_filter_update_ms, new_track.prob, new_track.label, frame_id);
}

void STrack::update(STrack &new_track, int frame_id)
{
	auto current_time_ms = new_track.last_filter_update_ms;

	#ifndef ACCEPT_OLD_DETECTION
	if(checkOldDetection(current_time_ms))
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

	auto mc = this->kalman_filter.update3D(this->mean_predicted, this->covariance_predicted, xy_yaw_aah_box);

	auto new_score = new_track.score;
	updateTrackState(mc, current_time_ms, new_score, new_track.class_label, frame_id);
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

	minwdh[0] = mean[0];	// x center
	minwdh[1] = mean[1];	// y center
	minwdh[2] = 0.0;		// z min
	minwdh[3] = mean[3];	// w/h
	minwdh[4] = mean[4];	// d/h
	minwdh[5] = mean[5];	// h 


	minwdh[3] *= minwdh[5];			// w
	minwdh[4] *= minwdh[5];			// d
	minwdh[0] -= minwdh[3] / 2;		// x min
	minwdh[1] -= minwdh[4] / 2;		// y min

}

void STrack::static_minwdh_predicted()
{
	minwdh[0] = mean_predicted[0];	// x center
	minwdh[1] = mean_predicted[1];	// y center
	minwdh[2] = 0.0;				// z min
	minwdh[3] = mean_predicted[3];	// w/h
	minwdh[4] = mean_predicted[4];	// d/h
	minwdh[5] = mean_predicted[5];	// h 

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

void STrack::multi_predict(vector<STrack*> &stracks, byte_kalman::EKF &kalman_filter, int64_t current_time_ms)
{
	//cout << "Multi predict" << endl;
	for (unsigned int i = 0; i < stracks.size(); i++)
	{
		// Predict objects also in the past
		double dt = static_cast<double>(current_time_ms - stracks[i]->last_filter_update_ms)/MILLIS_IN_SECONDS;
		//cout << "id: "<< stracks[i]->track_id << " - delta t = " << dt << endl;
		
		// if(dt<=0){	// Predict new value only if the current time is after the last update
		// 	cout << "try to predict back in time for track: " << stracks[i]->track_id << "\nOld time: " << stracks[i]->last_filter_update_ms << " | New time: " << current_time_ms << endl;
		// 	continue;
		// }

		stracks[i]->mean_predicted = stracks[i]->mean;
		stracks[i]->covariance_predicted = stracks[i]->covariance;

		kalman_filter.predict(stracks[i]->mean_predicted, stracks[i]->covariance_predicted, dt);

		stracks[i]->static_minwdh_predicted();
		stracks[i]->static_minmax();
	}
}



// Project the predicted mean into the image, separate the objects that are outside the image in another vector
void STrack::multi_project(vector<STrack*> &stracks, vector<STrack*> &outside_image, PROJ_MATRIX& P, TRANSFORMATION& V, uint32_t width, uint32_t height){
	// 1. filter out objects behind the camera
	auto isBehindCamera = [&](STrack* x){ return objectBehindCamera(x->mean_predicted[0], x->mean_predicted[1], x->mean_predicted[5]/2.0, V); };
	auto moveIt = std::remove_if(stracks.begin(), stracks.end(), isBehindCamera);
	outside_image.insert(outside_image.end(), std::make_move_iterator(moveIt), std::make_move_iterator(stracks.end()));
	stracks.erase(moveIt, stracks.end());

	// 2. project the ellipsoid
	for (unsigned int i = 0; i < stracks.size(); i++){
		ELLIPSE_STATE e_state = ellipseFromEllipsoidv2(stracks[i]->mean_predicted, V, P);
		// Define an Axis Aligned Bounding Box from the ellipse (it is used for the IoU) 
		stracks[i]->vis2D_tlbr = tlbrFromEllipse(e_state);
	}

	// TODO: choose what to filter out
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
	message.mean = 			vector<float>(mean.data(), mean.data() + mean.rows() * mean.cols());
	message.covariance = 	vector<float>(covariance.data(), covariance.data() + covariance.rows() * covariance.cols());
	message.class_name = 	classLabelString[(int)class_label];
	message.score = 		score;
	return message;
}
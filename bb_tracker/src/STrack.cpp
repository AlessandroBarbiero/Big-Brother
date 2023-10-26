#include <bb_tracker/STrack.h>


STrack::STrack(vector<float> minwdh_, float score, std::string class_name, int64_t time_ms)
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
	this->class_name = class_name;
	start_frame = 0;
	last_filter_update_ms = time_ms;
}

STrack::~STrack()
{
	// std::cout<<"track deleted, id: "<< track_id << " size: " << path_marker.points.size() << std::endl;
}

void STrack::activate(byte_kalman::EKF &kalman_filter, int frame_id)
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

	DETECTBOX3D xyaah_box;
	xyaah_box[0] = xyzaah[0];
	xyaah_box[1] = xyzaah[1];
	xyaah_box[2] = xyzaah[3];
	xyaah_box[3] = xyzaah[4];
	xyaah_box[4] = xyzaah[5];
	auto mc = this->kalman_filter.initiate3D(xyaah_box);
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
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

// TODO: implement
void STrack::re_activate(Object2D &new_track, int frame_id, bool new_id)
{
	vector<float> tlbr = new_track.tlbr;

	DETECTBOX2D xyabt_box;
	xyabt_box[0] = (tlbr[0]+tlbr[2])/2.0;	// x
	xyabt_box[1] = (tlbr[1]+tlbr[3])/2.0;	// y
	xyabt_box[2] = (tlbr[2]-tlbr[0])/2.0;	// a
	xyabt_box[3] = (tlbr[3]-tlbr[1])/2.0;	// b
	xyabt_box[4] = 0;						// theta

	auto mc = this->kalman_filter.update2D(this->mean_predicted, this->covariance_predicted, xyabt_box);

	auto current_time_ms = new_track.time_ms;
	updateTrackState(mc, current_time_ms, new_track.prob, frame_id, true);

	if (new_id)
		this->track_id = next_id();
}

void STrack::re_activate(STrack &new_track, int frame_id, bool new_id)
{
	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);

	DETECTBOX3D xyaah_box;
	xyaah_box[0] = xyzaah[0];
	xyaah_box[1] = xyzaah[1];
	xyaah_box[2] = xyzaah[3];
	xyaah_box[3] = xyzaah[4];
	xyaah_box[4] = xyzaah[5];

	auto mc = this->kalman_filter.update3D(this->mean_predicted, this->covariance_predicted, xyaah_box);

	auto current_time_ms = new_track.last_filter_update_ms;
	updateTrackState(mc, current_time_ms, new_track.score, frame_id, true);

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

void STrack::updateTrackState(KAL_DATA& updated_values, int64_t detection_time_ms, float new_score, int frame_id, bool reset_tracklet_len){
	
	if(reset_tracklet_len)
		this->tracklet_len = 0;
	else
		this->tracklet_len++;
	this->frame_id = frame_id;

	last_filter_update_ms = detection_time_ms;
	this->mean = updated_values.first;
	this->covariance = updated_values.second;
	this->mean_predicted = this->mean;
	this->covariance_predicted = this->covariance;
	theta = this->mean[2];

	static_minwdh();
	static_minmax();

	this->state = TrackState::Tracked;
	this->is_activated = true;

	// The new score is the average between the old*2 and new
	// TODO: this can be improved
	this->score = (this->score*2 + new_score)/3;
}

// TODO: implement
void STrack::update(Object2D &new_track, int frame_id)
{
	auto current_time_ms = new_track.time_ms;

	if(checkOldDetection(current_time_ms))
		return; // Saved state is more updated than detection, do not update

	// Handle detection and update
	vector<float> tlbr = new_track.tlbr;

	DETECTBOX2D xyabt_box;
	xyabt_box[0] = (tlbr[0]+tlbr[2])/2.0;	// x
	xyabt_box[1] = (tlbr[1]+tlbr[3])/2.0;	// y
	xyabt_box[2] = (tlbr[2]-tlbr[0])/2.0;	// a
	xyabt_box[3] = (tlbr[3]-tlbr[1])/2.0;	// b
	xyabt_box[4] = 0;						// theta

	auto mc = this->kalman_filter.update2D(this->mean_predicted, this->covariance_predicted, xyabt_box);

	auto new_score = new_track.prob;
	updateTrackState(mc, current_time_ms, new_score, frame_id);
}

void STrack::update(STrack &new_track, int frame_id)
{
	auto current_time_ms = new_track.last_filter_update_ms;

	if(checkOldDetection(current_time_ms))
		return; // Saved state is more updated than detection, do not update

	// Handle detection and update
	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);

	DETECTBOX3D xyaah_box;
	xyaah_box[0] = xyzaah[0];	// x
	xyaah_box[1] = xyzaah[1];	// y
	xyaah_box[2] = xyzaah[3];	// w/h
	xyaah_box[3] = xyzaah[4];	// d/h
	xyaah_box[4] = xyzaah[5];	// h

	auto mc = this->kalman_filter.update3D(this->mean_predicted, this->covariance_predicted, xyaah_box);

	auto new_score = new_track.score;
	updateTrackState(mc, current_time_ms, new_score, frame_id);
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
	for (unsigned int i = 0; i < stracks.size(); i++)
	{
		double dt = (static_cast<long long>(current_time_ms) - static_cast<long long>(stracks[i]->last_filter_update_ms))/MILLIS_IN_SECONDS;
		
		// Predict objects also in the past to exclude them from the association
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

	// 3. filter out objects outside image
	auto isOutsideImage = [&](STrack* x){ return !boxInImage(x->vis2D_tlbr[0], x->vis2D_tlbr[1], x->vis2D_tlbr[2], x->vis2D_tlbr[3], width, height); };
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
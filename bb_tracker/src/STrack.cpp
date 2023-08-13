#include <bb_tracker/STrack.h>

STrack::STrack(vector<float> minwdh_, float score, std::string class_name, long unsigned int time_ms)
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
	auto mc = this->kalman_filter.initiate(xyaah_box);
	this->mean = mc.first;
	this->covariance = mc.second;
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

void STrack::re_activate(STrack &new_track, int frame_id, bool new_id)
{
	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);
	auto current_time_ms = new_track.last_filter_update_ms;

	DETECTBOX3D xyaah_box;
	xyaah_box[0] = xyzaah[0];
	xyaah_box[1] = xyzaah[1];
	xyaah_box[2] = xyzaah[3];
	xyaah_box[3] = xyzaah[4];
	xyaah_box[4] = xyzaah[5];

	// TODO add dt
	double dt = (current_time_ms - last_filter_update_ms)/1000;
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyaah_box, dt);
	last_filter_update_ms = current_time_ms;
	this->mean = mc.first;
	this->covariance = mc.second;
	theta = mean[2];

	static_minwdh();
	static_minmax();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id();
}

void STrack::update(STrack &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;
	auto current_time_ms = new_track.last_filter_update_ms;

	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);

	DETECTBOX3D xyaah_box;
	xyaah_box[0] = xyzaah[0];
	xyaah_box[1] = xyzaah[1];
	xyaah_box[2] = xyzaah[3];
	xyaah_box[3] = xyzaah[4];
	xyaah_box[4] = xyzaah[5];

	// TODO add dt
	double dt = (current_time_ms - last_filter_update_ms)/1000;
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyaah_box, dt);
	last_filter_update_ms = current_time_ms;
	this->mean = mc.first;
	this->covariance = mc.second;
	theta = mean[2];

	static_minwdh();
	static_minmax();

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
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

	minwdh[0] = mean[0];
	minwdh[1] = mean[1];
	minwdh[2] = 0.0;
	minwdh[3] = mean[3];
	minwdh[4] = mean[4];
	minwdh[5] = mean[5];


	minwdh[3] *= minwdh[5];
	minwdh[4] *= minwdh[5];
	minwdh[0] -= minwdh[3] / 2;
	minwdh[1] -= minwdh[4] / 2;

}

void STrack::static_minmax()
{
	minmax.clear();
	minmax.assign(minwdh.begin(), minwdh.end());
	// 2D
	// tlbr[2] += tlbr[0];
	// tlbr[3] += tlbr[1];

	minmax[3] += minmax[0];
	minmax[4] += minmax[1];
	minmax[5] += minmax[2];
}

vector<float> STrack::minwdh_to_xyzaah(vector<float> minwdh_tmp)
{
	vector<float> minwdh_output = minwdh_tmp;
	minwdh_output[0] += minwdh_output[3] / 2;
	minwdh_output[1] += minwdh_output[4] / 2;
	minwdh_output[2] += minwdh_output[5] / 2;
	minwdh_output[3] /= minwdh_output[5];
	minwdh_output[4] /= minwdh_output[5];
	return minwdh_output;
}

vector<float> STrack::to_xyzaah()
{
	return minwdh_to_xyzaah(minwdh);
}

vector<float> STrack::minmax_to_minwdh(vector<float> &minmax)
{
	minmax[3] -= minmax[0];
	minmax[4] -= minmax[1];
	minmax[5] -= minmax[2];
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

void STrack::multi_predict(vector<STrack*> &stracks, byte_kalman::EKF &kalman_filter, unsigned long int current_time_ms)
{
	for (unsigned int i = 0; i < stracks.size(); i++)
	{
		// TODO add dt
		double dt = (current_time_ms - stracks[i]->last_filter_update_ms)/1000;
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance, dt);
		// stracks[i]->last_filter_update_ms = current_time_ms;
		stracks[i]->static_minwdh();
		stracks[i]->static_minmax();
	}
}
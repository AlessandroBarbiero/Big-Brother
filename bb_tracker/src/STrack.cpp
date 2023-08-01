#include <bb_tracker/STrack.h>

STrack::STrack(vector<float> minwdh_, float score)
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
	start_frame = 0;
}

STrack::~STrack()
{
}

void STrack::activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id)
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
	//TODO: change after DETECTBOX
	DETECTBOX xyah_box;
	xyah_box[0] = xyzaah[0];
	xyah_box[1] = xyzaah[1];
	xyah_box[2] = xyzaah[2];
	xyah_box[3] = xyzaah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

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
	vector<float> xyah = minwdh_to_xyzaah(new_track.minwdh);
	// TODO: change after DETECTBOX
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

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

	vector<float> xyzaah = minwdh_to_xyzaah(new_track.minwdh);
	// TODO: change after DETECTBOX
	DETECTBOX xyah_box;
	xyah_box[0] = xyzaah[0];
	xyah_box[1] = xyzaah[1];
	xyah_box[2] = xyzaah[2];
	xyah_box[3] = xyzaah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

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
	minwdh[2] = mean[2];
	minwdh[3] = mean[3];
	minwdh[4] = mean[4];
	minwdh[5] = mean[5];

	// 2D
	// tlwh[2] *= tlwh[3];
	// tlwh[0] -= tlwh[2] / 2;
	// tlwh[1] -= tlwh[3] / 2;

	minwdh[3] *= minwdh[5];
	minwdh[4] *= minwdh[5];
	minwdh[0] -= minwdh[3] / 2;
	minwdh[1] -= minwdh[4] / 2;
	minwdh[2] -= minwdh[5] / 2;

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

void STrack::multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			stracks[i]->mean[7] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_minwdh();
		stracks[i]->static_minmax();
	}
}
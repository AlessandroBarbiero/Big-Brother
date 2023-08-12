#pragma once

#include <opencv2/opencv.hpp>
#include "EKF.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
	STrack(vector<float> minwdh_, float score, std::string class_name);
	~STrack();

	vector<float> static minmax_to_minwdh(vector<float> &minmax);
	void static multi_predict(vector<STrack*> &stracks, byte_kalman::EKF &kalman_filter);
	void static_minwdh();
	void static_minmax();
	// xyzaah respresents the box independently of the dimensions, only the height is kept as it is
	// Returns a vector whose elements are {cx-cy-cz - width/height - depth/height - height}
	vector<float> minwdh_to_xyzaah(vector<float> minwdh_tmp);
	vector<float> to_xyzaah();
	void mark_lost();
	void mark_removed();
	int next_id();
	int end_frame();
	
	void activate(byte_kalman::EKF &kalman_filter, int frame_id);
	void re_activate(STrack &new_track, int frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);

public:
	bool is_activated;
	int track_id;
	int state;

	vector<float> _minwdh; 	// <--- ex _tlwh
	vector<float> minwdh;	// <--- ex tlwh
	vector<float> minmax;	// <--- ex tlbr
	float theta;
	
	int frame_id;
	int tracklet_len;
	int start_frame;
	visualization_msgs::msg::Marker path_marker;
	visualization_msgs::msg::Marker text_marker;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;
	std::string class_name;

private:
	byte_kalman::EKF kalman_filter;
};
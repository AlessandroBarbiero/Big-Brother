#pragma once

#include <opencv2/opencv.hpp>
#include "EKF.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "dataType.h"

#include "ellipsoid_ellipse.hpp"

using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
	STrack(vector<float> minwdh_, float score, std::string class_name, unsigned long int time_ms);
	~STrack();

	vector<float> static minmax_to_minwdh(vector<float> &minmax);
	void static multi_predict(vector<STrack*> &stracks, byte_kalman::EKF &kalman_filter, unsigned long int current_time_ms);
	void static multi_project(vector<STrack*> &stracks, PROJ_MATRIX& P, TRANSFORMATION& V, uint32_t width, uint32_t height);
	void static_minwdh();
	void static_minwdh_predicted();
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
	void re_activate(Object2D &new_track, int frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);
	void update(Object2D &new_track, int frame_id);

public:
	bool is_activated;
	int track_id;
	int state;

	// Used during the creation phase
	vector<float> _minwdh;

	// This value is used both for the last valid update and for the predicted value, 
	// if you want one of the two you can call before static_minwdh() or static_minwdh_predicted() functions
	vector<float> minwdh;
	// This value is used both for the last valid update and for the predicted value, 
	// It is always in line with minwdh,
	// if you want one of the two you can call static_minwdh() or static_minwdh_predicted() functions before static_minmax()
	vector<float> minmax;

	// TODO: Remember to use this
	vector<float> vis2D_tlbr; // Fill this value with a 2d representation in camera

	float theta;
	
	int frame_id;
	// milliseconds
	unsigned long int last_filter_update_ms;
	int tracklet_len;
	int start_frame;
	visualization_msgs::msg::Marker path_marker;
	visualization_msgs::msg::Marker text_marker;

	KAL_MEAN mean_predicted;
	KAL_COVA covariance_predicted;
	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;
	std::string class_name;

private:
	byte_kalman::EKF kalman_filter;
};
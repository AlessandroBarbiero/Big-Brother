#pragma once

#include <algorithm>
#include <memory>
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
	STrack(vector<float> minwdh_, float score, std::string class_name, int64_t time_ms);
	~STrack();

	vector<float> static minmax_to_minwdh(vector<float> &minmax);
	void static multi_predict(vector<STrack*> &stracks, byte_kalman::EKF &kalman_filter, int64_t current_time_ms);
	void static multi_project(vector<STrack*> &stracks, vector<STrack*> &outside_image, PROJ_MATRIX& P, TRANSFORMATION& V, uint32_t width, uint32_t height);
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
	void setViewProjection(std::shared_ptr<TRANSFORMATION> V, std::shared_ptr<PROJ_MATRIX> P);

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

	// 2d representation in camera of the 3d object as an axis aligned bounding box
	vector<float> vis2D_tlbr;

	float theta;
	
	int frame_id;
	// milliseconds
	int64_t last_filter_update_ms;
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

	// Return true if saved state is more updated than detection, in that case the projection is deleted
	bool checkOldDetection(int64_t detection_time_ms);

	// Update internal state of the STrack after a kalman filter update
	void updateTrackState(KAL_DATA& updated_values, int64_t detection_time_ms, float new_score, int frame_id, bool reset_tracklet_len = false);
};
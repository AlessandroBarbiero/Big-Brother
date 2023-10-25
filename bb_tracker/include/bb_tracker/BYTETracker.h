#pragma once

#include "STrack.h"
#include "EKF.hpp"
#include <Eigen/Geometry>
#include <map>
#include <vector>
#include "dataType.h"

using namespace std;

class BYTETracker
{
public:
	BYTETracker();
	~BYTETracker();

	/**
	 * Initializes the BYTETracker with specified parameters.
	 *
	 * @param time_to_lost Milliseconds a tracked object is not seen to declare it lost.
	 * @param unconfirmed_ttl Milliseconds an unconfirmed object is not seen before removing it.
	 * @param lost_ttl Number of milliseconds a track is kept if it is not seen in the scene.
	 * @param track_thresh This threshold is used to divide initially the detections into high and low score,
	 *                     where high score detections are considered first for matching.
	 * @param high_thresh This threshold is used to determine whether a high score detected object should be considered
	 *                    as a new object if it does not match with previously considered tracklets.
	 * @param match_thresh This threshold is used during the association step to establish the correspondence between
	 *                     existing tracks and newly detected objects.
	 * @param fixed_frame_desc The fixed frame the BYTETracker has to use; all the detections have to give a transform
	 *                          for this frame.
	 */
	void init(u_int time_to_lost = 300, u_int unconfirmed_ttl = 300, u_int lost_ttl = 1000, float track_thresh = 0.5, float high_thresh = 0.6, float match_thresh = 0.8);

	vector<STrack*> update(const vector<Object3D>& objects);

	/**
	 * Updates the BYTETracker with data coming from a detection 2D.
	 *
	 * @param objects 2D detections (BBox, classification and time)
	 * @param P Projection matrix from an object in the camera frame to an object in the image.
	 * @param V View matrix that transform a point from the fixed frame (the one objects are saved respect to) to the camera frame
	 * @param width width in pixels of the image the decetions are taken from
	 * @param height height in pixels of the image the decetions are taken from
	 * @return The list of STracks currently seen by the tracker 
	 */
	vector<STrack*> update(const vector<Object2D>& objects, PROJ_MATRIX& P, TRANSFORMATION& V, uint32_t width, uint32_t height);
	Scalar get_color(int idx);
	vector<STrack> getTrackedObj();

	static std::unordered_map<std::string, int> class_to_int;
	static std::unordered_map<int, std::string> int_to_class;

	byte_kalman::EKF kalman_filter;

private:
	/**
	 * Combines two vectors of STrack objects into a single vector.
	 * Ensures uniqueness of elements in res based on 'track_id' field.
	 *
	 * @param tlista First vector of STrack pointer objects.
	 * @param tlistb Second vector of STrack objects.
	 * @return Combined vector of unique STrack pointer objects.
	 */
	vector<STrack*> joint_stracks(vector<STrack*> &tlista, vector<STrack> &tlistb);
	/**
	 * Combines two vectors of STrack objects into a single vector.
	 * Ensures uniqueness of elements in res based on 'track_id' field.
	 *
	 * @param tlista First vector of STrack objects.
	 * @param tlistb Second vector of STrack objects.
	 * @return Combined vector of unique STrack objects.
	 */
	vector<STrack> joint_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);

	vector<STrack> sub_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);
	/**
	 * Removes duplicate elements from two sets of STrack vectors, 'stracksa' and 'stracksb'.
	 * Duplicate elements are identified by comparing the intersection-over-union (IOU) distance between elements in 'stracksa' and 'stracksb'.
	 * Elements that have an IOU distance less than 0.15 are considered duplicates.
	 * The non-duplicate elements are added to 'resa' and 'resb'.
	 * If an element is considered duplicated, keep only the one that has been tracked for the most time
	 *
	 * @param resa      The output vector where non-duplicate elements from 'stracksa' will be stored.
	 * @param resb      The output vector where non-duplicate elements from 'stracksb' will be stored.
	 * @param stracksa  The input vector of STrack elements from set A.
	 * @param stracksb  The input vector of STrack elements from set B.
	 */
	void remove_duplicate_stracks(vector<STrack> &resa, vector<STrack> &resb, vector<STrack> &stracksa, vector<STrack> &stracksb);

	/**
	 * Performs linear assignment on the given cost matrix using the Jonker-Volgenant algorithm.
	 *
	 * @param cost_matrix A 2D vector representing the cost matrix for linear assignment.
	 * @param cost_matrix_size The number of rows in the cost matrix.
	 * @param cost_matrix_size_size The number of columns in the cost matrix.
	 * @param thresh The threshold value used in the Jonker-Volgenant algorithm.
	 * @param matches A 2D vector that will store the matched indices as pairs [row, col]. matches[0] = row, matches[1] = col
	 * @param unmatched_a A vector that will store the indices of unmatched rows.
	 * @param unmatched_b A vector that will store the indices of unmatched columns.
	 *
	 * After the linear assignment process, the matches vector will contain pairs of indices representing matched rows and columns,
	 * and the unmatched_a and unmatched_b vectors will contain indices of rows and columns respectively that were not matched.
	 */
	void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
	vector<vector<float> > iou_distance(vector<STrack*> &atracks, vector<STrack> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > iou_distance(vector<STrack> &atracks, vector<STrack> &btracks);
	vector<vector<float> > iou_distance2d(vector<STrack*> &atracks, vector<Object2D> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > iou_distance2d(vector<STrack> &atracks, vector<Object2D> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &aminmaxs, vector<vector<float> > &bminmaxs);
	vector<vector<float> > ious_2d(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);

	double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

	void predict_at_current_time(vector<STrack*>& output_stracks, int64_t detection_time_ms);

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	u_int time_to_lost, lost_ttl, unconfirmed_ttl;
	// The current time is updated every time a detection come
	// if a detection time is after the predicted current time, that become the current time
	int64_t current_time_ms;
	std::chrono::time_point<std::chrono::system_clock> last_update_time;

	vector<STrack> tracked_stracks;
	vector<STrack> lost_stracks;
	vector<STrack> removed_stracks;

};
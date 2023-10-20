#include <bb_tracker/BYTETracker.h>

vector<STrack*> BYTETracker::update(const vector<Object2D>& objects)
{
	// TODO: implement this
	////////////////// Step 1: Get detections //////////////////
	this->frame_id++;

	vector<STrack> activated_stracks;
	vector<STrack> refind_stracks;
	vector<STrack> removed_stracks;
	vector<STrack> lost_stracks;

	vector<STrack> tracked_stracks_swap;
	vector<STrack> resa, resb;
	vector<STrack*> r_tracked_stracks;

	// Object2D
	vector<Object2D> detections;
	vector<Object2D> detections_low;
	vector<Object2D> detections_cp;

	// STrack
	vector<STrack*> unconfirmed;
	vector<STrack*> tracked_stracks;
	vector<STrack*> strack_pool;

	vector<STrack*> output_stracks;


	if (objects.size() > 0)
	{
		for (unsigned int i = 0; i < objects.size(); i++)
		{
			if (objects[i].prob >= track_thresh)
				detections.push_back(objects[i]);
			else
				detections_low.push_back(objects[i]);
		}
	}

	// Add newly detected tracklets to tracked_stracks
	for (unsigned int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (!this->tracked_stracks[i].is_activated)
			unconfirmed.push_back(&this->tracked_stracks[i]);
		else
			tracked_stracks.push_back(&this->tracked_stracks[i]);
	}

	//std::cout << "Step 2" << std::endl;
	////////////////// Step 2: First association, with IoU //////////////////
	strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);
	// Project everything with the last time of detection and then do association
	long unsigned int last_det_time_ms = objects.back().time_ms;
	STrack::multi_predict(strack_pool, this->kalman_filter, last_det_time_ms);

	vector<vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	dists = iou_distance2d(strack_pool, detections, dist_size, dist_size_size);

	vector<vector<int> > matches;
	vector<int> u_track, u_detection;
	linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

	for (unsigned int i = 0; i < matches.size(); i++)
	{
		STrack *track = strack_pool[matches[i][0]];
		Object2D *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	//std::cout << "Step 3" << std::endl;
	////////////////// Step 3: Second association, using low score dets //////////////////
	for (unsigned int i = 0; i < u_detection.size(); i++)
	{
		detections_cp.push_back(detections[u_detection[i]]);
	}
	detections.clear();
	detections.assign(detections_low.begin(), detections_low.end());
	
	for (unsigned int i = 0; i < u_track.size(); i++)
	{
		if (strack_pool[u_track[i]]->state == TrackState::Tracked)
		{
			r_tracked_stracks.push_back(strack_pool[u_track[i]]);
		}
	}

	dists.clear();
	dists = iou_distance2d(r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	for (unsigned int i = 0; i < matches.size(); i++)
	{
		STrack *track = r_tracked_stracks[matches[i][0]];
		Object2D *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id, false);
			refind_stracks.push_back(*track);
			std::cout<<"ByteTracker: it should not enter here. Check this. Line: "<< __LINE__ << std::endl;
		}
	}

	for (unsigned int i = 0; i < u_track.size(); i++)
	{
		STrack *track = r_tracked_stracks[u_track[i]];
		if (last_det_time_ms - track->last_filter_update_ms > time_to_lost)
		{
			track->mark_lost();
			lost_stracks.push_back(*track);
		}
		else{
			// Add object to the activated tracks without any update, the last time seen is conserved
			activated_stracks.push_back(*track);
		}
	}

	// Deal with unconfirmed tracks, usually tracks with only one beginning frame
	detections.clear();
	detections.assign(detections_cp.begin(), detections_cp.end());

	dists.clear();
	dists = iou_distance2d(unconfirmed, detections, dist_size, dist_size_size);

	matches.clear();
	vector<int> u_unconfirmed;
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

	for (unsigned int i = 0; i < matches.size(); i++)
	{
		unconfirmed[matches[i][0]]->update(detections[matches[i][1]], this->frame_id);
		activated_stracks.push_back(*unconfirmed[matches[i][0]]);
	}

	for (unsigned int i = 0; i < u_unconfirmed.size(); i++)
	{
		STrack *track = unconfirmed[u_unconfirmed[i]];
		if(last_det_time_ms - track->last_filter_update_ms > unconfirmed_ttl){
			track->mark_removed();
			removed_stracks.push_back(*track);
		}
		else{
			// Add object to the activated tracks without any update, the last time seen is conserved
			activated_stracks.push_back(*track);
		}
	}

	//std::cout << "Step 4" << std::endl;
	////////////////// Step 4: Init new stracks //////////////////
	// TODO: look if activate a track from 2d detection
	// for (unsigned int i = 0; i < u_detection.size(); i++)
	// {
	// 	Object2D *track = &detections[u_detection[i]];
	// 	if (track->score < this->high_thresh)
	// 		continue;
	// 	track->activate(this->kalman_filter, this->frame_id);
	// 	activated_stracks.push_back(*track);
	// }

	//std::cout << "Step 5" << std::endl;
	////////////////// Step 5: Update state //////////////////
	for (unsigned int i = 0; i < this->lost_stracks.size(); i++)
	{
		// if (this->frame_id - this->lost_stracks[i].end_frame() > this->track_buffer)  [OLD way with frame_id]
		if(last_det_time_ms - this->lost_stracks[i].last_filter_update_ms > lost_ttl)
		{
			this->lost_stracks[i].mark_removed();
			removed_stracks.push_back(this->lost_stracks[i]);
		}
	}
	
	for (unsigned int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (this->tracked_stracks[i].state == TrackState::Tracked)
		{
			tracked_stracks_swap.push_back(this->tracked_stracks[i]);
		}
	}
	this->tracked_stracks.clear();
	this->tracked_stracks.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

	this->tracked_stracks = joint_stracks(this->tracked_stracks, activated_stracks);
	this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);

	this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);
	for (unsigned int i = 0; i < lost_stracks.size(); i++)
	{
		this->lost_stracks.push_back(lost_stracks[i]);
	}

	this->lost_stracks = sub_stracks(this->lost_stracks, this->removed_stracks);
	for (unsigned int i = 0; i < removed_stracks.size(); i++)
	{
		this->removed_stracks.push_back(removed_stracks[i]);
	}
	
	remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks);

	this->tracked_stracks.clear();
	this->tracked_stracks.assign(resa.begin(), resa.end());
	this->lost_stracks.clear();
	this->lost_stracks.assign(resb.begin(), resb.end());

	//std::cout << "End step 5" << std::endl;
	
	for (unsigned int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (this->tracked_stracks[i].is_activated)
		{
			output_stracks.push_back(&this->tracked_stracks[i]);
		}
	}
	
	return output_stracks;
}
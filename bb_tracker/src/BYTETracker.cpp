#include <bb_tracker/BYTETracker.h>
#include <fstream>

BYTETracker::BYTETracker()
{
}

BYTETracker::~BYTETracker()
{
}

void BYTETracker::init(u_int time_to_lost, u_int unconfirmed_ttl, u_int lost_ttl, float track_thresh, float high_thresh, float match_thresh){
	this->track_thresh = track_thresh; //0.5;
	this->high_thresh  = high_thresh;  //0.6;
	this->match_thresh = match_thresh; //0.8;
	this->time_to_lost = time_to_lost;
	this->unconfirmed_ttl = unconfirmed_ttl;
	this->lost_ttl = lost_ttl;

	frame_id = 0;

	std::cout << "Init ByteTrack!"<< std::endl;
	std::cout << "Parameters: " 
	<< "\n\ttime_to_lost = " << time_to_lost
	<< "\n\tunconfirmed_ttl = " << unconfirmed_ttl
	<< "\n\tlost_ttl = " << lost_ttl
	<< "\n\ttrack_thresh = " << track_thresh
	<< "\n\thigh_thresh = " << high_thresh
	<< "\n\tmatch_thresh = " << match_thresh
	<< std::endl;
}

std::unordered_map<std::string, int> BYTETracker::class_to_int{
	{"person", 0},
	{"pedestrian", 0},
	{"vehicle", 1},
	{"car", 1},
	{"truck", 1},
	{"motorcycle", 1},
	{"bicycle", 2}
};

std::unordered_map<int, std::string> BYTETracker::int_to_class{
	{0, "person"},
	{1, "vehicle"},
	{2, "bicycle"}
};

vector<STrack*> BYTETracker::update(const vector<Object>& objects)
{
	//std::cout << "Step 1" << std::endl;
	////////////////// Step 1: Get detections //////////////////
	this->frame_id++;
	vector<STrack> activated_stracks;
	vector<STrack> refind_stracks;
	vector<STrack> removed_stracks;
	vector<STrack> lost_stracks;
	vector<STrack> detections;
	vector<STrack> detections_low;

	vector<STrack> detections_cp;
	vector<STrack> tracked_stracks_swap;
	vector<STrack> resa, resb;
	
	vector<STrack*> output_stracks;

	vector<STrack*> unconfirmed;
	vector<STrack*> tracked_stracks;
	vector<STrack*> strack_pool;
	vector<STrack*> r_tracked_stracks;

	// Project everything with the last time of detection and then do association
	long unsigned int last_det_time_ms = objects.back().time_ms;

	if (objects.size() > 0)
	{
		for (unsigned int i = 0; i < objects.size(); i++)
		{
			auto object = objects[i];
			vector<float> minmax_;
			minmax_.resize(6);
			minmax_[0] = object.box.center.position.x - object.box.size.x/2;
			minmax_[1] = object.box.center.position.y - object.box.size.y/2;
			minmax_[2] = object.box.center.position.z - object.box.size.z/2;
			minmax_[3] = object.box.center.position.x + object.box.size.x/2;
			minmax_[4] = object.box.center.position.y + object.box.size.y/2;
			minmax_[5] = object.box.center.position.z + object.box.size.z/2;

			float score = object.prob;
			std::string class_name = int_to_class[object.label];
			int time_ms = object.time_ms;

			STrack strack(STrack::minmax_to_minwdh(minmax_), score, class_name, time_ms);
			if (score >= track_thresh)
			{
				detections.push_back(strack);
			}
			else
			{
				detections_low.push_back(strack);
			}
			
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
	STrack::multi_predict(strack_pool, this->kalman_filter, last_det_time_ms);

	vector<vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

	vector<vector<int> > matches;
	vector<int> u_track, u_detection;
	linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

	for (unsigned int i = 0; i < matches.size(); i++)
	{
		STrack *track = strack_pool[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
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
	dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	for (unsigned int i = 0; i < matches.size(); i++)
	{
		STrack *track = r_tracked_stracks[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
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
	dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

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
	for (unsigned int i = 0; i < u_detection.size(); i++)
	{
		STrack *track = &detections[u_detection[i]];
		if (track->score < this->high_thresh)
			continue;
		track->activate(this->kalman_filter, this->frame_id);
		activated_stracks.push_back(*track);
	}

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
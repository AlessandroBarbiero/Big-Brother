#include <bb_tracker/BYTETracker.h>
#include <fstream>

BYTETracker::BYTETracker()
{
}

BYTETracker::~BYTETracker()
{
}

vector<STrack> BYTETracker::getTrackedObj(){
	return this->tracked_stracks;
}

void BYTETracker::init(int time_to_lost, int unconfirmed_ttl, int lost_ttl, int max_dt_past, float track_thresh, float high_thresh, float match_thresh){
	this->track_thresh = track_thresh; //0.5;
	this->high_thresh  = high_thresh;  //0.6;
	this->match_thresh = match_thresh; //0.8;
	this->time_to_lost = time_to_lost;
	this->unconfirmed_ttl = unconfirmed_ttl;
	this->lost_ttl = lost_ttl;
	this->max_dt_past = max_dt_past;

	frame_id = 0;
	current_time_ms = 0;
	last_update_time = chrono::system_clock::now();

	std::cout << "Init ByteTrack!"<< std::endl;
	std::cout << "Parameters: " 
	<< "\n\tmax_dt_past = " << max_dt_past
	<< "\n\ttime_to_lost = " << time_to_lost
	<< "\n\tunconfirmed_ttl = " << unconfirmed_ttl
	<< "\n\tlost_ttl = " << lost_ttl
	<< "\n\ttrack_thresh = " << track_thresh
	<< "\n\thigh_thresh = " << high_thresh
	<< "\n\tmatch_thresh = " << match_thresh
	<< std::endl;
}

std::unordered_map<std::string, ClassLabel> BYTETracker::class_to_label{
	{"", ClassLabel::Unknown},
	{"unknown", ClassLabel::Unknown},
	{"other", ClassLabel::Unknown},
	{"person", ClassLabel::Pedestrian},
	{"pedestrian", ClassLabel::Pedestrian},
	{"bicycle", ClassLabel::Bicycle},
	{"cyclist", ClassLabel::Bicycle},
	{"motorcycle", ClassLabel::Motorcycle},
	{"vehicle", ClassLabel::Car},
	{"car", ClassLabel::Car},
	{"truck", ClassLabel::Car}
};
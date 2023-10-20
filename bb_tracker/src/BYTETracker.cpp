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
	{"motorcycle", 3},
	{"bicycle", 2}
};

std::unordered_map<int, std::string> BYTETracker::int_to_class{
	{0, "Person"},
	{1, "Car"},
	{2, "Cyclist"},
	{3, "Motorcycle"}
};
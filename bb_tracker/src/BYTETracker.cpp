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

void BYTETracker::init(int time_to_lost, int unconfirmed_ttl, int lost_ttl, int max_dt_past, float track_thresh, float high_thresh, float match_thresh, bool left_handed_system){
	this->track_thresh = track_thresh; //0.5;
	this->high_thresh  = high_thresh;  //0.6;
	this->match_thresh = match_thresh; //0.8;
	this->time_to_lost = time_to_lost;
	this->unconfirmed_ttl = unconfirmed_ttl;
	this->lost_ttl = lost_ttl;
	this->max_dt_past = max_dt_past;
	this->left_handed_system = left_handed_system;
	constexpr size_t removed_to_keep = 1000;
	CircularBuffer<STrack> buff(removed_to_keep);
	this->removed_stracks = buff;

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
	<< "\n\tleft_handed_system = " << (left_handed_system?"true":"false")
	<< std::endl;
}

inline void copy_from_double_array(KAL_MEAN& eigen_v, std::vector<double>& c_vector){
	for(long int i = 0; i<eigen_v.size(); i++){
		eigen_v[i] = static_cast<float>(c_vector[i]);
	}
}
inline void copy_from_double_array(DETECTBOX3D& eigen_v, std::vector<double>& c_vector){
	for(long int i = 0; i<eigen_v.size(); i++){
		eigen_v[i] = static_cast<float>(c_vector[i]);
	}
}
inline void copy_from_double_array(DETECTBOX2D& eigen_v, std::vector<double>& c_vector){
	for(long int i = 0; i<eigen_v.size(); i++){
		eigen_v[i] = static_cast<float>(c_vector[i]);
	}
}

void BYTETracker::initVariance(std::vector<double>& v_mul_p03d, std::vector<double>& v_mul_p02d, std::vector<double>& v_mul_process_noise, std::vector<double>& v_mul_mn3d, std::vector<double>& v_mul_mn2d){
	KAL_MEAN mul_p03d, mul_p02d, mul_process_noise;
	DETECTBOX3D mul_mn3d;
	DETECTBOX2D mul_mn2d;
	copy_from_double_array(mul_p03d, v_mul_p03d);
	copy_from_double_array(mul_p02d, v_mul_p02d);
	copy_from_double_array(mul_process_noise, v_mul_process_noise);
	copy_from_double_array(mul_mn3d, v_mul_mn3d);
	copy_from_double_array(mul_mn2d, v_mul_mn2d);
	kalman_filter.initVariance(mul_p03d, mul_p02d, mul_process_noise, mul_mn3d, mul_mn2d);

	std::cout << "Variance Values: " 
	<< "\n\tP0 3D = " << mul_p03d
	<< "\n\tP0 2D = " << mul_p02d
	<< "\n\tProcess Noise = " << mul_process_noise
	<< "\n\tMeasurement Noise 3D = " << mul_mn3d
	<< "\n\tMeasurement Noise 2D = " << mul_mn2d
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
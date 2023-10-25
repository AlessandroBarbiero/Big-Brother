#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float, 1, 5, Eigen::RowMajor> DETECTBOX2D; // 2D xyab(theta)
typedef Eigen::Matrix<float, 1, 5, Eigen::RowMajor> DETECTBOX3D; // 3D xyaah
typedef Eigen::Matrix<float, Eigen::Dynamic, 5, Eigen::RowMajor> DETECTBOX3DSS;
typedef Eigen::Matrix<float, 1, 128, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, 128, Eigen::RowMajor> FEATURESS;
//typedef std::vector<FEATURE> FEATURESS;

//Kalmanfilter
typedef Eigen::Matrix<float, 1, 8, Eigen::RowMajor> KAL_MEAN;	// 3D {x,y,theta,l_ratio,d_ratio,h,  v,w}
typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 5, Eigen::RowMajor> KAL_HMEAN3D; // xyaah
typedef Eigen::Matrix<float, 5, 5, Eigen::RowMajor> KAL_HCOVA3D;
typedef Eigen::Matrix<float, 1, 5, Eigen::RowMajor> KAL_HMEAN2D; // xyab(theta) or [X_center, Y_center, semi_axis_A, semi_axis_B, theta (in radians)]
typedef Eigen::Matrix<float, 5, 5, Eigen::RowMajor> KAL_HCOVA2D;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA3D = std::pair<KAL_HMEAN3D, KAL_HCOVA3D>;
using KAL_HDATA2D = std::pair<KAL_HMEAN2D, KAL_HCOVA2D>;

//main
using RESULT_DATA = std::pair<int, DETECTBOX3D>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t {
	std::vector<MATCH_DATA> matches;
	std::vector<int> unmatched_tracks;
	std::vector<int> unmatched_detections;
}TRACKER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DYNAMICM;


typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> TRANSFORMATION;
typedef Eigen::Matrix<float, 3, 4, Eigen::RowMajor> PROJ_MATRIX;
typedef Eigen::Matrix<float, 1, 5, Eigen::RowMajor> ELLIPSE_STATE; // [X_center, Y_center, semi_axis_A, semi_axis_B, theta (in radians)]


#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

struct Object3D
{
    vision_msgs::msg::BoundingBox3D box; // float-based 3D bounding box with center.position and size
    int label;
    float prob;
	int64_t time_ms;
};

struct Object2D
{
    std::vector<float> tlbr; // float-based 2D bounding box with center and size 
    int label;
    float prob;
	int64_t time_ms;
};
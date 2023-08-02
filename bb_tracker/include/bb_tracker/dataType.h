#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float, 1, 6, Eigen::RowMajor> DETECTBOX; // 2D xyah - 3D xyzaah
typedef Eigen::Matrix<float, -1, 6, Eigen::RowMajor> DETECTBOXSS;
typedef Eigen::Matrix<float, 1, 128, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, 128, Eigen::RowMajor> FEATURESS;
//typedef std::vector<FEATURE> FEATURESS;

//Kalmanfilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 12, Eigen::RowMajor> KAL_MEAN;	// 2D {xyah position, xyah velocity} - 3D {xyzaah position, xyzaah velocity}
typedef Eigen::Matrix<float, 12, 12, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 6, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 6, 6, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

//main
using RESULT_DATA = std::pair<int, DETECTBOX>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t {
	std::vector<MATCH_DATA> matches;
	std::vector<int> unmatched_tracks;
	std::vector<int> unmatched_detections;
}TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;
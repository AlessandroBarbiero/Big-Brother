#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "dataType.h"

using namespace std;

// Utility functions to convert from an ellipsoid 3D to an ellipse 2D

/**
 * Compute center, axes lengths, and orientation of an ellipse in dual form.
 * Method used in 3D Object Localisation from Multi-view Image Detections (TPAMI 2017)
 *
 * @param C The ellipse in dual form [3x3].
 *
 * @return A tuple containing:
 *   - centre: Ellipse center in Cartesian coordinates [2x1].
 *   - axes: Ellipse axes lengths [2x1].
 *   - R_matrix: Ellipse orientation as a rotation matrix [2x2].
 */
std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix2f> dualEllipseToParameters(Eigen::Matrix3f &C);

/**
 * Compose a dual ellipsoid matrix given axes, rotation matrix, and center.
 *
 * @param axes   A vector of axis lengths.
 * @param R      The rotation matrix.
 * @param center The center vector.
 * @return       The composed dual ellipsoid matrix.
 */
Eigen::Matrix4f composeDualEllipsoid(const std::vector<float>& axes, const Eigen::Matrix3f& R, const Eigen::Vector3f& center);

/**
 * Generate a 3x3 rotation matrix in row-major order based on a yaw angle (in radians).
 *
 * @param yaw_angle The yaw angle in radians for the desired rotation.
 * @return          A 3x3 rotation matrix representing the rotation around the z-axis.
 */
Eigen::Matrix<float,3,3,Eigen::RowMajor> getRotationMatrix(float yaw_angle);

/**
 * Transform the state into an ellipse state inside the image represented by the View Matrix and the Projection matrix.
 * This version uses a series of matrix transformations and is clearer but slower.
 *
 * @param state  An 8-element vector representing the input state:
 *               - x coordinate
 *               - y coordinate
 *               - rotation angle (theta) in radians
 *               - length ratio (l_ratio)
 *               - depth ratio (d_ratio)
 *               - height (h)
 *               - velocity (v)
 *               - angular velocity (w)
 * @param vMat   The View Matrix used for transformation.
 * @param P      The Projection Matrix used for transformation.
 * @return       A 5-elements vector representing ellipse variables:
 *               - x coordinate of the ellipse center
 *               - y coordinate of the ellipse center
 *               - semi-axis a
 *               - semi-axis b
 *               - theta
 */
ELLIPSE_STATE ellipseFromEllipsoidv1(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION &vMat, PROJ_MATRIX &P);

/**
 * Transform the state into an ellipse state inside the image represented by the View Matrix and the Projection matrix.
 * This version uses a complex mathematical equation and is optimized for speed.
 *
 * @param state  An 8-element vector representing the input state:
 *               - x coordinate
 *               - y coordinate
 *               - rotation angle (theta) in radians
 *               - length ratio (l_ratio)
 *               - depth ratio (d_ratio)
 *               - height (h)
 *               - velocity (v)
 *               - angular velocity (w)
 * @param vMat   The View Matrix used for transformation.
 * @param P      The Projection Matrix used for transformation.
 * @return       A 5-elements vector representing ellipse variables:
 *               - x coordinate of the ellipse center
 *               - y coordinate of the ellipse center
 *               - semi-axis a
 *               - semi-axis b
 *               - theta (in radians | -M_PI < theta < M_PI)
 */
ELLIPSE_STATE ellipseFromEllipsoidv2(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION &vMat, PROJ_MATRIX &P);

// Compute the intersection between a vector starting from the camera center
// passing through the point 2D and the horiontal plane placed at a given height
Eigen::Vector3f projectPoint2D(Eigen::Vector2f point2D, float plane_height, TRANSFORMATION &vMat, PROJ_MATRIX &P);

KAL_MEAN ellipsoidFromEllipse(const ELLIPSE_STATE &state, ClassLabel class_label, TRANSFORMATION &vMat, PROJ_MATRIX &P);

inline bool ellipseInImage(float ecx, float ecy, float ea, float eb, int width, int height){
  return (ea > 0 && eb > 0 && ecx-ea > 0 && ecx+ea < width && ecy-eb > 0 && ecy+eb < height);
}

inline bool boxInImage(float min_x, float min_y, float max_x, float max_y, int width, int height){
  return (min_x>0 && min_y>0 && max_x<width && max_y<height);
}

inline bool objectBehindCamera(float cx, float cy, float cz, TRANSFORMATION &vMat){
  return (vMat(2,0)*cx +vMat(2,1)*cy + vMat(2,2)*cz + vMat(2,3) < 0);
}

// Utility functions to draw a box around an ellipse

/**
 * Calculate the coordinates of the top-left and bottom-right points of the axis aligned bounding box (TLBR) surrounding the ellipse
 * from ellipse parameters.
 *
 * @param ecx   X-coordinate of the center of the ellipse.
 * @param ecy   Y-coordinate of the center of the ellipse.
 * @param ea    Semi-axis 'a' of the ellipse.
 * @param eb    Semi-axis 'b' of the ellipse.
 * @param theta Angle of rotation of the ellipse.
 * @return      Vector of floats containing the coordinates of TLBR points: [min_x, min_y, max_x, max_y].
 */
vector<float> tlbrFromEllipse(float ecx, float ecy, float ea, float eb, float theta);

/**
 * Calculate the coordinates of the top-left and bottom-right points of the axis aligned bounding box (TLBR) surrounding the ellipse
 * from ellipse parameters.
 * 
 * @param e_state   A 5-elements vector representing ellipse variables:
 *                  - x coordinate of the ellipse center
 *                  - y coordinate of the ellipse center
 *                  - semi-axis a
 *                  - semi-axis b
 *                  - theta
 * @return          Vector of floats containing the coordinates of TLBR points: [min_x, min_y, max_x, max_y].
 */
inline vector<float> tlbrFromEllipse(ELLIPSE_STATE &e_state){
  return tlbrFromEllipse(e_state[0], e_state[1], e_state[2], e_state[3], e_state[4]);
}
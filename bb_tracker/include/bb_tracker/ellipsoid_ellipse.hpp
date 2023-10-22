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
inline std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix2f> dualEllipseToParameters(Eigen::Matrix3f C) {
  if (C(2, 2) != 0) {
      C /= -C(2, 2);
  }

  // Take the first two elements of last column and put in column vector
  Eigen::Vector2f centre = -C.block<2, 1>(0, 2);

  // T = [  I | -centre ]
  //     [  0 |     1   ]
  Eigen::Matrix3f T;
  T << Eigen::Matrix2f::Identity(), -centre, 0, 0, 1;

  Eigen::Matrix3f C_centre = T * C * T.transpose();
  C_centre = 0.5 * (C_centre + C_centre.transpose());

  cout << "C_sub = \n" << C_centre.block<2, 2>(0, 0) << endl;

  Eigen::EigenSolver<Eigen::Matrix2f> eigensolver(C_centre.block<2, 2>(0, 0));
  Eigen::Vector2f D = eigensolver.eigenvalues().real().cwiseSqrt();
  Eigen::Matrix2f V = eigensolver.eigenvectors().real();

  // cout << "Ellipse parameters:" << 
  //       "\n\tcentre:\n" << centre << 
  //       "\n\tT:\n" << T <<
  //       "\n\tD:\n" << D <<
  //       "\n\tV:\n" << V << endl;

  return std::make_tuple(centre, D, V);
}

/**
 * Compose a dual ellipsoid matrix given axes, rotation matrix, and center.
 *
 * @param axes   A vector of axis lengths.
 * @param R      The rotation matrix.
 * @param center The center vector.
 * @return       The composed dual ellipsoid matrix.
 */
inline Eigen::Matrix4f composeDualEllipsoid(const std::vector<float>& axes, const Eigen::Matrix3f& R, const Eigen::Vector3f& center) {
  Eigen::Matrix4f Q;
  Q.setIdentity();
  for (int i = 0; i < 3; ++i) {
    Q(i, i) = axes[i] * axes[i];
  }
  Q(3, 3) = -1.0;

  Eigen::Matrix4f Re_w = Eigen::Matrix4f::Identity();
  Re_w.block<3, 3>(0, 0) = R.transpose();

  Eigen::Matrix4f T_center_inv = Eigen::Matrix4f::Identity();
  T_center_inv.block<3, 1>(0, 3) = center;

  Eigen::Matrix4f Q_star = T_center_inv * Re_w.transpose() * Q * Re_w * T_center_inv.transpose();
  return Q_star;
}

/**
 * Generate a 3x3 rotation matrix in row-major order based on a yaw angle (in radians).
 *
 * @param yaw_angle The yaw angle in radians for the desired rotation.
 * @return          A 3x3 rotation matrix representing the rotation around the z-axis.
 */
inline Eigen::Matrix<float,3,3,Eigen::RowMajor> getRotationMatrix(float yaw_angle){
  Eigen::Matrix<float,3,3,Eigen::RowMajor> R;
  // Quaternion for general form
  // tf2::Quaternion quat;
  // quat.setRPY(0,0,yaw_angle);
  // tf2::Matrix3x3 rot_ellipsoid(quat);
  // R <<  rot_ellipsoid[0][0], rot_ellipsoid[0][1], rot_ellipsoid[0][2],
  //       rot_ellipsoid[1][0], rot_ellipsoid[1][1], rot_ellipsoid[1][2],
  //       rot_ellipsoid[2][0], rot_ellipsoid[2][1], rot_ellipsoid[2][2];

  R <<  cos(yaw_angle), -sin(yaw_angle), 0,
        sin(yaw_angle), cos(yaw_angle), 0,
        0, 0, 1;
  return R;
}

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
inline ELLIPSE_STATE ellipseFromEllipsoidv1(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION vMat, PROJ_MATRIX P){
  // State
  float x = state(0),
  y =       state(1),
  theta =   state(2),
  l_ratio = state(3),
  d_ratio = state(4),
  h =       state(5);
  // v =       state(6),
  // w =       state(7);

  // 1 >>> Transform bbox into ellipsoid 3D

  float z = h/2,
  a = l_ratio*h/2, 
  b = d_ratio*h/2, 
  c = h/2;
  std::vector<float> semi_axis = {a,b,c};
  Eigen::Matrix<float,3,3,Eigen::RowMajor> R = getRotationMatrix(theta);
  Eigen::Vector3f center_vec;
  center_vec << x,y,z;
  // This is the dual ellipsoid in fixed_frame
  Eigen::Matrix4f dual_ellipsoid_fix_f = composeDualEllipsoid(semi_axis, R, center_vec);
  Eigen::Matrix4f dual_ellipsoid;
  // The object is in the _fixed_frame tf, I want to convert its coordinate into camera frame before multiply for projection matrix
  dual_ellipsoid = vMat * dual_ellipsoid_fix_f * vMat.transpose();
  cout << "dual_ellipsoid: \n" << dual_ellipsoid << endl;

  // 2 >>> Transform Ellipsoid 3D in ellipse 2D

  Eigen::Matrix3f dual_ellipse;
  dual_ellipse = P * dual_ellipsoid * P.transpose();

  cout << "dual_ellipse: \n" << dual_ellipse << endl;

  // 3 >>> Compute parameters from conic form of ellipse

  std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix2f> ellipse_params = dualEllipseToParameters(dual_ellipse);
  Eigen::Vector2f e_center = std::get<0>(ellipse_params);
  Eigen::Vector2f e_axis = std::get<1>(ellipse_params);
  Eigen::Matrix2f e_rotmat = std::get<2>(ellipse_params);

  ELLIPSE_STATE result;
  result << 
      e_center(0),
      e_center(1),
      e_axis(0),
      e_axis(1),
      atan2(e_rotmat(1, 0), e_rotmat(0, 0)); // theta
  return result;
}

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
 *               - theta
 */
inline ELLIPSE_STATE ellipseFromEllipsoidv2(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION vMat, PROJ_MATRIX P){
  ELLIPSE_STATE result;
  // State
  float x = state(0),
  y =       state(1),
  theta =   state(2),
  l_ratio = state(3),
  d_ratio = state(4),
  h =       state(5),
  // v =       state(6),
  // w =       state(7),
  // View
  vr00 = vMat(0,0),
  vr01 = vMat(0,1),
  vr02 = vMat(0,2),
  vr10 = vMat(1,0),
  vr11 = vMat(1,1),
  vr12 = vMat(1,2),
  vr20 = vMat(2,0),
  vr21 = vMat(2,1),
  vr22 = vMat(2,2),
  vtx = vMat(0,3),
  vty = vMat(1,3),
  vtz = vMat(2,3),
  // Projection
  fx = P(0,0),
  fy = P(1,1),
  cx = P(0,2),
  cy = P(1,2);

  result <<
    ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))/((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) ,
    ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)))/((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) ,
    sqrt(0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) + 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) - sqrt(0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) - 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) + 0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) + 1.0*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) - 1.0/4.0*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) + (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) + ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4))) ,
    sqrt(0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) + 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) + sqrt(0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) - 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) + 0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) + 1.0*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) - 1.0/4.0*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) + (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) + ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4))) ,
    atan2(1, (1.0*sqrt((0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2), 2) - 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2)) + 0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2), 2) + 1.0*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) - (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) - ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4))*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) + 0.5*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) - 0.5*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) - 0.5*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2) + 0.5*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) - (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) - ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)))));
  return result;
}

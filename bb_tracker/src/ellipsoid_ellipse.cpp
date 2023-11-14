#include <bb_tracker/ellipsoid_ellipse.hpp>


// Utility functions to convert from an ellipsoid 3D to an ellipse 2D

std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix2f> dualEllipseToParameters(Eigen::Matrix3f &C) {
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

Eigen::Matrix4f composeDualEllipsoid(const std::vector<float>& axes, const Eigen::Matrix3f& R, const Eigen::Vector3f& center) {
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

Eigen::Matrix<float,3,3,Eigen::RowMajor> getRotationMatrix(float yaw_angle){
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

ELLIPSE_STATE ellipseFromEllipsoidv1(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION &vMat, PROJ_MATRIX &P){
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

ELLIPSE_STATE ellipseFromEllipsoidv2(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION &vMat, PROJ_MATRIX &P){
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

  float fixed_axis_part = 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) + 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2);

  float variable_axis_part = sqrt(0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) - 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) + 0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4) + 1.0*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*(-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) - 1.0/4.0*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) + (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) + ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))), 2)/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4));

  result <<
    ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))/((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) ,
    ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)))/((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) ,
    sqrt(fixed_axis_part - variable_axis_part) ,
    sqrt(fixed_axis_part + variable_axis_part) ,
    atan2(1, (1.0*sqrt((0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2), 2) - 0.5*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2))*(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2)) + 0.25*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) - pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2), 2) + 1.0*pow(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) - (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) - ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 4))*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2) + 0.5*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2)) - 0.5*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) - 0.5*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)), 2) + 0.5*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/(((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) - (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11))) - ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cx*vr22 + fx*vr02) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01)))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)))));

  // Ensure theta is in the range [-pi, pi]
  if (result(4) < -M_PI) {
      result(4) += 2 * M_PI;
  } else if (result(4) > M_PI) {
      result(4) -= 2 * M_PI;
  }

  return result;
}

Eigen::Vector3f find_3d_point_on_line(const Eigen::Vector3f& normalized_vector, float z_value) {
    // Calculate proportions
    float a = z_value / normalized_vector[2];

    // Calculate the final coordinates (x, y, z)
    float x = a * normalized_vector[0];
    float y = a * normalized_vector[1];

    return Eigen::Vector3f(x, y, z_value);
}


KAL_MEAN ellipsoidFromEllipse(const ELLIPSE_STATE &state, ClassLabel class_label, TRANSFORMATION &vMat, PROJ_MATRIX &P){
  KAL_MEAN result;
  float 
    x = state(0),
    y = state(1),
    // a = state(2),
    // b = state(3),
    // theta = state(4),
    tx = 0,
    ty = 0,
    // Projection
    fx = P(0,0),
    fy = P(1,1),
    cx = P(0,2),
    cy = P(1,2);
  Eigen::Vector3f ray;
  ray <<  (x-cx-tx)/fx, 
          (y-cy-ty)/fy,
          1.0;
  ray.normalize();

  Eigen::Vector3f dim = priorDimensions.at(class_label);
  Eigen::Vector3f z_vector_world(0.0,0.0,1.0), ground_normal, point_on_ground;
  Eigen::Matrix3f rotMat;
  rotMat = vMat.block<3, 3>(0, 0);
  ground_normal = rotMat * z_vector_world;
  ground_normal.normalize();
  point_on_ground = vMat.block<3,1>(0,3);
  // Intersection vector and plane parallel to the ground at half prior height
  float t = (point_on_ground(0)*ground_normal(0) + point_on_ground(1)*ground_normal(1) + point_on_ground(2)*ground_normal(2) + dim(2)/2.0) /
            (ray(0)*ground_normal(0) + ray(1)*ground_normal(1) + ray(2)*ground_normal(2));
  

  Eigen::Vector4f point_camera_frame, point_world_frame;
  point_camera_frame << ray * t, 1.0;
  point_world_frame = vMat.inverse() * point_camera_frame;
  point_world_frame /= point_world_frame(3);

  float l_ratio, d_ratio, h;
  
  l_ratio = dim(0)/dim(2);
  d_ratio = dim(1)/dim(2);
  h = dim(2);

  result << point_world_frame(0), // x
            point_world_frame(1), // y
            state(4),             // yaw
            l_ratio,
            d_ratio,
            h,
            0.0,                  // v
            0.0;                  // w
  return result;
}

// Utility functions to draw a box around an ellipse

vector<float> tlbrFromEllipse(float ecx, float ecy, float ea, float eb, float theta){
  float max_y, min_y, max_x, min_x;
  // Calculate the maximum and minimum x coordinates
  max_x = (-pow(ea, 3)*eb*pow(cos(theta), 2) - ea*pow(eb, 3)*pow(sin(theta), 2) + ecx*sqrt(pow(ea, 2)*pow(eb, 2)*(-pow(ea, 2)*pow(sin(theta), 2) + pow(ea, 2) + pow(eb, 2)*pow(sin(theta), 2))))/sqrt(pow(ea, 2)*pow(eb, 2)*(-pow(ea, 2)*pow(sin(theta), 2) + pow(ea, 2) + pow(eb, 2)*pow(sin(theta), 2)));
  min_x = (pow(ea, 3)*eb*pow(cos(theta), 2) + ea*pow(eb, 3)*pow(sin(theta), 2) + ecx*sqrt(pow(ea, 2)*pow(eb, 2)*(-pow(ea, 2)*pow(sin(theta), 2) + pow(ea, 2) + pow(eb, 2)*pow(sin(theta), 2))))/sqrt(pow(ea, 2)*pow(eb, 2)*(-pow(ea, 2)*pow(sin(theta), 2) + pow(ea, 2) + pow(eb, 2)*pow(sin(theta), 2)));
  // Ensure max_x is greater than min_x
  if(max_x<min_x)
    std::swap(max_x, min_x);
  
  // Calculate the maximum and minimum y coordinates
  max_y = (-pow(ea, 3)*eb*pow(sin(theta), 2) - ea*pow(eb, 3)*pow(cos(theta), 2) + ecy*sqrt(pow(ea, 2)*pow(eb, 2)*(pow(ea, 2)*pow(sin(theta), 2) - pow(eb, 2)*pow(sin(theta), 2) + pow(eb, 2))))/sqrt(pow(ea, 2)*pow(eb, 2)*(pow(ea, 2)*pow(sin(theta), 2) - pow(eb, 2)*pow(sin(theta), 2) + pow(eb, 2)));
  min_y = (pow(ea, 3)*eb*pow(sin(theta), 2) + ea*pow(eb, 3)*pow(cos(theta), 2) + ecy*sqrt(pow(ea, 2)*pow(eb, 2)*(pow(ea, 2)*pow(sin(theta), 2) - pow(eb, 2)*pow(sin(theta), 2) + pow(eb, 2))))/sqrt(pow(ea, 2)*pow(eb, 2)*(pow(ea, 2)*pow(sin(theta), 2) - pow(eb, 2)*pow(sin(theta), 2) + pow(eb, 2)));
  // Ensure max_y is greater than min_y
  if(max_y<min_y)
    std::swap(max_y, min_y);
  return {min_x, min_y, max_x, max_y};
}
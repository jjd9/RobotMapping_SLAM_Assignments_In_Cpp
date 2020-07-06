#include "tools.h"

/*
 Compute the error of a pose-landmark constraint
 x 3x1 vector (x,y,theta) of the robot pose
 l 2x1 vector (x,y) of the landmark
 z 2x1 vector (x,y) of the measurement, the position of the landmark in
   the coordinate frame of the robot given by the vector x

 Output
 e 2x1 error of the constraint
 A 2x3 Jacobian wrt x
 B 2x2 Jacobian wrt l
*/
LinearizedConstraintError linearize_pose_landmark_constraint(Vector3f x, Vector2f l, Vector2f z){

  // compute the error and the Jacobians of the error
  LinearizedConstraintError lin_c_error;

  Matrix2f R = v2t(x).block(0,0,2,2);
  lin_c_error.e = R.transpose()*(l-x.block(0,0,2,1)) - z;

  x(2) = std::atan2(R(1,0),R(0,0));

  lin_c_error.A = MatrixXf::Zero(2,3);
  Vector3f row0( -std::cos(x(2)), -std::sin(x(2)),   std::cos(x(2))*(l(1) - x(1)) - std::sin(x(2))*(l(0) - x(0)));
  lin_c_error.A.row(0) = row0.transpose();
  Vector3f row1(std::sin(x(2)), -std::cos(x(2)), - std::cos(x(2))*(l(0) - x(0)) - std::sin(x(2))*(l(1) - x(1)));
  lin_c_error.A.row(1) = row1.transpose(); 

  lin_c_error.B = Matrix2f::Zero();
  lin_c_error.B(0,0) =  std::cos(x(2));
  lin_c_error.B(0,1) =  std::sin(x(2));
  lin_c_error.B(1,0) = -std::sin(x(2));
  lin_c_error.B(1,1) = std::cos(x(2));


  return lin_c_error;

}
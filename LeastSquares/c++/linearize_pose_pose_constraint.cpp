#include "tools.h"

/*
 Compute the error of a pose-pose constraint
 x1 3x1 vector (x,y,theta) of the first robot pose
 x2 3x1 vector (x,y,theta) of the second robot pose
 z 3x1 vector (x,y,theta) of the measurement

 You may use the functions v2t() and t2v() to compute
 a Homogeneous matrix out of a (x, y, theta) vector
 for computing the error.

 Output
 e 3x1 error of the constraint
 A 3x3 Jacobian wrt x1
 B 3x3 Jacobian wrt x2
*/
LinearizedConstraintError linearize_pose_pose_constraint(Vector3f x1, Vector3f x2, Vector3f z){

  // compute the error and the Jacobians of the error
  LinearizedConstraintError lin_c_error;

  Matrix3f Z_pred = v2t(x1).inverse()*v2t(x2);
  Matrix3f Zinv = v2t(z).inverse();
  lin_c_error.e = t2v(Zinv*Z_pred);

  lin_c_error.A = Matrix3f::Zero(); // de12/dx1
  Vector3f Arow0(-std::cos(x1(2) + z(2)), -std::sin(x1(2) + z(2)), x2(1)*std::cos(x1(2) + z(2)) - x1(1)*std::cos(x1(2) + z(2)) + x1(0)*std::sin(x1(2) + z(2)) - x2(0)*std::sin(x1(2) + z(2)));
  lin_c_error.A.row(0) = Arow0.transpose();
  Vector3f Arow1(std::sin(x1(2) + z(2)), -std::cos(x1(2) + z(2)), x1(0)*std::cos(x1(2) + z(2)) - x2(0)*std::cos(x1(2) + z(2)) + x1(1)*std::sin(x1(2) + z(2)) - x2(1)*std::sin(x1(2) + z(2)));
  lin_c_error.A.row(1) = Arow1.transpose();
  lin_c_error.A(2,2) = -1;

  lin_c_error.B = Matrix3f::Zero(); // de12/dx2
  Vector3f Brow0( std::cos(x1(2) + z(2)), std::sin(x1(2) + z(2)), 0);
  lin_c_error.B.row(0) = Brow0.transpose();
  Vector3f Brow1(-std::sin(x1(2) + z(2)), std::cos(x1(2) + z(2)), 0);  
  lin_c_error.B.row(1) = Brow1.transpose();
  lin_c_error.B(2,2) = 1;  

  return lin_c_error;


}

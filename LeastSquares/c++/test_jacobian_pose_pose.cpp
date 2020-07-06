#include "tools.h"

int main(){
  float epsilon = 1e-5;

  Vector3f x1;
  x1 << 1.1, 0.9, 1;
  Vector3f x2;
  x2 << 2.2, 1.85, 1.2;
  Vector3f z ;
  z  << 0.9, 1.1, 1.05;

  // get the analytic Jacobian
  LinearizedConstraintError lin_c_error = linearize_pose_pose_constraint(x1, x2, z);
  Vector3f e = lin_c_error.e;
  MatrixXf A = lin_c_error.A;
  MatrixXf B = lin_c_error.B;


  // check the error vector
  Vector3f e_true;
  e_true << -1.06617,  -1.18076,  -0.85000;

  if ((e - e_true).norm() > epsilon){
    std::cout << "Your error function seems to return a wrong value"  << std::endl;
    std::cout << "Result of your function \n" << e << std::endl;
    std::cout << "True value \n" << e_true << std::endl;
  }else{
    std::cout << "The computation of the error vector appears to be correct" << std::endl;
  }

  // compute it numerically
  float delta = 1e-6;
  float scalar = 1 / (2*delta);

  // test for x1
  Matrix3f ANumeric = Matrix3f::Zero();
  for (int d = 0; d < 3; ++d){
    Vector3f curX = x1;
    curX(d) += delta;
    LinearizedConstraintError temp_error = linearize_pose_pose_constraint(curX, x2, z);
    VectorXf err = temp_error.e; 
    curX = x1;
    curX(d) -= delta;
    temp_error = linearize_pose_pose_constraint(curX, x2, z);
    err -= temp_error.e;

    ANumeric.col(d) = scalar * err;
  }

  Matrix3f Adiff = ANumeric - A;
  if (Adiff.array().abs().maxCoeff() > epsilon){
    std::cout << "Error in the Jacobian for x1" << std::endl;
    std::cout << "Your analytic Jacobian \n" << A << std::endl;
    std::cout << "Numerically computed Jacobian \n" << ANumeric << std::endl;
    std::cout << "Difference \n" << Adiff  << std::endl;
  }else{
    std::cout << "Jacobian for x1 appears to be correct" << std::endl;
  }


  // test for x2
  Matrix3f BNumeric = Matrix3f::Zero();
  for(int d = 0; d < 3; ++d){
    Vector3f curX = x2;
    curX(d) += delta;
    LinearizedConstraintError temp_error = linearize_pose_pose_constraint(x1, curX, z);
    VectorXf err = temp_error.e; 
    curX = x2;
    curX(d) -= delta;
    temp_error = linearize_pose_pose_constraint(x1, curX, z);
    err -= temp_error.e;

    BNumeric.col(d) = scalar * err;
  }

  MatrixXf Bdiff = BNumeric - B;
  if (Bdiff.array().abs().maxCoeff() > epsilon){
    std::cout << "Error in the Jacobian for x2" << std::endl;
    std::cout << "Your analytic Jacobian \n" << B << std::endl;
    std::cout << "Numerically computed Jacobian \n" << BNumeric << std::endl;
    std::cout << "Difference \n"  << Bdiff << std::endl;
  }else{
    std::cout << "Jacobian for x1 appears to be correct" << std::endl;
  }
  return 0;
}
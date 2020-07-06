#include "tools.h"

Vector3f t2v(Matrix3f A){
    // computes the pose vector v from an homogeneous transform A
    Vector3f v;
    v.block(0,0,2,1) = A.block(0,2,2,1); 
  	v(2)=std::atan2(A(1,0),A(0,0));
    return v;
}

Matrix3f v2t(Vector3f v){
    // computes the homogeneous transform matrix A of the pose vector v
    Matrix3f A;
  	float c=std::cos(v(2));
  	float s=std::sin(v(2));
	A << c, -s, v(0),
	     s,  c, v(1),
	     0,   0,  1;

    return A;
}

/*
 this function solves the odometry calibration problem
 given a measurement matrix Z.
 We assume that the information matrix is the identity
 for each of the measurements
 Every row of the matrix contains
 z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
 Z:	The measurement matrix
 X:	the calibration matrix
 returns the correction matrix X
*/

Matrix3f ls_calibrate_odometry(MatrixXf Z){
  // initial solution (the identity transformation)
  Matrix3f X = Matrix3f::Identity(); 
  Matrix3f Omega = Matrix3f::Identity(); 

  // initialize H and b of the linear system
  VectorXf b = VectorXf::Zero(9,1);
  MatrixXf H = MatrixXf::Zero(9,9);
  
  // Loop through the measurements and update H and b
  // You may call the functions error_function and jacobian, see below
  // We assume that the information matrix is the identity.
  for(int i = 0; i < Z.rows(); ++i){
    Vector3f e = error_function(i, X, Z);
    MatrixXf J = jacobian(i,Z);
    b += (e.transpose()*Omega*J).transpose(); // 9x1
    H += J.transpose()*Omega*J; // 9x9
  }
  
  // solve and update the solution
  VectorXf deltaX = -H.inverse()*b; // 9x1

  X.reshaped<RowMajor>() += deltaX;

  return X;
}

// this function computes the error of the i^th measurement in Z
// given the calibration parameters
// i:	the number of the measurement
// X:	the actual calibration parameters
// Z:	the measurement matrix, each row contains first the scan-match result
//       and then the motion reported by odometry
// e:	the error of the ith measurement
Vector3f error_function(int i, Matrix3f &X, MatrixXf &Z){
  // TODO compute the error of each measurement
  Vector3f e = Z.block(i,0,1,3).transpose() - X*Z.block(i,3,1,3).transpose();

  return e;
}

// derivative of the error function for the ith measurement in Z
// i:	the measurement number
// Z:	the measurement matrix
// J:	the jacobian of the ith measurement
MatrixXf jacobian(int i, MatrixXf Z){
  // compute the Jacobian
  MatrixXf J = MatrixXf::Zero(3,9);
  J.block(0,0,1,3) = Z.block(i,0,1,3);
  J.block(1,3,1,3) = Z.block(i,0,1,3);
  J.block(2,6,1,3) = Z.block(i,0,1,3);

  return -J;
}

/*
 computes a calibrated vector of odometry measurements
 by applying the bias term to each line of the measurements
 X: 	3x3 matrix obtained by the calibration process
 U: 	Nx3 matrix containing the odometry measurements
 C:	Nx3 matrix containing the corrected odometry measurements	
*/

MatrixXf apply_odometry_correction(Matrix3f X, MatrixXf U){
    // compute the calibrated motion vector, try to vectorize
    MatrixXf C = (X*U.transpose()).transpose();

    return C;
}

/*
 computes the trajectory of the robot by chaining up
 the incremental movements of the odometry vector
 U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
 T:	a (N+1)x3 matrix, each row contains the robot position (starting from 0,0,0)
*/
MatrixXf compute_trajectory(MatrixXf U){
  // initialize the trajectory matrix
  MatrixXf T = MatrixXf::Zero(U.rows()+1, 3);
  // the current pose in the chain
  Matrix3f currentPose = v2t(T.row(0).transpose());

  // compute the result of chaining up the odometry deltas
  // Note that U(i) results in T(i+1).
  // T(i+1) can be computed by calling t2v(currentPose)
  // after computing the current pose of the robot
  for(int i = 0; i < U.rows(); ++i){
    currentPose = currentPose*v2t(U.row(i).transpose());
    T.row(i+1) = t2v(currentPose).transpose();
  }

  return T;

}

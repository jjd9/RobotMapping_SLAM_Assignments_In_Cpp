#include "tools.h"

// performs one iteration of the Gauss-Newton algorithm
// each constraint is linearized and added to the Hessian

void assignToSparseMat(SparseMatrix<float> &H, int startRow, int startCol, MatrixXf mat){
  for(int row = 0; row < mat.rows(); ++row){
    for(int col = 0; col < mat.cols(); ++col){
      H.coeffRef(row+startRow,col+startCol) += mat(row,col);
    }
  }
}

VectorXf linearize_and_solve(Graph &g){

  int nnz = nnz_of_graph(g);

  // allocate the sparse H and the vector b
  SparseMatrix<float> H = SparseMatrix<float>(g.x.rows(), g.x.rows());
  H.reserve(nnz);
  VectorXf b = VectorXf::Zero(g.x.rows());

  bool needToAddPrior = true;

  // compute the addend term to H and b for each of our constraints
  std::cout << "linearize and build system" << std::endl;

  for (auto edge : g.edges){

    // pose-pose constraint
    if (edge.type == 80){
      // edge.fromIdx and edge.toIdx describe the location of
      // the first element of the pose in the state vector
      // You should use also this index when updating the elements
      // of the H matrix and the vector b.
      // edge.measurement is the measurement
      // edge.information is the information matrix
      Vector3f x1 = g.x.block(edge.fromIdx,0,3,1);  // the first robot pose
      Vector3f x2 = g.x.block(edge.toIdx,0,3,1);      // the second robot pose

      // Computing the error and the Jacobians
      // e the error vector
      // A Jacobian wrt x1
      // B Jacobian wrt x2
      LinearizedConstraintError lin_c_error = linearize_pose_pose_constraint(x1, x2, edge.measurement);

      // compute and add the term to H and b
      int i = edge.fromIdx;
      int j = edge.toIdx;
      assignToSparseMat(H, i,i, lin_c_error.A.transpose()*edge.information*lin_c_error.A);
      assignToSparseMat(H, i,j, lin_c_error.A.transpose()*edge.information*lin_c_error.B);
      assignToSparseMat(H, j,i, lin_c_error.B.transpose()*edge.information*lin_c_error.A);
      assignToSparseMat(H, j,j, lin_c_error.B.transpose()*edge.information*lin_c_error.B);

      b.block(i,0,3,1) += (lin_c_error.e.transpose()*edge.information*lin_c_error.A).transpose();
      b.block(j,0,3,1) += (lin_c_error.e.transpose()*edge.information*lin_c_error.B).transpose();

      if (needToAddPrior){
        // add the prior for one pose of this edge
        // This fixes one node to remain at its current location
        H.coeffRef(0,0) += 1.0;
        H.coeffRef(1,1) += 1.0;
        H.coeffRef(2,2) += 1.0;

        needToAddPrior = false;
      }

    // pose-landmark constraint
    }else if (edge.type == 76){
      // edge.fromIdx and edge.toIdx describe the location of
      // the first element of the pose and the landmark in the state vector
      // You should use also this index when updating the elements
      // of the H matrix and the vector b.
      // edge.measurement is the measurement
      // edge.information is the information matrix
      Vector3f x1 = g.x.block(edge.fromIdx,0,3,1);  // the robot pose
      Vector2f lm = g.x.block(edge.toIdx,0,2,1);      // the landmark

      // Computing the error and the Jacobians
      // e the error vector
      // A Jacobian wrt x1
      // B Jacobian wrt lm
      LinearizedConstraintError lin_c_error = linearize_pose_landmark_constraint(x1, lm, edge.measurement);


      // compute and add the term to H and b
      int i = edge.fromIdx;
      int j = edge.toIdx;
      assignToSparseMat(H, i,i, lin_c_error.A.transpose()*edge.information*lin_c_error.A);
      assignToSparseMat(H, i,j, lin_c_error.A.transpose()*edge.information*lin_c_error.B);
      assignToSparseMat(H, j,i, lin_c_error.B.transpose()*edge.information*lin_c_error.A);
      assignToSparseMat(H, j,j, lin_c_error.B.transpose()*edge.information*lin_c_error.B);
      b.block(i,0,3,1) += (lin_c_error.e.transpose()*edge.information*lin_c_error.A).transpose();
      b.block(j,0,2,1) += (lin_c_error.e.transpose()*edge.information*lin_c_error.B).transpose();


    }
  }

  std::cout << "solving system" << std::endl;

  // solve the linear system, whereas the solution should be stored in dx
  // Remember to use the backslash operator instead of inverting H
  VectorXf dx;

  // solve Ax = b
  SimplicialLDLT<SparseMatrix<float>> solver;
  solver.compute(H);
  dx = solver.solve(b);

  std::cout << solver.info() << std::endl;

  return -dx;

}

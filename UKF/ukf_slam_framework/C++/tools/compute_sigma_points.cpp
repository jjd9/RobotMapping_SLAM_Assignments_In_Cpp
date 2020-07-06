#include "tools.h"

MatrixXf compute_sigma_points(VectorXf mu, MatrixXf sigma){
    // Computes the 2n+1 sigma points according to the unscented transform,
    // where n is the dimensionality of the mean vector mu.
    // The sigma points should form the columns of sigma_points,
    // i.e. sigma_points is an nx2n+1 matrix.

    // Compute lambda
    int n = mu.rows();
    int num_sig = 2*n+1;
    float lambda = scale - n;

    // Compute sigma points
    MatrixXf sigmasqr( ((n + lambda)*sigma).llt().matrixL() );

    MatrixXf sigma_points = MatrixXf::Zero(n,2*n+1); 
    
    sigma_points.col(0) = mu;
    for(int i = 1; i <= n; ++i){
        sigma_points.col(i) = mu + sigmasqr.col(i-1);
    }
    for(int i = n+1; i <= 2*n; ++i){
        sigma_points.col(i) = mu - sigmasqr.col(i-n-1);
    }

    return sigma_points;
}

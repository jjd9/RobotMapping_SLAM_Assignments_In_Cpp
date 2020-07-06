#include "tools.h"

UKF_values compute_sigma_points(Vector2f mu, Matrix2f sigma, float lambda, float alpha, float beta){
    // This function samples 2n+1 sigma points from the distribution given by mu and sigma
    // according to the unscented transform, where n is the dimensionality of mu.
    // Each column of sigma_points should represent one sigma point
    // i.e. sigma_points has a dimensionality of nx2n+1.
    // The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
    // w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
    // They are later used to recover the mean and covariance respectively.

    int n = mu.rows();
    UKF_values ukf_vals;
    ukf_vals.sigma_points = MatrixXf::Zero(n,2*n+1);
    ukf_vals.w_m = VectorXf::Zero(2*n+1); 
    ukf_vals.w_c = VectorXf::Zero(2*n+1);

    // compute all sigma points
    Matrix2f matSqrt( ((n + lambda)*sigma).llt().matrixL() );
    ukf_vals.sigma_points.col(0) = mu;

    for(int i = 1; i <= n; ++i){
        ukf_vals.sigma_points.col(i) = mu + matSqrt.col(i-1);
    }
    for(int i = n+1; i <= 2*n; ++i){
        ukf_vals.sigma_points.col(i) = mu - matSqrt.col(i-n-1);
    }

    // compute weight vectors w_m and w_c
    ukf_vals.w_m(0) = lambda/(n+lambda);
    ukf_vals.w_c(0) = lambda/(n+lambda) + (1-alpha*alpha+beta);
    for(int i = 1; i <= 2*n; ++i){
        ukf_vals.w_m(i) = 1.0/2.0/(n+lambda);
        ukf_vals.w_c(i) = 1.0/2.0/(n+lambda);
    }

    return ukf_vals;
}

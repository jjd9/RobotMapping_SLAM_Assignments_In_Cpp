#include "tools.h"

Gaussian recover_gaussian(UKF_values ukf_vals){
    // This function computes the recovered Gaussian distribution (mu and sigma)
    // given the sigma points (size: nx2n+1) and their weights w_m and w_c:
    // w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
    // The weight vectors are each 1x2n+1 in size,
    // where n is the dimensionality of the distribution.

    int n = ukf_vals.sigma_points.rows();

    // ukf_vals.sigma_points, ukf_vals.w_m, ukf_vals.w_m
    Gaussian gauss_dist;

    for (int i = 0; i <= 2*n; ++i){
        // compute mu
        gauss_dist.mu += ukf_vals.w_m(i)*ukf_vals.sigma_points.col(i);
    }

    // Try to vectorize your operations as much as possible
    MatrixXf diff = ukf_vals.sigma_points-gauss_dist.mu.replicate(1,2*n+1);
    MatrixXf diffSig = diff*diff.transpose();
    for (int i = 0; i <= 2*n; ++i){
        // compute sigma
        gauss_dist.sigma += ukf_vals.w_c(i)*diffSig;
    }

    return gauss_dist;

}

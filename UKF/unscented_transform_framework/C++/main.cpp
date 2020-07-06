#include "tools.h"

// This is the main script for computing a transformed distribution according 
// to the unscented transform. This script calls all the required
// functions in the correct order.
// If you are unsure about the input and return values of functions you
// should read their documentation which tells you the expected dimensions.

int main(){
    // Initial distribution
    Matrix2f sigma = 0.1*Matrix2f::Identity();
    Vector2f mu = {1,2};
    int n = mu.rows();

    // Compute lambda
    float alpha = 0.9;
    float beta = 2;
    float kappa = 1;
    float lambda = alpha*alpha*(n+kappa)-n;

    // Compute the sigma points corresponding to mu and sigma
    std::cout << "compute sigma points" << std::endl;
    UKF_values ukf_vals = compute_sigma_points(mu, sigma, lambda, alpha, beta);

    // output original distribution with sampled sigma points

    // Transform sigma points
    std::cout << "transform" << std::endl;
    transform(ukf_vals.sigma_points);

    // Recover mu and sigma of the transformed distribution
    std::cout << "recover gaussian" << std::endl;
    Gaussian gauss_dist = recover_gaussian(ukf_vals);

    std::cout << "mu: " << gauss_dist.mu << std::endl;
    std::cout << "sigma: " << gauss_dist.sigma << std::endl;


    // output new distribution
    
    
    return 0;
}
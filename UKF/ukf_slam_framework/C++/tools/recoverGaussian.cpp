#include "tools.h"

Gaussian recoverGaussian(MatrixXf sigma_points){

    Gaussian gauss;

    int n = sigma_points.rows();
    float lambda = scale - n;
    float w0 = lambda/scale;

    VectorXf w_m = VectorXf::Zero(2*n+1);
    VectorXf w_c = VectorXf::Zero(2*n+1);
    w_m(0) = w0;
    w_c(0) = w0;
    for(int i = 1; i <= 2*n; ++i){
        w_m(i) = 1.0/2.0/scale;
        w_c(i) = 1.0/2.0/scale;
    }

    float cosines = ((sigma_points.row(2).array().cos().array())*(w_m.transpose().array())).sum();
    float sines = ((sigma_points.row(2).array().sin().array())*(w_m.transpose().array())).sum();
    // recompute the angle and normalize it
    float mu_theta = normalize_angle(std::atan2(sines, cosines));
    gauss.mu = (sigma_points.array() * w_m.transpose().replicate(sigma_points.rows(),1).array()).rowwise().sum();
    gauss.mu(2) = mu_theta;

    MatrixXf diff = sigma_points.array() - gauss.mu.replicate(1,sigma_points.cols()).array();
    // Normalize!
    diff.row(2) = normalize_angle(diff.row(2));
    gauss.sigma = ((w_c.transpose().replicate(diff.rows(), 1).array()) * diff.array()).matrix() * diff.transpose();

    return gauss;
}
#pragma once

#include <cmath>

#include "../../eigen/Eigen/Dense"
#include "tools/read_data.h"

using namespace Eigen;

VectorXf g(VectorXf mu, Odometry u){
    float dt = 1.0;
    VectorXf mu_hat = mu;
    mu_hat(0) += u.t*std::cos(mu(2)+u.r1);
    mu_hat(1) += u.t*std::sin(mu(2)+u.r1);
    mu_hat(2) += u.r1+u.r2;

    return mu_hat;
}

MatrixXf robotJacobian(VectorXf mu, Odometry u){
    float dt = 1.0;
    MatrixXf Gx = MatrixXf::Identity(3,3);
    Gx(0,2)= -u.t*std::cos(mu(2)+u.r1);
    Gx(1,2)= u.t*std::sin(mu(2)+u.r1);

    return Gx;
}


void prediction_step(VectorXf &mu, MatrixXf &sigma, Odometry u){
    /*
    Updates the belief concerning the robot pose according to the motion model,
    mu: 2N+3 x 1 vector representing the state mean
    sigma: 2N+3 x 2N+3 covariance matrix
    u: odometry reading (r1, t, r2)
    Use u.r1, u.t, and u.r2 to access the rotation and translation values
    */

    // TODO: Compute the new mu based on the noise-free (odometry-based) motion model
    // Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
    VectorXf mu_hat = g(mu, u);

    // TODO: Compute the 3x3 Jacobian Gx of the motion model
    MatrixXf Gx = robotJacobian(mu, u);

    // TODO: Construct the full Jacobian G
    MatrixXf G = MatrixXf::Identity(mu.size(),mu.size());
    G.block(0,0,3,3) = Gx;

    // Motion noise
    float motionNoise = 0.1;
    Matrix3f R3;
    R3 << motionNoise, 0, 0,
        0, motionNoise, 0,
        0, 0, motionNoise/10.0;
    MatrixXf R = MatrixXf::Zero(sigma.rows(), sigma.rows());
    R.block<3,3>(0,0) = R3;

    // TODO: Compute the predicted sigma after incorporating the motion
    sigma = G*sigma*G.transpose() + R;
    mu = mu_hat;
}
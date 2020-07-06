#pragma once

#include <iostream>
#include <cmath>

#include "../../../eigen/Eigen/Dense"

using namespace Eigen;

struct UKF_values{

    MatrixXf sigma_points;
    VectorXf w_m;
    VectorXf w_c;

};

struct Gaussian{

    Vector2f mu = Vector2f::Zero();
    Matrix2f sigma = Matrix2f::Zero();

};

UKF_values compute_sigma_points(Vector2f mu, Matrix2f sigma, float lambda, float alpha, float beta);

void transform(MatrixXf &points);

Gaussian recover_gaussian(UKF_values ukf_vals);
#pragma once

#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

#include "../../eigen/Eigen/Dense"

using namespace Eigen;


Vector3f t2v(Matrix3f A);

Matrix3f v2t(Vector3f v);

std::vector<std::string> splitBy(std::string str, std::string delimiter);

MatrixXf read_odom(std::string filename);

MatrixXf read_scans(std::string filename);


Matrix3f ls_calibrate_odometry(MatrixXf Z);

Vector3f error_function(int i, Matrix3f &X, MatrixXf &Z);

MatrixXf jacobian(int i, MatrixXf Z);

MatrixXf apply_odometry_correction(Matrix3f X, MatrixXf U);

MatrixXf compute_trajectory(MatrixXf U);


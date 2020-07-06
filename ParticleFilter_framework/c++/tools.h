#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "../../../eigen/Eigen/Dense"

using namespace Eigen;

struct Particle{
  float weight;
  Vector3f pose;
  std::vector<Vector3f> history;
};

std::vector<std::string> splitBy(std::string str, std::string delimiter);

MatrixXf read_data(std::string filename);

float normalize_angle(float phi);

std::vector<Particle> prediction_step(std::vector<Particle> particles, MatrixXf u, Vector3f noise, std::default_random_engine eng);

std::vector<Particle> resample(std::vector<Particle> particles, std::default_random_engine eng);


#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "../../eigen/Eigen/Dense"

using namespace Eigen;

struct LandmarkEstimate{

    bool observed;     // whether or not landmark has been observed
    Vector2f mu;  // x,y position of landmark
    Matrix2f sigma;  // covariance matrix of landmark position

};

struct Particle{
  float weight;
  Vector3f pose;
  std::vector<Vector3f> history;
  std::vector<LandmarkEstimate> landmarks;

};

struct MeasurementPrediction{

    Vector2f h;
    Matrix2f H;

};

struct Landmark{

    int id;     // id of the landmark
    float x;  // x-coordinate
    float y;  // y-coordinate

};

struct Odometry{

    float r1, t, r2;

};

struct Sensor{

    int id;
    float range;
    float bearing;

};

struct RobotData{

    std::vector<Odometry> odometryData;
    std::vector<std::vector<Sensor>> sensorData;
    std::vector<Sensor> sensorBuffer;
    int timesteps = 0;

    RobotData(){

    }

    void addOdometryReading(Odometry od){
        odometryData.push_back(od);
        ++timesteps;
    }
    void addSensorReading(Sensor sense, bool first){
        if (first){
            if(sensorBuffer.size() > 0){
                sensorData.push_back(sensorBuffer);
                sensorBuffer.clear();
            }
        }
        sensorBuffer.push_back(sense);        
    }

};

std::vector<std::string> splitBy(std::string str, std::string delimiter);

std::vector<Landmark> read_world(std::string filename);

RobotData read_data(std::string filename);

float normalize_angle(float phi);

void prediction_step(std::vector<Particle> &particles, Odometry u, Vector3f noise, std::default_random_engine eng);

void resample(std::vector<Particle> &particles, std::default_random_engine eng);

void correction_step(std::vector<Particle> &particles, std::vector<Sensor> z);

MeasurementPrediction measurement_model(const Particle &particle, Sensor z);

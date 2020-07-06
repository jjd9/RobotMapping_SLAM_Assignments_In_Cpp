#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>

#include "../../../../eigen/Eigen/Dense"

using namespace Eigen;

// For computing lambda
// scale = lambda + dimensionality
extern float scale;
extern float pi;

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

struct Landmark{

    int id;     // id of the landmark
    float x;  // x-coordinate
    float y;  // y-coordinate

};


struct WorldData{

    std::vector<Landmark> landmarks;

    void addLandmark(Landmark lm){
        landmarks.push_back(lm);
    }

    int numberOfLandmarks(){
        return landmarks.size();
    }

};

struct Gaussian{
    VectorXf mu;
    MatrixXf sigma;
};

std::vector<std::string> splitBy(std::string str, std::string delimiter);

WorldData read_world(std::string filename);

RobotData read_data(std::string filename);

float normalize_angle(float phi);

MatrixXf normalize_angle(MatrixXf mat);

void normalize_all_bearings(VectorXf &z);

MatrixXf vect_atan2(const MatrixXf &y, const MatrixXf & x);

std::pair<bool, int > findInVector(const std::vector<int>  & vecOfElements, int  & element);

MatrixXf compute_sigma_points(VectorXf mu, MatrixXf sigma);

Gaussian recoverGaussian(MatrixXf sigma_points);

void add_landmark_to_map(VectorXf &mu, MatrixXf &sigma, Sensor z, std::vector<int> &map, Matrix2f Q);

void prediction_step(VectorXf &mu, MatrixXf &sigma, Odometry u);

void correction_step(VectorXf &mu, MatrixXf &sigma, std::vector<Sensor> z, std::vector<int> &map);


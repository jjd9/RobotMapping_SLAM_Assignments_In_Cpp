#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "../../eigen/Eigen/Dense"

#include "tools.h"

using namespace Eigen;


std::vector<std::string> splitBy(std::string str, std::string delimiter){
    std::size_t pos = str.find(delimiter);
    std::vector<std::string> vec;
    if(pos == std::string::npos){
        vec.push_back(str);
        return vec;
    }
    std::size_t lastPos = 0;
    while(pos != std::string::npos){
        vec.push_back(str.substr(lastPos, pos-lastPos));
        lastPos = pos+1;
        pos = str.find(delimiter, lastPos);
    }
    vec.push_back(str.substr(lastPos, str.length() - lastPos));

    return vec;
}


// TO DO
std::vector<LaserReading> read_robotlaser(std::string filename){
    // read a file containing ROBOTLASER1 in CARMEN logfile format

    std::vector<LaserReading> laser;

    std::ifstream fid{filename};
    std::string line;

    if(!fid.is_open()){
        std::cout << "Unable to open file" << std::endl;
        return laser;
    }

    while(!fid.eof()){
        std::getline(fid, line);
        std::vector<std::string> tokens = splitBy(line, " ");

        if (tokens[0] != "ROBOTLASER1"){
            continue;
        }

        std::vector<float> num_tokens;
        int pass;
        for(int i = 2; i < tokens.size(); i++){
            try{
                num_tokens.push_back(std::stof(tokens[i]));
            }catch(std::exception ex){
                pass = 1;
            }
        }

        LaserReading currentReading;

        int tk = 0;
        currentReading.start_angle = num_tokens[tk++];
        tk++; // skip FOV
        currentReading.angular_resolution = num_tokens[tk++];
        currentReading.maximum_range = num_tokens[tk++];
        tk += 2; // skip accuracy, remission_mode

        int num_readings = num_tokens[tk++];
        currentReading.ranges = VectorXf::Zero(num_readings);
        for(int j = tk; j < tk+num_readings; ++j){
            currentReading.ranges(j-tk) = num_tokens[j];
        }
        tk += num_readings;

        // skip reading the remission values
        int num_remissions = num_tokens[tk++]; 
        tk += num_remissions;

        Vector3f laser_pose;
        laser_pose << num_tokens[tk], num_tokens[tk+1], num_tokens[tk+2];
        tk += 3;
        Vector3f pose;
        pose << num_tokens[tk], num_tokens[tk+1], num_tokens[tk+2];
        currentReading.pose = pose;
        tk += 3;

        currentReading.laser_offset = t2v(v2t(currentReading.pose).inverse() * v2t(laser_pose));

        tk += 5; // skip tv, rv, forward, side, turn
        currentReading.timestamp = num_tokens[tk++];

        laser.push_back(currentReading);

    }

    fid.close();

    return laser;
}
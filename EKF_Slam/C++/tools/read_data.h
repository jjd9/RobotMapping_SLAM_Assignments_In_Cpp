#pragma once

#include <iostream>
#include <fstream>
#include <string>

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

WorldData read_world(std::string filename){
    WorldData worldData; 

    std::ifstream input(filename);
    if(!input.is_open()){
        std::cout << "Failed to open data file..." <<std::endl;
    }else{
        std::string line;
        while(!input.eof()){
            std::getline(input, line);
            std::vector<std::string> vec = splitBy(line," ");

            if(vec.size() == 3){
                Landmark lm;
                lm.id=std::stoi(vec[0])-1;
                lm.x=std::stof(vec[1]);
                lm.y=std::stof(vec[2]);
                worldData.addLandmark(lm);
            }else{
                std::cout << "Not a valid line...";
            }

        }
        input.close();
    }
    return worldData;
}



RobotData read_data(std::string filename){
    /*
     Reads the odometry and sensor readings from a file.
    
     filename: path to the file to parse
     data: structure containing the parsed information
    
     The data is returned in a structure where the u_t and z_t are stored
     within a single entry. A z_t can contain observations of multiple
     landmarks.
    
     Usage:
     - access the readings for timestep i:
       data.timestep(i)
       this returns a structure containing the odometry reading and all
       landmark obsevations, which can be accessed as follows
     - odometry reading at timestep i:
       data.timestep(i).odometry
     - senor reading at timestep i:
       data.timestep(i).sensor
    
     Odometry readings have the following fields:
     - r1 : rotation 1
     - t  : translation
     - r2 : rotation 2
     which correspond to the identically labeled variables in the motion
     mode.
    
     Sensor readings can again be indexed and each of the entris has the
     following fields:
     - id      : id of the observed landmark
     - range   : measured range to the landmark
     - bearing : measured angle to the landmark (you can ignore this)
    
     Examples:
     - Translational component of the odometry reading at timestep 10
       data.timestep(10).odometry.t
     - Measured range to the second landmark observed at timestep 4
       data.timestep(4).sensor(2).range
       */

    RobotData robotData; 

    std::ifstream input(filename);
    if(!input.is_open()){
        std::cout << "Failed to open data file..." <<std::endl;
    }else{
        std::string line;
        bool first = true;
        while(!input.eof()){
            std::getline(input, line);
            std::vector<std::string> vec = splitBy(line," ");
            std::string readingType = vec[0];
            if(readingType == "ODOMETRY"){
                first = true;
                Odometry od;
                od.r1 = std::stof(vec[1]);
                od.t = std::stof(vec[2]);
                od.r2 = std::stof(vec[3]);
                robotData.addOdometryReading(od);
            }else if (readingType=="SENSOR"){                
                Sensor sense;
                sense.id = std::stoi(vec[1])-1;
                sense.range = std::stof(vec[2]);
                sense.bearing = std::stof(vec[3]);
                robotData.addSensorReading(sense, first);
                first = false;
            }else{
                std::cout << "Not a valid line...";
            }

        }
        input.close();
    }
 
    return robotData;

}
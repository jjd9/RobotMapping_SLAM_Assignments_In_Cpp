#include "tools.h"


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
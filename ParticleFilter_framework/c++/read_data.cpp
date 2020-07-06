#include "tools.h"

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


MatrixXf read_data(std::string filename){
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

    std::ifstream input{filename};
    std::string line;
    // count rows in file
    int rows = 331; //std::count(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>(), "\n");
    MatrixXf odometry(rows, 3);

    for(int i = 0; i < rows; ++i){
        std::getline(input, line);
        std::vector<std::string> arr = splitBy(line, " ");
        std::string type = arr[0];

        odometry(i, 0) = std::stof(arr[1]);
        odometry(i, 1) = std::stof(arr[2]);
        odometry(i, 2) = std::stof(arr[3]);
      }

    input.close();
  return odometry;
}

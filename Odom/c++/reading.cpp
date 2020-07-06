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

MatrixXf read_odom(std::string filename){
  MatrixXf odom_motions = MatrixXf::Zero(2000,3);
  std::ifstream input_odom{filename};
  if(input_odom.is_open()){
    std::string line;
    // skip first 5 lines
    for(int i = 0; i < 5; i++){
      std::getline(input_odom, line);
    }
    int t = 0;
    while(!input_odom.eof()){
      std::getline(input_odom, line);
      std::vector<std::string> parts = splitBy(line," ");
      for(int i = 1; i < parts.size(); ++i){
        odom_motions(t, i-1) = std::stof(parts[i]);
      }
      ++t;
    }

  }else{
    std::cout << "File cannot be opened" << std::endl;
  }
    return odom_motions;
}

MatrixXf read_scans(std::string filename){

    MatrixXf scanmatched_motions = MatrixXf::Zero(2000,3); 

  std::ifstream input_scan{filename};

  if(input_scan.is_open()){
    std::string line;
    // skip first 5 lines
    for(int i = 0; i < 5; i++){
      std::getline(input_scan, line);
    }
    int t = 0;
    while(!input_scan.eof()){
      std::getline(input_scan, line);
      std::vector<std::string> parts = splitBy(line," ");
      for(int i = 1; i < parts.size(); ++i){
        scanmatched_motions(t, i-1) = std::stof(parts[i]);
      }
      ++t;
    }

  }else{
    std::cout << "File cannot be opened";
  }
    return scanmatched_motions;
}
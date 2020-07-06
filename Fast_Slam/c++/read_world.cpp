#include "tools.h"

std::vector<Landmark> read_world(std::string filename){

    std::vector<Landmark> landmarks;

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
                landmarks.push_back(lm);
            }else{
                std::cout << "Not a valid line...";
            }

        }
        input.close();
    }
    return landmarks;
}

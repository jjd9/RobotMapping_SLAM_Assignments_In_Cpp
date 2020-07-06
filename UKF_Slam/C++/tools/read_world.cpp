#include "tools.h"


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

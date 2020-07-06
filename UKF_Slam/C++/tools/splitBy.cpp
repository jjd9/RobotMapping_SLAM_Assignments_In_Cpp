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

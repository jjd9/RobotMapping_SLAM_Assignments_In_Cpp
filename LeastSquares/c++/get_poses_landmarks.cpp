#include "tools.h"

void get_poses_landmarks(Graph &g, std::vector<int> &poses, std::vector<int> &landmarks){
  // extract the offset of the poses and the landmarks

  for (auto entry : g.idLookup){
    int dim = entry.second.dimension;
    int offset = entry.second.offset;
    if (dim == 3){
      poses.push_back(offset);
    }else if (dim == 2){
      landmarks.push_back(offset);
    }
  }
}
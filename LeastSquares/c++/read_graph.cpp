#include "tools.h"

Graph read_graph(std::string baseFileName){
  // reads graph data files
  // assumes the following extensions from the base file name exist:
  // <baseFileName>_x.csv
  // <baseFileName>_edge.csv
  // <baseFileName>_idLookup.csv

  Graph g;

  std::string line;

  std::fstream x_input{baseFileName + "_x.csv"};
  std::vector<float> x_values;
  if(x_input.is_open()){
    while(!x_input.eof()){
      std::getline(x_input, line);
      if(line != ""){
        x_values.push_back(std::stof(line));
      }
    }
  }else{
    std::cout << "Unable to open file: " << baseFileName + "_x.csv" << std::endl;
  }

  x_input.close();
  g.x = Map<VectorXf, Unaligned>(x_values.data(), x_values.size());

  std::fstream edge_input{baseFileName + "_edge.csv"};
  if(edge_input.is_open()){
    std::getline(edge_input, line); // skip file header
    while(!edge_input.eof()){
      std::getline(edge_input, line);
      if (line == ""){
        continue;
      }
      std::vector<std::string> cells = splitBy(line, ",");
      Edge edge;
      edge.type = std::stoi(cells[0]);
      edge.to = std::stoi(cells[1]);
      edge.from = std::stoi(cells[2]);
      if(edge.type == 80){ // Pose
        edge.measurement = Vector3f::Zero();
        edge.information = Matrix3f::Zero();
        for(int i = 0; 3+i <= 5; ++i){
          edge.measurement(i) = std::stof(cells[3+i]);
        }
        for(int i = 0; 6+i <= 14; ++i){
          int r= i/3;
          int c= i%3;
          edge.information(r,c) = std::stof(cells[6+i]);
        }
        edge.information.transposeInPlace();
      }else if(edge.type == 76){ // Landmark
        edge.measurement = Vector2f::Zero();
        edge.information = Matrix2f::Zero();
        for(int i = 0; 3+i <= 4; ++i){
          edge.measurement(i) = std::stof(cells[3+i]);
        }
        for(int i = 0; 6+i <= 9; ++i){
          int r= i/2;
          int c= i%2;
          edge.information(r,c) = std::stof(cells[6+i]);
        }
        edge.information.transposeInPlace();
      }
      edge.fromIdx = std::stoi(cells[15])-1;
      edge.toIdx = std::stoi(cells[16])-1;

      g.edges.push_back(edge);
    }
  }else{
    std::cout << "Unable to open file: " << baseFileName + "_edge.csv" << std::endl;
  }
  edge_input.close();

  std::fstream id_input{baseFileName + "_idLookup.csv"};
  if(id_input.is_open()){
    std::getline(id_input, line); // skip file header
    while(!id_input.eof()){
      std::getline(id_input, line);
      if (line == ""){
        continue;
      }
      std::vector<std::string> cells = splitBy(line, ",");
      int id = std::stoi(cells[0]);
      Entry entry;
      entry.offset = std::stoi(cells[1]);
      entry.dimension = std::stoi(cells[2]);
      g.idLookup.insert({id, entry});
    }
  }else{
    std::cout << "Unable to open file: " << baseFileName + "_idLookup.csv" << std::endl;
  }
  id_input.close();


  return g;

}


#include "tools.h"

// calculates the number of non-zeros of a graph
// Actually, it is an upper bound, as duplicate edges might be counted several times

int nnz_of_graph(Graph &g){

  int nnz = 0;

  // elements along the diagonal
  for (auto entry : g.idLookup){
    nnz += std::pow(entry.second.dimension,2);
  }

  // off-diagonal elements
  for (auto edge : g.edges){
    if (edge.type == 80){
      nnz += 2 * 9;
    }else if (edge.type == 76){
      nnz += 2 * 6;
    }
  }
  
  return nnz;
}

#include "tools.h"

int main(int argc, char** argv){
  // load the graph into the variable g
  // only leave one line uncommented

  Graph g = read_graph(argv[1]);

  // simulation datasets
  // Graph g = read_graph("data/simulation-pose-pose");
  // Graph g = read_graph("data/simulation-pose-landmark");

  // real-world datasets
  // Graph g = read_graph("data/intel");
  // Graph g = read_graph("data/dlr");

  // plot the initial state of the graph
  // plot_graph(g, 0);

  std::printf("Initial error %f\n", compute_global_error(g));

  // the number of iterations
  int numIterations = 100;

  // maximum allowed dx
  float EPSILON = 1e-3;

  // Error
  float err = 0;

  // carry out the iterations
  for(int i = 0; i < numIterations; ++i){
    std::printf("Performing iteration %d\n", i);

    VectorXf dx = linearize_and_solve(g);

    // apply the solution to the state vector g.x
    g.x += dx;

    err = compute_global_error(g);

    // Print current error
    std::printf("Current error %f\n", err);

    // implement termination criterion as suggested on the sheet
    if(dx.lpNorm<Infinity>() < EPSILON){
      break;
    }

  }

  std::printf("Final error %f\n", err);

  // print results to file
  std::vector<int> poses;
  std::vector<int> landmarks;
  get_poses_landmarks(g, poses, landmarks);  

  std::ofstream outputPose{"PoseData.csv"};
  
  for(int p : poses){
    outputPose << g.x(p) << "," << g.x(p+1) << "\n";
  }
  outputPose.close();

  std::ofstream outputLandmarks{"LandmarkData.csv"};

  for(int lm : poses){
    outputLandmarks << g.x(lm) << "," << g.x(lm+1) << "\n";
  }
  outputLandmarks.close();
  
  return 0;
}
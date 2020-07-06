#include <iostream>
#include <fstream>

#include "inv_sensor_model.h"

int main(){

    using namespace Eigen;

    // Read laser data
    std::vector<LaserReading> laser = read_robotlaser("data/csail.log");

    // Extract robot poses: Nx3 matrix where each row is in the form: [x y theta]
    int N = laser.size();
    MatrixXf poses(N,3);
    for(int i = 0; i < N; i++){
        poses.row(i) = laser[i].pose.transpose();
    }

    // Initial cell occupancy probability.
    float prior = 0.50;
    // Probabilities related to the laser range finder sensor model.
    float probOcc = 0.9;
    float probFree = 0.35;

    // Map grid size in meters. Decrease for better resolution.
    float gridSize = 0.1;

    // Set up map boundaries and initialize map.
    float border = 30;
    Vector3f min = poses.colwise().minCoeff();
    Vector3f max = poses.colwise().maxCoeff();
    
    float robXMin = min(0);
    float robXMax = max(0);
    float robYMin = min(1);
    float robYMax = max(1);
    std::vector<float> mapBox{robXMin-border, robXMax+border, robYMin-border, robYMax+border};
    float offsetX = mapBox[0];
    float offsetY = mapBox[2];
    std::vector<float> mapSizeMeters{mapBox[1]-offsetX, mapBox[3]-offsetY};
    int xdim = std::ceil(mapSizeMeters[0]/gridSize);
    int ydim = std::ceil(mapSizeMeters[1]/gridSize);
    std::vector<int> mapSize{xdim, ydim};

    // Used when updating the map. Assumes that prob_to_log_odds.m
    // has been implemented correctly.
    float logOddsPrior = prob_to_log_odds(prior);

    // The occupancy value of each cell in the map is initialized with the prior.
    MatrixXf map = logOddsPrior*MatrixXf::Ones(mapSize[0],mapSize[1]);
    std::cout << "Map initialized. Map size: " << map.rows() << "," << map.cols() << std::endl;


    // Map offset used when converting from world to map coordinates.
    std::vector<float> offset{offsetX, offsetY};

    VectorXf robPoseMapFrame;
    MatrixXf laserEndPntsMapFrame;

    // Main loop for updating map cells.
    // You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
    std::cout << "Time steps: " << N << std::endl;
    for(int t=0; t < N; ++t){
        // Robot pose at time t.
        Vector3f robPose = poses.row(t);

        // Laser scan made at time t.
        LaserReading sc = laser[t];

        // Compute the mapUpdate, which contains the log odds values to add to the map.
        MatrixXf mapUpdate = inv_sensor_model(map, sc, robPose, gridSize, offset, probOcc, probFree, robPoseMapFrame, laserEndPntsMapFrame);

        // Update the occupancy values of the affected cells.
        map += mapUpdate - logOddsPrior*MatrixXf::Ones(mapSize[0],mapSize[1]);
        
        // Plot current map and robot trajectory so far.
        // plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t);
    }

    MatrixXf probMap = log_odds_to_prob(map).array();

    // output map to file
    const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
    std::ofstream output{"GridMap.csv"};
    output << probMap.format(CSVFormat) << "\n";
    output.close();

    MatrixXf framePoses = world_to_map_coordinates(poses.block(0,0,N,2).transpose(),gridSize,offset).transpose();
    std::ofstream output2{"RobotPath.csv"};
    output2 << framePoses.format(CSVFormat) << "\n";
    output2.close();


    return 0;
}
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include "../../eigen/Eigen/Dense"
#include "prediction_step.h"
#include "correction_step.h"

using namespace Eigen;


int main(){

    std::cout << "Read files" << std::endl;
    // Read world data, i.e. landmarks. The true landmark positions are not given to the robot
    std::string world_filename = "data/world.dat";
    WorldData worldData = read_world(world_filename);

    // load sensor and odometry data;
    std::string data_filename = "data/sensor_data.dat";
    RobotData data = read_data(data_filename);

    float INF = 1000.0;

    // Get the number of landmarks in the map
    int N = worldData.numberOfLandmarks();

    // observedLandmarks is a vector that keeps track of which landmarks have been observed so far.
    // observedLandmarks(i) will be true if the landmark with id = i has been observed at some point by the robot
    std::vector<bool> observedLandmarks(false, N);

    /*
    Initialize belief:
    mu: 2N+3x1 vector representing the mean of the normal distribution
    The first 3 components of mu correspond to the pose of the robot,
    and the landmark poses (xi, yi) are stacked in ascending id order.
    sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
    */
    std::cout << "Initialize" << std::endl;
    VectorXf mu = VectorXf::Zero(2*N+3);

    //  sigma = [[robSigma robMapSigma];[robMapSigma' mapSigma]];
    MatrixXf sigma = MatrixXf::Zero((2*N+3),(2*N+3));
    sigma.block(3,3,2*N,2*N) = INF*MatrixXf::Identity(2*N, 2*N);

    // toogle the visualization type
    // showGui = true;  // show a window while the algorithm runs
    bool showGui = false; // plot to files instead

    // use this for writing to file
    std::ofstream output{"SLAM_estimates.csv"};
    const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

    // Perform filter update for each odometry-observation pair read from the
    // data file.
    for (int t = 0; t < data.timesteps; ++t){
        // Perform the prediction step of the EKF
        prediction_step(mu, sigma, data.odometryData[t]);

        // Perform the correction step of the EKF
        correction_step(mu, sigma, data.sensorData[t], observedLandmarks);

        //Generate visualization plots of the current state of the filter
        // plot_state(mu, sigma, landmarks, t, observedLandmarks, data.timestep(t).sensor, showGui);
        std::cout << "Current state vector:" << std::endl;
        std::cout << "mu = " << mu.transpose() << std::endl;
        output << mu.transpose().format(CSVFormat) << "\n";
    }

    // std::cout << "Final system covariance matrix:\n" << sigma << std::endl;
    // Display the final state estimate
    std::cout << "Final robot pose:" << std::endl;
    std::cout << "mu_robot = \n" << mu.block(0,0,3,1) << "\nsigma_robot = \n" << sigma.block(0,0,3,3) << std::endl;

    output.close();

    return 0;
}


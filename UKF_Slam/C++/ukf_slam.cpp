#include "tools/tools.h"

// This is the main unscented Kalman filter SLAM loop. This script calls all the required
// functions in the correct order.
//
// You can disable the plotting or change the number of steps the filter
// runs for to ease the debugging. You should however not change the order
// or calls of any of the other lines, as it might break the framework.
//
// If you are unsure about the input and return values of functions you
// should read their documentation which tells you the expected dimensions.

float scale, pi;


int main(){
    // set global variables scale and pi
    scale = 3.0;
    pi = 3.141592653589793;

    std::cout << "Read files" << std::endl;
    // Read world data, i.e. landmarks. The true landmark positions are not given to the robot
    std::string world_filename = "data/world.dat";
    WorldData worldData = read_world(world_filename);

    // load sensor and odometry data;
    std::string data_filename = "data/sensor_data.dat";
    RobotData data = read_data(data_filename);

    // load data
    // Initialize belief
    VectorXf mu = VectorXf::Zero(3);
    MatrixXf sigma = 0.001*MatrixXf::Identity(3,3);
    std::vector<int> map;

    // Perform filter update for each odometry-observation pair read from the
    // data file.
    std::ofstream output{"SLAM_estimates.csv"};
    std::string landmarkNames = "";
    for(int i = 0; i < 9; ++i){
        landmarkNames += ",lmx_"+std::to_string(i)+",lmx_"+std::to_string(i);
    }
    // file header
    output << "x,y,theta"+landmarkNames << "\n";

    const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
    for (int t = 0; t < data.timesteps; ++t){
        // std::printf("Time step t = %d \n", t);

        // Perform the prediction step of the UKF
        prediction_step(mu, sigma, data.odometryData[t]);

        // Perform the correction step of the UKF
        correction_step(mu, sigma, data.sensorData[t], map);

        // std::cout << "mu=\n"<< mu.transpose() << std::endl;

        output << mu.transpose().format(CSVFormat) << "\n";

    }

    // Display the final state estimate
    std::cout << "Final robot pose:" << std::endl;
    std::cout << mu.transpose() << std::endl;
    std::cout << "Final system covariance matrix:" << std::endl;
    std::cout << sigma << std::endl;

    output.close();

    return 0;
}
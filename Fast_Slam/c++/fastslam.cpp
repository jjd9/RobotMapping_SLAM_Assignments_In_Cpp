#include <iostream>

#include "tools.h"

/*
 This is the main FastSLAM loop. This script calls all the required
 functions in the correct order.

 You can disable the plotting or change the number of steps the filter
 runs for to ease the debugging. You should however not change the order
 or calls of any of the other lines, as it might break the framework.

 If you are unsure about the input and return values of functions you
 should read their documentation which tells you the expected dimensions.
*/

std::random_device r;
std::default_random_engine eng(r());

int main(){

  // Read world data, i.e. landmarks. The true landmark positions are not given to the robot
  std::vector<Landmark> landmarks = read_world("data/world.dat");
  // Read sensor readings, i.e. odometry and range-bearing sensor
  RobotData data = read_data("data/sensor_data.dat");

  // Get the number of landmarks in the map
  int N = landmarks.size();

  Vector3f noise;
  noise << 0.005f, 0.01f, 0.005f;

  // how many particles
  int numParticles = 100;

  // initialize the particles array
  std::vector<Particle> particles;
  for(int i = 0; i < numParticles; ++i){
    Particle p;
    p.weight = 1.0 / numParticles;
    p.pose = Vector3f::Zero();
    for(int l = 0; l < N; ++l){ // initialize the landmarks aka the map
      LandmarkEstimate lm;
      lm.observed = false;
      lm.mu = Vector2f::Zero();    // 2D position of the landmark
      lm.sigma = Matrix2f::Zero(); // covariance of the landmark
      p.landmarks.push_back(lm);
    }
    particles.push_back(p);
  }

  // Perform filter update for each odometry-observation pair read from the
  // data file.
  for (int t = 0; t < data.timesteps; ++t){ 

      // Perform the prediction step of the particle filter
      prediction_step(particles, data.odometryData[t], noise, eng);

      // Perform the correction step of the particle filter
      correction_step(particles, data.sensorData[t]);

      // Resample the particle set
      resample(particles, eng);
  }

// print map data to csv file
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
std::ofstream output{"Particles.csv"};

for (const auto &p : particles){
  int id = 0;
  for (const auto &lm : p.landmarks){
    if(lm.observed){
      output << std::to_string(id) << "," << lm.mu.transpose().format(CSVFormat) << "\n";
    }
    ++id;
  }
}

// print path data to csv file
std::ofstream output_path{"Path.csv"};
int id = 0;
for (const auto &p : particles){
  for (const auto &pose : p.history){
    output_path << std::to_string(id) << "," << pose.transpose().format(CSVFormat) << "\n";
  }
  ++id;
}

output_path.close();

  return 0;
}

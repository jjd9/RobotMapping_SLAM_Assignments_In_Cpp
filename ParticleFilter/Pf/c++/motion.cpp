#include "tools.h"


std::random_device r;
std::default_random_engine eng(r());


int main(){
  // Read sensor readings, i.e. odometry
  MatrixXf odometry = read_data("data/odometry.dat");

  Vector3f noise;
  noise << 0.005f, 0.01f, 0.005f;

  // how many particles
  int numParticles = 100;

  std::cout << "Data: " <<std::endl;
  std::cout << odometry.rows() << "," << odometry.cols() <<std::endl;

  // initialize the particles array
  std::vector<Particle> particles;
  for (int i = 0; i < numParticles; ++i){
    Particle p;
    p.weight = 1.0 / numParticles;
    p.pose = Vector3f::Zero();
    particles.push_back(p);
  }

  // Perform filter update for each odometry-observation read from the
  // data file.

  const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

  for (int t = 0; t < odometry.rows();++t){
    // std::printf("timestep = %d\n", t);
    // Perform the prediction step of the particle filter
    particles = prediction_step(particles, odometry.row(t), noise, eng);

    std::ofstream output{"motionOutput/csv/particles_t="+std::to_string(t)+".csv"};
    for (const auto &p :particles){
      output << p.pose.transpose().format(CSVFormat) << "\n";
    }
    output.close();

    // Generate visualization plots of the current state of the filter
    // plot_state(particles, t);
  }

  return 0;
}
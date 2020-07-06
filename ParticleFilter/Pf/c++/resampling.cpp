#include "tools.h"

std::random_device r;
std::default_random_engine eng(r());

int main(){
  // how many particles
  int numParticles = 1000;

  std::normal_distribution<float> norm_dist_x(0,1);
  std::normal_distribution<float> norm_dist_y(0,2);


  // initialize the particles array
  std::vector<Particle> particles;
  for (int i = 0; i < numParticles; ++i){
    Particle particle;
    particle.weight = 1.0 / numParticles;
    particle.pose(0) = norm_dist_x(eng);
    particle.pose(1) = norm_dist_y(eng);
    particles.push_back(particle);
  }

  // re-weight the particles according to their distance to [0 0]
  Matrix2f sigma = Matrix2f::Zero();
  sigma(0,0)=0.2;
  sigma(1,1)=0.2;
  Matrix2f inv_sigma = sigma.inverse();
  for (int i = 0; i < numParticles; ++i){
    Vector2f pose = particles[i].pose.block(0,0,2,1);
    particles[i].weight = std::exp(-0.5 * pose.transpose() * inv_sigma * pose);
  }

  auto resampledParticles = resample(particles, eng);

  // plot the particles before (red) and after resampling (blue)
  MatrixXf bpos(numParticles, 2);
  MatrixXf apos(numParticles, 2);
  for (int i = 0; i < numParticles; ++i){
    bpos.row(i) = particles[i].pose.block(0,0,2,1).transpose();
    apos.row(i) = resampledParticles[i].pose.block(0,0,2,1).transpose();
  }

  const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

  std::ofstream output_b{"resamplingOutput/bpos.csv"};
  std::ofstream output_a{"resamplingOutput/apos.csv"};
  for (int i = 0; i < numParticles; ++i){
    output_b << bpos.row(i).format(CSVFormat) << "\n";
    output_a << apos.row(i).format(CSVFormat) << "\n";
  }
  output_b.close();
  output_a.close();


  // plot(bpos(1,:), bpos(2,:), 'r+', 'markersize', 5, apos(1,:), apos(2,:), 'b*', 'markersize', 5);
  return 0;
}
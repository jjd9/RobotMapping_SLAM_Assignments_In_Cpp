#include "tools.h"

/*
Updates the particles by drawing from the motion model
Use u.r1, u.t, and u.r2 to access the rotation and translation values
which have to be pertubated with Gaussian noise.
The position of the i-th particle is given by the 3D vector
particles(i).pose which represents (x, y, theta).
*/
std::vector<Particle> prediction_step(std::vector<Particle> particles, MatrixXf u, Vector3f noise, std::default_random_engine eng){

  // noise parameters
  // Assume Gaussian noise in each of the three parameters of the motion model.
  // These three parameters may be used as standard deviations for sampling.

  std::normal_distribution<float> r1Noise(0,noise(0));
  std::normal_distribution<float> transNoise(0, noise(1));
  std::normal_distribution<float> r2Noise(0,noise(2));


  float r1;
  float trans;
  float r2;

  int numParticles = particles.size();

  for (auto & p : particles){

    // append the old position to the history of the particle
    p.history.push_back(p.pose);

    r1 =    u(0) + r1Noise(eng);
    trans = u(1) + transNoise(eng);
    r2 =    u(2) + r2Noise(eng);

    // TODO: sample a new pose for the particle
    p.pose(0) += trans*std::cos(p.pose(2)+r1);
    p.pose(1) += trans*std::sin(p.pose(2)+r1);
    p.pose(2) = normalize_angle(p.pose(2) + r1 + r2);
	
    
  }
  return particles;

}
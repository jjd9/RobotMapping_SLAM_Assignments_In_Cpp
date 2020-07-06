#include "tools.h"

/*
resample the set of particles.
A particle has a probability proportional to its weight to get
selected. A good option for such a resampling method is the so-called low
variance sampling, Probabilistic Robotics pg. 109
*/
std::vector<Particle> resample(std::vector<Particle> particles, std::default_random_engine eng){

  int numParticles = particles.size();

  VectorXf w(numParticles);
  for(int i = 0; i < numParticles; ++i){
    w(i) = particles[i].weight;
  }

  // normalize the weight
  float wSum = w.sum();
  if(wSum != 1.0){
    w = w.array() / w.sum();
    for(int i = 0; i < numParticles; ++i){
      particles[i].weight = w(i);
    }    
  }

  // consider number of effective particles, to decide whether to resample or not
  bool use_n_eff = true;
  if (use_n_eff){
    float n_eff = (1.0 / (w.array().square()).sum());
    if(n_eff > 0.5*numParticles){
      return particles;
    }
  }

  std::vector<Particle> newParticles;

  // TODO: implement the low variance re-sampling
  std::vector<float> C;
  C.push_back(0);
  for(int i = 0; i < numParticles; ++i){
    C.push_back(C[i] + w(i));
  }

  std::uniform_real_distribution<float> uniform_dist(0, 1.0);
  float u0 = uniform_dist(eng);
  int j = 0;
  for (int i = 0; i < numParticles; ++i){
      float u = (u0 + i) / numParticles;
      while (u > C[j]){
          ++j;
      }
      newParticles.push_back(particles[j-1]);
      newParticles[newParticles.size()-1].weight = 1.0/numParticles;
  }

  return newParticles;

}
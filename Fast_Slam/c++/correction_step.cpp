#include "tools.h"


void correction_step(std::vector<Particle> &particles, std::vector<Sensor> z){

  /*
  Weight the particles according to the current map of the particle
  and the landmark observations z.
  z: struct array containing the landmark observations.
  Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
  The vector observedLandmarks indicates which landmarks have been observed
  at some point by the robot.
  */

  float PI = 3.14159;

  // Number of particles
  int numParticles = particles.size();

  // Number of measurements in this time step
  int m = z.size();

  // Construct the sensor noise matrix Q_t (2 x 2)
  Matrix2f Q_t = Matrix2f::Zero();
  Q_t(0,0) = 0.1;
  Q_t(1,1) = 0.1;

  // process each particle
  for(int i = 0; i < numParticles; ++i){
    // process each measurement
    for(int j = 0; j < m; ++j){
      // Get the id of the landmark corresponding to the j-th observation
      // particles(i).landmarks(l) is the EKF for this landmark
      int l = z[j].id;

      // The (2x2) EKF of the landmark is given by
      // its mean particles(i).landmarks(l).mu
      // and by its covariance particles(i).landmarks(l).sigma

      // If the landmark is observed for the first time:
      if (particles[i].landmarks[l].observed == false){

        // Initialize its position based on the measurement and the current robot pose:
        float theta = particles[i].pose(2);
        particles[i].landmarks[l].mu(0) = particles[i].pose(0) + z[j].range*std::cos(theta+z[j].bearing);
        particles[i].landmarks[l].mu(1) = particles[i].pose(1) + z[j].range*std::sin(theta+z[j].bearing);

        // get the Jacobian with respect to the landmark position
        MeasurementPrediction z_pred = measurement_model(particles[i], z[j]);

        // initialize the EKF for this landmark
        Matrix2f Hinv = z_pred.H.inverse();
        particles[i].landmarks[l].sigma = Hinv*Q_t*Hinv.transpose();

        // Indicate that this landmark has been observed
        particles[i].landmarks[l].observed = true;

      }else{

        // get the expected measurement
        MeasurementPrediction z_pred = measurement_model(particles[i], z[j]);

        // compute the measurement covariance
        Matrix2f Q = z_pred.H*(particles[i].landmarks[l].sigma)*z_pred.H.transpose() + Q_t;
        // calculate the Kalman gain
        Matrix2f K = (particles[i].landmarks[l].sigma)*z_pred.H.transpose()*Q.inverse();
        // compute the error between the z and expectedZ (remember to normalize the angle)
        Vector2f innov;
        innov(0) = z[j].range - z_pred.h(0);
        innov(1) = z[j].bearing - z_pred.h(1);

        particles[i].landmarks[l].mu += K*innov;
        // update the mean and covariance of the EKF for this landmark
        particles[i].landmarks[l].sigma = (Matrix2f::Identity() - K*z_pred.H)*particles[i].landmarks[l].sigma;
        
        // compute the likelihood of this observation, multiply with the former weight
        //       to account for observing several features in one time step
        particles[i].weight = (1.0/std::sqrt((2*PI*Q).determinant()))*std::exp(-0.5*innov.transpose()*Q.inverse()*innov);

      }

    } // measurement loop
  } // particle loop

}

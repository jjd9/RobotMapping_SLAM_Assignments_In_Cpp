#include "tools.h"

void prediction_step(VectorXf &mu, MatrixXf &sigma, Odometry u){

     // Updates the belief concerning the robot pose according to the motion model.
     // mu: state vector containing robot pose and poses of landmarks obeserved so far
     // Current robot pose = mu(1:3)
     // Note that the landmark poses in mu are stacked in the order by which they were observed
     // sigma: the covariance matrix of the system.
     // u: odometry reading (r1, t, r2)
     // Use u.r1, u.t, and u.r2 to access the rotation and translation values

     // Compute sigma points
     MatrixXf sigma_points = compute_sigma_points(mu, sigma);

     // TODO: Transform all sigma points according to the odometry command
     // Remember to vectorize your operations and normalize angles
     // Tip: the function normalize_angle also works on a vector (row) of angles

    sigma_points.row(0) = sigma_points.row(0).array() + u.t*(sigma_points.row(2).array()+u.r1).cos();
    sigma_points.row(1) = sigma_points.row(1).array() + u.t*(sigma_points.row(2).array()+u.r1).sin();
    sigma_points.row(2) = sigma_points.row(2).array() + u.r1+u.r2;
     sigma_points.row(2) = normalize_angle(sigma_points.row(2));

     // TODO: recover mu and sigma.
     // Be careful when computing the robot's orientation (sum up the sines and
     // cosines and recover the 'average' angle via atan2)
     Gaussian gauss = recoverGaussian(sigma_points);
     mu = gauss.mu;
     sigma = gauss.sigma;

     // Motion noise
     float motionNoise = 0.1;
     Matrix3f R3;
     R3 << motionNoise, 0, 0, 
          0, motionNoise, 0,
          0, 0, motionNoise/10.0;
     MatrixXf R = MatrixXf::Zero(sigma.rows(),sigma.cols());
     R.block(0,0,3,3) = R3;

     // TODO: Add motion noise to sigma
     sigma = sigma + R;

}
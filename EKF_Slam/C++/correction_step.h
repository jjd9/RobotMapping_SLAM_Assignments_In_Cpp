#pragma once

#include "eigen/Eigen/Dense"

#include "tools/read_data.h"
#include "tools/normalize_all_bearings.h"


using namespace Eigen;

void correction_step(VectorXf &mu, MatrixXf &sigma, std::vector<Sensor> z, std::vector<bool> observedLandmarks){
    /*
    Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
    The employed sensor model measures the range and bearing of a landmark
    mu: 2N+3 x 1 vector representing the state mean.
    The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
    The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
    sigma: 2N+3 x 2N+3 is the covariance matrix
    z: struct array containing the landmark observations.
    Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
    The vector observedLandmarks indicates which landmarks have been observed
    at some point by the robot.
    observedLandmarks(j) is false if the landmark with id = j has never been observed before.
    */

    // Number of measurements in this time step
    int m = z.size();

    // Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
    // ExpectedZ: vectorized form of all expected measurements in the same form.
    // They are initialized here and should be filled out in the for loop below
    VectorXf Z(m*2);
    VectorXf expectedZ(m*2);

    // Iterate over the measurements and compute the H matrix
    // (stacked Jacobian blocks of the measurement function)
    // H will be 2m x 2N+3
    MatrixXf H(2*m, mu.size());
    for(int i = 0; i < m; ++i){
        // Get the id of the landmark corresponding to the i-th observation
        int landmarkId = z[i].id;

        int landmark_x_idx = 3 + landmarkId*2;
        int landmark_y_idx = 3 + landmarkId*2 + 1;

        // If the landmark is obeserved for the first time:
        if(observedLandmarks[landmarkId]==false){
            // Initialize its pose in mu based on the measurement and the current robot pose:
            mu(landmark_x_idx) = mu(0) + z[i].range*std::cos(mu(2)+z[i].bearing);
            mu(landmark_y_idx) = mu(1) + z[i].range*std::sin(mu(2)+z[i].bearing);
            // Indicate in the observedLandmarks vector that this landmark has been observed
            observedLandmarks[landmarkId] = true;
        }

        // Add the landmark measurement to the Z vector
        Z(2*i) = z[i].range;
        Z(2*i+1) = z[i].bearing;
        
        // Use the current estimate of the landmark pose
        // to compute the corresponding expected measurement in expectedZ:
        float dx = z[i].range*std::cos(mu(2)+z[i].bearing);
        float dy = z[i].range*std::sin(mu(2)+z[i].bearing);
        float q = std::pow(dx,2)+std::pow(dy,2);
        float sq = std::sqrt(q);
        expectedZ(2*i) = sq;
        expectedZ(2*i+1) = std::atan2(dy,dx) - mu(2);

        // Compute the Jacobian Hi of the measurement function h for this observation

        MatrixXf F = MatrixXf::Zero(5,mu.size());
        F(0,0) = 1;
        F(1,1) = 1;
        F(2,2) = 1;
        F(3, landmark_x_idx) = 1;
        F(4, landmark_y_idx) = 1;

        MatrixXf Hx(2,5);
        Hx << -sq*dx, -sq*dy, 0, sq*dx, sq*dy,
                            dy, -dx, -q, -dy, dx;

        MatrixXf Hi = (1.0/q)*Hx*F;

        // Augment H with the new Hi
        H.block(i*Hi.rows(), 0, Hi.rows(), Hi.cols()) = Hi;
    }

    // Construct the sensor noise matrix Q
    MatrixXf Q = MatrixXf::Identity(2*m,2*m)*0.01;

    // Compute the Kalman gain
    MatrixXf K = sigma*H.transpose()*(H*sigma*H.transpose() + Q).inverse();

    // Compute the difference between the expected and recorded measurements.
    // Remember to normalize the bearings after subtracting!
    // (hint: use the normalize_all_bearings function available in tools)
    VectorXf innov = Z - expectedZ;
    normalize_all_bearings(innov);

    // Finish the correction step by computing the new mu and sigma.
    // Normalize theta in the robot pose.
    mu = mu + K*innov;
    MatrixXf I = MatrixXf::Identity(mu.size(), mu.size());
    sigma = (I - K*H)*sigma;

}
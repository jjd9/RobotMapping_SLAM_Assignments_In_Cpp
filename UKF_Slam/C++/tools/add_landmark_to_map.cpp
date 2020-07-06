#include "tools.h"

// Add a landmark to the UKF.
// We have to compute the uncertainty of the landmark given the current state
// (and its uncertainty) of the newly observed landmark. To this end, we also
// employ the unscented transform to propagate Q (sensor noise) through the
// current state

void add_landmark_to_map(VectorXf &mu, MatrixXf &sigma, Sensor z, std::vector<int> &map, Matrix2f Q){

    int landmarkId = z.id;

    //add landmark to the map
    map.push_back(landmarkId);
    // TODO: Initialize its pose according to the measurement and add it to mu

    // Append the measurement to the state vector
    VectorXf mu_new = VectorXf::Zero(mu.size()+2);
    mu_new.block(0,0,mu.rows(),1) = mu;
    mu_new(mu.rows()) = z.range;
    mu_new(mu.rows()+1) = z.bearing;
    mu = mu_new;
    // Initialize its uncertainty and add it to sigma
    MatrixXf sigma_new = MatrixXf::Zero(sigma.rows()+Q.rows(), sigma.cols()+Q.cols());
    sigma_new.block(0,0,sigma.rows(), sigma.cols()) = sigma;
    sigma_new.block(sigma.rows(), sigma.cols(), Q.rows(), Q.cols()) = Q;
    sigma = sigma_new;

    // Transform from [range, bearing] to the x/y location of the landmark
    // This operation intializes the uncertainty in the position of the landmark
    // Sample sigma points
    MatrixXf sig_pnts_new = compute_sigma_points(mu, sigma);
    // Normalize!
    sig_pnts_new.row(2) = normalize_angle(sig_pnts_new.row(2));
    // Compute the xy location of the new landmark according to each sigma point
    // The last 2 components of the sigma points can now be replaced by the xy pose of the landmark
    int lastIndex = sig_pnts_new.rows()-1;
    MatrixXf tempX = sig_pnts_new.row(0).array() + sig_pnts_new.row(lastIndex-1).array()*((sig_pnts_new.row(2) + sig_pnts_new.row(lastIndex)).array().cos().array());
    MatrixXf tempY = sig_pnts_new.row(1).array() + sig_pnts_new.row(lastIndex-1).array()*((sig_pnts_new.row(2) + sig_pnts_new.row(lastIndex)).array().sin().array());
    sig_pnts_new.row(lastIndex-1) = tempX;
    sig_pnts_new.row(lastIndex) = tempY;
    // Recover mu and sigma
    Gaussian gauss = recoverGaussian(sig_pnts_new);
    mu=gauss.mu;
    sigma=gauss.sigma;

}
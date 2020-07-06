#include "tools.h"

void correction_step(VectorXf &mu, MatrixXf &sigma, std::vector<Sensor> z, std::vector<int> &map){

	// Updates the belief, i.e., mu and sigma after observing landmarks,
	// and augments the map with newly observed landmarks.
	// The employed sensor model measures the range and bearing of a landmark
	// mu: state vector containing robot pose and poses of landmarks obeserved so far.
	// Current robot pose = mu(1:3)
	// Note that the landmark poses in mu are stacked in the order by which they were observed
	// sigma: the covariance matrix of the system.
	// z: struct array containing the landmark observations.
	// Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
	// The vector 'map' contains the ids of all landmarks observed so far by the robot in the order
	// by which they were observed, NOT in ascending id order.

	// Number of measurements in this time step
	int m = z.size();

	// Measurement noise
	Matrix2f Q = 0.01*Matrix2f::Identity();

	for(int i = 0; i < m; ++i){

		// If the landmark is observed for the first time:
		auto result = findInVector(map, z[i].id);
		if (!result.first){
			// Add new landmark to the map
			add_landmark_to_map(mu, sigma, z[i], map, Q);
			// The measurement has been incorporated so we quit the correction step
			continue;
		}

		// Compute sigma points from the predicted mean and covariance
		// This corresponds to line 6 on slide 32
		MatrixXf sigma_points = compute_sigma_points(mu, sigma);
		// Normalize!
		sigma_points.row(2) = normalize_angle(sigma_points.row(2));

		// Compute lambda
		int n = mu.size();
		int num_sig = sigma_points.cols();
		float lambda = scale - n;

		// extract the current location of the landmark for each sigma point
		// Use this for computing an expected measurement, i.e., applying the h function
		int landmarkIndex = result.second;
		MatrixXf landmarkXs = sigma_points.row(2*landmarkIndex + 3);
		MatrixXf landmarkYs = sigma_points.row(2*landmarkIndex + 4);

		// TODO: Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
		// This corresponds to line 7 on slide 32
		MatrixXf z_points = MatrixXf::Zero(2,2*n+1);
        MatrixXf dx = landmarkXs - sigma_points.row(0);
        MatrixXf dy = landmarkYs - sigma_points.row(1);
        MatrixXf q = (dx.array()*dx.array()) + (dy.array()*dy.array());
        MatrixXf sq = q.array().sqrt();
        z_points.row(0) = sq;
        z_points.row(1) = normalize_angle(vect_atan2(dy,dx) - sigma_points.row(2));

		// setup the weight vector for mean and covariance
		VectorXf w_m = VectorXf::Zero(2*n+1);
		VectorXf w_c = VectorXf::Zero(2*n+1);
		w_m(0) = lambda/scale;
		w_c(0) = lambda/scale;
		for(int j = 1; j <= 2*n; ++j){
			w_m(j) = 1.0/2.0/scale;
			w_c(j) = 1.0/2.0/scale;
		}

		// TODO: Compute zm, line 8 on slide 32
		// zm is the recovered expected measurement mean from z_points.
		// It will be a 2x1 vector [expected_range; expected_bearing].
		// For computing the expected_bearing compute a weighted average by
		// summing the sines/cosines of the angle
		Vector2f zm = z_points*w_m;
		zm(1) = std::atan2(z_points.row(1).array().sin().matrix()*w_m, z_points.row(1).array().cos().matrix()*w_m);

		// TODO: Compute the innovation covariance matrix S (2x2), line 9 on slide 32
		// Remember to normalize the bearing after computing the difference
		MatrixXf diff_z = z_points - zm.replicate(1,2*n+1);
		diff_z.row(1) = normalize_angle(diff_z.row(1));
		MatrixXf diffMat = (diff_z.array()*w_c.transpose().replicate(diff_z.rows(),1).array()).matrix()*diff_z.transpose();
		MatrixXf S_t = diffMat + Q;

		// TODO: Compute Sigma_x_z, line 10 on slide 32
		// (which is equivalent to sigma times the Jacobian H transposed in EKF).
		// sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
		// Remember to normalize the bearing after computing the difference
		MatrixXf diff_x = sigma_points - mu.replicate(1,2*n+1);
		diff_x.row(2) = normalize_angle(diff_x.row(2));
		MatrixXf Sigma_x_z = (diff_x.array()*(w_c.transpose().replicate(diff_x.rows(),1).array())).matrix()*diff_z.transpose();

		// TODO: Compute the Kalman gain, line 11 on slide 32
		MatrixXf K = Sigma_x_z*S_t.inverse();

		// Get the actual measurement as a vector (for computing the difference to the observation)
		Vector2f z_actual = {z[i].range, z[i].bearing};

		// TODO: Update mu and sigma, line 12 + 13 on slide 32
		// normalize the relative bearing
		Vector2f z_diff = z_actual-zm;
		z_diff(1) = normalize_angle(z_diff(1));

		mu = mu + K*z_diff;
		sigma = sigma - K*S_t*K.transpose();

		// TODO: Normalize the robot heading mu(2)
		mu(2) = normalize_angle(mu(2));

	}

}
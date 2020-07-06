#include <iostream>

#include "tools.h"


int main(){
  // load the odometry measurements
  MatrixXf odom_motions = read_odom("data/odom_motions");

  // the motions as they are estimated by scan-matching
  MatrixXf scanmatched_motions = read_scans("data/scanmatched_motions");

  MatrixXf z(2000, 6);

  // create our measurements vector z
  z << scanmatched_motions, odom_motions;

  // perform the calibration
  Matrix3f X = ls_calibrate_odometry(z);
  std::cout << "calibration result" << X << std::endl;

  // apply the estimated calibration parameters
  MatrixXf calibrated_motions = apply_odometry_correction(X, odom_motions);

  // compute the current odometry trajectory, the scanmatch result, and the calibrated odom
  MatrixXf odom_trajectory = compute_trajectory(odom_motions);
  MatrixXf scanmatch_trajectory = compute_trajectory(scanmatched_motions);
  MatrixXf calibrated_trajectory = compute_trajectory(calibrated_motions);

  // write the trajectories to csv for plotting
  const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
  std::ofstream output1{"output/odom.csv"};
  output1 << odom_trajectory.format(CSVFormat) << "\n";
  output1.close();
  std::ofstream output2{"output/scanmatch.csv"};
  output2 << scanmatch_trajectory.format(CSVFormat) << "\n";
  output2.close();
  std::ofstream output3{"output/calibrated.csv"};
  output3 << calibrated_trajectory.format(CSVFormat) << "\n";
  output3.close();




  return 0;
}
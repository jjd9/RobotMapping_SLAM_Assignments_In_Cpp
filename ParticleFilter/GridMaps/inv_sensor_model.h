#pragma once

#include <iostream>
#include <vector>

#include "../../eigen/Eigen/Dense"

#include "tools.h"
#include "read_robot_laser.h"


// TO DO
MatrixXf inv_sensor_model(MatrixXf map, LaserReading scan, Vector3f robPose, float gridSize, std::vector<float> offset, float probOcc, float probFree, VectorXf &robPoseMapFrame, MatrixXf &laserEndPntsMapFrame){
/*
Compute the log odds values that should be added to the map based on the inverse sensor model
of a laser range finder.

map is the matrix containing the occupancy values (IN LOG ODDS) of each cell in the map.
scan is a laser scan made at this time step. Contains the range readings of each laser beam.
robPose is the robot pose in the world coordinates frame.
gridSize is the size of each grid in meters.
offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
when converting to map coordinates.
probOcc is the probability that a cell is occupied by an obstacle given that a
laser beam endpoint hit that cell.
probFree is the probability that a cell is occupied given that a laser beam passed through it.

mapUpdate is a matrix of the same size as map. It has the log odds values that need to be added for the cells
affected by the current laser scan. All unaffected cells should be zeros.
robPoseMapFrame is the pose of the robot in the map coordinates frame.
laserEndPntsMapFrame are map coordinates of the endpoints of each laser beam (also used for visualization purposes).
*/

// Initialize mapUpdate.
MatrixXf mapUpdate = MatrixXf::Zero(map.rows(),map.cols());

// Robot pose as a homogeneous transformation matrix.
Matrix3f robTrans = v2t(robPose);

// TODO: compute robPoseMapFrame. Use your world_to_map_coordinates implementation.
robPoseMapFrame = world_to_map_coordinates(robPose, gridSize, offset);

// Compute the Cartesian coordinates of the laser beam endpoints.
// Set the third argument to 'true' to use only half the beams for speeding up the algorithm when debugging.
MatrixXf laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

// Compute the endpoints of the laser beams in the world coordinates frame.
laserEndPnts = robTrans*laserEndPnts;

// TODO: compute laserEndPntsMapFrame from laserEndPnts. Use your world_to_map_coordinates implementation.
laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts, gridSize, offset);

// Iterate over each laser beam and compute freeCells.
// Use the bresenham method available to you in tools for computing the X and Y
// coordinates of the points that lie on a line.
// Example use for a line between points p1 and p2:
// [X,Y] = bresenham(map,[p1_x, p1_y; p2_x, p2_y]);
// You only need the X and Y outputs of this function.
for (int sc = 0; sc < laserEndPntsMapFrame.cols(); sc++){
        //TODO: compute the XY map coordinates of the free cells along the laser beam ending in laserEndPntsMapFrame(:,sc)
        Matrix2i endpoints;
        endpoints.row(0) = robPoseMapFrame.block(0,0,2,1).cast<int>().transpose();
        endpoints.row(1) = laserEndPntsMapFrame.block(0,sc,2,1).cast<int>().transpose();

        MatrixXi lineProfile = bresenham(endpoints);

        // update the log odds values in mapUpdate for each free cell according to probFree.
        for(int i = 0; i < lineProfile.rows(); ++i){
                mapUpdate(lineProfile(i,0),lineProfile(i,1)) = prob_to_log_odds(probFree);
        }

        // update the log odds values in mapUpdate for each laser endpoint according to probOcc.
        mapUpdate(endpoints(1,0),endpoints(1,1)) = prob_to_log_odds(probOcc);

}

return mapUpdate;
}
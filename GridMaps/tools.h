#pragma once

#include <cmath>

#include "../../eigen/Eigen/Dense"

using namespace Eigen;

struct LaserReading{
        float start_angle, angular_resolution;
        VectorXf ranges;
        Vector3f pose;
        Vector3f laser_offset;
        int timestamp, maximum_range;
};


float prob_to_log_odds(float p){
    // Convert proability values p to the corresponding log odds l.
    // p could be a scalar or a matrix.

    return std::log(p/(1-p));
}


MatrixXf prob_to_log_odds(MatrixXf p){
    // Convert proability values p to the corresponding log odds l.
    // p could be a scalar or a matrix.

    MatrixXf l = MatrixXf::Zero(p.rows(),p.cols());
    for(int i = 0; i <p.rows(); ++i){
        for(int j = 0; j <p.cols(); ++j){
            l(i,j) = prob_to_log_odds(p(i,j));
        }
    }

    return l;
}

float log_odds_to_prob(float l){
    // Convert log odds l to the corresponding probability values p.
    // l could be a scalar or a matrix.
    return 1.0 - 1.0/(1.0 + std::exp(l));
}


MatrixXf log_odds_to_prob(MatrixXf l){
    // Convert log odds l to the corresponding probability values p.
    // l could be a scalar or a matrix.

    MatrixXf p = MatrixXf::Zero(l.rows(),l.cols());
    for(int i = 0; i <l.rows(); ++i){
        for(int j = 0; j <l.cols(); ++j){
            p(i,j) = log_odds_to_prob(l(i,j));
        }
    }

    return p;
}

Vector3f t2v(Matrix3f A){
    // computes the pose vector v from an homogeneous transform A
    Vector3f v;
    v.block(0,0,2,1) = A.block(0,2,2,1); 
	v(2)=std::atan2(A(1,0),A(0,0));
    return v;
}

Matrix3f v2t(Vector3f v){
    // computes the homogeneous transform matrix A of the pose vector v
    Matrix3f A;
  	float c=std::cos(v(2));
  	float s=std::sin(v(2));
	A << c, -s, v(0),
	     s,  c, v(1),
	     0,   0,  1;

    return A;
}

template <typename T>
void swap(T &s, T &t){
    // function SWAP
    T temp = s;
    s = t;
    t = temp;
}


// Need to fix
MatrixXf world_to_map_coordinates(MatrixXf pntsWorld, float gridSize, std::vector<float> offset){
    /*
    Convert points from the world coordinates frame to the map frame.
    pntsWorld is a matrix of N points with each column representing a point in world coordinates (meters).
    gridSize is the size of each grid in meters.
    offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
    when converting to map coordinates.
    pntsMap is a 2xN matrix containing the corresponding points in map coordinates.
    */

    pntsWorld.row(0) = pntsWorld.row(0).array() - offset[0];
    pntsWorld.row(1) = pntsWorld.row(1).array() - offset[1];
    pntsWorld = (pntsWorld.array() / gridSize).array().floor();

    return pntsWorld;
}

MatrixXf robotlaser_as_cartesian(LaserReading rl, int maxRange=15, bool subsample = false){

    int numBeams = rl.ranges.size();
    maxRange=std::min(maxRange, rl.maximum_range);
    
    // apply the max range
    int step = 1;
    if(subsample){
        step = 2;
    }
    std::vector<int> idx;
    for(int i = 0 ; i < rl.ranges.size(); i+=step){
        if(rl.ranges(i) < maxRange & rl.ranges(i) > 0){
            idx.push_back(i);
        }
    }

    VectorXf angles(numBeams);
    angles.col(0) = ArrayXf::LinSpaced(numBeams, rl.start_angle, rl.start_angle + numBeams*rl.angular_resolution);

    MatrixXf points(3, idx.size());
    points.row(0) = rl.ranges(idx).array()*angles(idx).array().cos().array();
    points.row(1) = rl.ranges(idx).array()*angles(idx).array().sin().array();
    points.row(2) = MatrixXf::Ones(1, idx.size());
    
    Matrix3f transf = v2t(rl.laser_offset);

    // apply the laser offset
    points = transf * points;
    return points;

}

MatrixXi bresenham(Matrix2i mycoords){
    /*
    BRESENHAM: Generate a line profile of a 2d image 
            using Bresenham's algorithm
    [myline,mycoords] = bresenham(mymat,mycoords,dispFlag)
    - For a demo purpose, try >> bresenham();
    - mymat is an input image matrix.
    - mycoords is coordinate of the form: [x1, y1; x2, y2]
    which can be obtained from ginput function
    Ref: nprotech: Chackrit Sangkaew; Citec
    Ref: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm

    See also: tut_line_algorithm
    */

    VectorXi x = mycoords.col(0);
    VectorXi y = mycoords.col(1);

    bool isSteep = (std::abs(y(1)-y(0)) > std::abs(x(1)-x(0)));

    if (isSteep){
        swap(x,y);
    }

    if (x(0) > x(1)){
        swap(x(0),x(1));
        swap(y(0),y(1));
    }

    int delx = x(1)-x(0);
    int dely = std::abs(y(1)-y(0));
    int error = 0;
    int x_n = x(0);
    int y_n = y(0);
    int ystep;
    if (y(0) < y(1)){
        ystep = 1; 
    }else{
        ystep = -1;
    }

    MatrixXi lineProfile(delx, 2);

    for (int n = 0; n < delx; n++){
        if (isSteep){
            lineProfile(n,0) = y_n;
            lineProfile(n,1) = x_n;
        }else{
            lineProfile(n,0) = x_n;
            lineProfile(n,1) = y_n;
        } 
        x_n = x_n + 1;
        error = error + dely;
        if (2*error >= delx){ 
            y_n = y_n + ystep;
            error = error - delx;
        }
    }

    return lineProfile;   
}
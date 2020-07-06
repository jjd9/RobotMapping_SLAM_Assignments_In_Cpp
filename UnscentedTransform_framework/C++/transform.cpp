#include "tools.h"

void transform(MatrixXf &points){
    // This function applies a transformation to a set of points.
    // Each column in points is one point, i.e. points = [[x1; y1], [x2;y2], ...]
    // Select which function you want to use by uncommenting it
    // (deleting the corresponding //{...//}) while keeping all other functions commented.

    // // Function 1 (linear)
    // // Applies a translation to [x; y]
    // points.row(0) = points.row(0).array() + 1;
    // points.row(1) = points.row(1).array() + 2;

    // // Function 2 (nonlinear)
    // // Computes the polar coordinates corresponding to [x; y]
    // VectorXf x = points.row(0).transpose();
    // VectorXf y = points.row(1).transpose();
    // VectorXf r = (x.array()*x.array() + y.array()*y.array()).sqrt();
    // VectorXf theta = VectorXf::Zero(r.rows());
    // for(int i = 0; i < theta.rows(); ++i){
    //     theta(i) = std::atan2(y(i),x(i));
    // }
    // points.row(0) = r.transpose();
    // points.row(1) = theta.transpose();

    // Function 3 (nonlinear)    
    points.row(0) = points.row(0).array()*points.row(0).array().cos().array()*points.row(0).array().sin().array();
    points.row(1) = points.row(1).array()*points.row(1).array().cos().array()*points.row(1).array().sin().array();
}
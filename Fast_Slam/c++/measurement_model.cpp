#include "tools.h"

/*
 Compute the expected measurement for a landmark
 and the Jacobian with respect to the landmark
*/

MeasurementPrediction measurement_model(const Particle &particle, Sensor z){

    MeasurementPrediction z_pred;

    // extract the id of the landmark
    int landmarkId = z.id;
    // two 2D vector for the position (x,y) of the observed landmark
    Vector2f landmarkPos = particle.landmarks[landmarkId].mu;

    // use the current state of the particle to predict the measurment
    float landmarkX = landmarkPos(0);
    float landmarkY = landmarkPos(1);
    float expectedRange = std::sqrt(std::pow((landmarkX - particle.pose(0)),2) + std::pow((landmarkY - particle.pose(1)),2));
    float expectedBearing = normalize_angle(std::atan2(landmarkY-particle.pose(1), landmarkX-particle.pose(0)) - particle.pose(2));
    z_pred.h(0) = expectedRange;
    z_pred.h(1) = expectedBearing;

    // Compute the Jacobian H of the measurement function h wrt the landmark location
    z_pred.H = Matrix2f::Zero();
    z_pred.H(0,0) = (landmarkX - particle.pose(0))/expectedRange;
    z_pred.H(0,1) = (landmarkY - particle.pose(1))/expectedRange;
    z_pred.H(1,0) = (particle.pose(1) - landmarkY)/std::pow(expectedRange,2);
    z_pred.H(1,1) = (landmarkX - particle.pose(0))/std::pow(expectedRange,2);

    return z_pred;
}
#pragma once

#include "../eigen/Eigen/Dense"

#include "normalize_angle.h"


using namespace Eigen;

void normalize_all_bearings(VectorXf &z){
   // Go over the observations vector and normalize the bearings
   // The expected format of z is [range; bearing; range; bearing; ...]

   for(int i=1; i < z.size(); i+=2){
      z[i] = normalize_angle(z[i]);
   }
}
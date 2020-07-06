#include "tools.h"

Vector3f t2v(Matrix3f A){
    // computes the pose vector v from an homogeneous transform A
    Vector3f v;
    v.block(0,0,2,1) = A.block(0,2,2,1); 
  	v(2)=std::atan2(A(1,0),A(0,0));
    return v;
}
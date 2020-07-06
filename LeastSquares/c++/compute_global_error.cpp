#include "tools.h"

// Computes the total error of the graph
float  compute_global_error(Graph &g){

  float Fx = 0;

  // Loop over all edges
  for (auto edge : g.edges){

    // pose-pose constraint
    if (edge.type == 80){

      Matrix3f x1 = v2t(g.x.block(edge.fromIdx,0,3,1));  // the first robot pose
      Matrix3f x2 = v2t(g.x.block(edge.toIdx,0,3,1));      // the second robot pose

      // compute the error of the constraint and add it to Fx.
      // Use edge.measurement and edge.information to access the
      // measurement and the information matrix respectively.      
      Matrix3f Z_pred = x1.inverse()*x2;
      Matrix3f Zinv = v2t(edge.measurement).inverse();
      Vector3f e12 = t2v(Zinv*Z_pred);
      Fx += e12.transpose()*edge.information*e12;  

    // pose-landmark constraint
    }else if (edge.type == 76){
      Vector3f x = g.x.block(edge.fromIdx, 0, 3, 1);  // the robot pose
      Vector2f l = g.x.block(edge.toIdx, 0, 2, 1);      // the landmark

      // compute the error of the constraint and add it to Fx.
      // Use edge.measurement and edge.information to access the
      // measurement and the information matrix respectively.

      // assuming we observe (x,y) landmark position
      Matrix2f R = v2t(x).block(0,0,2,2);
      Vector2f e12 = R.transpose()*(l-x.block(0,0,2,1)) - edge.measurement;
      Fx += e12.transpose()*edge.information*e12;

    }

  }

  return Fx;
}
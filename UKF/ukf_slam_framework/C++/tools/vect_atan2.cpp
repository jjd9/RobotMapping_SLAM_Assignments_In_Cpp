#include "tools.h"

// element-wise atan2
MatrixXf vect_atan2(const MatrixXf &y, const MatrixXf & x){
	MatrixXf c = MatrixXf::Zero(y.rows(),y.cols());
	for(int i = 0; i < y.rows(); ++i){
		for(int j = 0; j < y.cols(); ++j){
			c(i,j) = std::atan2(y(i,j),x(i,j));
		}
	}
	return c;
}
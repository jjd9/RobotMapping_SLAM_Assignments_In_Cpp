#include "tools.h"


// Normalize phi to be between -pi and pi
float normalize_angle(float phi){

	return std::fmod((phi + pi), 2*pi) - pi;
}

// Normalize phi to be between -pi and pi
MatrixXf normalize_angle(MatrixXf mat){

	for(int i = 0; i < mat.rows(); ++i){
		for(int j = 0; j < mat.cols(); ++j){

			mat(i,j) = std::fmod((mat(i,j) + pi), 2*pi) - pi;

		}
	}

	return mat;
}
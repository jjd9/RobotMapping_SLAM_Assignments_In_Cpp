#include "tools.h"

float normalize_angle(float phi){

	float PI = 3.14159;

	// Normalize phi to be between -pi and pi
	while(phi>PI){
		phi = phi - 2*PI;
	}

	while(phi<-PI){
		phi = phi + 2*PI;
	}

	return phi;

}
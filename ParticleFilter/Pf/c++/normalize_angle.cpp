#include "tools.h"

float normalize_angle(float phi){
	// Normalize phi to be between -pi and pi
	float pi = 3.14159;
	while(phi>pi){
		phi = phi - 2*pi;
	}

	while(phi<-pi){
		phi = phi + 2*pi;
	}

	return phi;

}
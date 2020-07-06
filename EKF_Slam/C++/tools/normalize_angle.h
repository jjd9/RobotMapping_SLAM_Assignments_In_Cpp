float pi = 3.1415926;

// Normalize phi to be between -pi and pi
float normalize_angle(float phi){

	while(phi>pi){
		phi = phi - 2*pi;
	}

	while(phi<-pi){
		phi = phi + 2*pi;
	}

	return phi;
}
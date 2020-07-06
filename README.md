# RobotMapping_SLAM_Assignments_In_Cpp
I solved the assignments from the robot mapping course taught by Dr. Cyrill Stachniss at the University of Freiburg using C++.

## Status / TODO's:
I have solved the problems and my plots agree with those in the assignments or the "world" definitions; however, some of the code could certainly be improved in terms of proof of validity, readability, and efficiency. The general structuring of the project is also really bad at the moment, so I plan to fix this.

Dumping into csv and plotting with python "works", but I would really like to setup something like gnuplot so the plots can be generated as the code is running.

I would like to setup a set of cMake files to build the whole project at once so people do not have to compile each folder individually. 

I am working on this as a side project, so I will get to these issues as my freetime allows. 

## Sources:
[1] Jacob, B., and G. Guennebaud. "Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms." 2020. http://eigen.tuxfamily.org/index.php?title=Main_Page

[2] *Stachniss, Cyrill. Robot Mapping, University of Freiburg. http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/

[3] Thrun, Sebastian. "Probabilistic robotics." Communications of the ACM 45.3 (2002): 52-57.

*Credit for the data files, assignments, and Octave "tools" code off of which some of the c++ code is based goes to Dr. Cyrill Stachniss.

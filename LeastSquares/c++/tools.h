#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../../eigen/Eigen/Dense"
#include "../../eigen/Eigen/Sparse"
#include "../../eigen/Eigen/SparseCholesky"

using namespace Eigen;

struct LinearizedConstraintError{

    VectorXf e;
    MatrixXf A, B;

};

struct Entry{
    int offset;
    int dimension;
};

struct Edge{

    int type;
    int to;
    int from;
    VectorXf measurement;
    MatrixXf information;
    int toIdx;
    int fromIdx;

};

struct Graph{

    VectorXf x;
    std::vector<Edge> edges;
    std::unordered_map<int, Entry> idLookup;

};

struct Mesh{
    MatrixXi xx, yy;
};

std::vector<std::string> splitBy(std::string str, std::string delimiter);

Graph read_graph(std::string baseFileName);

Matrix3f v2t(Vector3f v);

Vector3f t2v(Matrix3f A);

int nnz_of_graph(Graph &g);

LinearizedConstraintError linearize_pose_landmark_constraint(Vector3f x, Vector2f l, Vector2f z);

LinearizedConstraintError linearize_pose_pose_constraint(Vector3f x1, Vector3f x2, Vector3f z);

float  compute_global_error(Graph &g);

VectorXf linearize_and_solve(Graph &g);

void get_poses_landmarks(Graph &g, std::vector<int> &poses, std::vector<int> &landmarks);
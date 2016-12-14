#include "ALGLIB/optimization.h"
#include "Eigen/Dense"
#include <iostream>

using namespace alglib;
using namespace Eigen;
using namespace std;

real_2d_array Eigen_2_2d_array(MatrixXd &M);
real_1d_array Eigen_2_1d_array(VectorXd &V);
integer_1d_array Eigen_2_1d_integer_array(VectorXi &V);
VectorXd NormalOptimization(MatrixXd &N, MatrixXd &M, VectorXd &qdot, VectorXd &fi);
VectorXd FrictionOptimization(double mu, MatrixXd &N, MatrixXd &D, MatrixXd &M, VectorXd &qdot, VectorXd &alpha);

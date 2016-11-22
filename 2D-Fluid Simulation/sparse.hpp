#ifndef SPARSE_HPP
#define SPARSE_HPP

#include <Eigen/Sparse>
#include <vector>
using namespace Eigen;
using namespace std;

// helper class for incremental construction of sparse matrices
class SparseMatrixBuilder {
public:
    int m, n;
    vector< Triplet<double> > triplets;
    SparseMatrixBuilder(int m, int n): m(m), n(n) {}
    void add(int i, int j, double value);
    void addBlock(int i, int j, int p, int q, MatrixXd block);
    SparseMatrix<double> getMatrix();
    VectorXd solve(const VectorXd &b);
    int testMethod(int val);
};

#endif

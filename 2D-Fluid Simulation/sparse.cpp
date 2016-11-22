#include "sparse.hpp"

void SparseMatrixBuilder::add(int i, int j, double value) {
    triplets.push_back(Triplet<double>(i, j, value));
}

SparseMatrix<double> SparseMatrixBuilder::getMatrix() {
    SparseMatrix<double> A(m, n);
    A.setFromTriplets(triplets.begin(), triplets.end());
    return A;
}

VectorXd SparseMatrixBuilder::solve(const VectorXd &b) {
    SparseMatrix<double> A(m, n);
    A.setFromTriplets(triplets.begin(), triplets.end());
    ConjugateGradient< SparseMatrix<double> > solver;
    solver.setTolerance(1e-6);
    solver.setMaxIterations(200);
    return solver.compute(A).solve(b);
}

int SparseMatrixBuilder::testMethod(int val){
	return val;
}

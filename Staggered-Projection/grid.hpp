#ifndef GRID_HPP
#define GRID_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

using namespace Eigen;

template <typename T>
class Grid3D {
public:
    Vector3i n;            // number of grid cells
    Vector3d x0;           // world-space position of cell (0,0,0)
    Vector3d dx;           // grid cell spacing
    std::vector<T> values; // grid cell values
    Grid3D() {
        n = Vector3i(0,0,0);
    }
    Grid3D(Vector3i n, Vector3d x0, Vector3d dx):
        n(n), x0(x0), dx(dx), values(n[0]*n[1]*n[2], T()) {
    }
    // value at cell (i,j,k)
    T& get(Vector3i index);
    T& operator()(Vector3i index) {
        return get(index);
    }
    // assign all values
    void assign(T value);
    // interpolated value at world-space position x
    T interpolate(Vector3d x);
    // interpolated finite-difference gradient at world-space position x,
    // returns a Vector3d if the cell type T is double
    Matrix<T,3,1> gradient(Vector3d x);
};

void readGrid(Grid3D<double> &grid, std::string filename);

#endif

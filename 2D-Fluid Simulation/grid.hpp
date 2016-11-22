#ifndef GRID_HPP
#define GRID_HPP

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

template <typename T>
class Grid {
public:
    int m, n;         // number of grid cells
    Vector2d x0;      // world-space position of cell (0, 0)
    double dx;        // grid cell spacing
    vector<T> values; // grid cell values
    Grid() {}
    Grid(int m, int n, Vector2d x0, double dx):
        m(m), n(n), x0(x0), dx(dx), values(m*n) {}
    // value at cell (i, j)
    const T& get(int i, int j) const;
    T& operator()(int i, int j) {
        return (T&)get(i, j);
    }
    const T& operator()(int i, int j) const {
        return get(i, j);
    }
    // assign all values
    void assign(T value);
    // interpolated value at world-space position x
    T interpolate(Vector2d x) const;
};

class StaggeredGrid {
public:
    int m, n;          // number of grid cells
    Vector2d x0;       // world-space position of cell (0, 0)
    double dx;         // grid cell spacing
    Grid<double> x, y; // x and y components of vector field
    StaggeredGrid() {}
    StaggeredGrid(int m, int n, Vector2d x0, double dx);
    // assign all values
    void assign(Vector2d value);
    // interpolated value at world-space position p
    Vector2d interpolate(Vector2d p) const;
};

// Writes gradient of p into grad_p.
// Assumes both are initialized with compatible sizes.
void gradient(const Grid<double> &p, StaggeredGrid &grad_p);

// Writes divergence of u into div_u.
// Assumes both are initialized with compatible sizes.
void divergence(const StaggeredGrid &u, Grid<double> &div_u);

#endif

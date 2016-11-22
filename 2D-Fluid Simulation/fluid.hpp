#ifndef FLUID_HPP
#define FLUID_HPP

#include "grid.hpp"
#include "sparse.hpp"

// Returns the point x0 advected under u for time dt
Vector2d advectPoint(const StaggeredGrid &u, Vector2d x0, double dt);

// Advects c0 under the velocity field u for time dt and writes it to c1
void advectScalarField(const StaggeredGrid &u, Grid<double> &c0,
                       Grid<double> &c1, double dt);

// Advects v0 under the velocity field u for time dt and writes it to v1
void advectVectorField(const StaggeredGrid &u, const StaggeredGrid &v0,
                       StaggeredGrid &v1, double dt);

// Writes vector index of each grid cell and returns total number of cells
int enumerateCells(Grid<int> &index);

class Fluid {
public:
    int m, n, N;              // grid width, height, total number of cells
    Vector2d x0;              // domain origin
    double dx;                // grid cell size
    StaggeredGrid *u, *u_tmp; // velocity
    Grid<double> p;           // pressure
    Grid<int> index;          // cell indices
    Fluid(int m, int n, Vector2d x0, double dx);
    void step(double dt);
    void advection(double dt);
    void pressureProjection();
};

#endif

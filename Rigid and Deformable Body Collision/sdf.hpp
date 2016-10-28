#ifndef SDF_HPP
#define SDF_HPP

#include <Eigen/Dense>
#include <string>
#include "grid.hpp"

using namespace Eigen;

class SDF {
public:
    virtual double signedDistance(Vector3d x) = 0;
    virtual Vector3d normal(Vector3d x) = 0;
};

class PlaneSDF: public SDF {
public:
    Vector3d x0, n;
    PlaneSDF(Vector3d x0, Vector3d n):
        x0(x0), n(n.normalized()) {
    }
    virtual double signedDistance(Vector3d x) {
    	return (x-x0).dot(n);
    }
    virtual Vector3d normal(Vector3d x) {
        return n;
    }
};

class SphereSDF: public SDF {
public:
    Vector3d x0;
    double r;
    SphereSDF(Vector3d x0, double r):
        x0(x0), r(r) {
    }
    virtual double signedDistance(Vector3d x) {
        return (x-x0).norm()-r;
    }
    virtual Vector3d normal(Vector3d x) {
        return (x-x0).normalized();
    }
};

class GridSDF: public SDF {
public:
    Grid3D<double> grid;
    GridSDF(std::string filename) {
        readGrid(grid, filename);
    }
    virtual double signedDistance(Vector3d x) {
        return grid.interpolate(x);
    }
    virtual Vector3d normal(Vector3d x) {
        return grid.gradient(x);
    }
};
#endif

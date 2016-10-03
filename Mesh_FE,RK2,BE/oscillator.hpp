#ifndef OSCILLATOR_HPP
#define OSCILLATOR_HPP

#include "timeintegration.hpp"

class DampedHarmonicOscillator: public PhysicalSystem {
public:
    // m x'' = -k x - c x'
    double x, v, t;
    double m; // mass
    double k; // spring constant
    double c; // damping constant
    DampedHarmonicOscillator(double m, double k, double c):
        m(m), k(k), c(c) {
    }
    virtual int getNumDOFs();
    virtual void getState(VectorXd &x, VectorXd &v, double &t);
    virtual void setState(const VectorXd &x, const VectorXd &v, double t);
    virtual void getInertia(MatrixXd &M);
    virtual void getForces(VectorXd &f);
    virtual void getAccelerations(VectorXd &a);
    virtual void getJacobians(MatrixXd &Jx, MatrixXd &Jv);
};

#endif

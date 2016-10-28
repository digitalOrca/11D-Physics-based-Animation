#ifndef TIMEINTEGRATION_HPP
#define TIMEINTEGRATION_HPP
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;

class PhysicalSystem {
    // parent class for physical systems that support implicit integration
public:
    // returns number of *positional* degrees of freedom (not velocity)
    virtual int getNumDOFs() = 0;
    // writes position, velocity, time into arguments
    virtual void getState(VectorXd &x, VectorXd &v, double &t) = 0;
    // reads position, velocity, time from arguments
    virtual void setState(const VectorXd &x, const VectorXd &v, double t) = 0;
    // writes inertia matrix
    virtual void getInertia(MatrixXd &M) = 0;
    // writes forces
    virtual void getForces(VectorXd &f) = 0;
    // writes accelerations (should be the same as M^-1 f)
    virtual void getAccelerations(VectorXd &a) = 0;
    // writes Jacobians
    virtual void getJacobians(MatrixXd &Jx, MatrixXd &Jv) = 0;
};

class TimeIntegrator {
    // parent class for time integration schemes for PhysicalSystems
public:
    virtual void step(double dt) = 0;
};

class SymplecticEuler: public TimeIntegrator {
public:
    PhysicalSystem *system;
    double t;
    VectorXd x, v, a;
    SymplecticEuler(PhysicalSystem *system) {
        this->system = system;
        int n = system->getNumDOFs();
        // preallocate memory
        x = VectorXd(n);
        v = VectorXd(n);
        a = VectorXd(n);
    }
    virtual void step(double dt){
    	system->getState(x, v, t);
		system->getAccelerations(a);
		system->setState(x + (v + a*dt)*dt, v + a*dt, t + dt);
    }
};
#endif

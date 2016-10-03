#ifndef TIMEINTEGRATION_HPP
#define TIMEINTEGRATION_HPP

#include <Eigen/Dense>
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

class ForwardEuler: public TimeIntegrator {
public:
	PhysicalSystem *sys;
    int NumDOFs;
    double t;
    VectorXd pos;	//position matrix
    VectorXd vel;	//velocity matrix
    VectorXd a;		//acceleration matrix
    VectorXd f;		//forces matrix
    MatrixXd M;		//inverse of inertia matrix
    
    ForwardEuler(PhysicalSystem *system);
    virtual void step(double dt);  
};

class RungeKutta2: public TimeIntegrator {
public:
	PhysicalSystem *sys;
	int NumDOFs;
    double t;
    VectorXd pos;	//position matrix
    VectorXd vel;	//velocity matrix
    VectorXd a;		//acceleration matrix
    VectorXd a_new;		//acceleration matrix
    VectorXd f;		//forces matrix
    MatrixXd M;		//inverse of inertia matrix
	
    RungeKutta2(PhysicalSystem *system);
    virtual void step(double dt);
};

class BackwardEuler: public TimeIntegrator {
public:
	PhysicalSystem *sys;
	int NumDOFs;
    double t;
    VectorXd pos;	//position matrix
    VectorXd vel;	//velocity matrix
    VectorXd a;		//acceleration matrix
    VectorXd f;		//forces matrix
	MatrixXd M;		//inverse of inertia matrix
	MatrixXd Jx;
	MatrixXd Jv;
	MatrixXd A;
	VectorXd b;
	
    BackwardEuler(PhysicalSystem *system);
    virtual void step(double dt);
};

#endif

#ifndef PARTICLES_HPP
#define PARTICLES_HPP

#include "mesh.hpp"
#include "timeintegration.hpp"

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Particle;
class Force;
class SpringForce;

class ParticleSystem: public PhysicalSystem {
public:
    double t ;
    
	vector<Particle*> particles;	//particle pointers
	vector<SpringForce*> forces;	//force pointers

	vector<Particle> particleArray;	//particle objects
	vector<SpringForce> forceArray;	//force objects
	
	double time;
	
	int NumDOFs;
	MatrixXd M; //inverse of inertia
	VectorXd a; //accelerations
	VectorXd f; //per vertex force (compiled)
	
	void load(Mesh2D &mesh);

    virtual int getNumDOFs();
    virtual void getState(VectorXd &x, VectorXd &v, double &t);
    virtual void setState(const VectorXd &x, const VectorXd &v, double t);
    virtual void getInertia(MatrixXd &M);
    virtual void getForces(VectorXd &f);
    virtual void getAccelerations(VectorXd &a);
    virtual void getJacobians(MatrixXd &Jx, MatrixXd &Jv);
};

class Particle {
public:
    Vector2d x, v, f; //position, velocity, force
    double m; //mass
};

class Force {
public:
	//My implementation will treat mouseforce as a spring force
};

class SpringForce: public Force {
    // connects two particles by a spring
public:
	double len;
	Vector2i ends;
	Particle *target;
	double ks, kd;
	Vector2d x;    // point to anchor it to (Anchor Only)
	SpringForce(){ ks=10; kd=5;}	//Define string properties here
	SpringForce(Particle *target, Vector2d x, double ks, double kd): target(target), x(x), ks(ks), kd(kd) {};
};

class AnchorForce: public Force {
    // attaches a particle to a fixed point by a spring
public:
    Particle *p;   // particle
    Vector2d x;    // point to anchor it to
    double ks, kd; // spring constant, damping coefficient
    AnchorForce(Particle *p, Vector2d x, double ks, double kd): p(p), x(x), ks(ks), kd(kd){};
    // TODO
};

#endif

#ifndef PARTICLES_HPP
#define PARTICLES_HPP

#include "mesh.hpp"
#include "sdf.hpp"
#include "timeintegration.hpp"

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Particle;

class Force;

class ParticleSystem: public PhysicalSystem {
public:
    double t;
    vector<Particle*> particles;
    vector<Force*> forces;
    double mu; // coefficient of friction
    ParticleSystem():
        t(0), mu(0) {
    }
    // PhysicalSystem methods
    virtual int getNumDOFs();
    virtual void getState(VectorXd &x, VectorXd &v, double &t);
    virtual void setState(const VectorXd &x, const VectorXd &v, double t);
    virtual void getInertia(MatrixXd &M);
    virtual void getForces(VectorXd &f);
    virtual void getAccelerations(VectorXd &a);
    virtual void getJacobians(MatrixXd &Jx, MatrixXd &Jv);
    // other methods
    void setFrictionCoefficient(double mu) {
        this->mu = mu;
    }
    void clearForces();
};

class Particle {
public:
    int i;       // index
    double m;    // mass
    Vector3d x; // position
    Vector3d v; // velocity
    Vector3d f_ext; // external forces
    Particle(int i, double m, Vector3d x, Vector3d v):
        i(i), m(m), x(x), v(v) {
    }
};

class Force {
public:
    virtual void addForces(VectorXd &f) = 0;
    virtual void addJacobians(MatrixXd &Jx, MatrixXd &Jv) = 0;
};

class GravityForce: public Force {
public:
    ParticleSystem *ps; // apply gravity to all particles
    Vector3d g;         // acceleration vector
    GravityForce(ParticleSystem *ps, Vector3d g): ps(ps), g(g) {}
    virtual void addForces(VectorXd &f);
    virtual void addJacobians(MatrixXd &Jx, MatrixXd &Jv);
};

class SpringForce: public Force {
    // connects two particles by a spring
public:
    Particle *p0, *p1; // particles
    double ks, kd;     // spring constant, damping coefficient
    double l0;         // rest length
    SpringForce(Particle *p0, Particle *p1, double ks, double kd, double l0):
        p0(p0), p1(p1), ks(ks), kd(kd), l0(l0) {}
    virtual void addForces(VectorXd &f);
    virtual void addJacobians(MatrixXd &Jx, MatrixXd &Jv);
};

void createMassSpringSystem(ParticleSystem &ps, const TetMesh &bunnyMesh, double m, double ks, double kd);

void updateBunnyMesh(TetMesh &bunnyMesh, const ParticleSystem &ps);

void collisionProjection(ParticleSystem &ps, SDF &sdf, double dt);
#endif

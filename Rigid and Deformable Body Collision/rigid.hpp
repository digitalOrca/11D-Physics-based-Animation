#ifndef RIGID_HPP
#define RIGID_HPP

#include "mesh.hpp"
#include "sdf.hpp"
#include "timeintegration.hpp"

#include <Eigen/Dense>
using namespace Eigen;

class RigidBody {
public:
    TriMesh *mesh;             // Mesh
    Vector3d xcom, vcom;       // Position and velocity of center of mass
    Vector3d L;                // Angular momentum
    Quaterniond q;             // Orientation
    struct {
        Vector3d xcom, vcom;
        Vector3d L;
        Quaterniond q;
    } previous;                // State at beginning of current time step
    double m;                  // Mass
    Matrix3d Ibody, Ibody_inv; // Moment of inertia in body space
    double mu, epsilon;        // Coefficients of friction and restitution
    SDF *sdf;                  // Signed distance field in body space
    Vector3d f_ext, tau_ext;   // External force and torque accumulators
    RigidBody(TriMesh *mesh, double m, Matrix3d Ibody, SDF *sdf) {
        this->mesh = mesh;
        this->m = m;
        this->Ibody = Ibody;
        this->Ibody_inv = Ibody.inverse();
        xcom = Vector3d(0,0,0);
        vcom = Vector3d(0,0,0);
        q = Quaterniond(1,0,0,0);
        L = Vector3d(0,0,0);
        f_ext = Vector3d(0,0,0);
        tau_ext = Vector3d(0,0,0);
        mu = 0;
        epsilon = 0;
        this->sdf = sdf;
    }
    // Return velocity of point at world-space position x
    Vector3d velocityAt(Vector3d x) const;
    // Return current angular velocity of body
    Vector3d angularVelocity() const;
    // Reset force accumulators f_ext and tau_ext to zero
    void clearForces();
    // Add an external force through world-space point x
    void addForce(Vector3d f, Vector3d x);
    // Add an instantaneous impulse that immediately changes velocity
    void addImpulse(Vector3d j, Vector3d x);
    // Return the body-to-world transformation T. Transform a body-space point
    // using T*x. Transform a vector using T.linear()*x.
    Affine3d transform() const;
    // Return the world-to-body transformation T^-1
    Affine3d inverseTransform() const;
    // Draw the body in its current configuration
    void draw(Vector4d fillColor, Vector4d edgeColor) const;
    // Revert the body to its previous state. (Used for collision resolution)
    void rewind();
};

// Advance the rigid body over time dt in a symplectic-style update
void step(RigidBody &rb, double dt);
// Advance only the linear and angular velocity terms
void stepVelocity(RigidBody &rb, double dt);
// Advance the position and orientation terms
void stepPosition(RigidBody &rb, double dt);

#endif

#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "particles.hpp"
#include "rigid.hpp"
#include "sdf.hpp"

#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

struct Collision {
    // May or may not be an actual collision. Check if isCollision() is true.
    int index0, index1; // index of colliding part e.g. particle
    Vector3d x0, x1;    // contact point(s)
    Vector3d n;         // contact normal
    double d;           // penetration depth, zero or negative if no collision
    Collision():
        index0(-1), index1(-1), d(0), x0(Vector3d(0,0,0)), x1(Vector3d(0,0,0)) {
    }
    bool isCollision() {
        return d > 0;
    }
};

// Finds all particles that lie inside the SDF
void findCollisions(ParticleSystem &ps, SDF *sdf, vector<Collision> &collisions);

// Finds vertex of rigid body that is deepest inside SDF
Collision findCollision(RigidBody &rb, SDF *sdf);

// Finds all particles that lie inside the SDF of the rigid body
void findCollisions(ParticleSystem &ps, RigidBody &rb, vector<Collision> &collisions);

// Project particles to outside of static obstacle (and apply friction)
void collisionProjection(ParticleSystem &ps, SDF *sdf, double dt);

// Apply collision/contact resolution of a rigid body and a static obstacle
void resolveCollisions(RigidBody &rb, SDF *sdf, double dt);
void resolveContacts(RigidBody &rb, SDF *sdf, double dt);

// Add collision penalty forces to both particles and rigid body
void addPenaltyForces(ParticleSystem &ps, RigidBody &rb, double ks, double kd);

#endif

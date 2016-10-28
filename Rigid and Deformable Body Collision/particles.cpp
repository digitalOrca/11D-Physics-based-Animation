#include "particles.hpp"

int ParticleSystem::getNumDOFs() {
    return 3*particles.size();
}

void ParticleSystem::getState(VectorXd &x, VectorXd &v, double &t) {
    t = this->t;
    for (int p = 0; p < particles.size(); p++) {
        Particle *particle = particles[p];
        x.segment(p*3, 3) = particle->x;
        v.segment(p*3, 3) = particle->v;
    }
}

void ParticleSystem::setState(const VectorXd &x, const VectorXd &v, double t) {
    this->t = t;
    for (int p = 0; p < particles.size(); p++) {
        Particle *particle = particles[p];
        particle->x = x.segment(p*3, 3);
        particle->v = v.segment(p*3, 3);
    }
}

void ParticleSystem::getInertia(MatrixXd &M) {
    M.setZero();
    for (int p = 0; p < particles.size(); p++)
        M.block(p*3,p*3, 3,3) = particles[p]->m*Matrix3d::Identity();
}

void ParticleSystem::getForces(VectorXd &f) {
    f.setZero();
    for (int i = 0; i < forces.size(); i++)
        forces[i]->addForces(f);
}

void ParticleSystem::getAccelerations(VectorXd &a) {
    a.setZero();
    for (int i = 0; i < forces.size(); i++)
        forces[i]->addForces(a);
    for (int p = 0; p < particles.size(); p++){
    		a.segment(p*3,3) += particles[p]->f_ext;	// include external force
       		a.segment(p*3,3) /= particles[p]->m;
       	}
}

void ParticleSystem::getJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    Jx.setZero();
    Jv.setZero();
    for (int i = 0; i < forces.size(); i++)
        forces[i]->addJacobians(Jx, Jv);
}

void ParticleSystem::clearForces(){
	for (int p = 0; p < particles.size(); p++)
		particles[p]->f_ext = Vector3d(0,0,0);
}

void GravityForce::addForces(VectorXd &f) {
	for(int i=0; i<ps->particles.size(); i+=3){
		f.segment(i,3) += (ps->particles[i])->m*g;
	}
}

void GravityForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    // TODO
}

void SpringForce::addForces(VectorXd &f) {
    Vector3d x0 = p0->x, x1 = p1->x, dx = x1 - x0;
    Vector3d v0 = p0->v, v1 = p1->v, dv = v1 - v0;
    double l = (p1->x - p0->x).norm();
    Vector3d dxhat = dx/l;
    Vector3d force = -ks*(l - l0)*dxhat - kd*dv.dot(dxhat)*dxhat;
    f.segment(p0->i*3, 3) -= force;
    f.segment(p1->i*3, 3) += force;
}

void SpringForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    int i0 = p0->i, i1 = p1->i;
    Vector3d x0 = p0->x, x1 = p1->x, dx = x1 - x0;
    double l = (p1->x - p0->x).norm();
    Vector3d dxhat = dx/l;
    Matrix3d jx = -ks*(max(1 - l0/l, 0.0)*
                       (Matrix3d::Identity() - dxhat*dxhat.transpose())
                       + dxhat*dxhat.transpose());
    Matrix3d jv = -kd*dxhat*dxhat.transpose();
    Jx.block(i0*3,i0*3, 3,3) += jx;
    Jx.block(i0*3,i1*3, 3,3) -= jx;
    Jx.block(i1*3,i0*3, 3,3) -= jx;
    Jx.block(i1*3,i1*3, 3,3) += jx;
    Jv.block(i0*3,i0*3, 3,3) += jv;
    Jv.block(i0*3,i1*3, 3,3) -= jv;
    Jv.block(i1*3,i0*3, 3,3) -= jv;
    Jv.block(i1*3,i1*3, 3,3) += jv;
}

void createMassSpringSystem(ParticleSystem &ps, const TetMesh &bunnyMesh, double m, double ks, double kd){
	for (int i = 0; i < bunnyMesh.vertices.size(); i++) {
        Particle *p = new Particle(i, m, bunnyMesh.vertices[i], Vector3d(0,0,0));
        ps.particles.push_back(p);
    }
    for (int e = 0; e < bunnyMesh.edges.size(); e++) {
        int i = bunnyMesh.edges[e][0], j = bunnyMesh.edges[e][1];
        double l0 = (bunnyMesh.vertices[i] - bunnyMesh.vertices[j]).norm();
        SpringForce *f = new SpringForce(ps.particles[i], ps.particles[j], ks, kd, l0);
        ps.forces.push_back(f);
    }
}

void updateBunnyMesh(TetMesh &bunnyMesh, const ParticleSystem &ps){
    for (int i = 0; i < bunnyMesh.vertices.size(); i++) {
        bunnyMesh.vertices[i] = ps.particles[i]->x;
    }
}


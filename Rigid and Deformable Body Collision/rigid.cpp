#include "rigid.hpp"

#include "collision.hpp"
#include <iostream>
#include <GLFW/glfw3.h>

void RigidBody::clearForces() {
    f_ext = Vector3d(0,0,0);
    tau_ext = Vector3d(0,0,0);
}

void RigidBody::addForce(Vector3d f, Vector3d x) {
    f_ext += f;
    tau_ext += (x-xcom).cross(f);
}

void RigidBody::addImpulse(Vector3d j, Vector3d x) {
    vcom += j/m;
    L += (x-xcom).cross(j);
}

Vector3d RigidBody::velocityAt(Vector3d x) const {
    return vcom + angularVelocity().cross(x-xcom);
}

Vector3d RigidBody::angularVelocity() const {
	Matrix3d Iworld_inv;
	Iworld_inv = ( (transform().linear()*Ibody) * inverseTransform().linear() ).inverse();
	return  Iworld_inv*L;
}

Affine3d RigidBody::transform() const {
    return Translation<double,3>(xcom)*q;
}

Affine3d RigidBody::inverseTransform() const {
    return q.inverse()*Translation<double,3>(-xcom);
}

void RigidBody::draw(Vector4d fillColor, Vector4d edgeColor) const {
    glPushMatrix();
    double angle; angle = 2*acos(q.w());
    Vector3d axis; axis= q.vec();
    glTranslated(xcom[0], xcom[1], xcom[2]);
    if (angle > 0)
        glRotated(angle*180/M_PI, axis[0], axis[1], axis[2]);
    mesh->draw(fillColor, edgeColor);
    glPopMatrix();
}

void RigidBody::rewind() {
    xcom = previous.xcom;
    vcom = previous.vcom;
    q = previous.q;
    L = previous.L;
}

void stepVelocity(RigidBody &rb, double dt) {
    rb.previous.vcom = rb.vcom;
    rb.previous.L = rb.L;  
    Matrix3d Iworld_inv;
	Iworld_inv = ( (rb.transform().linear()*rb.Ibody) * rb.inverseTransform().linear() ).inverse();
    rb.vcom += (rb.f_ext/rb.m)*dt;
    rb.L += rb.tau_ext*dt;
}

void stepPosition(RigidBody &rb, double dt) {
    rb.previous.xcom = rb.xcom;
    rb.previous.q = rb.q;
    rb.xcom += rb.vcom*dt;
    Vector3d omega = rb.angularVelocity();
    double theta; theta = omega.norm()*dt;
    Vector3d qvec; qvec = sin(theta/2)*omega.normalized(); 
    if(omega.norm()==0)
    	qvec << 0,0,0;
    Quaterniond qt; qt.w() = cos(theta/2); qt.vec() = qvec;
    rb.q = (qt*rb.q).normalized();
}

void step(RigidBody &rb, double dt) {
    stepVelocity(rb, dt);
    stepPosition(rb, dt);
}

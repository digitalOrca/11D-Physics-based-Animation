#include <iostream>
using namespace std;

#include "collision.hpp"
#include "gui.hpp"
#include "mesh.hpp"
#include "particles.hpp"
#include "rigid.hpp"
#include "sdf.hpp"

void runInclinedPlane();
void runBoxToss();
void runMainDemo();

#define RUN_SCENARIO runMainDemo

int main() {
    RUN_SCENARIO();
}

void drawPoint(Vector3d x, Vector4d color) {
    glColor4f(color[0], color[1], color[2], color[3]);
    glPointSize(10.0f);	//increase size
    glBegin(GL_POINTS);
    glVertex3f(x[0], x[1], x[2]);
    glEnd();
}

void runInclinedPlane() {
    // Time step length
    double fps = 60.;
    double dt = 1/fps;
    // Inclined plane
    double slope = 0.3;
    Vector3d origin(0,0,0);
    Vector3d slopeAxis = Vector3d(1,slope,0).normalized();
    Vector3d normal = Vector3d(-slope,1,0).normalized();
    TriMesh planeMesh = TriMesh::rectangle(origin,
                                           1*Vector3d(0,0,1),
                                           10*slopeAxis);
    PlaneSDF plane(origin, normal);
    // Particle
    ParticleSystem oneParticle;
    double m = 0.1;
    Vector3d x0(1, slope + 1, 0);
    Vector3d v0(0,0,0);
    double mu = 0.8;
    oneParticle.particles.push_back(new Particle(0, m, x0, v0));
    oneParticle.setFrictionCoefficient(mu);
    Force *gravity = new GravityForce(&oneParticle, Vector3d(0,-9.8,0));
    oneParticle.forces.push_back(gravity);
    SymplecticEuler integrator(&oneParticle);
    // Create the window
    Window3D window(Vector2i(720,720));
    window.camera.lookAt(Vector3d(0,1,3), Vector3d(0,0,0), Vector3d(0,1,0));
    window.camera.setFOV(45);
    while (!window.shouldClose()) {
        integrator.step(dt);
        // Perform collision projection on sphere and floor
        collisionProjection(oneParticle, plane, dt);
        planeMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
        drawPoint(oneParticle.particles[0]->x, Vector4d(0,0,0,1));
        window.updateAndWait(1/fps);
    }
}

void runBoxToss() {
    // Time step length
    double fps = 60.;
    double dt = 0.2/fps; // 5x slowmo
    // Obstacles
    TriMesh floorMesh = TriMesh::rectangle(Vector3d(0,0,0),
                                           Vector3d(20,0,0),
                                           Vector3d(0,0,-20));
    PlaneSDF floor(Vector3d(0,0,0), Vector3d(0,1,0));
    // Box
    double w = 0.2, h = 0.3, d = 0.05;
    TriMesh boxMesh = TriMesh::box(Vector3d(0,0,0),
                                   Vector3d(w,0,0),
                                   Vector3d(0,h,0),
                                   Vector3d(0,0,d));
    double m = 0.1;
    Matrix3d I = m/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody box(&boxMesh, m, I, nullptr);
    box.xcom = Vector3d(0,0.2,0);
    box.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    box.mu = 0.2;
    box.epsilon = 0.5;
    box.addImpulse(Vector3d(0,0.4,0), box.xcom); // toss the box upwards
    //box.addImpulse(Vector3d(0,0,-0.1), box.xcom+Vector3d(0,0.1,0)); // X spin 1
    //box.addImpulse(Vector3d(0,0,0.1), box.xcom+Vector3d(0,-0.1,0)); // X spin 2
    //box.addImpulse(Vector3d(0,0,-0.1), box.xcom+Vector3d(0.1,0,0)); // Y spin 1
    //box.addImpulse(Vector3d(0,0,0.1), box.xcom+Vector3d(-0.1,0,0)); // Y spin 2
    //box.addImpulse(Vector3d(0,0.1,0), box.xcom+Vector3d(-0.1,0,0)); // Z spin 1
    //box.addImpulse(Vector3d(0,-0.1,0), box.xcom+Vector3d(0.1,0,0)); // Z spin 2
    //box.addImpulse(Vector3d(-0.1,0.1,0), box.xcom+Vector3d(0.1,0.1,0)); //Random
    // Create the window
    Window3D window(Vector2i(720,720));
    window.camera.lookAt(Vector3d(0,0.2,2), Vector3d(0,0.2,0), Vector3d(0,1,0));
    window.camera.setFOV(45);
    while (!window.shouldClose()) {
        box.clearForces();
        Vector3d g(0,-9.8,0);
        box.addForce(g*box.m, box.xcom);
        step(box, dt);
        // TODO: Perform collision resolution here
        resolveCollisions(box, &floor, dt);
        floorMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
        box.draw(Vector4d(0.6,0.2,0.2,1), Vector4d(0,0,0,0));
        window.updateAndWait(1/fps);
    }
}

void scaleMesh(TetMesh &mesh, double s) {
    for (int v = 0; v < mesh.vertices.size(); v++)
        mesh.vertices[v] *= s;
}

void translateMesh(TetMesh &mesh, Vector3d t) {
    for (int v = 0; v < mesh.vertices.size(); v++)
        mesh.vertices[v] += t;
}

void runMainDemo() {
    // Time step length
    double fps = 60.;
    int substeps = 10;
    double dt = 1/fps;
    // Obstacles
    TriMesh sphereMesh = TriMesh::sphere(Vector3d(-1.5,-0.5,0), 1.);
    SphereSDF sphere(Vector3d(-1.5,-0.5,0), 1.);
    TriMesh floorMesh = TriMesh::rectangle(Vector3d(0,0,0), Vector3d(20,0,0), Vector3d(0,0,-20));
    PlaneSDF floor(Vector3d(0,0,0), Vector3d(0,1,0));
    // Mass-spring bunny
    TetMesh bunnyMesh("bunny");
    scaleMesh(bunnyMesh, 5);
    translateMesh(bunnyMesh, Vector3d(-1,0.5,0));
    ParticleSystem bunny;
    // Create bunny mass-spring system from bunnyMesh
    createMassSpringSystem(bunny, bunnyMesh, 0.01, 500., 0.5); //m, k, c
    // Set friction coefficient
    bunny.setFrictionCoefficient(0.2);
    // Include gravity
    Force *gravity = new GravityForce(&bunny, Vector3d(0,-9.8,0));
    bunny.forces.push_back(gravity);
    // Instantiate time integrator
    SymplecticEuler integrator(&bunny);
    // Rigid wheel
    TriMesh wheelMesh("wheel.obj");
    double m = 1;
    double r = 0.4, h = 0.15;
    Matrix3d I = m/12*Vector3d(3*r*r+h*h, 3*r*r+h*h, 6*r*r).asDiagonal();
    GridSDF wheelSDF("wheel.sdf");
    RigidBody wheel(&wheelMesh, m, I, &wheelSDF);
    wheel.xcom = Vector3d(1,1,0.3);
    wheel.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    wheel.vcom = Vector3d(0,0,0);
    wheel.L = 1.5*Vector3d(0,0,1);
    wheel.mu = 0.8;
    wheel.epsilon = 0.5;
    // Create the window
    Window3D window(Vector2i(720,720));
    window.camera.lookAt(Vector3d(0,1,5), Vector3d(0,0,0), Vector3d(0,1,0));
    window.camera.setFOV(45);
    bool running = false;
    while (!window.shouldClose()) {
    	if (window.keyPressed(GLFW_KEY_SPACE))
            running = true;
            // Perform collision detection between bunny and wheel
            wheel.clearForces();
            bunny.clearForces();
            addPenaltyForces(bunny, wheel, 500, 0.5);	//k, d
        if (running) {
        	for (int i = 0; i < substeps; i++) {
            	integrator.step(dt/substeps);
            	// Perform collision projection on sphere and floor
            	collisionProjection(bunny, sphere, dt/substeps);
            	collisionProjection(bunny, floor, dt/substeps);
        	}
        	Vector3d g(0,-9.8,0);
        	wheel.addForce(g*wheel.m, wheel.xcom);
        	step(wheel, dt);
        	resolveCollisions(wheel, &sphere, dt);
        	resolveCollisions(wheel, &floor, dt);
            // TODO: Perform collision resolution of wheel with sphere and floor
        }
        updateBunnyMesh(bunnyMesh, bunny);
        floorMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
        sphereMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
        bunnyMesh.draw(Vector4d(1.0,0.8,0.2,1), Vector4d(0,0,0,0));
        wheel.draw(Vector4d(0.6,0.7,0.9,1), Vector4d(0,0,0,0));
        window.updateAndWait(1/fps);
    }
}

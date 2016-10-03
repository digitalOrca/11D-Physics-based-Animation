#include "gui.hpp"
#include "mesh.hpp"
#include "oscillator.hpp"
#include "particles.hpp"
#include "timeintegration.hpp"
#include <iostream>
//#include <vector>
//#include <Eigen/Dense>

using namespace std;
//using namespace Eigen;

void runOscillator();
void runMassSpring();
void updateMesh(Mesh2D &mesh, ParticleSystem &ms);
void updateMouseForce(ParticleSystem ms, SpringForce &mouseForce, Vector2d pos, bool down);
// Set RUN_SCENARIO to runOscillator or runMassSpring
#define RUN_SCENARIO runMassSpring

// Set INTEGRATOR to ForwardEuler, RungeKutta2, or BackwardEuler
// then inside runOscillator() and runMassSpring() do
//     TimeIntegrator *integrator = new INTEGRATOR(...);
#define INTEGRATOR BackwardEuler

int main() {
    RUN_SCENARIO();
}

void runOscillator() {
	double fps = 60;
    double dt = 1/fps;
    
	DampedHarmonicOscillator dho(1, 100, 2);	//m, k, c
    INTEGRATOR Simulate(&dho);
    //initial condition
    dho.x = 1; 
    dho.v = 0;
    
    while(Simulate.t < 1){	//simulate for 1 second
    	Simulate.step(dt);
    	cout << "Time: " << Simulate.t << "\tPosition: " << Simulate.pos << endl;
    }    
}

void runMassSpring() {
    // Mesh
    string filename = "donut.1";
    Mesh2D mesh(filename);
    // Time step length
    double fps = 100.;
    double dt = 1/fps;
	/*------------------------INITIALIZATION------------------------*/
    ParticleSystem ms;	//ms: mass-spring system
	ms.load(mesh);
	INTEGRATOR Simulate(&ms);
	SpringForce mouseForce(nullptr, Vector2d(0,0), 10, 5);
	ms.forceArray[ms.forces.size()-1] = mouseForce;
	ms.forces[ms.forces.size()-1] = &mouseForce;
	/*------------------------LOOP------------------------*/
	Window2D window(Vector2i(720,720), Vector2d(0,0), Vector2d(1,1));
    while (!window.shouldClose()) {
		Simulate.step(dt);
		updateMouseForce(ms, mouseForce, window.mousePos(), window.mouseDown());
		updateMesh(mesh, ms);
        mesh.draw(Vector4d(0.2,0.6,0.2,1), Vector4d(0,0,0,1));
        window.updateAndWait(dt);
    }
}

/*------------------------END MAIN------------------------*/
void updateMouseForce(ParticleSystem ms, SpringForce &mouseForce,
                      Vector2d pos, bool down) {
    mouseForce.x = pos;
    if (down && mouseForce.target == nullptr) {
        // find nearest particle
        double dmin = 0.2; // ignore particles farther than 0.2
        for (int i = 0; i < ms.particles.size(); i++) {
            Particle *p = ms.particles[i];
            double d = (p->x - pos).norm();
            if (d < dmin) {
                mouseForce.target = p;
                dmin = d;
            }
        }
    } else if (!down)
        mouseForce.target = nullptr;
}

void updateMesh(Mesh2D &mesh, ParticleSystem &ms){
	for(int i=0; i<ms.particles.size(); i++){
		mesh.vertices[i] = ms.particles[i]->x;
	}
}


#include "particles.hpp"
#include <iostream>	//for debug
using namespace std;

void ParticleSystem::load(Mesh2D &mesh){
	int numVertex = mesh.vertices.size();
	int numEdge = mesh.edges.size();

	//initialize particles and forces
	this->particleArray = vector<Particle> (numVertex, Particle());	//initialize all the particles
	this->particles.resize(numVertex);	//reserve particle pointer array
	this->forceArray = vector<SpringForce> (numEdge+1, SpringForce());	//initialize all the forces
	this->forces.resize(numEdge+1);		//reserve force pointer array

	for(int i=0; i<numVertex; i++){	//initialize vertices coordinates
			particles[i] = &(particleArray[i]);
			particleArray[i].x = mesh.vertices[i];
			particleArray[i].m = 1.0;	//define vertex mass-----------------------TEMP
	}	
	
	for(int i=0; i<numEdge; i++){	//initialize spring neutral length
			forces[i] = &(forceArray[i]);
			
			Vector2d v1 = mesh.vertices[ (mesh.edges[i])[0] ];
			Vector2d v2 = mesh.vertices[ (mesh.edges[i])[1] ];
			forceArray[i].len = (v1-v2).norm();
			
			forceArray[i].target = particles[ (mesh.edges[i])[0] ];	//store pushed particle pointer
			forceArray[i].ends = mesh.edges[i];	//store ends vertices index
	}	
}

int ParticleSystem::getNumDOFs(){
	return 2*particleArray.size();
};

void ParticleSystem::getState(VectorXd &x, VectorXd &v, double &t){
	for(int i= 0; i < particles.size(); i++){
		x(2*i) = (particles[i]->x)(0);
		x(2*i+1) = (particles[i]->x)(1);
		v(2*i) = (particles[i]->v)(0);
		v(2*i+1) = (particles[i]->v)(1);
	}
	t = time;
};

void ParticleSystem::setState(const VectorXd &x, const VectorXd &v, double t){
	for(int i = 0; i< particles.size(); i++){
		(particles[i]->x)(0) = x(2*i);
		(particles[i]->x)(1) = x(2*i+1);
		(particles[i]->v)(0) = v(2*i);
		(particles[i]->v)(1) = v(2*i+1);
	}
	time = t;
};

void ParticleSystem::getInertia(MatrixXd &M){
	for(int i= 0; i < particles.size(); i++){
		M(i, i) = particles[i]->m;	//determine inverse of inertia matrix
	}
};

void ParticleSystem::getForces(VectorXd &f){
	//Clear all the particle forces
	for(int i=0; i<particles.size(); i++){
		(particles[i]->f).fill(0.0);
	}
	
	//Iterate through all force objects and add terms to each particle
	for(int i=0; i<forces.size()-1; i++){
	
		double len = forceArray[i].len;		//original spring length
	
		Particle *end1 = particles[ forceArray[i].ends[0] ];	//target
		Particle *end2 = particles[ forceArray[i].ends[1] ];	
		
		Vector2d x1 = end1->x;
		Vector2d x2 = end2->x;
		double length = (x1-x2).norm();		//current spring length
		Vector2d unit = (x1-x2) / length;
		double dx = length - len;
		
		Vector2d v1 = end1->v;
		Vector2d v2 = end2->v;
		double dv = unit.dot(v1-v2);
		
		double magnitude = -forceArray[i].ks * dx - forceArray[i].kd * dv;
		Vector2d force = (x1-x2) / len * magnitude;
		
		forceArray[i].target->f += force;			//Calculate forces -> put in particle objects -> compile to f matrix
	}
	//add mouse force to the connecting particle
	SpringForce *sf = forces[forces.size()-1];
	if( (sf->target) != nullptr ){
		Vector2d dx = ((sf->target)->x) - (sf->x);
		Vector2d unit = dx/(dx.norm());
		Vector2d kforce = unit * (sf->ks) * dx.norm();
		Vector2d dforce = (sf->kd) * unit * (unit.dot( (sf->target)->v ) );
		(sf->target)->f += (-kforce + dforce);
	}	
	//Iterate through all particles and add terms to force matrix
	f.fill(0.0); //clear previous current forces	
	for(int i=0; i<particles.size(); i++){
		f(2*i) = (particles[i]->f)[0];
		f(2*i+1) = (particles[i]->f)[1];
	}
};

void ParticleSystem::getAccelerations(VectorXd &a){
	//Iterate through all particles and calculate acceleration
	for(int i=0; i<particles.size(); i++){
		a(2*i) = (particles[i]->f)[0] / (particles[i]->m);
		a(2*i+1) = (particles[i]->f)[1] / (particles[i]->m);
	}
};

void ParticleSystem::getJacobians(MatrixXd &Jx, MatrixXd &Jv){
	//clear previous jacobian
	Jx.fill(0.0);
	Jv.fill(0.0);
	//Iterate through all force obejcts to construct local jacobian matrix
	MatrixXd xlocal(2,2);
	MatrixXd vlocal(2,2);
	Vector2d xij;
	Vector2d xtil;
	MatrixXd xx(2,2);
	Particle *pi;
	Particle *pj;
	int index1;
	int index2;
	
	for(int n=0; n<forces.size()-1; n++){ //skip the last one (mouseForce)
		index1 = (forces[n]->ends)[0];
		index2 = (forces[n]->ends)[1];
		pi = particles[index1];
		pj = particles[index2];
		//create local jacobian
		xij = (pi->x) - (pj->x);
		xtil = xij/(xij.norm());
		xx = xtil*(xtil.transpose()); 
		xlocal = -forceArray[0].ks * ( max(0.0, 1.0-( forceArray[n].len /(xij.norm()) ))*(MatrixXd::Identity(2,2)-xx) + xx );	//ks,kd arbitrarily chosen from the list
		vlocal = -forceArray[0].kd * xx;
		
		//insert local jacobian into global jacobian
		Jx.block(2*index1, 2*index1, 2, 2) += xlocal; // ( index of force contribution , index of particle )
		Jv.block(2*index1, 2*index1, 2, 2) += vlocal;
	}	
};

#include "timeintegration.hpp"

#include <Eigen/Sparse>

VectorXd conjugateGradientSolve(const MatrixXd &A, const VectorXd &b) {
    SparseMatrix<double> spA = A.sparseView();
    ConjugateGradient< SparseMatrix<double> > solver;
    solver.setTolerance(1e-6);
    return solver.compute(spA).solve(b);
}

ForwardEuler::ForwardEuler(PhysicalSystem *system){
	this->sys = system;
	this->NumDOFs = system->getNumDOFs();
	this->pos.resize(system->getNumDOFs());
	this->vel.resize(system->getNumDOFs());
	this->a.resize(system->getNumDOFs());
	this->f.resize(system->getNumDOFs());
	this->M.resize(system->getNumDOFs(), system->getNumDOFs());
}

void ForwardEuler::step(double dt){
	//load from PhysicalSystem
	sys->getState(this->pos, this->vel, this->t);
	sys->getInertia(this->M);
	sys->getForces(this->f);
	sys->getAccelerations(this->a);
	//compute new state
	this->pos += vel * dt;
	this->vel += a * dt;
	this->t += dt;
	//set PhysicalSystem new state
	sys->setState(this->pos, this->vel, this->t);
}

RungeKutta2::RungeKutta2(PhysicalSystem *system){
	this->sys = system;
	this->NumDOFs = system->getNumDOFs();
	this->pos.resize(system->getNumDOFs());
	this->vel.resize(system->getNumDOFs());
	this->a.resize(system->getNumDOFs());
	this->a_new.resize(system->getNumDOFs());
	this->f.resize(system->getNumDOFs());
	this->M.resize(system->getNumDOFs(), system->getNumDOFs());
}

void RungeKutta2::step(double dt){
	//load from PhysicalSystem
	sys->getState(this->pos, this->vel, this->t);
	sys->getInertia(this->M);
	sys->getForces(this->f);
	sys->getAccelerations(this->a);
	//get acceleration for the next step
	sys->setState(pos+vel*dt, vel+a*dt, t);
	sys->getForces(this->f);	//must calculate force before acceleration
	sys->getAccelerations(a_new);	//get acceleration at half time step
	this->pos += (vel+vel+a*dt)/2*dt;	//take a full step
	this->vel += (a+a_new)/2*dt;
	this->t += dt;
	//set PhysicalSystem new state
	sys->setState(this->pos, this->vel, this->t);
}

BackwardEuler::BackwardEuler(PhysicalSystem *system){
	this->sys = system;
	this->NumDOFs = system->getNumDOFs();
	this->pos.resize(system->getNumDOFs());
	this->vel.resize(system->getNumDOFs());
	this->a.resize(system->getNumDOFs());
	this->f.resize(system->getNumDOFs());
	this->M.resize(system->getNumDOFs(), system->getNumDOFs());
	this->Jx.resize(system->getNumDOFs(), system->getNumDOFs());
	this->Jv.resize(system->getNumDOFs(), system->getNumDOFs());
	this->A.resize(system->getNumDOFs(), system->getNumDOFs());
	this->b.resize(system->getNumDOFs());
}

void BackwardEuler::step(double dt){
	//load from PhysicalSystem
	sys->getState(this->pos, this->vel, this->t);
	sys->getInertia(this->M);
	sys->getForces(this->f);
	sys->getJacobians(this->Jx, this->Jv);
	//set up for using the CG solver
	A = M - Jx*dt*dt - Jv*dt;
	b = f + Jx*vel*dt;
	//update states
	this->vel += conjugateGradientSolve(A, b)*dt;
	this->pos += vel * dt;	//update pos after vel
	this->t += dt;
	//set PhysicalSystem to new state
	sys->setState(this->pos, this->vel, this->t);
}

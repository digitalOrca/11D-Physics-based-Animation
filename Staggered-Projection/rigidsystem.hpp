#ifndef RIGIDSYSTEM_HPP
#define RIGIDSYSTEM_HPP

#include "rigid.hpp"

class RigidSystem{
public:
	vector<RigidBody*> bodies;	// rigid body pointers
	PlaneSDF *environment; // environment
	MatrixXd M;		// mass matrix
	VectorXd v;		// velocity
	VectorXd vp;	// predictor velocity
		
	VectorXd f;		// friction impulse
	VectorXd c;		// contact impulse
	VectorXd f_tmp; // friction impulse for substep
	VectorXd c_tmp; // contact impulse for substep
	
	VectorXd n_q;	// nk matrix, entry for N
	MatrixXd d_q;	// dk matrix, entry for D
	MatrixXd gamma_k;  // temporary gamma
	
	VectorXd alpha;	// normal impulse magnitude
	VectorXd beta;  // frictional impulse magnitude
	MatrixXd N;		// normal impulse unit vector
	MatrixXd D; 	// frictional impulse unit vector
	
	vector<Vector3i> contacts; 	//body index, vertex index, sdf source index
	int numTSample = 4;
	
	int getDoF();		// get system degree of freedom
	void resizeSystem();	// resize vector and matrix
	void updateM();
	void clearForces();
	void forceField(Vector3d g);
	void predictSystem(double dt);		// compute q_dot_p
	void drawSystem();
	void collisionDetection();
	void updateContactMatrix();		// resize matrix dimension for N, D, alpha, beta	
	void optimize(double err, int max_iters, double dt);
	MatrixXd addGammaBlock(Vector3d &v);
	void updateNkDk(Vector3i &c);	// Update Nk and Dk
	void generalizeImpulse();	// Build N matrix
	VectorXd staggeredNormalProjection(MatrixXd &N, MatrixXd &M, VectorXd &vp, VectorXd &f);
	VectorXd staggeredFrictionalProjection(double mu, MatrixXd &N, MatrixXd &D, MatrixXd &M, VectorXd &v, VectorXd &alpha);
	double relativeError(VectorXd &f, VectorXd &f_prev);
	double residual();
	void updateSystem(double dt);
	void updateReactionForce();
	void drawContact(vector<Vector3i> &c);
	void drawNormalForce(vector<Vector3i> &c);
	void saveState(ofstream &myfile);
};

#endif

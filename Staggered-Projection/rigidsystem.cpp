#include "rigidsystem.hpp"
#include "qpsolver.hpp"
#include <iostream>
#include <fstream>

int RigidSystem::getDoF(){
	return 6*this->bodies.size();
}

void RigidSystem::resizeSystem(){
	int dof = getDoF();
	this->v.resize(dof);	this->v.setZero();
	this->vp.resize(dof);	
	this->f.resize(dof);	this->f.setZero();
	this->f_tmp.resize(dof);	this->f_tmp.setZero();
	this->c.resize(dof);
	this->c_tmp.resize(dof);
	this->M.resize(dof, dof);	this->M.setZero();
	this->n_q.resize(dof);		
	this->d_q.resize(dof, numTSample);
}

void RigidSystem::updateM(){
	this->M.setZero();
	for(int i=0; i<bodies.size(); i++){
		Matrix3d m(3,3);
		m = MatrixXd::Identity(3,3) * bodies[i]->m;
		Matrix3d I(3,3);
		I = (bodies[i]->transform().linear()*bodies[i]->Ibody) * bodies[i]->inverseTransform().linear();
		MatrixXd M_body(6,6);
		M_body.setZero();
		M_body.block(0,0,3,3) = m;
		M_body.block(3,3,3,3) = I;
		this->M.block(6*i, 6*i, 6, 6) = M_body;
	}
}

void RigidSystem::clearForces(){
	for(int i=0; i<bodies.size(); i++){
		bodies[i]->clearForces();
	}
}

void RigidSystem::forceField(Vector3d g){
	for(int i=0; i<bodies.size(); i++){
		bodies[i]->addForce(g*bodies[i]->m, bodies[i]->xcom);
	}
}

void RigidSystem::predictSystem(double dt){
	for(int i=0; i<bodies.size(); i++){
		stepVelocity(*bodies[i], dt);	//step forward to get prediction
		Vector3d vel, w;
		w = bodies[i]->angularVelocity();
		vel = bodies[i]->vcom;
		// save (vcom,w) to vp
		VectorXd q_dot(6); q_dot << vel, w;
		vp.segment(6*i, 6) = q_dot;
		stepPosition(*bodies[i], dt);
		bodies[i]->rewind(); //restore state
	}
}

void RigidSystem::drawSystem(){
	for(int i=0; i<bodies.size(); i++){
		bodies[i]->draw(Vector4d(0.6,0.2,0.2,1), Vector4d(0,0,0,0));
	}
}

void RigidSystem::collisionDetection(){
	for(int i=0; i<bodies.size(); i++){		// iterate through all sdf
		for(int j=0; j<bodies.size(); j++){		// iterate through all other objects
			if(i!=j){
				vector<Vector3d> v;
				v = bodies[j]->mesh->vertices;
				for(int p=0; p<v.size(); p++){	// iterate through all vertices
					Vector3d p_world = bodies[j]->transform()*v[p];
					Vector3d p_body = bodies[i]->inverseTransform()*p_world;
					double d = bodies[i]->sdf->signedDistance( p_body );
					if(d<0){
						contacts.push_back( Vector3i(j, p, i) );
					}
				}
			}
		}
	}
	
	for(int j=0; j<bodies.size(); j++){		// iterate through all other objects
		vector<Vector3d> v;
		v = bodies[j]->mesh->vertices;
		for(int p=0; p<bodies[j]->mesh->vertices.size(); p++){
			Vector3d p_world = bodies[j]->transform()*v[p];		
			double d = environment->signedDistance( p_world );
			if(d<0){
				contacts.push_back( Vector3i(j, p, -1) ); // -1 for environment
			}
		}
	} 
}

void RigidSystem::updateContactMatrix(){
	int l = this->contacts.size();
	this->N.resize(getDoF(), l);
	this->D.resize(getDoF(), l*numTSample);
	this->alpha.resize(l);
	this->beta.resize(l*numTSample);	
}

void RigidSystem::optimize(double err, int max_iters, double dt){

	predictSystem(dt);		//1
	if(this->contacts.size() < 1){	
		this->v = this->vp;
	}else{	
/*		VectorXd f_iter = this->f;		// 2
		VectorXd f_iter_prev = this->f;
		double min_res = INFINITY;
		double res_val = INFINITY;
		double rel_err = INFINITY;
		int i=0;					// 3
		
		while( rel_err>err && i<max_iters ){
			c_tmp = staggeredNormalProjection(this->N, this->M, this->vp, f_iter);		// 5
			f_iter = staggeredFrictionalProjection(0.8 , this->N, this->D, this->M, this->vp, this->alpha);	// 6
			rel_err = relativeError(f_iter, f_iter_prev);	// 7
			res_val = residual();				// 8
			f_iter_prev = f_iter;
			
			cout.precision(5);
			cout << "iteration: " << i+1 << "\trel_err: " << rel_err << "\tresidual: " << res_val << endl;
			
			if( res_val < min_res ){
				min_res = res_val;
				this->f_tmp = f_iter;
			}  
			i++;
		}
		cout << "--------------------------------------------------------------------------------------" << endl;
		this->f = f_tmp;					*/	//15		
		c = staggeredNormalProjection(this->N, this->M, this->vp, this->f);		//16
		this->v = this->vp + this->M.inverse()*(c+f);		//17
	}
}

MatrixXd RigidSystem::addGammaBlock(Vector3d &v){
	MatrixXd gammaBlock(3, 6);
	gammaBlock.block(0,0,3,3) = Matrix3d::Identity(3,3);
	Matrix3d r;
	r <<    0, v[2], -v[1],
		-v[2],    0,  v[0],
		 v[1],-v[0],     0;
	gammaBlock.block(0,3,3,3) = r;
	return gammaBlock;
}

void RigidSystem::updateNkDk(Vector3i &c){ //body index, vertex index, sdf source index
	// Build Gamma_ij
	Vector3d ri_world, rj_world, ri, rj;
	MatrixXd gamma_ij(3,getDoF());
	gamma_ij.setZero();
	rj_world = bodies[c[0]]->transform()*(bodies[c[0]]->mesh->vertices[c[1]]);	// contact point in world coordinate
	rj = rj_world - bodies[c[0]]->xcom;   // correct by xcom
	if(c[2]!=-1){
		gamma_ij.block(0,6*c[0],3,6) = -addGammaBlock( rj );	// add to block corresponding to body index
	}else{
		gamma_ij.block(0,6*c[0],3,6) = addGammaBlock( rj );	// add to block corresponding to body index
	}
	Vector3d normal = this->environment->normal(rj_world);		// default environment normal
	if(c[2]!=-1){
		normal = (bodies[c[2]]->transform().linear())*(bodies[c[2]]->sdf->normal( bodies[c[2]]->inverseTransform()*rj_world ));
		ri_world = rj_world - bodies[c[2]]->sdf->signedDistance( bodies[c[2]]->inverseTransform()*rj_world )*normal;
		ri = ri_world - bodies[c[2]]->xcom;
		gamma_ij.block(0,6*c[2],3,6) = addGammaBlock( ri );
		//cout << "gamma_ij: " << gamma_ij << endl;
		//cout << "----------------------------------------------------------------------" << endl;
	}
	this->n_q = gamma_ij.transpose()*normal;
	//------------Debug--------------
	Vector3d vi, vj;
	MatrixXd gamma_i(3,getDoF());	gamma_i.setZero();
	MatrixXd gamma_j(3,getDoF());	gamma_j.setZero();
	double rv;
	gamma_j.block(0,6*c[0],3,6) = addGammaBlock(rj);
	if(c[2]!=-1){
		gamma_i.block(0,6*c[2],3,6) = addGammaBlock(ri);
		vi = gamma_i*this->vp;
		vj = gamma_j*this->vp;
		rv = (vi-vj).dot(normal.normalized());
		if(rv > 0)
			this->n_q = -gamma_ij.transpose()*normal;
	}
	//-------------------------------*/
	// Build Tk
	vector<Vector3d> t_prototype;
	MatrixXd Tk(3, numTSample);
	if(c[2] != -1){
		Vector3d p = bodies[c[2]]->inverseTransform()*(bodies[c[0]]->transform()*bodies[c[0]]->mesh->vertices[c[1]]);		// contact point in SDF body space
		vector<Vector3d> t_prototype = bodies[c[2]]->sdf->sampleT(p, numTSample);
		for(int i=0; i<t_prototype.size(); i++){
			Tk.col(i) = bodies[c[2]]->transform().linear()*t_prototype[i];
		}
	}else{
		Vector3d p = bodies[c[0]]->transform()*bodies[c[0]]->mesh->vertices[c[1]];		// contact point in world to sdf environment
		vector<Vector3d> t_prototype = environment->sampleT(p, numTSample);
		for(int i=0; i<t_prototype.size(); i++){
			Tk.col(i) = t_prototype[i];
		}	
	}
	this->d_q = gamma_ij.transpose()*Tk;
}

void RigidSystem::generalizeImpulse(){
	int l = this->contacts.size();
	this->N.setZero();
	this->D.setZero();
	// Build N and D matrix
	for(int i=0; i<l; i++){
		updateNkDk( contacts[i] );
		this->N.col(i) = this->n_q;
		//cout << "N: " << this->N.transpose() << endl;
		this->D.block(0, numTSample*i, getDoF(), numTSample) = this->d_q;
	}
}

VectorXd RigidSystem::staggeredNormalProjection(MatrixXd &N, MatrixXd &M, VectorXd &vp, VectorXd &f){
	this->alpha = NormalOptimization(N, M, vp, f_tmp);
	return this->N*this->alpha;
}

VectorXd RigidSystem::staggeredFrictionalProjection(double mu, MatrixXd &N, MatrixXd &D, MatrixXd &M, VectorXd &v, VectorXd &alpha){	
	this->beta = FrictionOptimization(mu , N, D, M, v, alpha);
	return this->D*this->beta;
}

double RigidSystem::relativeError(VectorXd &f, VectorXd &f_prev){
	double num = (f-f_prev).transpose()*M.inverse()*(f-f_prev);
	double denum = f_prev.transpose()*M.inverse()*f_prev;
	return num/denum;
}

double RigidSystem::residual(){
	double res = 0;
	for(int i=0; i<contacts.size(); i++){
		res += abs( alpha[i]*(N.col(i).transpose()) * (vp + M.inverse()*N*alpha + M.inverse()*D*beta) );
	}
	return res;
}

void RigidSystem::updateSystem(double dt){
	for(int i=0; i<bodies.size(); i++){
		// update previous state
		bodies[i]->previous.vcom = bodies[i]->vcom;
    	bodies[i]->previous.L = bodies[i]->L;
		bodies[i]->previous.xcom = bodies[i]->xcom;
    	bodies[i]->previous.q = bodies[i]->q;
		// update velocity
    	Matrix3d Iworld, Iworld_inv;
    	Iworld = (bodies[i]->transform().linear()*bodies[i]->Ibody) * bodies[i]->inverseTransform().linear();
		Iworld_inv = Iworld.inverse();
		bodies[i]->vcom = this->v.segment(6*i,3);
		bodies[i]->L = Iworld * this->v.segment(6*i+3,3);
		// update rotation
		Vector3d vel = bodies[i]->vcom;
		Vector3d omega = Iworld_inv*bodies[i]->L;
		// update position
		bodies[i]->xcom += vel*dt;
		// update orientation
    	double theta; theta = omega.norm()*dt;
    	Vector3d qvec; qvec = sin(theta/2)*omega.normalized(); 
    	if(omega.norm()==0)
    		qvec << 0,0,0;
    	Quaterniond qt; qt.w() = cos(theta/2); qt.vec() = qvec;
    	bodies[i]->q = (qt*bodies[i]->q).normalized();
	}
}

void RigidSystem::drawContact(vector<Vector3i> &c){
	for(int i=0; i<c.size(); i++){
		Vector3i k = c[i];
		vector<Vector3d> contactPoint;
		contactPoint.push_back( this->bodies[k[0]]->transform() * (this->bodies[k[0]]->mesh->vertices[k[1]]) );
		Vector4d color;
		color << 0,1,0,1;
		this->bodies[k[0]]->mesh->drawVertex(contactPoint, color);
	}
}

void RigidSystem::drawNormalForce(vector<Vector3i> &c){
	for(int i=0; i<c.size(); i++){
		Vector3i k = c[i];
		Vector3d n;
		Vector3d start; start = (this->bodies[k[0]]->transform())*(this->bodies[k[0]]->mesh->vertices[k[1]]);
		if(k[2]!=-1){
			n = this->bodies[k[2]]->transform().linear()*(this->bodies[k[2]]->sdf->normal(  this->bodies[k[2]]->inverseTransform()*start ));
		}else{
			n = this->environment->normal(start);
		}
		Vector3d end = start + n*this->alpha[i]*250;	// 500 factor for visualization
		this->bodies[k[0]]->mesh->drawLine(start, end);	
	}
}

void RigidSystem::saveState(ofstream &myfile){
  	if (myfile.is_open()){
  		for(int i=0; i<this->bodies.size(); i++){
  			Vector3d xcom, vec; 
  			xcom = bodies[i]->xcom;
  			vec = bodies[i]->q.vec();
  			myfile << xcom[0] << " " << xcom[1] << " " << xcom[2] << " ";
  			myfile << bodies[i]->q.w() << " " << vec[0] << " " << vec[1] << " " << vec[2] << " ";
  		}
  		myfile << "\n";
  		//myfile.close();
  	}else{
  		cout << "Unable to open file." << endl;
  	}
}

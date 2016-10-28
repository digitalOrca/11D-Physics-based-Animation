#include "collision.hpp"
#include <iostream>
void findCollisions(ParticleSystem &ps, SDF *sdf,
                    vector<Collision> &collisions) {
    // TODO if implementing projection method using Collision objects
}

Collision findCollision(RigidBody &rb, SDF *sdf) {	// or return an empty Collision() if no vertex penetrates
    double max_val = 0;
    int max_i = -1;
    Vector3d max_p(0.0, 0.0, 0.0);
    for(int i=0; i<rb.mesh->vertices.size(); i++){
    	Vector3d pworld = rb.transform()*(rb.mesh->vertices[i]);
    	double d = -sdf->signedDistance(pworld);
    	if(d>0 && d > max_val){
    		max_val = d;
    		max_i = i;
    		max_p = pworld;
    	}
    }
    if(max_i != -1){
    	Collision collision;
    		collision.index0 = max_i;
    		collision.x0 = max_p;
    		collision.n = sdf->normal(max_p);
    		collision.d = max_val; //penetration depth is a positive value
    	return collision;
    }
    return Collision();
}

void findCollisions(ParticleSystem &ps, RigidBody &rb,
                    vector<Collision> &collisions) {
    // and insert Collision objects for them into the collisions array
    Vector3d x, xb;
    double d;
    for(int i=0; i<ps.particles.size(); i++){
    	x = ps.particles[i]->x;
    	xb = rb.inverseTransform()*x;	//point in body coordinate
    	d = -rb.sdf->signedDistance(xb);
    	if(d>0){  // values in collision are in world coordinate
    		Collision collision;
    		collision.index0 = i;
    		collision.x0 = rb.transform()*xb;	
    		collision.x1 = rb.transform()*(xb +d*rb.sdf->normal(xb));	//nearest point on the sdf boundary 
    		collision.n = (rb.transform().linear()*rb.sdf->normal(xb)).normalized();
    		collision.d = d;
    		collisions.push_back(collision);
    	}
    }
}

void collisionProjection(ParticleSystem &ps, SDF &sdf, double dt){
	int dof = ps.getNumDOFs();
	Vector3d v_cons, x_cons, n, dv, vt, fn, ft, fnet;
	Matrix3d Jt;
	VectorXd f(dof); ps.getForces(f);
	double val;
	
	for(int i=0; i<ps.particles.size(); i++){
		val = sdf.signedDistance(ps.particles[i]->x);
		if(val < 0){
			n = sdf.normal(ps.particles[i]->x);
			// Constrained x and v
			dv = (-val * n) / dt;
			v_cons = ps.particles[i]->v + dv;
			x_cons = ps.particles[i]->x + dv*dt;
			// Compute friction
			fn = ps.particles[i]->m*dv/dt;
			Vector3d temp = v_cons + ((fn+ft)/ps.particles[i]->m)*dt;
			Vector3d vt = temp - (temp.dot(n))*n;
			fnet = ps.particles[i]->m*vt/dt;
			
			if(fnet.norm() < ps.mu*(fn.norm())){
				ft = -fnet;
			}else{
				ft = -ps.mu*(fn.norm())*fnet.normalized();
			}
			ps.particles[i]->v += dv + ((ft)/ps.particles[i]->m)*dt;
			ps.particles[i]->x += (dv + ((ft)/ps.particles[i]->m)*dt)*dt;
		}
	}
}

void resolveCollisions(RigidBody &rb, SDF *sdf, double dt) {
    // collision impulse, then step the body forward again
    Collision collision;
    collision = findCollision(rb, sdf);
    if(collision.d > 0){	//collision detection
    	rb.rewind();
    	Vector3d x; x = collision.x0;
    	//cout << "angv: " << rb.angularVelocity().transpose() << endl;
    	//cout << "r: " << (x-rb.xcom).transpose() << endl;
    	Vector3d vn; vn = ( rb.velocityAt(x).dot(sdf->normal(x)) * sdf->normal(x).normalized() );
    	Vector3d vt; vt = rb.velocityAt(x) - vn;
    	// Sticking condition
    	Vector3d dv0; dv0 = -rb.epsilon*vn - rb.velocityAt(x);
    	Vector3d r; r = x - rb.xcom;
    	Matrix3d rx; rx <<    0,-r[2], r[1],
    					   r[2],	0,-r[0],
    					  -r[1], r[0],	  0;  
    	Matrix3d Iworld_inv; Iworld_inv = ( (rb.transform().linear()*rb.Ibody) * rb.inverseTransform().linear() ).inverse();
    	Matrix3d K; K = (1.0/rb.m)*Matrix3d::Identity() + rx.transpose()*Iworld_inv*rx;
    	// Sticking j
    	Vector3d j; j = K.inverse()*dv0;
    	// Check for friction limit
    	Vector3d dvn, T, jn;
    	double NKN;
    	if( (j-( j.dot(sdf->normal(x)) * sdf->normal(x) )).norm() > rb.mu*j.dot(sdf->normal(x))){
    		dvn = -(1+rb.epsilon)*vn;
    		T = vt.normalized(); //(rb.velocityAt(x) - vn).norm() * vn.normalized();  //directoin determined by j	
    		if(vt.norm()==0){ // dv will depend on the impulse direction
    			jn = j.dot(sdf->normal(x))*(sdf->normal(x));
    			T = (j-jn).normalized();
    		}
    		NKN = collision.n.transpose() * K * (collision.n - rb.mu*T);
    		jn = dvn/NKN;
    		j = jn + rb.mu*jn.norm()*(-T);
    	}
    	// Apply impulse and step forward again
    	rb.addImpulse(j, collision.x0);
    	step(rb, dt);
    	// Check for collision again
    	Collision collision2 = findCollision(rb, sdf);
    	if(collision2.d > 0){
    		resolveContacts(rb, sdf, dt);
    	}
    }    
}

void resolveContacts(RigidBody &rb, SDF *sdf, double dt) {
    // compute and apply the contact impulse, then step the position
    rb.rewind();
    // Perform velocity update before computing impulse
    stepVelocity(rb, dt);
    Collision collision;
    collision = findCollision(rb, sdf);
    Vector3d x = collision.x0;
    // dv required to correct dx
    Vector3d vn; vn = ( rb.velocityAt(x).dot(sdf->normal(x)) * sdf->normal(x).normalized() );
    Vector3d vt; vt = rb.velocityAt(x) - vn;
    // Sticking condition
    Vector3d dv0 = -vt + collision.d*collision.n/dt;
    Vector3d r = collision.x0 - rb.xcom;
    Matrix3d rx; rx <<    0,-r[2], r[1],
    				   r[2],	0,-r[0],
    				  -r[1], r[0],	  0;
    Matrix3d Iworld_inv; Iworld_inv = ( (rb.transform().linear()*rb.Ibody) * rb.inverseTransform().linear() ).inverse();
    Matrix3d K; K = (1.0/rb.m)*Matrix3d::Identity() + rx.transpose()*Iworld_inv*rx;
    Vector3d j; j = K.inverse()*dv0;
    // Check for friction limit
    Vector3d dvn, T, jn;
    double NKN;
    if( (j-( j.dot(sdf->normal(x)) * sdf->normal(x) )).norm() > rb.mu*j.dot(sdf->normal(x))){
    	dvn = collision.d*collision.n/dt; // dx compensation
    	T = vt.normalized();
    	if(vt.norm()==0){ // dv will depend on the impulse direction
    		jn = j.dot(sdf->normal(x))*(sdf->normal(x));
    		T = (j-jn).normalized();
    	}
    	NKN = collision.n.transpose() * K * (collision.n - rb.mu*T);
    	jn = dvn/NKN;
    	j = jn + rb.mu*jn.norm()*(-T);
    }
    rb.addImpulse(j, collision.x0);
    // Update position
    stepPosition(rb, dt);
}

void addPenaltyForces(ParticleSystem &ps, RigidBody &rb, double ks, double kd) {
    // as external forces to the particles and the rigid body
    vector<Collision> collisions;
    findCollisions(ps, rb, collisions);
    int index;
    double d, dv;
    Vector3d x, n, force_ps, dvv;

    for(int i=0; i<collisions.size(); i++){
    	index = collisions[i].index0;
    	d = collisions[i].d;
    	x = collisions[i].x1;
    	n = collisions[i].n;
    	dv = (rb.velocityAt(x) - ps.particles[index]->v).dot(n);   //particle velocity relative to the sdf contact point   
    	force_ps = ks*d*n + kd*dv*n;		//IGNORE FRICTION FOR NOW  CHECK THE SIGNS
    	ps.particles[collisions[i].index0]->f_ext += force_ps;
    	rb.addForce(-force_ps, x);
    }
}

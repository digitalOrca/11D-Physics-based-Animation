#include "fluid.hpp"
#include <iostream>

Vector2d advectPoint(const StaggeredGrid &u, Vector2d x0, double dt) {
    //get the current velocity
    Vector2d u0 = u.interpolate(x0);
    //get eulerian step velocity
    Vector2d x1 = x0 + u0*dt;
    Vector2d u1 = u.interpolate(x1);
    //average two velocities
    Vector2d umid = (u0+u1)/2;
    Vector2d xrk = x0 + umid*dt;
    return xrk;
}

void advectScalarField(const StaggeredGrid &u, Grid<double> &c0,
                       Grid<double> &c1, double dt) {
    for(int i=0; i<u.m; i++){
    	for(int j=0; j<u.n; j++){
    		c1(i,j) = c0.interpolate( advectPoint(u, Vector2d(i*c0.dx, j*c0.dx)+c0.x0, -dt) );
    	}
    }
}

void advectVectorField(const StaggeredGrid &u, StaggeredGrid &v0,
                       StaggeredGrid &v1, double dt) {
    Vector2d pnt;
    for(int i=0; i<u.m; i++){
    	for(int j=0; j<u.n; j++){
    		pnt = Vector2d(i*v0.dx, j*v0.dx) + v0.x0;
    		v1.x(i,j)= v0.x.interpolate( advectPoint(u, pnt, -dt) );
    		v1.y(i,j)= v0.y.interpolate( advectPoint(u, pnt, -dt) );
    	}
    }
}

int enumerateCells(Grid<int> &index) {
    int N = 0;
    for (int i = 0; i < index.m; i++)
        for (int j = 0; j < index.n; j++)
            index(i,j) = N++;
    return N;
}

Fluid::Fluid(int m, int n, Vector2d x0, double dx) {
    this->m = m;
    this->n = n;
    this->x0 = x0;
    this->dx = dx;
    this->p = Grid<double>(m,n,x0,dx);
    p.assign(1);
    this->index = Grid<int>(m,n,x0,dx);
    this->N = enumerateCells(index);
}

void Fluid::step(double dt) {
    advection(dt);
    pressureProjection();
}

void Fluid::advection(double dt) {
    // u self advection
    advectVectorField(*u, *u, *u_tmp, dt);
    // Swap pointer
    StaggeredGrid *handle;
    handle = u;
    u = u_tmp;
    u_tmp = handle;
}

void Fluid::pressureProjection() {
	//for(vector<double>::iterator it= p.values.begin(); it != p.values.end(); ++it)
    //	cout << ' ' << *it << " ";
   	//cout << endl;

    Grid<double> u_div(m, n, Vector2d(0,0), dx);
    divergence(*u, u_div);
    // Copy from grid to vectorxd
    VectorXd div(N);
    VectorXd pressure(N);
   	for(int i=0; i<m; i++)
   		for(int j=0; j<n; j++)
   			div[ index(i,j) ] = u_div(i,j);
   	// Build A matrix
   	double up, down, left, right, self;
   	int key0, key1, key2, key3, key4;
   	SparseMatrixBuilder A(N,N);
   	for(int i=0; i<m; i++){
   		for(int j=0; j<n; j++){
   			self = p(i,j); key0 = index(i,j);
   			if(i==0)  {key1=key0;} 	else{key1=index(i-1,j);}
   			if(i==m-1){key2=key0;}	else{key2=index(i+1,j);}
   			if(j==0)  {key3=key0;} 	else{key3=index(i,j-1);}
   			if(j==n-1) {key4=key0;}   	else{key4=index(i,j+1);}
   			//cout<<"s: "<<self<<"\tl: "<<left<<"\tr: "<<right<<"\tu: "<<up<<"\td: "<<down<<endl;
   			A.add( key0, key1, 1/(dx*dx) );
   			A.add( key0, key2, 1/(dx*dx) );
   			A.add( key0, key3, 1/(dx*dx) );
   			A.add( key0, key4, 1/(dx*dx) );
   			A.add( key0, key0, -4.0/(dx*dx) );
   		}
   	}
   	// Solve for pressure
   	pressure = A.solve(div);
   	for(int i=0; i<m; i++){
   		for(int j=0; j<n; j++){
   			p(i,j) = pressure[ index(i,j) ];
   		}
    }
   	// Modify velocity field
   	StaggeredGrid dp(m,n,x0,dx);
   	gradient(p, dp);
   	for(int i=0; i<m; i++){
   		for(int j=0; j<n; j++){
   			u->x(i,j) -= dp.x(i,j);
   			u->y(i,j) -= dp.y(i,j);
   		}
   	}
   	/**/
}

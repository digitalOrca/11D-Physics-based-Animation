#ifndef SDF_HPP
#define SDF_HPP

#include <Eigen/Dense>
#include <string>
#include "grid.hpp"

#define PI 3.1415926535897932384626433832795

using namespace Eigen;

class SDF {
public:
    virtual double signedDistance(Vector3d x) = 0;
    virtual Vector3d normal(Vector3d x) = 0;
    virtual vector<Vector3d> sampleT(Vector3d x, int numSamples) = 0;
};

class PlaneSDF: public SDF {
public:
    Vector3d x0, n;
    PlaneSDF(Vector3d x0, Vector3d n):
        x0(x0), n(n.normalized()) {
    }
    virtual double signedDistance(Vector3d x) {
    	return (x-x0).dot(n);
    }
    virtual Vector3d normal(Vector3d x) {
        return n;
    }
    virtual vector<Vector3d> sampleT(Vector3d x, int numSamples){
    	vector<Vector3d> Ts;
		Vector3d t1, t2;
		t1 = (this->n.cross( Vector3d(1,1,1) )).normalized();
		t2 = (this->n.cross( Vector3d(t1) )).normalized();
		for(int i=0; i<numSamples; i++){
			double frac = (double)i/(double)numSamples;
			Vector3d t = t1*sin(frac*2*PI) + t2*cos(frac*2*PI);
			Ts.push_back(t.normalized());
		}
		return Ts;
    }
};

class SphereSDF: public SDF {
public:
    Vector3d x0;
    double r;
    SphereSDF(Vector3d x0, double r):
        x0(x0), r(r) {
    }
    virtual double signedDistance(Vector3d x) {
        return (x-x0).norm()-r;
    }
    virtual Vector3d normal(Vector3d x) {
        return (x-x0).normalized();
    }
    virtual vector<Vector3d> sampleT(Vector3d x, int numSamples){
    	vector<Vector3d> Ts;
    		// UNUSED
		return Ts;
    }
};

class GridSDF: public SDF {
public:
    Grid3D<double> grid;
    GridSDF(std::string filename) {
        readGrid(grid, filename);
    }
    virtual double signedDistance(Vector3d x) {
        return grid.interpolate(x);
    }
    virtual Vector3d normal(Vector3d x) {
        return grid.gradient(x);
    }
    virtual vector<Vector3d> sampleT(Vector3d x, int numSamples){
    	vector<Vector3d> Ts;
    		// UNUSED
		return Ts;
    }
};

class BoxSDF: public SDF {	
public:
	vector<PlaneSDF> faces;
	Vector3d n, t1, t2, x, y, z;
	
	BoxSDF(Vector3d i, Vector3d j, Vector3d x, Vector3d y, Vector3d z){
		this->x = x;
		this->y = y;
		this->z = z;
		PlaneSDF face1(i, x);
		PlaneSDF face2(i, y);
		PlaneSDF face3(i, z);
		PlaneSDF face4(j, -x);
		PlaneSDF face5(j, -y);
		PlaneSDF face6(j, -z);	
		faces.push_back(face1);
		faces.push_back(face2);
		faces.push_back(face3);
		faces.push_back(face4);
		faces.push_back(face5);
		faces.push_back(face6);
	}
	virtual double signedDistance(Vector3d x){
		double d = -INFINITY;
		for(int i=0; i<6; i++){
			double d_temp = faces[i].signedDistance(x);
			if (d_temp > d){
				d = d_temp;
				n = faces[i].normal(x);
				switch (i){		// Inefficient, but I may optimize it later   // Check for t1 t2
					case 1:  t1 = y; t2 = z;
						break;
					case 2:  t1 = z; t2 = x;
						break;
					case 3:  t1 = x; t2 = y;
						break;
					case 4:  t1 = z; t2 = y;
						break;
					case 5:  t1 = x; t2 = z;
						break;
					case 6:  t1 = y; t2 = x;
						break;
				}
			}
			
		}
		return d;
	}
	
	virtual Vector3d normal(Vector3d x){
		signedDistance(x);			//TODO: ------------ TO SIMPLIFY THIS-------------------
		return n;
	}
	
	virtual vector<Vector3d> sampleT(Vector3d x, int numSamples){
		vector<Vector3d> Ts;
		normal(x); // update normal and t1, t2
		for(int i=0; i<numSamples; i++){
			double frac = (double)i/(double)numSamples;
			Vector3d t = this->t1*sin(frac*2*PI) + this->t2*cos(frac*2*PI);
			Ts.push_back(t.normalized());
		}
		return Ts;
	}
};
#endif

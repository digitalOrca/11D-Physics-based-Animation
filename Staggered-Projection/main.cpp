#include <iostream>
#include <fstream>
#include "gui.hpp"
#include "rigid.hpp"
#include "rigidsystem.hpp"
#include "sdf.hpp"
#include "qpsolver.hpp"

using namespace std;

void runBoxToss();
void runCardHouse();
void runReplay();

#define RUN_SCENARIO runReplay

int main() {
    RUN_SCENARIO();
}

void runBoxToss() {
    // Time step length
    double fps = 120.;
    double dt = 0.2/fps; // 5x slowmo
    
    // Obstacles
    TriMesh floorMesh = TriMesh::rectangle(Vector3d(0,0,0), Vector3d(20,0,0), Vector3d(0,0,-20));
    PlaneSDF floor(Vector3d(0,0,0), Vector3d(0,1,0));
    //TriMesh floorMesh = TriMesh::rectangle(Vector3d(0,0,0), Vector3d(20,0,0), Vector3d(0,8,-20));
    //PlaneSDF floor(Vector3d(0,0,0), Vector3d(0,1,0.4));
    
    // Create rigid bodies 1 system
    double w = 0.5, h = 0.1, d = 0.5;
    TriMesh boxMesh1 = TriMesh::box(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m1 = 0.1;
    Matrix3d I1 = m1/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody box1(&boxMesh1, m1, I1, nullptr);
    box1.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    box1.mu = 0.2;
    box1.epsilon = 0.5;
    // Create SDF for box
    	vector<Vector3d> points;
    	points = box1.mesh->vertices;
    	Vector3d i = points[0];
    	Vector3d j = points[7];
    	Vector3d x = points[0] - points[1];
    	Vector3d y = points[0] - points[2];
    	Vector3d z = points[0] - points[4];
    	BoxSDF boundary1(i, j, x, y, z);
    box1.sdf = &boundary1;
    // Create rigid bodies 2 system
    w = 0.3, h = 0.2, d = 0.3;
    TriMesh boxMesh2 = TriMesh::box(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m2 = 0.1;
    Matrix3d I2 = m2/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody box2(&boxMesh2, m2, I2, nullptr);
    box2.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    box2.mu = 0.2;
    box2.epsilon = 0.5;
    // Create SDF for box
    	points = box2.mesh->vertices;
    	i = points[0];
    	j = points[7];
    	x = points[0] - points[1];
    	y = points[0] - points[2];
    	z = points[0] - points[4];
    	BoxSDF boundary2(i, j, x, y, z);
    box2.sdf = &boundary2;
    // Create rigid bodies 3 system
    w = 0.1, h = 0.1, d = 0.1;
    TriMesh boxMesh3 = TriMesh::box(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m3 = 0.1;
    Matrix3d I3 = m3/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody box3(&boxMesh3, m3, I3, nullptr);
    box3.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    box3.mu = 0.2;
    box3.epsilon = 0.5;
    // Create SDF for box
    	points = box3.mesh->vertices;
    	i = points[0];
    	j = points[7];
    	x = points[0] - points[1];
    	y = points[0] - points[2];
    	z = points[0] - points[4];
    	BoxSDF boundary3(i, j, x, y, z);
    box3.sdf = &boundary3;
    
    box1.q = Quaterniond(1,0,0,0).normalized();
    box2.q = Quaterniond(1,0,0,0).normalized();
    box3.q = Quaterniond(1,0,0,0).normalized();
    
    box1.xcom = Vector3d(0,0.1,0);
    box2.xcom = Vector3d(0,0.5,0);
    //box2.xcom = Vector3d(0.3,0.5,0.3);
	box3.xcom = Vector3d(0,0.8,0);
	
	// Debugging case
	//box1.xcom = Vector3d(0,0.15,0);
	//box2.xcom = Vector3d(0,0.5,0);
	//box1.q = Quaterniond(1,0,0,0).normalized();
    //box2.q = Quaterniond(1,0.5,0.5,0.5).normalized();
    //box1.addImpulse(Vector3d(0,0.2,0), box1.xcom); // toss the box upwards
    
    // Add bodies to the system
    RigidSystem rs;
    rs.environment = &floor;
    rs.bodies.push_back(&box1);
    rs.bodies.push_back(&box2);
    rs.bodies.push_back(&box3);
    
    // Initial velocity
    //box1.addImpulse(Vector3d(0,0.4,0), box1.xcom); // toss the box upwards
    //box2.addImpulse(Vector3d(0,0,-0.1), box1.xcom+Vector3d(0,0.1,0)); // X spin 1
    //box2.addImpulse(Vector3d(0,0,0.1), box1.xcom+Vector3d(0,-0.1,0)); // X spin 2
    //box2.addImpulse(Vector3d(0,0,-0.1), box2.xcom+Vector3d(0.1,0,0)); // Y spin 1
    //box2.addImpulse(Vector3d(0,0,0.1), box2.xcom+Vector3d(-0.1,0,0)); // Y spin 2
    //box3.addImpulse(Vector3d(0,0.1,0), box3.xcom+Vector3d(-0.1,0,0)); // Z spin 1
    //box3.addImpulse(Vector3d(0,-0.1,0), box3.xcom+Vector3d(0.1,0,0)); // Z spin 2
    
    // Create the window
    Window3D window(Vector2i(720,720));
    window.camera.lookAt(Vector3d(0,0.5,2), Vector3d(0,0.2,0), Vector3d(0,1,0));
    window.camera.setFOV(45);
    
    // Pre-allocate memories and setup matrix
    rs.resizeSystem();
    rs.updateM();
    Vector3d g(0,-9.8,0);	// define gravity
    bool running = false;
    
    while (!window.shouldClose()) {
    	
    	if (window.keyPressed(GLFW_KEY_SPACE))
            running = true;
    	if(running){
    		rs.clearForces();	// clear external forces
    		rs.forceField(g);	// apply external forces
    		rs.contacts.resize(0);	// clear contacts
        	rs.collisionDetection();	// detect collision between objects and to the floor
        		rs.drawContact(rs.contacts);	// debugging
        	rs.updateM();		// update rotational inertia
        	rs.updateContactMatrix();		// update contact related matrix dimension
        	rs.generalizeImpulse();			// build N matrix
        	rs.optimize(1e-6, 100, dt);	// optimize contact force coupling
        		rs.drawNormalForce(rs.contacts);	// debugging
        	rs.updateSystem(dt);	// update system
        }	
        floorMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
        rs.drawSystem();
        window.updateAndWait(1/fps);
    }
}


void runCardHouse(){
	// Time step length
    double fps = 60.;
    double dt = 0.2/fps; // 5x slowmo
    // Floor
    TriMesh floorMesh = TriMesh::rectangle(Vector3d(0,0,0), Vector3d(20,0,0), Vector3d(0,0,-20));
    PlaneSDF floor(Vector3d(0,0,0), Vector3d(0,1,0));
    
    // Create supporting card
    double w = 0.2, h = 0.35, d = 0.02;
    TriMesh cardMesh1 = TriMesh::card(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m1 = 0.1;
    Matrix3d I1 = m1/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody card1(&cardMesh1, m1, I1, nullptr);
    card1.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    card1.mu = 0.2;
    card1.epsilon = 0.5;
    // Create SDF for box
    	vector<Vector3d> points;
    	points = card1.mesh->vertices;
    	Vector3d i = points[0];
    	Vector3d j = points[7];
    	Vector3d x = points[0] - points[1];
    	Vector3d y = points[0] - points[2];
    	Vector3d z = points[0] - points[4];
    	BoxSDF boundary1(i, j, x, y, z);
    card1.sdf = &boundary1;
    
    // Create horizontal card
     w = 0.3, h = 0.5, d = 0.04;
   TriMesh cardMesh5 = TriMesh::card(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m5 = 0.1;
    Matrix3d I5 = m5/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody card5(&cardMesh5, m5, I5, nullptr);
    card5.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    card5.mu = 0.2;
    card5.epsilon = 0.5;
    // Create SDF for box
    	points = card5.mesh->vertices;
    	i = points[0];
    	j = points[7];
    	x = points[0] - points[1];
    	y = points[0] - points[2];
    	z = points[0] - points[4];
    	BoxSDF boundary5(i, j, x, y, z);
        card5.sdf = &boundary5;
    
  // Duplicate Cards
    RigidBody card2 = card1;
    RigidBody card3 = card1;
    RigidBody card4 = card1;
    RigidBody card6 = card1;
    RigidBody card7 = card1;
    
    // Setup card location 
    //--> layer one
    card1.xcom = Vector3d(0,0.1566,-0.0962+0.39/2);
    card2.xcom = Vector3d(0,0.1566, 0.0962+0.39/2);
    card3.xcom = Vector3d(0,0.1566,-0.0962-0.39/2);
    card4.xcom = Vector3d(0,0.1566, 0.0962-0.39/2);
    card1.q = Quaterniond(0.9659, 0.2588, 0, 0).normalized();
    card2.q = Quaterniond(0.9659,-0.2588, 0, 0).normalized();
    card3.q = Quaterniond(0.9659, 0.2588, 0, 0).normalized();
    card4.q = Quaterniond(0.9659,-0.2588, 0, 0).normalized();
    // --> horizontal
    card5.xcom = Vector3d(0,0.3231+0.03,0);
    card5.q = Quaterniond(0.707, 0.707, 0, 0).normalized();
    // --> layer two
    card6.xcom = Vector3d(0,0.1566+2*0.1566+0.02+0.05,-0.0962);
    card7.xcom = Vector3d(0,0.1566+2*0.1566+0.02+0.05, 0.0962);
    card6.q = Quaterniond(0.9659, 0.2588, 0, 0).normalized();
    card7.q = Quaterniond(0.9659,-0.2588, 0, 0).normalized();
    
    // Add bodies to the system
    RigidSystem rs;
    rs.environment = &floor;
    rs.bodies.push_back(&card1);
    rs.bodies.push_back(&card2);
    rs.bodies.push_back(&card3);
    rs.bodies.push_back(&card4);
    rs.bodies.push_back(&card5);
    rs.bodies.push_back(&card6);
    rs.bodies.push_back(&card7);
    
    // Create the window
    Window3D window(Vector2i(720,720));
    window.camera.lookAt(Vector3d(-1,1,-1), Vector3d(0,0.4,0), Vector3d(0,1,0));
    window.camera.setFOV(60);
    
    // Pre-allocate memories and setup matrix
    rs.resizeSystem();
    rs.updateM();
    Vector3d g(0,-9.8,0);	// define gravity
    
    // get file ready
    ofstream myfile;
  	myfile.open ("cardhouse.txt", ios::app);
    
    while (!window.shouldClose()) {
    	rs.clearForces();	// clear external forces
    	rs.forceField(g);	// apply external forces
    	rs.contacts.resize(0);	// clear contacts
        rs.collisionDetection();	// detect collision between objects and to the floor
        	rs.drawContact(rs.contacts);	// debugging
        rs.updateM();		// update rotational inertia
        rs.updateContactMatrix();		// update contact related matrix dimension
        rs.generalizeImpulse();			// build N matrix
        rs.optimize(1e-6, 100, dt);	// optimize contact force coupling
        	rs.drawNormalForce(rs.contacts);	// debugging
        rs.updateSystem(dt);	// update system
        floorMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
        rs.drawSystem();
        rs.saveState(myfile);
		window.updateAndWait(1/fps);
    }
    myfile.close();
}

void runReplay(){
	// Time Step
	double fps = 60.;
    double dt = 0.2/fps; // 5x slowmo
	// setup the objects
    // Floor
    TriMesh floorMesh = TriMesh::rectangle(Vector3d(0,0,0), Vector3d(20,0,0), Vector3d(0,0,-20));
    PlaneSDF floor(Vector3d(0,0,0), Vector3d(0,1,0));
    
    // Create supporting card
    double w = 0.2, h = 0.35, d = 0.02;
    TriMesh cardMesh1 = TriMesh::card(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m1 = 0.1;
    Matrix3d I1 = m1/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody card1(&cardMesh1, m1, I1, nullptr);
    card1.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    card1.mu = 0.2;
    card1.epsilon = 0.5;
    // Create SDF for box
    	vector<Vector3d> points;
    	points = card1.mesh->vertices;
    	Vector3d i = points[0];
    	Vector3d j = points[7];
    	Vector3d x = points[0] - points[1];
    	Vector3d y = points[0] - points[2];
    	Vector3d z = points[0] - points[4];
    	BoxSDF boundary1(i, j, x, y, z);
    card1.sdf = &boundary1;
    
    // Create horizontal card
    w = 0.25, h = 0.4, d = 0.05;
   TriMesh cardMesh5 = TriMesh::card(Vector3d(0,0,0), Vector3d(w,0,0), Vector3d(0,h,0), Vector3d(0,0,d));
    double m5 = 0.1;
    Matrix3d I5 = m5/12*Vector3d(h*h+d*d, w*w+d*d, w*w+h*h).asDiagonal();
    RigidBody card5(&cardMesh5, m5, I5, nullptr);
    card5.q = Quaterniond(1,0.01,0.01,0.01).normalized();
    card5.mu = 0.2;
    card5.epsilon = 0.5;
    // Create SDF for box
    	points = card5.mesh->vertices;
    	i = points[0];
    	j = points[7];
    	x = points[0] - points[1];
    	y = points[0] - points[2];
    	z = points[0] - points[4];
    	BoxSDF boundary5(i, j, x, y, z);
        card5.sdf = &boundary5;
    
  // Duplicate Cards
    RigidBody card2 = card1;
    RigidBody card3 = card1;
    RigidBody card4 = card1;
    RigidBody card6 = card1;
    RigidBody card7 = card1;
    
    // Setup card location 
    //--> layer one
    card1.xcom = Vector3d(0,0.1566,-0.0962+0.39/2);
    card2.xcom = Vector3d(0,0.1566, 0.0962+0.39/2);
    card3.xcom = Vector3d(0,0.1566,-0.0962-0.39/2);
    card4.xcom = Vector3d(0,0.1566, 0.0962-0.39/2);
    card1.q = Quaterniond(0.9659, 0.2588, 0, 0).normalized();
    card2.q = Quaterniond(0.9659,-0.2588, 0, 0).normalized();
    card3.q = Quaterniond(0.9659, 0.2588, 0, 0).normalized();
    card4.q = Quaterniond(0.9659,-0.2588, 0, 0).normalized();
    // --> horizontal
    card5.xcom = Vector3d(0,0.3231+0.03,0);
    card5.q = Quaterniond(0.707, 0.707, 0, 0).normalized();
    // --> layer two
    card6.xcom = Vector3d(0,0.1566+2*0.1566+0.02+0.03,-0.0962-0.01);
    card7.xcom = Vector3d(0,0.1566+2*0.1566+0.02+0.03, 0.0962+0.01);
    card6.q = Quaterniond(0.9659, 0.2588, 0, 0).normalized();
    card7.q = Quaterniond(0.9659,-0.2588, 0, 0).normalized();
    
    // Add bodies to the system
    RigidSystem rs;
    rs.environment = &floor;
    rs.bodies.push_back(&card1);
    rs.bodies.push_back(&card2);
    rs.bodies.push_back(&card3);
    rs.bodies.push_back(&card4);
    rs.bodies.push_back(&card5);
    rs.bodies.push_back(&card6);
    rs.bodies.push_back(&card7);
    
    // Create the window
    Window3D window(Vector2i(720,720));
    window.camera.lookAt(Vector3d(-1,1,-1), Vector3d(0,0.4,0), Vector3d(0,1,0));
    window.camera.setFOV(60);
	
	
	while (!window.shouldClose()){
		fstream file("cardhouse.txt", ios::in);
	    if(!file) {
	        cerr << "Error: failed to open file " << endl;
	        return;
	    }
	    string line;
	    //get_valid_line(file, line);
	    //window.updateAndWait(10);  // get the camera ready
	    while( getline(file, line) && !window.shouldClose()){
	    	stringstream linestream(line);
	    	Vector3d xcom;
	    	Quaterniond q;
	    	for(int i=0; i<rs.bodies.size(); i++){
	    		linestream >> xcom[0] >> xcom[1] >> xcom[2] >> q.w() >> q.vec()[0] >> q.vec()[1] >> q.vec()[2];
	    		rs.bodies[i]->xcom = xcom;
	    		rs.bodies[i]->q = q;
	    	}
	    	rs.drawSystem();
	    	floorMesh.draw(Vector4d(0.8,0.8,0.8,1), Vector4d(0,0,0,0));
			window.updateAndWait(1/fps);
			//cout << "xcom: " << xcom.transpose() << "\tq: " << q.w() << " " << q.vec().transpose() << endl;
	 	}	
	}
}

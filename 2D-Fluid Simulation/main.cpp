#include "fluid.hpp"
#include "grid.hpp"
#include "gui.hpp"
#include <iostream>
using namespace std;

#define RUN_SCENARIO mainDemo
int n = 100; // Grid resolution. Later, try 20, 50, 100

void advectionTest();
void mainDemo();

int main() {
    RUN_SCENARIO();
}

Vector2d velocityField(Vector2d x) {
    return Vector2d(-sin(M_PI*x[0])*cos(M_PI*x[1]),
                    cos(M_PI*x[0])*sin(M_PI*x[1]));
}

double random(double xmin, double xmax) {
    return xmin + (xmax-xmin)*rand()/RAND_MAX;
}

void drawPoint(Vector2d x, Vector4d color) {
    glPointSize(5);
    glColor4f(color[0], color[1], color[2], color[3]);
    glBegin(GL_POINTS);
    glVertex2f(x[0], x[1]);
    glEnd();
}

// Colormap: white = zero, orange = positive, blue = negative
Vector3d colormap(double c) {
    Vector3d blue(0.2, 0.6, 0.9), orange(0.9, 0.6, 0.2);
    if (c >= 0)
        return Vector3d(pow(orange[0],c), pow(orange[1],c), pow(orange[2],c));
    else
        return Vector3d(pow(blue[0],-c), pow(blue[1],-c), pow(blue[2],-c));
}

void drawGrid(Grid<double> &c) {
    double scale = 1./c.dx;						//MODIFIED
    glBegin(GL_QUADS);
    for (int i = 0; i < c.m; i++) {
        for (int j = 0; j < c.n; j++) {
            Vector2d x = c.x0 + Vector2d(i, j)*c.dx;	//FIXED
            Vector3d color = colormap(c(i,j)*scale);
            glColor3f(color[0], color[1], color[2]);
            glVertex2f(x(0), x(1));
            glVertex2f(x(0)+c.dx, x(1));
            glVertex2f(x(0)+c.dx, x(1)+c.dx);
            glVertex2f(x(0), x(1)+c.dx);
        }
    }
    glEnd();
}

void drawStaggeredGrid(StaggeredGrid &u) {
    glBegin(GL_TRIANGLES);
    glColor4f(0.2,0.2,0.8,0.6);
    for (int i = 0; i <= u.m; i++) {
        for (int j = 0; j < u.n; j++) {
            Vector2d x = u.x0 + Vector2d(i, j+0.5)*u.dx;
            glVertex2f(x(0),               x(1)-u.dx*0.1);
            glVertex2f(x(0)+u.x(i,j)*u.dx, x(1));
            glVertex2f(x(0),               x(1)+u.dx*0.1);
        }
    }
    for (int i = 0; i < u.m; i++) {
        for (int j = 0; j <= u.n; j++) {
            Vector2d x = u.x0 + Vector2d(i+0.5, j)*u.dx;
            glVertex2f(x(0)-u.dx*0.1, x(1));
            glVertex2f(x(0),          x(1)+u.y(i,j)*u.dx);
            glVertex2f(x(0)+u.dx*0.1, x(1));
        }
    }
    glEnd();
}

void advectionTest() {
    // Velocity field
    StaggeredGrid u(n,n, Vector2d(0,0), 1./n);
    // Initialize entries of u using velocityField() function
    Vector2d node = Vector2d(0,0);
    for(int i=0; i<n; i++){
    	for(int j=0; j<n; j++){
    		node = velocityField(Vector2d(i*u.dx,j*u.dx)-u.x0);
    		u.x(i,j) = node[0];
    		u.y(i,j) = node[1];
    	}
    }
    // Tracer particles
    vector<Vector2d> tracers;
    srand(0);
    for (int i = 0; i < 1000; i++)
        tracers.push_back(Vector2d(random(0,1), random(0,1)));
    // Scalar field
    Grid<double> *c = new Grid<double>(n,n, Vector2d(0.5/n,0.5/n), 1./n);
    c->assign(0);
    Vector2d center(0.75, 0.5);
    double radius = 0.2;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Vector2d x = c->x0 + Vector2d(i,j)*c->dx;
            if ((x - center).norm() < radius)
                (*c)(i,j) = 1;
        }
    }
    Grid<double> *c_tmp = new Grid<double>(n,n, Vector2d(0.5/n,0.5/n), 1./n);
    Grid<double> *handle = c_tmp;
    // Time step length
    double fps = 30.;
    double dt = 1/fps;
    // Create the window
    Window2D window(Vector2i(720,720), Vector2d(0,0), Vector2d(1,1));
    while (!window.shouldClose()) {
        // Advect c into c_tmp
        advectScalarField(u, *c, *c_tmp, dt);
        // Swap pointers
        handle = c_tmp;
        c_tmp = c;
        c = handle;
        
        drawGrid(*c);
        for (int i = 0; i < tracers.size(); i++) {
            // Advect tracer particles
            tracers[i] = advectPoint(u, tracers[i], dt);
            drawPoint(tracers[i], Vector4d(0,0,0,1));
        }
        drawStaggeredGrid(u);
        window.updateAndWait(dt);
    }
}

void mouseInteraction(Fluid &fluid, Vector2d mousePos, bool mouseDown, double dt);

void mainDemo() {
	// Create fluid
    Fluid fluid(n,n, Vector2d(0,0), 1./n);
    // Create velocity field staggered grid
    StaggeredGrid v(n,n,Vector2d(0,0), 1./n);	fluid.u = &v;
    StaggeredGrid v_tmp(n,n,Vector2d(0,0), 1./n);	fluid.u_tmp = &v_tmp;
    // Tracer particles
    vector<Vector2d> tracers;
    srand(0);
    for (int i = 0; i < 10000; i++)
        tracers.push_back(Vector2d(random(0,1), random(0,1)));
    // Time step length
    double fps = 30.;
    double dt = 1/fps;
    // Create the window
    Window2D window(Vector2i(720,720), Vector2d(0,0), Vector2d(1,1));
    /*// Validation case
    Grid<double> c(3,3,Vector2d(0,0),1);
    c.assign(0);
    c(1,1) = 1;
    StaggeredGrid test_grad(3,3,Vector2d(0,0),1);
    gradient(c,test_grad);
    cout << "grad_x" << endl;
    for(vector<double>::iterator it= test_grad.x.values.begin(); it != test_grad.x.values.end(); ++it)
    	cout << ' ' << *it;
    cout << "\n" << "grad_y" << endl;;
    for(vector<double>::iterator it= test_grad.y.values.begin(); it != test_grad.y.values.end(); ++it)
    	cout << ' ' << *it;
    cout << "\n divergence \n";
   	Grid<double> test_div(3,3,Vector2d(0,0),1);
    divergence(test_grad ,test_div);
    for(vector<double>::iterator it=test_div.values.begin(); it!=test_div.values.end(); ++it)
    	cout << ' ' << *it;
    cout << "\n";
    Grid<int> index(3,3,Vector2d(0,0), 1);
    index(0,0) = 0;
    index(1,0) = 1;
    index(2,0) = 2;
    index(0,1) = 3;
    index(1,1) = 4;
    index(2,1) = 5;
    index(0,2) = 6;
    index(1,2) = 7;
    index(2,2) = 8;
    double up, down, left, right, self;
   	int key0, key1, key2, key3, key4;
   	SparseMatrixBuilder A(9,9);
   	for(int i=0; i<3; i++){
   		for(int j=0; j<3; j++){
   			self = c(i,j); key0 = index(i,j);
   			if(i==0){left  = self; key1=index(i,j);} 	else{left = c(i-1,j);  key1=index(i-1,j);}
   			if(i==2){right = self; key2=index(i,j);}	else{right = c(i+1,j); key2=index(i+1,j);}
   			if(j==0){down  = self; key3=index(i,j);} 	else{down = c(i,j-1);  key3=index(i,j-1);}
   			if(j==2){up    = self; key4=index(i,j);}   	else{up = c(i,j+1);    key4=index(i,j+1);}
   			A.add( index(i,j), key1, left );
   			A.add( index(i,j), key2, right );
   			A.add( index(i,j), key3, down );
   			A.add( index(i,j), key4, up );
   			A.add( index(i,j), key0,  -4*self );
   		}
   	}
   	VectorXd cv(9);
   	cv << 0,0,0,0,1,0,0,0,0;
   	VectorXd test_A;
   	test_A = A.getMatrix()*cv;
   	for(int i=0; i<9; i++)
   		cout << test_A[i] << endl;
	cout << "\n";
    */
    while (!window.shouldClose()) {
        mouseInteraction(fluid, window.mousePos(), window.mouseDown(), dt);
        fluid.step(dt); // While debugging pressureProjection, consider disabling this
        drawGrid(fluid.p);
        for (int i = 0; i < tracers.size(); i++) {
            tracers[i] = advectPoint(*fluid.u, tracers[i], dt);
            drawPoint(tracers[i], Vector4d(0,0,0,1));
        }
        drawStaggeredGrid(*fluid.u);
        window.updateAndWait(dt);
    }
}

Vector2d lastMousePos;

void mouseInteraction(Fluid &fluid, Vector2d mousePos, bool mouseDown, double dt) {
    if (mouseDown) {
        Vector2d v = (mousePos - lastMousePos)/dt;
        double r = 2*fluid.dx; // control velocity in a 2-cell radius
        for (int i = 1; i < fluid.n; i++) { // don't change boundary velocities
            for (int j = 0; j < fluid.n; j++) {
                Vector2d x = fluid.x0 + Vector2d(i,j+0.5)*fluid.dx;
                if ((x - mousePos).norm() < r)
                    fluid.u->x(i,j) = v[0];
            }
        }
        for (int i = 0; i < n; i++) {
            for (int j = 1; j < n; j++) { // don't change boundary velocities
                Vector2d x = fluid.x0 + Vector2d(i+0.5,j)*fluid.dx;
                if ((x - mousePos).norm() < r)
                    fluid.u->y(i,j) = v[1];
            }
        }
    }
    lastMousePos = mousePos;
}

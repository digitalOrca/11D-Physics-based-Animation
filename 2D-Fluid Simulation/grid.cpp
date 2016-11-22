#include "grid.hpp"

template <typename T>
const T& Grid<T>::get(int i, int j) const {
    return values[i+m*j];
}

template <typename T>
void Grid<T>::assign(T value) {
    values.assign(values.size(), value);
}

template <typename T>
T Grid<T>::interpolate(Vector2d x) const {
    Vector2d index = (x - x0)/dx;
    int i = floor(index(0)), j = floor(index(1)); // integer part of index
    double fi = index(0) - i, fj = index(1) - j;  // fractional part
    if (i < 0)
        {i = 0; fi = 0;}
    else if (i >= m-1)
        {i = m-2; fi = 1;}
    if (j < 0)
        {j = 0; fj = 0;}
    else if (j >= n-1)
        {j = n-2; fj = 1;}
    return
        + (1-fi)*(1-fj)*get(i,   j)
        + (  fi)*(1-fj)*get(i+1, j)
        + (1-fi)*(  fj)*get(i,   j+1)
        + (  fi)*(  fj)*get(i+1, j+1);
}

// explicit template instantiation
template class Grid<char>;
template class Grid<int>;
template class Grid<double>;

StaggeredGrid::StaggeredGrid(int m, int n, Vector2d x0, double dx) {
    this->m = m;
    this->n = n;
    this->x0 = x0;
    this->dx = dx;
    this->x = Grid<double>(m+1, n, x0, dx);
    this->y = Grid<double>(m, n+1, x0, dx);
}

void StaggeredGrid::assign(Vector2d value) {
    // TODO
}

Vector2d StaggeredGrid::interpolate(Vector2d p) const { 
	return Vector2d(x.interpolate(p), y.interpolate(p));  
}

void gradient(const Grid<double> &p, StaggeredGrid &grad_p) {
    for(int i=1; i<p.m; i++){
    	for(int j=1; j<p.n; j++){
    		grad_p.x(i,j) = ( p(i,j)-p(i-1,j) )/p.dx;
    		grad_p.y(i,j) = ( p(i,j)-p(i,j-1) )/p.dx;
    	}
    }
    for(int i=0; i<p.m; i++){
    	grad_p.y(i,0) = 0;
    	grad_p.y(i,p.n) = 0;
    }
    for(int j=0; j<p.n; j++){
    	grad_p.x(0,j) = 0;
    	grad_p.x(p.m,j) = 0;
    }
}

void divergence(const StaggeredGrid &u, Grid<double> &div_u) {
    for(int i=0; i<u.m; i++){
    	for(int j=0; j<u.n; j++){
    		div_u(i,j) = ( u.x(i+1,j)+u.y(i,j+1)-u.x(i,j)-u.y(i,j) )/u.dx;
    	}
    }
}

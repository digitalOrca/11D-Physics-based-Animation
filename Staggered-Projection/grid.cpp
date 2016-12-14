#include "grid.hpp"
#include <fstream>

using namespace std;

void readGrid(Grid3D<double> &grid, string filename) {
    fstream file(filename.c_str(), ios::in);
    file >> grid.n[0] >> grid.n[1] >> grid.n[2];
    file >> grid.x0[0] >> grid.x0[1] >> grid.x0[2];
    file >> grid.dx[0];
    grid.dx[1] = grid.dx[2] = grid.dx[0];
    grid.values.resize(grid.n[0]*grid.n[1]*grid.n[2]);
    for (int i = 0; i < grid.n[0]*grid.n[1]*grid.n[2]; i++)
        file >> grid.values[i];
}

template <typename T>
T& Grid3D<T>::get(Vector3i index) {
    return values[index[0] + n[0]*(index[1] + n[1]*index[2])];
}

template <typename T>
void Grid3D<T>::assign(T value) {
    values.assign(values.size(), value);
}

// linear interpolation between a (when f = 0) and b (when f = 1)
template <typename T>
T lerp(double f, T a, T b) {
    return a + f*(b - a);
}

template <typename T>
T Grid3D<T>::interpolate(Vector3d x) {
    Vector3d location = (x - x0).array()/dx.array();
    // integer part of grid location
    Vector3i i(floor(location[0]), floor(location[1]), floor(location[2]));
    // fractional part
    Vector3d f = location - i.cast<double>();
    for (int d = 0; d < 3; d++) {
        if (i[d] < 0) {
            i[d] = 0; f[d] = 0;
        } else if (i[d] >= n[d]-1) {
            i[d] = n[d]-2; f[d] = 1;
        }
    }
    Vector3i di0 = Vector3i(1,0,0),
             di1 = Vector3i(0,1,0),
             di2 = Vector3i(0,0,1);
    return lerp(f[0],
                lerp(f[1],
                     lerp(f[2], get(i), get(i+di2)),
                     lerp(f[2], get(i+di1), get(i+di1+di2))),
                lerp(f[1],
                     lerp(f[2], get(i+di0), get(i+di0+di2)),
                     lerp(f[2], get(i+di0+di1), get(i+di0+di1+di2))));
}

template <typename T>
Matrix<T,3,1> Grid3D<T>::gradient(Vector3d x) {
    Vector3d dx0 = Vector3d(dx[0],0,0),
             dx1 = Vector3d(0,dx[1],0),
             dx2 = Vector3d(0,0,dx[2]);
    return Vector3d(interpolate(x+dx0/2)-interpolate(x-dx0/2),
                    interpolate(x+dx1/2)-interpolate(x-dx1/2),
                    interpolate(x+dx2/2)-interpolate(x-dx2/2)).array()/dx.array();
}

// explicit template instantiation so Grid3D<double> is available at link time
template class Grid3D<double>;

#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Mesh2D {
public:
    typedef Vector2d Vertex;
    typedef Vector2i Edge;
    typedef Vector3i Triangle;
    vector<Vertex> vertices;
    vector<Edge> edges;
    vector<Triangle> triangles;
    Mesh2D(const string &filename);
    void draw(Vector4d fillColor, Vector4d edgeColor);
};

#endif

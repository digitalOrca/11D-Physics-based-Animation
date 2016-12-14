#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class TriMesh {
public:
    vector<Vector3d> vertices;
    vector<Vector2i> edges;
    vector<Vector3i> triangles;
    TriMesh() {}
    TriMesh(const string &file);
    static TriMesh sphere(Vector3d center, double radius);
    static TriMesh rectangle(Vector3d center, Vector3d axis1, Vector3d axis2);
    static TriMesh box(Vector3d center,Vector3d axis1, Vector3d axis2, Vector3d axis3);
    static TriMesh card(Vector3d center,Vector3d axis1, Vector3d axis2, Vector3d axis3);
    void addVertex(Vector3d vertex);
    void addTriangle(Vector3i triangle);
    void drawLine(Vector3d s, Vector3d e);
    void drawVertex(vector<Vector3d> &vertices, Vector4d vertColor);
    void draw(Vector4d fillColor, Vector4d edgeColor);
};

class TetMesh {
public:
    vector<Vector3d> vertices;
    vector<Vector2i> edges, surfaceEdges;
    vector<Vector3i> triangles, surfaceTriangles;
    vector<Vector4i> tetrahedra;
    TetMesh() {}
    TetMesh(const string &file);
    void draw(Vector4d fillColor, Vector4d edgeColor);
};

#endif

#include "mesh.hpp"

#include <fstream>
#include <iostream>
#include "gui.hpp"

using namespace std;

template <typename T>
int isIn(T x, vector<T> list) {
    return find(list.begin(), list.end(), x) != list.end();
}

template <typename T>
int findIndex(T x, vector<T> list) {
    return find(list.begin(), list.end(), x) - list.begin();
}

Vector2i makeEdge(int i, int j) {
    if (i < j)
        return Vector2i(i, j);
    else
        return Vector2i(j, i);
}

void TriMesh::addVertex(Vector3d vert) {
    vertices.push_back(vert);
}

void TriMesh::addTriangle(Vector3i tri) {
    triangles.push_back(tri);
    for (int e = 0; e < 3; e++) {
        Vector2i edge = makeEdge(tri[e], tri[(e+1)%3]);
        if (!isIn(edge, edges))
            edges.push_back(edge);
    }
}

TriMesh::TriMesh(const string &filename) {
    fstream file(filename.c_str(), ios::in);
    if(!file) {
        cerr << "Error: failed to open file " << filename << endl;
        return;
    }
    while (file) {
        string line;
        do
            getline(file, line);
        while (file && (line.length() == 0 || line[0] == '#'));
        stringstream linestream(line);
        string keyword;
        linestream >> keyword;
        if (keyword == "v") {
            // vertex
            Vector3d x;
            linestream >> x[0] >> x[1] >> x[2];
            addVertex(x);
        } else if (keyword == "f") {
            // face(s)
            vector<int> polygon;
            string w;
            while (linestream >> w) {
                stringstream wstream(w);
                int v;
                wstream >> v;
                polygon.push_back(v-1);
                int n = polygon.size();
                if (n >= 3)
                    addTriangle(Vector3i(polygon[0], polygon[n-2], polygon[n-1]));
            }
        }
    }
}

void drawTriangles(vector<Vector3d> &vertices, vector<Vector3i> &triangles) {
    glBegin(GL_TRIANGLES);
    for (int t = 0; t < triangles.size(); t++) {
        Vector3i tri = triangles[t];
        Vector3d e1 = vertices[tri[1]] - vertices[tri[0]],
                 e2 = vertices[tri[2]] - vertices[tri[0]];
        Vector3d normal = e1.cross(e2).normalized();
        glNormal3f(normal[0], normal[1], normal[2]);
        for (int v = 0; v < 3; v++) {
            Vector3d vertex = vertices[tri[v]];
            glVertex3f(vertex[0], vertex[1], vertex[2]);
        }
    }
    glEnd();
}

void drawEdges(vector<Vector3d> &vertices, vector<Vector2i> &edges) {
    glBegin(GL_LINES);
    for (int e = 0; e < edges.size(); e++) {
        Vector2i edge = edges[e];
        for (int v = 0; v < 2; v++) {
            Vector3d vertex = vertices[edge[v]];
            glVertex3f(vertex[0], vertex[1], vertex[2]);
        }
    }
    glEnd();
}

void TriMesh::draw(Vector4d fillColor, Vector4d edgeColor) {
    glColor4f(fillColor[0], fillColor[1], fillColor[2], fillColor[3]);
    drawTriangles(vertices, triangles);
    glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
    drawEdges(vertices, edges);
}

TriMesh TriMesh::sphere(Vector3d center, double radius) {
    TriMesh mesh;
    int stacks = 50, slices = 100;
    for (int a=0; a<=stacks; a++) {
        for (int c=0; c<slices; c++) {
            int b = c;
            double lat = M_PI*a/stacks - M_PI/2;
            double lon = 2*M_PI*b/slices;
            Vector3d vert = center + radius*
                Vector3d(sin(lon)*cos(lat), sin(lat), cos(lon)*cos(lat));
            mesh.addVertex(vert);
            if (a > 0) {
                int v = mesh.vertices.size() - 1;
                if (c == 0) {
                    mesh.addTriangle(Vector3i(v, v-1, v-slices));
                    mesh.addTriangle(Vector3i(v, v+slices-1, v-1));
                } else {
                    mesh.addTriangle(Vector3i(v, v-slices-1, v-slices));
                    mesh.addTriangle(Vector3i(v, v-1, v-slices-1));
                }
            }
        }
    }
    return mesh;
}

TriMesh TriMesh::rectangle(Vector3d center, Vector3d axis1, Vector3d axis2) {
    TriMesh mesh;
    mesh.addVertex(center - axis1/2 - axis2/2);
    mesh.addVertex(center + axis1/2 - axis2/2);
    mesh.addVertex(center + axis1/2 + axis2/2);
    mesh.addVertex(center - axis1/2 + axis2/2);
    mesh.addTriangle(Vector3i(0,1,2));
    mesh.addTriangle(Vector3i(0,2,3));
    return mesh;
}

TriMesh TriMesh::box(Vector3d center,
                     Vector3d axis1, Vector3d axis2, Vector3d axis3) {
    TriMesh mesh;
    for (int i = -1; i <= 1; i += 2)
        for (int j = -1; j <= 1; j += 2)
            for (int k = -1; k <= 1; k += 2)
                mesh.addVertex(center + i*axis1/2 + j*axis2/2 + k*axis3/2);
    Vector3i triangles[] = {
        Vector3i(0,1,2),
        Vector3i(1,3,2),
        Vector3i(0,4,1),
        Vector3i(1,4,5),
        Vector3i(0,2,4),
        Vector3i(2,6,4),
        Vector3i(1,5,3),
        Vector3i(3,5,7),
        Vector3i(2,3,6),
        Vector3i(3,7,6),
        Vector3i(4,6,5),
        Vector3i(5,6,7)
    };
    for (int t = 0; t < 12; t++)
        mesh.addTriangle(triangles[t]);
    return mesh;
}

void get_valid_line(istream &in, string &line) {
    do
        getline(in, line);
    while (in && (line.length() == 0 || line[0] == '#'));
}

Vector3i makeTriangle(int i, int j, int k) {
    if (i < j && i < k)
        return Vector3i(i, j, k);
    else if (j < i && j < k)
        return Vector3i(j, k, i);
    else
        return Vector3i(k, i, j);
}

TetMesh::TetMesh(const string &filename) {
    { // nodes
        string nodefile = filename + ".node";
        fstream file(nodefile.c_str(), ios::in);
        if(!file) {
            cerr << "Error: failed to open file " << nodefile << endl;
            return;
        }
        string line;
        get_valid_line(file, line);
        stringstream linestream(line);
        int nv, dim, na, nb;
        linestream >> nv >> dim >> na >> nb;
        for (int i = 0; i < nv; i++) {
            get_valid_line(file, line);
            stringstream linestream(line);
            int index;
            linestream >> index;
            Vector3d vert;
            linestream >> vert[0] >> vert[1] >> vert[2];
            vertices.push_back(vert);
        }
    }
    { // tets
        string elefile = filename + ".ele";
        fstream file(elefile.c_str(), ios::in);
        if(!file) {
            cerr << "Error: failed to open file " << elefile << endl;
            return;
        }
        string line;
        get_valid_line(file, line);
        stringstream linestream(line);
        int nt, nn, na;
        linestream >> nt >> nn >> na;
        for (int i = 0; i < nt; i++) {
            get_valid_line(file, line);
            stringstream linestream(line);
            int index;
            linestream >> index;
            Vector4i tet;
            linestream >> tet[0] >> tet[1] >> tet[2] >> tet[3];
            tetrahedra.push_back(tet);
        }
    }
    { // triangles
        vector<bool> isSurfaceTriangle;
        for (int t = 0; t < tetrahedra.size(); t++) {
            Vector4i tet = tetrahedra[t];
            Vector3i tris[4] = {
                makeTriangle(tet[0], tet[2], tet[1]),
                makeTriangle(tet[0], tet[1], tet[3]),
                makeTriangle(tet[0], tet[3], tet[2]),
                makeTriangle(tet[1], tet[2], tet[3])
            };
            for (int r = 0; r < 4; r++) {
                Vector3i tri = tris[r];
                Vector3i revTri(tri[0], tri[2], tri[1]);
                int revIndex = findIndex(revTri, triangles);
                if (revIndex == triangles.size()) {
                    // not found
                    triangles.push_back(tri);
                    isSurfaceTriangle.push_back(true);
                } else {
                    isSurfaceTriangle[revIndex] = false;
                }
            }
        }
        for (int r = 0; r < triangles.size(); r++)
            if (isSurfaceTriangle[r])
                surfaceTriangles.push_back(triangles[r]);
    }
    { // edges
        for (int r = 0; r < triangles.size(); r++) {
            Vector3i tri = triangles[r];
            for (int e = 0; e < 3; e++) {
                Vector2i edge(tri[e], tri[(e+1)%3]);
                if (edge[1] < edge[0])
                    edge = Vector2i(edge[1], edge[0]);
                if (!isIn(edge, edges))
                    edges.push_back(edge);
            }
        }
        for (int r = 0; r < surfaceTriangles.size(); r++) {
            Vector3i tri = surfaceTriangles[r];
            for (int e = 0; e < 3; e++) {
                Vector2i edge(tri[e], tri[(e+1)%3]);
                if (edge[1] < edge[0])
                    edge = Vector2i(edge[1], edge[0]);
                if (!isIn(edge, surfaceEdges))
                    surfaceEdges.push_back(edge);
            }
        }
    }
}

void TetMesh::draw(Vector4d fillColor, Vector4d edgeColor) {
    glColor4f(fillColor[0], fillColor[1], fillColor[2], fillColor[3]);
    drawTriangles(vertices, surfaceTriangles);
    glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
    drawEdges(vertices, surfaceEdges);
}

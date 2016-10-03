#include "mesh.hpp"

#include <fstream>
#include "gui.hpp"

void get_valid_line (istream &in, string &line) {
    do
        getline(in, line);
    while (in && (line.length() == 0 || line[0] == '#'));
}

Mesh2D::Mesh2D(const string &filename) {
    { // nodes
        string nodefile = filename + ".node";
        fstream file(nodefile.c_str(), ios::in);
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
            Vector2d vert;
            linestream >> vert[0] >> vert[1];
            vertices.push_back(vert);
        }
    }
    { // triangles
        string elefile = filename + ".ele";
        fstream file(elefile.c_str(), ios::in);
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
            Vector3i tri;
            linestream >> tri[0] >> tri[1] >> tri[2];
            triangles.push_back(tri);
        }
    }
    { // edges
        for (int t = 0; t < triangles.size(); t++) {
            Vector3i tri = triangles[t];
            for (int e = 0; e < 3; e++) {
                Vector2i edge(tri[e], tri[(e+1)%3]);
                Vector2i revEdge(edge[1], edge[0]);
                if (find(edges.begin(), edges.end(), edge) == edges.end()
                    && find(edges.begin(), edges.end(), revEdge) == edges.end())
                    edges.push_back(edge);
                    edges.push_back(revEdge);
            }
        }
    }
}

void Mesh2D::draw(Vector4d fillColor, Vector4d edgeColor) {
    glColor4f(fillColor[0], fillColor[1], fillColor[2], fillColor[3]);
    glBegin(GL_TRIANGLES);
    for (int t = 0; t < triangles.size(); t++) {
        Vector3i tri = triangles[t];
        for (int v = 0; v < 3; v++) {
            Vector2d vertex = vertices[tri[v]];
            glVertex2f(vertex[0], vertex[1]);
        }
    }
    glEnd();
    glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
    glBegin(GL_LINES);
    for (int e = 0; e < edges.size(); e++) {
        Vector2i edge = edges[e];
        for (int v = 0; v < 2; v++) {
            Vector2d vertex = vertices[edge[v]];
            glVertex2f(vertex[0], vertex[1]);
        }
    }
    glEnd();
}

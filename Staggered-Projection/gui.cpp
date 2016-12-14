#include "gui.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
using namespace std;

void Camera::lookAt(Vector3d eye, Vector3d target, Vector3d up) {
    this->target = target;
    this->u = up;
    Vector3d r = (eye - target).normalized();
    this->w = up.cross(r).normalized();
    this->v = w.cross(u).normalized();
    this->distance = (eye - target).norm();
    this->latitude = asin(r.dot(up))*180/M_PI;
    this->longitude = 45;
}

void Camera::setFOV(double fov) {
    this->fov = fov;
}

void Camera::apply(Vector2i size) {
    double aspect = (double)size[0]/size[1];
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, aspect, 0.01*distance, 100*distance);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -distance);
    glRotatef(latitude, 1, 0, 0);
    glRotatef(longitude, 0, 1, 0);
    glTranslatef(-target(0), -target(1), -target(2));
}

static void error_callback(int error, const char* description) {
    cerr << "Error: " << description << endl;
}

Window3D::Window3D(Vector2i size) {
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(1);
    window = glfwCreateWindow(size[0], size[1], "Animation", nullptr, nullptr);
    glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);
    glfwMakeContextCurrent(window);
    prepareDisplay();
}

Window3D::~Window3D() {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Window3D::prepareDisplay() {
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    // glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    glEnable(GL_NORMALIZE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1,1);
    Vector2i size;
    glfwGetFramebufferSize(window, &size[0], &size[1]);
    camera.apply(size);
    lastTime = glfwGetTime();
}

bool Window3D::shouldClose() {
    return glfwWindowShouldClose(window);
}

void Window3D::updateAndWait(double dt) {
    glfwSwapBuffers(window);
    glfwPollEvents();
    double currentTime = glfwGetTime();
    double waitTime = lastTime + dt - currentTime;
    lastTime = currentTime;
    this_thread::sleep_for(chrono::microseconds(int(waitTime*1e6)));
    Vector2d mousePos;
    glfwGetCursorPos(window, &mousePos[0], &mousePos[1]);
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        double dragSpeed = 0.5; // degrees per pixel
        Vector2d drag = mousePos - lastMousePos;
        camera.longitude += drag[0]*dragSpeed;
        camera.latitude += drag[1]*dragSpeed;
    }
    lastMousePos = mousePos;
    prepareDisplay();
}

bool Window3D::keyPressed(int key) {
    return glfwGetKey(window, key);
}

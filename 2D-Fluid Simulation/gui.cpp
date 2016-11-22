#include "gui.hpp"

#include <iostream>
#include <chrono>
#include <thread>
using namespace std;

static void error_callback(int error, const char* description) {
    cerr << "Error: " << description << endl;
}

Window2D::Window2D(Vector2i size, Vector2d xmin, Vector2d xmax):
    xmin(xmin), xmax(xmax) {
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(1);
    window = glfwCreateWindow(size[0], size[1], "Animation", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    prepareDisplay();
}

Window2D::~Window2D() {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Window2D::prepareDisplay() {
    glClearColor(1,1,1, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(xmin[0], xmax[0], xmin[1], xmax[1], -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    lastTime = glfwGetTime();
}

bool Window2D::shouldClose() {
    return glfwWindowShouldClose(window);
}

void Window2D::updateAndWait(double dt) {
    glfwSwapBuffers(window);
    glfwPollEvents();
    double currentTime = glfwGetTime();
    double waitTime = lastTime + dt - currentTime;
    lastTime = currentTime;
    this_thread::sleep_for(chrono::microseconds(int(waitTime*1e6)));
    prepareDisplay();
}

Vector2d Window2D::mousePos() {
    Vector2d pos;
    glfwGetCursorPos(window, &pos[0], &pos[1]);
    Vector2i size;
    glfwGetWindowSize(window, &size[0], &size[1]);
    pos = Vector2d(xmin[0] + pos[0]/size[0] * (xmax[0] - xmin[0]),
                   xmax[1] - pos[1]/size[1] * (xmax[1] - xmin[1]));
    return pos;
}

bool Window2D::mouseDown() {
    return (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
}

#ifndef GUI_HPP
#define GUI_HPP

#include <GLFW/glfw3.h>
#include <Eigen/Dense>
using namespace Eigen;

class Window2D {
public:
    GLFWwindow *window;
    Vector2d xmin, xmax;
    double lastTime;
    Window2D(Vector2i size, Vector2d xmin, Vector2d xmax);
    ~Window2D();
    void prepareDisplay();
    bool shouldClose();
    void updateAndWait(double dt);
    Vector2d mousePos();
    bool mouseDown();
};

#endif

#ifndef GUI_HPP
#define GUI_HPP

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
using namespace Eigen;

class Camera {
public:
    // a basic camera that orbits the target and never rolls
    Vector3d target;                      // target position
    Vector3d u, v, w;                     // coordinate system with up vector
    double distance, latitude, longitude; // position relative to target
    double fov;                           // vertical field of view
    Camera() {
        lookAt(Vector3d(0,0,1), Vector3d(0,0,0), Vector3d(0,1,0));
        setFOV(90);
    }
    void lookAt(Vector3d eye, Vector3d target, Vector3d up);
    void setFOV(double fov);
    void apply(Vector2i size);
};

class Window3D {
public:
    GLFWwindow *window;
    Camera camera;
    double lastTime;
    Vector2d lastMousePos;
    Window3D(Vector2i size);
    ~Window3D();
    void prepareDisplay();
    bool shouldClose();
    void updateAndWait(double dt);
    bool keyPressed(int key);
};

#endif

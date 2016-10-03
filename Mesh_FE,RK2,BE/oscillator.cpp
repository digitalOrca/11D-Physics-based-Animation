#include "oscillator.hpp"

int DampedHarmonicOscillator::getNumDOFs() {
    return 1;
}

void DampedHarmonicOscillator::getState(VectorXd &pos, VectorXd &vel, double &time) {
    pos[0] = x;
    vel[0] = v;
    time = t;
}

void DampedHarmonicOscillator::setState(const VectorXd &pos, const VectorXd &vel, double time) {
    x = pos[0];
    v = vel[0];
    t = time;
}

void DampedHarmonicOscillator::getInertia(MatrixXd &M) {
    M(0,0) = m;
}

void DampedHarmonicOscillator::getForces(VectorXd &f) {
    f[0] = -k*x - c*v;
}

void DampedHarmonicOscillator::getAccelerations(VectorXd &a) {
    a[0] = (-k*x - c*v)/m;
}

void DampedHarmonicOscillator::getJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    Jx(0,0) = -k;
    Jv(0,0) = -c;
}

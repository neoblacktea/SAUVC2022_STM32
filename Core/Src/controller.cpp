#include "controller.h"
#include "math.h"

Controller::Controller(geometry::Vector &x, geometry::Vector &v, geometry::Vector &R, geometry::Vector &Omega, float alpha): Kx(x), Kv(v), KR(R), KOmega(Omega), Alpha_sonar(alpha)
{
}

Controller::~Controller()
{
}

void Controller::adjust_yaw(Dynamics &s, float ys)
{
    Quaternion qy(cosf(ys / 2.0), 0, 0, sinf(ys / 2.0));
    s.orientation = (1.0 - Alpha_sonar) * s.orientation + Alpha_sonar * qy;
}

void Controller::update(Dynamics &s, const geometry::Vector &ex, const geometry::Vector &ev, float yaw_sonar, Kinematics &ctrl_input)
{
    adjust_yaw(s, yaw_sonar);

}
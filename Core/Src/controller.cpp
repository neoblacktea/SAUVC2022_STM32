#include "controller.h"
#include "math.h"

/**
 * @brief Constructor of Controller
 * @param x Kx
 * @param v Kv
 * @param R KR
 * @param Omega KOmega
 * @param alpha 0~1 how much we trust sonar yaw
 */ 
Controller::Controller(geometry::Vector x, geometry::Vector v, geometry::Vector R, geometry::Vector Omega, float alpha): Kx(x), Kv(v), KR(R), KOmega(Omega), Alpha_sonar(alpha)
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
    eR.x = (-4) * s.orientation.w * s.orientation.x;
    eR.y = (-4) * s.orientation.w * s.orientation.y;
    eR.z = (-4) * s.orientation.w * s.orientation.z;
    eOmega.x = s.velocity.angular.x;
    eOmega.y = s.velocity.angular.y;
    eOmega.z = s.velocity.angular.z;

    ctrl_input.linear.x = Kx.x * ex.x + Kv.x * ev.x;
    ctrl_input.linear.y = Kx.y * ex.y + Kv.y * ev.y;
    ctrl_input.linear.z = Kx.z * ex.z + Kv.z * ev.z;
    ctrl_input.angular.x = KR.x * eR.x + KOmega.x * eOmega.x;
    ctrl_input.angular.y = KR.y * eR.y + KOmega.y * eOmega.y;
    ctrl_input.angular.z = KR.z * eR.z + KOmega.z * eOmega.z;
}
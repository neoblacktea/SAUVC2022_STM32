#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Datatype/dynamics.h"

class Controller
{
private:
    const float Kx;
    const float Kv;
    const float KR;
    const float KOmega;
    const float Alpha_sonar;  //0~1 how much we trust sonar yaw
    // geometry::Vector ex;
    // geometry::Vector ev;
    // geometry::Vector eR;
    // geometry::Vector eOmega;

public:
    Controller(float x, float v, float R, float Omega, float alpha);
    ~Controller();
    void adjust_yaw(Dynamics &s, float ys);
    void update(Dynamics &s, const geometry::Vector &ex, const geometry::Vector &ev, float yaw_sonar, Kinematics &ctrl_input);
};



#endif
#ifndef QUATERNION_H
#define QUATERNION_H

struct Quaternion
{
public:
    float w;
    float x;
    float y;
    float z;

    Quaternion();
    Quaternion(float q1, float q2, float q3, float q4);
    void reset(float q1 = 0, float q2 = 0, float q3 = 0, float q4 = 0);
    Quaternion operator+(const Quaternion &q2);
    Quaternion operator-(const Quaternion &q2);
    Quaternion operator*(const Quaternion &q2);
    Quaternion operator*(const float &scalar);
    void operator*=(const Quaternion &q2);
    void operator*=(const float &scalar);
    Quaternion conjugate();
    float norm();
    void normalize();
};

Quaternion operator*(const float &scalar, const Quaternion &q1);

#endif
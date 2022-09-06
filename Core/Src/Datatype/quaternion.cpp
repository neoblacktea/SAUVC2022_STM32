#include "Datatype/quaternion.h"
#include <math.h>

/**
 * @brief a Quaterninon class
 * @param q1 w
 * @param q2 x
 * @param q3 y
 * @param q4 z
 */
Quaternion::Quaternion()
{
    w = 0;
    y = 0;
    z = 0;
    x = 0;
}

/**
 * @brief a Quaterninon class
 * @param q1 w
 * @param q2 x
 * @param q3 y
 * @param q4 z
 */
Quaternion::Quaternion(float q1, float q2, float q3, float q4)
{
    w = q1;
    x = q2;
    y = q3;
    z = q4;
}

void Quaternion::reset(float q1, float q2, float q3, float q4)
{
    w = q1;
    y = q2;
    z = q3;
    x = q4;
}

Quaternion Quaternion::operator+(const Quaternion &q2)
{
    return Quaternion(w + q2.w, x + q2.x, y + q2.y, z + q2.z);
}

Quaternion Quaternion::operator-(const Quaternion &q2)
{
    return Quaternion(w - q2.w, x - q2.x, y - q2.y, z - q2.z);
}

Quaternion Quaternion::operator*(const Quaternion &q2)
{
    Quaternion q3;
    
    q3.w = w * q2.w - x * q2.x - y * q2.y - z * q2.z;
    q3.x = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
    q3.y = w * q2.y - x * q2.z + y * q2.w + z * q2.x;
    q3.z = w * q2.z + x * q2.y - y * q2.x + z * q2.w;
    
    return q3;
}

Quaternion Quaternion::operator*(const float &scalar)
{
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
}

void Quaternion::operator*=(const Quaternion &q2)
{
    w = w * q2.w - x * q2.x - y * q2.y - z * q2.z;
    x = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
    y = w * q2.y - x * q2.z + y * q2.w + z * q2.x;
    z = w * q2.z + x * q2.y - y * q2.x + z * q2.w;
}

void Quaternion::operator*=(const float &scalar)
{
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
}

Quaternion Quaternion::conjugate()
{
    Quaternion q2(*this);
    q2.x = -x;
    q2.y = -y;
    q2.z = -z;
    return q2;
}

float Quaternion::norm()
{
    return sqrt(w * w + x * x + y * y + z * z);
}

void Quaternion::normalize()
{
    float norm = this->norm();

    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
}

Quaternion operator*(const float &scalar, const Quaternion &q1)
{
    return Quaternion(q1.w * scalar, q1.x * scalar, q1.y * scalar, q1.z * scalar);
}
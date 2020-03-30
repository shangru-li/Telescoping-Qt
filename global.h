#pragma once
#ifndef GLOBAL_H
#define GLOBAL_H

#include <glm/glm.hpp>

// glm::translate, glm::rotate, and glm::scale.
#include <glm/gtc/matrix_transform.hpp>

// glm::translate, glm::rotate, and glm::scale.
#include <glm/gtx/matrix_transform_2d.hpp>

// glm::to_string.
#include <glm/gtx/string_cast.hpp>

#include <glm/gtx/quaternion.hpp>


#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_2_Core>
#include <QApplication>
#include <QKeyEvent>
#include <QTextStream>

#include <vector>
#include <array>
#include <memory>

#include <iostream>
#include <math.h>
#define M_PI 3.141592

using namespace std;

extern glm::vec4 _red;
extern glm::vec4 _green;
extern glm::vec4 _yellow;
extern glm::vec4 _blue;
extern glm::vec4 _black;
extern glm::vec4 _white;
extern glm::vec4 _grey;
extern glm::vec4 _purple;
extern glm::vec4 _normal;
extern glm::mat4 _I;
extern float _MIN_SHELL_LENGTH;

inline void pl(string s)
{
    cout << s << endl;
}

inline void pl(float f, string s = "")
{
    cout << s << ": " << f << endl;
}

inline void pl(glm::vec2 v, string s = "")
{
    cout << s << ": " << v.x << ", " << v.y << endl;
}

inline void pl(glm::vec3 v, string s = "")
{
    cout << s << ": " << glm::to_string(v) << endl;
}

inline void pl(glm::vec4 v, string s = "")
{
    cout << s << ": " << glm::to_string(v) << endl;
}

inline void pl(glm::mat3 m, string s = "")
{
    cout << s << ": " << glm::to_string(m) << endl;
}

inline void pl(glm::mat4 m, string s = "")
{
    cout << s << ": " << glm::to_string(m) << endl;
}

inline std::string fromFile(const char *fileName)
{
    QString text;
    QFile file(fileName);
    if (file.open(QFile::ReadOnly))
    {
        QTextStream in(&file);
        text = in.readAll();
        text.append('\0');
    }
    return text.toStdString();
}

inline float angleBetween(glm::vec3 from, glm::vec3 to, glm::vec3 up)
{
    glm::vec3 cross = glm::cross(from, to);
    float sgn = 0.0f;
    float dot = glm::dot(cross, up);
    if(dot > 0.0f || fabs(dot) < FLT_EPSILON)
        sgn = 1.0f;
    else
        sgn = -1.0f;
    return glm::degrees(sgn * atan2(glm::length(cross), glm::dot(from, to)));
}

inline glm::fquat lookRotation(glm::vec3 forward, glm::vec3 upward)
{
    forward = glm::normalize(forward);

    glm::vec3 vector = glm::normalize(forward);
    glm::vec3 vector2 = glm::normalize(glm::cross(upward, vector));
    glm::vec3 vector3 = glm::cross(vector, vector2);
    float m00 = vector2.x;
    float m01 = vector2.y;
    float m02 = vector2.z;
    float m10 = vector3.x;
    float m11 = vector3.y;
    float m12 = vector3.z;
    float m20 = vector.x;
    float m21 = vector.y;
    float m22 = vector.z;


    float num8 = (m00 + m11) + m22;
    glm::fquat quaternion;
    if (num8 > 0.0f)
    {
        float num = (float)sqrt(num8 + 1.0f);
        quaternion.w = num * 0.5f;
        num = 0.5f / num;
        quaternion.x = (m12 - m21) * num;
        quaternion.y = (m20 - m02) * num;
        quaternion.z = (m01 - m10) * num;
        return quaternion;
    }
    if ((m00 >= m11) && (m00 >= m22))
    {
        float num7 = (float)sqrt(((1.0f + m00) - m11) - m22);
        float num4 = 0.5f / num7;
        quaternion.x = 0.5f * num7;
        quaternion.y = (m01 + m10) * num4;
        quaternion.z = (m02 + m20) * num4;
        quaternion.w = (m12 - m21) * num4;
        return quaternion;
    }
    if (m11 > m22)
    {
        float num6 = (float)sqrt(((1.0f + m11) - m00) - m22);
        float num3 = 0.5f / num6;
        quaternion.x = (m10+ m01) * num3;
        quaternion.y = 0.5f * num6;
        quaternion.z = (m21 + m12) * num3;
        quaternion.w = (m20 - m02) * num3;
        return quaternion;
    }
    float num5 = (float)sqrt(((1.0f + m22) - m00) - m11);
    float num2 = 0.5f / num5;
    quaternion.x = (m20 + m02) * num2;
    quaternion.y = (m21 + m12) * num2;
    quaternion.z = 0.5f * num5;
    quaternion.w = (m01 - m10) * num2;
    return quaternion;
}



#endif // GLOBAL_H

#ifndef GLOBAL_H
#define GLOBAL_H

#include <glm/glm.hpp>

// glm::translate, glm::rotate, and glm::scale.
#include <glm/gtc/matrix_transform.hpp>

// glm::translate, glm::rotate, and glm::scale.
#include <glm/gtx/matrix_transform_2d.hpp>

// glm::to_string.
#include <glm/gtx/string_cast.hpp>


#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_2_Core>
#include <QApplication>
#include <QKeyEvent>
#include <QTextStream>

#include <vector>
#include <memory>

#include <iostream>
using namespace std;

inline void pl(string s)
{
    cout << s << endl;
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

#endif // GLOBAL_H

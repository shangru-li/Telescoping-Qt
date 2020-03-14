#-------------------------------------------------
#
# Project created by QtCreator 2020-03-09T13:37:40
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RayTracerGPU
TEMPLATE = app

CONFIG += c++1z

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
        RenderUtility/glcontext.cpp \
        Geometry/drawable.cpp \
    Geometry/squareplane.cpp \
    RenderUtility/shaderprogram.cpp \
    RenderUtility/camera.cpp \
    Geometry/cube.cpp \
    global.cpp \
    RayCasting/intersection.cpp \
    RayCasting/ray.cpp \
    Geometry/scene.cpp \
    Geometry/cubearray.cpp \
    Geometry/curve.cpp

HEADERS += \
        mainwindow.h \
        RenderUtility/glcontext.h \
        global.h \
        Geometry/drawable.h \
    Geometry/squareplane.h \
    RenderUtility/shaderprogram.h \
    RenderUtility/camera.h \
    Geometry/cube.h \
    RayCasting/intersection.h \
    RayCasting/ray.h \
    Geometry/scene.h \
    Geometry/cubearray.h \
    Geometry/curve.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += include

DISTFILES +=

RESOURCES += resources.qrc

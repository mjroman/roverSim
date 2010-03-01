# -------------------------------------------------
# Project created by QtCreator 2009-10-02T12:35:15
# -------------------------------------------------
CONFIG += qt
DESTDIR = build
# QT-= gui to build project without gui library
QT += opengl # add network, xml, xmlpatterns for more qt modules
QT += network

TARGET = QtRoverSimulator
CONFIG(debug, debug|release){
	mac: TARGET = $$join(TARGET,,,_debug)
}

unix:QMAKE_DEL_FILE=rm -rf
 
OBJECTS_DIR = build/obj
MOC_DIR = build/moc
UI_DIR = build/ui
RCC_DIR = build/rcc
 
QMAKE_DISTCLEAN += build/obj build/moc build/ui build/rcc build


TEMPLATE = app \
    /src
INCLUDEPATH += . \
    src
SOURCES += main.cpp \
    src/mainGUI.cpp \
	src/simcontrol.cpp \
    src/physicsWorld.cpp \
    src/simglview.cpp \
    src/camera.cpp \
    src/utility/rvgs.c \
    src/utility/rngs.c \
    src/utility/definitions.c \
    src/terrain.cpp \
	src/robot.cpp \
    src/sr2rover.cpp \
    src/laserscanner.cpp \
    src/utility/glshapes.c \
	src/utility/glparticle.cpp \
    src/tools/terraintool.cpp \
    src/tools/simtool.cpp \
    src/tools/obstacletool.cpp \
    src/skydome.cpp
HEADERS += src/mainGUI.h \
	src/simcontrol.h \
    src/physicsWorld.h \
    src/simglview.h \
    src/camera.h \
    src/utility/rvgs.h \
    src/utility/rngs.h \
    src/utility/definitions.h \
    src/terrain.h \
	src/robot.h \
    src/sr2rover.h \
    src/laserscanner.h \
    src/simglobject.h \
    src/utility/glshapes.h \
	src/utility/glparticle.h \
    src/tools/terraintool.h \
    src/tools/simtool.h \
    src/tools/obstacletool.h \
    src/skydome.h
FORMS += src/mainGUI.ui \
    src/tools/terraintool.ui \
    src/tools/obstacletool.ui
LIBS += -framework BulletCollision \
    -framework BulletDynamics \
    -framework LinearMath
RESOURCES += images.qrc
RC_FILE = redrobot.icns

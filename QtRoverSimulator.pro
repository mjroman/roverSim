# -------------------------------------------------
# Project created by QtCreator 2009-10-02T12:35:15
# -------------------------------------------------
CONFIG += qt
DESTDIR = build
# QT-= gui to build project without gui library
QT += opengl # add network, xml, xmlpatterns for more qt modules
QT += network
QT += xml

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
	src/obstacles.cpp \
	src/robot.cpp \
    src/sr2rover.cpp \
    src/laserscanner.cpp \
    src/utility/glshapes.c \
	src/utility/glparticle.cpp \
    src/tools/terraintool.cpp \
	src/tools/obstacletool.cpp \
    src/tools/simtool.cpp \
	src/tools/waypointtool.cpp \
    src/skydome.cpp \
	src/autoCode.cpp \
	src/cSpace.cpp \
	src/pathPlan.cpp \
	src/tools/pathtool.cpp
	
HEADERS += src/mainGUI.h \
	src/simcontrol.h \
    src/physicsWorld.h \
    src/simglview.h \
    src/camera.h \
    src/utility/rvgs.h \
    src/utility/rngs.h \
    src/utility/definitions.h \
	src/utility/structures.h \
    src/terrain.h \
	src/obstacles.h \
	src/robot.h \
    src/sr2rover.h \
    src/laserscanner.h \
    src/simglobject.h \
    src/utility/glshapes.h \
	src/utility/glparticle.h \
    src/tools/terraintool.h \
	src/tools/obstacletool.h \
    src/tools/simtool.h \
	src/tools/waypointtool.h \
    src/skydome.h \
	src/autoCode.h \
	src/cSpace.h \
	src/pathPlan.h \
	src/tools/pathtool.h
	
FORMS += src/mainGUI.ui \
    src/tools/terraintool.ui \
    src/tools/obstacletool.ui \
	src/tools/waypointtool.ui \
	src/tools/navigationtool.ui \
	src/tools/pathtool.ui
	
# Bullet 2.75 frameworks	
#LIBS += -framework LinearMath \
#	-framework BulletDynamics \
#	-framework BulletCollision 
    
# Bullet 2.76 frameworks built for i386 and not DOUBLE_PRECISION
LIBS += -L/usr/local \
		-lBulletDynamics \
		-lBulletCollision \
		-lLinearMath	
		
RESOURCES += resource.qrc
RC_FILE = redrobot.icns

#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QtGui>    // needed for mouse events
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

enum viewIndex {
        FreeView,
        RoverCenter,
        RoverFollow,
        RoverView,
        RoverPanCam
};

#define CAMERAINDEXSIZE 5 // should be equal to the number of elements in the above enum

class rover;

class camera : public QObject
{
private:
    btVector3           position,direction,upVector;
    float               pitch[CAMERAINDEXSIZE];
    float               yaw[CAMERAINDEXSIZE];
    float               zoom[CAMERAINDEXSIZE];
    rover               *robot;

public:
    int                 cameraView;

    camera(const btVector3& pos,const btVector3& dir);
    void cameraUpdate();
    void cameraMouseMove(QPoint delta,QMouseEvent *event);
    void cameraMouseWheel(QWheelEvent *event);
    void cameraToggleView();
	QString	cameraViewName();
    void cameraSetRoverPointer(rover* rp){ robot = rp; }
    btVector3 cameraDirection(){ return direction; }
    btVector3 cameraPitchYawZoom();
};

#endif // CAMERA_H

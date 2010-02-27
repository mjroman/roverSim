#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QtGui>    // needed for mouse events
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

#define CAMERAINDEXSIZE 5 // should be equal to the number of elements in the BELOW enum
enum viewIndex {
        FreeView,
        RoverCenter,
        RoverFollow,
        RoverView,
        RoverPanCam
};

class SR2rover;

class camera : public QObject
{
private:
    btVector3           position,direction,upVector;
    float               pitch[CAMERAINDEXSIZE];
    float               yaw[CAMERAINDEXSIZE];
    float               zoom[CAMERAINDEXSIZE];
    SR2rover            *bot;

public:
    int                 cameraView;

    camera(const btVector3& pos,const btVector3& dir);
    void cameraUpdate();
    void cameraMouseMove(QPoint delta,QMouseEvent *event);
    void cameraMouseWheel(QWheelEvent *event);
    void cameraToggleView();
	void cameraSetView(viewIndex x);
	QString	cameraViewName();
    void cameraSetRoverPointer(SR2rover* rp){ bot = rp; }
    btVector3 cameraDirection(){ return direction; }
    btVector3 cameraPitchYawZoom();

	void cameraFreeView();
	void cameraRoverCenter();
	void cameraRoverFollow();
	void cameraRoverView();
	void cameraRoverPanCam();
};

#endif // CAMERA_H

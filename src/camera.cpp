#include "camera.h"
#include "rover.h"
#include "utility/definitions.h"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

camera::camera(const btVector3& pos,const btVector3& dir)
{
    cameraView = FreeView;
    pitch[FreeView] = 75.;
    yaw[FreeView] = 45.;
    zoom[FreeView] = 40.0;

    pitch[RoverCenter] = 75.;
    yaw[RoverCenter] = 45.;
    zoom[RoverCenter] = 5.0;

    pitch[RoverFollow] = 75.;
    yaw[RoverFollow] = 0;
    zoom[RoverFollow] = 3.6;

    pitch[RoverView] = -15;
    yaw[RoverView] = 0;
    zoom[RoverView] = 1.0;

    pitch[RoverPanCam] = 0;
    yaw[RoverPanCam] = 0;
    zoom[RoverPanCam] = 1.0;

    position = pos;
    robot = 0;
    direction = dir;
	
    btVector3 upVector(0.f,1.f,0.f);
}

void camera::cameraUpdate()
{
    if(!robot) cameraView = FreeView;

    float tPitch = pitch[cameraView];
    float tYaw = yaw[cameraView];

    switch(cameraView){
    case RoverView: // this is the view from the rover nav cams
        {
            zoom[cameraView] = 1.0;
            yaw[cameraView] = 0;

            btTransform rFrame = robot->getRoverTransform();
            position = rFrame(btVector3(0,0.022,0));
            direction = rFrame(btVector3(0,cos(DEGTORAD(tPitch)),sin(DEGTORAD(tPitch))));
            upVector = rFrame(btVector3(0,0,1)) - robot->position;
			robot->paintBodyLaser(false);
            break;
        }
    case RoverPanCam:   // the PanCam view
        {
            zoom[cameraView] = 1.0;
            yaw[cameraView] = 0;
            pitch[cameraView] =  0;

            float pan = -DEGTORAD(robot->panAngle);
            float tilt = DEGTORAD(robot->tiltAngle);
            direction.setX(sin(pan)*cos(tilt));
            direction.setY(cos(pan)*cos(tilt));
            direction.setZ(sin(tilt));
            btVector3 horz(sin(pan+PI/2),cos(pan+PI/2),0);
            btVector3 camUp = horz.cross(direction);

            btTransform rFrame = robot->getRoverTransform();
            position = rFrame(btVector3(0,0.03,0.4));
            direction = rFrame(direction+btVector3(0,0.03,0.4));
            upVector = rFrame(camUp) - robot->position;
            break;
        }
    case RoverFollow:   // floating view which follows the rovers heading
        tYaw = RADTODEG(robot->heading) + yaw[cameraView];
    case RoverCenter:   // floating view which follows the rovers position but not heading
        direction = robot->position;
		robot->paintBodyLaser(true);
    case FreeView:      // free camera view
    default:

        position.setX(direction.x() - zoom[cameraView]*sin(DEGTORAD(tPitch))*sin(DEGTORAD(tYaw)));
        position.setY(direction.y() - zoom[cameraView]*sin(DEGTORAD(tPitch))*cos(DEGTORAD(tYaw)));
        position.setZ(direction.z() + zoom[cameraView]*cos(DEGTORAD(tPitch)));

        upVector.setX(cos(DEGTORAD(tPitch))*sin(DEGTORAD(tYaw)));
        upVector.setY(cos(DEGTORAD(tPitch))*cos(DEGTORAD(tYaw)));
        upVector.setZ(sin(DEGTORAD(tPitch)));
		
        break;
    }

    gluLookAt(position.x(), position.y(), position.z(), direction.x(), direction.y(), direction.z(), upVector.x(), upVector.y(), upVector.z());
}

void camera::cameraMouseMove(QPoint delta,QMouseEvent *event)
{
    if(event->modifiers() & Qt::ShiftModifier)
    {
        // the camera can be zoomed in and out
        zoom[cameraView] *= (1.f+((float)delta.y())*0.005);
        if(zoom[cameraView] < 1.0) zoom[cameraView] = 1.0;
        else if(zoom[cameraView] > 100) zoom[cameraView] = 100;
    }
    else if(event->modifiers() & Qt::ControlModifier)
    {
        // the pan and tilt of the camera can be adjusted
        pitch[cameraView] -= ((float)delta.y())*0.1;
        //if(pitch[cameraView] > 90) pitch[cameraView] = 90;
        //if(pitch[cameraView] < -90) pitch[cameraView] = -90;
        yaw[cameraView] += ((float)delta.x())*0.1;
        if(yaw[cameraView] < 0) yaw[cameraView] += 360;
        else if(yaw[cameraView] >= 360) yaw[cameraView] -= 360;
    }
    else
    {
        // the camera can be moved around in the terrain
        direction.setX(direction.x() + (cos(DEGTORAD(yaw[cameraView]))*(-delta.x()) + sin(DEGTORAD(yaw[cameraView]))*(delta.y())) * position.z()*0.003);
        direction.setY(direction.y() + (cos(DEGTORAD(yaw[cameraView]))*(delta.y()) + sin(DEGTORAD(yaw[cameraView]))*(delta.x())) * position.z()*0.003);
    }
}

void camera::cameraMouseWheel(QWheelEvent *event)
{
    float scrl = (float)event->delta();
    zoom[cameraView] += scrl/270;
    if(zoom[cameraView] < 1.0) zoom[cameraView] = 1.0;
    else if(zoom[cameraView] > 100) zoom[cameraView] = 100;
}

void camera::cameraToggleView()
{
    cameraView++;
    if(cameraView == CAMERAINDEXSIZE) cameraView = FreeView;
    //qDebug("View index:%d",cameraView);
}

void camera::cameraSetView(viewIndex x)
{
	if(x > CAMERAINDEXSIZE || x < 0) return;
	cameraView = x;
}

QString camera::cameraViewName()
{
	switch(cameraView){
		case FreeView:
		return QString("Camera Free View");
		case RoverCenter:
		return QString("Camera Rover Center");
		case RoverFollow:
		return QString("Camera Rover Follow");
		case RoverView:
		return QString("Camera Rover View");
		case RoverPanCam:
		return QString("Camera PanCam View");
	}
}

btVector3 camera::cameraPitchYawZoom()
{
    btVector3 camParm(yaw[cameraView],pitch[cameraView],zoom[cameraView]);
    return camParm;
}

void camera::cameraFreeView() {cameraView = FreeView;}
void camera::cameraRoverCenter() {cameraView = RoverCenter;}
void camera::cameraRoverFollow() {cameraView = RoverFollow;}
void camera::cameraRoverView() {cameraView = RoverView;}
void camera::cameraRoverPanCam() {cameraView = RoverPanCam;}
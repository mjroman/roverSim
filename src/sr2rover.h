#ifndef SR2SR2rover_H
#define SR2SR2rover_H

#include <QObject>
#include "laserscanner.h"
#include "robot.h"
#include "utility/glparticle.h"

class SR2rover : public QObject, public robot
{
private:
    int             m_gearTrain;
    int             m_encoderRes;
	int				m_rightEncoder;
	int				m_leftEncoder;

	// openGL lists
    GLuint          m_aWheel;
    GLuint          m_aSuspension;
    GLuint          m_aBody;
    GLuint          m_aSPanel;

    laserScanner*   bodyLaser;
    laserScanner*   panelLaser;
    laserScanner*   profileLaser;

	//GLParticle*		testParticle;

    void constructRover(const btVector3& positionOffset);
    void generateGLLists();

public:
    SR2rover(simGLView* glView);
    ~SR2rover();

    float       rightSpeed,leftSpeed;
	float		differentialAngle;
    float       panAngle,tiltAngle;

	void resetRobot();
    void updateRobot();
    int leftEncoder(){ return m_leftEncoder; }
    int rightEncoder(){ return m_rightEncoder; }
    void setRightSpeed(float spd);
    void setLeftSpeed(float spd);
    void incRightSpeed(float spd);
    void incLeftSpeed(float spd);
	void stopRobot();
	
    void paintLasers(bool state);
	void paintBodyLaser(bool state);
    void toggleSensors();
    void renderGLObject();
};

#endif // SR2rover_H

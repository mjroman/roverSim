#ifndef SR2SR2rover_H
#define SR2SR2rover_H

#include <QObject>
#include "laserscanner.h"
#include "robot.h"
#include "utility/glparticle.h"

// SR2 laser position labels
#define BODYLASER		0
#define PANELLASER		1
#define PROFILELASER	2

class SR2rover : public robot
{
	Q_OBJECT
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

	QList<laserScanner*>	m_laserList;
	float*					m_PLheights;
	float*					m_PRheights;

	//GLParticle*		mudParticle;

    void constructRover();
    void generateGLLists();

public:
    SR2rover(simGLView* glView);
    ~SR2rover();

    float       rightSpeed,leftSpeed;
	float		differentialAngle;
    float       panAngle,tiltAngle;

	void resetRobot();
    int leftEncoder(){ return m_leftEncoder; }
    int rightEncoder(){ return m_rightEncoder; }
    void setRightSpeed(float spd);
    void setLeftSpeed(float spd);
    void incRightSpeed(float spd);
    void incLeftSpeed(float spd);
	void stopRobot();
	
	
    void paintLasers(int state);
	void paintBodyLaser(bool state);
	laserScanner* getLaserScanner(int i) { return m_laserList[i]; }
	float* getPanelLaserHeights();
	float* getProfileLaserHeights();
    void toggleSensors();
    void renderGLObject();

public slots:
	void updateRobot();

signals:
	void updated();
};

#endif // SR2rover_H

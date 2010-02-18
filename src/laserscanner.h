#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QObject>
#include <OpenGL/gl.h>
#include "physicsWorld.h"
#include "utility/definitions.h"
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

class laserScanner : public QObject
{
private:
    int         m_dataSize;
    float*      m_rangeData;
    btVector3*  m_beamVector;
    bool        m_displayBeam;
	bool		m_displayBody;
    btTransform m_scanTrans;
    GLuint      m_aScanner;
	
    physicsWorld*   arena;

public:
    float       dTheta;
    float       offsetTheta;
    float       theta;
    float       maxRange;
    float       minRange;

    laserScanner(btTransform xForm,float th,float delta,float offset);
    ~laserScanner();

    void configure();
    int getData(float *data);
    void setTransform(const btTransform& xForm){ m_scanTrans = xForm; }
    void update(btTransform botTrans);
    void setBeamVisable(bool q){ m_displayBeam = q; }
	void setBodyVisable(bool q){ m_displayBody = q; }
    void drawLaser(btTransform botTrans);
};

#endif // LASERSCANNER_H

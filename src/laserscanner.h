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
    int	        m_displayBeam;
	bool		m_displayBody;
    btTransform m_scanTrans;
    GLuint      m_aScanner;
	
    physicsWorld*   arena;

	void configure();

public:
    float       dTheta;
    float       offsetTheta;
    float       theta;
    float       maxRange;
    float       minRange;

    laserScanner(btTransform xForm,float th,float delta,float offset);
    ~laserScanner();

	btVector3 getPosition() { return m_scanTrans.getOrigin(); }
	float* getData() {return m_rangeData; }
	int getDataSize() { return m_dataSize; }
    void setTransform(const btTransform& xForm){ m_scanTrans = xForm; }
    void update(btTransform botTrans);
    void setBeamVisable(int q){ m_displayBeam = q; }
	void setBodyVisable(bool q){ m_displayBody = q; }
    void drawLaser(btTransform botTrans);
};

#endif // LASERSCANNER_H

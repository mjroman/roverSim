#ifndef SIMCONTROL_H
#define	SIMCONTROL_H

#include <QTimer>
#include "physicsWorld.h"

class terrain;
class rover;
class skydome;
class simGLView;

class simControl : public QObject
{
    Q_OBJECT
	private:
		physicsWorld    *arena;
		terrain         *ground;
		skydome         *sky;
		rover			*sr2;
		QTimer          *simTimer;
	    double          delTime;
		simGLView		*glView;
	
	public:
		// obstacle members
		int				m_obstType;
		int				m_obstCount;
		float			m_dropHeight;
		btVector3		m_minObstSize;
		btVector3		m_maxObstSize;
		float			m_minObstYaw;
		float			m_maxObstYaw;
		float			m_obstDensity;
		
		simControl(simGLView* vw=0);
		~simControl();
		
		// simulation control functions
		void setGravity(btVector3 g);
		void startSimTimer(int msec);
		void stopSimTimer();
		
		void stepTimevals(float tStep,float fixedtStep,int subSteps);
		void pauseSim();
	
		// terrain control functions
		terrain* getGround() { return ground; }
		void rescaleGround(btVector3 scale);
		
		// rover control functions
		rover* getRover() { return sr2; }
		bool removeRover();
		
		
	public slots:
	// obstacle control functions
		void removeObstacles();
		void generateObstacles();
		void stepSim();
		
		void openNewGround(QString filename);
		void flattenGround();
		
		void newRover();
};
#endif  //SIMCONTROL_H
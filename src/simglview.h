#ifndef SIMGLVIEW_H
#define SIMGLVIEW_H

#include <QGLWidget>
#include "camera.h"
#include "physicsWorld.h"
#include "utility/definitions.h"
#include "utility/structures.h"

class simGLObject;

class simGLView : public QGLWidget
{
	Q_OBJECT

	public:
		simGLView(QWidget *parent = 0);
		~simGLView();

		QSize sizeHint() const;
		void printText(QString st);
		void overlayString(QString s);
		void toggleDrawing();
		void stopDrawing();
		void startDrawing();

		camera* getCamera() { return m_eye; }
		btVector3 getCameraPosition() { return m_eye->cameraPosition(); }
		void setViewAngle(float angle);
		float getViewAngle(){ return m_viewAngle; }
		GLuint getTexture(int n);

		void registerGLObject(simGLObject *obj);
		void unregisterGLObject(simGLObject *obj);

		void setWaypointList(QList<WayPoint> *list) { WPlist = list; }
		void toggleFog();
		btVector3 mouseRayTo(QPoint mousePoint);
		void setPickObject(pickValue* pk){ m_pickObject = pk; }

	signals:
		void refreshView();
		void pickingVector(btVector3,btVector3);
		void movingVector(btVector3,btVector3);
		void dropPicked();
		void spinPicked(float);
		void loftPicked(float);
		void outputText(QString);

	protected:
		struct renderString
		{
			QString	text;
			float fade;	
		};
		void loadTextures();
		void initializeGL();
		void resizeGL(int width, int height);
		void paintGL();
		void overlayGL();
		void overlayText();

		void mousePressEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent *event);

		void drawTest();
		void drawPlane(btScalar constant,const btVector3 normal);
		void drawWaypoints();
		void drawFrame();
		void drawPickingHalo();

	private:
		physicsWorld        *arena;
		camera              *m_eye;
		float               m_viewAngle;
		QTimer              *m_timer;
		bool				m_paused;
		QPoint              m_lastMousePoint;
		QList<simGLObject*> renderList;
		QList<GLuint>		m_textureList;
		QList<WayPoint>*	WPlist;
		pickValue*			m_pickObject;
		QList<renderString> m_overlayStringList;
};

#endif // SIMGLVIEW_H

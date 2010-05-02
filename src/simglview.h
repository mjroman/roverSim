#ifndef SIMGLVIEW_H
#define SIMGLVIEW_H

#include <QGLWidget>
#include "camera.h"
#include "physicsWorld.h"
#include "utility/definitions.h"


class simGLObject;

class simGLView : public QGLWidget
{
    Q_OBJECT

private:
    float               m_viewAngle;
    QTimer              *m_timer;
    QPoint              m_lastMousePoint;
    physicsWorld        *arena;
    camera              *m_eye;
    QList<simGLObject*> renderList;
    GLuint              m_texture[5];
	QList<WayPoint>		*WPlist;

protected:
    void loadTextures();
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();
    void overlayGL();

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    void drawTest();
    void drawPlane(btScalar constant,const btVector3 normal);
    void drawWorld();
	void drawWaypoints();
	void drawFrame(btTransform &tr);

public:
	int debugVal;
    simGLView(QWidget *parent = 0);
    ~simGLView();

    QSize sizeHint() const;
	void stopDrawing();
	void startDrawing();
    camera* getCamera() { return m_eye; }
    void setViewAngle(float angle);
    float getViewAngle(){ return m_viewAngle; }
    GLuint getTexture(int n);
    void registerGLObject(simGLObject *obj);
    void unregisterGLObject(simGLObject *obj);
	void setWaypointList(QList<WayPoint> *list) { WPlist = list; };

signals:
    void refreshView();
};

#endif // SIMGLVIEW_H

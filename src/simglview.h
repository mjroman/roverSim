#ifndef SIMGLVIEW_H
#define SIMGLVIEW_H

#include <QGLWidget>
#include "camera.h"
#include "physicsWorld.h"

class simGLObject;

class simGLView : public QGLWidget
{
    Q_OBJECT

private:
    float               m_viewAngle;
    QTimer              *timer;
    QPoint              lastMousePoint;
    physicsWorld        *arena;
    camera              *eye;
    QList<simGLObject*> renderList;
    GLuint              m_texture[2];

protected:
    void loadTextures();
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();
    void overlayView();

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    void drawTest();
    void drawPlane(btScalar constant,const btVector3 normal);
    void drawWorld();

public:
    simGLView(QWidget *parent = 0);
    ~simGLView();

    QSize sizeHint() const;
    camera* getCamera() { return eye; }
    void setViewAngle(float angle);
    float getViewAngle(){ return m_viewAngle; }
    GLuint getTexture(int n);
    void registerGLObject(simGLObject *obj);
    void unregisterGLObject(simGLObject *obj);

signals:
    void refreshView();
};

#endif // SIMGLVIEW_H

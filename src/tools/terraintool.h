#ifndef TERRAINTOOL_H
#define TERRAINTOOL_H

#include <QtGui>
#include "ui_terraintool.h"
#include <LinearMath/btVector3.h>

class physicsWorld;

class terrainTool : public QWidget, private Ui::terraintool {
    Q_OBJECT
private:
	physicsWorld    *arena;
    btVector3       m_scale;
    float           m_heightLimit;
    float           m_heightIncrement;
    float           m_toolDiameter;

public:
    terrainTool(QWidget *parent = 0);
    ~terrainTool();
    void setScale(btVector3 scale);
    btVector3 getScale(){ return m_scale; }
    float diameter(){ return m_toolDiameter; }
    float increment(){ return m_heightIncrement; }

public slots:
    void setGravity();
    void rescale();
    void setToolProps();
    void setToolDiameter();

signals:
    void scaleUpdate(btVector3);

};

#endif // TERRAINTOOL_H

#ifndef TERRAINTOOL_H
#define TERRAINTOOL_H

#include <QtGui>
#include "ui_terraintool.h"
#include <LinearMath/btVector3.h>

class terrainTool : public QWidget, private Ui::terraintool {
    Q_OBJECT
private:
    btVector3       m_scale;
    btVector3       m_gravity;
    float           m_heightLimit;
    float           m_heightIncrement;
    float           m_toolDiameter;

public:
    terrainTool(QWidget *parent = 0);
    ~terrainTool();
    void setScale(btVector3 scale);
    btVector3 scale(){ return m_scale; }
    btVector3 gravity(){ return m_gravity; }
    float diameter(){ return m_toolDiameter; }
    float increment(){ return m_heightIncrement; }

public slots:
	void on_buttonClose_clicked();
    void setGravity();
    void rescale();
    void setToolProps();
    void setToolDiameter();
    void raise();

signals:
    void gravityUpdate(btVector3);
    void scaleUpdate(btVector3);

};

#endif // TERRAINTOOL_H

#ifndef OBSTACLETOOL_H
#define OBSTACLETOOL_H

#include <QtGui>
#include "ui_obstacletool.h"


class obstacleTool : public QWidget, private Ui::obstacleTool {
    Q_OBJECT
public:
    obstacleTool(QWidget *parent = 0);
    ~obstacleTool();
	
    void setObstacleCount(int num){
        m_obstacleCount = num;
        SpinBoxObstCount->setValue(num);
    }
    int obstacleCount(){ return m_obstacleCount; }
    int obstacleType(){ return ComboShapeType->currentIndex(); }
    float minOLength(){ return m_minObstLength; }
    float minOWidth(){ return m_minObstWidth; }
    float minOHeight(){ return m_minObstHeight; }
    float maxOLength(){ return m_maxObstLength; }
    float maxOWidth(){ return m_maxObstWidth; }
    float maxOHeight(){ return m_maxObstHeight; }
    float minOYaw(){ return m_minObstYaw; }
    float maxOYaw(){ return m_maxObstYaw; }
    float density(){ return m_density; }
    float dropHeight(){ return m_dropHeight; }
	
public slots:
    void raise();
    void on_ButtonGenerate_clicked(bool checked=false);
    void updateLabels();
	
signals:
    void regenerateObstacles();
	
private:
    int             m_obstacleCount;
    float           m_minObstLength,m_minObstWidth,m_minObstHeight;
    float           m_maxObstLength,m_maxObstWidth,m_maxObstHeight;
    float           m_minObstYaw,m_maxObstYaw;
    float           m_density;
    float           m_dropHeight;
	
};

#endif // OBSTACLETOOL_H

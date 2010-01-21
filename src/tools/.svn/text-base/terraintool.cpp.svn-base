#include "terraintool.h"

terrainTool::terrainTool(QWidget *parent) :
    QWidget(parent),
    m_heightLimit(10),
    m_heightIncrement(0.1),
    m_toolDiameter(0.02)
{
    setupUi(this);
    QWidget::setWindowFlags(Qt::Window);

    m_scale.setValue(1,1,5);
    m_gravity.setValue(0,0,-9.8);

    lineEdit_ScaleWidth->setText(QString::number(m_scale.x()));
    lineEdit_ScaleLength->setText(QString::number(m_scale.y()));
    lineEdit_ScaleHeight->setText(QString::number(m_scale.z()));

    lineEdit_GX->setText(QString::number(m_gravity.x()));
    lineEdit_GY->setText(QString::number(m_gravity.y()));
    lineEdit_GZ->setText(QString::number(m_gravity.z()));

    connect(lineEdit_GX,SIGNAL(editingFinished()),this,SLOT(setGravity()));
    connect(lineEdit_GY,SIGNAL(editingFinished()),this,SLOT(setGravity()));
    connect(lineEdit_GZ,SIGNAL(editingFinished()),this,SLOT(setGravity()));

    lineEdit_MaxHeight->setText(QString::number(m_heightLimit));
    lineEdit_Increment->setText(QString::number(m_heightIncrement));
    this->setToolDiameter();

    connect(lineEdit_MaxHeight,SIGNAL(editingFinished()),this,SLOT(setToolProps()));
    connect(lineEdit_Increment,SIGNAL(editingFinished()),this,SLOT(setToolProps()));
    connect(lineEdit_toolDiameter,SIGNAL(editingFinished()),this,SLOT(setToolProps()));
    connect(slider_Diameter,SIGNAL(sliderReleased()),this,SLOT(setToolDiameter()));

    connect(button_rescale,SIGNAL(clicked()),this,SLOT(rescale()));
}

terrainTool::~terrainTool()
{
}

void terrainTool::setGravity()
{
    m_gravity.setX(lineEdit_GX->text().toFloat());
    m_gravity.setY(lineEdit_GY->text().toFloat());
    m_gravity.setZ(lineEdit_GZ->text().toFloat());
    emit gravityUpdate();
}

void terrainTool::setScale(btVector3 scale)
{
    m_scale = scale;
    lineEdit_ScaleWidth->setText(QString::number(scale.x()));
    lineEdit_ScaleLength->setText(QString::number(scale.y()));
    lineEdit_ScaleHeight->setText(QString::number(scale.z()));
    update();
}

void terrainTool::rescale()
{
    m_scale.setX(lineEdit_ScaleWidth->text().toFloat());
    m_scale.setY(lineEdit_ScaleLength->text().toFloat());
    m_scale.setZ(lineEdit_ScaleHeight->text().toFloat());
    emit scaleUpdate();
}

void terrainTool::setToolProps()
{
    m_heightLimit = lineEdit_MaxHeight->text().toFloat();
    m_heightIncrement = lineEdit_Increment->text().toFloat();
    if(m_heightIncrement < 0) m_heightIncrement = 0;
    m_toolDiameter = lineEdit_toolDiameter->text().toFloat();
    if(m_toolDiameter < 0) m_toolDiameter = 0;
    slider_Diameter->setValue((int)m_toolDiameter);
}

void terrainTool::setToolDiameter()
{
    m_toolDiameter = slider_Diameter->value();
    lineEdit_toolDiameter->setText(QString::number(m_toolDiameter));
}

void terrainTool::raise()
{
    this->activateWindow();
    this->show();
}

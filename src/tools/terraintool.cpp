#include "terraintool.h"
#include "physicsWorld.h"

terrainTool::terrainTool(QWidget *parent) :
QWidget(parent),
m_heightLimit(10),
m_heightIncrement(0.1),
m_toolDiameter(0.02)
{
    setupUi(this);
    QWidget::setWindowFlags(Qt::Sheet);

	arena = physicsWorld::instance(); // get the physics world object
	
    m_scale.setValue(1,1,1);
    lineEdit_ScaleWidth->setText(QString::number(m_scale.x()));
    lineEdit_ScaleLength->setText(QString::number(m_scale.y()));
    lineEdit_ScaleHeight->setText(QString::number(m_scale.z()));

	btVector3 gravity(0,0,-9.8);
    lineEdit_GX->setText(QString::number(gravity.x()));
    lineEdit_GY->setText(QString::number(gravity.y()));
    lineEdit_GZ->setText(QString::number(gravity.z()));

    lineEdit_MaxHeight->setText(QString::number(m_heightLimit));
    lineEdit_Increment->setText(QString::number(m_heightIncrement));
    this->setToolDiameter();

	// GUI line edits
    connect(lineEdit_MaxHeight,SIGNAL(editingFinished()),this,SLOT(setToolProps()));
    connect(lineEdit_Increment,SIGNAL(editingFinished()),this,SLOT(setToolProps()));
    connect(lineEdit_toolDiameter,SIGNAL(editingFinished()),this,SLOT(setToolProps()));
    connect(diameterSlider,SIGNAL(sliderReleased()),this,SLOT(setToolDiameter()));
	
	// GUI buttons
	connect(gravityButton,SIGNAL(clicked()),this,SLOT(setGravity()));
    connect(rescaleButton,SIGNAL(clicked()),this,SLOT(rescale()));
	connect(closeButton, SIGNAL(clicked()),this, SLOT(close()));
}

terrainTool::~terrainTool()
{
}

void terrainTool::setGravity()
{
	btVector3 gravity;
    gravity.setX(lineEdit_GX->text().toFloat());
    gravity.setY(lineEdit_GY->text().toFloat());
    gravity.setZ(lineEdit_GZ->text().toFloat());
	
	arena->setGravity(gravity);
}

void terrainTool::setScale(btVector3 scale)
{
    m_scale = scale;
    lineEdit_ScaleWidth->setText(QString::number(scale.x()));
    lineEdit_ScaleLength->setText(QString::number(scale.y()));
    lineEdit_ScaleHeight->setText(QString::number(scale.z()));
	
	emit scaleUpdate(m_scale);
}

void terrainTool::rescale()
{   
	m_scale.setX(lineEdit_ScaleWidth->text().toFloat());
    m_scale.setY(lineEdit_ScaleLength->text().toFloat());
    m_scale.setZ(lineEdit_ScaleHeight->text().toFloat());
	
    emit scaleUpdate(m_scale);
}

void terrainTool::setToolProps()
{
    m_heightLimit = lineEdit_MaxHeight->text().toFloat();
    m_heightIncrement = lineEdit_Increment->text().toFloat();
    if(m_heightIncrement < 0) m_heightIncrement = 0;
    m_toolDiameter = lineEdit_toolDiameter->text().toFloat();
    if(m_toolDiameter < 0) m_toolDiameter = 0;
    diameterSlider->setValue((int)m_toolDiameter);
}

void terrainTool::setToolDiameter()
{
    m_toolDiameter = diameterSlider->value();
    lineEdit_toolDiameter->setText(QString::number(m_toolDiameter));
}

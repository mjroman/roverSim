#include "terraintool.h"
#include "physicsWorld.h"

terrainTool::terrainTool(QWidget *parent) :
QWidget(parent),
m_size(100,100,1),
m_heightLimit(10),
m_heightIncrement(0.1),
m_toolDiameter(0.02)
{
    setupUi(this);
    QWidget::setWindowFlags(Qt::Sheet);

	arena = physicsWorld::instance(); // get the physics world object
	
    lineEdit_Width->setText(QString::number(m_size.x()));
    lineEdit_Length->setText(QString::number(m_size.y()));
    lineEdit_Height->setText(QString::number(m_size.z()));

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
    connect(resizeButton,SIGNAL(clicked()),this,SLOT(resize()));
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

void terrainTool::setSize(btVector3 size)
{
    m_size = size;
    lineEdit_Width->setText(QString::number(size.x()));
    lineEdit_Length->setText(QString::number(size.y()));
    lineEdit_Height->setText(QString::number(size.z()));
}

void terrainTool::resize()
{
	m_size.setX(lineEdit_Width->text().toFloat());
    m_size.setY(lineEdit_Length->text().toFloat());
    m_size.setZ(lineEdit_Height->text().toFloat());
	
    emit sizeUpdate(m_size);
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

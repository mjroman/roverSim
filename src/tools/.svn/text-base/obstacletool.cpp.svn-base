#include "obstacletool.h"

obstacleTool::obstacleTool(QWidget *parent) :
QWidget(parent),
m_obstacleCount(50),
m_minObstLength(0.1),
m_minObstWidth(0.1),
m_minObstHeight(0.25),
m_maxObstLength(1.0),
m_maxObstWidth(1.0),
m_maxObstHeight(0.5),
m_minObstYaw(0),
m_maxObstYaw(45),
m_density(5.),
m_dropHeight(5.0)
{
    setupUi(this);
    QWidget::setWindowFlags(Qt::Sheet);
	
    connect(ComboShapeType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLabels()));
	
    ComboShapeType->addItem("Box",0);
    ComboShapeType->addItem("Sphere",0);
    ComboShapeType->addItem("Cone",0);
    ComboShapeType->addItem("Cylinder",0);
	
    SpinBoxObstCount->setRange(0,5000);
    SpinBoxObstCount->setValue(m_obstacleCount);
	
    LineEditMinLength->setText(QString::number(m_minObstLength));
    LineEditMinWidth->setText(QString::number(m_minObstWidth));
    LineEditMinHeight->setText(QString::number(m_minObstHeight));
    LineEditMaxLength->setText(QString::number(m_maxObstLength));
    LineEditMaxWidth->setText(QString::number(m_maxObstWidth));
    LineEditMaxHeight->setText(QString::number(m_maxObstHeight));
    LineEditDropHeight->setText(QString::number(m_dropHeight));
    LineEditMinYaw->setText(QString::number(m_minObstYaw));
    LineEditMaxYaw->setText(QString::number(m_maxObstYaw));
    LineEditDensity->setText(QString::number(m_density));
    updateLabels();
}

obstacleTool::~obstacleTool()
{
}

void obstacleTool::raise()
{
    show();
    SpinBoxObstCount->setValue(m_obstacleCount);
}

void obstacleTool::updateLabels()
{
    switch(ComboShapeType->currentIndex()){
        case 0:
            LabelLength->setText("Length");
            LabelWidth->setText("Width");
            LineEditMinWidth->show();
            LineEditMaxWidth->show();
            LabelHeight->setText("Height");
            LineEditMinHeight->show();
            LineEditMaxHeight->show();
            LabelYaw->setText("Yaw");
            LineEditMinYaw->show();
            LineEditMaxYaw->show();
            break;
        case 1:
            LabelLength->setText("Radius");
            LabelWidth->clear();
            LineEditMinWidth->hide();
            LineEditMaxWidth->hide();
            LabelHeight->clear();
            LineEditMinHeight->hide();
            LineEditMaxHeight->hide();
            LabelYaw->clear();
            LineEditMinYaw->hide();
            LineEditMaxYaw->hide();
            break;
        case 2:
            LabelLength->setText("Height");
            LabelWidth->setText("Radius");
            LineEditMinWidth->show();
            LineEditMaxWidth->show();
            LabelHeight->clear();
            LineEditMinHeight->hide();
            LineEditMaxHeight->hide();
            LabelYaw->clear();
            LineEditMinYaw->hide();
            LineEditMaxYaw->hide();
            break;
        case 3:
            LabelLength->setText("Height");
            LabelWidth->setText("Radius");
            LineEditMinWidth->show();
            LineEditMaxWidth->show();
            LabelHeight->clear();
            LineEditMinHeight->hide();
            LineEditMaxHeight->hide();
            LabelYaw->setText("Yaw");
            LineEditMinYaw->show();
            LineEditMaxYaw->show();
            break;
    }
}

void obstacleTool::on_ButtonGenerate_clicked(bool)
{
    m_obstacleCount = SpinBoxObstCount->value();
    // get minimum sizes from tool
    m_minObstLength = LineEditMinLength->text().toFloat();
    m_minObstWidth = LineEditMinWidth->text().toFloat();
    m_minObstHeight = LineEditMinHeight->text().toFloat();
    m_minObstYaw = LineEditMinYaw->text().toFloat();
    // get drop height
    m_dropHeight = LineEditDropHeight->text().toFloat();
    // get density
    m_density = LineEditDensity->text().toFloat();
	
    // get maximum sizes from tool and make sure they are
    // greater than the minimum sizes
    if(m_minObstLength < LineEditMaxLength->text().toFloat()) m_maxObstLength = LineEditMaxLength->text().toFloat();
    else{
        m_maxObstLength = m_minObstLength;
        LineEditMaxLength->setText(QString::number(m_maxObstLength));
    }
    if(m_minObstWidth < LineEditMaxWidth->text().toFloat()) m_maxObstWidth = LineEditMaxWidth->text().toFloat();
    else{
        m_maxObstWidth = m_minObstWidth;
        LineEditMaxWidth->setText(QString::number(m_maxObstWidth));
    }
    if(m_minObstHeight < LineEditMaxHeight->text().toFloat()) m_maxObstHeight = LineEditMaxHeight->text().toFloat();
    else{
        m_maxObstHeight = m_minObstHeight;
        LineEditMaxHeight->setText(QString::number(m_maxObstHeight));
    }
	
    if(m_minObstYaw < LineEditMaxYaw->text().toFloat()) m_maxObstYaw = LineEditMaxYaw->text().toFloat();
    else{
        m_maxObstYaw = m_minObstYaw;
        LineEditMaxYaw->setText(QString::number(m_maxObstYaw));
    }
	
    emit regenerateObstacles();
    this->close();
}

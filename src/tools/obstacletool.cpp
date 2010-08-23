#include "obstacletool.h"

#define OBSTSHAPEGROUP	"ObstacleShape" // settings group key value

obstacleTool::obstacleTool(QWidget *parent) :
QWidget(parent),
// location of the settings file ~/.config/OUengineering/Rover_Sim.ini
obstSettings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim")
{
    setupUi(this);
    QWidget::setWindowFlags(Qt::Sheet);
	
	this->initSettingsNames();
	
	if(!QFile::exists(obstSettings.fileName())) defaultSettings();						// no settings file exitst
	else if(!obstSettings.childGroups().contains(OBSTSHAPEGROUP)) defaultSettings();		// no obstacle settings in the file exits
	
	this->getSettings();
	
    ComboShapeType->addItem("Box");
    ComboShapeType->addItem("Sphere");
    ComboShapeType->addItem("Cone");
    ComboShapeType->addItem("Cylinder");
	
    SpinBoxObstCount->setRange(0,5000);
	SpinBoxObstCount->setSingleStep(25);
    SpinBoxObstCount->setValue(obstCount.stuff.toInt());

	updateLabels(obstShape.stuff.toInt());

 	connect(ComboShapeType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLabels(int)));
	connect(ButtonCancel, SIGNAL(clicked()), this, SLOT(hide()));
	hide();
}

obstacleTool::~obstacleTool()
{
}

void obstacleTool::show()
{
	updateLabels(obstShape.stuff.toInt());
	QWidget::show();
}

void obstacleTool::updateLabels(int index)
{
    switch(index){
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

	LineEditMinLength->setText(QString::number(obstMinLength.stuff.toFloat()));
    LineEditMinWidth->setText(QString::number(obstMinWidth.stuff.toFloat()));
    LineEditMinHeight->setText(QString::number(obstMinHeight.stuff.toFloat()));
    LineEditMaxLength->setText(QString::number(obstMaxLength.stuff.toFloat()));
    LineEditMaxWidth->setText(QString::number(obstMaxWidth.stuff.toFloat()));
    LineEditMaxHeight->setText(QString::number(obstMaxHeight.stuff.toFloat()));
    LineEditMinYaw->setText(QString::number(obstMinYaw.stuff.toFloat()));
    LineEditMaxYaw->setText(QString::number(obstMaxYaw.stuff.toFloat()));
	LineEditDropHeight->setText(QString::number(obstDropHeight.stuff.toFloat()));
    LineEditDensity->setText(QString::number(obstDensity.stuff.toFloat()));
}

void obstacleTool::on_ButtonGenerate_clicked(bool)
{
	obstShape.stuff.setValue(ComboShapeType->currentIndex());				// get shape type
    obstCount.stuff.setValue(SpinBoxObstCount->value());					// get obstacle count
    obstDropHeight.stuff.setValue(LineEditDropHeight->text().toFloat());	// get drop height
    obstDensity.stuff.setValue(LineEditDensity->text().toFloat());			// get density
    
    obstMinLength.stuff.setValue(LineEditMinLength->text().toFloat());		// get minimum sizes
    obstMinWidth.stuff.setValue(LineEditMinWidth->text().toFloat());
    obstMinHeight.stuff.setValue(LineEditMinHeight->text().toFloat());
    obstMinYaw.stuff.setValue(LineEditMinYaw->text().toFloat());
	
	obstMaxLength.stuff.setValue(LineEditMaxLength->text().toFloat());		// get maximum sizes from tool and make sure they are
    if(obstMinLength.stuff.toFloat() > obstMaxLength.stuff.toFloat()){		// greater than the minimum sizes
        obstMaxLength.stuff = obstMinLength.stuff;
        LineEditMaxLength->setText(QString::number(obstMaxLength.stuff.toFloat()));
    }

	obstMaxWidth.stuff.setValue(LineEditMaxWidth->text().toFloat());
    if(obstMinWidth.stuff.toFloat() > obstMaxWidth.stuff.toFloat()){
        obstMaxWidth.stuff = obstMinWidth.stuff;
        LineEditMaxWidth->setText(QString::number(obstMaxWidth.stuff.toFloat()));
    }

	obstMaxHeight.stuff.setValue(LineEditMaxHeight->text().toFloat());
    if(obstMinHeight.stuff.toFloat() > obstMaxHeight.stuff.toFloat()){
        obstMaxHeight.stuff = obstMinHeight.stuff;
        LineEditMaxHeight->setText(QString::number(obstMaxHeight.stuff.toFloat()));
    }
	
	obstMaxYaw.stuff.setValue(LineEditMaxYaw->text().toFloat());
    if(obstMinYaw.stuff.toFloat() > obstMaxYaw.stuff.toFloat()){
        obstMaxYaw.stuff = obstMinYaw.stuff;
        LineEditMaxYaw->setText(QString::number(obstMaxYaw.stuff.toFloat()));
    }

	this->setSettings();
    emit regenerateObstacles();
    hide();
}

/////////////////////////////////////////
// settings function calls
/////////////
void obstacleTool::getSettings()
{
	 // open a new group in the settings file
	obstSettings.beginGroup(OBSTSHAPEGROUP);
	// set the parameter value from the settings file
	obstShape.stuff.setValue(obstSettings.value(obstShape.name,0).toInt());
	obstCount.stuff.setValue(obstSettings.value(obstCount.name,50).toInt());
	obstMinLength.stuff.setValue(obstSettings.value(obstMinLength.name,0.1).toFloat());
	obstMinWidth.stuff.setValue(obstSettings.value(obstMinWidth.name,0.1).toFloat());
	obstMinHeight.stuff.setValue(obstSettings.value(obstMinHeight.name,0.25).toFloat());
	obstMaxLength.stuff.setValue(obstSettings.value(obstMaxLength.name,1.0).toFloat());
	obstMaxWidth.stuff.setValue(obstSettings.value(obstMaxWidth.name,1.0).toFloat());
	obstMaxHeight.stuff.setValue(obstSettings.value(obstMaxHeight.name,0.5).toFloat());
	obstMinYaw.stuff.setValue(obstSettings.value(obstMinYaw.name,0).toFloat());
	obstMaxYaw.stuff.setValue(obstSettings.value(obstMaxYaw.name,45.0).toFloat());
	obstDensity.stuff.setValue(obstSettings.value(obstDensity.name,5.0).toFloat());
	obstDropHeight.stuff.setValue(obstSettings.value(obstDropHeight.name,5.0).toFloat());
	obstSettings.endGroup();
}

void obstacleTool::setSettings()
{
	obstSettings.beginGroup(OBSTSHAPEGROUP);
	obstSettings.setValue(obstShape.name,obstShape.stuff);
	obstSettings.setValue(obstCount.name,obstCount.stuff);
	obstSettings.setValue(obstMinLength.name,obstMinLength.stuff);
	obstSettings.setValue(obstMinWidth.name,obstMinWidth.stuff);
	obstSettings.setValue(obstMinHeight.name,obstMinHeight.stuff);
	obstSettings.setValue(obstMaxLength.name,obstMaxLength.stuff);
	obstSettings.setValue(obstMaxWidth.name,obstMaxWidth.stuff);
	obstSettings.setValue(obstMaxHeight.name,obstMaxHeight.stuff);
	obstSettings.setValue(obstMinYaw.name,obstMinYaw.stuff);
	obstSettings.setValue(obstMaxYaw.name,obstMaxYaw.stuff);
	obstSettings.setValue(obstDensity.name,obstDensity.stuff);
	obstSettings.setValue(obstDropHeight.name,obstDropHeight.stuff);
	obstSettings.endGroup();
	obstSettings.sync();
}

void obstacleTool::defaultSettings()
{
	obstSettings.beginGroup(OBSTSHAPEGROUP); // open a new group in the settings file
	obstSettings.setValue(obstShape.name,0);
	obstSettings.setValue(obstCount.name,50);
	obstSettings.setValue(obstMinLength.name,0.1);
	obstSettings.setValue(obstMinWidth.name,0.1);
	obstSettings.setValue(obstMinHeight.name,0.25);
	obstSettings.setValue(obstMaxLength.name,1.0);
	obstSettings.setValue(obstMaxWidth.name,1.0);
	obstSettings.setValue(obstMaxHeight.name,0.5);
	obstSettings.setValue(obstMinYaw.name,0);
	obstSettings.setValue(obstMaxYaw.name,45);
	obstSettings.setValue(obstDensity.name,5.0);
	obstSettings.setValue(obstDropHeight.name,5.0);
	obstSettings.endGroup();
	obstSettings.sync();
}

void obstacleTool::initSettingsNames()
{
	obstShape.name = "Shape";
	obstCount.name = "Count";
	obstMinLength.name = "Min Length";
	obstMinWidth.name = "Min Width";
	obstMinHeight.name = "Min Height";
	obstMaxLength.name = "Max Length";
	obstMaxWidth.name = "Max Width";
	obstMaxHeight.name = "Max Height";
	obstMinYaw.name = "Min Yaw";
	obstMaxYaw.name = "Max Yaw";
	obstDensity.name = "Density";
	obstDropHeight.name = "Drop Height";
}
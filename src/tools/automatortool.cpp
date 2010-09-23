#include "automatortool.h"

automatorTool::automatorTool(QWidget *parent)
: QDialog(parent),
configFile(QDir::currentPath() + "/mission/config",QSettings::IniFormat)
{
	setupUi(this);

	setWindowTitle("Automation Configure Setup");
	connect(buttonCancel, SIGNAL(clicked()), this, SLOT(reject()));
	connect(buttonSave, SIGNAL(clicked()), this, SLOT(saveData()));
	connect(buttonRun, SIGNAL(clicked()), this, SLOT(runData()));
	connect(buttonTerrainFile, SIGNAL(clicked()), this, SLOT(setTerrainFile()));
	
	lineEditWorldSizeX->setText(QString::number(configFile.value("World_Size_X",50).toFloat()));
	lineEditWorldSizeY->setText(QString::number(configFile.value("World_Size_Y",50).toFloat()));
	lineEditWorldSizeZ->setText(QString::number(configFile.value("World_Size_Z",5).toFloat()));
	
	QString tfile = configFile.value("Terrain","NULL").toString();
	if(tfile != "NULL") {
		terrainFileInfo.setFile(tfile);
		tfile = terrainFileInfo.baseName();
	}
	buttonTerrainFile->setText(tfile);
	
	spinBoxObstacleCount->setValue(configFile.value("Obstacle_Count",100).toInt());
	
	lineEditObstMinX->setText(QString::number(configFile.value("Obstacle_Min_X",0.1).toFloat()));
	lineEditObstMinY->setText(QString::number(configFile.value("Obstacle_Min_Y",0.1).toFloat()));
	lineEditObstMinZ->setText(QString::number(configFile.value("Obstacle_Min_Z",0.15).toFloat()));
	
	//QVector3D maxSize = configFile.value("Obstacle Max",QVariant(QVector3D(1.0,1.0,0.25))).value<QVector3D>();
	lineEditObstMaxX->setText(QString::number(configFile.value("Obstacle_Max_X",1.0).toFloat()));
	lineEditObstMaxY->setText(QString::number(configFile.value("Obstacle_Max_Y",1.0).toFloat()));
	lineEditObstMaxZ->setText(QString::number(configFile.value("Obstacle_Max_Z",0.25).toFloat()));
	
	lineEditObstYawMin->setText(QString::number(configFile.value("Obstacle_Yaw_Min",0).toFloat()));
	lineEditObstYawMax->setText(QString::number(configFile.value("Obstacle_Yaw_Max",180).toFloat()));
	
	lineEditPathMin->setText(QString::number(configFile.value("Path_Size_Min",25).toFloat()));
	lineEditPathMax->setText(QString::number(configFile.value("Path_Size_Max",100).toFloat()));
	lineEditPathEff->setText(QString::number(configFile.value("Path_Eff_Limit",0.5).toFloat()));
	lineEditPathSpin->setText(QString::number(configFile.value("Path_Spin_Progress",6).toFloat()));
	
	lineEditSensorRanges->setText(configFile.value("Sensor_Ranges","0,1,2,3,4").toString());
	lineEditSensorStep->setText(QString::number(configFile.value("Sensor_Step",0.15).toFloat()));
	lineEditSensorSpace->setText(QString::number(configFile.value("Sensor_Cspace",0.65).toFloat()));
	
	lineEditIterations->setText(QString::number(configFile.value("Iterations",10).toInt()));
	lineEditSeed->setText(QString::number(configFile.value("Seed",333).toLongLong()));
	lineEditTrialName->setText(configFile.value("Trial_Name","trial").toString());
	
	checkBoxPathDrawing->setChecked(configFile.value("Path_Drawing",0).toInt());
}

void automatorTool::acceptData()
{
	configFile.setValue("World_Size_X",lineEditWorldSizeX->text());
	configFile.setValue("World_Size_Y",lineEditWorldSizeY->text());
	configFile.setValue("World_Size_Z",lineEditWorldSizeZ->text());
	
	if(terrainFileInfo.exists())
		configFile.setValue("Terrain",terrainFileInfo.absoluteFilePath());
	else
		configFile.setValue("Terrain","NULL");
	
	configFile.setValue("Obstacle_Count",spinBoxObstacleCount->value());
	
	configFile.setValue("Obstacle_Min_X",lineEditObstMinX->text());
	configFile.setValue("Obstacle_Min_Y",lineEditObstMinY->text());
	configFile.setValue("Obstacle_Min_Z",lineEditObstMinZ->text());
	
	configFile.setValue("Obstacle_Max_X",lineEditObstMaxX->text());
	configFile.setValue("Obstacle_Max_Y",lineEditObstMaxY->text());
	configFile.setValue("Obstacle_Max_Z",lineEditObstMaxZ->text());
	
	configFile.setValue("Obstacle_Yaw_Min",lineEditObstYawMin->text());
	configFile.setValue("Obstacle_Yaw_Max",lineEditObstYawMax->text());
	
	configFile.setValue("Path_Size_Min",lineEditPathMin->text());
	configFile.setValue("Path_Size_Max",lineEditPathMax->text());
	configFile.setValue("Path_Eff_Limit",lineEditPathEff->text());
	configFile.setValue("Path_Spin_Progress",lineEditPathSpin->text());
	
	configFile.setValue("Sensor_Ranges",lineEditSensorRanges->text());
	configFile.setValue("Sensor_Step",lineEditSensorStep->text());
	configFile.setValue("Sensor_Cspace",lineEditSensorSpace->text());
	
	configFile.setValue("Iterations",lineEditIterations->text().toInt());
	configFile.setValue("Seed",lineEditSeed->text().toLongLong());
	configFile.setValue("Trial_Name",lineEditTrialName->text());
	
	configFile.setValue("Path_Drawing",checkBoxPathDrawing->isChecked());
	configFile.sync();
}

void automatorTool::setTerrainFile()
{
	QString filename = QFileDialog::getOpenFileName(this,"Open Terrain", "/Users","Image File (*.png)");
	if(filename == NULL) return; // if cancel is pressed dont do anything
	terrainFileInfo.setFile(filename);
	buttonTerrainFile->setText(terrainFileInfo.baseName());
}

void automatorTool::saveData()
{
	this->acceptData();
	this->done(1);
}

void automatorTool::runData()
{
	this->acceptData();
	this->done(2);	
}
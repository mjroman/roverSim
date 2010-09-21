#include "waypointtool.h"
#include "../terrain.h"

waypointTool::waypointTool(terrain *t,QWidget *parent)
:
QWidget(parent),
ground(t)
{
	setupUi(this);
	QWidget::setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Waypoint Editor");
	
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");	// set tool window position
	if(!QFile::exists(settings.fileName()) || !settings.contains("WaypointWindowGeom")){
		move(20,350);
		settings.setValue("WaypointWindowGeom", saveGeometry());
	}
	else
		this->restoreGeometry(settings.value("WaypointWindowGeom").toByteArray());
	
	waypointScienceKeyMapping();
	waypointStateKeyMapping();
	
	setComboScienceList();
	setComboStateList();
	
	connect(ground, SIGNAL(newTerrain()), this, SLOT(setHeights()));
	connect(comboWpSelect, SIGNAL(currentIndexChanged(int)), this, SLOT(refreshGUI(int)));
	connect(lineEditUuid, SIGNAL(editingFinished()), this, SLOT(uuidEdited()));
	connect(lineEditPositionX, SIGNAL(editingFinished()), this, SLOT(positionEdited()));
	connect(lineEditPositionY, SIGNAL(editingFinished()), this, SLOT(positionEdited()));
	connect(buttonReset, SIGNAL(clicked()), this, SLOT(resetStates()));
	enableGUI(false);
}

waypointTool::~waypointTool()
{
	wayList.clear();
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");
	settings.setValue("WaypointWindowGeom", saveGeometry());
}

void waypointTool::show()
{
	QPropertyAnimation *anim = new QPropertyAnimation(this,"pos");
	anim->setDuration(1000);
	anim->setStartValue(QPoint(pos().x(),pos().y()-25));
	anim->setEndValue(pos());
	anim->setEasingCurve(QEasingCurve::OutElastic);
	anim->start();
	QWidget::show();
}

// reset all the waypoint states to NEW
void waypointTool::resetStates()
{
	for(int i = 0; i < wayList.size(); ++i)
		wayList[i].state = WPstateNew;
	refreshGUI(comboWpSelect->currentIndex());
}

void waypointTool::on_buttonSetCurrent_clicked()
{
	if(wayList.isEmpty()) return;
	emit currentWaypoint(comboWpSelect->currentIndex());
}

/////////////////////////////////////////
// Combo box mapping methods
/////////////
void waypointTool::waypointScienceKeyMapping()
{
	WSmap[WPscienceNone] = "No Science";
	WSmap[WPsciencePanorama] = "Panorama";
	WSmap[WPscienceSpectra] = "Spectra";
	WSmap[WPsciencePanoramaAndSpectra] = "Pan and Spectra";
}
void waypointTool::waypointStateKeyMapping()
{
	WPmap[WPstateNew] = "To be visited";
	WPmap[WPstateOld] = "Visited";
	WPmap[WPstateCurrent] = "Driving to now";
	WPmap[WPstateSkipped] = "Skipped";
}

void waypointTool::setComboScienceList()
{
	QMapIterator<int, QString> i(WSmap);
	
	while(i.hasNext()){
		i.next();
		comboScience->addItem(i.value(), QVariant(i.key()));
	}
}
void waypointTool::setComboStateList()
{
	QMapIterator<int, QString> i(WPmap);
	
	while(i.hasNext()){
		i.next();
		comboState->addItem(i.value(), QVariant(i.key()));
	}
}

void waypointTool::on_comboScience_activated(int s)
{
	int index = comboWpSelect->currentIndex();
	wayList[index].science = (WPscience)s;
}
void waypointTool::on_comboState_activated(int s)
{
	int index = comboWpSelect->currentIndex();
	wayList[index].state = (WPstate)s;
}

void waypointTool::enableGUI(bool state)
{
	buttonDelete->setEnabled(state);
	lineEditUuid->setEnabled(state);
	comboScience->setEnabled(state);
	comboState->setEnabled(state);
	lineEditPositionX->setEnabled(state);
	lineEditPositionY->setEnabled(state);
}
// updates the GUI when the waypoint combo box is activated
void waypointTool::refreshGUI(int index)
{
	if(index < 0) return;
	
	WayPoint wp = wayList[index];
	lineEditUuid->setText(QString::number(wp.uuid));
	lineEditPositionX->setText(QString::number(wp.position.x(),'f',3));
	lineEditPositionY->setText(QString::number(wp.position.y(),'f',3));
	comboScience->setCurrentIndex(wp.science);
	comboState->setCurrentIndex(wp.state);
}

void waypointTool::addWaypoint(WayPoint wp)
{
	if(wayList.isEmpty())
		enableGUI(true);
		
	int index = comboWpSelect->currentIndex() + 1;							// add waypoints after the selected item
	wayList.insert(index, wp);												// add the waypoint to the list
	comboWpSelect->insertItem(index,QString::number(wayList[index].uuid));	// add the item to the combo box
	comboWpSelect->setCurrentIndex(index);									// signals to refresh the GUI as well
}

void waypointTool::on_buttonAdd_clicked()
{
	WayPoint wp;	
	wp.uuid = lineEditUuid->text().toInt() + 1;	
	wp.position.setX(lineEditPositionX->text().toFloat()+0.5);
	wp.position.setY(lineEditPositionY->text().toFloat()+0.5);
	wp.position.setZ(ground->terrainHeightAt(wp.position));
	wp.state = WPstateNew;
	wp.science = WPscienceNone;
	
	addWaypoint(wp);
}

void waypointTool::on_buttonDelete_clicked()
{
	int index = comboWpSelect->currentIndex();
	wayList.removeAt(index);						// remove the waypoint from the list
	comboWpSelect->removeItem(index);				// remove the item fromt the combo box
	
	if(wayList.isEmpty()){
		enableGUI(false);
		return;
	}
	
	index --;
	if(index < 0) index = 0;
	comboWpSelect->setCurrentIndex(index);
}

void waypointTool::removeWaypoints()
{
	wayList.clear();
	comboWpSelect->clear();
	enableGUI(false);
}

void waypointTool::uuidEdited()
{
	int index = comboWpSelect->currentIndex();
	wayList[index].uuid = lineEditUuid->text().toInt();
	comboWpSelect->setItemText(index, QString::number(wayList[index].uuid));
}
void waypointTool::positionEdited()
{
	int index = comboWpSelect->currentIndex();
	btVector3 pos;
	
	pos.setX(lineEditPositionX->text().toFloat());
	pos.setY(lineEditPositionY->text().toFloat());
	pos.setZ(ground->terrainHeightAt(pos));
	wayList[index].position = pos;
}
void waypointTool::moveWaypoint(int index, btVector3 pos)
{
	wayList[index].position = pos;
}

void waypointTool::setHeights()
{
	for(int i=0; i < wayList.size(); i++){
		btVector3 pos = wayList[i].position;
		pos.setZ(ground->terrainHeightAt(pos));	// set the z position of the waypoint for visability
		wayList[i].position = pos;
	}
}

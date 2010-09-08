#include "waypointtool.h"

waypointTool::waypointTool(QWidget *parent)
:
QWidget(parent)
{
	setupUi(this);
	QWidget::setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Waypoint Editor");
	
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");
	if(!QFile::exists(settings.fileName()) || !settings.contains("WaypointWindowGeom")){
		move(20,350);
		settings.setValue("WaypointWindowGeom", saveGeometry());
	}
	else
		this->restoreGeometry(settings.value("WaypointWindowGeom").toByteArray());
	
	cIndex = 0;
	waypointScienceKeyMapping();
	setComboScienceList();
	
	connect(lineEditPositionX, SIGNAL(editingFinished()), this, SLOT(edited()));
	connect(lineEditPositionY, SIGNAL(editingFinished()), this, SLOT(edited()));
	connect(buttonReset, SIGNAL(clicked()), this, SLOT(resetStates()));
}

waypointTool::~waypointTool()
{
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

void waypointTool::raiseWaypointEditor(QList<WayPoint>* list)
{
	WPlist = list;
	updateComboWaypointList();
	show();
}

void waypointTool::waypointScienceKeyMapping()
{
	WSmap[WPscienceNone] = "No Science";
	WSmap[WPsciencePanorama] = "Panorama";
	WSmap[WPscienceSpectra] = "Spectra";
	WSmap[WPsciencePanoramaAndSpectra] = "Pan and Spectra";
}
void waypointTool::setComboScienceList()
{
	QMapIterator<int, QString> i(WSmap);
	
	while(i.hasNext()){
		i.next();
		comboScience->addItem(i.value(), QVariant(i.key()));
	}
}
void waypointTool::waypointStateKeyMapping()
{
	WPmap[WPstateNew] = "To be visited";
	WPmap[WPstateOld] = "Visited";
	WPmap[WPstateCurrent] = "Driving to now";
	WPmap[WPstateSkipped] = "Skipped";
}
void waypointTool::updateComboWaypointList() // clears then fills the combobox
{
	comboWpSelect->clear();
	if(WPlist->isEmpty()){
		cIndex = 0;
		buttonDelete->setEnabled(false);
		lineEditUuid->setText(QString::number(1));
	}
	else{
		int first = WPlist->at(0).uuid;
		for(int i = 0; i < WPlist->size(); ++i)
		{
			WayPoint wp = WPlist->at(i);
			wp.uuid = i+first;
			WPlist->replace(i,wp);
			comboWpSelect->addItem(QString::number(WPlist->at(i).uuid),QVariant(i));
		}
		on_comboWpSelect_activated(cIndex);
		comboWpSelect->setCurrentIndex(cIndex);		
	}
}

void waypointTool::on_comboWpSelect_activated(int index)
{
	cIndex = index;
	WayPoint wp = WPlist->at(index);
	lineEditUuid->setText(QString::number(wp.uuid));
	lineEditPositionX->setText(QString::number(wp.position.x(),'f',3));
	lineEditPositionY->setText(QString::number(wp.position.y(),'f',3));
	comboScience->setCurrentIndex(wp.science);
}

void waypointTool::on_buttonAdd_clicked()
{
	if(WPlist->isEmpty()) buttonDelete->setEnabled(true);
		
	cIndex = comboWpSelect->currentIndex();
	WayPoint wp;
	
	wp.uuid = lineEditUuid->text().toInt();	
	wp.position.setX(lineEditPositionX->text().toFloat());
	wp.position.setY(lineEditPositionY->text().toFloat());
	wp.position.setZ(0);
	wp.state = WPstateNew;
	wp.science = WPscienceNone;
	
	cIndex++;
	emit addedWP(wp,cIndex);
	updateComboWaypointList();
}

void waypointTool::on_buttonDelete_clicked()
{
	cIndex--;
	if(cIndex < 0) cIndex = 0;
	WPlist->removeAt(comboWpSelect->currentIndex());
	updateComboWaypointList();
}

void waypointTool::on_comboScience_activated(int index)
{
	this->edited();
}

void waypointTool::edited()
{
	if(WPlist->isEmpty()) return;
	WayPoint wp = WPlist->at(cIndex);
	wp.science = (WPscience)comboScience->currentIndex();
	wp.position.setX(lineEditPositionX->text().toFloat());
	wp.position.setY(lineEditPositionY->text().toFloat());
	WPlist->replace(cIndex,wp);
	emit editedWP(cIndex);
}

void waypointTool::resetStates()
{
	emit resetWP();
}

// void navigationTool::displayCurrentWaypoint()
// {
// 	currentWaypointDisplay = true;
// 	updateGUI();
// }
// 
// void navigationTool::on_comboWpSelect_activated(int index)
// {
// 	updateGUI();
// 	if(buttonRunning->isChecked()) QTimer::singleShot(5000,this, SLOT(displayCurrentWaypoint()));
// }
// 
// void navigationTool::updateGUI()
// {
// 	labelState->setText(RSmap[state]);
// 	label_error->setText(REmap[error]);
// 	label_wpCount->setText(QString::number(wpIndex));
// 	
// 	if(blockedDirection == 0.0) label_obstDirection->setText("No Obstacles");
// 	else label_obstDirection->setText(QString::number(blockedDirection,'f',4));
// 	
// 	int i;
// 	if(currentWaypointDisplay) {
// 		i = wpIndex;
// 		combo_wpSelect->setCurrentIndex(i);
// 	}
// 	else i = combo_wpSelect->currentIndex();
// 	
// 	WayPoint w = sr2->waypointList[i];
// 	label_wpState->setText(WPmap[w.state]);
// 	label_wpScience->setText(WSmap[w.science]);
// 	label_wpPosition->setText(QString("(%1, %2, %3)")
// 								.arg(w.position.x,0,'f',2)
// 								.arg(w.position.y,0,'f',2)
// 								.arg(w.position.z,0,'f',2));
// 	label_wpDistance->setText(QString::number(distanceToWaypoint(i),'f',3));
// }
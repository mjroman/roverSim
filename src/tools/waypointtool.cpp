#include "waypointtool.h"

waypointTool::waypointTool(QWidget *parent)
:
QWidget(parent)
{
	setupUi(this);
	move(20,100);
	QWidget::setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Waypoint Editor");
	
	cIndex = 0;
	waypointScienceKeyMapping();
	setComboScienceList();
	
	connect(lineEditPositionX, SIGNAL(editingFinished()), this, SLOT(edited()));
	connect(lineEditPositionY, SIGNAL(editingFinished()), this, SLOT(edited()));
}

waypointTool::~waypointTool()
{
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
	lineEditPositionX->setText(QString::number(wp.position.x,'f',3));
	lineEditPositionY->setText(QString::number(wp.position.y,'f',3));
	comboScience->setCurrentIndex(wp.science);
}

void waypointTool::on_buttonAdd_clicked()
{	
	if(WPlist->isEmpty()) buttonDelete->setEnabled(true);
		
	cIndex = comboWpSelect->currentIndex();
	WayPoint wp;
	
	wp.uuid = lineEditUuid->text().toInt();	
	wp.position.x = lineEditPositionX->text().toFloat();
	wp.position.y = lineEditPositionY->text().toFloat();
	wp.position.z = 0;
	wp.state = WPstateNew;
	wp.science = WPscienceNone;
	
	cIndex++;
	WPlist->insert(cIndex,wp);
	updateComboWaypointList();
	emit addedWP();
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
	wp.position.x = lineEditPositionX->text().toFloat();
	wp.position.y = lineEditPositionY->text().toFloat();
	WPlist->replace(cIndex,wp);	
}
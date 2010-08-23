#include "simtool.h"
#include "physicsWorld.h"

simtool::simtool(QWidget *parent) 
:
QDialog(parent)
{
	physicsWorld *arena = physicsWorld::instance(); 					// get the physics world object
    setWindowTitle("Simulation Timing Parameters");
	//QWidget::setWindowFlags(Qt::Sheet);

    updateButton = new QPushButton("&Update");
	updateButton->setMinimumWidth(80);
	updateButton->setMinimumHeight(20);
	updateButton->setDefault(true);
    updateButton->setStyleSheet("QPushButton:default{background: red; border: 2px solid darkred; border-radius:10; color: white;} QPushButton:pressed{background: white; border-color: red;}");
	
	cancelButton = new QPushButton("Cancel");
	cancelButton->setMinimumWidth(80);
	cancelButton->setMinimumHeight(20);
	cancelButton->setStyleSheet("QPushButton:enabled{background: yellow; border: 2px solid white; border-radius:10; color: black;} QPushButton:pressed{background: white; border-color: yellow;}");
		
    timeStepLine = new QLineEdit;										// time step
    timeStepLine->setText(QString::number(arena->simTimeStep));
    timeStepLine->setFixedWidth(80);
	timeStepLabel = new QLabel("Time step:");
	
    fixedStepLine = new QLineEdit;										// fixed step
    fixedStepLine->setText(QString::number(arena->simFixedTimeStep));
    fixedStepLine->setFixedWidth(80);
    fixedStepLabel = new QLabel("Fixed step:");

    subStepLine = new QLineEdit;										// sub step
    subStepLine->setText(QString::number(arena->simSubSteps));
    subStepLine->setFixedWidth(80);
    subStepLabel = new QLabel("Sub step:");

    QHBoxLayout *layoutH = new QHBoxLayout;								// add items to the layout
    layoutH->addWidget(timeStepLabel);
    layoutH->addWidget(timeStepLine);
    layoutH->addWidget(fixedStepLabel);
    layoutH->addWidget(fixedStepLine);
    layoutH->addWidget(subStepLabel);
    layoutH->addWidget(subStepLine);

	QHBoxLayout *buttonLayoutH = new QHBoxLayout;
	//QSpacerItem *space = new QSpacerItem(100,10);
	//buttonLayoutH->addItem(space);
	buttonLayoutH->addWidget(updateButton);
	buttonLayoutH->addWidget(cancelButton);
	//buttonLayoutH->addItem(space);
	
    QVBoxLayout *layoutV = new QVBoxLayout;
    layoutV->addLayout(layoutH);
    layoutV->addLayout(buttonLayoutH);

    this->setLayout(layoutV);

	connect(updateButton,SIGNAL(clicked()),this,SLOT(acceptData()));
	connect(cancelButton,SIGNAL(clicked()),this,SLOT(reject()));
}

void simtool::acceptData()
{
	step = timeStepLine->text().toFloat();
	fixedStep = fixedStepLine->text().toFloat();
	subStep = subStepLine->text().toInt();
	this->accept();
}

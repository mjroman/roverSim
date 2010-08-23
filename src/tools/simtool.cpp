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
    updateButton->setStyleSheet(QString("background: red; color:white; border:2px solid rgb(169, 2, 6); border-radius:10px;"));
	
	cancelButton = new QPushButton("Cancel");
	cancelButton->setMinimumWidth(80);
	cancelButton->setStyleSheet("QPushButton:enabled{background: yellow; border: 2px solid white; border-radius:10; color: black;} QPushButton:pressed{background: white; border-color: yellow;}");

    timeStepLabel = new QLabel(this);									// time step
    timeStepLabel->setText(QString("Time step:"));
    timeStepLine = new QLineEdit;
    timeStepLine->setText(QString::number(arena->simTimeStep));
    timeStepLine->setFixedWidth(80);

    fixedStepLine = new QLineEdit;										// fixed step
    fixedStepLine->setText(QString::number(arena->simFixedTimeStep));
    fixedStepLine->setFixedWidth(80);
    fixedStepLabel = new QLabel(this);
    fixedStepLabel->setText(QString("Fixed step:"));

    subStepLine = new QLineEdit;										// sub step
    subStepLine->setText(QString::number(arena->simSubSteps));
    subStepLine->setFixedWidth(80);
    subStepLabel = new QLabel(this);
    subStepLabel->setText(QString("Sub step:"));

    QHBoxLayout *layoutH = new QHBoxLayout;								// add items to the layout
    layoutH->addWidget(timeStepLabel);
    layoutH->addWidget(timeStepLine);
    layoutH->addWidget(fixedStepLabel);
    layoutH->addWidget(fixedStepLine);
    layoutH->addWidget(subStepLabel);
    layoutH->addWidget(subStepLine);

	QHBoxLayout *buttonLayoutH = new QHBoxLayout;
	QSpacerItem *space = new QSpacerItem(100,10);
	buttonLayoutH->addItem(space);
	buttonLayoutH->addWidget(updateButton);
	buttonLayoutH->addWidget(cancelButton);
	buttonLayoutH->addItem(space);
	
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

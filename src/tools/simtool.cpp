#include <QtGui>

#include "simtool.h"
#include "physicsWorld.h"

simtool::simtool(QWidget *parent) : QWidget(parent)
{
	physicsWorld *arena = physicsWorld::instance(); 					// get the physics world object
    QWidget::setWindowFlags(Qt::Sheet);

    updateButton = new QPushButton(tr("&Update"));
    updateButton->setStyleSheet(QString("background:rgb(255, 0, 0); color:white; border:2px solid rgb(169, 2, 6); border-radius:10px;"));
	updateButton->setFixedWidth(100);
	
    connect(updateButton, SIGNAL(clicked()), this, SLOT(close()));
	
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
	buttonLayoutH->addItem(space);
	
    QVBoxLayout *layoutV = new QVBoxLayout;
    layoutV->addLayout(layoutH);
    layoutV->addLayout(buttonLayoutH);

    setLayout(layoutV);
}

simtool::~simtool()
{
}

void simtool::closeEvent(QCloseEvent *event)
{
    emit paramUpdate();
    event->accept();
}

float simtool::getTimeStep()
{
    return timeStepLine->text().toFloat();
}

float simtool::getFixedTimeStep()
{
    return fixedStepLine->text().toFloat();
}

int simtool::getSubSteps()
{
    return subStepLine->text().toInt();
}

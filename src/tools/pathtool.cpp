#include "pathtool.h"
#include "../robot.h"

ColorListEditor::ColorListEditor(QWidget *widget) 
: QComboBox(widget)
{
	populateList();
}
QColor ColorListEditor::color() const
{
	return qVariantValue<QColor>(itemData(currentIndex(), Qt::DecorationRole));
}
void ColorListEditor::setColor(QColor c)
{
	setCurrentIndex(findData(c, int(Qt::DecorationRole)));
}
void ColorListEditor::populateList()
{
	QStringList colorNames = QColor::colorNames();

	for (int i = 0; i < colorNames.size(); ++i) {
		QColor color(colorNames[i]);

		insertItem(i, colorNames[i]);
		setItemData(i, color, Qt::DecorationRole);
	}
}

pathEditDialog::pathEditDialog(pathPlan *ph, QWidget *parent)
: QDialog(parent), path(ph)
{
	setWindowTitle("Set Path Parameters");
	
	// main input layout
	QGridLayout *inputLayout = new QGridLayout;
	colorLabel = new QLabel("Path Color");
	inputLayout->addWidget(colorLabel,0,0);
	colorComboBox = new ColorListEditor;
	colorComboBox->setColor(path->getColor());
	if(colorComboBox->currentIndex() < 0){
		int z = rand() % QColor::colorNames().size();
		colorComboBox->setCurrentIndex(z);
	}
	inputLayout->addWidget(colorComboBox,0,1);
	
  	rangeLabel = new QLabel("Sensor Range");
	inputLayout->addWidget(rangeLabel,1,0);
	rangeLineEdit = new QLineEdit;
	rangeLineEdit->setText(QString::number(path->getRange()));
	rangeLineEdit->setAlignment(Qt::AlignHCenter);
	inputLayout->addWidget(rangeLineEdit,1,1);
	
    stepLabel = new QLabel("Step Distance");
	inputLayout->addWidget(stepLabel,2,0);
	stepLineEdit = new QLineEdit;
	stepLineEdit->setText(QString::number(path->getStep()));
	stepLineEdit->setAlignment(Qt::AlignHCenter);
	inputLayout->addWidget(stepLineEdit,2,1);
	
	breadthLabel = new QLabel("Search Breadth");
	inputLayout->addWidget(breadthLabel,3,0);
	breadthLineEdit = new QLineEdit;
	breadthLineEdit->setText(QString::number(path->getBreadth()));
	breadthLineEdit->setAlignment(Qt::AlignHCenter);
	inputLayout->addWidget(breadthLineEdit,3,1);
	
	saveAllCheckBox = new QCheckBox("Save All Paths");
	saveAllCheckBox->setChecked(ph->getSaveOn());
	inputLayout->addWidget(saveAllCheckBox,4,1);
	
	// Display group box setup
	QVBoxLayout *displayLayout = new QVBoxLayout;
	displayGroupBox = new QGroupBox("Path display items");
    
	baselineCheckBox = new QCheckBox("Baseline");
	displayLayout->addWidget(baselineCheckBox);
	
	lightTrailCheckBox = new QCheckBox("Light Trail");
	displayLayout->addWidget(lightTrailCheckBox);
	
	crowFlyCheckBox = new QCheckBox("Crow Fly Line");
	displayLayout->addWidget(crowFlyCheckBox);
	
	saveDisplayCheckBox = new QCheckBox("Saved Paths");
	displayLayout->addWidget(saveDisplayCheckBox);
	
	displayGroupBox->setLayout(displayLayout);

	// Dialong button layout
	buttonBox = new QDialogButtonBox(Qt::Vertical);
	doneButton = new QPushButton("Done");
	doneButton->setMinimumWidth(80);
	doneButton->setStyleSheet("QPushButton:default{background: green; border: 2px solid darkgreen; border-radius:10; color: white;} QPushButton:pressed{background: red; border: 2px solid darkred; border-radius:10;}");
	doneButton->setDefault(true);
	buttonBox->addButton(doneButton, QDialogButtonBox::AcceptRole);
	
	cancelButton = new QPushButton("Cancel");
	cancelButton->setMinimumWidth(80);
	cancelButton->setStyleSheet("QPushButton:enabled{background: yellow; border: 2px solid white; border-radius:10; color: black;} QPushButton:pressed{background: white; border-color: yellow;}");
	buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
	
	//connect(buttonBox,SIGNAL(),this,SLOT(done(int)));
	connect(buttonBox,SIGNAL(accepted()),this,SLOT(acceptData()));
	connect(buttonBox,SIGNAL(rejected()),this,SLOT(reject()));
	
	connect(baselineCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayPath(bool)));
	connect(lightTrailCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayLightTrail(bool)));
	connect(crowFlyCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayCrowFly(bool)));
	connect(saveDisplayCheckBox,SIGNAL(clicked(bool)),path,SLOT(displaySavedPaths(bool)));
	
	baselineCheckBox->setChecked(path->m_displayPath);
	lightTrailCheckBox->setChecked(path->m_displayLightTrail);
	crowFlyCheckBox->setChecked(path->m_displayCrowFly);
	saveDisplayCheckBox->setChecked(path->m_displaySavedPaths);
	
	// Main window layout
	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addLayout(inputLayout);
	mainLayout->addWidget(displayGroupBox);
	mainLayout->addWidget(buttonBox);
	
	this->setLayout(mainLayout);
}

void pathEditDialog::acceptData()
{
	path->setColor(colorComboBox->color());
	path->setRange(rangeLineEdit->text().toFloat());
	path->setStep(stepLineEdit->text().toFloat());
	path->setBreadth(breadthLineEdit->text().toFloat());
	path->setSaveOn(saveAllCheckBox->isChecked());
	this->accept();
}

pathTool::pathTool(robot *bot, simGLView* glView, QWidget* parent)
:
QWidget(parent),
rover(bot),
view(glView),
selectedPath(0)
{
	setupUi(this);
	move(20,400);
	setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Path Creation");
	
	tableSetup();
	show();
}

pathTool::~pathTool()
{
	removePaths();
}

void pathTool::removePaths()
{
	if(pathList.isEmpty()) return;
	int i;
	for(i=0; i<pathList.size(); i++) delete pathList[i];
	pathList.clear();
	pathTableWidget->clearContents();
}

void pathTool::resetPaths()
{
	int i;
	for(i=0; i<pathList.size(); i++) pathList[i]->reset();
}

void pathTool::tableSetup()
{
	pathTableWidget->setColumnCount(5);
	pathTableWidget->setRowCount(0);
	pathTableWidget->showGrid();
	pathTableWidget->setAlternatingRowColors(false);
	pathTableWidget->verticalHeader()->setResizeMode(QHeaderView::Fixed);
	pathTableWidget->horizontalHeader()->setResizeMode(QHeaderView::Fixed);

	QTableWidgetItem* item = new QTableWidgetItem(" ");		// color
	pathTableWidget->setHorizontalHeaderItem(0,item);		
	pathTableWidget->setColumnWidth(0,28);
	item = new QTableWidgetItem("Range");					// range
	pathTableWidget->setHorizontalHeaderItem(1,item);		
	pathTableWidget->setColumnWidth(1,45);
	item = new QTableWidgetItem("Length");					// length
	pathTableWidget->setHorizontalHeaderItem(2,item);		
	pathTableWidget->setColumnWidth(2,80);
	item = new QTableWidgetItem("Time (s)");				// time
	pathTableWidget->setHorizontalHeaderItem(3,item);		
	pathTableWidget->setColumnWidth(3,100);
	item = new QTableWidgetItem("Efficiency");				// efficiency
	pathTableWidget->setHorizontalHeaderItem(4,item);		
	pathTableWidget->setColumnWidth(4,100);

	this->setMaximumWidth(393);
	
	connect(pathTableWidget, SIGNAL(cellClicked(int,int)),this,SLOT(tableDataChange(int,int)));
	connect(pathTableWidget, SIGNAL(cellDoubleClicked(int,int)),this,SLOT(tableDataEdit(int,int)));
}

// when the [+] button is pressed
void pathTool::on_buttonAdd_clicked()
{
	// execute a dialog here to create a new path
	pathPlan *path = new pathPlan(view);
	pathEditDialog eDialog(path,this);
	if(eDialog.exec() == QDialog::Rejected){
		delete path;
		return;
	}
	
	if(pathList.isEmpty()) buttonDelete->setEnabled(true);				// enable the delete button if the list is empty
	
	pathList.insert(selectedPath, path);								// add the new path to the list
	pathTableWidget->insertRow(selectedPath);							// insert a new row into the table

	QTableWidgetItem* color = new QTableWidgetItem();					// add the color item
	color->setData(Qt::DecorationRole,path->getColor());
	color->setFlags(Qt::ItemIsEnabled);
	pathTableWidget->setItem(selectedPath,0,color);
	
	QTableWidgetItem* range = new QTableWidgetItem();					// add the range item
	range->setData(Qt::DisplayRole,path->getRange());
	QFont ft = range->font();
	ft.setPointSize(12);
	range->setFont(ft);
	range->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	range->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,1,range);
	
	QTableWidgetItem* length = new QTableWidgetItem("...");				// add the length item
	length->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	length->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,2,length);
	
	QTableWidgetItem* stint = new QTableWidgetItem("...");				// add the time item
	stint->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	stint->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,3,stint);
	
	QTableWidgetItem* eff = new QTableWidgetItem("...");				// add the efficiency item
	eff->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	eff->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,4,eff);
}

// when the [-] button is pressed
void pathTool::on_buttonDelete_clicked()
{
	if(selectedPath >= pathList.size()) selectedPath = pathList.size()-1;
	delete pathList[selectedPath];
	pathList.removeAt(selectedPath);
	
	if(pathList.isEmpty()) buttonDelete->setEnabled(false);
	pathTableWidget->removeRow(selectedPath);
}

void pathTool::on_buttonGenerate_clicked()
{
	int i;	
	for(i=0; i<pathList.size(); i++) pathList[i]->reset();		// delete all old paths before creating new ones
	
	for(i=0; i<pathList.size(); i++)
	{
		pathList[i]->goForGoal(rover->position - btVector3(0,0,0.34),goalPoint);
		updateTool();
	}
}

// update the data in the table, this is called after the paths are generated
void pathTool::updateTool()
{
	int i,j;
	for(i=0; i<pathList.size(); i++){
		goalPath* gp = pathList[i]->getShortestPath();
		for(j=0; j<pathTableWidget->columnCount(); j++){
			QTableWidgetItem* item = pathTableWidget->item(i,j);
			switch(j){
				case 0:
				{
					item->setData(Qt::DecorationRole,pathList[i]->getColor());
					break;
				}
				case 1:
				{
					item->setData(Qt::DisplayRole,pathList[i]->getRange());
					break;
				}
				case 2:
				{
					item->setData(Qt::DisplayRole,gp->length);
					break;
				}
				case 3:
				{
					item->setData(Qt::DisplayRole,QVariant(gp->time/1000.));
					break;
				}
				case 4:
				{
					item->setData(Qt::DisplayRole,QVariant(gp->efficiency));
					break;
				}
				default:
				break;
			}
		}
	}
}

void pathTool::tableDataChange(int row, int column)
{
	selectedPath = row;
}

void pathTool::tableDataEdit(int row, int column)
{
	selectedPath = row;
	
	// bring up editing dialog box
	pathEditDialog eDialog(pathList[row],this);
	eDialog.exec();
	updateTool();
}
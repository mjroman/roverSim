#include "pathtool.h"
#include "../robot.h"
#include "../obstacles.h"
#include "../pathPlan.h"
#include "utility/SimDomElement.h"
#include <QSound>

/////////////////////////////////////////
// custom color combo box
/////////////
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

/////////////////////////////////////////
// Path editing custom dialog
/////////////
pathEditDialog::pathEditDialog(pathPlan *ph, QWidget *parent)
: QDialog(parent), path(ph)
{
	setWindowTitle("Path Parameters");
	
	// main input layout
	QGridLayout *inputLayout = new QGridLayout;
	colorLabel = new QLabel("Path Color");								// path color
	inputLayout->addWidget(colorLabel,0,0);
	colorComboBox = new ColorListEditor;
	colorComboBox->setColor(path->getColor());
	if(colorComboBox->currentIndex() < 0){
		int z = rand() % QColor::colorNames().size();
		colorComboBox->setCurrentIndex(z);
	}
	colorComboBox->setMaximumWidth(100);
	inputLayout->addWidget(colorComboBox,0,1);
	
  	rangeLabel = new QLabel("Sensor Range");							// sensor range
	inputLayout->addWidget(rangeLabel,1,0);
	rangeLineEdit = new QLineEdit(this);
	rangeLineEdit->setText(QString::number(path->getRange()));
	rangeLineEdit->setToolTip("The distance in meters that obstacles are detected.\nSet to 0 for Infinite range");
	rangeLineEdit->setAlignment(Qt::AlignHCenter);
	rangeLineEdit->setMaximumWidth(100);
	this->setFocusProxy(rangeLineEdit);		// sets the range line edit as the focused item when the dialog is shown
	rangeLineEdit->selectAll();
	inputLayout->addWidget(rangeLineEdit,1,1);
	
	growLabel = new QLabel("CSpace Growth");							// C-Space growth margin
	inputLayout->addWidget(growLabel,2,0);
	growLineEdit = new QLineEdit(this);
	growLineEdit->setText(QString::number(path->getMargin()));
	growLineEdit->setToolTip("The growth size of the C-Space objects.");
	growLineEdit->setAlignment(Qt::AlignHCenter);
	growLineEdit->setMaximumWidth(100);
	inputLayout->addWidget(growLineEdit,2,1);
	
    stepLabel = new QLabel("Step Distance");							// step distance
	inputLayout->addWidget(stepLabel,3,0);
	stepLineEdit = new QLineEdit(this);
	stepLineEdit->setText(QString::number(path->getStep()));
	stepLineEdit->setToolTip("The distance between sensor updates");
	stepLineEdit->setAlignment(Qt::AlignHCenter);
	stepLineEdit->setMaximumWidth(100);
	stepLineEdit->setEnabled(path->getRange() != 0);
	inputLayout->addWidget(stepLineEdit,3,1);
	
	efficiencyLabel = new QLabel("Efficiency Limit");					// efficiency limit
	inputLayout->addWidget(efficiencyLabel,4,0);
	efficiencyLineEdit = new QLineEdit(this);
	efficiencyLineEdit->setText(QString::number(path->getEffLimit()));
	efficiencyLineEdit->setToolTip("Path searching exits if efficiency degrades to this limit");
	efficiencyLineEdit->setAlignment(Qt::AlignHCenter);
	efficiencyLineEdit->setMaximumWidth(100);
	efficiencyLineEdit->setEnabled(path->getRange() != 0);
	inputLayout->addWidget(efficiencyLineEdit,4,1);

	spinLabel = new QLabel("Spin Progress Limit");						// spin progress
	inputLayout->addWidget(spinLabel,5,0);
	QHBoxLayout *spinLayout = new QHBoxLayout;
	spinLineEdit = new QLineEdit(this);
	spinLineEdit->setText(QString::number(path->getSpinLimit()));
	spinLineEdit->setToolTip("The distance the path search will step if a local minima is reached");
	spinLineEdit->setAlignment(Qt::AlignHCenter);
	spinLineEdit->setMaximumWidth(35);
	spinLineEdit->setEnabled(path->getRange() != 0);
	spinLayout->addWidget(spinLineEdit);
	spinBaseBox = new QComboBox(this);
	spinBaseBox->addItem("Dist.");
	spinBaseBox->addItem("Step");
	spinBaseBox->setCurrentIndex(path->getSpinBase());
	spinBaseBox->setToolTip("Sets how the progress distance will be compared.");
	spinBaseBox->setMaximumWidth(65);
	spinBaseBox->setEnabled(path->getRange() != 0);
	spinLayout->addWidget(spinBaseBox);
	inputLayout->addLayout(spinLayout,5,1);
	
	breadthLabel = new QLabel("Search Breadth");						// path search breadth
	inputLayout->addWidget(breadthLabel,6,0);
	breadthLineEdit = new QLineEdit(this);
	breadthLineEdit->setText(QString::number(path->getBreadth()));
	breadthLineEdit->setToolTip("The maximum number of paths to search at each path node.\nSet to 0 for complete search");
	breadthLineEdit->setAlignment(Qt::AlignHCenter);
	breadthLineEdit->setMaximumWidth(100);
	inputLayout->addWidget(breadthLineEdit,6,1);
	
	saveAllCheckBox = new QCheckBox("Save All Paths");					// save paths 
	saveAllCheckBox->setChecked(ph->getSaveOn());
	saveAllCheckBox->setToolTip("Saves all potential paths to the goal while searching for shortest.\nFor viewing only");
	inputLayout->addWidget(saveAllCheckBox,7,1);
	
	// Display group box setup
	QVBoxLayout *displayLayout = new QVBoxLayout;
	displayGroupBox = new QGroupBox("Path display items");
    
	baselineCheckBox = new QCheckBox("Baseline");
	baselineCheckBox->setToolTip("Draws the shortest path in the selected color");
	displayLayout->addWidget(baselineCheckBox);
	
	lightTrailCheckBox = new QCheckBox("Light Trail");
	displayLayout->addWidget(lightTrailCheckBox);
	
	crowFlyCheckBox = new QCheckBox("Crow Fly Line");
	crowFlyCheckBox->setToolTip("Displays a vector line from the start to goal.\nFor debugging");
	displayLayout->addWidget(crowFlyCheckBox);
	
	saveDisplayCheckBox = new QCheckBox("Saved Paths");
	saveDisplayCheckBox->setToolTip("Display all potential paths if they have been saved");
	displayLayout->addWidget(saveDisplayCheckBox);
	
	cspaceDisplayCheckBox = new QCheckBox("Config Space");
	cspaceDisplayCheckBox->setToolTip("Display Configuration Space while searching for paths");
	displayLayout->addWidget(cspaceDisplayCheckBox);
	
	displayGroupBox->setLayout(displayLayout);

	// Dialong button layout
	QVBoxLayout *buttonBox = new QVBoxLayout;
	doneButton = new QPushButton("Accept");
	doneButton->setMinimumWidth(80);
	doneButton->setStyleSheet("QPushButton:default{background: green; border: 2px solid darkgreen; border-radius:10; color: white;} QPushButton:pressed{background: red; border: 2px solid darkred; border-radius:10;}");
	doneButton->setDefault(true);
	buttonBox->addWidget(doneButton);
	
	cancelButton = new QPushButton("Cancel");
	cancelButton->setMinimumWidth(80);
	cancelButton->setStyleSheet("QPushButton:enabled{background: yellow; border: 2px solid white; border-radius:10; color: black;} QPushButton:pressed{background: white; border-color: yellow;}");
	buttonBox->addWidget(cancelButton);
	
	connect(doneButton,SIGNAL(clicked()),this,SLOT(acceptData()));
	connect(cancelButton,SIGNAL(clicked()),this,SLOT(reject()));
	
	connect(baselineCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayPath(bool)));
	connect(lightTrailCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayLightTrail(bool)));
	connect(crowFlyCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayCrowFly(bool)));
	connect(saveDisplayCheckBox,SIGNAL(clicked(bool)),path,SLOT(displaySavedPaths(bool)));
	connect(cspaceDisplayCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayCspace(bool)));
	
	connect(rangeLineEdit,SIGNAL(editingFinished()),this,SLOT(enableLines()));
	connect(rangeLineEdit,SIGNAL(editingFinished()),this,SLOT(stepWarning()));
	connect(growLineEdit,SIGNAL(editingFinished()),this,SLOT(stepWarning()));
	connect(stepLineEdit,SIGNAL(editingFinished()),this,SLOT(stepWarning()));
	
	baselineCheckBox->setChecked(path->m_displayPath);
	lightTrailCheckBox->setChecked(path->m_displayLightTrail);
	crowFlyCheckBox->setChecked(path->m_displayCrowFly);
	saveDisplayCheckBox->setChecked(path->m_displaySavedPaths);
	cspaceDisplayCheckBox->setChecked(path->m_displayCS);
	
	// Main window layout
	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addLayout(inputLayout);
	mainLayout->addWidget(displayGroupBox);
	mainLayout->addLayout(buttonBox);
	
	this->setLayout(mainLayout);
}

void pathEditDialog::enableLines()
{
	if(rangeLineEdit->text().toFloat() == 0){
		stepLineEdit->setEnabled(false);
		efficiencyLineEdit->setEnabled(false);
		spinLineEdit->setEnabled(false);
		spinBaseBox->setEnabled(false);
	}
	else{
		stepLineEdit->setEnabled(true);
		efficiencyLineEdit->setEnabled(true);
		spinLineEdit->setEnabled(true);
		spinBaseBox->setEnabled(true);
	}
}
void pathEditDialog::stepWarning()
{
	float range = rangeLineEdit->text().toFloat();
	float grow = growLineEdit->text().toFloat();
	float step = stepLineEdit->text().toFloat();
	
	if(range == 0) return;
	
	if(step <= 0){
		QMessageBox::warning(this,
							"Caution !!!",
							"Sensor update step must be greater than ZERO.",
							QMessageBox::Ok,
							QMessageBox::Ok);
		stepLineEdit->setText(QString::number(0.01));
	}
	else if(range < (grow + step)){
		QMessageBox::warning(this,
							"Caution !!!",
							"Sensor range is too small with respect to C-Space margin and update step size.\n\nSENSOR RANGE > STEP SIZE + C-SPACE",
							QMessageBox::Ok,
							QMessageBox::Ok);
		rangeLineEdit->setText(QString::number(grow + step + 0.01));
	}
	else if(step > range - grow)
	{
		QMessageBox::warning(this,
							"Caution !!!",
							"Update step size is too large with respect to sensor range or C-Space margin.\n\nSENSOR RANGE >= STEP SIZE + C-SPACE",
							QMessageBox::Ok,
							QMessageBox::Ok);
		rangeLineEdit->setText(QString::number(grow + step + 0.01));
	}
}

void pathEditDialog::acceptData()
{
	float temp;
	path->setColor(colorComboBox->color());
	path->setRange(rangeLineEdit->text().toFloat());
	path->setMargin(growLineEdit->text().toFloat());
	
	temp = stepLineEdit->text().toFloat();
	if(temp > path->getRange()) path->setStep(path->getRange());
	else path->setStep(temp);
	
	temp = fabs(efficiencyLineEdit->text().toFloat());
	if(temp > 1 || temp == 0) path->setEffLimit(1);
	else path->setEffLimit(temp);
	
	temp = fabs(spinLineEdit->text().toFloat());
	path->setSpinLimit(temp);
	
	path->setSpinBase(spinBaseBox->currentIndex());
	
	temp = fabs(breadthLineEdit->text().toInt());
	path->setBreadth(temp);
	
	path->setSaveOn(saveAllCheckBox->isChecked());
	this->accept();
}

/////////////////////////////////////////
// Path creation main tool
/////////////
pathTool::pathTool(robot *bot, obstacles *obs, simGLView* glView)
:
QWidget(glView->parentWidget()),
rover(bot),
blocks(obs),
view(glView),
m_selectedPath(0),
m_foundSound(QDir::currentPath() + "/Resources/sounds/singleBeep2.wav"),
m_filename(NULL),
m_file(NULL),
m_xmlDoc( "roverSimDoc" )
{
	setupUi(this);
	setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Path Creation");
	
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");
	if(!QFile::exists(settings.fileName()) || !settings.contains("PathToolWindowGeom")){
		move(20,540);
		resize(380,150);
		settings.setValue("PathToolWindowGeom", saveGeometry());
	}
	else
		this->restoreGeometry(settings.value("PathToolWindowGeom").toByteArray());
	
	
	buttonDelete->setEnabled(false);
	tableSetup();
	
	pathTableWidget->setToolTip("Double click a path to edit parameters.\nUse X and Z to walk over selected path!");
	buttonGenerate->setToolTip("Generates the paths in the list above");
	checkBoxSave->setToolTip("Check this box BEFORE generating paths to save them!");
	
	connect(this,SIGNAL(changeBackground(int,QBrush)),this,SLOT(setRowBackground(int,QBrush)));
	connect(this,SIGNAL(computePath(int)),this,SLOT(processPath(int)));
	connect(pathTableWidget, SIGNAL(cellClicked(int,int)),this,SLOT(tableDataChange(int,int)));
	connect(pathTableWidget, SIGNAL(cellDoubleClicked(int,int)),this,SLOT(tableDataEdit(int,int)));	
	show();
}

pathTool::~pathTool()
{
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");
	settings.setValue("PathToolWindowGeom", saveGeometry());
	removePaths();
	if(m_file) delete m_file;
}

void pathTool::tableSetup()
{
	this->setUpdatesEnabled(true);
	pathTableWidget->setColumnCount(7);
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
	item = new QTableWidgetItem("Comp. Efficiency");		// Infinite range comparison efficiency
	pathTableWidget->setHorizontalHeaderItem(4,item);
	pathTableWidget->setColumnWidth(4,100);
	item = new QTableWidgetItem("Efficiency");				// efficiency
	pathTableWidget->setHorizontalHeaderItem(5,item);		
	pathTableWidget->setColumnWidth(5,100);
	item = new QTableWidgetItem("State");					// path State
	pathTableWidget->setHorizontalHeaderItem(6,item);
	pathTableWidget->setColumnWidth(6,100);

	this->setMaximumWidth(553+39);	// max width of table, all items width + right collumn width
}

void pathTool::show()
{
	QPropertyAnimation *anim = new QPropertyAnimation(this,"pos");
	anim->setDuration(1000);
	anim->setStartValue(QPoint(pos().x(),pos().y()-25));
	anim->setEndValue(pos());
	anim->setEasingCurve(QEasingCurve::OutElastic);
	anim->start();
	QWidget::show();
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
	for(i=0; i<pathList.size(); i++){
		pathList[i]->reset();									// delete all old paths before creating new ones
		emit changeBackground(i,QBrush(QColor(Qt::white)));
	}
}

// when the [+] button is pressed
void pathTool::on_buttonAdd_clicked()
{
	// execute a dialog here to create a new path
	pathPlan *path = new pathPlan(blocks,view);
	pathEditDialog eDialog(path,this);
	if(eDialog.exec() == QDialog::Rejected){
		delete path;
		return;
	}
	
	int selected;
	
	if(pathList.isEmpty()) buttonDelete->setEnabled(true);				// enable the delete button if the list is empty
	
	if(!pathList.isEmpty() && path->getRange() == 0){
		if(pathList.last()->getRange() == 0) return;
		selected = pathList.size();
		pathList << path;											// add the infinite range path last
	}
	else{
		selected = m_selectedPath;
		pathList.insert(selected, path);							// add the new path to the list
	}
	
	pathTableWidget->insertRow(selected);							// insert a new row into the table

	QTableWidgetItem* color = new QTableWidgetItem();					// add the color item
	color->setData(Qt::DecorationRole,path->getColor());
	color->setFlags(Qt::ItemIsEnabled);
	pathTableWidget->setItem(selected,0,color);
	
	QTableWidgetItem* range = new QTableWidgetItem();					// add the range item
	if(path->getRange() == 0) {
		range->setData(Qt::DisplayRole,QString::fromWCharArray(L"\x221e"));
	}
	else {
		range->setData(Qt::DisplayRole,path->getRange());
	}
	range->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	range->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selected,1,range);
	
	QTableWidgetItem* length = new QTableWidgetItem("...");				// add the length item
	length->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	length->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selected,2,length);
	
	QTableWidgetItem* stint = new QTableWidgetItem("...");				// add the time item
	stint->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	stint->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	stint->setToolTip("Path computation time in Seconds");
	pathTableWidget->setItem(selected,3,stint);
	
	QTableWidgetItem* comp = new QTableWidgetItem("...");				// add comparison efficiency item
	comp->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	comp->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	comp->setToolTip("Efficiency based on infinite sensor range path");
	pathTableWidget->setItem(selected,4,comp);
	
	QTableWidgetItem* eff = new QTableWidgetItem("...");				// add the efficiency item
	eff->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	eff->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	eff->setToolTip("Efficiency based on straight line distance to goal");
	pathTableWidget->setItem(selected,5,eff);
	
	QTableWidgetItem* state = new QTableWidgetItem("...");				// add the path state item
	state->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	state->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	state->setToolTip("Path computation exit state");
	pathTableWidget->setItem(selected,6,state);
}

// when the [-] button is pressed
void pathTool::on_buttonDelete_clicked()
{
	if(m_selectedPath >= pathList.size()) m_selectedPath = pathList.size()-1;
	delete pathList[m_selectedPath];
	pathList.removeAt(m_selectedPath);
	
	if(pathList.isEmpty()) buttonDelete->setEnabled(false);
	pathTableWidget->removeRow(m_selectedPath);
}

void pathTool::on_buttonGenerate_clicked()
{
	if(pathList.isEmpty()) return;
	pathList[m_selectedPath]->reset();
	
	m_allPaths = false;
	emit changeBackground(m_selectedPath,QBrush(QColor("springgreen")));
	emit computePath(m_selectedPath);
}

void pathTool::on_buttonGenAll_clicked()
{
	if(pathList.isEmpty()) return;
	resetPaths();
	if(!initSaveFile()) return;
	
	m_allPaths = true;
	emit changeBackground(0,QBrush(QColor("springgreen")));
	emit computePath(0);
}

void pathTool::processPath(int x)
{
	qApp->processEvents();
	if(x >= pathList.size()){																		// finished computing paths
		m_allPaths = false;
		QSound::play(QDir::currentPath() + "/Resources/sounds/singleBell.wav");
		if(pathList.last()->getRange() == 0)
			updateCompEfficiency(pathList.last()->getShortestLength());								// updates comparison efficiency and writes data to file
		return;
	}
	
	if(x != 0) m_foundSound.play();

	pathList[x]->goForGoal(rover->position - btVector3(0,0,0.34),goalPoint);						// find the shortest path from start to goal points
	
	if(checkBoxSave->isChecked()){
		m_xmlDoc.documentElement().appendChild(SimDomElement::pathToNode(m_xmlDoc,pathList[x]));	// write the data to the xml document
		m_xmlStream.seek(0);																		// start writing data to the begining of the file
		m_xmlDoc.save(m_xmlStream,5);																// sync the data to the file, incase the simulation crashes
	}
	updateTool();
	
	if(pathList[x]->getState() != PS_COMPLETE)
		emit changeBackground(x,QBrush(QColor(Qt::yellow)));
	else
		emit changeBackground(x,QBrush(QColor("lightsteelblue")));
		
	if(m_allPaths){																						// if all paths are being generated
		x++;																							// increment to the next path in the list
		emit changeBackground(x,QBrush(QColor("springgreen")));
		if(x < pathList.size()) pathTableWidget->scrollToItem(pathTableWidget->item(x,0));				// scroll the table view down
		emit computePath(x);
	}
}

// update the data in the table, this is called after the paths are generated
void pathTool::updateTool()
{
	int i,j;
	for(i=0; i<pathList.size(); i++){
		const goalPath* gp = pathList[i]->getShortestPath();
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
					float r = pathList[i]->getRange();
					if(r == 0)	item->setData(Qt::DisplayRole,QString::fromWCharArray(L"\x221e"));
					else item->setData(Qt::DisplayRole,r);
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
					//item->setText("...");
					break;
				}
				case 5:
				{
					item->setData(Qt::DisplayRole,QVariant(gp->efficiency));
					break;
				}
				case 6:
				{
					int ps = pathList[i]->getState();
					if(ps == PS_SEARCHING) item->setData(Qt::DisplayRole, "Searching");
					else if(ps == PS_COMPLETE) item->setData(Qt::DisplayRole, "Complete");
					else if(ps == PS_PATHNOTFOUND) item->setData(Qt::DisplayRole, "No Path");
					else if(ps == PS_SWITCHBACKLEFT) item->setData(Qt::DisplayRole, "Switchback LEFT");
					else if(ps == PS_SWITCHBACKRIGHT) item->setData(Qt::DisplayRole, "Switchback RIGHT");
					else if(ps == PS_NOPROGRESS) item->setData(Qt::DisplayRole, "No Progress");
					else item->setData(Qt::DisplayRole, "unknown");
					break;
				}
				default:
				break;
			}
		}
	}
}

void pathTool::updateCompEfficiency(float gLength)
{
	int i;
	for(i=0; i<pathList.size(); i++){
		QTableWidgetItem* item = pathTableWidget->item(i,4);
		item->setData(Qt::DisplayRole,QVariant(gLength/pathList[i]->getShortestLength()));
	}
	if(m_xmlStream.device()){
		m_xmlStream.seek(0);																			// start writing data to the begining of the file
		m_xmlDoc.save(m_xmlStream,5);																	// sync the data to the file, incase the simulation crashes
		m_file->close();
	}
}

// sets the table row background color
void pathTool::setRowBackground(int row, QBrush stroke)
{
	if(row >= pathTableWidget->rowCount()) return;
	for(int i=1;i<pathTableWidget->columnCount();i++){
		QTableWidgetItem* item = pathTableWidget->item(row,i);
		item->setBackground(stroke);
	}
}

void pathTool::tableDataChange(int row, int column)
{
	pathTableWidget->selectRow(row);					// highlights the row
	if(row == m_selectedPath) return;					// return if the same row is selected twice
	if(m_selectedPath < pathList.size())
		pathList[m_selectedPath]->togglePathReset();	// reset the previously selected path
		
	m_selectedPath = row;
	pathList[m_selectedPath]->displayPath(false);		// turn off normal path display
	pathList[m_selectedPath]->displayBuildPath(true);	// make the path blink
}

// called when user double clicks on an item brings up the path editing dialog
void pathTool::tableDataEdit(int row, int column)
{
	m_selectedPath = row;
	
	// bring up editing dialog box
	pathEditDialog eDialog(pathList[row],this);
	eDialog.exec();
	updateTool();
}

// called when the user goes to path view
void pathTool::stepOnPath(int dir)
{
	if(pathList.isEmpty()) return;
	if(m_selectedPath < pathList.size() && m_selectedPath >= 0)
		pathList[m_selectedPath]->togglePathPoint(dir);
}

// sets up an XML file to save path data too
// returns FALSE if there is an error
bool pathTool::initSaveFile()
{
	if(!checkBoxSave->isChecked()) return true;

	if(m_filename == NULL){
		m_filename = QFileDialog::getSaveFileName(view->parentWidget(),"Save Path Data", QDir::homePath());	// open a Save File dialog and select location and filename
		if(m_filename == NULL){
			checkBoxSave->setChecked(false);														// if cancel is pressed turn off saving
			return true;
		}
		m_runCount = 0;																				// reset the run count for a new series
	}
	else{
		m_runCount++;																				// increment the filename
		m_xmlDoc.clear();
	}

	int hyp = m_filename.lastIndexOf("-");															// remove old file endings
	if(hyp != -1) m_filename.truncate(hyp);
	if(m_filename.endsWith(".xml")) m_filename.replace(".xml","");
	
	if(!blocks->isSaved()) 
		blocks->saveLayout(m_filename + "_layout_" + QString::number(m_runCount) + ".xml"); 		// save the obstacle layout
	
	m_file = new QFile(m_filename + "_" + QString::number(m_runCount) + ".xml");
	if (!m_file->open(QIODevice::WriteOnly)){														// open XML file
		view->printText("Save File Error - " + m_filename);
		return false;
	}
	m_xmlStream.setDevice(m_file);																	// set the stream device
	
	QDomElement root = m_xmlDoc.createElement( "PathData" );										// create a root element
	m_xmlDoc.appendChild(root);
	QString docInfo = "This XML document represents calculated path data from the RoverSim application";
	root.appendChild(m_xmlDoc.createComment(docInfo));
	
	root.setAttribute( "layoutFile", blocks->getLayoutName());										// write layout name
	
	btVector3 startPoint = rover->position - btVector3(0,0,0.34);
	QDomElement goalLine = m_xmlDoc.createElement( "startgoal" );
	goalLine.setAttribute( "distance", QString::number(startPoint.distance(goalPoint)));			// write straight line distance
	goalLine.appendChild(SimDomElement::vectorToNode(m_xmlDoc,startPoint));							// write start point
	goalLine.appendChild(SimDomElement::vectorToNode(m_xmlDoc,goalPoint));							// write goal point
	root.appendChild(goalLine);
	
	return true;
}
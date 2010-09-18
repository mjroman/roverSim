#include "terrain.h"
#include <QFileDialog>

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#define SAFETYPLANE    -5

GLfloat GroundColor[4] = { 0.95f, 0.95f, 0.95f, 1.0f };

// Adds the terrain mesh into the physics world for collision detection
// A flat plane is also placed below the terrain to catch any objects
// that might fall out of the broadphase limits, which could cause problems.
terrain::terrain(QString filename,simGLView* glView)
:
simGLObject(glView),
m_terrainVerts(0),
m_terrainColors(0),
m_terrainNormals(0),
m_terrainTriangles(0),
m_terrainMaxHeight(0),
m_terrainMinHeight(0),
m_pixelSize(100,100),
m_terrainModified(false)
{
    arena = physicsWorld::instance(); 					// get the physics world object
	tTool = new terrainTool(m_view->parentWidget());
	
    connect(tTool, SIGNAL(sizeUpdate(btVector3)), this, SLOT(setTerrainSize(btVector3)));

	this->openTerrain(filename);
}

terrain::~terrain()
{
    terrainClear();
	delete tTool;
}

// deletes the terrain vertices, colors, normals and triangle indices
// be sure to stop drawing and simulation calculations when doing this
void terrain::terrainClear()
{
    int i = m_terrainObjects.size();
	m_view->stopDrawing();	// do not draw while items are deleted
 	arena->stopSimTimer();			// pause simulation
	
	while(i>0){
		btCollisionObject* obj = m_terrainObjects[i-1];
		arena->getDynamicsWorld()->removeCollisionObject(obj);
		m_terrainObjects.pop_back();
		i = m_terrainObjects.size();
	}

   	m_terrainShapes.clear();

    if(m_terrainVerts) { delete m_terrainVerts; }
    m_terrainVerts = 0;
    if(m_terrainColors) { delete m_terrainColors; }
    m_terrainColors = 0;
    if(m_terrainNormals) { delete m_terrainNormals; }
    m_terrainNormals = 0;
    if(m_terrainTriangles) { delete m_terrainTriangles; }
    m_terrainTriangles = 0;
	
	arena->resetWorld();	// reset the world and unpause simulation	
	m_view->startDrawing();	// draw obstacles
}

// loads a BMP file as the height map for the terrain. If the data
// from filename is valid the world is reset and new terrain is created.
int terrain::terrainLoadFile()
{
    QImage heightMap(m_terrainFilename);
    if(heightMap.isNull()){
        m_view->printText("Terrain File could not open");
        return 0; // if the file does not open return
    }

    QStringList keyList = heightMap.textKeys();
    if(keyList.contains("sizex")) m_terrainSize.setX(heightMap.text("sizex").toFloat());	// import the size of the terrain from the image
    if(keyList.contains("sizey")) m_terrainSize.setY(heightMap.text("sizey").toFloat());
    if(keyList.contains("sizez")) m_terrainSize.setZ(heightMap.text("sizez").toFloat());
	
	tTool->setSize(m_terrainSize);															// write the new size to the tool
	
    m_pixelSize = heightMap.size();															// set the pixel size of the map

    int imgSize = m_pixelSize.width()*m_pixelSize.height();									// get the number of pixels

    unsigned int *imgData = new unsigned int[imgSize+1];									// create a new array to hold all the height data from the image
    if(imgData == NULL) {
        m_view->printText("Memory error");
        return 0;
    }

    imgData[imgSize] = 0; 																	// the last element in the array contains the maximum height
	
    for(int j=0; j<m_pixelSize.height(); j++){													// load all the height data from the gray scale part of the height image
        for(int i=0; i<m_pixelSize.width(); i++){
            int k = i + j * m_pixelSize.width();
            imgData[k] = qGray(heightMap.pixel(i,j));										// get the height of the pixel
            if(imgData[k] > imgData[imgSize]) 
				imgData[imgSize] = imgData[k];												// save the highest height as the last element
        }
    }

	terrainCreateMesh(imgData); 															// create the new terrain mesh
	
    delete imgData;																			// free the temporary height data
    return 1;
}

// creates a new triangle based terrain mesh, be sure heightData is large enough
// to fit the world which must be set prior. This function fills the proper arrays
// for both OpenGL and Bullet.
void terrain::terrainCreateMesh(unsigned int *heightData)
{
    int i,j,k;
	btVector3 scale;
    int xWorld = m_pixelSize.width();
    int yWorld = m_pixelSize.height();
	
	scale.setX(m_terrainSize.x() / m_pixelSize.width());
	scale.setY(m_terrainSize.y() / m_pixelSize.height());
	if(heightData != NULL) scale.setZ(m_terrainSize.z() / (float)heightData[m_terrainVertexCount]);
	else scale.setZ(1);
	
    m_terrainVertexCount = xWorld * yWorld;   				// calculate the number of verticies
    m_terrainTriangleCount = (xWorld-1) * (yWorld-1)*2; 	// calculate the number of triangles
	
    // allocate new data arrays
    m_terrainVerts = new Vertex[m_terrainVertexCount];
    m_terrainColors = new Vertex[m_terrainVertexCount];
    m_terrainNormals = new Vertex[m_terrainVertexCount];
    m_terrainTriangles = new Triangle[m_terrainTriangleCount];
	m_terrainMaxHeight = m_terrainSize.z();
	m_terrainMinHeight = 0;
	
    // generate the vertices for the triangle mesh
    // This sets the proper coordinates for the vertices that
    // will make up the terrain triangle mesh. The color of
    // each vertex is based on the height of from the BMP
    for(i=0; i<m_terrainVertexCount; i++)
    {
        m_terrainVerts[i].x = (i % xWorld) * scale.x();
        m_terrainVerts[i].y = (i / xWorld) * scale.y();

		if(heightData != NULL) 
			m_terrainVerts[i].z = heightData[i] * scale.z();	// get the height from the image data and scale it
		else 
			m_terrainVerts[i].z = m_terrainSize.z();
			
        m_terrainColors[i].x = 0.7 * m_terrainVerts[i].z / m_terrainMaxHeight;		// set the color based on the terrain height
        m_terrainColors[i].y = 0.7 * m_terrainVerts[i].z / m_terrainMaxHeight;
        m_terrainColors[i].z = 0.7 * m_terrainVerts[i].z / m_terrainMaxHeight;
    }
	
    // generate the indices which point to the vertices of the triangle mesh
    // the m_terrainTriangle array is used to tell OpenGL and Bullet
    // what vertices make up each triangle. This is done so the same vertices
    // can be re-used for different triangles
    k=0;
    for(j=1;j<yWorld;j++)
    {
        for(i=1;i<xWorld;i++)
        {
            int ind1 = ((j-1) * xWorld) + (i-1);
            int ind2 = (j * xWorld) + (i-1);
            int ind3 = ((j-1) * xWorld) + i;
            int ind4 = (j * xWorld) + i;
			
            // low triangle
            m_terrainTriangles[k].v1 = ind1;
            m_terrainTriangles[k].v2 = ind3;
            m_terrainTriangles[k].v3 = ind2;
            // high triangle
            m_terrainTriangles[k+1].v1 = ind2;
            m_terrainTriangles[k+1].v2 = ind3;
            m_terrainTriangles[k+1].v3 = ind4;
            k += 2;
        }
    }
    buildNormals();
}

// Generate the normal vectors for the terrain mesh
// this is important for viewing, without proper normal calculations
// it is difficult to see the 3rd dimension of shapes with sides all
// the same color
void terrain::buildNormals()
{
    int i,j;
	
    for(j=0;j<m_pixelSize.height();j++)
    {
        for(i=0;i<m_pixelSize.width();i++)
        {
            int v0 = j*m_pixelSize.width() + i;					// get the middle vertex and the four vertices surrounding it
            int v1 = (j-1)*m_pixelSize.width() + i;
            int v2 = j*m_pixelSize.width() + (i+1);
            int v3 = (j+1)*m_pixelSize.width() + i;
            int v4 = j*m_pixelSize.width() + (i-1);
			
            if(j-1 < 0)					{ v1 = -1; }		// if a middle vertex is near an edge discard the corresponding vertex
            if(i+1 == m_pixelSize.width()) 	{ v2 = -1; }
            if(j+1 == m_pixelSize.height()) 	{ v3 = -1; }
            if(i-1 < 0) 				{ v4 = -1; }
			
            m_terrainNormals[v0] = sumNormals(v0,v1,v2,v3,v4,m_terrainVerts);
        }
    }
}

// the center of editing takes place at HERE
// raise terrain DIR = 1, lower terrain DIR = -1
void terrain::terrainEdit(btVector3 here, int dir)
{
	float area = tTool->diameter();
	float amount = tTool->increment();
	
	if(area <= 0 || amount <= 0) return;
	
	amount *= dir;															// determine the direction of editing, up or down
	
	float xp = here.x();													// make sure the X location of the edit is over the terrain
    if(xp < 0) xp = 0;
    else if(xp > m_terrainSize.x()) xp = m_terrainSize.x();
	
	float yp = here.y();													// make sure the Y location of the edit is over the terrain
    if(yp < 0) yp = 0;
    else if(yp > m_terrainSize.y()) yp = m_terrainSize.y();
	
	for(int i=0; i<m_terrainVertexCount; i++)								// loop through the terrain vertex array
	{
        float x = (i % m_pixelSize.width()) * m_terrainSize.x()/m_pixelSize.width();// get the X and Y incides of the terrain vertices
        float y = (i / m_pixelSize.width()) * m_terrainSize.y()/m_pixelSize.height();

		float xd = xp - x;
		float yd = yp - y;

		float d = (float)sqrt((xd*xd) + (yd*yd));							// calculate the diameter of the area
		float a = (m_terrainVerts[i].z + amount) - (amount*(d / area));		// calculate the new height of the vertex
		
		if((dir == 1 && a > m_terrainVerts[i].z) || (dir == -1 && a < m_terrainVerts[i].z)) {
			m_terrainVerts[i].z = a;
			
			if(a > m_terrainMaxHeight) 
				m_terrainMaxHeight = a;
			else if(a < m_terrainMinHeight)
				m_terrainMinHeight = a;
				
			m_terrainColors[i].x = 0.7 * a / m_terrainMaxHeight;
			m_terrainColors[i].y = 0.7 * a / m_terrainMaxHeight;
			m_terrainColors[i].z = 0.6 * a / m_terrainMaxHeight;
		}
	}
	
	buildNormals();

    this->terrainRefresh();
	m_terrainModified = true;
}

float terrain::terrainHeightAt(btVector3 pt)
{
	float avgHeight;
	
	int px = (pt.x() / m_terrainSize.x()) * m_pixelSize.width();								// get the array indices of the point
	int py = (pt.y() / m_terrainSize.y()) * m_pixelSize.height();
	
    int index = px + py * m_pixelSize.width();	
	if(px <= 0){
		if(py <= 0) return m_terrainVerts[0].z;												// return the lower left corner height
		if(py >= m_pixelSize.height()) return m_terrainVerts[(m_pixelSize.height()-1)*m_pixelSize.width()].z;	// return the upper left corner height
		return m_terrainVerts[py * m_pixelSize.width()].z;										// return height of point on left edge of terrain
	}
	else if(px >= m_pixelSize.width()-1){
		if(py <= 0) return m_terrainVerts[m_pixelSize.width()-1].z; 									// return the lower right corner heigth
		if(py >= m_pixelSize.height()) return m_terrainVerts[(m_pixelSize.height()-1)*(m_pixelSize.width()-1)].z; // return the upper right corner height
		return m_terrainVerts[py*m_pixelSize.width() - 1].z; 											// return height of point on right edge of terrain
	}
	if(py <= 0) return m_terrainVerts[px].z;															// return height of point on bottom edge of terrain
	else if(py >= m_pixelSize.height()-1) return m_terrainVerts[px + (m_pixelSize.height()-1)*m_pixelSize.width()].z; // return height of point on top edge

	// otherwise take an average of the heights on the corners of a square containing the point
    avgHeight = m_terrainVerts[index].z;
    avgHeight += m_terrainVerts[index+1].z;
    avgHeight += m_terrainVerts[index+m_pixelSize.width()].z;
    avgHeight += m_terrainVerts[index+1+m_pixelSize.width()].z;

    return avgHeight/4;
}

// if the terrain mesh is edited it must be refit to the physics world
void terrain::terrainRefresh()
{
    arena->getDynamicsWorld()->updateAabbs();
    m_triMesh->refitTree(btVector3(0,0,m_terrainMinHeight),btVector3(m_terrainSize.x(),m_terrainSize.y(),m_terrainMaxHeight));
    //m_broadphase->cleanProxyFromPairs(m_meshBody->getBroadphaseHandle()); // not sure what this does
}

void terrain::renderGLObject()
{
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glColorPointer(3, GL_FLOAT, sizeof(Vertex), m_terrainColors);
    glVertexPointer(3, GL_FLOAT, sizeof(Vertex), m_terrainVerts);
    glNormalPointer(GL_FLOAT, sizeof(Vertex), m_terrainNormals);
    glDrawElements(GL_TRIANGLES, m_terrainTriangleCount*3, GL_UNSIGNED_INT, m_terrainTriangles);

    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    float side = m_terrainSize.x();
    if(side < m_terrainSize.y()) side = m_terrainSize.y();
    side /= 2;
    glColor3f(0.3,0.3,0.3);
    this->drawPlane(side,side);
}

void terrain::createPlane(btVector3 norm, float cons, btVector3 orig)
{
    // create a plane shape
    btCollisionShape* planeShape = new btStaticPlaneShape(norm,cons);
    m_terrainShapes.push_back(planeShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(orig);

    // create a static plane and add it to the world
    m_planeBody = arena->createRigidBody(0,groundTransform,planeShape);
	m_terrainObjects.push_back(m_planeBody);
}

void terrain::drawPlane(float x, float y)
{
    const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(m_planeBody->getCollisionShape());
    btVector3 normal = staticPlaneShape->getPlaneNormal();

   //btVector3 planeOrigin = normal * constant;
    btVector3 planeOrigin = m_planeBody->getCenterOfMassPosition();
    btVector3 vec0,vec1;
    btPlaneSpace1(normal,vec0,vec1);
    btVector3 pt0 = planeOrigin + (vec0*y + vec1*x);
    btVector3 pt1 = planeOrigin - (vec0*y + vec1*x);
    btVector3 pt2 = planeOrigin + (vec0*y - vec1*x);
    btVector3 pt3 = planeOrigin - (vec0*y - vec1*x);

    glNormal3f(normal.x(),normal.y(),normal.z());
    glBegin(GL_QUADS);
    glVertex3f(pt0.getX(),pt0.getY(),pt0.getZ());
    glVertex3f(pt3.getX(),pt3.getY(),pt3.getZ());
    glVertex3f(pt1.getX(),pt1.getY(),pt1.getZ());
    glVertex3f(pt2.getX(),pt2.getY(),pt2.getZ());
    glEnd();
}

void terrain::generateGround()
{
    arena->resetWorld();
    // create a plane below the terrain to catch fallen objects, keep in the world
    this->createPlane(btVector3(0,0,1),0,btVector3(m_terrainSize.x()/2,m_terrainSize.y()/2,SAFETYPLANE));

    btTriangleIndexVertexArray *meshInterface = new btTriangleIndexVertexArray(m_terrainTriangleCount,
                                                                               &(m_terrainTriangles[0].v1),
                                                                               sizeof(Triangle),
                                                                               m_terrainVertexCount,
                                                                               &(m_terrainVerts[0].x),
                                                                               sizeof(Vertex));
    // create a mesh shape using the data from the BMP file
    m_triMesh = new btBvhTriangleMeshShape(meshInterface, true);

    // set the position of the terrain
    btTransform terrainTransform;
    terrainTransform.setIdentity();
    terrainTransform.setOrigin(btVector3(0,0,0));

    // create the terrain rigid body and add it to the world
    m_meshBody = arena->createRigidBody(0,terrainTransform,m_triMesh);
	m_terrainObjects.push_back(m_meshBody);
    // set the collision flags
    m_meshBody->setCollisionFlags(m_meshBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
}

/////////////////////////////////////////
// Terrain file slots
/////////////
void terrain::openTerrain(QString filename)
{
	if(filename == NULL){
	// open an Open File dialog to look for a PNG image to represent a height map
    	filename = QFileDialog::getOpenFileName(m_view->parentWidget(),tr("Open Terrain"), tr("/Users"),tr("Image File (*.png)"));
		if(filename == NULL) return; // if cancel is pressed dont do anything
	}
	
	QFileInfo terrainInfo(filename);
	this->terrainClear();
	m_terrainFilename = filename;
	m_terrainSize = tTool->getSize();										// set the terrain size, loading a file will overwrite it
	
	if(filename != NULL && filename != "NULL" && terrainLoadFile()) 		// image data is good create terrain
	{
		m_terrainShortname = terrainInfo.baseName();
		this->generateGround();
	}
    else {																	// no image file, create flat terrain
		m_terrainShortname = "NULL";
		m_terrainFilename = "NULL";
		
        this->terrainCreateMesh(NULL);
        this->generateGround();
    }

	m_view->printText("Terrain Loaded: " + terrainInfo.baseName());
	
	m_terrainModified = false;
	emit newTerrain();
}

void terrain::saveTerrain()
{
	// open a Save File dialog and select location and filename
	QString filename = QFileDialog::getSaveFileName(m_view->parentWidget(),tr("Save Terrain PNG"), QDir::homePath(),tr("Image File (*.png)"));
	if(filename == NULL) return; // if cancel is pressed dont do anything

    if(!filename.endsWith(".png")) filename.append(".png");

	QFileInfo terrainInfo(filename);
	m_terrainShortname = terrainInfo.baseName();
    m_terrainFilename = filename;
    QImage modTerrain(m_pixelSize,QImage::Format_ARGB32);

    for(int j=0;j<m_pixelSize.height();j++){
        for(int i=0;i<m_pixelSize.width();i++){
            int k = i+j*m_pixelSize.width();
            int value = (m_terrainVerts[k].z - m_terrainMinHeight)/(m_terrainMaxHeight - m_terrainMinHeight) * 255;	// Minimum height should be negative
            modTerrain.setPixel(i,j, qRgba(value,value,value,255));													// set the color of the pixel
        }
    }
    modTerrain.setText("sizex",QString::number(m_terrainSize.x()));
    modTerrain.setText("sizey",QString::number(m_terrainSize.y()));
	modTerrain.setText("sizez",QString::number(m_terrainMaxHeight - m_terrainMinHeight));
    //modTerrain.setText("sizez",QString::number(m_terrainSize.z()));
    modTerrain.save(m_terrainFilename,"png");
	m_terrainModified = false;
}

void terrain::setTerrainSize(btVector3 size)
{
	if(size == m_terrainSize) return;
	
	btVector3 modscale = (size - m_terrainSize) / m_terrainSize;	// the difference between the new and old size divided by the old size
	
	if(m_terrainSize.z() == 0) modscale.setZ(size.z());				// incase of NAN
	
	m_terrainMinHeight = m_terrainMaxHeight = 0;
    for(int i=0; i<m_terrainVertexCount; i++){
        m_terrainVerts[i].x += modscale.x() * m_terrainVerts[i].x;
        m_terrainVerts[i].y += modscale.y() * m_terrainVerts[i].y;
        
		if(m_terrainSize.z() == 0)
			m_terrainVerts[i].z = size.z();
		else
			m_terrainVerts[i].z += modscale.z() * m_terrainVerts[i].z;

		if(m_terrainVerts[i].z > m_terrainMaxHeight)
			m_terrainMaxHeight = m_terrainVerts[i].z;
		else if(m_terrainVerts[i].z < m_terrainMinHeight)
			m_terrainMinHeight = m_terrainVerts[i].z;
    }
    buildNormals();

	m_terrainSize = size;
	tTool->setSize(m_terrainSize);
	
	m_view->printText(QString("Terrain Resized %1,%2,%3").arg(m_terrainSize.x()).arg(m_terrainSize.y()).arg(m_terrainSize.z()));

    this->terrainRefresh();
	emit newTerrain();
}

void terrain::flattenTerrain()
{
	this->terrainClear();
	this->openTerrain("NULL");
	emit newTerrain();
}


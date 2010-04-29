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
m_terrainMaxHeight(3),
m_terrainMinHeight(0),
m_pixelx(100),
m_pixely(100)
{
    arena = physicsWorld::instance(); // get the physics world object

	this->openTerrain(filename);
}

terrain::~terrain()
{
    qDebug("deleting terrain");
    terrainClear();
}

// deletes the terrain vertices, colors, normals and triangle indices
// be sure to stop drawing and simulation calculations when doing this
void terrain::terrainClear()
{
    arena->deleteGroup(TERRAIN_GROUP);

    m_terrainShapes.clear();

    if(m_terrainVerts) { delete m_terrainVerts; }
    m_terrainVerts = 0;
    if(m_terrainColors) { delete m_terrainColors; }
    m_terrainColors = 0;
    if(m_terrainNormals) { delete m_terrainNormals; }
    m_terrainNormals = 0;
    if(m_terrainTriangles) { delete m_terrainTriangles; }
    m_terrainTriangles = 0;
}

// loads a BMP file as the height map for the terrain. If the data
// from filename is valid the world is reset and new terrain is created.
int terrain::terrainLoadFile()
{
    unsigned int status = 1;
	
    QImage heightMap(m_terrainFilename);
    if(heightMap.isNull()){
        qDebug("Terrain File could not open");
        return 0; // if the file does not open return
    }

    QStringList keyList = heightMap.textKeys();
     if(keyList.contains("scalex")) m_terrainScale.setX(heightMap.text("scalex").toFloat());
     if(keyList.contains("scaley")) m_terrainScale.setY(heightMap.text("scaley").toFloat());
     if(keyList.contains("scalez")) m_terrainScale.setZ(heightMap.text("scalez").toFloat());

    m_pixelx = heightMap.width();
    m_pixely = heightMap.height();
    // get the number of pixels
    int imgSize = m_pixelx*m_pixely;

    // create a new array to hold all the height data from the image
    unsigned int *imgData = new unsigned int[imgSize+1];
    if(imgData == NULL) {
        qDebug("Memory error");
        return 0;
    }
    unsigned int tHeight;
    imgData[imgSize] = 0; // the last element in the array contains the maximum height
	
    // load all the height data from the gray scale part of the height image
    for(int j=0;j<m_pixely;j++){
        for(int i=0;i<m_pixelx;i++){
            int k = i + j*m_pixelx;
            tHeight = qGray(heightMap.pixel(i,j));
            imgData[k] = tHeight;
            if(tHeight > imgData[imgSize]) imgData[imgSize] = tHeight;
        }
    }

    // if data is ok
    if(status) terrainCreateMesh(imgData); // create the new terrain mesh
	
    delete imgData;
    return status;
}

// creates a new triangle based terrain mesh, be sure heightData is large enough
// to fit the world which must be set prior. This function fills the proper arrays
// for both OpenGL and Bullet.
void terrain::terrainCreateMesh(unsigned int *heightData)
{
    int i,j,k;
    float temp;
    int xWorld = m_pixelx;
    int yWorld = m_pixely;
	
    m_terrainVertexCount = xWorld * yWorld;   // calculate the number of verticies
    m_terrainTriangleCount = (xWorld-1) * (yWorld-1)*2; // calculate the number of triangles
	
    // allocate new data arrays
    m_terrainVerts = new Vertex[m_terrainVertexCount];
    m_terrainColors = new Vertex[m_terrainVertexCount];
    m_terrainNormals = new Vertex[m_terrainVertexCount];
    m_terrainTriangles = new Triangle[m_terrainTriangleCount];
    m_terrainMaxHeight = m_terrainMinHeight = 0;
	
    // generate the vertices for the triangle mesh
    // This sets the proper coordinates for the vertices that
    // will make up the terrain triangle mesh. The color of
    // each vertex is based on the height of from the BMP
    for(i=0;i<m_terrainVertexCount;i++)
    {
        m_terrainVerts[i].x = (i % xWorld) * m_terrainScale.x();
        m_terrainVerts[i].y = (i / xWorld) * m_terrainScale.y();
        if(heightData != NULL) temp = (float)heightData[i]/(float)heightData[m_terrainVertexCount]; // get the height from the image data
		else temp = 0.5;
        m_terrainVerts[i].z = temp * m_terrainScale.z();  // scale the height
        if(m_terrainVerts[i].z > m_terrainMaxHeight) m_terrainMaxHeight = m_terrainVerts[i].z;
		
        // set the color based on the terrain height
        m_terrainColors[i].x = 0.7 * temp;
        m_terrainColors[i].y = 0.7 * temp;
        m_terrainColors[i].z = 0.7 * temp;
    }
	
    // generate the indices which point to the vertices of the triangle mesh
    // the m_terrainTriangle array is used to tell OpenGL and the Bullet
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
            m_terrainTriangles[k].v2 = ind2;
            m_terrainTriangles[k].v3 = ind3;
            // high triangle
            m_terrainTriangles[k+1].v1 = ind2;
            m_terrainTriangles[k+1].v2 = ind4;
            m_terrainTriangles[k+1].v3 = ind3;
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
	
    for(j=0;j<m_pixely;j++)
    {
        for(i=0;i<m_pixelx;i++)
        {
            int v0 = j*m_pixelx + i;
            int v1 = (j-1)*m_pixelx + i;
            int v2 = j*m_pixelx + (i+1);
            int v3 = (j+1)*m_pixelx + i;
            int v4 = j*m_pixelx + (i-1);
			
            if(j-1 < 0){ v1 = -1; }
            if(i+1 == m_pixelx) { v2 = -1; }
            if(j+1 == m_pixely) { v3 = -1;}
            if(i-1 < 0) { v4 = -1;}
			
            m_terrainNormals[v0] = sumNormals(v0,v1,v2,v3,v4,m_terrainVerts);
            //qDebug("x:%f y:%f z:%f",m_terrainNormals[v0].x,m_terrainNormals[v0].y,m_terrainNormals[v0].z);
        }
    }
}

void terrain::terrainRaise(btVector3 dir, float amount, float area)
{
    if(area == 0) return;
    float xp,yp;
    if(dir.x() < 0) xp = 0;
    else if(dir.x() > m_worldSize.x()) xp = m_worldSize.x();
    else xp = dir.x();
	
    if(dir.y() < 0) yp = 0;
    else if(dir.y() > m_worldSize.y()) yp = m_worldSize.y();
    else yp = dir.y();
	
    for(int i=0; i<m_terrainVertexCount; i++)
	{
                float x = (i % m_pixelx) * m_terrainScale.x();
                float y = (i / m_pixelx) * m_terrainScale.y();

		float xd = xp - x;
		float yd = yp - y;

		float d = (float)sqrt((xd*xd) + (yd*yd));
		
                float a = (m_terrainVerts[i].z + amount) - (amount*(d / area));
		
		if (a > m_terrainVerts[i].z) {
			m_terrainVerts[i].z = a;
			
			m_terrainColors[i].x = 0.7 * a / m_terrainMaxHeight;
			m_terrainColors[i].y = 0.7 * a / m_terrainMaxHeight;
			m_terrainColors[i].z = 0.6 * a / m_terrainMaxHeight;
			if(a > m_terrainMaxHeight) m_terrainMaxHeight = a;
		}
        }
        buildNormals();

        this->terrainRefresh();
}

void terrain::terrainLower(btVector3 dir, float amount, float area)
{
    if(area == 0) return;
    float xp,yp;
    if(dir.x() < 0) xp = 0;
    else if(dir.x() > m_worldSize.x()) xp = m_worldSize.x();
    else xp = dir.x();
	
    if(dir.y() < 0) yp = 0;
    else if(dir.y() > m_worldSize.y()) yp = m_worldSize.y();
    else yp = dir.y();
	
    for(int i=0; i<m_terrainVertexCount; i++)
	{
                float x = (i % m_pixelx) * m_terrainScale.x();
                float y = (i / m_pixelx) * m_terrainScale.y();
		
		float xd = xp - x;
		float yd = yp - y;
		float d = (float)sqrt((xd*xd) + (yd*yd));
		
                float a = (m_terrainVerts[i].z - amount) + (amount*(d / area));
		
		if (a < m_terrainVerts[i].z) {
			m_terrainVerts[i].z = a;
			
			m_terrainColors[i].x = 0.7 * a / m_terrainMaxHeight;
			m_terrainColors[i].y = 0.7 * a / m_terrainMaxHeight;
			m_terrainColors[i].z = 0.6 * a / m_terrainMaxHeight;
			if(a < m_terrainMinHeight) m_terrainMinHeight = a;
		}
	}
	buildNormals();

        this->terrainRefresh();
}

float terrain::terrainHeightAt(btVector3 pt)
{
	float avgHeight;
    int index = pt.x() + pt.y()*m_worldSize.x();
	if(pt.x() < 0) index = pt.y()*m_worldSize.x() + 1;			// return height of point on left edge of terrain
	if(pt.x() > m_worldSize.x()) index = pt.y()*m_worldSize.x();// return height of point on right edge of terrain
	if(pt.y() < 0) index = pt.x();								// return height of point on bottom edge of terrain
	if(pt.y() > m_worldSize.y()) index = pt.x() + (m_worldSize.y()-1)*m_worldSize.x(); // return height of point on top edge
    
    avgHeight = m_terrainVerts[index].z;
    avgHeight += m_terrainVerts[index+1].z;
    avgHeight += m_terrainVerts[index+(int)m_worldSize.x()].z;
    avgHeight += m_terrainVerts[index+1+(int)m_worldSize.x()].z;

    return avgHeight/4;
}

void terrain::terrainRescale(btVector3 scale)
{
    m_worldSize *= scale;
    m_terrainMaxHeight *= scale.z();
    m_terrainMinHeight *= scale.z();

    for(int i=0;i<m_terrainVertexCount;i++){
        m_terrainVerts[i].x *= scale.x();
        m_terrainVerts[i].y *= scale.y();
        m_terrainVerts[i].z *= scale.z();
    }
    buildNormals();

	m_terrainScale.setX(m_terrainScale.x() * scale.x());
	m_terrainScale.setY(m_terrainScale.y() * scale.y());
	m_terrainScale.setZ(m_terrainScale.z() * scale.z());

    arena->setWorldSize(m_worldSize);

    this->terrainRefresh();
}

// if the terrain mesh is edited it must be refit to the physics world
void terrain::terrainRefresh()
{
    arena->getDynamicsWorld()->updateAabbs();
    m_triMesh->refitTree(btVector3(0,0,m_terrainMinHeight),btVector3(m_worldSize.x(),m_worldSize.y(),m_terrainMaxHeight));
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

    float side = m_worldSize.x();
    if(side < m_worldSize.y()) side = m_worldSize.y();
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
    m_planeBody = arena->createRigidBody(0,groundTransform,planeShape,TERRAIN_GROUP);
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
    m_worldSize.setValue(m_pixelx,m_pixely,m_terrainMaxHeight);
    m_worldSize *= m_terrainScale;
    arena->setWorldSize(m_worldSize);
    arena->resetWorld();
    // create a plane below the terrain to catch fallen objects, keep in the world
    this->createPlane(btVector3(0,0,1),0,btVector3(m_worldSize.x()/2,m_worldSize.y()/2,SAFETYPLANE));

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
    m_meshBody = arena->createRigidBody(0,terrainTransform,m_triMesh,TERRAIN_GROUP);
    // set the collision flags
    m_meshBody->setCollisionFlags(m_meshBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
}

void terrain::terrainFlatten()
{
	this->terrainClear();
	this->openTerrain(NULL);
}

void terrain::openTerrain(QString filename)
{
	this->terrainClear();
	m_terrainFilename = filename;
	m_terrainScale.setValue(1,1,1);
	
	if(filename != NULL && terrainLoadFile()) 
		this->generateGround();  // if the image data is good create terrain
    else {
        this->terrainCreateMesh(NULL);
        this->generateGround();
    }
}

void terrain::saveTerrain(QString filename)
{
    m_terrainFilename = filename;
    QImage modTerrain(m_pixelx,m_pixely,QImage::Format_ARGB32);

    for(int j=0;j<m_pixely;j++){
        for(int i=0;i<m_pixelx;i++){
            int k = i+j*m_pixelx;
            int value = (m_terrainVerts[k].z - m_terrainMinHeight)/(m_terrainMaxHeight - m_terrainMinHeight) * 255;
            modTerrain.setPixel(i,j, qRgba(value,value,value,255));
        }
    }
    modTerrain.setText("scalex",QString::number(m_terrainScale.x()));
    modTerrain.setText("scaley",QString::number(m_terrainScale.y()));
    modTerrain.setText("scalez",QString::number(m_terrainScale.z()));
    modTerrain.save(m_terrainFilename,"png");
}

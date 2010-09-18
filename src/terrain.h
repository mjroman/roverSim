#ifndef TERRAIN_H
#define TERRAIN_H

#include <OpenGL/gl.h>
#include "simglobject.h"
#include "physicsWorld.h"
#include "tools/terraintool.h"
#include "utility/definitions.h"
#include <LinearMath/btVector3.h>

class	btRigidBody;
class   btBvhTriangleMeshShape;

class terrain : public simGLObject
{
	Q_OBJECT
public:
    terrain(QString filename, simGLView* glView = NULL);
    ~terrain();

	void showTool() { tTool->show(); }
	void hideTool() { tTool->hide(); }
	
	btVector3 terrainSize() { return m_terrainSize; }
    QString terrainFilename() { return m_terrainFilename; }
	QString terrainShortname() { return m_terrainShortname; }
    float maxHeight() { return m_terrainMaxHeight; }
    float minHeight() { return m_terrainMinHeight; }

	void terrainEdit(btVector3 here, int dir);
	bool terrainBeenModified() { return m_terrainModified; }
    float terrainHeightAt(btVector3 pt);
    void renderGLObject();

public slots:
	void openTerrain(QString filename = NULL);
    void saveTerrain();
	void setTerrainSize(btVector3 size);
	void flattenTerrain();

signals:
	void newTerrain();

protected:
    btAlignedObjectArray<btCollisionShape*>     m_terrainShapes;
	btAlignedObjectArray<btCollisionObject*>	m_terrainObjects;

private:
	terrainTool     		*tTool;
    physicsWorld            *arena;

    Vertex          		*m_terrainVerts;
    Vertex          		*m_terrainColors;
    Vertex          		*m_terrainNormals;
    Triangle        		*m_terrainTriangles;
    int             		m_terrainTriangleCount;
    int             		m_terrainVertexCount;
    float           		m_terrainMaxHeight;
    float           		m_terrainMinHeight;
    btVector3       		m_terrainSize;
    QSize             		m_pixelSize;
    QString        			m_terrainFilename;
	QString					m_terrainShortname;
	bool					m_terrainModified;

    btRigidBody             *m_planeBody;
    btRigidBody             *m_meshBody;
    btBvhTriangleMeshShape  *m_triMesh;

    void terrainCreateMesh(unsigned int *heightData);
    int terrainLoadFile();
    void terrainClear();
    void terrainRefresh();
    void buildNormals();
    void createPlane(btVector3 norm, float cons, btVector3 orig);
    void drawPlane(float x,float y);
    void generateGround();
};

#endif // TERRAIN_H

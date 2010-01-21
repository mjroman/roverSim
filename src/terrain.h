#ifndef TERRAIN_H
#define TERRAIN_H

#include <QObject>
#include <OpenGL/gl.h>
#include "simglobject.h"
#include "physicsWorld.h"
#include "utility/definitions.h"
#include <LinearMath/btVector3.h>

class	btRigidBody;
class   btBvhTriangleMeshShape;

class terrain : public QObject, public simGLObject
{
private:
    Vertex          *m_terrainVerts;
    Vertex          *m_terrainColors;
    Vertex          *m_terrainNormals;
    Triangle        *m_terrainTriangles;
    int             m_terrainTriangleCount;
    int             m_terrainVertexCount;
    btVector3       m_terrainScale;
    float           m_terrainMaxHeight;
    float           m_terrainMinHeight;
    btVector3       m_worldSize;
    int             m_pixelx,m_pixely;
    QString         m_terrainFilename;

    btRigidBody             *m_planeBody;
    btRigidBody             *m_meshBody;
    btBvhTriangleMeshShape  *m_triMesh;
    physicsWorld            *arena;

    void terrainCreateMesh(unsigned int *heightData);
    int terrainLoadFile();
    void terrainClear();
    void terrainRefresh();
    void buildNormals();
    void createPlane(btVector3 norm, float cons, btVector3 orig);
    void drawPlane(float x,float y);
    void generateGround();

protected:
    btAlignedObjectArray<btCollisionShape*>     m_terrainShapes;

public:
    terrain(QString filename, simGLView* glView = 0);
    ~terrain();

    btVector3 terrainScale() { return m_terrainScale; }
    QString terrainFileName() { return m_terrainFilename; }
    float maxHeight() { return m_terrainMaxHeight; }
    float minHeight() { return m_terrainMinHeight; }

    void terrainRescale(btVector3 scale);
    void terrainRaise(btVector3 dir, float amount, float area);
    void terrainLower(btVector3 dir, float amount, float area);
    float terrainHeightAt(btVector3 pt);
    void renderGLObject();
    void saveTerrain(QString filename);
};

#endif // TERRAIN_H

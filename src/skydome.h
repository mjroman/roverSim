#ifndef SKYDOME_H
#define SKYDOME_H

#include <QObject>
#include <OpenGL/gl.h>

#include "simglobject.h"
#include "utility/definitions.h"

class skydome : public QObject, public simGLObject
{
private:
    Vertex          *m_domeVerts;
    Point           *m_domeTextPoints;
    int             m_domeVertexCount;
    float           m_domeRadius;
    float           m_delLat;
    float           m_delLon;

    void buildDome();
    void createTextCords();

public:
    skydome(simGLView* glView = 0);
    ~skydome();

    void renderGLObject();
};

#endif // SKYDOME_H

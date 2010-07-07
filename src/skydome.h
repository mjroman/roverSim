#ifndef SKYDOME_H
#define SKYDOME_H

#include <OpenGL/gl.h>

#include "simglobject.h"
#include "utility/definitions.h"

class skydome : public simGLObject
{
private:
    Vertex          *m_domeVerts;
    Point           *m_domeTextPoints;
    int             m_domeVertexCount;
	GLuint			m_texture;
    float           m_domeRadius;
    float           m_delLat;
    float           m_delLon;

    void buildDome();
    void createTextCords();

public:
    skydome(simGLView* glView = NULL);
    ~skydome();

    void renderGLObject();
};

#endif // SKYDOME_H

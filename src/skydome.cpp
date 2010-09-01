#include "skydome.h"

skydome::skydome(simGLView* glView)
    :
    simGLObject(glView),
    m_domeRadius(800.0),
    m_delLat(15),
    m_delLon(15)
{
	if(glView){
		this->buildDome();
		this->createTextCords();
	}
}

skydome::~skydome()
{
    if(m_domeVerts) delete m_domeVerts;
    m_domeVerts = 0;
    if(m_domeTextPoints) delete m_domeTextPoints;
    m_domeTextPoints = 0;
}

// generates the vertex coordinates for a dome
void skydome::buildDome()
{
    int lat,lon,n=0;
    m_domeVertexCount = (int)((360/m_delLon)*(90/m_delLat)*4);

    m_domeVerts = new Vertex[m_domeVertexCount];

    for(lat=0;lat <= 90-m_delLat;lat+=m_delLat){
        for(lon=0;lon <= 360-m_delLon;lon+=m_delLon){
            m_domeVerts[n].x = m_domeRadius * sin(DEGTORAD(lat)) * cos(DEGTORAD(lon));
            m_domeVerts[n].y = m_domeRadius * sin(DEGTORAD(lat)) * sin(DEGTORAD(lon));
            m_domeVerts[n].z = m_domeRadius * cos(DEGTORAD(lat));
            n++;

            m_domeVerts[n].x = m_domeRadius * sin(DEGTORAD(lat+m_delLat)) * cos(DEGTORAD(lon));
            m_domeVerts[n].y = m_domeRadius * sin(DEGTORAD(lat+m_delLat)) * sin(DEGTORAD(lon));
            m_domeVerts[n].z = m_domeRadius * cos(DEGTORAD(lat+m_delLat));
            n++;

            m_domeVerts[n].x = m_domeRadius * sin(DEGTORAD(lat)) * cos(DEGTORAD(lon+m_delLon));
            m_domeVerts[n].y = m_domeRadius * sin(DEGTORAD(lat)) * sin(DEGTORAD(lon+m_delLon));
            m_domeVerts[n].z = m_domeRadius * cos(DEGTORAD(lat));
            n++;

            if(lat > -90 && lat < 90){
                m_domeVerts[n].x = m_domeRadius * sin(DEGTORAD(lat+m_delLat)) * cos(DEGTORAD(lon+m_delLon));
                m_domeVerts[n].y = m_domeRadius * sin(DEGTORAD(lat+m_delLat)) * sin(DEGTORAD(lon+m_delLon));
                m_domeVerts[n].z = m_domeRadius * cos(DEGTORAD(lat+m_delLat));
                n++;
            }
        }
    }
}

// creates the points used for mapping the texture to the surface
// the U and V coordinates are spherical coordinates
// hTile and vTile are for tiled maps.
void skydome::createTextCords()
{
    int i;
    float vx,vy,vz,mag;
    float hTile,vTile;

    hTile = 1;
    vTile = 1;

    m_domeTextPoints = new Point[m_domeVertexCount];

    for(i=0; i < m_domeVertexCount; i++){
        vx = m_domeVerts[i].x;
        vy = m_domeVerts[i].y;
        vz = m_domeVerts[i].z;

        mag = (float)sqrt(SQ(vx)+SQ(vy)+SQ(vz));
        vx /= mag;
        vy /= mag;
        vz /= mag;
		
		// all texture coordinates must be between 0 and 1
		// horizontal texture point is from 0 to 2PI, atan2 returns -PI to PI
		m_domeTextPoints[i].u = (hTile * (float)(atan2(vx, vy)/PI) + 1)*0.5;
		// vertical texture point is from 0 to HALFPI, since it is only the top half of a sphere
		m_domeTextPoints[i].v = vTile * (float)(asin(vz))/HALFPI;
		
		//m_domeTextPoints[i].u = hTile * (float)(atan2(vx, vy)/(PI*2)) + 0.5f;
		//m_domeTextPoints[i].v = vTile * (float)(asin(vz) / PI) + 0.5f;
		//m_domeTextPoints[i].v = vTile * (float)(asin(vz) / PI)/0.75f + 0.25f;
        //qDebug("%d (%f,%f,%f) : %f,%f",i,m_domeVerts[i].x,m_domeVerts[i].y,m_domeVerts[i].z,m_domeTextPoints[i].u,m_domeTextPoints[i].v);
    }

    for(i=0; i < m_domeVertexCount - 2; i++){
        if (m_domeTextPoints[i].u - m_domeTextPoints[i+1].u > 0.9f)
            m_domeTextPoints[i+1].u += 1.0f;
        if (m_domeTextPoints[i+1].u - m_domeTextPoints[i].u > 0.9f)
            m_domeTextPoints[i].u += 1.0f;
        if (m_domeTextPoints[i].u - m_domeTextPoints[i+2].u > 0.9f)
            m_domeTextPoints[i+2].u += 1.0f;
        if (m_domeTextPoints[i+2].u - m_domeTextPoints[i].u > 0.9f)
            m_domeTextPoints[i].u += 1.0f;
        if (m_domeTextPoints[i+1].u - m_domeTextPoints[i+2].u > 0.9f)
            m_domeTextPoints[i+2].u += 1.0f;
        if (m_domeTextPoints[i+2].u - m_domeTextPoints[i+1].u > 0.9f)
            m_domeTextPoints[i+1].u += 1.0f;

        if (m_domeTextPoints[i].v - m_domeTextPoints[i+1].v > 0.8f)
            m_domeTextPoints[i+1].v += 1.0f;
        if (m_domeTextPoints[i+1].v - m_domeTextPoints[i].v > 0.8f)
            m_domeTextPoints[i].v += 1.0f;
        if (m_domeTextPoints[i].v - m_domeTextPoints[i+2].v > 0.8f)
            m_domeTextPoints[i+2].v += 1.0f;
        if (m_domeTextPoints[i+2].v - m_domeTextPoints[i].v > 0.8f)
            m_domeTextPoints[i].v += 1.0f;
        if (m_domeTextPoints[i+1].v - m_domeTextPoints[i+2].v > 0.8f)
            m_domeTextPoints[i+2].v += 1.0f;
        if (m_domeTextPoints[i+2].v - m_domeTextPoints[i+1].v > 0.8f)
            m_domeTextPoints[i+1].v += 1.0f;
    }
}

void skydome::renderGLObject()
{
    int i;
    glDisable(GL_LIGHTING);
	glCullFace(GL_FRONT);
    glEnable(GL_TEXTURE_2D);
    glColor3f(1.0f, 1.0f, 1.0f);
	//glColor3f(0.f,0.93f,0.93f);

	glPushMatrix();
	btVector3 pos = m_view->getCameraPosition();
	glTranslatef(pos.x(),pos.y(),-20);
	glBindTexture(GL_TEXTURE_2D, m_view->getTexture(0));
    glBegin(GL_TRIANGLE_STRIP);
    for (i=0; i < m_domeVertexCount; i++){
        glTexCoord2f(m_domeTextPoints[i].u, m_domeTextPoints[i].v);
        glVertex3f(m_domeVerts[i].x, m_domeVerts[i].y, m_domeVerts[i].z);
    }
    glEnd();
	glPopMatrix();
    glDisable(GL_TEXTURE_2D);
	glCullFace(GL_BACK);
    glEnable(GL_LIGHTING);
}

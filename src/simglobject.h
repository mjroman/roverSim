#ifndef SIMGLOBJECT_H
#define SIMGLOBJECT_H

#include "simGLView.h";

class simGLObject : public QObject
{
protected:
    simGLView      *m_view;

public:
    simGLObject(simGLView *view=NULL):m_view(view) {
        if(m_view) m_view->registerGLObject(this);
    }
    virtual ~simGLObject() {
        if(m_view) m_view->unregisterGLObject(this);
    }
    virtual void renderGLObject(){};
};
#endif // SIMGLOBJECT_H

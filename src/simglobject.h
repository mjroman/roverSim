#ifndef SIMGLOBJECT_H
#define SIMGLOBJECT_H

#include "simGLView.h";

class simGLObject
{
protected:
    simGLView      *m_view;

public:
    simGLObject(simGLView *view):m_view(view) {
        m_view->registerGLObject(this);
    }
    virtual ~simGLObject() {
        m_view->unregisterGLObject(this);
    }
    virtual void renderGLObject(){};
};
#endif // SIMGLOBJECT_H

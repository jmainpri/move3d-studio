#ifndef QTOPENGLVIEWER_H
#define QTOPENGLVIEWER_H

#ifdef MOVE3D_CORE
#include "qtLibrary.hpp"
#endif

namespace Ui {
    class qtOpenGLViewer;
}

class qtOpenGLViewer : public QWidget {
    Q_OBJECT
public:
    qtOpenGLViewer(QWidget *parent = 0);
    ~qtOpenGLViewer();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::qtOpenGLViewer *m_ui;
};

#endif // QTOPENGLVIEWER_H

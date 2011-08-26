#ifndef MAINWINDOWREMOTE_HPP
#define MAINWINDOWREMOTE_HPP

class GLWidget;

#include "qtLibrary.hpp"



namespace Ui
{
        class MainWindowRemote;
}

/**
 * @ingroup qtMainWindowRemote
 * @brief Qt Main Window container
 * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
 \image html Designer.png
 */
class MainWindowRemote : public QMainWindow
{
        Q_OBJECT

public:
        MainWindowRemote(QWidget *parent = 0);
        ~MainWindowRemote();

        GLWidget*		getOpenGL();

public slots:
    void drawAllWinActive();

private slots:
        void restoreView();

protected:
        void keyPressEvent(QKeyEvent *e);
        void keyReleaseEvent(QKeyEvent *e);

private:
        Ui::MainWindowRemote*	m_ui;

};

// Global MainWindowRemote Pointer
//extern MainWindowRemote* global_w;

#endif // MAINWINDOWREMOTE_HPP

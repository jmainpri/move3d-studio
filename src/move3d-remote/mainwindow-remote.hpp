#ifndef MAINWINDOWREMOTE_HPP
#define MAINWINDOWREMOTE_HPP

class GLWidget;
class FetchEnvironment;

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
        void setSparkRefresh();
        void sparkSaveScenario();
        void myLoop();

protected:
        void keyPressEvent(QKeyEvent *e);
        void keyReleaseEvent(QKeyEvent *e);

private:
        Ui::MainWindowRemote*	m_ui;
        FetchEnvironment* m_posterHandler;
};

#endif // MAINWINDOWREMOTE_HPP

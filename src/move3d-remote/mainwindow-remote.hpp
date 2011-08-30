#ifndef MAINWINDOWREMOTE_HPP
#define MAINWINDOWREMOTE_HPP

class GLWidget;
class FetchEnvironment;

#include "qtLibrary.hpp"
#include "p3d_sys.h"
#include "../p3d/env.hpp"
#include "../p3d/ParametersEnv.hpp"

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

        void setBoolGhost(bool value);
        void setBoolBb(bool value);
        void setBoolFloor(bool value);
        void setBoolTiles(bool value);
        void setBoolWalls(bool value);
        void setBoolSmooth(bool value);
        void setBoolShadows(bool value);
        void setBoolFilaire(bool value);
        void setBoolJoints(bool value);
        void setBoolContour(bool value);
        void setBoolEnableLight(bool value);
        void setBoolEnableShaders(bool value);
        void restoreView();
        void setSparkStatus(bool updating);

        void changeLightPosX();
        void changeLightPosY();
        void changeLightPosZ();

protected:
        void keyPressEvent(QKeyEvent *e);
        void keyReleaseEvent(QKeyEvent *e);

private:
        Ui::MainWindowRemote*	m_ui;
        FetchEnvironment* m_posterHandler;
        void initLightSource();
        void connectCheckBoxes();
        /**
         * Function tro create sliders and checkboxes TODO move somwhere else
         */
        void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
};

#endif // MAINWINDOWREMOTE_HPP

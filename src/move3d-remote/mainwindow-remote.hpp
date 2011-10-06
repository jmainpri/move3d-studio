#ifndef MAINWINDOWREMOTE_HPP
#define MAINWINDOWREMOTE_HPP

class GLWidget;
class PosterReader;

#include "qtLibrary.hpp"
#include <QSettings>
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
        MainWindowRemote(PosterReader *pr, QWidget *parent = 0);
        ~MainWindowRemote();


public slots:

        void setSparkStatusText(bool updating);


signals:
    void drawAllWinActive();
  
private slots:

        void on_switchSparkView_clicked(bool checked);
        void on_pushButtonSaveSettings_clicked();
        void on_pushButtonLoadSettings_clicked();
        void setSparkRefresh();
        void sparkSaveScenario();

        void setRobotAsCurrent();




        void looptest();

        //void on_checkBox_clicked(bool checked);



protected:
        //void keyPressEvent(QKeyEvent *e);
        void keyReleaseEvent(QKeyEvent *e);
        void closeEvent(QCloseEvent *event);

public:
        Ui::MainWindowRemote*	m_ui;

private:
        PosterReader *m_pr;
  

  
        std::vector<QAction*> m_RobotsInMenu;


        void initRobotsMenu();

        bool userReallyWantsToQuit();
        bool userWantsToLoadSettings();
        void initCamera();
        void saveDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget);
        void loadDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget);



        
};

#endif // MAINWINDOWREMOTE_HPP

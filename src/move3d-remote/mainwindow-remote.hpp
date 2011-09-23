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
        MainWindowRemote(QWidget *parent = 0);
        ~MainWindowRemote();
        GLWidget*		getOpenGL();

public slots:
        void drawAllWinActive();

        void setNiutIsAlive(bool state);
        void setNiutColorLabel(int idLabel, int color);
  
private slots:

        void on_switchSparkView_clicked(bool checked);
        void on_pushButtonSaveSettings_clicked();
        void on_pushButtonLoadSettings_clicked();
        void setSparkRefresh();
        void sparkSaveScenario();

        void setRobotAsCurrent();
        void setBoolGhost(bool value);
        void setBoolBb(bool value);
        void setBoolFloor(bool value);
        void setBoolSky(bool value);
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
        void setSparkStatusText(bool updating);

        void changeLightPosX();
        void changeLightPosY();
        void changeLightPosZ();

        void updateImageLeft();
        void updateImageRight();

        void looptest();

        void on_checkBox_clicked(bool checked);

public:
        QLabel *labelImgLeft(){return _labelImageLeft;}
        QLabel *labelImgRight(){return _labelImageRight;}

protected:
        void keyPressEvent(QKeyEvent *e);
        void keyReleaseEvent(QKeyEvent *e);
        void closeEvent(QCloseEvent *event);
public:
        QLabel* _labelImageLeft;
        QLabel* _labelImageRight;
private:
        Ui::MainWindowRemote*	m_ui;

        PosterReader* m_posterHandler;
        QImage* _qimageLeft;
        QImage* _qimageRight;

  
        std::vector<QLabel*> _niutLabels;
        QPixmap _niutPmAlive;
        QPixmap _niutPmDead;
        QPixmap _niutPmRed;
        QPixmap _niutPmOrange;
        QPixmap _niutPmYellow;
        QPixmap _niutPmGreen;
  
        std::vector<QAction*> m_RobotsInMenu;
        uchar *_dataImageLeft;
        uchar *_dataImageRight;
  
        void initLightSource();
        void initRobotsMenu();
        void initNiut();
        bool userReallyWantsToQuit();
        bool userWantsToLoadSettings();
        void initCamera();
        void saveDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget);
        void loadDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget);


        /**
         * Function tro create sliders and checkboxes TODO move somwhere else
         */
        void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
        void connectCheckBoxes();
        
};

#endif // MAINWINDOWREMOTE_HPP

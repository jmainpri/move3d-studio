#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "p3d_sys.h"
#include "../p3d/env.hpp"

#include "qtMainInterface/kcdpropertieswindow.hpp"
#include "qtBase/qt_widgets.hpp"
#include "qtFormRobot/moverobot.hpp"
#include "../p3d/ParametersEnv.hpp"

#ifdef USE_QWT
#include "qtPlot/histoWin.hpp"
#endif

#ifndef GLWIDGET_H
class GLWidget;
#endif

#ifndef MAIN_WINDOW_TEST_FUNCTIONS
class MainWindowTestFunctions;
#endif

#include <vector>
#include <QTimer>

namespace Ui
{
	class MainWindow;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Main Window container
 * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
 \image html Designer.png
 */
class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
	
	/**
	 * Function tro create sliders and checkboxes TODO move somwhere else
	 */
	void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
	void connectCheckBoxToEnv(QCheckBox* box, PlanParam::boolParameter p);
	
	LabeledSlider* createSlider(QString s, Env::intParameter p,int lower, int upper);
	LabeledDoubleSlider* createDoubleSlider(QString s,Env::doubleParameter p, double lower, double upper);

  /* returns the Ui auto genrated structure */
	Ui::MainWindow* Ui() { return m_ui; }
	
	GLWidget*		getOpenGL();
	MoveRobot*	getMoveRobot();
	
public slots:
	void drawAllWinActive();
	void isPlanning();
	void planningFinished();
	
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
	
	void setJointToDraw(int joint);
	
	void setCurrentTraj(p3d_traj* traj);
  void refreshConstraintedDoFs();

signals:
  void runClicked();
  void stopClicked();
  void resetClicked();
	
protected:
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);
	
public slots:
  void loadParametersQuick();
	
private slots:
  void saveInterfaceParameters();
  void loadInterfaceParameters();
  void saveParametersQuick();
	
	void initRobotsMenu();
	void setRobotAsCurrent();
	
	void openScenario();
	void saveScenario();
	
	void loadGraph();
	void saveGraph();
	void saveXYZGraphToDot();
	
	void loadTraj();
	void saveTraj();
	
	void changeLightPosX();
	void changeLightPosY();
	void changeLightPosZ();
	void addTrajToDraw();
	void clearTrajToDraw();
	void colorTrajChange(int color);
        void enableRunAndResetButtons();
        void enableStopButton();
        void enableRunButton();
	void showTraj();
        void showTrace();
	void restoreView();
	void mobileCamera();
        void changeCamera();
	
	void test();
        void saveVideo();
        void saveVideoTimer();
	
	// Global
	//    void setLineEditWithNumber(Env::intParameter p , int val );
	void changeEvent(QEvent *e);
  
signals:
  void selectedPlanner(QString);
	
private:
	
	Ui::MainWindow*							m_ui;
	
	KCDpropertiesWindow*				mKCDpropertiesWindow;
	
	MainWindowTestFunctions*		m_testFunctions;
	
	std::vector<QAction*>				m_RobotsInMenu;
	
	void connectCheckBoxes();

	void initRunButtons();
	void initViewerButtons();
	void initLightSource();
	
	p3d_traj* m_currentTraj;

        QTimer *timer;
        bool isRecording;
};

// Global MainWindow Pointer 
extern MainWindow* global_w;


#endif // MAINWINDOW_H

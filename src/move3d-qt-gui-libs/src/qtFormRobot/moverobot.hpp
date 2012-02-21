#ifndef MOVEROBOT_HPP
#define MOVEROBOT_HPP

#include "qtLibrary.hpp"
#include <QtGui/QStackedLayout>

#if defined( MOVE3D_CORE ) 
#include "API/Device/joint.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "qtBase/SpinBoxSliderConnector_p.hpp"
#ifndef ROBOT_HPP
class Robot;
#endif
#endif

#ifndef GLWIDGET_H
class GLWidget;
#endif

class FormRobot;

namespace Ui {
	class MoveRobot;
}

/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates one DoF slider structure
 */
class DofSlider : public QObject
{
	Q_OBJECT
	
public:
	DofSlider() {}
	
	DofSlider(Robot* R, GLWidget* Gl, FormRobot* FR) : 
	mRobot(R),
	mOpenGl(Gl),
	mFormRobot(FR)
	{}
	
	~DofSlider() {}
  
  /**
   * Get the DoF id in configuration
   */
  int getDofIdInConf() { return mDofNum; } 
	
	/**
	 * Creates a slider with a spinbox and a label
	 */
	void makeSlider(QGridLayout* gridLayout, Joint *jntPt, int DofNumOnJnt);
	
	void setValue(double value) { mConnector->setValue(value); }
	
	QDoubleSpinBox* getDoubleSpinBox() { return mDoubleSpinBox;}
	QSlider* getHorizontalSlider() { return mHorizontalSlider; }
	
	QtShiva::SpinBoxSliderConnector* getConnector() {return mConnector;}
	
	
#if defined( MOVE3D_CORE ) 
	Robot* getRobot() { return mRobot; }
#endif
	
	public slots:
	void dofValueChanged(double value);
  void showDofConstraints();
	
private:
#if defined( MOVE3D_CORE ) 
	Robot*  mRobot;
#endif
	
	int								mDofNum;
	GLWidget*					mOpenGl;
	
	FormRobot*				mFormRobot;
	
	QLabel*						mLabel;
  QPushButton*			mConstraints;
	QDoubleSpinBox*		mDoubleSpinBox;
	QSlider*					mHorizontalSlider;
	
	QtShiva::SpinBoxSliderConnector* mConnector;
};

/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates the sliders structure, Contains a vector of DofSlider
 */
class FormRobot : public QObject {
	
	Q_OBJECT
	
public:
	FormRobot() { mTrajectories.clear(); }
	
	FormRobot(Robot* R, QGridLayout* GL, QComboBox* pos, QComboBox* configs, QComboBox* trajs, GLWidget* openGl) :
	mRobot(R),
	mGridLayout(GL),
	mPositions(pos),
  mConfigNames(configs),
	mTrajectoriesNames(trajs),
	mOpenGl(openGl)
	{}
	
	~FormRobot() {};
	
	/**
	 * Initializes the sliders associated to the Dofs of ptrRob
	 */
	void initSliders();
	
	/**
	 * Sets the associated sliders to the values int ptrConf
	 */
	void setSliders(Configuration& ptrConf);

	
	/**
	 * Returns the robot structure
	 */
	Robot* getRobot() { return mRobot; }
	
	/**
	 *
	 */
	QComboBox* getComboBox() { return mPositions; }
	
	public slots:
  /**
	 * Resets the constrained DoFs
	 */
	void resetConstraintedDoFs();
  
	/**
	 * Sets the current Configuration to Init or Goto
	 */
	void setCurrentPosition(int position);
  void setCurrentConfig(int index);
	
	/**
	 * Save current configuration
	 */
  void saveCurrentConfigToInit();
	void saveCurrentConfigToGoal();
  void saveCurrentConfigToVector();
	
	/**
	 * Add Traj
	 */
	void addTraj( std::string& name, p3d_traj* trajPt );
  void addConfig( std::string& name, configPt q );
  
  void setConfAndTrajFromManipTest();
	
	/**
	 * Function called to set the current trajectory
	 */
	void setCurrentTraj(int id);
  
  /**
   * Shows the current constraints and multi-localpath
   */
  void showConstraints();
#ifdef MULTILOCALPATH
  void showMultiLocalpath();
#endif
	
private:
	
	int calc_real_dof(void);
	
	Robot*										mRobot;
	
	std::vector<DofSlider*>		mSliders;
	QGridLayout*							mGridLayout;
	
	QComboBox*								mPositions;
	
  QComboBox*                mConfigNames;
  std::vector< std::tr1::shared_ptr<Configuration> >		mConfigurations;
  
	QComboBox*								mTrajectoriesNames;
	std::vector<p3d_traj*>		mTrajectories;
	
	GLWidget*									mOpenGl;
};


/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates the FormRobots Stucture, Contains a vector of FormRobot
 */
class MoveRobot : public QWidget 
{
	Q_OBJECT
	
public:
	MoveRobot(QWidget *parent = 0);
	~MoveRobot();
	
	/**
	 * Initilizes all forms
	 */
	void initAllForms(GLWidget* ptrOpenGl);
	
	/**
	 * Updates all robot pos
	 */
	void updateAllRobotInitPos();
	
	/**
	 * Get the robot form by names
	 */
	FormRobot* getRobotFormByName(std::string name);
	
	/**
	 * Sets the constraints
	 */
	void setRobotConstraintedDof(Robot* ptrRob);
  
  /**
   * Refresh all constrainted DoFs
   */
  void refreshConstraintedDoFs();
	
protected:
	void changeEvent(QEvent *e);
	
private:
	Ui::MoveRobot *m_ui;
	
#if defined( MOVE3D_CORE ) 
	/**
	 * Creates a new gridLayout inside a tabWidget
	 */
	//FormRobot* newGridLayoutForRobot(Robot* ptrRob);
	FormRobot* newGridLayoutForRobotStacked(Robot* ptrRob);
	
#endif
	
	/**
	 * Members
	 */
	std::vector<FormRobot*>			mRobots;
	
	// Tab Widget
	QTabWidget*                 mTabWidget;
	
	// Stacked Widgets
	QStackedLayout*							m_StackedLayout;
	QComboBox*									m_pageComboBox;
	
	GLWidget*										mOpenGl;
};

#endif // MOVEROBOT_HPP

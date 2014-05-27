/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
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

    DofSlider( Move3D::Robot* R, GLWidget* Gl, FormRobot* FR) :
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
    void makeSlider(QGridLayout* gridLayout, Move3D::Joint *jntPt, int DofNumOnJnt);

    void setValue(double value) { mConnector->setValue(value); }

    QDoubleSpinBox* getDoubleSpinBox() { return mDoubleSpinBox;}
    QSlider* getHorizontalSlider() { return mHorizontalSlider; }

    QtShiva::SpinBoxSliderConnector* getConnector() {return mConnector;}


#if defined( MOVE3D_CORE ) 
    Move3D::Robot* getRobot() { return mRobot; }
#endif

public slots:
    void dofValueChanged(double value);
    void showDofConstraints();

private:
#if defined( MOVE3D_CORE ) 
    Move3D::Robot*  mRobot;
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
    FormRobot( Move3D::Robot* R, QGridLayout* GL, QComboBox* pos, QComboBox* configs, QComboBox* trajs, GLWidget* openGl);

    ~FormRobot() {}

    /**
     * Initializes the sliders associated to the Dofs of ptrRob
     */
    void initSliders();

    /**
     * Sets the associated sliders to the values int ptrConf
     */
    void setSliders( Move3D::Configuration& ptrConf );


    /**
     * Returns the robot structure
     */
    Move3D::Robot* getRobot() { return mRobot; }

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

    /**
     * Initializes the configurations stored in the robot structure
     */
    void initConfigs();

    /**
   * Add configuration from name
   */
    void addConfig( const std::string& name, configPt q );

    /**
   * Add conf and traj
   */
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

    Move3D::Robot*              mRobot;

    std::vector<DofSlider*>		mSliders;
    QGridLayout*				mGridLayout;

    QComboBox*					mPositions;

    QComboBox*                  mConfigNames;
    std::vector< Move3D::confPtr_t >	mConfigurations;

    QComboBox*					mTrajectoriesNames;
    std::vector<p3d_traj*>		mTrajectories;

    GLWidget*					mOpenGl;
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
    void setRobotConstraintedDof( Move3D::Robot* ptrRob );

    /**
   * Refresh all constrainted DoFs
   */
    void refreshConstraintedDoFs();

protected:
    void changeEvent( QEvent *e );

private:
    Ui::MoveRobot *m_ui;

#if defined( MOVE3D_CORE ) 
    /**
     * Creates a new gridLayout inside a tabWidget
     */
    //FormRobot* newGridLayoutForRobot( Move3D::Robot* ptrRob);
    FormRobot* newGridLayoutForRobotStacked( Move3D::Robot* ptrRob);

#endif

    /**
     * Members
     */
    std::vector<FormRobot*>		mRobots;

    // Tab Widget
    QTabWidget*                 mTabWidget;

    // Stacked Widgets
    QStackedLayout*				m_StackedLayout;
    QComboBox*					m_pageComboBox;

    GLWidget*					mOpenGl;
};

#endif // MOVEROBOT_HPP

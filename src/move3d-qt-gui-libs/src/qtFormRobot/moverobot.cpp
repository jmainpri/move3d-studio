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
#include "moverobot.hpp"
#include "ui_moverobot.h"

#include "qtOpenGL/glwidget.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtLibrary.hpp"
#include "qtConstraints.hpp"

#include <iostream>
#include <iomanip>
#include <tr1/memory>

#include "P3d-pkg.h"
#include "Collision-pkg.h"

#include "sliderfunction.hpp"

#include "API/project.hpp"

#ifdef MULTILOCALPATH
#include "LightPlanner-pkg.h"
#include "qtMultiLocalPath.hpp"
extern ManipulationTestFunctions* global_manipPlanTest;
#endif

using namespace Move3D;
using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

MoveRobot::MoveRobot(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::MoveRobot)
{
    m_ui->setupUi(this);

}

MoveRobot::~MoveRobot()
{
    delete m_ui;
}

/*!
 * This function intialize all Robot Forms
 * and sets them in a Tab widget
 */
void MoveRobot::initAllForms(GLWidget* ptrOpenGl)
{
    mOpenGl= ptrOpenGl;

    Scene* envPt = global_Project->getActiveScene();

    for(unsigned int i=0;i<envPt->getNumberOfRobots();i++)
    {
        //		if(i==0)
        //		{
        //			mTabWidget = new QTabWidget(this);
        //			mTabWidget->setUsesScrollButtons(true);
        //		}

        if(i==0)
        {
            m_pageComboBox = new QComboBox;
            m_StackedLayout = new QStackedLayout;

            m_ui->MainLayout->addWidget( m_pageComboBox );
            m_ui->MainLayout->addLayout( m_StackedLayout );

            connect(m_pageComboBox, SIGNAL(activated(int)), m_StackedLayout, SLOT(setCurrentIndex(int)));
        }

//#if defined( MOVE3D_CORE )
        Move3D::Robot* ptrRob = envPt->getRobot(i);

        FormRobot* form = newGridLayoutForRobotStacked( ptrRob );

        form->initSliders();
        form->resetConstraintedDoFs();

        confPtr_t ptrConf = ptrRob->getInitPos();

        ptrRob->setAndUpdate( *ptrConf );
        form->setSliders( *ptrConf );
        mRobots.push_back( form );
        //cout << "MoveRobot::ptrRob->getP3dRobotStruct()->getNumberOfJoints() = "  << ptrRob->getNumberOfJoints() << endl;
//#endif
#ifdef WITH_XFORMS
        std::string str = "g3d_draw_allwin_active";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
        if(!ENV.getBool(Env::isRunning))
        {
            mOpenGl->updateGL();
        }
#endif
    }
    //m_ui->MainLayout->addLayout(m_StackedLayout);
}

FormRobot* MoveRobot::getRobotFormByName(string name)
{
    for( unsigned int i=0; i<mRobots.size(); i++ )
    {
        if ( mRobots[i]->getRobot()->getName().compare(name) == 0 )
        {
            return mRobots[i];
        }
    }
    cout << "Robot " << name << "doesn't exist" << endl;
    return 0x00;
}

void MoveRobot::setRobotConstraintedDof( Move3D::Robot* ptrRob)
{
    FormRobot* form = getRobotFormByName(ptrRob->getName());
    form->resetConstraintedDoFs();
    form->setSliders(*ptrRob->getCurrentPos());
}

/*!
 * This function intialize a Robot Form from a
 * Robot structure
 */
/**FormRobot* MoveRobot::newGridLayoutForRobot(Robot* ptrRob)
{
    QString robotName(ptrRob->getName().c_str());
    QWidget *tab;
    QVBoxLayout *robotLayoutTab;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QHBoxLayout *horizontalLayoutScrollArea;
    QGridLayout *gridLayout;

    tab = new QWidget();

    robotLayoutTab = new QVBoxLayout(tab);
    scrollArea = new QScrollArea(tab);
    scrollArea->setWidgetResizable(true);
    scrollAreaWidgetContents = new QWidget();
    //scrollAreaWidgetContents->setGeometry(QRect(0, 0, 700, 250));
    horizontalLayoutScrollArea = new QHBoxLayout(scrollAreaWidgetContents);

    gridLayout = new QGridLayout();
    horizontalLayoutScrollArea->addLayout(gridLayout);
    scrollArea->setWidget(scrollAreaWidgetContents);
    robotLayoutTab->addWidget(scrollArea);


    QComboBox* positions = new QComboBox();
    positions->addItem(QString("Start"));
    positions->addItem(QString("Goal"));
    robotLayoutTab->addWidget(positions);

    QPushButton* saveButton = new QPushButton("Save Current");
    robotLayoutTab->addWidget(saveButton);

    mTabWidget->addTab(tab, QString());
    mTabWidget->setTabText(mTabWidget->indexOf(tab), robotName );

    QComboBox* trajectoriesNames = new QComboBox();
    robotLayoutTab->addWidget(trajectoriesNames);

    m_ui->MainLayout->addWidget(mTabWidget);

    FormRobot* formRobot = new FormRobot(ptrRob,gridLayout,positions,trajectoriesNames,mOpenGl);

    connect(positions, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentPosition(int)));
    positions->setCurrentIndex( 0 );

    connect(trajectoriesNames, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentTraj(int)));
    positions->setCurrentIndex( 0 );

    connect(saveButton,SIGNAL(clicked()),formRobot,SLOT(saveCurrentConfigToPosition()));

    return formRobot;
}*/

/*!
 * Sets the Robot Forms to the initial
 * configuration of their associated Robots
 */
void MoveRobot::updateAllRobotInitPos()
{
    for(unsigned int i=0;i<mRobots.size();i++)
    {
        Move3D::Robot* robot = mRobots[i]->getRobot();
        mRobots[i]->setSliders( *robot->getInitPos() );
        mRobots[i]->getComboBox()->setCurrentIndex(0);
        cout << "Set Robot " << robot->getName() << " to its initial position" << endl;
    }
}

void MoveRobot::refreshConstraintedDoFs()
{
    for(int i=0;i<int(mRobots.size());i++)
    {
        mRobots[i]->resetConstraintedDoFs();
    }
}

void MoveRobot::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

/*!
 * This function intialize a Robot Form from a
 * Robot structure
 */
FormRobot* MoveRobot::newGridLayoutForRobotStacked( Move3D::Robot* ptrRob)
{
    QString						robotName(ptrRob->getName().c_str());
    QWidget*					tab;
    QVBoxLayout*			robotLayoutTab;
    QScrollArea*			scrollArea;
    QWidget*					scrollAreaWidgetContents;
    QHBoxLayout*			horizontalLayoutScrollArea;
    QGridLayout*			gridLayout;

    tab = new QWidget();

    robotLayoutTab = new QVBoxLayout(tab);
    robotLayoutTab->setContentsMargins(0,0,0,0);
    scrollArea = new QScrollArea(tab);
    scrollArea->setWidgetResizable(true);
    scrollAreaWidgetContents = new QWidget();
    //scrollAreaWidgetContents->setGeometry(QRect(0, 0, 700, 250));
    horizontalLayoutScrollArea = new QHBoxLayout(scrollAreaWidgetContents);

    gridLayout = new QGridLayout();
    horizontalLayoutScrollArea->addLayout(gridLayout);
    scrollArea->setWidget(scrollAreaWidgetContents);
    robotLayoutTab->addWidget(scrollArea);

    //-----------------------------------------------------
    //-----------------------------------------------------
    // Init and goal configuration

    QHBoxLayout* startAndGoalLayout = new QHBoxLayout();
    startAndGoalLayout->setSpacing(6);
    startAndGoalLayout->setContentsMargins(0,0,0,0);

    QComboBox* positions = new QComboBox();
    positions->addItem(QString("Init"));
    positions->addItem(QString("Goal"));
    startAndGoalLayout->addWidget(positions);

    QPushButton* saveInitButton = new QPushButton("Save (as init)");
    startAndGoalLayout->addWidget(saveInitButton);

    QPushButton* saveGoalButton = new QPushButton("Save (as goal)");
    startAndGoalLayout->addWidget(saveGoalButton);

    QWidget* startAndGoal = new QWidget();
    startAndGoal->setLayout( startAndGoalLayout );
    robotLayoutTab->addWidget( startAndGoal );

    //-----------------------------------------------------
    //-----------------------------------------------------
    // Save current configuration into a vector

    QHBoxLayout* configLayout = new QHBoxLayout();
    configLayout->setSpacing(6);
    configLayout->setContentsMargins(0,0,0,0);

    QLabel* configLabel = new QLabel("Config. : ");
    configLayout->addWidget(configLabel);

    QComboBox* configNames = new QComboBox();
    configLayout->addWidget(configNames);

    QPushButton* saveCurrButton = new QPushButton("Save Curr.");
    configLayout->addWidget(saveCurrButton);

    QPushButton* setConfAndTrajFromManipButton = new QPushButton("Set traj & conf from manip.");
    configLayout->addWidget(setConfAndTrajFromManipButton);

    QWidget* configectoriesWidget = new QWidget();
    configectoriesWidget->setLayout( configLayout );
    robotLayoutTab->addWidget( configectoriesWidget );

    //-----------------------------------------------------
    //-----------------------------------------------------
    // Saved trajectories

    QHBoxLayout* trajLayout = new QHBoxLayout();
    trajLayout->setSpacing(6);
    trajLayout->setContentsMargins(0,0,0,0);

    QLabel* trajLabel = new QLabel("Trajectories : ");
    trajLayout->addWidget(trajLabel);

    QComboBox* trajectoriesNames = new QComboBox();
    trajLayout->addWidget(trajectoriesNames);

    QWidget* trajectoriesWidget = new QWidget();
    trajectoriesWidget->setLayout( trajLayout );
    robotLayoutTab->addWidget( trajectoriesWidget );

    //-----------------------------------------------------
    //-----------------------------------------------------
    // Constraint and Mutli-localpaths

    QHBoxLayout* cntrtsAndLocalpathLayout = new QHBoxLayout();
    cntrtsAndLocalpathLayout->setSpacing(6);
    cntrtsAndLocalpathLayout->setContentsMargins(0,0,0,0);

    QLabel* cntrtsLabel = new QLabel("Constraints And MultiLocapaths : ");
    cntrtsAndLocalpathLayout->addWidget(cntrtsLabel);

    QPushButton* refreshConstraints = new QPushButton("Refresh Cntrts.");
    cntrtsAndLocalpathLayout->addWidget(refreshConstraints);

    QPushButton* showConstraints = new QPushButton("Cntrts.");
    cntrtsAndLocalpathLayout->addWidget(showConstraints);

    QPushButton* showMultiLocalpath = new QPushButton("Multi-LP.");
    cntrtsAndLocalpathLayout->addWidget(showMultiLocalpath);

    QWidget* cntrtsAndLocalpathWidget = new QWidget();
    cntrtsAndLocalpathWidget->setLayout( cntrtsAndLocalpathLayout );

    robotLayoutTab->addWidget( cntrtsAndLocalpathWidget );

    //-----------------------------------------------------
    //-----------------------------------------------------

    m_StackedLayout->addWidget( tab );
    m_pageComboBox->addItem(robotName);

    FormRobot* formRobot = new FormRobot( ptrRob , gridLayout ,
                                          positions , configNames, trajectoriesNames , mOpenGl );

    // Connect the init and goal configuration
    // Combo widget
    connect(positions, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentPosition(int)));
    positions->setCurrentIndex( 0 );

    // Connect the stored configs
    connect(configNames, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentConfig(int)));
    configNames->setCurrentIndex( 0 );

    // Connect the trajectories
    // Combo widget
    connect(trajectoriesNames, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentTraj(int)));
    trajectoriesNames->setCurrentIndex( 0 );

    // Connect the formRobot save current configuration
    // Buttons
    connect(saveInitButton,SIGNAL(clicked()),formRobot,SLOT(saveCurrentConfigToInit()));
    connect(saveGoalButton,SIGNAL(clicked()),formRobot,SLOT(saveCurrentConfigToGoal()));
    connect(saveCurrButton,SIGNAL(clicked()),formRobot,SLOT(saveCurrentConfigToVector()));
    connect(setConfAndTrajFromManipButton,SIGNAL(clicked()),formRobot,SLOT(setConfAndTrajFromManipTest()));

    // Connect the constraints button
    connect(showConstraints,SIGNAL(clicked()),formRobot,SLOT(showConstraints()));
    connect(showMultiLocalpath,SIGNAL(clicked()),formRobot,SLOT(showMultiLocalpath()));
    connect(refreshConstraints,SIGNAL(clicked()),formRobot,SLOT(resetConstraintedDoFs()));

    return formRobot;
}

//---------------------------------------------------------------------
// FormRobot
//---------------------------------------------------------------------

FormRobot::FormRobot( Move3D::Robot* R, QGridLayout* GL, QComboBox* pos, QComboBox* configs, QComboBox* trajs, GLWidget* openGl) :
    mRobot(R),
    mGridLayout(GL),
    mPositions(pos),
    mConfigNames(configs),
    mTrajectoriesNames(trajs),
    mOpenGl(openGl)
{
    p3d_rob* rob = mRobot->getP3dRobotStruct();

    for(int i=0; i<rob->nconf; i++)
    {
        std::string name = rob->conf[i]->name;
        cout << "Add config : " << name << endl;
        addConfig( name, rob->conf[i]->q );
    }
}

/*!
 * Computes the number of dof of the Robot
 */
int FormRobot::calc_real_dof(void)
{
    int nrd;
    int njnt,i,j,k;
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    nrd = robotPt->nb_user_dof;
    njnt = p3d_get_robot_njnt();

    //#ifdef P3D_COLLISION_CHECKING
    //    if(njnt > MAX_NJNTS_IN_ROBOTFORM) {
    //        return 0;
    //    }
    //#endif

    for(i=0; i<=njnt; i++) {
        for(j=0; j<robotPt->joints[i]->dof_equiv_nbr; j++) {
            k = robotPt->joints[i]->index_dof + j;
            if((! p3d_jnt_get_dof_is_user(robotPt->joints[i], j)) &&
                    (robotPt->cntrt_manager->in_cntrt[k] == 1)) {
                nrd++;
            }
        }
    }

    return nrd;
}

/*!
 * Initializes the
 * DofSlider Object (one per dof)
 */
void FormRobot::initSliders()
{
    //    int       i, j, k, ir, ord;
    int k=0;
    //int /*njnt,*/ nb_dof;
    //configPt robot_pos_deg;
    //p3d_rob *robotPt;
    Joint * jntPt;

    //nb_dof =    mRobot->getP3dRobotStruct()->nb_dof; //p3d_get_robot_ndof();
    //ir =        ptrRob->getP3dRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
    //robotPt =   mRobot->getP3dRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    if(calc_real_dof() > 0)
    {
        for(unsigned int i=0; i<mRobot->getNumberOfJoints(); i++)
        {
            jntPt = mRobot->getJoint(i);

            if( jntPt->getName() == "J0" )
                continue;

            for(unsigned int j=0; j<jntPt->getNumberOfDof(); j++)
            {
                k = jntPt->getIndexOfFirstDof() + j;

                if(/*(p3d_jnt_get_dof_is_user(jntPt->getP3dJointStruct(),j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1)*/ true )
                {
                    DofSlider* oneSlider = new DofSlider(mRobot,mOpenGl,this);
                    oneSlider->makeSlider( mGridLayout, jntPt, j );
                    mSliders.push_back( oneSlider );
                }
            }
        }
    }

    QSpacerItem *verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    mGridLayout->addItem(verticalSpacer, k+1, 0, 1, 1);
}

void FormRobot::resetConstraintedDoFs()
{
    p3d_rob* robotPt =   mRobot->getP3dRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    if(calc_real_dof() > 0)
    {
        for(unsigned int i=0;i<mSliders.size();i++)
        {
            int j;
            int numDof = mSliders[i]->getDofIdInConf();
            p3d_jnt* jntPt = p3d_robot_dof_to_jnt(robotPt, numDof, &j);

            if (jntPt)
            {
                if ( (!p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) || robotPt->cntrt_manager->in_cntrt[numDof] == 2 )
                {
                    mSliders[i]->getDoubleSpinBox()->setDisabled(true);
                    mSliders[i]->getHorizontalSlider()->setDisabled(true);
                }
                else
                {
                    mSliders[i]->getDoubleSpinBox()->setDisabled(false);
                    mSliders[i]->getHorizontalSlider()->setDisabled(false);
                }
            }
            else {
                cout << "Error in FormRobot::resetConstraintedDoFs!!!" << endl;
            }
        }
    }
}

/*!
 * Sets the slider value
 * with an input configuration
 */
void FormRobot::setSliders(Configuration& ptrConfRad)
{
    if( mSliders.empty() )
    {
        return;
    }

    string RobotName = ptrConfRad.getRobot()->getName();

    confPtr_t ptrConfDeg = ptrConfRad.getConfigInDegree();

    if( mRobot->getName().compare( RobotName ) == 0 )
    {
        p3d_rob* robotPt = ptrConfDeg->getRobot()->getP3dRobotStruct();
        int numDof = 0;

        if(calc_real_dof() > 0)
        {
            for(int i=0; i<=robotPt->njoints; i++)
            {
                p3d_jnt* jntPt = robotPt->joints[i];

                if( string(jntPt->name) == "J0" )
                    continue;

                for(int j=0; j<jntPt->dof_equiv_nbr; j++)
                {
                    int k = jntPt->index_dof + j;

                    if(/*(p3d_jnt_get_dof_is_user(jntPt,j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1)*/ true)
                    {
                            // cout << ptrConfDeg->getConfigStruct() << endl;
                            disconnect(mSliders[numDof]->getConnector(),SIGNAL(valueChanged(double)),
                                       mSliders[numDof],SLOT(dofValueChanged(double)));

                            // mSliders[numDof]->setValue( ptrConfDeg->at(k) );

                            mSliders[numDof]->getConnector()->setValue( ptrConfDeg->at(k) );

                            connect(mSliders[numDof]->getConnector(),SIGNAL(valueChanged(double)),
                                    mSliders[numDof],SLOT(dofValueChanged(double)));

                            numDof++;

                            if (robotPt->cntrt_manager->in_cntrt[k] == 2)
                            {
                                // mSliders.back().back()->doubleSpinBox->setDisabled(true);
                                // mSliders.back().back()->horizontalSlider->setDisabled(true);
                            }
                    }
                }
            }
        }
    }
}

/*!
 * Sets the robot in a particular configuration
 *
 */
void FormRobot::setCurrentConfig(int index)
{
    if ( mConfigurations.empty() || ((unsigned int)index >= mConfigurations.size()) )
    {
        return;
    }

    mRobot->setAndUpdate( *mConfigurations[index] );
    setSliders( *mRobot->getCurrentPos() );

    if(!ENV.getBool(Env::isRunning))
    {
        mOpenGl->updateGL();
    }
}

/*!
 * Sets the robot in
 *  0 - Init
 *  1 - Goto
 */
void FormRobot::setCurrentPosition(int position)
{
    shared_ptr<Configuration> ptrConf;

    if( position == 0 )
    {
        cout << "Robot " << mRobot->getName() << " to Initial Pos" << endl;

        ptrConf = mRobot->getInitPos();
        mRobot->setAndUpdate(*ptrConf);
        setSliders( *mRobot->getCurrentPos() );
    }

    if (position == 1)
    {
        cout << "Robot " << mRobot->getName() << " to Goto Pos" << endl;
        ptrConf = mRobot->getGoalPos();
        mRobot->setAndUpdate(*ptrConf);
        setSliders( *mRobot->getCurrentPos() );
    }

#ifdef WITH_XFORMS
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
    if(!ENV.getBool(Env::isRunning))
    {
        mOpenGl->updateGL();
    }
#endif
}

/*!
 * Saves the current Config into the robot configuration
 */
/* void FormRobot::saveCurrentConfigToPosition()
{	
    int index = mPositions->currentIndex();

    if( index == 0 )
    {
        mRobot->setInitialPosition(*mRobot->getCurrentPos());
        cout << "Save Config in Pos: " << index << endl;
    }

    if( index == 1 )
    {
        mRobot->setGoTo(*mRobot->getCurrentPos());
        cout << "Save Config in Pos: " << index << endl;
    }
}*/

/*!
 * Saves the current Config into the robot Init conf
 */
void FormRobot::saveCurrentConfigToInit()
{	
    mRobot->setInitPos( *mRobot->getCurrentPos() );
    cout << "Save Config in Robot Init" << endl;
}

/*!
 * Saves the current Config into the robot Goal conf
 */
void FormRobot::saveCurrentConfigToGoal()
{	
    mRobot->setGoalPos( *mRobot->getCurrentPos() );
    cout << "Save Config in GoTo" << endl;
}

void FormRobot::initConfigs()
{

}

/*!
 * Saves the current Config into the vector of configuration
 */
void FormRobot::saveCurrentConfigToVector()
{	
    mConfigurations.push_back( mRobot->getCurrentPos() );

    std::ostringstream configname;
    configname << "Config" << std::setw( 3 ) << std::setfill( '_' ) << mConfigurations.size();

    QString name = QString( configname.str().c_str() ).arg( mConfigurations.size() );

    mConfigNames->addItem( name );
    mConfigNames->setCurrentIndex( mConfigurations.size() - 1 );

    cout << "Create new config to save" << endl;
    p3d_set_new_robot_config(mRobot->getP3dRobotStruct(), name.toStdString().c_str(),
                             mRobot->getCurrentPos()->getConfigStruct(), NULL,
                             mRobot->getP3dRobotStruct()->confcur);

    cout << "Save Config in Pos: " << mConfigurations.size() << endl;
}

/**
 * Sets from the manpulation planner
 */
void FormRobot::setConfAndTrajFromManipTest()
{
    if( global_manipPlanTest == NULL )
    {
        cout << "global_manipPlanTest is not set" << endl;
    }

    std::string robotName( global_manipPlanTest->getManipPlanner()->robot()->name );

    if( robotName != mRobot->getName() )
    {
        cout << "Wrong robot in Manipulation test" << endl;
    }

    // Delete configurations
    mConfigurations.clear();
    mConfigNames->clear();

    // Delete trajectories
    for (int i=0; i<int(mTrajectories.size()); i++)
        delete mTrajectories[i];

    mTrajectories.clear();
    mTrajectoriesNames->clear();

    // Add all config in global_manipPlanTest
    vector< pair<string,double*> > configs = global_manipPlanTest->getConfVector();
    for (int i=0; i<int(configs.size()); i++)
    {
        addConfig( configs[i].first, configs[i].second );
    }

    // Add all trajs in global_manipPlanTest
    vector< pair<string,p3d_traj*> > trajs = global_manipPlanTest->getTrajVector();
    for (int i=0; i<int(trajs.size()); i++)
    {
        addTraj( trajs[i].first, trajs[i].second );
    }
}

/*!
 * Saves a config in the vector
 */
void FormRobot::addConfig( const std::string& name, configPt q )
{
    shared_ptr<Configuration> qStore( new Configuration( mRobot , q ) );

    mConfigurations.push_back( qStore );

    QString storeName = QString( name.c_str() );

    mConfigNames->addItem( storeName );
    mConfigNames->setCurrentIndex( mConfigurations.size() - 1 );
}

/*!
 * Saves the current Config into the robot configuration
 */
void FormRobot::addTraj(std::string& name,p3d_traj* trajPt)
{
    QString trajName(name.c_str());

    // Have to be done at the same time
    // To maintain the id correspondance
    mTrajectoriesNames->addItem(trajName);
    mTrajectories.push_back(trajPt);
}


/*!
 * Saves the current Config into the robot configuration
 */
void FormRobot::setCurrentTraj(int id)
{
    if ((unsigned int)id >= mTrajectories.size() || id < 0) {
        cout << "Wierd in FormRobot::setCurrentTraj" << endl;
        return;
    }

    if (mTrajectories[id] == 0) {
        return;
    }
    // TODO fix this
    cout << "mTrajectories[id]->num = " << mTrajectories[id]->num << endl;

    mRobot->getP3dRobotStruct()->tcur = mTrajectories[id];

    mOpenGl->updateGL();

    //global_w->setCurrentTraj( mTrajectories[id] );
}

/*!
 * Shows a form with the current constraints
 */
void FormRobot::showConstraints()
{
    ConstraintsWidget* constraints = new ConstraintsWidget( 0, this );
    constraints->show();
}

#ifdef MULTILOCALPATH
/*!
 * Show the multi-localpath
 */
void FormRobot::showMultiLocalpath()
{
    MultiLocalPathWidget* localPathGroups = new MultiLocalPathWidget( 0, mRobot->getP3dRobotStruct() );
    localPathGroups->show();
}
#endif

//---------------------------------------------------------------------
// DofSlider
//---------------------------------------------------------------------
/*!
 * Makes a slider
 *
 * LABEL + SPINBOX + SLIDER
 */
void DofSlider::makeSlider(QGridLayout* gridLayout, Joint *jnt, int DofNumOnJnt)
{
    p3d_jnt* jntPt = jnt->getP3dJointStruct();
    mDofNum = jntPt->index_dof + DofNumOnJnt;
    QString dofName( p3d_jnt_get_dof_name(jntPt, DofNumOnJnt) );

    double max;
    double min;

    p3d_jnt_get_dof_bounds_deg(jntPt, DofNumOnJnt, &min, &max);

    // Label
    mLabel = new QLabel();
    mLabel->setObjectName("LabelDof");
    mLabel->setText(dofName);

    QFont testFont( "Times", 10, QFont::Bold );
    mLabel->setFont ( testFont );

    gridLayout->addWidget(mLabel, mDofNum, 0, 1, 1);

    // Push Buttons
    mConstraints = new QPushButton("Cntrts");
    gridLayout->addWidget(mConstraints, mDofNum, 1, 1, 1);

    // SpinBox
    mDoubleSpinBox = new QDoubleSpinBox();
    mDoubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
    mDoubleSpinBox->setDecimals(3);
    mDoubleSpinBox->setSingleStep(1e-03);
    mDoubleSpinBox->setMaximum(max);
    mDoubleSpinBox->setMinimum(min);

    gridLayout->addWidget(mDoubleSpinBox, mDofNum, 2, 1, 1);

    // Slider
    mHorizontalSlider = new QSlider();
    mHorizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
    mHorizontalSlider->setOrientation(Qt::Horizontal);
    mHorizontalSlider->setMaximum(10000);

    gridLayout->addWidget(mHorizontalSlider, mDofNum, 3, 1, 1);

    // Connector
    mConnector = new QtShiva::SpinBoxSliderConnector(
                this, mDoubleSpinBox, mHorizontalSlider );

    connect(mConnector,SIGNAL(valueChanged(double)),this,SLOT(dofValueChanged(double)),Qt::DirectConnection);
    connect(mConstraints,SIGNAL(clicked()),this,SLOT(showDofConstraints()),Qt::DirectConnection);
}

/**
 * Returns the dof constraints
 */
void DofSlider::showDofConstraints()
{
    CntrtDoFWidget* cntrts = new CntrtDoFWidget(NULL,mFormRobot,mDofNum);
    cntrts->show();
}

/*!
 * Call Back called when a slider is moved
 */
void DofSlider::dofValueChanged(double value)
{
    //    std::string str = "ChangeDof";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //    CB_position_obj(NULL,value);

    configPt p=NULL, p_deg=NULL;
    //int nb_dof;
    //int ir;
    int i_dof;
    p3d_rob *robotPt;
    p3d_jnt *jntPt;
    //int ncol=0;
    int I_can;
    int nreshoot;   // <-modif Juan
    int *ikSol = NULL;

    double val = value; //fl_get_slider_value(ob);
    int arg = mDofNum;

    //    ir = p3d_get_desc_curnum(P3D_ROBOT);
    //    robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    //    nb_dof = p3d_get_robot_ndof();

    //nb_dof =    mRobot->getP3dRobotStruct()->nb_dof; //p3d_get_robot_ndof();
    /*ir =*/        mRobot->getP3dRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
    robotPt =   mRobot->getP3dRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    //  cout << "robotPt = " << robotPt << endl;

    //#ifdef HRI_PLANNER
    //	if (GLOBAL_AGENTS)
    //	{
    //double distance = 0.0;
    //double cost = hri_distance_cost(GLOBAL_AGENTS,distance);
    //	}
    //#endif

    p = p3d_alloc_config(robotPt);
    //    p = mRobot->getNewConfig()->getConfigStruct();
    //     p_deg = p3d_alloc_config(robotPt);

    //        if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
    //          p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg);
    //          ikSol = robotPt->ikSolPos;
    //        }
    //        else{
    //          p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO, &p_deg);
    //          ikSol = robotPt->ikSolGoto;
    //        }


    p_deg = p3d_get_robot_config_deg(robotPt);
    //    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg)
    //    p_deg = mRobot->getCurrentPos()->getConfigInDegree()->getConfigStruct();

    p_deg[arg] = val;
    p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);

    /*update the configuration of the current robot */
    /* sustitution pour la partie de la fonction dans l'anciene version de Move3D */
    /*   p3d_set_and_update_robot_conf(p); */ /* <- dans la fonction  remplacee */

    p3d_set_robot_config(robotPt, p);

    //   I_can = p3d_update_robot_pos();
    I_can = p3d_update_this_robot_pos_multisol(robotPt, NULL, 0, ikSol);

    //cout << "I_can = "  << I_can << endl;

    if (robotPt->cntrt_manager->cntrts != NULL)
    {
        // modif Juan
        nreshoot = 0;
        if(!I_can && p3d_get_RLG())
        {
            if(robotPt->cntrt_manager->in_cntrt[arg] == 1)
            {
                while(!I_can && (nreshoot < 100))
                {
                    if(p3d_random_loop_generator_without_parallel(robotPt, p))
                    {
                        I_can = p3d_update_robot_pos();
                    }
                    nreshoot++;
                }
            }
        }
        //         fmodif Juan
        if(I_can)
        {
            p3d_get_robot_config_into(robotPt, &p);
            p3d_get_robot_config_deg_into(robotPt, &p_deg);
            //for(i=0; i<nb_dof; i++)
            //{
            //                if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
            //                {
            //                    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
            //                }
            //}

            //print_config(robotPt,p);
            //            p3d_copy_config_into(robotPt, p_deg, &last_p_deg[ir]);
        }
        else
        {
            //            p3d_copy_config_into(robotPt, last_p_deg[ir], &p_deg);
            p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);
            //for(i=0; i<nb_dof; i++)
            //{
            //                if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
            //                {
            //                    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
            //                }
            //}
            jntPt = p3d_robot_dof_to_jnt(robotPt, arg, &i_dof);
            p3d_jnt_set_dof_deg(jntPt, i_dof, p_deg[arg]);  /* ceneccasy lines for some cases of cntrts !!! */
            p3d_update_this_robot_pos_without_cntrt(robotPt);
        }
    }

    /* update the field current position or goal position of the
     current robot depending on field GOTO_OBJ */
    //    if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
    //        p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_POS));
    //    }
    //    else{
    //        p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_GOTO));
    //    }

    /* update trajectory if track_traj mode is activated */
    //    if (ROBOTS_FORM[ir].TRACK_TRAJ)
    //    {
    //        CB_stop_obj(ob,0);
    //        CB_stop_optim_obj(ob,0);
    //        if (!p3d_trackCurrentTraj(robotPt,3, 0.001, p3d_get_d0(),p3d_get_QUICK_DESCENT()))
    //        {
    //            CB_specific_search_obj(ob,0);
    //        }
    //        p3d_set_and_update_robot_conf(p);
    //    }

    qtSliderFunction(robotPt, p);

    p3d_destroy_config(robotPt, p);
    p3d_destroy_config(robotPt, p_deg);


#ifdef WITH_XFORMS
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
    if(!ENV.getBool(Env::isRunning))
    {
        mOpenGl->updateGL();
    }
#endif

    // Sets the actual configuration to all sliders
    mFormRobot->setSliders(*mRobot->getCurrentPos());
}

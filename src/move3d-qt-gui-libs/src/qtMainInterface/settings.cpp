//
//  settings.cpp
//  move3d-studio
//
//  Created by Jim Mainprice on 27/10/11.
//  Copyright 2011 LAAS-CNRS. All rights reserved.
//

#include "settings.hpp"

//! @file Settings Functions to load and save UI settings

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#include "API/Device/robot.hpp"
#include "API/project.hpp"
#include "planner/planEnvironment.hpp"
#include "hri_costspace/Gestures/HRICS_GestParameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"

//#include "env.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"

#include <QtCore/QMetaProperty>
#include <QtCore/QSettings>

using namespace Move3D;
using namespace std;

//---------------------------------------------------------
//! Save Camera and Axis info in a QSetting struct
void qt_saveCameraAndAxis(bool print, g3d_states& vs, QSettings& settings) 
{	
    settings.beginGroup("Camera");

    settings.setValue( QString("ox"), vs.x );
    settings.setValue( QString("oy"), vs.y );
    settings.setValue( QString("oz"), vs.z);

    settings.setValue( QString("dist"), vs.zo );
    settings.setValue( QString("az"), vs.az );
    settings.setValue( QString("el"), vs.el);

    settings.setValue( QString("up0"), vs.up[0] );
    settings.setValue( QString("up1"), vs.up[1] );
    settings.setValue( QString("up2"), vs.up[2] );

    settings.endGroup();
}

//---------------------------------------------------------
//! Load Camera and Axis info in a QSetting struct
void qt_loadCameraAndAxis(bool print, g3d_states& vs, QSettings& settings) 
{	
    settings.beginGroup("Camera");

    vs.x = settings.value("ox", vs.x).toFloat();
    vs.y = settings.value("oy", vs.y).toFloat();
    vs.z = settings.value("oz", vs.z).toFloat();

    vs.zo = settings.value("dist", vs.zo).toFloat();
    vs.az = settings.value("az", vs.az).toFloat();
    vs.el = settings.value("el", vs.el).toFloat();

    vs.up[0] = settings.value("up0", vs.up[0]).toFloat();
    vs.up[1] = settings.value("up1", vs.up[1]).toFloat();
    vs.up[2] = settings.value("up2", vs.up[2]).toFloat();
    vs.up[3]=0.0;

    settings.endGroup();
}

//---------------------------------------------------------
//! Save multilocalpath info in a QSetting struct
void qt_saveMultiLocalPath(bool print, Robot* rob, QSettings& settings) 
{	
    if( rob->getRobotStruct()->mlp->nblpGp < 1 )
        return;

    settings.beginGroup("ActiveRobotLocalpath");
    settings.setValue( QString("RobotName"), QString(rob->getName().c_str()));

    // over all localpaths
    for(int i=0; i<rob->getRobotStruct()->mlp->nblpGp; i++)
    {
        // get group name
        std::string LocalpathName = rob->getRobotStruct()->mlp->mlpJoints[i]->gpName;

        bool value = (p3d_multiLocalPath_get_value_groupToPlan( rob->getRobotStruct(), i) == 1);
        settings.setValue( QString(LocalpathName.c_str()),value);
    }
    settings.endGroup();
}

//---------------------------------------------------------
//! Load multilocalpath info in a QSetting struct
void qt_loadMultiLocalPath(bool print, Robot* rob, QSettings& settings) 
{	
    if( rob->getRobotStruct()->mlp->nblpGp < 1 )
        return;

    settings.beginGroup("ActiveRobotLocalpath");
    std::string robotName = settings.value(QString("RobotName"),"").toString().toStdString();

    if( robotName!= rob->getName() )
    {
        settings.endGroup();
        return;
    }

    // over all localpaths
    for(int i=0; i<rob->getRobotStruct()->mlp->nblpGp; i++)
    {
        // get group name
        std::string LocalpathName = rob->getRobotStruct()->mlp->mlpJoints[i]->gpName;

        bool value = settings.value( QString( LocalpathName.c_str() )).toBool();
        p3d_multiLocalPath_set_groupToPlan( rob->getRobotStruct(), i , value, false );
    }

    if(print)
    {
        p3d_multilocapath_print_group_info( rob->getRobotStruct() );
    }
    settings.endGroup();
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//! Save cntrts info in a QSetting struct
void qt_saveCntrts(bool print, Robot* rob, QSettings& settings) 
{	
    if( rob->getRobotStruct()->cntrt_manager->ncntrts < 1 )
        return;

    settings.beginGroup("ActiveRobotCntrts");
    settings.setValue( QString("RobotName"), QString(rob->getName().c_str()));

    // over all constraints
    for(int i=0; i<rob->getRobotStruct()->cntrt_manager->ncntrts; i++)
    {
        // get constraint from the cntrts manager
        p3d_cntrt* ct = rob->getRobotStruct()->cntrt_manager->cntrts[i];

        std::string cntName = ct->namecntrt;

        if( ct->active == 1)
            settings.setValue( QString(cntName.c_str()),true);
        else
            settings.setValue( QString(cntName.c_str()),false);
    }
    settings.endGroup();
}

//---------------------------------------------------------
//! Load cntrts info in a QSetting struct
void qt_loadCntrts(bool print, Robot* rob, QSettings& settings) 
{	
    if( rob->getRobotStruct()->cntrt_manager->ncntrts < 1 )
        return;

    settings.beginGroup("ActiveRobotCntrts");
    std::string robotName = settings.value(QString("RobotName"),"").toString().toStdString();

    if( robotName!= rob->getName() )
    {
        settings.endGroup();
        return;
    }

    // over all constraints
    for(int i=0; i<rob->getRobotStruct()->cntrt_manager->ncntrts; i++)
    {
        // get constraint from the cntrts manager
        p3d_cntrt* ct = rob->getRobotStruct()->cntrt_manager->cntrts[i];

        std::string cntName = ct->namecntrt;

        if( settings.value(QString(cntName.c_str())).toBool())
            p3d_activateCntrt( rob->getRobotStruct(), ct );
        else
            p3d_desactivateCntrt( rob->getRobotStruct(), ct );
    }
    settings.endGroup();
}

//////////////////////////////////////////////////////////////////////////////
// LOAD
//////////////////////////////////////////////////////////////////////////////

void qt_loadInterfaceParameters(bool print, std::string fileName, bool opengl)
{ 
    QSettings settings(QString(fileName.c_str()), QSettings::IniFormat);
    // ------------------------------------------------------------------
    // ENV
    // ------------------------------------------------------------------

    const QMetaObject* metaObject = ENV.metaObject();
    QMetaEnum metaEnum;

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        ENV.setBool(Env::boolParameter(i),settings.value(QString(metaEnum.key(i)),false).toBool());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << ENV.getBool(Env::boolParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        ENV.setInt(Env::intParameter(i),settings.value(QString(metaEnum.key(i)),0).toInt());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << ENV.getInt(Env::intParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        ENV.setDouble(Env::doubleParameter(i),settings.value(QString(metaEnum.key(i)),0.0).toFloat());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << ENV.getDouble(Env::doubleParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        ENV.setString(Env::stringParameter(i),settings.value(QString(metaEnum.key(i)),"").toString().toStdString());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << ENV.getString(Env::stringParameter(i)) << endl;
        }
    }
    settings.endGroup();

    // ------------------------------------------------------------------
    // PlanParam
    // ------------------------------------------------------------------
    settings.beginGroup("PlanParam");

    metaObject = EnumPlannerParameterObject->metaObject();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        PlanEnv->setBool(PlanParam::boolParameter(i),settings.value(QString(metaEnum.key(i)),false).toBool());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << PlanEnv->getBool(PlanParam::boolParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        PlanEnv->setInt(PlanParam::intParameter(i),settings.value(QString(metaEnum.key(i)),0).toInt());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << PlanEnv->getInt(PlanParam::intParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        PlanEnv->setDouble(PlanParam::doubleParameter(i),settings.value(QString(metaEnum.key(i)),0.0).toFloat());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << PlanEnv->getDouble(PlanParam::doubleParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        PlanEnv->setString(PlanParam::stringParameter(i),settings.value(QString(metaEnum.key(i)),"").toString().toStdString() );

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << PlanEnv->getString(PlanParam::stringParameter(i)) << endl;
        }
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // GestParam
    // ------------------------------------------------------------------
    settings.beginGroup("GestParam");

    metaObject = EnumGestureParameterObject->metaObject();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        GestEnv->setBool(GestParam::boolParameter(i),settings.value(QString(metaEnum.key(i)),false).toBool());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << GestEnv->getBool(GestParam::boolParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        GestEnv->setInt(GestParam::intParameter(i),settings.value(QString(metaEnum.key(i)),0).toInt());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << GestEnv->getInt(GestParam::intParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        GestEnv->setDouble(GestParam::doubleParameter(i),settings.value(QString(metaEnum.key(i)),0.0).toFloat());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << GestEnv->getDouble(GestParam::doubleParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        GestEnv->setString(GestParam::stringParameter(i),settings.value(QString(metaEnum.key(i)),"").toString().toStdString() );

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << GestEnv->getString(GestParam::stringParameter(i)) << endl;
        }
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // HricsParam
    // ------------------------------------------------------------------
    settings.beginGroup("HricsParam");

    metaObject = EnumHricsParameterObject->metaObject();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        HriEnv->setBool(HricsParam::boolParameter(i),settings.value(QString(metaEnum.key(i)),false).toBool());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << HriEnv->getBool(HricsParam::boolParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        HriEnv->setInt(HricsParam::intParameter(i),settings.value(QString(metaEnum.key(i)),0).toInt());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << HriEnv->getInt(HricsParam::intParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        HriEnv->setDouble(HricsParam::doubleParameter(i),settings.value(QString(metaEnum.key(i)),0.0).toFloat());

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << HriEnv->getDouble(HricsParam::doubleParameter(i)) << endl;
        }
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    if(print)
        cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        HriEnv->setString(HricsParam::stringParameter(i),settings.value(QString(metaEnum.key(i)),"").toString().toStdString() );

        if(print)
        {
            cout << "Key : " << metaEnum.key(i);
            cout << " , Value : " << HriEnv->getString(HricsParam::stringParameter(i)) << endl;
        }
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // Cntrts and Localpaths
    // ------------------------------------------------------------------
    Scene* sce = global_Project->getActiveScene();

    for (int i=0; i<int(sce->getNumberOfRobots()); i++) {
        qt_loadCntrts(print,sce->getRobot(i),settings);
        qt_loadMultiLocalPath(print,sce->getRobot(i),settings);
    }

    if( opengl )
        qt_loadCameraAndAxis( print, g3d_get_cur_states(), settings) ;
}

//////////////////////////////////////////////////////////////////////////////
// SAVE
//////////////////////////////////////////////////////////////////////////////

void qt_saveInterfaceParameters(bool print, std::string fileName)
{
    QSettings settings(QString(fileName.c_str()), QSettings::IniFormat);
    settings.clear();
    // ------------------------------------------------------------------
    // ENV
    // ------------------------------------------------------------------
    cout << "---------------------------" << endl;
    cout << "ENV" << endl;
    const QMetaObject* metaObject = ENV.metaObject();
    QMetaEnum metaEnum;

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        string key = metaEnum.key(i);

        if( key == "isRunning" )
            continue;

        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << ENV.getBool(Env::boolParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), ENV.getBool(Env::boolParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << ENV.getInt(Env::intParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), ENV.getInt(Env::intParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << ENV.getDouble(Env::doubleParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), ENV.getDouble(Env::doubleParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << ENV.getString(Env::stringParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), QString(ENV.getString(Env::stringParameter(i)).c_str()));
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // PlanParam
    // ------------------------------------------------------------------
    cout << "---------------------------" << endl;
    cout << "PLAN PARAM" << endl;
    settings.beginGroup("PlanParam");

    metaObject = EnumPlannerParameterObject->metaObject();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << PlanEnv->getBool(PlanParam::boolParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), PlanEnv->getBool(PlanParam::boolParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << PlanEnv->getInt(PlanParam::intParameter(i)) << endl;
        settings.setValue(QString(metaEnum.key(i)), PlanEnv->getInt(PlanParam::intParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << PlanEnv->getDouble(PlanParam::doubleParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), PlanEnv->getDouble(PlanParam::doubleParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << PlanEnv->getString(PlanParam::stringParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), QString(PlanEnv->getString(PlanParam::stringParameter(i)).c_str()) );
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // GestParam
    // ------------------------------------------------------------------
    cout << "---------------------------" << endl;
    cout << "GEST PARAM" << endl;
    settings.beginGroup("GestParam");

    metaObject = EnumGestureParameterObject->metaObject();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << GestEnv->getBool(GestParam::boolParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), GestEnv->getBool(GestParam::boolParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << GestEnv->getInt(GestParam::intParameter(i)) << endl;
        settings.setValue(QString(metaEnum.key(i)), GestEnv->getInt(GestParam::intParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << GestEnv->getDouble(GestParam::doubleParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), GestEnv->getDouble(GestParam::doubleParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << GestEnv->getString(GestParam::stringParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), QString(GestEnv->getString(GestParam::stringParameter(i)).c_str()) );
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // HricsParam
    // ------------------------------------------------------------------
    cout << "---------------------------" << endl;
    cout << "HRICS PARAM" << endl;
    settings.beginGroup("HricsParam");

    metaObject = EnumHricsParameterObject->metaObject();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "boolParameter" ) );
    settings.beginGroup("boolParameter");
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << HriEnv->getBool(HricsParam::boolParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), HriEnv->getBool(HricsParam::boolParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "intParameter" ) );
    settings.beginGroup("intParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << HriEnv->getInt(HricsParam::intParameter(i)) << endl;
        settings.setValue(QString(metaEnum.key(i)), HriEnv->getInt(HricsParam::intParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "doubleParameter" ) );
    settings.beginGroup("doubleParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << HriEnv->getDouble(HricsParam::doubleParameter(i)) << endl;
        settings.setValue( QString(metaEnum.key(i)), HriEnv->getDouble(HricsParam::doubleParameter(i)));
    }
    settings.endGroup();

    metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "stringParameter" ) );
    settings.beginGroup("stringParameter");
    cout << "---------------------------" << endl;
    for( int i=0;i<metaEnum.keyCount();i++)
    {
        cout << "Key : " << metaEnum.key(i);
        cout << " , Value : " << HriEnv->getString(HricsParam::stringParameter(i)) << endl; // QString(ENV.getString(Env::stringParameter(i)).c_str())
        settings.setValue( QString(metaEnum.key(i)),QString(HriEnv->getString(HricsParam::stringParameter(i)).c_str()));
    }
    settings.endGroup();

    settings.endGroup();

    // ------------------------------------------------------------------
    // Cntrts and Localpaths
    // ------------------------------------------------------------------
    Scene* sce = global_Project->getActiveScene();

    for (int i=0; i<int(sce->getNumberOfRobots()); i++) {
        qt_saveCntrts(print,sce->getRobot(i),settings);
        qt_saveMultiLocalPath(print,sce->getRobot(i),settings);
    }

    qt_saveCameraAndAxis( print, g3d_get_cur_states(), settings) ;

    //
    //  metaEnum = metaObject->enumerator( metaObject->indexOfEnumerator( "vectorParameter" ) );
    //  cout << "---------------------------" << endl;
    //  for( int i=0;i<metaEnum.keyCount();i++)
    //  {
    //    cout << "Key : " << metaEnum.key(i) << endl;
    //    //cout << " , Value : " << ENV.getVector(Env::vectorParameter(i)) << endl;
    //  }
}

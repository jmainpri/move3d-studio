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
#include "guiparams.hpp"

#include "mainwindowGenerated.hpp"
#include "move3d-headless.h"
#include <iostream>

#include <QtCore/QMetaProperty>
#include <QtCore/QSettings>

using std::cout;
using std::endl;

//#include "../p3d/env.hpp"

// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disabled this just acts as a normal container

// Definition of the parameter container
Parameters<GuiParam::boolParameter,
           GuiParam::intParameter,
           GuiParam::doubleParameter,
           GuiParam::stringParameter,
           GuiParam::vectorParameter>* GuiEnv = NULL;

GuiParam* EnumGuiParameterObject = NULL;

GuiParam::GuiParam() {}

GuiParam::~GuiParam() {}

// @brief Function that inizializes the
// Parameter container
void initGuiParameters() {
  EnumGuiParameterObject = new GuiParam;

  // Create 5 maps for all types and fill the 5 maps
  // ------------------------------------------------------------------
  std::map<GuiParam::boolParameter, boolContainer*> myBoolMap;
  std::map<GuiParam::intParameter, intContainer*> myIntMap;
  std::map<GuiParam::doubleParameter, doubleContainer*> myDoubleMap;
  std::map<GuiParam::stringParameter, stringContainer*> myStringMap;
  std::map<GuiParam::vectorParameter, vectorContainer*> myVectorMap;

  // Bool
  // ------------------------------------------------------------------
  myBoolMap.insert(std::make_pair(GuiParam::tete, new boolContainer(false)));

  // Int
  // ------------------------------------------------------------------
  myIntMap.insert(std::make_pair(GuiParam::mainwin_x, new intContainer(0)));
  myIntMap.insert(std::make_pair(GuiParam::mainwin_y, new intContainer(0)));
  myIntMap.insert(std::make_pair(GuiParam::mainwin_w, new intContainer(1000)));
  myIntMap.insert(std::make_pair(GuiParam::mainwin_h, new intContainer(600)));
  myIntMap.insert(
      std::make_pair(GuiParam::tab_index_main, new intContainer(-1)));
  myIntMap.insert(
      std::make_pair(GuiParam::tab_index_cost, new intContainer(-1)));

  // Double
  // ------------------------------------------------------------------
  myDoubleMap.insert(std::make_pair(GuiParam::toto, new doubleContainer(10.0)));

  // cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble(
  // PlanParam::env_objectNessecity ) << endl;

  // String
  // ------------------------------------------------------------------
  myStringMap.insert(
      std::make_pair(GuiParam::titi, new stringContainer("titi")));

  // Vector
  // ------------------------------------------------------------------
  std::vector<double> tutu;
  tutu.push_back(1);
  tutu.push_back(8);

  myVectorMap.insert(std::make_pair(GuiParam::tutu, new vectorContainer(tutu)));

  // Make the new parameter container
  GuiEnv = new Parameters<GuiParam::boolParameter,
                          GuiParam::intParameter,
                          GuiParam::doubleParameter,
                          GuiParam::stringParameter,
                          GuiParam::vectorParameter>(
      myBoolMap, myIntMap, myDoubleMap, myStringMap, myVectorMap);
}

//////////////////////////////////////////////////////////////////////////////
// GUI
//////////////////////////////////////////////////////////////////////////////

//! Save gui settings
void qt_saveGuiParameters(bool print, std::string fileName, MainWindow* mw) {
  QSettings settings(QString(fileName.c_str()), QSettings::IniFormat);
  QMetaEnum metaEnum;

  const QMetaObject* metaObject = EnumGuiParameterObject->metaObject();

  settings.beginGroup("GuiParam");

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("boolParameter"));
  settings.beginGroup("boolParameter");
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    cout << "Key : " << metaEnum.key(i);
    cout << " , Value : " << GuiEnv->getBool(GuiParam::boolParameter(i))
         << endl;
    settings.setValue(QString(metaEnum.key(i)),
                      GuiEnv->getBool(GuiParam::boolParameter(i)));
  }
  settings.endGroup();

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("intParameter"));
  settings.beginGroup("intParameter");
  cout << "---------------------------" << endl;
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    cout << "Key : " << metaEnum.key(i);
    cout << " , Value : " << GuiEnv->getInt(GuiParam::intParameter(i)) << endl;
    settings.setValue(QString(metaEnum.key(i)),
                      GuiEnv->getInt(GuiParam::intParameter(i)));
  }
  settings.endGroup();

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("doubleParameter"));
  settings.beginGroup("doubleParameter");
  cout << "---------------------------" << endl;
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    cout << "Key : " << metaEnum.key(i);
    cout << " , Value : " << GuiEnv->getDouble(GuiParam::doubleParameter(i))
         << endl;
    settings.setValue(QString(metaEnum.key(i)),
                      GuiEnv->getDouble(GuiParam::doubleParameter(i)));
  }
  settings.endGroup();

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("stringParameter"));
  settings.beginGroup("stringParameter");
  cout << "---------------------------" << endl;
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    cout << "Key : " << metaEnum.key(i);
    cout << " , Value : " << GuiEnv->getString(GuiParam::stringParameter(i))
         << endl;
    settings.setValue(
        QString(metaEnum.key(i)),
        QString(GuiEnv->getString(GuiParam::stringParameter(i)).c_str()));
  }
  settings.endGroup();

  settings.setValue("splitterSizes", mw->Ui()->vSplitter->saveState());
  settings.setValue("mainWindowGeometry", mw->saveGeometry());
  settings.setValue("mainWindowState", mw->saveState());

  settings.endGroup();
}

void qt_loadGuiParameters(bool print, std::string fileName, MainWindow* mw) {
  QSettings settings(QString(fileName.c_str()), QSettings::IniFormat);
  QMetaEnum metaEnum;

  settings.beginGroup("GuiParam");

  const QMetaObject* metaObject = EnumGuiParameterObject->metaObject();

  cout << "metaObject : " << metaObject << endl;

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("boolParameter"));
  settings.beginGroup("boolParameter");
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    GuiEnv->setBool(GuiParam::boolParameter(i),
                    settings.value(QString(metaEnum.key(i)), false).toBool());

    if (print) {
      cout << "Key : " << metaEnum.key(i);
      cout << " , Value : " << GuiEnv->getBool(GuiParam::boolParameter(i))
           << endl;
    }
  }
  settings.endGroup();

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("intParameter"));
  settings.beginGroup("intParameter");
  if (print) cout << "---------------------------" << endl;
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    GuiEnv->setInt(GuiParam::intParameter(i),
                   settings.value(QString(metaEnum.key(i)), 0).toInt());

    if (print) {
      cout << "Key : " << metaEnum.key(i);
      cout << " , Value : " << GuiEnv->getInt(GuiParam::intParameter(i))
           << endl;
    }
  }
  settings.endGroup();

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("doubleParameter"));
  settings.beginGroup("doubleParameter");
  if (print) cout << "---------------------------" << endl;
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    GuiEnv->setDouble(GuiParam::doubleParameter(i),
                      settings.value(QString(metaEnum.key(i)), 0.0).toFloat());

    if (print) {
      cout << "Key : " << metaEnum.key(i);
      cout << " , Value : " << GuiEnv->getDouble(GuiParam::doubleParameter(i))
           << endl;
    }
  }
  settings.endGroup();

  metaEnum =
      metaObject->enumerator(metaObject->indexOfEnumerator("stringParameter"));
  settings.beginGroup("stringParameter");
  if (print) cout << "---------------------------" << endl;
  for (int i = 0; i < metaEnum.keyCount(); i++) {
    GuiEnv->setString(
        GuiParam::stringParameter(i),
        settings.value(QString(metaEnum.key(i)), "").toString().toStdString());

    if (print) {
      cout << "Key : " << metaEnum.key(i);
      cout << " , Value : " << GuiEnv->getString(GuiParam::stringParameter(i))
           << endl;
    }
  }
  settings.endGroup();

  if( mw != NULL )
  {
    mw->Ui()->vSplitter->restoreState(
        settings.value("splitterSizes").toByteArray());
    mw->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    mw->restoreState(settings.value("mainWindowState").toByteArray());
  }

  settings.endGroup();
}

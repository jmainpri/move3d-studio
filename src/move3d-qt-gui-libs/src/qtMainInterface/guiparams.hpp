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
#ifndef GUI_PARAMETERS_HPP
#define GUI_PARAMETERS_HPP

#include <libmove3d/p3d/ParametersEnv.hpp>

#include "mainwindow.hpp"

#include <QtCore/QSettings>

#ifdef QT_LIBRARY
class GuiParam : public QObject {
  Q_OBJECT;
  Q_ENUMS(boolParameter);
  Q_ENUMS(intParameter);
  Q_ENUMS(doubleParameter);
  Q_ENUMS(stringParameter);
  Q_ENUMS(vectorParameter);

 public:
  GuiParam();
  ~GuiParam();

#else
namespace GuiParam {
#endif
  enum boolParameter { tete };

  enum intParameter {
    mainwin_x,
    mainwin_y,
    mainwin_w,
    mainwin_h,
    tab_index_main,
    tab_index_cost
  };

  enum doubleParameter { toto };

  enum stringParameter { titi };

  enum vectorParameter { tutu };
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<GuiParam::boolParameter,
                  GuiParam::intParameter,
                  GuiParam::doubleParameter,
                  GuiParam::stringParameter,
                  GuiParam::vectorParameter>* GuiEnv;

// Functions that initializes the planner
// Parameters
void initGuiParameters();

#ifdef QT_LIBRARY
extern GuiParam* EnumGuiParameterObject;
#endif

// Parameters of individual (computer, account) settings
void qt_saveGuiParameters(bool print, std::string fileName, MainWindow* mw);
void qt_loadGuiParameters(bool print, std::string fileName, MainWindow* mw=NULL);


#endif  // GUI_PARAMETERS_HPP

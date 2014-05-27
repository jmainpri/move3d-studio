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
#ifndef QTLIBRARY_HPP
#define QTLIBRARY_HPP

#include "Graphic-pkg.h"

#undef Status
#undef Bool
#undef Black
#undef CursorShape
#undef None
#undef KeyPress
#undef KeyRelease
#undef FocusIn
#undef FocusOut
#undef FontChange
#undef Unsorted

#include <QtOpenGL/QGLWidget>

#include <QtCore/QObject>

#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QSemaphore>
#include <QtCore/QDebug>
#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>

//#include <QtGui>
#include <QtGui/QCDEStyle>
#include <QtGui/QCleanlooksStyle>
#include <QtGui/QCommonStyle>
#include <QtGui/QMotifStyle>
#include <QtGui/QPlastiqueStyle>
#include <QtGui/QWindowsStyle>
//#include <QtGui/QMacStyle>
#include <QtGui/QWindowsVistaStyle>

#include <QtGui/QMainWindow>

#include <QtGui/QWidget>
#include <QtGui/QApplication>
#include <QtGui/QFileDialog>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QLabel>
#include <QtGui/QLayout>
#include <QtGui/QSlider>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QCheckBox>
#include <QtGui/QPushButton>
#include <QtGui/QComboBox>
#include <QtGui/QStackedWidget>
#include <QtGui/QSpacerItem>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QLabel>
#include <QtGui/QScrollBar>
#include <QtGui/QTextEdit>
#include <QtGui/QKeyEvent>
#include <QtGui/QScrollArea>

#define Bool int
#define Status int
#define True 1
#define False 0
#define Black 0

#endif // QTLIBRARY_HPP

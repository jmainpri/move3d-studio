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
#include <QtGui/qapplication.h>
#include <QtGui/qmainwindow.h>
#include <qwt_counter.h>
#include <QtGui/qtoolbar.h>
#include <QtGui/qlabel.h>
#include <QtGui/qlayout.h>

#include "dataPlot.hpp"
#include "tempWin.hpp"

TempWin::TempWin()
{
	QToolBar *toolBar = new QToolBar(this);
        toolBar->setFixedHeight(50);

        this->resize(500,300);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        this->setSizePolicy(sizePolicy);

#if QT_VERSION < 0x040000
	setDockEnabled(TornOff, true);
	setRightJustification(true);
#else
	toolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::BottomToolBarArea);
#endif
	QWidget *hBox = new QWidget(toolBar);
	QLabel *label = new QLabel("Timer Interval", hBox);
	QwtCounter *counter = new QwtCounter(hBox);
        counter->setRange(-1.0, 10.0, 1.0);

	QLabel *label2 = new QLabel("Maximum T", hBox);
	QwtCounter *counter2 = new QwtCounter(hBox);
        counter2->setRange(0.0000000001, 100.0, 0.1);

	QHBoxLayout *layout = new QHBoxLayout(hBox);
	layout->addWidget(label);
	layout->addWidget(counter);
	layout->addWidget(label2);
	layout->addWidget(counter2);
	layout->addWidget(new QWidget(hBox), 10); // spacer);

#if QT_VERSION >= 0x040000
	toolBar->addWidget(hBox);
#endif
	addToolBar(toolBar);

	DataPlot *plot = new DataPlot(this);

	setCentralWidget(plot);

	connect(counter, SIGNAL(valueChanged(double)), plot,
			SLOT(setTimerInterval(double)));

	connect(counter2, SIGNAL(valueChanged(double)), plot,
				SLOT(setMax(double)));

	counter->setValue(20.0);
	counter2->setValue(1.0);
}

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
#ifndef QT_BASE_WIN_HH
#define QT_BASE_WIN_HH

#include "../p3d/env.hpp"

#if defined( MOVE3D_CORE )
#include "qtLibrary.hpp"
#include "qtBase/qt_widgets.hpp"
#endif

/**
 * @ingroup qtOldWidget
 * @brief Qt Window base container
 */
class qtBaseWindow : public QObject
{
	Q_OBJECT;

protected:
	QString string;
	QGroupBox* box;
	QGridLayout* Layout;

public:
	// Constructor
	qtBaseWindow();

	// Getters
	QString 		getString();
	QGroupBox* 		getBox();
	QGridLayout* 	getLayout();

	// Initialization function
	//virtual void  init() = 0;
	//void init();

	// Create sliders and check boxes
	LabeledSlider* createSlider(QString s, Env::intParameter p, int lower, int upper);
	LabeledDoubleSlider* createDoubleSlider(QString s, Env::doubleParameter p, double lower, double upper);
	QCheckBox* createCheckBox(QString s, Env::boolParameter p);

public:
	// Destructor
	~qtBaseWindow();

};
#endif

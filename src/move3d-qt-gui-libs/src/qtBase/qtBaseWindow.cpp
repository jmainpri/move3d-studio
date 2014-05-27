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
#include "qtBaseWindow.hpp"
#include <iostream>

// Constructor
qtBaseWindow::qtBaseWindow()
{
	string = tr("Name of Window");

	Layout = new QGridLayout();
	Layout->setSpacing(3);
	Layout->setContentsMargins(3, 3, 3, 3);

	box = new QGroupBox(string);
	box->setLayout(Layout);
}

// Getters
QString qtBaseWindow::getString()
{
	return string;
}

QGroupBox* qtBaseWindow::getBox()
{
	return box;
}

QGridLayout* qtBaseWindow::getLayout()
{
	return Layout;
}

LabeledSlider* qtBaseWindow::createSlider(QString s, Env::intParameter p,
		int lower, int upper)
{
	LabeledSlider* slider = new LabeledSlider(lower, upper, lower, s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(int)), slider,
			SLOT(setValue(int)), Qt::DirectConnection);
	connect(slider, SIGNAL(valueChanged(int)), ENV.getObject(p),
			SLOT(set(int)), Qt::DirectConnection);
	slider->setValue(ENV.getInt(p));
	return (slider);
}

LabeledDoubleSlider* qtBaseWindow::createDoubleSlider(QString s,
		Env::doubleParameter p, double lower, double upper)
{
	LabeledDoubleSlider* slider = new LabeledDoubleSlider(lower, upper, lower,
			s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(double)), slider,
			SLOT(setValue(double)), Qt::DirectConnection);
	connect(slider, SIGNAL(valueChanged(double)), ENV.getObject(p),
			SLOT(set(double)), Qt::DirectConnection);
	slider->setValue(ENV.getDouble(p));
	return (slider);
}

QCheckBox* qtBaseWindow::createCheckBox(QString s, Env::boolParameter p)
{
	QCheckBox* box = new QCheckBox(s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box,
			SLOT(setChecked(bool)), Qt::DirectConnection);
	connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)),
			Qt::DirectConnection);
	box->setChecked(ENV.getBool(p));
	return (box);
}

qtBaseWindow::~qtBaseWindow()
{
	//delete box;
	//delete Layout;
}


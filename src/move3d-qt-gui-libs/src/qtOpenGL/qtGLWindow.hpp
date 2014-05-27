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
/**
 * QT Open GL Window
 */

#ifndef QT_GL_WINDOW_H
#define QT_GL_WINDOW_H

#include "Graphic-pkg.h"
#include "qtOpenGL/glwidget.hpp"

#ifdef MOVE3D_CORE
#include "qtLibrary.hpp"
#endif

/**
  * @ingroup qtOldWidget
  * @brief Open GL widget containing the qtOpenGL viewer implemetation in Qt
  */
class qtGLWindow: public QWidget
{
Q_OBJECT

public:
	qtGLWindow();

	GLWidget* getOpenGLWidget() { return glWidget; }

private:
	void createCheckBoxes();
	QSlider *createSlider();

	GLWidget *glWidget;

	QSlider *xSlider;
	QSlider *ySlider;
	QSlider *zSlider;
	QSlider *zoom;

	QCheckBox* wcop; // "Copy"
	QCheckBox* vsav; // "Save\nView"
	QCheckBox* vres; // "Restore\n View"
	QCheckBox* vfil; // "Poly/\nLine"
	QCheckBox* vcont; // "Contours"
	QCheckBox* vGhost; //"Ghost"
	QCheckBox* vBb; // "BB"
	QCheckBox* vgour; // "Smooth"
	QCheckBox* wfree; // "Freeze"
	QCheckBox* mcamera; // "Mobile\n Camera"
	QCheckBox* done; // "Done"
	QCheckBox* unselect; // "Unselect\n Joint"

	QCheckBox* opfloor; //"Floor"
	QCheckBox* optiles; // "Tiles"
	QCheckBox* walls; // "Walls"
	QCheckBox* shadows; // "Shadows"

	G3D_Window* win;

private slots:
	void setBoolGhost(bool value);
	void setBoolBb(bool value);
	void setBoolFloor(bool value);
	void setBoolTiles(bool value);
	void setBoolWalls(bool value);
	void setBoolShadows(bool value);

};

#endif

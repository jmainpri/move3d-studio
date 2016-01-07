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
/*
 * histoWin.hpp
 *
 *  Created on: Sep 14, 2009
 *      Author: jmainpri
 */

#ifndef HISTOWIN_HPP_
#define HISTOWIN_HPP_

#include <qwt_plot.h>
#include <qmainwindow.h>

#include <qwt_point_3d.h>
#include <qwt_compat.h>

/**
  * @ingroup qtWindow
  * @defgroup qtPlot
  * This modules relies on qwt which is an open source library that relies on the C++ Qt library,
  * it implements such wigets as Plots, Histogram, logarithmic sliders and other scientific interfaces
  * The example code plots the trajectory cost
  \code
    BasicPlot* myPlot = new BasicPlot(this->plot);

    Trajectory* traj = currentTrajPt;

    int nbSample = myPlot->getPlotSize();
    double step = traj->getParamMax() / (double) nbSample;

    vector<double> cost;

    for( double param=0; param<traj.getParamMax(); param = param + step)
    {
        cost.push_back(traj->configAtParam(param)->cost());
    }

    myPlot->setData(cost);
    myPlot->show();
   \endcode
  */

/**
  * @ingroup qtPlot
  * @brief Qt Histogram widget relies on qwt
  */

class HistoWindow  {

public:
	HistoWindow();

	void startWindow();

private:
    QwtPlot* plot;
};

#endif /* HISTOWIN_HPP_ */

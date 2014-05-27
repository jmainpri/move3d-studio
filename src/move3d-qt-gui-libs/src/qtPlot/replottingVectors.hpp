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
#ifndef _REPLOTTING_PLOT_H
#define _REPLOTTING_PLOT_H

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <vector>

const int REPLOT_SIZE = 100;      // 0 to 200

/**
 * @ingroup qtPlot
 * @brief Qt simple plot relies on qwt
 */
class ReplottingVectors : public QwtPlot
{
    Q_OBJECT

public:
    ReplottingVectors(QWidget* = NULL);
    int getPlotSize() { return REPLOT_SIZE; }

    void addData(const std::vector<const std::vector<double>* >& data);
    void clearData();

    void setTimerInterval(double interval);

protected:
    virtual void timerEvent(QTimerEvent *e);

private:
    void alignScales();

    std::vector< QwtPlotCurve* > cData;
    std::vector<double> Max_y;
    QwtArray< double >								d_x;
    std::vector< QwtArray< double > > d_y;
    std::vector< const std::vector<double>* > m_data;

    bool init;
    double m_Max_y;
    double m_Min_y;

    int d_interval; // timer in ms
    int d_timerId;
};

#endif

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
#ifndef niutWIDGET_H
#define niutWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"

namespace Ui {
    class niutWidget;
    class ParamWidget;
}

class niutWidget : public QWidget 
{
    Q_OBJECT
public:
    niutWidget(QWidget *parent = 0);
    ~niutWidget();
    void init(PosterReader *pr, Ui::ParamWidget *m_ui_parent);
    bool isToHideBar();

public slots:
    void setNiutIsAlive(bool state);
    void setNiutColorLabel(int idLabel, int color);

protected:
    void changeEvent(QEvent *e);

private:
    PosterReader *m_pr;
    Ui::ParamWidget *m_ui_p;
    Ui::niutWidget *m_ui;
    
    std::vector<QLabel*> _niutLabels;
    QPixmap _niutPmAlive;
    QPixmap _niutPmDead;
    QPixmap _niutPmRed;
    QPixmap _niutPmOrange;
    QPixmap _niutPmYellow;
    QPixmap _niutPmGreen;

    std::vector<bool> _showedPics;
    std::vector<int> _idDict;

};

#endif // niutWIDGET_H

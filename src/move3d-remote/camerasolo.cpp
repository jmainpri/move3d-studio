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
#include "camerasolo.hpp"
#include "ui_camerasolo.h"

CameraSolo::CameraSolo(PosterReader *pr,  Ui::MainWindowRemote *m_ui_parent, QWidget *parent) :
    QWidget(parent),
    m_pr(pr),
    m_ui_p(m_ui_parent),
    m_ui(new Ui::CameraSolo)
{
    m_ui->setupUi(this);
    _qimage = NULL;

    _dataImage =NULL;

    _labelImage = m_ui->labelImage;

    connect( m_pr->_picowebLeftImg, SIGNAL(imageReady()), this,SLOT(updateImage()));
}

CameraSolo::~CameraSolo()
{
    if(_qimage)
    {
        delete _qimage;
    }
    delete m_ui;
}

void CameraSolo::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void CameraSolo::updateImage()
{
    m_ui->labelImage->setPixmap(m_pr->_picowebLeftImg->image().scaled(QSize(400, 400),Qt::KeepAspectRatio,Qt::FastTransformation));
    QSizePolicy labelSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    labelSizePolicy.setHeightForWidth(true);
    m_ui->labelImage->setSizePolicy(labelSizePolicy);
    m_ui->labelImage->setScaledContents(true);
}

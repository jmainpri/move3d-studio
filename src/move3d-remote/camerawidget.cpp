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
#include "camerawidget.h"
#include "ui_camerawidget.h"

#include <iostream>

using namespace std;

cameraWidget::cameraWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::cameraWidget)
{
    m_ui->setupUi(this);
}

void cameraWidget::init(PosterReader *pr, Ui::ParamWidget *ui_param)
{
  if( (!pr) || (!ui_param))
  {
    cout << "cameraWidget not well initialized!!!" << endl;
    return;
  }
  
  m_pr = pr;
  m_ui_p = ui_param;

  _qimageLeft = NULL;
  _qimageRight = NULL;
  _dataImageLeft = NULL;
  _dataImageRight = NULL;
  _labelImageLeft = m_ui->labelImageLeft;
  _labelImageRight = m_ui->labelImageRight;
  
  connect( m_pr->_picowebLeftImg, SIGNAL(imageReady()), this,SLOT(updateImageLeft()));
  connect( m_pr->_picowebRightImg, SIGNAL(imageReady()), this,SLOT(updateImageRight()));
}

cameraWidget::~cameraWidget()
{
    if(_qimageLeft)
    {
        delete _qimageLeft;
    }
    if(_qimageRight)
    {
        delete _qimageRight;
    }
    delete m_ui;
}

void cameraWidget::changeEvent(QEvent *e)
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

void cameraWidget::updateImageLeft()
{
    m_ui->labelImageLeft->setPixmap(m_pr->_picowebLeftImg->image().scaled(QSize(400, 400),
                                                                          Qt::KeepAspectRatio,Qt::FastTransformation));
    QSizePolicy labelSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    labelSizePolicy.setHeightForWidth(true);
    m_ui->labelImageLeft->setSizePolicy(labelSizePolicy);
    m_ui->labelImageLeft->setScaledContents(true);
}

void cameraWidget::updateImageRight()
{
    m_ui->labelImageRight->setPixmap(m_pr->_picowebRightImg->image().scaled(QSize(400, 400),
                                                                            Qt::KeepAspectRatio,Qt::FastTransformation));
    QSizePolicy labelSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    labelSizePolicy.setHeightForWidth(true);
    m_ui->labelImageRight->setSizePolicy(labelSizePolicy);
    m_ui->labelImageRight->setScaledContents(true);
}



void cameraWidget::on_checkBox_toggled(bool checked)
{
    if(checked)
    {
        m_pr->_picowebLeftImg->start();
        m_pr->_picowebRightImg->start();
    }else{
        m_pr->_picowebLeftImg->stop();
        m_pr->_picowebRightImg->stop();
    }
}




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



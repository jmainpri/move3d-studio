#include "camerawidget.h"
#include "ui_camerawidget.h"

#include <iostream>

using namespace std;

cameraWidget::cameraWidget(PosterReader *pr,  Ui::MainWindowRemote *m_ui_parent, QWidget *parent) :
    QWidget(parent),
    m_pr(pr),
    m_ui_p(m_ui_parent),
    m_ui(new Ui::cameraWidget)
{
    m_ui->setupUi(this);
  
  _qimageLeft = NULL;
  _qimageRight = NULL;
  _dataImageLeft = NULL;
  _dataImageRight = NULL;
  _labelImageLeft = m_ui->labelImageLeft;
  _labelImageRight = m_ui->labelImageRight;
  
  init(m_pr,m_ui_p);
}

void cameraWidget::init(PosterReader *pr, Ui::MainWindowRemote *ui_parent)
{
  if( (!pr) || (!ui_parent))
  {
    cout << "cameraWidget not well initialized!!!" << endl;
    return;
  }
  
  m_pr = pr;
  m_ui_p = ui_parent;
  
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
    m_ui->labelImageLeft->setPixmap(m_pr->_picowebLeftImg->image().scaled(QSize(400, 400),Qt::KeepAspectRatio,Qt::FastTransformation));
    QSizePolicy labelSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    labelSizePolicy.setHeightForWidth(true);
    m_ui->labelImageLeft->setSizePolicy(labelSizePolicy);
    m_ui->labelImageLeft->setScaledContents(true);
}

void cameraWidget::updateImageRight()
{
    m_ui->labelImageRight->setPixmap(m_pr->_picowebRightImg->image().scaled(QSize(400, 400),Qt::KeepAspectRatio,Qt::FastTransformation));
    QSizePolicy labelSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    labelSizePolicy.setHeightForWidth(true);
    m_ui->labelImageRight->setSizePolicy(labelSizePolicy);
    m_ui->labelImageRight->setScaledContents(true);
}



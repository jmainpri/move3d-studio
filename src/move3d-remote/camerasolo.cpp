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

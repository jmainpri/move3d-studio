#include "niutwidget.h"
#include "ui_niutwidget.h"
#include "images/green-man-xpm.h"
#include "images/orange-man-xpm.h"
#include "images/red-man-xpm.h"
#include "images/yellow-death-xpm.h"
#include "images/yellow-man-xpm.h"
#include "images/im_still_alive-xpm.h"
#include <iostream>

using namespace std;

niutWidget::niutWidget(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent, QWidget *parent) :
        m_pr(pr),
        QWidget(parent),
        m_ui_p(m_ui_parent),
        m_ui(new Ui::niutWidget)
{
    m_ui->setupUi(this);

    initNiut();
    connect(m_pr,SIGNAL(niutIsAlive(bool)), this, SLOT(setNiutIsAlive(bool)));
    connect(m_pr,SIGNAL(setNiutColorLabel(int,int)), this, SLOT(setNiutColorLabel(int,int)));
}

niutWidget::~niutWidget()
{
    delete m_ui;
}

void niutWidget::changeEvent(QEvent *e)
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

void niutWidget::initNiut()
{
    _niutLabels.push_back( m_ui->labelNiut1 );
    _niutLabels.push_back( m_ui->labelNiut2 );
    _niutLabels.push_back( m_ui->labelNiut3 );
    _niutLabels.push_back( m_ui->labelNiut4 );
    _niutLabels.push_back( m_ui->labelNiut5 );
    _niutLabels.push_back( m_ui->labelNiut6 );
    _niutLabels.push_back( m_ui->labelNiut7 );
    _niutLabels.push_back( m_ui->labelNiut8 );

    _niutLabels.push_back( m_ui->labelNiut9 );
    _niutLabels.push_back( m_ui->labelNiut10 );
    _niutLabels.push_back( m_ui->labelNiut11 );
    _niutLabels.push_back( m_ui->labelNiut12 );
    _niutLabels.push_back( m_ui->labelNiut13 );
    _niutLabels.push_back( m_ui->labelNiut14 );
    _niutLabels.push_back( m_ui->labelNiut15 );
    _niutLabels.push_back( m_ui->labelNiut16 );

    _niutPmAlive = QPixmap(im_still_alive_img);
    _niutPmDead = QPixmap(yellow_death);
    _niutPmRed = QPixmap(red_man);
    _niutPmOrange = QPixmap(orange_man);
    _niutPmYellow = QPixmap(yellow_man);
    _niutPmGreen = QPixmap(green_man);

    _niutPmRed = _niutPmRed.scaledToHeight(70);
    _niutPmOrange = _niutPmOrange.scaledToHeight(70);
    _niutPmYellow = _niutPmYellow.scaledToHeight(70);
    _niutPmGreen = _niutPmGreen.scaledToHeight(70);

    _niutPmAlive = _niutPmAlive.scaledToHeight(70);

    for (unsigned int i=0; i<_niutLabels.size(); i++)
    {
        _niutLabels[i]->setPixmap(_niutPmRed);
        _niutLabels[i]->show();
    }
    m_ui->labelNiutDead->setPixmap(_niutPmAlive);
    m_ui->labelNiutDead->show();
}

void niutWidget::setNiutIsAlive(bool state)
{
    if(state) {
        m_ui->labelNiutDead->setPixmap(_niutPmAlive);
    } else {
        m_ui->labelNiutDead->setPixmap(_niutPmDead);
    }
}

void niutWidget::setNiutColorLabel(int id, int color)
{
    if (id<0 || id>=((int)_niutLabels.size())) {
        cout << "id : " << id << " color : " << color << "lable.size : "<< (int)_niutLabels.size()<< endl;
        cout << "Error in " << __FILE__ << __func__ << endl;
        return;
    }

    switch (color) {
    case 0:
        _niutLabels[id]->setPixmap(_niutPmRed);
        break;

    case 1:
        _niutLabels[id]->setPixmap(_niutPmOrange);
        break;

    case 2:
        _niutLabels[id]->setPixmap(_niutPmYellow);
        break;

    case 3:
        _niutLabels[id]->setPixmap(_niutPmGreen);
        break;

    default:
        break;
    }
}

#include "softmotionwidget.hpp"
#include "ui_softmotionwidget.h"
#include <iostream>


#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"


using namespace std;

softmotionWidget::softmotionWidget(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent, QWidget *parent) :
        QWidget(parent),
        m_pr(pr),
        m_ui_p(m_ui_parent),
        m_ui(new Ui::softmotionWidget)
{
    m_ui->setupUi(this);

}


softmotionWidget::~softmotionWidget()
{
    delete m_ui;
}



void softmotionWidget::init(PosterReader *pr, Ui::MainWindowRemote *ui_parent)
{
  if( (!pr) || (!ui_parent))
  {
    cout << "niutWidget not well initialized!!!" << endl;
    return;
  }


}

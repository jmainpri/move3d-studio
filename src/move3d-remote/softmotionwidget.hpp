#ifndef SOFTMOTIONWIDGET_HPP
#define SOFTMOTIONWIDGET_HPP

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"

namespace Ui {
    class softmotionWidget;
    class ParamWidget;
}

class softmotionWidget : public QWidget
{
    Q_OBJECT
public:
    softmotionWidget(QWidget *parent = 0);
    ~softmotionWidget();
    void init(PosterReader *pr, Ui::ParamWidget *m_ui_parent);



private:
    PosterReader *m_pr;
    Ui::ParamWidget *m_ui_p;
    Ui::softmotionWidget *m_ui;





private slots:
    void on_checkBoxSoftMotionDrawTraj_toggled(bool checked);
    void on_checkBoxMonitoringSphere_toggled(bool checked);
};

#endif // SOFTMOTIONWIDGET_HPP



#ifndef SOFTMOTIONWIDGET_HPP
#define SOFTMOTIONWIDGET_HPP

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"
#include "ui_mainwindow-remote.h"

namespace Ui {
    class softmotionWidget;
}

class softmotionWidget : public QWidget
{
    Q_OBJECT
public:
    softmotionWidget(PosterReader *pr = NULL, Ui::MainWindowRemote *m_ui_parent = NULL, QWidget *parent = 0);
    ~softmotionWidget();
    void init(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent);

private:
    PosterReader *m_pr;
    Ui::MainWindowRemote *m_ui_p;
    Ui::softmotionWidget *m_ui;


};

#endif // SOFTMOTIONWIDGET_HPP



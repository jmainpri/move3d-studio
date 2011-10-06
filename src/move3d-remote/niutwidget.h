#ifndef niutWIDGET_H
#define niutWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"
#include "ui_mainwindow-remote.h"

namespace Ui {
    class niutWidget;
}

class niutWidget : public QWidget {
    Q_OBJECT
public:
    niutWidget(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent, QWidget *parent = 0);
    ~niutWidget();

public slots:
    void setNiutIsAlive(bool state);
    void setNiutColorLabel(int idLabel, int color);

protected:
    void changeEvent(QEvent *e);

private:
    Ui::niutWidget *m_ui;
    Ui::MainWindowRemote *m_ui_p;
    PosterReader *m_pr;
    std::vector<QLabel*> _niutLabels;
    QPixmap _niutPmAlive;
    QPixmap _niutPmDead;
    QPixmap _niutPmRed;
    QPixmap _niutPmOrange;
    QPixmap _niutPmYellow;
    QPixmap _niutPmGreen;

    void initNiut();
};

#endif // niutWIDGET_H

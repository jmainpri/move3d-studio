#ifndef QTMOVINGHUMAN_HPP
#define QTMOVINGHUMAN_HPP

#include <QWidget>

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtBase/SpinBoxSliderConnector_p.hpp"

namespace Ui {
    class MovingHuman;
}

class MovingHuman : public QWidget {
    Q_OBJECT
public:
    MovingHuman(QWidget *parent = 0);
    MovingHuman(double x, double y, double rz, QWidget *parent = 0);
    ~MovingHuman();

    void init(double x, double y, double rz);
    void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MovingHuman *ui;
    MainWindow* m_mainWindow;

    QtShiva::SpinBoxSliderConnector*	m_k_x;
    QtShiva::SpinBoxSliderConnector*	m_k_y;
    QtShiva::SpinBoxSliderConnector*	m_k_rz;

private slots:
    void on_MovingHuman_destroyed();
    void updateMainWindow(double );
};

#endif // QTMOVINGHUMAN_HPP

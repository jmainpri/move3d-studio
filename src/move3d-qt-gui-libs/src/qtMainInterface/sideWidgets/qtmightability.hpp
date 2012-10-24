#ifndef QTMIGHTABILITY_H
#define QTMIGHTABILITY_H

#include <QWidget>

namespace Ui {
    class qtMightability;
}

class qtMightability : public QWidget {
    Q_OBJECT
public:
    qtMightability(QWidget *parent = 0);
    ~qtMightability();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::qtMightability *ui;

private slots:
    void on_pushButtonTaskability_clicked();
    void on_pushButtonInitialiazeComputeMM_clicked();
    void on_pushButtonInitialiazePrepareState_clicked();
    void on_pushButtonInitialiazeCreateAgents_clicked();
    void on_checkBox_toggled(bool checked);
    void on_pushButtonInitialiaze_clicked();
};

#endif // QTMIGHTABILITY_H

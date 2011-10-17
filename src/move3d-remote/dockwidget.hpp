#ifndef DockWindow_HPP
#define DockWindow_HPP

#include <QWidget>

#include "qtLibrary.hpp"
#include "posterreader.hpp"
#include "ui_mainwindow-remote.h"

namespace Ui
{
  class DockWindow;
}

/**
 * @ingroup qtDockWindow
 * @brief Qt Main Window container
 * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
 \image html Designer.png
 */
class DockWindow : public QWidget
{
  Q_OBJECT
  
public:
  DockWindow(PosterReader *pr = NULL, Ui::MainWindowRemote *ui_parent = NULL, QWidget *parent = 0);
  ~DockWindow();
  
private:
  Ui::DockWindow *m_ui;
};

#endif // DockWindow_HPP

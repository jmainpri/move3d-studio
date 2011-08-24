/*
 *  pocolibsPoster.hpp
 *  Move3D-Qt-Gui
 *
 *  Created by Jim Mainprice on 28/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtLibrary.hpp"

#include "mainwindow.hpp" 

class FetchEnvironment : public QObject
{
  Q_OBJECT
  
public:
  //! Initializes a timer and 
  //! finds the emiting Environment Poster
  FetchEnvironment(QWidget* = NULL);
  
  //! Initializes the poster
  //! returns true if OK
  bool init(MainWindow* win);
  
protected slots:
  //! Reads a pocolib poster
  //! Sets every robot in Move3D to match the poster description
  //! @return true if the operation has succeded
  bool refresh();

private:
  //! Display functions
  MainWindow* m_win;

};

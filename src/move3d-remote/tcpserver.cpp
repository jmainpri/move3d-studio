#include "tcpserver.hpp"
#include <QString>

#include "qtOpenGL/glwidget.hpp"
#include "P3d-pkg.h"

#include <iostream>

using namespace std;

TcpServer::TcpServer(QObject *parent)
{
  listen(QHostAddress::LocalHost,50007);
  QObject:: connect(this, SIGNAL(newConnection()),
                    this, SLOT(demande_connexion()));
  cout << "server listenning" << endl;
}

TcpServer::~TcpServer()
{
  cout << "tcp server died"  << endl;
}

// si un client demande une connexion
void TcpServer::demande_connexion()
{
  //  emit vers_IHM_connexion(); // on envoie un signal à l'IHM
  // on crée une nouvelle socket pour ce client
  clientConnection = nextPendingConnection();
  // si on reçoit des données, le slot lecture() est appelé
  QObject::connect(clientConnection, SIGNAL(readyRead()), this, SLOT(lecture()));
}

void TcpServer::lecture()
{
  cout << "TCP read" << endl;
  QString ligne;
  
  cout << "can read " << clientConnection->canReadLine() << endl;
  
  while(clientConnection->canReadLine())    // tant qu'on peut lire sur la socket
  {
    ligne = clientConnection->readAll(); // on lit une ligne
    cout << "TCP MESSAGE : " << ligne.toStdString() << endl;
  }
  
  std::string msg = ligne.toStdString();
  
  if( msg == "hello\n" )
  {
    cout << "HELLO world OK!!!" << endl;
  }
  
  if( msg == "changeCamera\n" )
  {
    changeCamera(true);
  }
  
  if( msg == "resetCamera\n" )
  {
   changeCamera(false);
  }
  
  QTextStream texte(clientConnection); 
  texte << "new message received : " << ligne << endl;
  cout << "END READING" << endl;
}

void TcpServer::changeCamera(bool reset)
{
  G3D_Window *win = qt_get_cur_g3d_win();

  if( reset )
  {
    p3d_rob* r = (p3d_rob *) p3d_get_robot_by_name("PR2_ROBOT");
    p3d_jnt* j = r->joints[3];
    p3d_matrix4* m_transf = &(j->abs_pos);
    p3d_matrix4 T1,T2,T3,offset;
    
    p3d_mat4Copy(*m_transf,offset);
//    p3d_mat4Pos(T1, 0, 0, 0, 0, 0.0, 0.4);
    p3d_mat4Pos(T1, 0, 0, 0, 0, 0.4, 0.0);
    p3d_mat4Pos(T2, -1.5, 0, 0, 0, 0, 0);
    p3d_mat4Mult(T1,T2,T3);
    p3d_mat4Mult(*m_transf,T3,offset);
    
    cout << "Change camera" << endl;
    g3d_save_win_camera(win->vs);
    g3d_set_camera_parameters_from_frame(offset, win->vs);
    g3d_set_projection_matrix(win->vs.projection_mode);
    qt_change_mob_frame(win,m_transf);
  }
  else
  {
    cout << "Reset camera" << endl;
    qt_reset_mob_frame(win);
    g3d_restore_win_camera(win->vs);
  }
}

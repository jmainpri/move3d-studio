/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "tcpserver.hpp"
#include "sparkcamera.hpp"

#include <QString>
#include <iostream>

using namespace std;

TcpServer::TcpServer(QObject *parent)
{
  listen(QHostAddress::Any,50007);
  
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
  spark_camera_change(reset);
}

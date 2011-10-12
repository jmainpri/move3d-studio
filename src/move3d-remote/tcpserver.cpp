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

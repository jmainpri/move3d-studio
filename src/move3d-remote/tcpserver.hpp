#ifndef TCPSERVER_HPP
#define TCPSERVER_HPP

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

class TcpServer : public QTcpServer
{
  Q_OBJECT
  
public :
  TcpServer(QObject *parent=0);
  ~TcpServer();
  
private slots :
  void demande_connexion() ;
  void lecture();
  
signals :
  void vers_IHM_connexion();
  void vers_IHM_texte(QString);
  
private :
  QTcpSocket *clientConnection;
  
  void changeCamera(bool reset);
};



#endif // TCPSERVER_HPP

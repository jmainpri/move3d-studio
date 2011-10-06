#include "tcpserver.hpp"


#include <QString>

#include <iostream>

using namespace std;
TcpServer :: TcpServer (QObject *parent)
{
    listen(QHostAddress::Any,50007);
    QObject:: connect(this, SIGNAL(newConnection()),
                      this, SLOT(demande_connexion()));
    cout << "server listenning "<< endl;
}

TcpServer::~TcpServer()
{
    cout << "mouru " << endl;
}

// si un client demande une connexion
void TcpServer :: demande_connexion()
{
    emit vers_IHM_connexion(); // on envoie un signal à l'IHM
    // on crée une nouvelle socket pour ce client
    clientConnection = nextPendingConnection();
    // si on reçoit des données, le slot lecture() est appelé
    QObject:: connect(clientConnection, SIGNAL(readyRead()), this, SLOT(lecture()));
}

void TcpServer ::lecture()
{
    cout << "LECTURE" << endl;
    QString ligne;

    cout << "can read " << clientConnection->canReadLine() << endl;
    while(clientConnection->canReadLine())    // tant qu'on peut lire sur la socket
    {
        ligne = clientConnection->readAll(); // on lit une ligne
        cout << "MESSAGE : " << ligne.toStdString() << endl;
        emit vers_IHM_texte(ligne);           // on l'envoie à l'IHM
    }
          cout << "MESSAGE : " << ligne.toStdString() << endl;
    QTextStream texte(clientConnection);      // création d'un flux pour écrire dans la socket
    texte << "nouveau message recu " << ligne << endl;          // message à envoyer au client


     cout << "FIN LECTURE" << endl;
}

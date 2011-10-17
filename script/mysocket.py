#!/usr/bin/python
#Echo server program
import socket

HOST = 'localhost'    # The remote host
PORT = 50007          # The same port as used by the server

def move3d_remote_hello() :
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send('hello\n')
    data = s.recv(1024)
    s.close()
    print 'Received', repr(data)

def move3d_remote_change_camera() :
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send('changeCamera\n')
    data = s.recv(1024)
    s.close()
    print 'Received', repr(data)
    
def move3d_remote_reset_camera() :
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send('resetCamera\n')
    data = s.recv(1024)
    s.close()
    print 'Received', repr(data)
    

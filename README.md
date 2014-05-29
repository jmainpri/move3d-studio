move3d-studio
=============

Graphical user interface for libmove3d

### Stand alone install

Depends on libmove3d and libmove3d-hri

    mkdir build && cd build
    ccmake .. -DMOVE3D_QT=ON -DUSE_QWT=OFF

Choose the type of build you want (MOVE3D_QT builds the original Qt interface)    

    make install
    
### Full install

See move3d-launch: https://github.com/jmainpri/move3d-launch

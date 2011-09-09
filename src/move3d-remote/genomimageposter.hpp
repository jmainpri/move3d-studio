#ifndef GENOMIMAGEPOSTER_HPP
#define GENOMIMAGEPOSTER_HPP

#include "genomposter.hpp"

#include "duplicatedGenomStruct.hpp"

#include <stdio.h>
#include "myopencv.hpp"

class GenomImagePoster : public GenomPoster
{
     Q_OBJECT
public:
    GenomImagePoster(std::string name, unsigned long rate);
    ~GenomImagePoster();

    IplImage * iplImgLeft(){return _iplImgLeft;}
    IplImage * iplImgRight(){return _iplImgRight;}

protected:
    void update();

private:
     ViamImageBank *_viamImageBank;
     IplImage * _iplImgLeft;
     IplImage * _iplImgRight;
     bool _posterTaked;

     bool myPosterTake();
     bool myPosterGive();
};

#endif // GENOMIMAGEPOSTER_HPP

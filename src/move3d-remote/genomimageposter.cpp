#include "genomimageposter.hpp"
#include <iostream>

using namespace std;

GenomImagePoster::GenomImagePoster(std::string name, unsigned long rate) :
        GenomPoster(name, (char*)(_viamImageBank), sizeof(ViamImageBank), rate)
{
    _viamImageBank = NULL;
    _iplImgLeft =  NULL;
    _iplImgRight =  NULL;
    _posterTaked = false;
}

GenomImagePoster::~GenomImagePoster()
{
    myPosterGive();
    cout << "INFO: ~GenomImagePoster() posterGive OK" << endl;
}

bool GenomImagePoster::myPosterTake()
{
    if(_posterTaked == false)
    {
        cout << "try to take the poster " << endl;
        if (posterTake(_posterID, POSTER_READ) != OK )
        {
            cout << " posterTake error " << endl;
            return false;
        }
    }
    cout << " poster taked " << endl;
    _posterTaked = true;
    return true;
}

bool GenomImagePoster::myPosterGive()
{
    if(_posterTaked == true)
    {
        posterGive(_posterID);
    }
    _posterTaked = false;
    return true;
}

void GenomImagePoster::update() {

        cout << " update from GenomImagePoster" << endl;
    if(_posterID == NULL)
    {
        if(findPoster() == false)
        {
            _updatingStatus = false;
        }
    } else {

        if (myPosterTake() == false)
        {
            return;
        }
    cout << " get the poster address" << endl;
        _viamImageBank =(ViamImageBank *)posterAddr(_posterID);

        if(_viamImageBank == NULL)
        {
            myPosterGive();
            cout << " poster viam is NULL" << endl;
            return;
        }
        if(_viamImageBank->nImages <= 0)
        {
            printf("There is no image acquired!\n");
            myPosterGive();
            return;
        }

        if(_iplImgLeft == NULL)
        {
            cout << " image[0] width=" <<_viamImageBank->image[0].width << ", height=" << _viamImageBank->image[0].height << ", size= " << _viamImageBank->image[0].imageSize << endl;
            _iplImgLeft   = cvCreateImage(cvSize(_viamImageBank->image[0].width, _viamImageBank->image[0].height), 8, 3);
        }

        memcpy(_iplImgLeft->imageData, _viamImageBank->image[0].data+_viamImageBank->image[0].dataOffset,_viamImageBank->image[0].imageSize);

        if(_viamImageBank->nImages > 1 &&  _iplImgRight == NULL)
        {
            _iplImgRight   = cvCreateImage(cvSize(_viamImageBank->image[1].width, _viamImageBank->image[1].height), 8, 3);
        }
        if(_viamImageBank->nImages > 1)
        {
            memcpy(_iplImgRight->imageData, _viamImageBank->image[1].data+_viamImageBank->image[1].dataOffset,_viamImageBank->image[1].imageSize);
        }
        myPosterGive();
    }
    return;
}



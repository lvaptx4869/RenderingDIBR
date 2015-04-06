#include "cv.h"
#include "highgui.h"

//////////////////////////////////////////////////////////////////////
// 
// Warning: Use code at your own risk
//
// Functions to read calibration parameters, 
// convert values in depth images to real Z,
// and project given camera matrices and Z
//
//////////////////////////////////////////////////////////////////////
// this should really be placed in a header file...

typedef struct {
    double m_K[3][3]; // 3x3 intrinsic matrix
    double m_RotMatrix[3][3]; // rotation matrix
    double m_Trans[3]; // translation vector

    double m_ProjMatrix[4][4]; // projection matrix
} CalibStruct;
// globals (for illustration only)
 //CalibStruct m_CalibParams[8];
 //int m_NumCameras = 8;
 //int m_Width = 1024, m_Height = 768; // camera resolution is 1024x768

//////////////////////////////////////////////////////////////////////
// The four function definitions are the only ones necessary if 
// depths and reprojections are required.
// You can easily write your version of epipolar extraction 
//(for stereo) from the code.

void readCalibrationFile(char *fileName);
void computeProjectionMatrices();
void InitializeFromFile(char *fileName);
double DepthLevelToZ( unsigned char d );
double projXYZtoUV(double projMatrix[4][4], double x, double y, double z, double *u, double *v);
void projUVZtoXY(double projMatrix[4][4], double u, double v, double z, double *x, double *y);

//////////////////////////////////////////////////////////////////////
// This function is to illustrate how to use the projection functions

void TestProject(double pts[8][2], int u, int v, unsigned char d);

//////////////////////////////////////////////////////////////////////

// API for the Lvmc
int ShowMain();
//void LoadTextureAndDepthMap(char *TexturePath , char *DepthPath, IplImage *Texture , IplImage *Depth);
void ShowImagePair(IplImage *srcTexture, IplImage * srcDepth);
void DepthToZmap(IplImage *DepthMap,IplImage *Zmap, double MaxZ ,double MinZ);
void ForwardWarping(IplImage *srcTexture, IplImage * srcDepth, IplImage *dstTexture,IplImage *dstDepth,int srcCam,int dstCam);
void InverseWarping(IplImage *dstTexture, IplImage * dstDepth, IplImage *srcTexture,IplImage *srcDepth,int dstCam,int srcCam);
void WarpingProcessing(IplImage *srcTexture, IplImage * srcDepth, IplImage *dstTexture,IplImage *dstDepth,int srcCam,int dstCam);
void TestWarping(char *TexturePath , char *DepthPath);
void ImageBlending(IplImage *TextureL,IplImage *TexutreR,IplImage *BlendImg, double alpha);

void TestSplitDepth(char *DepthPath);
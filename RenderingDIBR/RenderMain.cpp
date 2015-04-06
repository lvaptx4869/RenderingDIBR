#include "Rendering.h"
extern CalibStruct m_CalibParams[8];

int main()
{
	//double pts[8][2];
	//TestProject(pts, int u, int v, unsigned char d);
	InitializeFromFile("calibParams-ballet.txt");
	// load the left image data
	int srcCamL = 3; //cam 0 1 2 3 4 5 6 7 
	int dstCam = 4;
	int srcCamR = 5;

	char *TexturePathL = "F:\\Research\\CVprojects\\Codes\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam3\\color-cam3-f000.jpg";
	char *DepthPathL = "F:\\Research\\CVprojects\\Codes\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam3\\depth-cam3-f000.png";
	IplImage* TextureL = cvLoadImage(TexturePathL);
	IplImage* DepthL = cvLoadImage(DepthPathL);
	IplImage *dstTextureL = cvCreateImage(cvGetSize(TextureL),IPL_DEPTH_8U,3);
	IplImage *dstDepthL = cvCreateImage(cvGetSize(DepthL),IPL_DEPTH_32F,1);
	WarpingProcessing(TextureL, DepthL, dstTextureL,dstDepthL, srcCamL, dstCam);

	//load the right image data
	char *TexturePathR = "F:\\Research\\CVprojects\\Codes\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam5\\color-cam5-f000.jpg";
	char *DepthPathR = "F:\\Research\\CVprojects\\Codes\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam5\\depth-cam5-f000.png";
	IplImage* TextureR = cvLoadImage(TexturePathR);
	IplImage* DepthR = cvLoadImage(DepthPathR);
	IplImage *dstTextureR = cvCreateImage(cvGetSize(TextureR),IPL_DEPTH_8U,3);
	IplImage *dstDepthR = cvCreateImage(cvGetSize(DepthR),IPL_DEPTH_32F,1);
	WarpingProcessing(TextureR, DepthR, dstTextureR,dstDepthR, srcCamR, dstCam);

	ShowImagePair(dstTextureL,dstTextureR);

	double alpha = 0.5;
	IplImage *RenderTexture = cvCreateImage(cvGetSize(TextureL),IPL_DEPTH_32F,3);
	cvZero(RenderTexture);
	ImageBlending(dstTextureL,dstTextureR,RenderTexture, alpha);
	ShowImagePair(dstTextureL,RenderTexture);
		cvSaveImage("RenderTexture.bmp",RenderTexture);
	//cvAddWeighted(dstTextureL,1.0/2.0,dstTextureR, 1./2.,0.0,dstTextureR);
	//ShowImagePair(dstTextureL,RenderTexture);

	//load the virtual view of ground truth
	char *TexturePath = "F:\\Research\\CVprojects\\Codes\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam4\\color-cam4-f000.jpg";
	char *DepthPath = "F:\\Research\\CVprojects\\Codes\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam4\\depth-cam4-f000.png";
	IplImage* Texture = cvLoadImage(TexturePath);
	IplImage* Depth = cvLoadImage(DepthPath);
	//TestWarping(TexturePath ,DepthPath);
	//TestSplitDepth(DepthPath);
	//ShowMain();
	cvReleaseImage(&TextureL);
	cvReleaseImage(&DepthL);
	cvReleaseImage(&TextureR);
	cvReleaseImage(&DepthR);
	cvReleaseImage(&Texture);
	cvReleaseImage(&Depth);
	return 1;
}
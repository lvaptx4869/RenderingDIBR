#include "Rendering.h"
// d is pixel value in the depth images. 
// d represents depth, but is NOT the actual depth value.
// This function returns actual z (depth)


 //CalibStruct m_CalibParams[8];
 //int m_NumCameras = 8;
 //int m_Width = 1024, m_Height = 768; // camera resolution is 1024x768

#define DEBUG 1

CalibStruct m_CalibParams[8];
int m_NumCameras = 8;
int m_Width = 1024, m_Height = 768; // camera resolution is 1024x768
void readCalibrationFile(char *fileName)
{
	int i, j, k;
	FILE *pIn;
	double dIn; // dummy variable
	int camIdx;
	
	if(pIn = fopen(fileName, "r"))
	{
		for(k=0; k<m_NumCameras; k++)
		{
			// camera index
			fscanf(pIn, "%d", &camIdx);

			// camera intrinsics
            for (i=0; i<3; i++)
    			fscanf(pIn, "%lf\t%lf\t%lf", &(m_CalibParams[camIdx].m_K[i][0]),
	    			&(m_CalibParams[camIdx].m_K[i][1]), &(m_CalibParams[camIdx].m_K[i][2]));

			// read barrel distortion params (assume 0)
			fscanf(pIn, "%lf", &dIn);
			fscanf(pIn, "%lf", &dIn);
			
			// read extrinsics
			for(i=0;i<3;i++)
			{
				for(j=0;j<3;j++)
				{
					fscanf(pIn, "%lf", &dIn);
					m_CalibParams[camIdx].m_RotMatrix[i][j] = dIn;
				}
				
				fscanf(pIn, "%lf", &dIn);
				m_CalibParams[camIdx].m_Trans[i] = dIn;
			}

		}
		
		fclose(pIn);
	}
} // readCalibrationFile

void computeProjectionMatrices()
{
	int i, j, k, camIdx;
	double (*inMat)[3];
	double exMat[3][4];

	for(camIdx=0; camIdx<m_NumCameras; camIdx++)
	{
		// The intrinsic matrix
        inMat = m_CalibParams[camIdx].m_K;

		// The extrinsic matrix
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				exMat[i][j] = m_CalibParams[camIdx].m_RotMatrix[i][j];
			}
		}

		for(i=0;i<3;i++)
		{
			exMat[i][3] = m_CalibParams[camIdx].m_Trans[i];
		}

		// Multiply the intrinsic matrix by the extrinsic matrix to find our projection matrix
		for(i=0;i<3;i++)
		{
			for(j=0;j<4;j++)
			{
				m_CalibParams[camIdx].m_ProjMatrix[i][j] = 0.0;

				for(k=0;k<3;k++)
				{
					m_CalibParams[camIdx].m_ProjMatrix[i][j] += inMat[i][k]*exMat[k][j];
				}
			}
		}

		m_CalibParams[camIdx].m_ProjMatrix[3][0] = 0.0;
		m_CalibParams[camIdx].m_ProjMatrix[3][1] = 0.0;
		m_CalibParams[camIdx].m_ProjMatrix[3][2] = 0.0;
		m_CalibParams[camIdx].m_ProjMatrix[3][3] = 1.0;
	}
}

void InitializeFromFile(char *fileName)
{
    readCalibrationFile(fileName);
    computeProjectionMatrices();
}

double DepthLevelToZ( unsigned char d )
{
    double z;
    double MinZ = 44.0, MaxZ = 120.0;

    z = 1.0/((d/255.0)*(1.0/MinZ - 1.0/MaxZ) + 1.0/MaxZ);
    return z;
}

void projUVZtoXY(double projMatrix[4][4], double u, double v, double z, double *x, double *y)
{
	double c0, c1, c2;

	// image (0,0) is bottom lefthand corner
	v = (double) m_Height - v - 1.0;

	c0 = z*projMatrix[0][2] + projMatrix[0][3];
	c1 = z*projMatrix[1][2] + projMatrix[1][3];
	c2 = z*projMatrix[2][2] + projMatrix[2][3];

	*y = u*(c1*projMatrix[2][0] - projMatrix[1][0]*c2) +
		v*(c2*projMatrix[0][0] - projMatrix[2][0]*c0) +
		projMatrix[1][0]*c0 - c1*projMatrix[0][0];

	*y /= v*(projMatrix[2][0]*projMatrix[0][1] - projMatrix[2][1]*projMatrix[0][0]) +
		u*(projMatrix[1][0]*projMatrix[2][1] - projMatrix[1][1]*projMatrix[2][0]) +
		projMatrix[0][0]*projMatrix[1][1] - projMatrix[1][0]*projMatrix[0][1];
		
	*x = (*y)*(projMatrix[0][1] - projMatrix[2][1]*u) + c0 - c2*u;
	*x /= projMatrix[2][0]*u - projMatrix[0][0];
} // projUVZtoXY


double projXYZtoUV(double projMatrix[4][4], double x, double y, double z, double *u, double *v)
{
	double w;

	*u = projMatrix[0][0]*x + 
		 projMatrix[0][1]*y + 
		 projMatrix[0][2]*z + 
		 projMatrix[0][3];

	*v = projMatrix[1][0]*x + 
		 projMatrix[1][1]*y + 
		 projMatrix[1][2]*z + 
		 projMatrix[1][3];

	w = projMatrix[2][0]*x + 
		projMatrix[2][1]*y + 
		projMatrix[2][2]*z + 
		projMatrix[2][3];

	*u /= w;
	*v /= w;

	// image (0,0) is bottom lefthand corner
	*v = (double) m_Height - *v - 1.0;

	return w;

} // projXYZtoUV

// test:
// Given (u,v) in camera 0 and d at (u,v) in the depth map of camera 0,
// find the projected points at the other 7 cameras
// All projected points are stored in pts
void TestProject(double pts[8][2], int u, int v, unsigned char d)
{
    double x, y, z = DepthLevelToZ(d);

    printf( "Testing projection of pt (%d,%d) in camera 0 with d = %d (z = %f) to other cameras\n",
        u, v, d, z );

    projUVZtoXY(m_CalibParams[0].m_ProjMatrix, (double)u, (double)v, z, &x, &y);
    printf( "3D pt = (%f, %f, %f) [coordinates wrt reference camera]\n", x, y, z );

    for (int cam=0; cam<8; cam++)
    {
        double *pt = pts[cam];
        
        projXYZtoUV(m_CalibParams[cam].m_ProjMatrix, x, y, z, &pt[0], &pt[1]);
        printf( "Camera %d: (%f, %f)\n", cam, pt[0], pt[1] );
    }
}
//test for cpp
int ShowMain()
{
	//double pts[8][2];
	//TestProject(pts, int u, int v, unsigned char d);
	InitializeFromFile("calibParams-ballet.txt");
	double pts[8][2];
	int u = 15;
	int v = 20; 
	unsigned char d = 65;
	double x, y, z = DepthLevelToZ(d);

    printf( "Testing projection of pt (%d,%d) in camera 0 with d = %d (z = %f) to other cameras\n",
        u, v, d, z );

    projUVZtoXY(m_CalibParams[3].m_ProjMatrix, (double)u, (double)v, z, &x, &y);
    printf( "3D pt = (%f, %f, %f) [coordinates wrt reference camera]\n", x, y, z );

    for (int cam=0; cam<8; cam++)
    {
        double *pt = pts[cam];
        
        projXYZtoUV(m_CalibParams[cam].m_ProjMatrix, x, y, z, &pt[0], &pt[1]);
        printf( "Camera %d: (%f, %f)\n", cam, pt[0], pt[1] );
    }
	return 1;
}

//void LoadTextureAndDepthMap(char *TexturePath , char *DepthPath, IplImage *Texture , IplImage *Depth)
//{
//	Texture = cvLoadImage(TexturePath);
//	Depth = cvLoadImage(DepthPath);
//	return;
//}

// show the images in the windows
void ShowImagePair(IplImage *srcTexture, IplImage * srcDepth)
{
	cvNamedWindow("Texture",1);
	cvNamedWindow("Depth",1);

	cvShowImage("Texture",srcTexture);
	cvShowImage("Depth",srcDepth);
	while(1)
	{
		if((cvWaitKey(10)&0x7f)==27)
			break;
	}
	cvDestroyWindow("Texture");
	cvDestroyWindow("Depth");
	return;
}

//convert the depth map to the Z map
void DepthToZmap(IplImage *DepthMap,IplImage *Zmap, double MaxZ ,double MinZ)
{
	double z = 0 ;
	CvScalar S;
	for(int j = 0; j< DepthMap->height;j++)
	{
		uchar *ptr = (uchar*)(DepthMap->imageData + j*DepthMap->widthStep); 
		//double *Zptr = (double*)(Zmap->imageData + j*Zmap->widthStep); 
		for(int i = 0; i< DepthMap->width;i++)
		{
			//printf("%d ",ptr[i]);
			//printf("%d ",Zptr[i]);
			S = cvGet2D(DepthMap,j,i);
			//printf("Scalar %f %f %f %f\n",S.val[0],S.val[1],S.val[2],S.val[3]);
			z = 1.0/((S.val[0]/255.0)*(1.0/MinZ - 1.0/MaxZ) + 1.0/MaxZ);
			//printf("%f ",z);
			//z = 1.0/((ptr[i*DepthMap->nChannels]/255.0)*(1.0/MinZ - 1.0/MaxZ) + 1.0/MaxZ);
			//printf("Ptr %d %d %d\n",ptr[i*DepthMap->nChannels],ptr[i*DepthMap->nChannels+1],ptr[i*DepthMap->nChannels+2]);
			S.val[0] = z;
			S.val[1] = 0.0;
			S.val[2] = 0.0;
			cvSet2D(Zmap,j,i,S);
			//Zptr[i*Zmap->nChannels] = z;
			//printf("%f ",z);
			//printf("%f\n ",Zptr[i]); //show format
			//cvWaitKey(10000);
		}
	}
	return ;
}

void ForwardWarping(IplImage *srcTexture, IplImage * srcDepth, IplImage *dstTexture,IplImage *dstDepth,	int srcCam,int dstCam)
{
	double z = 0 ;
	CvScalar S;
	double x, y;
	double dstU = 0.0, dstV = 0.0;
	for(int j = 0; j< srcTexture->height;j++)
	{
		for(int i = 0; i< srcTexture->width;i++)
		{
			S = cvGet2D(srcDepth,j,i);
			z = S.val[0];
			projUVZtoXY(m_CalibParams[srcCam].m_ProjMatrix, (double)i, (double)j, z, &x, &y);
			projXYZtoUV(m_CalibParams[dstCam].m_ProjMatrix, x, y, z, &dstU, &dstV);
			//printf( "Testing projection of pt (%d,%d) in camera 0 with z = %f to other cameras\n",i, j, z);
			//printf( "3D pt = (%f, %f, %f) [coordinates wrt reference camera]\n", x, y, z );
			//printf( "Camera %d: (%f, %f)\n", dstCam, dstU, dstV);
			if(0<dstU && dstU<dstTexture->width && 0<dstV && dstV<dstTexture->height)
			{
				if(z <= (cvGet2D(dstDepth,(int)dstV,(int)dstU)).val[0]|| (cvGet2D(dstDepth,(int)dstV,(int)dstU)).val[0] ==0.0)
				{
					S.val[0] = z;
					S.val[1] = 0.0;
					S.val[2] = 0.0;
					cvSet2D(dstDepth,dstV,dstU,S);
					cvSet2D(dstTexture,dstV,dstU,cvGet2D(srcTexture,j,i));
				}
			}
		}
	}
	return ;
}


void InverseWarping(IplImage *dstTexture, IplImage * dstDepth, IplImage *srcTexture,IplImage *srcDepth,int dstCam,int srcCam)
{
	double z = 0 ;
	CvScalar S;
	double x, y;
	double srcU = 0.0, srcV = 0.0;
	for(int j = 0; j< dstTexture->height;j++)
	{
		for(int i = 0; i< dstTexture->width;i++)
		{
			S = cvGet2D(dstDepth,j,i);
			z = S.val[0];
			projUVZtoXY(m_CalibParams[dstCam].m_ProjMatrix, (double)i, (double)j, z, &x, &y);
			projXYZtoUV(m_CalibParams[srcCam].m_ProjMatrix, x, y, z, &srcU, &srcV);
			//printf( "Testing projection of pt (%d,%d) in camera 0 with z = %f to other cameras\n",i, j, z);
			//printf( "3D pt = (%f, %f, %f) [coordinates wrt reference camera]\n", x, y, z );
			//printf( "Camera %d: (%f, %f)\n", dstCam, dstU, dstV);
			if(0<srcU && srcU<srcTexture->width && 0<srcV && srcV<srcTexture->height)
			{
					cvSet2D(dstTexture,j,i,cvGet2D(srcTexture,srcV,srcU));
					//printf("here");
			}
		}
	}
	return ;
}

void WarpingProcessing(IplImage *srcTexture, IplImage * srcDepth, IplImage *dstTexture,IplImage *dstDepth,int srcCam,int dstCam)
{
	double MinZ = 42.0, MaxZ = 130.0; //ballet
	IplImage *Zmap = cvCreateImage(cvGetSize(srcDepth),IPL_DEPTH_32F,1);

	cvZero(Zmap);
	//printf("Depth n %d ",Depth->nChannels);
	//printf("step %d",Depth->widthStep);
	//printf("stepZ %d", Zmap ->widthStep);
	//printf("Depth %d\n",Depth->depth);
	DepthToZmap(srcDepth,Zmap, MaxZ ,MinZ);
//#ifdef	DEBUG
//	cvSaveImage("Zmap.bmp",Zmap);
//#endif
	//IplImage *dstTexture = cvCreateImage(cvGetSize(srcTexture),IPL_DEPTH_8U,3);
	//IplImage *dstDepth = cvCreateImage(cvGetSize(srcDepth),IPL_DEPTH_32F,1);
	cvZero(dstTexture);
	cvZero(dstDepth);

	ForwardWarping(srcTexture, Zmap, dstTexture,dstDepth,srcCam,dstCam);
	IplImage *dstDepthSmooth = cvCreateImage(cvGetSize(dstDepth),IPL_DEPTH_32F,1);
	cvZero(dstDepthSmooth);
	cvSmooth(dstDepth,dstDepthSmooth,CV_MEDIAN,3,3);

	InverseWarping(dstTexture,dstDepthSmooth,srcTexture, Zmap, dstCam,srcCam);
	//ShowImagePair(srcTexture,dstTexture);
	return;
}


void TestWarping(char *TexturePath , char *DepthPath)
{
	IplImage* Texture = cvLoadImage(TexturePath);
	IplImage* Depth = cvLoadImage(DepthPath);
	//IplImage * rDepth = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	//IplImage * g = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	//IplImage * b = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	////split the image into the color plane
	//cvSplit(Depth,rDepth,g,b,NULL);
	//ShowImagePair(Texture,Depth);

	// Z map range
	//double MinZ = 44.0, MaxZ = 120.0; //breakdancers
	double MinZ = 42.0, MaxZ = 130.0; //ballet
	IplImage *Zmap = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_32F,1);
	//IplImage *Zmap = cvCreateImageHeader(cvGetSize(Depth),IPL_DEPTH_32F,1);
	//Zmap ->origin = Depth ->origin;  
	//Zmap ->widthStep = Depth ->widthStep/Depth->nChannels;  
	cvZero(Zmap);
	//printf("Depth n %d ",Depth->nChannels);
	//printf("step %d",Depth->widthStep);
	//printf("stepZ %d", Zmap ->widthStep);
	//printf("Depth %d\n",Depth->depth);
	DepthToZmap(Depth,Zmap, MaxZ ,MinZ);
//#ifdef	DEBUG
//	cvSaveImage("Zmap.bmp",Zmap);
//#endif
	IplImage *dstTextureL = cvCreateImage(cvGetSize(Texture),IPL_DEPTH_8U,3);
	IplImage *dstDepthL = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_32F,1);
	cvZero(dstTextureL);
	cvZero(dstDepthL);
	int srcCam = 2;
	int dstCam = 3;
	ForwardWarping(Texture, Zmap, dstTextureL,dstDepthL,srcCam,dstCam);
	IplImage *dstDepthSmoothL = cvCreateImage(cvGetSize(dstDepthL),IPL_DEPTH_32F,1);
	cvZero(dstDepthSmoothL);
	cvSmooth(dstDepthL,dstDepthSmoothL,CV_MEDIAN,3,3);
	//ShowImagePair(Texture,dstTextureL);
	InverseWarping(dstTextureL,dstDepthSmoothL,Texture, Zmap, dstCam,srcCam);
	ShowImagePair(Texture,dstTextureL);

	//srcCam = 4;
	//IplImage *dstTextureR = cvCreateImage(cvGetSize(Texture),IPL_DEPTH_8U,3);
	//IplImage *dstDepthR = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_32F,1);
	//cvZero(dstTextureR);
	//cvZero(dstDepthR);
	//ForwardWarping(Texture, Zmap, dstTextureR,dstDepthR,srcCam,dstCam);
	//IplImage *dstDepthSmoothR = cvCreateImage(cvGetSize(dstDepthR),IPL_DEPTH_32F,1);
	//cvZero(dstDepthSmoothR);
	//cvSmooth(dstDepthR,dstDepthSmoothR,CV_BILATERAL);
	//ShowImagePair(dstTextureL,dstTextureR);

	//for(int j=0;j<Zmap->height;j++)
	//	for(int i =0; i<Zmap->width;i++)
	//	{
	//		CvScalar s;
	//		s = cvGet2D(Zmap,j,i);
	//		//printf("value %f",s.val[0]);
	//	}

	//ShowImagePair(Texture,Zmap);
	return ; 
}

void ImageBlending(IplImage *TextureL,IplImage *TextureR,IplImage *Texture, double alpha)
{
	CvScalar S=cvScalarAll(0.0);
	CvScalar SL=cvScalarAll(0.0);
	CvScalar SR=cvScalarAll(0.0);
	for(int j = 0; j< Texture->height;j++)
	{
		for(int i = 0; i< Texture->width;i++)
		{
			SL = cvGet2D(TextureL,j,i);
			SR = cvGet2D(TextureR,j,i);
			if(SL.val[0] != 0.0 && SR.val[0] !=0.0)
			{
				S.val[0] = alpha*SL.val[0] + (1.0-alpha)*SR.val[0];
				S.val[1] = alpha*SL.val[1] + (1.0-alpha)*SR.val[1];
				S.val[2] = alpha*SL.val[2] + (1.0-alpha)*SR.val[2];
				S.val[3] = alpha*SL.val[3] + (1.0-alpha)*SR.val[3];
				cvSet2D(Texture,j,i,S);
				//printf("here");
			}
			else if(SL.val[0] == 0.0 && SR.val[0] !=0.0)
			{
				alpha = 0;
				S.val[0] = alpha*SL.val[0] + (1.0-alpha)*SR.val[0];
				S.val[1] = alpha*SL.val[1] + (1.0-alpha)*SR.val[1];
				S.val[2] = alpha*SL.val[2] + (1.0-alpha)*SR.val[2];
				S.val[3] = alpha*SL.val[3] + (1.0-alpha)*SR.val[3];
				cvSet2D(Texture,j,i,S);
			}
			else if(SL.val[0] != 0.0 && SR.val[0] ==0.0)
			{
				alpha = 1;
				S.val[0] = alpha*SL.val[0] + (1.0-alpha)*SR.val[0];
				S.val[1] = alpha*SL.val[1] + (1.0-alpha)*SR.val[1];
				S.val[2] = alpha*SL.val[2] + (1.0-alpha)*SR.val[2];
				S.val[3] = alpha*SL.val[3] + (1.0-alpha)*SR.val[3];
				cvSet2D(Texture,j,i,S);
			}
			else
			{
				S = cvScalarAll(0.0);
				cvSet2D(Texture,j,i,S);
			}

		}
	}
	return;
}

void TestSplitDepth(char *DepthPath)
{
	IplImage* Depth = cvLoadImage(DepthPath);
	IplImage * rDepth = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	IplImage * g = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	IplImage * b = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	//split the image into the color plane
	cvSplit(Depth,rDepth,g,b,NULL);
	cvSaveImage("r.bmp",rDepth);
	cvSaveImage("g.bmp",g);
	cvSaveImage("b.bmp",b);

	IplImage * gray = cvCreateImage(cvGetSize(Depth),IPL_DEPTH_8U,1);
	cvConvertImage(Depth,gray);
	cvSaveImage("gray.bmp",gray);
	return;
}
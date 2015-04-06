//////////////////////////////////////////////////////////////////////
// 
// Warning: Use code at your own risk
//
// Functions to read calibration parameters, 
// convert values in depth images to real Z,
// and project given camera matrices and Z
//
//////////////////////////////////////////////////////////////////////
#include <stdio.h>
// this should really be placed in a header file...
typedef struct {
    double m_K[3][3]; // 3x3 intrinsic matrix
    double m_RotMatrix[3][3]; // rotation matrix
    double m_Trans[3]; // translation vector

    double m_ProjMatrix[4][4]; // projection matrix
} CalibStruct;

// globals (for illustration only)
CalibStruct m_CalibParams[8];
int m_NumCameras = 8;
int m_Width = 1024, m_Height = 768; // camera resolution is 1024x768

//////////////////////////////////////////////////////////////////////
// The four function definitions are the only ones necessary if 
// depths and reprojections are required.
// You can easily write your version of epipolar extraction 
//(for stereo) from the code.

void InitializeFromFile(char *fileName);
double DepthLevelToZ( unsigned char d );
double projXYZtoUV(double projMatrix[4][4], 
double x, double y, double z, double *u, double *v);
void projUVZtoXY(double projMatrix[4][4], 
double u, double v, double z, double *x, double *y);

//////////////////////////////////////////////////////////////////////
// This function is to illustrate how to use the projection functions

void TestProject(double pts[8][2], int u, int v, unsigned char d);

//////////////////////////////////////////////////////////////////////


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

// d is pixel value in the depth images. 
// d represents depth, but is NOT the actual depth value.
// This function returns actual z (depth)
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


int main()
{
	InitializeFromFile("calibParams-ballet.txt");
	double pts[8][2];
	int u = 15;
	int v = 20; 
	unsigned char d = 129;
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

/********************************HEADER FILE***********************************/
struct RV3DPOINT
{
	int u, v, d;
};

void ReadKinectPics(CString pathRGB, IplImage *DepthImage, int *DepthMap, RV3DPOINT *Point3DArray, int &n3DPoints);
BOOL Plane(RV3DPOINT *pPoint3D0, RV3DPOINT *pPoint3D1, RV3DPOINT *pPoint3D2,double &a, double &b, double &c);
void Consensus(RV3DPOINT *Point3DArray, int n3DPoints,double &a, double &b, double &c, RV3DPOINT **ConsensusSet, int &nConsensus);

inline int RVRandom(int yMin, int yMax)
{
	return  (rand() %  yMax)  + yMin ;
}
/*********************************************************************************/


/*****************************HELPER FUNCTIONS*************************************/
void ReadKinectPics(CString pathRGB, IplImage *DepthImage, int *DepthMap, RV3DPOINT *Point3DArray, int &n3DPoints)
{

	int u,v,d;
	int dmin = 2047;
	int dmax = 0;

	RV3DPOINT *pPoint3D = Point3DArray;

	n3DPoints = 0;

	//Get DepthImage file path
	CString pathDepth = pathRGB.Left(pathRGB.GetLength() - 4);
	pathDepth = pathDepth + "-D.txt";

	FILE *fp;

	fopen_s(&fp, pathDepth, "r");

	if(fp)
	{
		BOOL bOK = TRUE;

		//Determine max and min depth values and get Depth map
		for(v=0;v<DepthImage->height;v++)
		{
			for(u=0;u<DepthImage->width;u++)
			{
				if(!(bOK = (fscanf(fp, "%d ", &d) == 1)))
					break;

				
				if(d==2047)
				{
					d=-1;
				}
				else
				{
					//count number of valid 3D points
					n3DPoints++;

					//determine min and max d
					if(d<dmin)
						dmin = d;

					if(d>dmax)
						dmax = d;
				}

				DepthMap[v*DepthImage->width + u] = d;


				if(d != -1)
				{
					pPoint3D->u = u;
					pPoint3D->v = v;
					pPoint3D->d = d;

					pPoint3D++;
				}

				
			}
		}	

		fclose(fp);
	}


	//Form grayscale pic -> Scale from 1 to 255 (reserve 0 for undefined regions)
	for(v=0;v<DepthImage->height;v++)
	{
		for(u=0;u<DepthImage->width;u++)
		{
			d = DepthMap[v*DepthImage->width + u];

			if(d != -1)
				d = ((d - dmin) * 254/ (dmax - dmin)) + 1;  
			else
				d=0;

			((uchar *)(DepthImage->imageData + v*DepthImage->widthStep))[u]=d;
		}
	}


}


BOOL Plane(RV3DPOINT *pPoint3D0, RV3DPOINT *pPoint3D1, RV3DPOINT *pPoint3D2,double &a, double &b, double &c)
{
	double A[3 * 3];

	A[0*3+0] = pPoint3D0->u;
	A[0*3+1] = pPoint3D0->v;
	A[0*3+2] = 1.0;

	A[1*3+0] = pPoint3D1->u;
	A[1*3+1] = pPoint3D1->v;
	A[1*3+2] = 1.0;

	A[2*3+0] = pPoint3D2->u;
	A[2*3+1] = pPoint3D2->v;
	A[2*3+2] = 1.0;

	double Z[3];

	Z[0] = pPoint3D0->d;
	Z[1] = pPoint3D1->d;
	Z[2] = pPoint3D2->d;

	CvMat *A_ = cvCreateMatHeader(3, 3, CV_64FC1);
	A_->data.db = A;
	CvMat *Z_ = cvCreateMatHeader(3, 1, CV_64FC1);
	Z_->data.db = Z;

	double p[3];
	CvMat *p_ = cvCreateMatHeader(3, 1, CV_64FC1);
	p_->data.db = p;
	
	if(cvSolve(A_, Z_, p_))
	{
		a = p[0];
		b = p[1];
		c = p[2];	

		cvReleaseMatHeader(&A_);
		cvReleaseMatHeader(&Z_);
		cvReleaseMatHeader(&p_);

		return TRUE;
	}
	else
	{
		cvReleaseMatHeader(&A_);
		cvReleaseMatHeader(&Z_);
		cvReleaseMatHeader(&p_);

		return FALSE;
	}
}


void Consensus(RV3DPOINT *Point3DArray, int n3DPoints,double &a, double &b, double &c, RV3DPOINT **ConsensusSet, int &nConsensus)
{
	RV3DPOINT *pPoint3D = Point3DArray;

	RV3DPOINT **pConsensusSet = ConsensusSet;	
	double Tol = 4.0;
	
	int i;
	double e;

	nConsensus = 0;

	for(i = 0; i < n3DPoints; i++, pPoint3D++)
	{
		e = pPoint3D->d - (a * pPoint3D->u + b * pPoint3D->v + c);

		if(e > Tol)
			continue;

		if(e >= -Tol)
			*(pConsensusSet++) = pPoint3D;

		
	}

	nConsensus = pConsensusSet - ConsensusSet;	
	
}
/*********************************************************************************/


/*****************************MAIN FUNCTION*************************************/
IplImage *pRGBImage = 0;
IplImage *pDepthImage = 0;
int *DepthMap = 0;
int n3DPoints = 0;

int iWidth, iHeight;

RV3DPOINT *Point3DArray = 0; 




CFileDialog dlg(TRUE, _T("*.bmp"), "", OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY,
"Image files (*.bmp; *.jpg; *.pgm; *.png) |*.bmp;*.jpg;*.pgm;*.png|All Files (*.*)|*.*||",NULL);

dlg.m_ofn.lpstrTitle = _T("Select Image"); 


if (dlg.DoModal() == IDOK) 
{ 
	CString pathRGB = dlg.GetPathName();		
	
	//Get RGB image
	pRGBImage = cvLoadImage(pathRGB);

	iWidth = pRGBImage->width;
	iHeight = pRGBImage->height;
	
	DepthMap = new int[iWidth * iHeight];
	memset(DepthMap,0, iWidth * iHeight*sizeof(int));

	Point3DArray = new RV3DPOINT[iWidth * iHeight];

	memset(Point3DArray, 0, iWidth * iHeight * sizeof(RV3DPOINT));

	pDepthImage = cvCreateImage(cvSize(pRGBImage->width,pRGBImage->height),IPL_DEPTH_8U,1);


	ReadKinectPics(pathRGB, pDepthImage, DepthMap, Point3DArray, n3DPoints);
			
	cvNamedWindow ("RGB image");
	cvShowImage("RGB image", pRGBImage);


	cvNamedWindow ("Depth image");
	cvShowImage("Depth image", pDepthImage);
	
}



double a,b,c;
double aBest, bBest, cBest;

int nConsensus;

int nBest = 0;


int iter;

int maxRANSACIter = 1000; //Max RANSAC iterations

RV3DPOINT *pPoint3D, *pPoint3D0, *pPoint3D1, *pPoint3D2;

RV3DPOINT **Point3DPtrBuff1 = new RV3DPOINT *[n3DPoints];
RV3DPOINT **Point3DPtrBuff2 = new RV3DPOINT *[n3DPoints];

RV3DPOINT **ConsensusSet = Point3DPtrBuff1;
RV3DPOINT **BestConsensusSet = Point3DPtrBuff2;

RV3DPOINT **tmp;


int iR0, iR1, iR2;

//main RANSAC loop
for(iter=0;iter<maxRANSACIter;iter++)
{
	// randomly select 3 points from set A
	iR0 = RVRandom(0, n3DPoints - 1);
	iR1 = RVRandom(0, n3DPoints - 1);
	iR2 = RVRandom(0, n3DPoints - 1);

	pPoint3D0 = (Point3DArray + iR0);
	pPoint3D1 = (Point3DArray + iR1);
	pPoint3D2 = (Point3DArray + iR2);

	if(!Plane(pPoint3D0, pPoint3D1, pPoint3D2, a,b,c))
			continue;

	Consensus(Point3DArray, n3DPoints, a,b,c, ConsensusSet, nConsensus);

	//Test best values and consensus set
	if(nConsensus > nBest)
	{
		nBest = nConsensus;

		aBest = a;
		bBest = b;
		cBest = c;

		tmp = ConsensusSet;
		ConsensusSet = BestConsensusSet;
		BestConsensusSet = tmp;
	}		

}

IplImage *pColor = cvCreateImage( cvGetSize(pDepthImage), IPL_DEPTH_8U, 3 );
cvCvtColor( pDepthImage, pColor, CV_GRAY2RGB );


for(int i=0;i<nBest;i++)
{
	pPoint3D = BestConsensusSet[i];
	((uchar *)(pColor->imageData + pPoint3D->v*pColor->widthStep))[pPoint3D->u*3]=0;
	((uchar *)(pColor->imageData + pPoint3D->v*pColor->widthStep))[pPoint3D->u*3 + 1]=255;
	((uchar *)(pColor->imageData + pPoint3D->v*pColor->widthStep))[pPoint3D->u*3 + 2]=0;

}

cvNamedWindow ("Dominant plane");
cvShowImage("Dominant plane", pColor);

//exit elegantly 
delete[] DepthMap;
delete[] Point3DArray;
delete[] Point3DPtrBuff1;
delete[] Point3DPtrBuff2;
/*********************************************************************************/
		


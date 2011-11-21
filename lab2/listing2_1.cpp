void template_match(IplImage* img, CvRect rect)
{
	
	cvSetImageROI(img, rect);
	IplImage *templ = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);
	cvCopy(img, templ, NULL);

	cvResetImageROI(img);

	int iwidth = img->width  - templ->width  + 1;
	int iheight = img->height - templ->height + 1;
 
	// Allocate Output Image:
	IplImage *res = cvCreateImage(cvSize(iwidth,iheight),IPL_DEPTH_32F, 1); //32

	int istep = res->widthStep;

	/* perform template matching */
	cvMatchTemplate(img, templ, res, CV_TM_CCOEFF_NORMED); 

	/* find location of best matches */
	float thresh = 0.8f;

	int windowSize = 11;  //must be an odd number

	int windowRad  = (int)(windowSize/2);

	float centerPixelValue, currentPixelValue;
	bool bcenterPixelMax = false;

	
	uchar *pImageData = (uchar *)res->imageData;

	//center pixel must be greater than threshold and all pixels in subwindow less than center pixel.
	//if only one pixel is greater than center pixel discard that window
	for (int y=0; y<iheight; y++)
	{
		for (int x=0; x<iwidth; x++)
		{
			//subwindow
			if((x - windowRad >= 0) &&
			   (x + windowRad <= iwidth-1) &&
			   (y - windowRad >= 0) &&
			   (y + windowRad <= iheight-1)
			  )
			{
				centerPixelValue = ((float *)(pImageData + istep * y))[x];

				if(centerPixelValue > thresh)
				{
					bcenterPixelMax = true;

					for (int j = y-windowRad; j <= y+windowRad; j++)
					{
						for (int i = x - windowRad; i <= x+windowRad; i++)
						{
							currentPixelValue = ((float *)(pImageData + istep * j))[i];
					
							if(currentPixelValue > centerPixelValue)
							{
								bcenterPixelMax = false;
								break; // exit i
							}
						}

						if(!bcenterPixelMax)
							break;  //exit j
					}
					if(bcenterPixelMax)  /* draw rectangle */
						cvRectangle(img,
								cvPoint(x, y),
								cvPoint(x + templ->width, y + templ->height),
								cvScalar(0x00,0xff,0x00));
				
				}
			}
		}
	}


	cvNamedWindow ("Matched Template",CV_WINDOW_AUTOSIZE );
	cvShowImage("Matched Template", img);

	cvReleaseImage(&templ);
	cvReleaseImage(&res);

	
}
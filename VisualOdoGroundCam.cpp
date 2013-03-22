/*
 * A demonstration of visual odometry for car-like vehicles using a downward
 * looking camera. Please refer to the publicaiton:
 *		Navid Nourani-Vatani and Paulo VK Borges
 *			Correlation-based Visual Odometry for Car-like Vehicles
 *			Journal of Field Robotics, September 2011
 *  Copyright (C) 2012  Navid Nourani-Vatani
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "VisualOdoGroundCam.h"

/**
 * Set IMU off set values
 */
const double VisualOdo_t::imuOffsetRoll = 0.0;
const double VisualOdo_t::imuOffsetPitch = 0.0;
const double VisualOdo_t::imuOffsetYaw = 0.0;


/**
 * \brief Looks in the directory and returns a sorted vector of files matching
 * 		the fileName.
 */
int getDirContent (string dir,
		vector<string> &files,
		string fileName,
		string extension = "") {

	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL) {
		cout << "\nError(" << errno << ") opening " << dir << endl;
		return 0;
	}

	list<string> fileList;
	while ((dirp = readdir(dp)) != NULL) {
		string file = string(dirp->d_name);
		bool nameOK = false;
		bool extOK = false;

		// File name matches
		if (fileName == "" || fileName == "*" || file.find(fileName)
				!= string::npos) {
			nameOK = true;
		}
		// File extension matches
		if (extension == "" || extension == "*" || file.find(extension)
				!= string::npos) {
			extOK = true;
		}

		if (nameOK && extOK) {
			fileList.push_back(file);
		}
	}
	closedir(dp);

	// Sort the directory listing and copy to the vector
	files.clear();
	fileList.sort();
	list<string>::iterator it;
	for (it = fileList.begin(); it != fileList.end(); it++) {
		files.push_back(*it);
	}

	return files.size();
}

/*
 * \brief Calculate the median of the list.
 */
inline double median (std::list<double> l) {
	std::list<double>::iterator it;
	l.sort();
	// Get the index location of the median
	int medianIdx = 0;
	if (l.size() % 2 == 0) {
		medianIdx = l.size() / 2;
	}
	else {
		medianIdx = l.size() / 2 + 1;
	}
	// Loop to this location
	it = l.begin();
	for (int i = 0; i < medianIdx; i++) {
		it++;
	}
	return *it;
}

/**
 * 	\brief Limits a value to a min/max
 */
inline double limit (double val, double minVal = 0, double maxVal = 1) {
	if (val < minVal) {
		val = minVal;
	}
	else if (val > maxVal) {
		val = maxVal;
	}
	return val;
}

/**
 * 	\brief Makes sure the value is within the image boundary
 */
inline void boundaryCheck (IplImage * frame,
		int templateWinLength,
		int * xStart,
		int * yStart) {
	if (*xStart < 0) {
		*xStart = 0;
	}
	else if (*xStart >= frame->width - templateWinLength) {
		*xStart = frame->width - templateWinLength - 1;
	}

	if (*yStart < 0) {
		*yStart = 0;
	}
	else if (*yStart >= frame->height - templateWinLength) {
		*yStart = frame->height - templateWinLength - 1;
	}
}

/** ***************************************************************************
 *	\brief Use OpenCV's MatchTemplate to calculate the flow. The result is a min and a max val and their locations in the second frame.
 *	\param lastFrame - is the first frame
 *	\param currFrame - is the second frame we are trying to find a match in
 *	\param win - is the area in the first frame we want to find in the second frame
 * 	\param minVal - The min value in the result frame
 *	\param maxVal - The max value in the result frame
 *	\param minLoc - Location of minVal
 * 	\param maxLoc - Location of maxVal
 *	\param resize - Resize original frame too speed up detection. If result is bad it will try with original frame size.
 *	\param matchMethod - The matching method used (See OpenCV ref for details)
 */
bool templateMatching (IplImage * lastFrame,
		IplImage * currFrame,
		CvRect win,
		double * minVal,
		double * maxVal,
		CvPoint * minLoc,
		CvPoint * maxLoc,
		double resizeFactor,
		int matchMethod) {

	static IplImage * tmpCurrFrame = NULL;
	static IplImage * tmpLastFrame = NULL;
	static CvRect tmpWin;
	static IplImage * corrResult = NULL;
	static CvMat * tmplate = NULL;

	if (resizeFactor <= 0) {
		resizeFactor = 1;
	}

	if (!tmpCurrFrame) {
		tmpCurrFrame = cvCreateImage(cvSize(currFrame->width / resizeFactor,
				currFrame->height / resizeFactor), IPL_DEPTH_8U,
				currFrame->nChannels);
	}
	if (!tmpLastFrame) {
		tmpLastFrame = cvCreateImage(cvSize(lastFrame->width / resizeFactor,
				lastFrame->height / resizeFactor), IPL_DEPTH_8U,
				lastFrame->nChannels);
	}
	tmpWin = cvRect(win.x / resizeFactor, win.y / resizeFactor, win.width
			/ resizeFactor, win.height / resizeFactor);

	// Resize everything
	if (resizeFactor != 1.0) {
		cvResize(currFrame, tmpCurrFrame);
		cvResize(lastFrame, tmpLastFrame);
	}
	else {
		cvCopy(currFrame, tmpCurrFrame);
		cvCopy(lastFrame, tmpLastFrame);
	}

	// Create the template and extract it from the source image
	if (!tmplate) {
		tmplate = cvCreateMat(tmpWin.height, tmpWin.width, CV_8UC1);
	}
	// Create template Matrix from last frame
	cvGetSubRect(tmpLastFrame, tmplate, tmpWin);

	// Specify the size needed by the match function
	int resultW = tmpCurrFrame->width - tmplate->width + 1;
	int resultH = tmpCurrFrame->height - tmplate->height + 1;
	// create the result image
	cvReleaseImage(&corrResult);
	corrResult = cvCreateImage(cvSize(resultW, resultH), IPL_DEPTH_32F, 1);

	// See if we can find tmplate in currFrame
	cvMatchTemplate(tmpCurrFrame, tmplate, corrResult, matchMethod);

	//cvNamedWindow( "corr res", 1 );
	//cvShowImage( "corr res", corrResult );
	//cvWaitKey( 1 );

	// Get min/max values
	cvMinMaxLoc(corrResult, minVal, maxVal, minLoc, maxLoc);
	if (minLoc) {
		minLoc->x = (int) (minLoc->x * resizeFactor);
		minLoc->y = (int) (minLoc->y * resizeFactor);
	}
	if (maxLoc) {
		maxLoc->x = (int) (maxLoc->x * resizeFactor);
		maxLoc->y = (int) (maxLoc->y * resizeFactor);
	}

	cvReleaseImage(&tmpCurrFrame);
	cvReleaseImage(&tmpLastFrame);
	cvReleaseData(tmplate);

	return true;
}

#if USE_LIBMUSIC
/**
 * Uses Linear Prediction Filter to perform forward prediction
 * @param x - History of data points
 * @param filtSize - Filter size
 * @return prediction
 */
double predict( const double * x, const int filtSize ) {

	double ac[filtSize];
	double ref[filtSize];
	double lpc[filtSize];
	double est = 0;

	// Auto-corr
	autocorrelation(filtSize, x, filtSize, ac);
	if( 0 ) {
		cout << "ac: ";
		for( int i = 0; i < filtSize; i++ ) {
			cout << ac[i] << " ";
		}
		cout << endl;
	}

	// LPC
	levinson_durbin( ac, ref, lpc );
	if( 0 ) {
		cout << "LPC: ";
		for( int i = 0; i < filtSize; i++ ) {
			cout << lpc[i] << " ";
		}
		cout << endl;
	}

	// Estimate
	for( int i = 0; i < filtSize-1; i++ ) {
		est += -lpc[i] * x[filtSize-i-2];
		//cout << "-lpc[" << i << "]=" << -lpc[i] << ", x[" << filtSize-i-2 << "]=" << x[filtSize-i-2] << ", est=" << est << endl;
	}

	return est;
}
#endif

/**
 * \brief Calculates the "quality" of the template window using different
 * 		measures.
 */
double getTemplateQuality (IplImage * img, /** Template image */
int method /** 0=Auto-correlation, 1=Std Dev, 2=Variance, 3=MAD, 4=Entropy */
) {

	double quality = 0;

	// Auto-correlation variables
#ifdef USE_CIMG
	CImg<float> tmp;
	CImg<float> corr;
	IplImage * iplImg = NULL;
	CvPoint loc;
	double firstMax = 0;
	double secondMax = 0;
#endif

	// MAD & Variance/StdDev variables
	CvScalar mean;
	CvScalar std_dev;
	CvScalar mean_deviation;
	double med = 0;
	list<double> medianList;

	// Entroty variables
	int histSizes = 256;
	float s_ranges[] = {0, histSizes};
	float * ranges[] = {s_ranges};
	static CvHistogram * hist = NULL;
	if (hist == NULL && method == 4) {
		hist = cvCreateHist(1, &histSizes, CV_HIST_ARRAY, ranges, 1);
	}
	double entropy = 0;

	switch (method) {
		/* Quality from auto-correlation */
		case 0:
#ifdef USE_CLIMG
			tmp.assign( img );
			tmp = tmp - tmp.mean();
			corr = tmp.correlate( tmp, 0, true );

			// Get max location
			iplImg = corr.get_IPL();
			cvMinMaxLoc( iplImg, NULL, &firstMax, NULL, &loc, NULL );

			// Set the max to 0.0
			cvSetReal2D( iplImg, loc.x, loc.y, 0 );

			// Get second max location
			cvMinMaxLoc( iplImg, NULL, &secondMax, NULL, &loc, NULL );

			if( secondMax != 0.0 ) {
				quality = (secondMax/firstMax);
			}
#else
			cerr
					<< "This feature is unavailable since we have not linked against CImg."
					<< endl;
#endif
			break;
			/* Quality from variance */
		case 1:
			cvAvgSdv(img, &mean, &std_dev);
			quality = pow(std_dev.val[0], 2.0) / MAX_THEORETICAL_VARIANCE;
			break;
			/* Quality from std dev */
		case 2:
			cvAvgSdv(img, &mean, &std_dev);
			quality = std_dev.val[0] / MAX_THEORETICAL_STDEV;
			break;
			/* Quality from Me(di)an Absolute Deviation */
		case 3:
#if 1		/* Median */
			// calculate the median
			for (register int j = 0; j < img->height; j++) {
				for (register int i = 0; i < img->width; i++) {
					medianList.push_back(img->imageData[j * img->widthStep + i]);
				}
			}
			med = median(medianList);

			// calculate the absolute deviation
			medianList.clear();
			for (register int j = 0; j < img->height; j++) {
				for (register int i = 0; i < img->width; i++) {
					medianList.push_back(fabs(img->imageData[j * img->widthStep
							+ i] - med));
				}
			}
			quality = median(medianList);

#else		// Mean
			mean = cvAvg( img, NULL );
			cvAbsDiffS( img, img, mean );
			mean_deviation = cvSum( img );

			quality = mean_deviation.val[0] / MAX_THEORETICAL_MAD;
			//printf( "mad: %d, max: %d, qual: %f\n",
			//		(int)mean_deviation.val[0], MAX_THEORETICAL_MAD, quality );
#endif
			break;
			/* Quality from Entropy */
		case 4:
			cvCalcHist(&img, hist);
			cvNormalizeHist(hist, 1.0);
			entropy = 0;
			for (register int i = 0; i < histSizes; i++) {
				float histValue = cvQueryHistValue_1D( hist, i );
				if (histValue) {
					entropy += histValue * log2(histValue);
				}
			}
			quality = -entropy;
			break;
		default:
			cerr << "Unknown method given: " << method << endl;
			break;
	}

	return quality;
}

/**
 * \brief Tries a number of different templates and returns the most suitable
 * IplImage * testFrame - The image the template is going to be extracted from
 * CvRect * rectOrg -  The original template location and size
 * int tries - How many different regions it should try
 * int MAX_SHIFT - The max shift of template location
 * int TQM - The template quality measure to use
 */
double getBestTemplate (IplImage * testFrame,
		CvRect * rectOrg,
		int tries,
		int MAX_SHIFT,
		int TQM) {

	IplImage * tempTestFrame = NULL;

	bool skip;
	CvPoint maxLoc;
	CvPoint minLoc;
	double maxVal;
	double minVal;
	double scale;
	double quality[tries];
	double bestQuality = -1e6;
	CvRect rect[tries];
	CvRect bestRect = *rectOrg;

	/* LUT for template locations */
	int w = rectOrg->width;
	int x = rectOrg->x;
	int y = rectOrg->y;
	int dX[tries];
	int dY[tries];
	switch (tries) {
		case 1:
			dX[0] = 0;
			dY[0] = 0;
			//dX[0] = -MAX_SHIFT/2; dY[0] = -MAX_SHIFT/2;
			break;
		case 2:
			dX[0] = -MAX_SHIFT / 2;
			dY[0] = 0;
			dX[1] = +MAX_SHIFT / 2;
			dY[1] = 0;
			//dX[0] = -MAX_SHIFT/2; dY[0] = -MAX_SHIFT/2;
			//dX[1] = +MAX_SHIFT/2; dY[1] = -MAX_SHIFT/2;
			break;
		case 3:
			dX[0] = -MAX_SHIFT / 2;
			dY[0] = 0;
			dX[1] = +MAX_SHIFT / 2;
			dY[1] = 0;
			dX[2] = 0;
			dY[2] = 0;
			break;
		case 4:
			dX[0] = -MAX_SHIFT / 2;
			dY[0] = -MAX_SHIFT / 2;
			dX[1] = +MAX_SHIFT / 2;
			dY[1] = -MAX_SHIFT / 2;
			dX[2] = -MAX_SHIFT / 2;
			dY[2] = +MAX_SHIFT / 2;
			dX[3] = +MAX_SHIFT / 2;
			dY[3] = +MAX_SHIFT / 2;
			break;
		case 5:
			dX[0] = -MAX_SHIFT / 2;
			dY[0] = -MAX_SHIFT / 2;
			dX[1] = +MAX_SHIFT / 2;
			dY[1] = -MAX_SHIFT / 2;
			dX[2] = -MAX_SHIFT / 2;
			dY[2] = +MAX_SHIFT / 2;
			dX[3] = +MAX_SHIFT / 2;
			dY[3] = +MAX_SHIFT / 2;
			dX[4] = 0;
			dY[4] = 0;
			break;
		case 7:
			dX[0] = -MAX_SHIFT / 2;
			dY[0] = -MAX_SHIFT;
			dX[1] = +MAX_SHIFT / 2;
			dY[1] = -MAX_SHIFT;
			dX[2] = -MAX_SHIFT;
			dY[2] = 0;
			dX[3] = 0;
			dY[3] = 0;
			dX[4] = +MAX_SHIFT;
			dY[4] = 0;
			dX[5] = -MAX_SHIFT / 2;
			dY[5] = +MAX_SHIFT;
			dX[6] = +MAX_SHIFT / 2;
			dY[6] = +MAX_SHIFT;
			break;
		case 8:
			dX[0] = -MAX_SHIFT;
			dY[0] = -MAX_SHIFT / 2;
			dX[1] = +MAX_SHIFT;
			dY[1] = -MAX_SHIFT / 2;
			dX[2] = 0;
			dY[2] = -MAX_SHIFT / 2;
			dX[3] = -MAX_SHIFT / 2;
			dY[3] = 0;
			dX[4] = +MAX_SHIFT / 2;
			dY[4] = 0;
			dX[5] = -MAX_SHIFT;
			dY[5] = +MAX_SHIFT / 2;
			dX[6] = +MAX_SHIFT;
			dY[6] = +MAX_SHIFT / 2;
			dX[7] = 0;
			dY[7] = +MAX_SHIFT / 2;
			break;
		default:
			cerr << "Can't do " << tries << " tests!" << endl;
			return -1;
	}

	/* Now test the quality of several template areas */
	for (int i = 0; i < tries; i++) {
		skip = false;

		/* Location of template is selected from the LUT */
		rect[i] = cvRect(x + dX[i], y + dY[i], w, w);

		/* Boundary test */
		boundaryCheck(testFrame, rect[i].width, &rect[i].x, &rect[i].y);
		//		if( rect[i].x < 0 ) {
		//			rect[i].x = 0;
		//		}
		//		if( rect[i].x + rect[i].width >= testFrame->width ) {
		//			rect[i].x = testFrame->width - rect[i].width - 1;
		//		}
		//		if( rect[i].y < 0 ) {
		//			rect[i].y = 0;
		//		}
		//		if( rect[i].y + rect[i].height >= testFrame->height ) {
		//			rect[i].y = testFrame->height - rect[i].height - 1;
		//		}

		/* Extract the template image */
		while (true) {
			/* Make sure the template is inside the image boundary */
			if (rect[i].x < 0 || rect[i].y < 0 || rect[i].x + rect[i].width
					> testFrame->width || rect[i].y + rect[i].height
					> testFrame->height) {
				skip = true;
				cout << "WARNING: Skipping template @ " << rect[i].x << ", "
						<< rect[i].y << endl;
				break;
			}
			cvSetImageROI(testFrame, rect[i]);
			CvRect r = cvGetImageROI(testFrame);

			if (r.x == rect[i].x && r.y == rect[i].y && r.width
					== rect[i].width && r.height == rect[i].height) {
				if (tempTestFrame) {
					cvReleaseImage(&tempTestFrame);
				}
				tempTestFrame = cvCreateImage(cvSize(r.width, r.height),
						testFrame->depth, testFrame->nChannels);
				cvCopy(testFrame, tempTestFrame);
				cvResetImageROI(testFrame);
				break;
			}
			/* template is outside image region. Let's stick to the original
			 location. NOTE: there can be multiple of these! */
			else {
				rect[i] = *rectOrg;
				cerr << "\nERROR: This should not happen!" << endl;
				abort();
			}
		}

		if (skip) {
			continue;
			quality[i] = 0;
		}

		/* Get the template quality */
		quality[i] = getTemplateQuality(tempTestFrame, TQM);

		if (quality[i] > bestQuality) {
			bestQuality = quality[i];
			bestRect = rect[i];
		}

		cvReleaseImage(&tempTestFrame);

		if( show ) {
			char s[32];
			cvRectangle(
				prevDrawFrame, cvPoint( rect[i].x, rect[i].y ),
				cvPoint( rect[i].x + rect[i].width, rect[i].y + rect[i].height ),
				CV_RGB(255,255,0), 3 );
			sprintf( s, "%.3f", quality[i] );
			cvPutText( prevDrawFrame, s, cvPoint( rect[i].x + 5, rect[i].y + 12 ), &font, CV_RGB(64,64,64) );
		 }
	}

	/* return the best location back to the caller */
	*rectOrg = bestRect;
	return bestQuality;
}


/** ***************************************************************************
 *
 */
void calcPoseAfterCamShift (VisualOdo_t * vodo,
		int deltaX,
		int deltaY,
		double deltaTime) {

	SMALL::Matrix<4, 4, double> H;
	SMALL::Matrix<4, 4, double> Hd;
	SMALL::Pose3D poseTmp, poseShift, poseHd;
	double rollAng = 0;
	double pitchAng = 0;

	// Add the influence of roll and pitch
#if imuAvailable
	if( imu ) {
		rollAng = imu->roll - imuOffsetRoll;
		pitchAng = imu->pitch - imuOffsetPitch;

	}
#endif

	// Convert to real world displacement
	poseTmp.setPosition(deltaX * vodo->zConst, deltaY * vodo->zConst, 0.0);
	// Convert to vehicle front frame
	poseShift = vodo->A3D * poseTmp;

	// Displacement seen from the vehicle
	double x = poseShift.getX() * cos(pitchAng);
	double y = 0; // No side-ways displacement
	double z = poseShift.getX() * sin(pitchAng);
	double th = atan2(poseShift.getY(), vodo->poseCam.getX());
	poseHd.setPosition(x, y, z);
	poseHd.setAxisAngle(SMALL::makeVector(0, 0, 1), th);

	// Increment pose
	vodo->pose = vodo->pose * poseHd;

}

/******************************************************************************
 *
 *  Main worker thread
 *
 *****************************************************************************/
void * worker_thread (void * arg) {

	VisualOdo_t * vodo = (VisualOdo_t *) arg;

	// Init working frames
	IplImage * currFrame = cvCreateImage(cvGetSize(vodo->frame), IPL_DEPTH_8U,
			1);
	IplImage * prevFrame = cvCreateImage(cvGetSize(vodo->frame), IPL_DEPTH_8U,
			1);
	IplImage * tmpFrame =
			cvCreateImage(cvGetSize(vodo->frame), IPL_DEPTH_8U, 1);
	IplImage * tempTemplateImg = cvCreateImage(cvSize(vodo->templateWinLength,
			vodo->templateWinLength), IPL_DEPTH_8U, 1);

	// Variables for drawing purposes
	CvPoint p;
	CvPoint q;
	if (show) {
		prevDrawFrame = cvCreateImage(cvGetSize(vodo->frame), IPL_DEPTH_8U, 3);
		currDrawFrame = cvCreateImage(cvGetSize(vodo->frame), IPL_DEPTH_8U, 3);

		cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0);
		cvNamedWindow("Previous Frame", 0);
		cvMoveWindow("Previous Frame", 510, 10);
		cvNamedWindow("Current Frame", 0);
		cvMoveWindow("Current Frame", vodo->frame->width / 2 + 520, 10);
	}

	// Template matching stuff
	int xStart = vodo->frame->width / 2 - vodo->templateWinLength / 2;
	int yStart = vodo->frame->height / 2 - vodo->templateWinLength / 2;
	CvRect win = cvRect(xStart, yStart, vodo->templateWinLength,
			vodo->templateWinLength);

	CvPoint corrPredictLocation;
	CvPoint corrMatchLocation;
	double corrMatchVal = 0.0;

	int deltaX = 0;
	int deltaY = 0;
	int deltaXPrev = 0;
	int deltaYPrev = 0;
	CvRect subWin = cvRect(0, 0, vodo->frame->width, vodo->frame->height);
	double quality = 0.0;
	double MIN_GOOD_CORR_RESULT = 0.7;
	int numBadCorrelationsInSeq = 0;
	int numBadCorrelations = 0;

	int loopCount = 0;


	struct timeval tv;

	long prevFrameNum = 0;
	double prevFrameTimeDouble = 0;
	double deltaTimeRough;
	double deltaTime;

	double loopStartTime = 0;
	double loopEndTime = 0;

	// Velocity filtering
	int velFiltSize = 5;
	std::list<double> velVec(velFiltSize, 0);

	if (vodo->startFrame) {
		cout << "Skipping until frame " << vodo->startFrame << endl;
		if( getFrameFromImages ) {
			vodo->frameNum = vodo->startFrame;
			prevFrameNum = vodo->frameNum;
		}
		else {
			prevFrameNum = vodo->startFrame;
		}
	}

	/*************************************************************************/
	while (!done) {
		gettimeofday(&tv, NULL);
		loopStartTime = tv.tv_sec + tv.tv_usec * 1e-6;

		if (verbose)
			cout << endl << endl;
		/**********************************************************************
		 *
		 * Read Frame
		 *
		 **********************************************************************/
		if (1) {
			// Get frame
			if (getFrameFromImages) {
				if( vodo->frame ) {
					cvReleaseImage( &vodo->frame );
				}
				vodo->frame
						= cvLoadImage(
								(fileDir + "/" + fileVec[vodo->frameNum]).c_str(),
								CV_LOAD_IMAGE_COLOR);
				// TODO: parse file name as grab time
				vodo->frameGrabTimeDouble = (1.0 / vodo->MAX_FRAME_RATE)
						* vodo->frameNum + 1;
			}
			else {
				vodo->frame = cvQueryFrame(vodo->capture);
				// Get time
				gettimeofday(&tv, NULL);
				vodo->frameGrabTimeDouble = tv.tv_sec + tv.tv_usec * 1e-6;
			}

			// Make sure the grabbed frame has different time stamp than the
			//	previous frame
			if (vodo->frameGrabTimeDouble == prevFrameTimeDouble) {
				continue;
			}

			// Frame counter is internal (Use counter from camera to be more precise and detected missed frames
			vodo->frameNum++;

			// Skip first frames
			if ( vodo->frameNum < vodo->startFrame ) {
//				cout << "Skipping until frame " << vodo->startFrame << " ("
//						<< vodo->frameNum << ")" << endl;
				continue;
			}

			// Missed frame
			// To ensure we can run at the max frame rate we need to always capture consecutive frames
			if ((vodo->frameNum - prevFrameNum) > 1) {
				cout << "Warning: Missed " << vodo->frameNum - prevFrameNum
						<< " frames. (" << prevFrameNum << " -> " << vodo->frameNum << ")" << endl;
				done = true;
				continue;
			}

#if imuAvailable
			// Read IMU if available
#endif

			// Get delta time and filter it as the time can be noisy.
			// 		I am ensuring that the delta time is a product of the max frame rate
			deltaTimeRough = vodo->frameGrabTimeDouble - prevFrameTimeDouble;
			deltaTime = round(deltaTimeRough * vodo->MAX_FRAME_RATE)
					/ vodo->MAX_FRAME_RATE;

			// Copy back
			cvCopy(currFrame, prevFrame);
			prevFrameTimeDouble = vodo->frameGrabTimeDouble;
			prevFrameNum = vodo->frameNum;

			// Work on gray scale images
			if (vodo->frame->nChannels == currFrame->nChannels) {
				cvCopy(vodo->frame, currFrame);
			}
			else if (vodo->frame->nChannels == 3) {
				cvCvtColor(vodo->frame, currFrame, CV_RGB2GRAY);
			}

			// Prepare drawing frames
			if (show) {
				cvMerge(currFrame, currFrame, currFrame, NULL, currDrawFrame);
				cvMerge(prevFrame, prevFrame, prevFrame, NULL, prevDrawFrame);
			}

		}

		/**********************************************************************
		 *
		 *	Image displacement calculation
		 *
		 *********************************************************************/
		if (1) {
			CvPoint offset = cvPoint(0, 0);
			deltaXPrev = deltaX;
			deltaYPrev = deltaY;

			// Difference between prediction and actual location
			int diffPreTrueU = fabs(vodo->predictDeltaU - deltaXPrev);
			int diffPreTrueV = fabs(vodo->predictDeltaV - deltaYPrev);

			/* ************************************************
			 *
			 * Setup correlation mask location based on the
			 * 	prediction
			 *
			 */
#if USE_LIBMUSIC
			// Using linear forward prediction FIR filter
			if( dUHist.size() >= filtSize ) {
				dUIter = dUHist.begin();
				dVIter = dVHist.begin();
				int ii = 0;
				for(; dUIter != dUHist.end(); dUIter++, dVIter++, ii++ ) {
					xHist[ii] = *dUIter;
					yHist[ii] = *dVIter;
				}

				predictDeltaU = predict( xHist, filtSize ) * 1.12;
				predictDeltaV = predict( yHist, filtSize ) * 1.12;

				dUHist.pop_front();
				dVHist.pop_front();
			}
			// 	The previous match was good so we shift

			else if( corrMatchVal >= MIN_GOOD_CORR_RESULT ) {
				predictDeltaU = deltaXPrev;
				predictDeltaV = deltaYPrev;
			}
			// Pick the location in the center

			else {
				predictDeltaU = 0;
				predictDeltaV = 0;

			}

			xStart = templateFixCorner.x + predictDeltaU / 2;
			yStart = templateFixCorner.y + predictDeltaV / 2;

#else
			// Using Kalman filters to predict the location of the template window
			cvKalmanPredict(vodo->kalmanU);
			cvKalmanPredict(vodo->kalmanV);

			vodo->predictDeltaU = vodo->kalmanU->state_pre->data.fl[0]; //yU_k->data.fl[0];
			vodo->predictDeltaV = vodo->kalmanV->state_pre->data.fl[0]; //yV_k->data.fl[0];

			xStart = vodo->frame->width / 2 - vodo->templateWinLength / 2
					+ vodo->predictDeltaU / 2;
			yStart = vodo->frame->height / 2 - vodo->templateWinLength / 2
					+ vodo->predictDeltaV / 2;

#endif
			// Boundary check
			boundaryCheck(vodo->frame, vodo->templateWinLength, &xStart,
					&yStart);

			// Template location
			win = cvRect(xStart, yStart, vodo->templateWinLength,
					vodo->templateWinLength);


			/* ************************************************
			 *
			 * Test the quality of the template
			 *
			 */
			quality = -1;
			if (vodo->numTemplateTests > 1) {
				quality = getBestTemplate(prevFrame, &win,
						vodo->numTemplateTests, win.width,
						vodo->templateQualityMeasure);
			}

			/* ************************************************
			 *
			 * Template location based on the prediction and
			 * 	the best fit location
			 *
			 */
			corrPredictLocation = cvPoint(win.x - vodo->predictDeltaU, win.y
					- vodo->predictDeltaV);

			/* ************************************************
			 *
			 *	Restricted search
			 * 	Take out a sub image for the correlation to find a match
			 *
			 */
			if ( vodo->restrictSearch && (corrMatchVal >= MIN_GOOD_CORR_RESULT
					|| numBadCorrelationsInSeq <= 1)) {
				int h, w;
#if 0
				// Linear model
				h = (1.5 + (deltaTime*MAX_FRAME_RATE/2) + vel/MAX_VEL) * MIN_SIZE + fabs(yU_k->data.fl[0]);
				w = (1.5 + (deltaTime*MAX_FRAME_RATE/2) + vel/MAX_VEL) * MIN_SIZE + fabs(yU_k->data.fl[0]);
#elif 0
				// Velocity model
				double alpha = 1;
				double beta = 40;

				h = alpha * (frameNum - prevFrameNum) * fabs(deltaYPrev) + beta;
				w = alpha * (frameNum - prevFrameNum) * fabs(deltaXPrev) + beta;

#else
				// Constant Size Model
				h = vodo->restrictedSearchArea;
				w = vodo->restrictedSearchArea;
#endif

				h = limit(h, vodo->restrictedSearchArea, vodo->frame->height - 1);
				w = limit(w, vodo->restrictedSearchArea, vodo->frame->width - 1);

				int x = corrPredictLocation.x + win.width / 2 - w / 2;
				int y = corrPredictLocation.y + win.height / 2 - h / 2;

				// Boundary Check
				x = limit( x, 0, currFrame->width - w - 1);
				y = limit( y, 0, currFrame->height - h - 1);

				offset = cvPoint(x, y);

				// Take out sub-image
				CvRect tmpRect = cvRect(x, y, w, h);
				if (tmpFrame->width != w || tmpFrame->height != h) {
					cvReleaseImage(&tmpFrame);
					tmpFrame = cvCreateImage(cvSize(w, h), 8, 1);
				}
				cvSetImageROI(currFrame, tmpRect);
				cvCopy(currFrame, tmpFrame);
				cvResetImageROI(currFrame);

				if (show) {
					// draw restriction
					cvRectangle(currDrawFrame, cvPoint(x, y), cvPoint(x + w, y
							+ h), CV_RGB( 255, 0, 255 ), 3);
				}

			}
			// Pass on the original image
			else {
				if (tmpFrame->width != currFrame->width || tmpFrame->height
						!= currFrame->height) {
					cvReleaseImage(&tmpFrame);
					tmpFrame = cvCreateImage(cvGetSize(currFrame),
							IPL_DEPTH_8U, 1);
				}
				cvCopy(currFrame, tmpFrame);
			}

			/* ************************************************
			 *
			 * Calculate template match
			 *
			 */
			templateMatching(prevFrame, tmpFrame, win, NULL, &corrMatchVal,
					NULL, &corrMatchLocation, vodo->resizeFactor,
					CV_TM_CCOEFF_NORMED);

			/* ************************************************
			 *
			 * Displacement in pixels.
			 *
			 */
			//compensate for any offset due to sub-framing
			corrMatchLocation.x += offset.x;
			corrMatchLocation.y += offset.y;

			deltaX = win.x - corrMatchLocation.x;
			deltaY = win.y - corrMatchLocation.y;

			/* ************************************************
			 *
			 * Deal with bad correlation
			 *
			 */
			if (corrMatchVal < MIN_GOOD_CORR_RESULT ) {
				numBadCorrelationsInSeq++;

				//				if( numBadCorrelationsInSeq < 1 ) {
				cerr << "\nERROR: Bad Correlation (" << corrMatchVal
						<< ")! Estimate: dX/dY=" << deltaX << "/" << deltaY
						<< " Using prediction: dX/dY=" << corrPredictLocation.x
						<< "/" << corrPredictLocation.y << endl;
				deltaX = win.x - corrPredictLocation.x;
				deltaY = win.y - corrPredictLocation.y;
				//				}
			}
			else {
				numBadCorrelationsInSeq = 0;
			}

#if USE_LIBMUSIC
			// Add measurement to history
			dUHist.push_back( deltaX );
			dVHist.push_back( deltaY );
#else
			// Update Kalman with new measurement
			cvSet(vodo->zU_k, cvScalar(deltaX));
			cvSet(vodo->zV_k, cvScalar(deltaY));
			// Correction step
			cvKalmanCorrect(vodo->kalmanU, vodo->zU_k);
			cvKalmanCorrect(vodo->kalmanV, vodo->zV_k);
			// Add process noise
			cvRandSetRange(&(vodo->rng), 0, sqrt(
					vodo->kalmanU->process_noise_cov->data.fl[0]), 0);
			cvRand(&(vodo->rng), vodo->wU_k);
			cvRandSetRange(&(vodo->rng), 0, sqrt(
					vodo->kalmanV->process_noise_cov->data.fl[0]), 0);
			cvRand(&(vodo->rng), vodo->wV_k);
			// Step
			const float F[] = {1, deltaTime * vodo->MAX_FRAME_RATE, 0, 1};
			memcpy(vodo->kalmanU->transition_matrix->data.fl, F, sizeof(F)); // F
			memcpy(vodo->kalmanV->transition_matrix->data.fl, F, sizeof(F));
			cvMatMulAdd( vodo->kalmanU->transition_matrix, vodo->xU_k, vodo->wU_k, vodo->xU_k );
			cvMatMulAdd( vodo->kalmanV->transition_matrix, vodo->xV_k, vodo->wV_k, vodo->xV_k );
#endif
		}

		/**********************************************************************
		 *
		 *	Odometry Calculations
		 *
		 *********************************************************************/
		if (1) {
			SMALL::Pose3D posePrev = vodo->pose; // Used for velocity calculation

			// Get displacement seen by camera
			calcPoseAfterCamShift(vodo, deltaX, deltaY, deltaTime);

			// Set direction of motion
			if (deltaY > 0) {
				vodo->directionOfTravel = MOTION_REVERSE;
			}
			else if (deltaY < 0) {
				vodo->directionOfTravel = MOTION_FORWARD;
			}
			else {
				vodo->directionOfTravel = MOTION_NONE;
			}

			// Velocity calculation
			if (deltaTime) {
				double vel = vodo->directionOfTravel * hypot(vodo->pose.getX()
						- posePrev.getX(), vodo->pose.getY() - posePrev.getY())
						/ deltaTime;

				velVec.pop_front();
				velVec.push_back(vel);
				vodo->vel = median(velVec);
			}
		}

		if (verbose) {
			char s[128];
			sprintf(
					s,
					"%f %4d %4d %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f",
					vodo->frameGrabTimeDouble, deltaX, deltaY,
					vodo->pose.getX(), vodo->pose.getY(), vodo->pose.getZ(),
					vodo->pose.getRollRad(), vodo->pose.getPitchRad(),
					vodo->pose.getYawRad(), vodo->vel, corrMatchVal, quality);

			cout << s << endl;
		}

		/**********************************************************************
		 *
		 *	Drawing section
		 *
		 *********************************************************************/
		if (show) {
			char s[128];

			/*
			 * Draw Prev Frame
			 */
			// draw a grid
			p = cvPoint(0, prevDrawFrame->height / 2);
			q = cvPoint(prevDrawFrame->width, prevDrawFrame->height / 2);
			cvLine(prevDrawFrame, p, q, CV_RGB(128,128,128));
			p = cvPoint(prevDrawFrame->width / 2, 0);
			q = cvPoint(prevDrawFrame->width / 2, prevDrawFrame->height);
			cvLine(prevDrawFrame, p, q, CV_RGB(128,128,128));

			// highlight template
			p = cvPoint(win.x, win.y);
			q = cvPoint(p.x + vodo->templateWinLength, p.y
					+ vodo->templateWinLength);
			cvRectangle(prevDrawFrame, p, q, CV_RGB( 0, 0, 255 ), 3, CV_AA, 0);
			// Draw frame number on Feature frame
			sprintf(s, "frame %ld", vodo->frameNum - 1);
			cvPutText(prevDrawFrame, s, cvPoint(10, 20), &font,
					CV_RGB(64,64,64));

			/*
			 * Draw Curr frame
			 */
			// draw a grid
			p = cvPoint(0, currDrawFrame->height / 2);
			q = cvPoint(currDrawFrame->width, currDrawFrame->height / 2);
			cvLine(currDrawFrame, p, q, CV_RGB(128,128,128));
			p = cvPoint(currDrawFrame->width / 2, 0);
			q = cvPoint(currDrawFrame->width / 2, currDrawFrame->height);
			cvLine(currDrawFrame, p, q, CV_RGB(128,128,128));

			// draw prediction
			p = corrPredictLocation;
			q = cvPoint(p.x + vodo->templateWinLength, p.y
					+ vodo->templateWinLength);
			cvRectangle(currDrawFrame, p, q, CV_RGB(0, 255, 0 ), 3, CV_AA);

			// draw correlation result
			p = corrMatchLocation;
			q = cvPoint(p.x + vodo->templateWinLength, p.y
					+ vodo->templateWinLength);
			cvRectangle(currDrawFrame, p, q, CV_RGB(255*corrMatchVal, 0, 0 ),
					3, CV_AA, 0);
			sprintf(s, "%.2f", corrMatchVal);
			cvPutText(currDrawFrame, s, cvPoint(p.x - 2, p.y - 2), &font,
					CV_RGB(255,0,0));

			// Draw frame number on Optical Flow frame
			sprintf(s, "frame %ld", vodo->frameNum);
			cvPutText(currDrawFrame, s, cvPoint(10, 20), &font,
					CV_RGB(64,64,64));

			cvShowImage("Previous Frame", prevDrawFrame);
			cvShowImage("Current Frame", currDrawFrame);

		} // end show

		/**********************************************************************
		 *
		 * Other stuff
		 *
		 *********************************************************************/
		if (1) {
			// Write frames to image file
			if (writeFramesAsImage && show) {
				char s[128];
				sprintf(s, "prev_frame_%ld.tif", vodo->frameNum - 1);
				cvSaveImage(s, prevDrawFrame);
				sprintf(s, "curr_frame_%ld.tif", vodo->frameNum);
				cvSaveImage(s, currDrawFrame);
			}
			loopCount++;

			gettimeofday(&tv, NULL);
			loopEndTime = tv.tv_sec + tv.tv_usec * 1e-6;
			double loopTime = loopEndTime - loopStartTime;

			printf("\r%ld: (%3d ms -> %.1f fps) [%.2f %.2f %.2f] < %.2f @ %.2f m/s    ",
					vodo->frameNum, (int)(loopTime*1e3), 1./loopTime,
					vodo->pose.getX(), vodo->pose.getY(), vodo->pose.getZ(),
					vodo->pose.getYawRad() / M_PI * 180, vodo->vel);

			// Reached last frame
			if (vodo->frameNum >= vodo->endFrame) {
				cout << "\nEnd frame reached (" << vodo->frameNum
						<< "). Quitting." << endl;
				done = true;
			}

			if (stepByStep) {
				cvWaitKey(0);
			}
			else if( show ) {
				// Handle Key Clicks
				int c = cvWaitKey(10);
				// Pause on Key press
				switch (c) {
					case -1:
						// time out
					case 0:
					case 65361:
					case 65362:
					case 65364:
					case 65365:
						break;
					default:
						// On any key stroke except arrow keys
						cout
								<< "Press any key to continue again. Ctrl+C to quit"
								<< endl;
						if (cvWaitKey(0) == 262243) {
							done = true;
						}
						break;
				}
			}
		}

	}

	return NULL;
}

/******************************************************************************
 *
 *	MAIN
 *
 *****************************************************************************/
int main (int argc, char ** argv) {
	cout << "VisualOdoGroundCam  Copyright (C) 2012 Navid Nourani-Vatani" << endl << endl;
	srand(time(NULL));

	VisualOdo_t vodo;

	if( writeFramesAsImage && ! show ) {
		cerr << "\nERROR: show has to be on before we can write frames (you can easily change this)" << endl;
		return -1;
	}

	// Init frame and FrameGrabber
	if (getFrameFromImages) {
		if( fileDir == "" ) {
			cerr << "\nERROR: You need to set the location of the images or choose to grab the frames from a camera!" << endl;
			return -1;
		}
		cout << "Loading from images..." << endl;
		int numFiles = getDirContent(fileDir, fileVec, "*", fileExt);
		if( numFiles == 0 ) {
			cerr << "\nERROR: Did not find any image files." << endl;
			return -1;
		}
		cout << "Found " << numFiles << " images." << endl;
		vodo.endFrame = numFiles;

		vodo.frame = cvLoadImage((fileDir + "/" + fileVec[0]).c_str(),
				CV_LOAD_IMAGE_COLOR);
	}
	else {
		if ((vodo.capture = cvCaptureFromCAM(CV_CAP_ANY)) == NULL) {
			cerr << "\nERROR: Failed to open image capture" << endl;
			return -1;
		}
		if ((vodo.frame = cvQueryFrame(vodo.capture)) == NULL) {
			cerr << "\nERROR: Failed to get image" << endl;
			return -1;
		}
	}

	// Read camera position values and convert angles to radians
	float camX = 3.7, camY = 0.0, camZ = 0.42, camRoll = 0.0, camPitch = 0.0, camYaw = 1.47/180*M_PI;
	vodo.poseCam.set(SMALL::makeVector(camX, camY, camZ), SMALL::Rotation3D(
			camRoll, camPitch, camYaw));
	cout << "\n\t DID YOU REMEMBER TO SET THE CAMERA POSITION???? \n\n";
	
	/* The equation we use to calculate real motion compared to pixel motion is:
	 *	Obj_size = d / f * pixel_size * screen_size
	 * where d = camera-ground-distance, f = camera focal length, pixel_size = sensor pixel size,
	 * screen_size = displacement on the screen.
	 */
	float f = 6.6e-3; // Camera focal length [m]
	float ps = 6.45e-6; // Camera pixel size [m]
	float camConst = ps / f;
	vodo.MAX_FRAME_RATE = 40;
	vodo.zConst = vodo.poseCam.getZ() * camConst;
	cout << "\n\t DID YOU REMEMBER TO SET THE CAMERA PARAMETERS???? \n\n";

	// Calculates the transformation matrix
	// Transformation matrix to convert from camera frame to robot bumper frame:
	//		The camera frame            robot front frame
	//		x-------> X					^ Y
	//		|							|
	//		|							|
	//		v Y							.-------> X
	SMALL::Matrix<4, 4, double> A;
	SMALL::Matrix<4, 4, double> A1;
	SMALL::Matrix<4, 4, double> A2;
	double ang1 = vodo.poseCam.getRotation().getYawRad() - M_PI/2;
	double ang2 = M_PI;
	A1 = cos(ang1), -sin(ang1), 0.00000, 0.00000, sin(ang1), cos(ang1), 0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 0.00000, 0.00000, 0.00000, 0.00000, 1.00000;

	A2 = 1.00000, 0.00000, 0.00000, 0.00000, 0.00000, cos(ang2), -sin(ang2), 0.00000, 0.00000, sin(
			ang2), cos(ang2), 0.00000, 0.00000, 0.00000, 0.00000, 1.00000;
	A = A1 * A2;
	vodo.A3D = SMALL::Pose3D(A);
	cout << "poseCam: " << vodo.poseCam.toString() << endl;

	/*
	 * Estimate MAX_VEL from estimated frame rate
	 */
	vodo.MAX_VEL = (max(vodo.frame->width, vodo.frame->height)
			- vodo.templateWinLength) * vodo.zConst * vodo.MAX_FRAME_RATE;
	cout << "Max estimated velocity = " << vodo.MAX_VEL << " m/s ("
			<< vodo.MAX_VEL * 3.6 << " km/h) assuming " << vodo.MAX_FRAME_RATE
			<< " fps." << endl;

	/* Make sure the template window fits the frame or has the size of the
	 * 	power of 2 */
	double corrWinRatio = 1.0 / 5.0;
	if (vodo.templateWinLength >= min(vodo.frame->width, vodo.frame->height)) {
		cerr << "\nERROR: Given template window size is larger than frame."
				<< endl;
		vodo.templateWinLength = pow(2, round(log(min(vodo.frame->width,
				vodo.frame->height) * corrWinRatio) / log(2)));
	}
	cout << "The size of the template window is " << vodo.templateWinLength
			<< endl;

	if (vodo.restrictSearch) {
		vodo.restrictedSearchArea = vodo.templateWinLength + 30;
		cout << "Restricting the search area to " << vodo.restrictedSearchArea
				<< endl;
	}

	/**
	 * The maximum theoretical std dev is achieved in an image consisting of
	 * 	black and white pixels resulting in a std.dev of 128 (in an 8bit image).
	 * The maximum variance is the square of this of course.
	 */
	MAX_THEORETICAL_STDEV = pow(2.0, vodo.frame->depth) / 2;
	MAX_THEORETICAL_VARIANCE = MAX_THEORETICAL_STDEV * 2;
	MAX_THEORETICAL_MAD = vodo.frame->width * vodo.frame->height * pow(2.0,
			vodo.frame->depth);

		
#ifdef USE_LIBMUSIC

#else
	/*
	 * Kalman filter stuff
	 */
	cvRandInit(&(vodo.rng), 0, 1, -1, CV_RAND_UNI);
	vodo.kalmanU = cvCreateKalman(2, 1, 0);
	vodo.kalmanV = cvCreateKalman(2, 1, 0);
	vodo.xU_k = cvCreateMat(2, 1, CV_32FC1);
	vodo.xV_k = cvCreateMat(2, 1, CV_32FC1);
	vodo.wU_k = cvCreateMat(2, 1, CV_32FC1);
	vodo.wV_k = cvCreateMat(2, 1, CV_32FC1);
	vodo.zU_k = cvCreateMat(1, 1, CV_32FC1);
	vodo.zV_k = cvCreateMat(1, 1, CV_32FC1);
	cvZero(vodo.zU_k);
	cvZero(vodo.zV_k);
	// transition matrix
	const float F[] = {1, 1, 0, 1};
	memcpy(vodo.kalmanU->transition_matrix->data.fl, F, sizeof(F)); // F
	memcpy(vodo.kalmanV->transition_matrix->data.fl, F, sizeof(F));
	// other kalman parameters
	cvSetIdentity(vodo.kalmanU->measurement_matrix, cvRealScalar(1)); // H
	cvSetIdentity(vodo.kalmanV->measurement_matrix, cvRealScalar(1));
	cvSetIdentity(vodo.kalmanU->process_noise_cov, cvRealScalar(30)); // Q
	cvSetIdentity(vodo.kalmanV->process_noise_cov, cvRealScalar(70));
	cvSetIdentity(vodo.kalmanU->measurement_noise_cov, cvRealScalar(5)); // R
	cvSetIdentity(vodo.kalmanV->measurement_noise_cov, cvRealScalar(5));
	cvSetIdentity(vodo.kalmanU->error_cov_post, cvRealScalar(1)); // P_k
	cvSetIdentity(vodo.kalmanV->error_cov_post, cvRealScalar(1));
	// random initial state
	cvRandSetRange(&(vodo.rng), 0, 0.1, 0);
	vodo.rng.disttype = CV_RAND_NORMAL;
	cvRand(&(vodo.rng), vodo.xU_k);
	cvRand(&(vodo.rng), vodo.xV_k);
	cvRand(&(vodo.rng), vodo.kalmanU->state_post);
	cvRand(&(vodo.rng), vodo.kalmanV->state_post);
#endif

	// Create worker thread
	pthread_t thread;
	pthread_create(&thread, NULL, worker_thread, &vodo);

	// Wait till threads are complete before main continues.
	pthread_join(thread, NULL);

	// Clean up
	cvReleaseImage(&vodo.frame);
	cvDestroyAllWindows();

	return 0;
}

